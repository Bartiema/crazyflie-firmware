/**
 * mode_manager.c
 *
 * Main 100 Hz orchestration task. Implements the complete heading-fusion
 * logic from blimp.cpp, adapted for the CrazyFlie + Flow Deck v2.
 *
 * ── Fusion algorithm (direct port of blimp.cpp main loop) ─────────────────
 *
 *  Each FFT frame:
 *   1. Read pd[] from deck → push into FFT → extract per-channel FFT
 *      magnitudes at each target frequency
 *   2. Bearing estimation:
 *        bearing_angle = weighted_circular_mean of sensor angles (BearingAngle)
 *        smooth_bearing += BEARING_SMOOTH_FACTOR * angular_error   ← low-pass
 *        abs_bearing_yaw = current_yaw + smooth_bearing            ← world frame
 *   3. Map update:
 *        wlsGradientControllerAddMapPoint(x, y, total_magnitude)
 *   4. Map-based gradient estimation:
 *        grad_valid = wlsGradientControllerUpdateMap(x, y, &wOut)
 *        grad_ready = grad_valid && mapSize >= MIN_MAP_POINTS
 *                                && grad_mag >= GRADIENT_THRESHOLD
 *   5. Heading fusion (weighted circular mean of two world-frame headings):
 *        if !grad_ready:
 *            cmd_yaw = abs_bearing_yaw       (bearing only, w_b=1, w_g=0)
 *        else:
 *            cmd_yaw = weighted_circular_mean(abs_bearing_yaw, W_BEARING,
 *                                             grad_angle_world,  W_GRADIENT)
 *   6. Waypoint navigator driven by bearing angle + SNR (unchanged)
 *   7. Inject yaw + forward velocity setpoint into stabiliser
 *
 * ── Key differences from blimp.cpp ────────────────────────────────────────
 *   - GPS → Flow Deck v2 odometry (stateEstimatorGetX/Y)
 *   - World heading commanded via yaw-rate setpoint (not direct yaw angle)
 *   - No Webots emitter; setpoints go to commanderSetSetpoint()
 *   - Bearing smooth_bearing maintained per unique frequency
 *
 * ── Constants (mirrors blimp.cpp) ─────────────────────────────────────────
 *   W_BEARING            = 0.5  (50% bearing weight when both ready)
 *   W_GRADIENT           = 0.5  (50% gradient weight when both ready)
 *   BEARING_SMOOTH_FACTOR= 0.3  (low-pass on bearing angle)
 *   MIN_MAP_POINTS       = 3    (map cells before gradient used)
 *   GRADIENT_THRESHOLD   = 0.5  (minimum gradient magnitude)
 */

#define DEBUG_MODULE "MODEMGR"

#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "sensfusion6.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "static_mem.h"
#include "system.h"
#include "num.h"

#include "photodiode_deck.h"
#include "pd_fft_analyzer.h"
#include "bearing_angle_controller.h"
#include "wls_gradient_controller.h"
#include "waypoint_navigator.h"
#include "mode_manager.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Fusion constants — mirrors blimp.cpp
 * ────────────────────────────────────────────────────────────────────────── */
static float fusionWBearing        = 0.5f;   /* W_BEARING  */
static float fusionWGradient       = 0.5f;   /* W_GRADIENT */
static float bearingSmoothFactor   = 0.3f;   /* BEARING_SMOOTH_FACTOR */
static float gradientThreshold     = 0.5f;   /* GRADIENT_THRESHOLD (magnitude gate) */
static float minTotalLight         = 0.05f;  /* MIN_TOTAL_LIGHT — aggregate gate; FFT inputs are ADC/4095 so magnitudes are small */
static uint8_t fusionMinMapPoints  = 3;      /* MIN_MAP_POINTS */
/* Consecutive invalid FFT frames before cmdYaw falls back to hold-heading.
 * Bridges short dropouts so that bearingValid flickering does not cause
 * large sudden jumps in spYawDeg. */
static uint8_t bearingHoldFrames   = 3;

/* ──────────────────────────────────────────────────────────────────────────
 * Flight parameters
 * ────────────────────────────────────────────────────────────────────────── */
static float navAltTarget   = 1.0f;   /* hold altitude via Flow Deck */
static float navMaxVel      = 0.30f;  /* forward velocity clamp (m/s) */
static float navFwdSpeed      = 0.20f;  /* forward speed when approaching */
static float navAlignFwdTol   = 30.0f;  /* ±bearing (deg) for slow fwd during ALIGNING */
static float navAlignFwdSpeed = 0.10f;  /* forward speed during rough-aligned ALIGNING (m/s) */
static uint32_t pdTimeoutMs   = 500U;   /* revert MANUAL if no PD data */

/* ──────────────────────────────────────────────────────────────────────────
 * Default mission — override via upload_frequency_waypoints() from Python
 * ────────────────────────────────────────────────────────────────────────── */
typedef struct { float freq; float dwell_ms; } WpEntry;
static const WpEntry DEFAULT_MISSION[] = {
    { 170.0f, 1000.0f },
    { 150.0f, 1000.0f },
    { 170.0f, 1000.0f },
};
#define DEFAULT_MISSION_LEN  (sizeof(DEFAULT_MISSION) / sizeof(DEFAULT_MISSION[0]))

/* ──────────────────────────────────────────────────────────────────────────
 * Internal state
 * ────────────────────────────────────────────────────────────────────────── */
static DroneMode currentMode  = MODE_MANUAL;
static uint32_t  lastPdOkTick = 0;

/* Per-frequency tracking state (one slot per unique frequency in mission) */
#define MAX_TRACKED_FREQS  WP_NAV_MAX_FREQUENCIES

typedef struct {
    float  freq;
    float  smoothBearing;          /* low-pass filtered relative bearing (deg) */
    bool   bearingValid;
    uint8_t bearingInvalidFrames;  /* consecutive frames with bearingValid=false */
    float  channelSnr[8];          /* per-channel SNR at this frequency */
    float  totalMagnitude;         /* sum of all channel FFT magnitudes */
    float  maxSnr;
} FreqState;

static FreqState  freqStates[MAX_TRACKED_FREQS];
static int        numTrackedFreqs = 0;

/* Per-frequency readings passed to waypoint navigator */
static WpNavFreqReading freqReadings[MAX_TRACKED_FREQS];


/* Per-channel SNR for the active frequency, scaled ×100 and stored as int16
 * to fit within the 26-byte log packet limit (8 × 2 = 16 bytes).
 * Divide by 100 in Python to recover SNR in original units. */
static int16_t logChSnr[8];

/* Logged fusion diagnostics */
static float logBearingAngle = 0.0f;
static float logGradAngle    = 0.0f;
static float logGradMag      = 0.0f;
static float logCmdYaw       = 0.0f;
static float logWB           = 1.0f;
static float logWG           = 0.0f;
static int32_t logMapSize    = 0;
static float logMaxSnr       = 0.0f;

/* Live setpoint — updated each FFT frame, injected every 100 Hz tick.
 * spHolding=true means "command currentYaw" rather than a fixed world angle,
 * used during HOLDING/COMPLETE and before the first FFT frame arrives. */
static float spFwdVel  = 0.0f;
static float spYawDeg  = 0.0f;
static bool  spHolding = true;   /* safe default: hold wherever drone is pointing */

/* ──────────────────────────────────────────────────────────────────────────
 * Helpers
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * normalize_angle() — identical to blimp.cpp
 * Returns angle in (-180, +180].
 */
static float normalizeAngle(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/**
 * weighted_circular_mean() — exact port of blimp.cpp
 * Returns the weighted circular mean of angle1 and angle2 (degrees).
 * w1 + w2 should equal 1.0.
 */
static float weightedCircularMean(float angle1_deg, float w1,
                                   float angle2_deg, float w2)
{
    float r1 = angle1_deg * (float)M_PI / 180.0f;
    float r2 = angle2_deg * (float)M_PI / 180.0f;
    float cx = w1 * cosf(r1) + w2 * cosf(r2);
    float cy = w1 * sinf(r1) + w2 * sinf(r2);
    return atan2f(cy, cx) * 180.0f / (float)M_PI;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Mode safety
 * ────────────────────────────────────────────────────────────────────────── */
static bool safeToNavigate(void)
{
    if (!pdDeckIsReady()) {
        DEBUG_PRINT("MODEMGR: NAVIGATE refused — PD deck not ready\n");
        return false;
    }
    return true;
}

static void applyModeChange(DroneMode req)
{
    if (req == MODE_NAVIGATE) {
        if (!safeToNavigate()) { currentMode = MODE_MANUAL; return; }
        wlsGradientControllerClearMap();
        waypointNavigatorResetMission();
        waypointNavigatorStartMission();
        /* Reset bearing state on NAVIGATE entry */
        for (int f = 0; f < numTrackedFreqs; f++) {
            freqStates[f].smoothBearing        = 0.0f;
            freqStates[f].bearingValid         = false;
            freqStates[f].bearingInvalidFrames = 0;
        }
        /* Reset live setpoint: hold heading until the first FFT frame arrives */
        spFwdVel  = 0.0f;
        spYawDeg  = 0.0f;
        spHolding = true;
    }
    currentMode = req;
    DEBUG_PRINT("MODEMGR: mode → %d\n", (int)currentMode);
}

static void modeParamCallback(void) { applyModeChange(currentMode); }

/* ──────────────────────────────────────────────────────────────────────────
 * Setpoint injection
 * ────────────────────────────────────────────────────────────────────────── */
static void injectSetpoint(float vx, float yaw_deg)
{
    vx = constrain(vx, -navMaxVel, navMaxVel);

    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.mode.x        = modeVelocity;
    sp.mode.y        = modeVelocity;
    sp.mode.z        = modeAbs;
    sp.mode.yaw      = modeAbs;
    sp.velocity.x    = vx;
    sp.velocity.y    = 0.0f;
    sp.velocity_body = true;
    sp.position.z    = navAltTarget;
    sp.attitude.yaw  = yaw_deg;
    commanderSetSetpoint(&sp, COMMANDER_PRIORITY_EXTRX);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Main 100 Hz task
 * ────────────────────────────────────────────────────────────────────────── */
#define MODE_TASK_STACKSIZE  (10 * configMINIMAL_STACK_SIZE)
#define MODE_TASK_PRIORITY   1

STATIC_MEM_TASK_ALLOC(modeTask, MODE_TASK_STACKSIZE);

static void modeTask(void *param)
{
    (void)param;
    systemWaitStart();
    TickType_t lastWake = xTaskGetTickCount();

    while (1) {

        /* ── 1. Check photodiode health ─────────────────────────────────── */
        if (pdDeckIsReady()) {
            lastPdOkTick = xTaskGetTickCount();
        }

        /* ── 2. PD timeout safety ───────────────────────────────────────── */
        if (currentMode != MODE_MANUAL) {
            uint32_t sinceMs = (xTaskGetTickCount() - lastPdOkTick) * portTICK_RATE_MS;
            if (sinceMs > pdTimeoutMs) {
                DEBUG_PRINT("MODEMGR: PD timeout → MANUAL\n");
                currentMode = MODE_MANUAL;
            }
        }

        /* ── 3. Read yaw every tick — needed for 100 Hz setpoint injection ─ */
        float currentYaw = 0.0f;
        {
            float roll = 0.0f, pitch = 0.0f;
            sensfusion6GetEulerRPY(&roll, &pitch, &currentYaw);
        }

        /* ── 4. Run FFT when a new window is ready ──────────────────────── */
        /* pdFftAnalyzerRun() returns true only when PD_FFT_AVERAGES hops have
         * been accumulated and a fresh averaged spectrum has been published. */
        bool newSpectrum = false;
        if (pdFftAnalyzerWindowReady()) {
            newSpectrum = pdFftAnalyzerRun();
        }

        /* ── 5. Process new spectrum ────────────────────────────────────── */
        if (newSpectrum && numTrackedFreqs > 0) {

            /* Position only needed for map updates — read here, not every tick */
            float posX = 0.0f;
            float posY = 0.0f;
            {
                point_t pos;
                estimatorKalmanGetEstimatedPos(&pos);
                posX = pos.x;
                posY = pos.y;
            }

            /* ── Phase 1: compute ALL freqReadings before touching the navigator.
             *
             * CRITICAL: waypointNavigatorUpdate() must be called ONCE per FFT
             * frame, not once per frequency.  Calling it inside the frequency
             * loop drove it twice per frame; with both lights visible the second
             * call could advance the state machine a second time in the same
             * frame (HOLDING→SEARCHING→APPROACHING→HOLDING for the next WP),
             * causing the drone to "complete" the whole mission without moving.
             *
             * cmdYaw for setpoint injection is taken from the current waypoint's
             * frequency slot so the drone always faces the active target.
             * ──────────────────────────────────────────────────────────────── */
            float cmdYaws[MAX_TRACKED_FREQS];   /* per-frequency fused yaw targets */
            for (int i = 0; i < MAX_TRACKED_FREQS; i++) cmdYaws[i] = currentYaw;

            for (int f = 0; f < numTrackedFreqs; f++) {
                FreqState *fs = &freqStates[f];

                /* Extract per-channel magnitudes at this frequency */
                float magnitudes[BA_SENSOR_COUNT];
                float totalMag = 0.0f, maxSnr = 0.0f;

                for (int ch = 0; ch < BA_SENSOR_COUNT; ch++) {
                    PdFreqResult res;
                    pdFftAnalyzerGetFrequency(ch, fs->freq, 2.0f, &res);
                    magnitudes[ch]     = res.magnitude;
                    fs->channelSnr[ch] = res.snr;
                    totalMag          += res.magnitude;
                    if (res.snr > maxSnr) maxSnr = res.snr;
                }
                fs->totalMagnitude = totalMag;
                fs->maxSnr         = maxSnr;

                /* ── Bearing estimation (port of blimp.cpp bearing block) ── */
                BearingAngleInput baIn;
                memcpy(baIn.magnitude, magnitudes, sizeof(baIn.magnitude));
                baIn.max_snr = maxSnr;
                BearingAngleOutput baOut;
                bearingAngleControllerUpdate(&baIn, &baOut);

                fs->bearingValid = baOut.valid && (totalMag > minTotalLight);

                if (fs->bearingValid) {
                    fs->bearingInvalidFrames = 0;
                    /* smooth_bearing += SMOOTH_FACTOR * normalize(bearing - smooth)
                     * Mirrors blimp.cpp exactly. bearing is relative to body. */
                    float diff = normalizeAngle(baOut.bearing_deg - fs->smoothBearing);
                    fs->smoothBearing = normalizeAngle(
                        fs->smoothBearing + bearingSmoothFactor * diff);
                } else {
                    if (fs->bearingInvalidFrames < 255) fs->bearingInvalidFrames++;
                }

                /* Absolute world-frame bearing heading */
                float absBearingYaw = normalizeAngle(currentYaw + fs->smoothBearing);

                /* ── WLS: instantaneous gradient (always computed for log) ─ */
                WlsGradientInput wIn;
                memcpy(wIn.pd, magnitudes, sizeof(wIn.pd));
                WlsGradientOutput wOutInstant;
                wlsGradientControllerUpdateInstantaneous(&wIn, &wOutInstant);

                /* ── Map update ────── */
                if (currentMode != MODE_MANUAL) {
                    wlsGradientControllerAddMapPoint(posX, posY, totalMag);
                }

                /* ── Map-based gradient (mirrors blimp.cpp estimate_gradient) */
                WlsGradientOutput wOutMap;
                bool gradValid = wlsGradientControllerUpdateMap(posX, posY, &wOutMap);
                int  mapSz     = wlsGradientControllerGetMapSize();

                bool gradReady = gradValid
                                 && mapSz >= (int)fusionMinMapPoints
                                 && wOutMap.gradMagnitude >= gradientThreshold;

                /* ── Heading fusion (port of blimp.cpp fusion block) ────────
                 *
                 * blimp.cpp logic:
                 *   if !grad_ready:
                 *       cmd_yaw = abs_bearing_yaw  (w_b=1, w_g=0)
                 *   else:
                 *       cmd_yaw = weighted_circular_mean(abs_bearing_yaw, W_BEARING,
                 *                                        grad_angle_world, W_GRADIENT)
                 *
                 * grad_angle from blimp.cpp is already world-frame (atan2(gy, gx)
                 * of the world-frame gradient). Here wOutMap.gradAngleDeg is in the
                 * drone body frame, so we add current yaw to convert to world frame.
                 * ───────────────────────────────────────────────────────────── */
                float cmdYaw  = absBearingYaw;
                float wB = 1.0f, wG = 0.0f;

                if (fs->bearingValid && gradReady) {
                    float gradAngleWorld = normalizeAngle(
                        currentYaw + wOutMap.gradAngleDeg);
                    wB = fusionWBearing;
                    wG = fusionWGradient;
                    cmdYaw = weightedCircularMean(absBearingYaw,   wB,
                                                  gradAngleWorld,  wG);
                } else if (!fs->bearingValid &&
                           fs->bearingInvalidFrames > bearingHoldFrames) {
                    /* Signal lost for more than bearingHoldFrames consecutive frames —
                     * fall back to hold-heading.  Short dropouts keep the last valid
                     * bearing direction to avoid large sudden jumps in spYawDeg. */
                    cmdYaw = currentYaw;
                }

                /* Store reading — navigator will consume these after the loop */
                freqReadings[f].frequency_hz = fs->freq;
                freqReadings[f].bearing_deg  = fs->smoothBearing;
                freqReadings[f].max_snr      = maxSnr;
                freqReadings[f].valid        = fs->bearingValid;

                /* Store per-frequency fused yaw for phase-2 selection */
                cmdYaws[f] = cmdYaw;

                /* Update LOG diagnostics (last frequency wins for single display) */
                logBearingAngle = fs->smoothBearing;
                logGradAngle    = wOutMap.gradAngleDeg;
                logGradMag      = wOutMap.gradMagnitude;
                logCmdYaw       = cmdYaw;
                logWB           = wB;
                logWG           = wG;
                logMapSize      = mapSz;
                logMaxSnr       = maxSnr;
                for (int i = 0; i < 8; i++) {
                    logChSnr[i] = (int16_t)(fs->channelSnr[i] * 100.0f);
                }
            }

            /* ── Phase 2: call navigator ONCE with all readings populated ── */
            if (currentMode == MODE_NAVIGATE) {
                WpNavSetpoint navSp = waypointNavigatorUpdate(
                    freqReadings, numTrackedFreqs);

                /* Pick the cmdYaw for the frequency the navigator is actively
                 * targeting.  waypointNavigatorGetCurrentFreq() returns the
                 * current waypoint's frequency; match it against freqStates[].
                 * Default to currentYaw if no match (e.g. COMPLETE state). */
                float navCmdYaw    = currentYaw;
                float activeFreq   = waypointNavigatorGetCurrentFreq();
                bool  activeFreqValid = false;
                int   activeF      = -1;          /* index into freqStates[] */
                for (int f = 0; f < numTrackedFreqs; f++) {
                    /* Use a tight tolerance (0.5 Hz) — frequencies are at least
                     * 50 Hz apart in our mission so there is no ambiguity. */
                    if (fabsf(freqStates[f].freq - activeFreq) < 0.5f) {
                        navCmdYaw       = cmdYaws[f];
                        activeFreqValid = freqReadings[f].valid;
                        activeF         = f;
                        break;
                    }
                }

                spFwdVel  = 0.0f;
                spHolding = false;

                /* Full forward speed when properly aligned and approaching */
                if (activeFreqValid && navSp.state == WP_NAV_APPROACHING) {
                    spFwdVel = navFwdSpeed;
                }
                /* Slow forward creep during ALIGNING when roughly facing target —
                 * avoids stationary spinning while the bearing converges. */
                else if (activeFreqValid && navSp.state == WP_NAV_ALIGNING && activeF >= 0) {
                    if (fabsf(freqStates[activeF].smoothBearing) < navAlignFwdTol) {
                        spFwdVel = navAlignFwdSpeed;
                    }
                }

                if (navSp.state == WP_NAV_HOLDING ||
                    navSp.state == WP_NAV_COMPLETE) {
                    spFwdVel  = 0.0f;
                    spHolding = true;   /* lock yaw to live currentYaw */
                }

                spYawDeg = navCmdYaw;

                /* Overwrite log vars with active-frequency data */
                if (activeF >= 0) {
                    logBearingAngle = freqStates[activeF].smoothBearing;
                    logMaxSnr       = freqStates[activeF].maxSnr;
                    logCmdYaw       = cmdYaws[activeF];
                    for (int i = 0; i < 8; i++) {
                        logChSnr[i] = (int16_t)(freqStates[activeF].channelSnr[i] * 100.0f);
                    }
                }
            }
            /* MODE_DATA_GATHER: LOG variables updated, no setpoint targets */
        }

        /* ── 6. Inject setpoint at 100 Hz in NAVIGATE mode ──────────────── */
        /* Keeps the commander's stale-setpoint watchdog satisfied every tick
         * rather than only on FFT frames (~2 Hz). spHolding=true commands the
         * live currentYaw so the drone holds its actual heading with no jump. */
        if (currentMode == MODE_NAVIGATE) {
            float yaw = spHolding ? currentYaw : spYawDeg;
            injectSetpoint(spFwdVel, yaw);
        }

        /* ── MODE_MANUAL / MODE_DATA_GATHER: external commander controls drone */

        vTaskDelayUntil(&lastWake, M2T(10));   /* 100 Hz */
    }
}

/* ──────────────────────────────────────────────────────────────────────────
 * Init
 * ────────────────────────────────────────────────────────────────────────── */
void modeManagerInit(void)
{
    if (!pdFftAnalyzerInit()) {
        DEBUG_PRINT("MODEMGR: FFT init FAILED\n");
    }
    bearingAngleControllerInit();
    wlsGradientControllerInit();
    waypointNavigatorInit();

    /* Load default mission */
    for (size_t i = 0; i < DEFAULT_MISSION_LEN; i++) {
        waypointNavigatorAddWaypoint(DEFAULT_MISSION[i].freq,
                                      DEFAULT_MISSION[i].dwell_ms);
    }
    waypointNavigatorBuildFreqTable();

    /* Populate frequency tracking slots from navigator */
    numTrackedFreqs = waypointNavigatorGetNumUniqueFreqs();
    for (int f = 0; f < numTrackedFreqs; f++) {
        freqStates[f].freq               = waypointNavigatorGetUniqueFreq(f);
        freqStates[f].smoothBearing        = 0.0f;
        freqStates[f].bearingValid         = false;
        freqStates[f].bearingInvalidFrames = 0;
        freqStates[f].totalMagnitude       = 0.0f;
        freqStates[f].maxSnr               = 0.0f;
    }

    STATIC_MEM_TASK_CREATE(modeTask, modeTask, "modeTask", NULL, MODE_TASK_PRIORITY);
    DEBUG_PRINT("MODEMGR: init OK (%d tracked frequencies)\n", numTrackedFreqs);
}

DroneMode modeManagerGetMode(void) { return currentMode; }

/* ── PARAM ─────────────────────────────────────────────────────────────── */
PARAM_GROUP_START(nav)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, mode,        &currentMode,        modeParamCallback)
    PARAM_ADD(PARAM_FLOAT,               altTarget,   &navAltTarget)
    PARAM_ADD(PARAM_FLOAT,               maxVel,      &navMaxVel)
    PARAM_ADD(PARAM_FLOAT,               fwdSpeed,    &navFwdSpeed)
    PARAM_ADD(PARAM_UINT32,              pdTimeout,   &pdTimeoutMs)
    /* Fusion tuning */
    PARAM_ADD(PARAM_FLOAT,               wBearing,    &fusionWBearing)
    PARAM_ADD(PARAM_FLOAT,               wGradient,   &fusionWGradient)
    PARAM_ADD(PARAM_FLOAT,               smoothFact,  &bearingSmoothFactor)
    PARAM_ADD(PARAM_FLOAT,               gradThresh,  &gradientThreshold)
    PARAM_ADD(PARAM_FLOAT,               minLight,    &minTotalLight)
    PARAM_ADD(PARAM_UINT8,               minMapPts,    &fusionMinMapPoints)
    PARAM_ADD(PARAM_UINT8,               bearingHold,  &bearingHoldFrames)
    /* Aligning forward motion */
    PARAM_ADD(PARAM_FLOAT,               alignFwdTol,  &navAlignFwdTol)
    PARAM_ADD(PARAM_FLOAT,               alignFwdSpd,  &navAlignFwdSpeed)
PARAM_GROUP_STOP(nav)

/* ── LOG ───────────────────────────────────────────────────────────────── */
LOG_GROUP_START(nav)
    LOG_ADD(LOG_UINT8,  mode,    &currentMode)
    LOG_ADD(LOG_FLOAT,  bearing, &logBearingAngle)
    LOG_ADD(LOG_FLOAT,  gradAng, &logGradAngle)
    LOG_ADD(LOG_FLOAT,  gradMag, &logGradMag)
    LOG_ADD(LOG_FLOAT,  cmdYaw,  &logCmdYaw)
    LOG_ADD(LOG_FLOAT,  wB,      &logWB)
    LOG_ADD(LOG_FLOAT,  wG,      &logWG)
    LOG_ADD(LOG_FLOAT,  snr,     &logMaxSnr)
    LOG_ADD(LOG_INT32,  mapSize, &logMapSize)
    /* Per-channel SNR ×100 as int16 (divide by 100 in Python to recover SNR) */
    LOG_ADD(LOG_INT16,  snr0,    &logChSnr[0])
    LOG_ADD(LOG_INT16,  snr1,    &logChSnr[1])
    LOG_ADD(LOG_INT16,  snr2,    &logChSnr[2])
    LOG_ADD(LOG_INT16,  snr3,    &logChSnr[3])
    LOG_ADD(LOG_INT16,  snr4,    &logChSnr[4])
    LOG_ADD(LOG_INT16,  snr5,    &logChSnr[5])
    LOG_ADD(LOG_INT16,  snr6,    &logChSnr[6])
    LOG_ADD(LOG_INT16,  snr7,    &logChSnr[7])
LOG_GROUP_STOP(nav)
