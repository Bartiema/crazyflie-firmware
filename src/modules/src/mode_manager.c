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
static float minTotalLight         = 1.0f;   /* MIN_TOTAL_LIGHT — aggregate gate */
static uint8_t fusionMinMapPoints  = 3;      /* MIN_MAP_POINTS */

/* ──────────────────────────────────────────────────────────────────────────
 * Flight parameters
 * ────────────────────────────────────────────────────────────────────────── */
static float navAltTarget   = 1.0f;   /* hold altitude via Flow Deck */
static float navMaxVel      = 0.30f;  /* forward velocity clamp (m/s) */
static float navMaxYaw      = 60.0f;  /* yaw rate clamp (deg/s) */
static float navFwdSpeed    = 0.20f;  /* forward speed when approaching */
static float navYawGain     = 2.0f;   /* heading error (deg) → yaw rate (deg/s) */
static uint32_t pdTimeoutMs = 500U;   /* revert MANUAL if no PD data */

/* ──────────────────────────────────────────────────────────────────────────
 * Default mission — override via upload_frequency_waypoints() from Python
 * ────────────────────────────────────────────────────────────────────────── */
typedef struct { float freq; float dwell_ms; } WpEntry;
static const WpEntry DEFAULT_MISSION[] = {
    { 150.0f, 3000.0f },
    { 200.0f, 3000.0f },
    { 150.0f, 2000.0f },
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
    float  smoothBearing;    /* low-pass filtered relative bearing (deg) */
    bool   bearingValid;
    float  totalMagnitude;   /* sum of all channel FFT magnitudes */
    float  maxSnr;
} FreqState;

static FreqState  freqStates[MAX_TRACKED_FREQS];
static int        numTrackedFreqs = 0;

/* Per-frequency readings passed to waypoint navigator */
static WpNavFreqReading freqReadings[MAX_TRACKED_FREQS];


/* Logged fusion diagnostics */
static float logBearingAngle = 0.0f;
static float logGradAngle    = 0.0f;
static float logGradMag      = 0.0f;
static float logCmdYaw       = 0.0f;
static float logWB           = 1.0f;
static float logWG           = 0.0f;
static int32_t logMapSize    = 0;

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
    }
    currentMode = req;
    DEBUG_PRINT("MODEMGR: mode → %d\n", (int)currentMode);
}

static void modeParamCallback(void) { applyModeChange(currentMode); }

/* ──────────────────────────────────────────────────────────────────────────
 * Setpoint injection
 * ────────────────────────────────────────────────────────────────────────── */
static void injectSetpoint(float vx, float yaw_rate_deg)
{
    vx           = constrain(vx,           -navMaxVel, navMaxVel);
    yaw_rate_deg = constrain(yaw_rate_deg, -navMaxYaw, navMaxYaw);

    setpoint_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.mode.x            = modeVelocity;
    sp.mode.y            = modeVelocity;
    sp.mode.z            = modeAbs;
    sp.mode.yaw          = modeVelocity;
    sp.velocity.x        = vx;
    sp.velocity.y        = 0.0f;
    sp.position.z        = navAltTarget;
    sp.attitudeRate.yaw  = yaw_rate_deg;
    commanderSetSetpoint(&sp, COMMANDER_PRIORITY_EXTRX);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Main 100 Hz task
 * ────────────────────────────────────────────────────────────────────────── */
#define MODE_TASK_STACKSIZE  (6 * configMINIMAL_STACK_SIZE)
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

        /* ── 3. Run FFT when a new window is ready ──────────────────────── */
        bool newSpectrum = false;
        if (pdFftAnalyzerWindowReady()) {
            pdFftAnalyzerRun();
            newSpectrum = true;
        }

        /* ── 4. Process new spectrum ────────────────────────────────────── */
        if (newSpectrum && numTrackedFreqs > 0) {

            /* Read current drone state (read-only, safe to call from any task).
             * estimatorKalmanGetEstimatedPos() is the correct getter for position
             * when the Kalman estimator is active (required with Flow Deck v2).
             * Attitude (yaw) is always available via the attitude log variables
             * but we access it through the state struct directly here. */
            float currentYaw = 0.0f;
            float posX       = 0.0f;
            float posY       = 0.0f;

            {
                point_t pos;
                estimatorKalmanGetEstimatedPos(&pos);
                posX = pos.x;
                posY = pos.y;

                /* Yaw is available from the sensfusion / attitude estimator */
                float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
                sensfusion6GetEulerRPY(&roll, &pitch, &yaw);
                currentYaw = yaw;
            }

            for (int f = 0; f < numTrackedFreqs; f++) {
                FreqState *fs = &freqStates[f];

                /* Extract per-channel magnitudes at this frequency */
                float magnitudes[BA_SENSOR_COUNT];
                float totalMag = 0.0f, maxSnr = 0.0f;

                for (int ch = 0; ch < BA_SENSOR_COUNT; ch++) {
                    PdFreqResult res;
                    pdFftAnalyzerGetFrequency(ch, fs->freq, 2.0f, &res);
                    magnitudes[ch] = res.magnitude;
                    totalMag      += res.magnitude;
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
                    /* smooth_bearing += SMOOTH_FACTOR * normalize(bearing - smooth)
                     * Mirrors blimp.cpp exactly. bearing is relative to drone body. */
                    float diff = normalizeAngle(baOut.bearing_deg - fs->smoothBearing);
                    fs->smoothBearing = normalizeAngle(
                        fs->smoothBearing + bearingSmoothFactor * diff);
                }

                /* Absolute world-frame bearing heading */
                float absBearingYaw = normalizeAngle(currentYaw + fs->smoothBearing);

                /* ── WLS: instantaneous gradient (always computed for log) ─ */
                WlsGradientInput wIn;
                memcpy(wIn.pd, magnitudes, sizeof(wIn.pd));
                WlsGradientOutput wOutInstant;
                wlsGradientControllerUpdateInstantaneous(&wIn, &wOutInstant);

                /* ── Map update (mirrors blimp.cpp update_map) ───────────── */
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
                } else if (!fs->bearingValid) {
                    /* No signal — hover, matches blimp.cpp "No light signal" branch */
                    cmdYaw = currentYaw;
                }

                /* Store for waypoint navigator and LOG */
                freqReadings[f].frequency_hz = fs->freq;
                freqReadings[f].bearing_deg  = fs->smoothBearing;
                freqReadings[f].max_snr      = maxSnr;
                freqReadings[f].valid        = fs->bearingValid;

                /* Update LOG diagnostics (last frequency wins for single display) */
                logBearingAngle = fs->smoothBearing;
                logGradAngle    = wOutMap.gradAngleDeg;
                logGradMag      = wOutMap.gradMagnitude;
                logCmdYaw       = cmdYaw;
                logWB           = wB;
                logWG           = wG;
                logMapSize      = mapSz;

                /* ── Mode execution ──────────────────────────────────────── */
                if (currentMode == MODE_NAVIGATE) {
                    /* Derive yaw-rate error from cmd_yaw vs current_yaw.
                     * A proportional controller converts the heading error to
                     * a yaw rate command (deg/s). Gain tunable via PARAM.   */
                    float yawErr     = normalizeAngle(cmdYaw - currentYaw);
                    float yawRateCmd = yawErr * navYawGain;

                    /* Waypoint navigator provides forward velocity */
                    WpNavSetpoint navSp = waypointNavigatorUpdate(
                        freqReadings, numTrackedFreqs);

                    float fwdVel = 0.0f;
                    if (fs->bearingValid &&
                        (navSp.state == WP_NAV_APPROACHING ||
                         navSp.state == WP_NAV_ALIGNING ||
                         navSp.state == WP_NAV_SEARCHING)) {
                        fwdVel = navFwdSpeed;
                    }
                    if (navSp.state == WP_NAV_HOLDING ||
                        navSp.state == WP_NAV_COMPLETE) {
                        fwdVel = 0.0f;
                        yawRateCmd = 0.0f;
                    }

                    injectSetpoint(fwdVel, yawRateCmd);
                }
                /* MODE_DATA_GATHER: all LOG variables updated, no setpoint */
            }
        }

        /* ── MODE_MANUAL: nothing to do ───────────────────────────────── */

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
        freqStates[f].freq         = waypointNavigatorGetUniqueFreq(f);
        freqStates[f].smoothBearing = 0.0f;
        freqStates[f].bearingValid  = false;
        freqStates[f].totalMagnitude= 0.0f;
        freqStates[f].maxSnr        = 0.0f;
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
    PARAM_ADD(PARAM_FLOAT,               maxYaw,      &navMaxYaw)
    PARAM_ADD(PARAM_FLOAT,               fwdSpeed,    &navFwdSpeed)
    PARAM_ADD(PARAM_FLOAT,               yawGain,     &navYawGain)
    PARAM_ADD(PARAM_UINT32,              pdTimeout,   &pdTimeoutMs)
    /* Fusion tuning */
    PARAM_ADD(PARAM_FLOAT,               wBearing,    &fusionWBearing)
    PARAM_ADD(PARAM_FLOAT,               wGradient,   &fusionWGradient)
    PARAM_ADD(PARAM_FLOAT,               smoothFact,  &bearingSmoothFactor)
    PARAM_ADD(PARAM_FLOAT,               gradThresh,  &gradientThreshold)
    PARAM_ADD(PARAM_FLOAT,               minLight,    &minTotalLight)
    PARAM_ADD(PARAM_UINT8,               minMapPts,   &fusionMinMapPoints)
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
    LOG_ADD(LOG_INT32,  mapSize, &logMapSize)
LOG_GROUP_STOP(nav)
