/**
 * mode_manager.c
 *
 * Main 100 Hz orchestration task.
 *
 * The waypoint navigator is absorbed directly into this file — there is no
 * longer a separate waypoint_navigator module.  Each navigation state has a
 * dedicated handler that is responsible for one thing only:
 *
 *   handleSearching()  — no valid bearing; rotate to scan
 *   handleAligning()   — bearing valid but off-axis; rotate toward target
 *   handleApproaching()— aligned; fly forward while tracking bearing
 *   handleDwelling()   — arrived; hold position for dwell_ms then advance
 *
 * Each handler sets spFwdVel / spYawDeg and returns the next NavState.
 * The 100 Hz task injects the live setpoint every tick; FFT-based state
 * updates happen at ~2 Hz (every PD_FFT_AVERAGES hops).
 *
 * Yaw control uses absolute angle setpoints throughout so the CrazyFlie's
 * tuned position controller handles the actual rotation.  SEARCHING is the
 * only state that advances spYawDeg at 100 Hz (continuous scan rotation);
 * all other states set spYawDeg once per FFT frame from the bearing.
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
#include "mode_manager.h"

/* ── Navigation state ───────────────────────────────────────────────────── */
/* Values chosen to match the old wpNav.state numbers that Python already
 * knows about (DWELLING=4 was HOLDING, COMPLETE=6 was COMPLETE). */
typedef enum {
    NAV_IDLE       = 0,
    NAV_SEARCHING  = 1,
    NAV_ALIGNING   = 2,
    NAV_APPROACHING= 3,
    NAV_DWELLING   = 4,
    NAV_COMPLETE   = 6,
} NavState;

/* ── Waypoints ──────────────────────────────────────────────────────────── */
#define NAV_MAX_WAYPOINTS  10

typedef struct { float freq; float dwell_ms; } NavWaypoint;
static NavWaypoint waypoints[NAV_MAX_WAYPOINTS];
static int         waypointCount = 0;
static int         waypointIndex = 0;

/* ── Navigation parameters (tunable via wpNav param group) ─────────────── */
static float   navAlignTol  = 15.0f;   /* ±deg to be considered aligned     */
static float   navAcqSnr    =  5.0f;   /* min SNR to trust bearing          */
static float   navArrSnr    =  7.0f;   /* SNR threshold for "arrived"       */
static float   navYawRate   = 30.0f;   /* search scan speed (deg/s)         */

/* ── Fusion parameters ──────────────────────────────────────────────────── */
static float    fusionWBearing      = 0.5f;
static float    fusionWGradient     = 0.5f;
static float    bearingSmoothFactor = 0.7f;  /* 0=immediate, 1=frozen; default keeps 30 % old per frame */
static float    gradientThreshold   = 0.5f;
static float    minTotalLight       = 0.05f;
static uint8_t  fusionMinMapPoints  = 3;
static uint8_t  bearingHoldFrames   = 3;

/* ── Flight parameters ──────────────────────────────────────────────────── */
static float    navAltTarget    = 1.0f;
static float    navMaxVel       = 0.30f;
static float    navFwdSpeed     = 0.20f;
static float    navAlignFwdTol  = 30.0f;
static float    navAlignFwdSpeed= 0.10f;
static uint32_t pdTimeoutMs     = 500U;

/* ── Default mission ────────────────────────────────────────────────────── */
static const NavWaypoint DEFAULT_MISSION[] = {
    { 150.0f, 1000.0f },
    { 170.0f, 1000.0f },
    { 150.0f, 1000.0f },
};
#define DEFAULT_MISSION_LEN  (sizeof(DEFAULT_MISSION) / sizeof(DEFAULT_MISSION[0]))

/* ── Runtime state ──────────────────────────────────────────────────────── */
static DroneMode currentMode  = MODE_MANUAL;
static uint32_t  lastPdOkTick = 0;

static NavState  navState     = NAV_IDLE;
static uint32_t  dwellStart   = 0;

/* Active-frequency tracking */
static float    activeFreq           = 0.0f;
static float    smoothBearing        = 0.0f;
static bool     bearingInitialized   = false;  /* true after first valid raw reading */
static bool     bearingValid         = false;
static uint8_t  bearingInvalidFrames = 0;

/* Live setpoint — injected at 100 Hz */
static float spFwdVel = 0.0f;
static float spYawDeg = 0.0f;

/* ── Log variables ──────────────────────────────────────────────────────── */
static int16_t logChSnr[8];
static float   logBearingAngle = 0.0f;
static float   logGradAngle    = 0.0f;
static float   logGradMag      = 0.0f;
static float   logCmdYaw       = 0.0f;
static float   logWB           = 1.0f;
static float   logWG           = 0.0f;
static int32_t logMapSize      = 0;
static float   logMaxSnr       = 0.0f;

/* ── Helpers ────────────────────────────────────────────────────────────── */
static float normalizeAngle(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float weightedCircularMean(float a1, float w1, float a2, float w2)
{
    float r1 = a1 * (float)M_PI / 180.0f;
    float r2 = a2 * (float)M_PI / 180.0f;
    float cx = w1 * cosf(r1) + w2 * cosf(r2);
    float cy = w1 * sinf(r1) + w2 * sinf(r2);
    return atan2f(cy, cx) * 180.0f / (float)M_PI;
}

static void resetActiveState(void)
{
    smoothBearing        = 0.0f;
    bearingInitialized   = false;
    bearingValid         = false;
    bearingInvalidFrames = 0;
    spFwdVel             = 0.0f;
    wlsGradientControllerClearMap();
    pdFftAnalyzerResetAccumulator();
    memset(logChSnr, 0, sizeof(logChSnr));
    logMaxSnr = 0.0f; logBearingAngle = 0.0f;
    logGradAngle = 0.0f; logGradMag = 0.0f;
    logMapSize = 0; logWB = 1.0f; logWG = 0.0f;
}

/* ── Navigation state handlers ──────────────────────────────────────────── */

/* No valid bearing — rotate continuously to scan for the target.
 * spYawDeg is advanced in the 100 Hz loop; this handler only checks
 * whether the signal has been acquired. */
static NavState handleSearching(float snr, bool valid)
{
    spFwdVel = 0.0f;
    if (valid && snr > navAcqSnr)
        return NAV_ALIGNING;
    return NAV_SEARCHING;
}

/* Signal acquired but not aligned — rotate toward the target.
 * Allows a slow forward creep when roughly facing the right direction. */
static NavState handleAligning(float bearing, float snr, bool valid, float currentYaw)
{
    spFwdVel = (fabsf(bearing) < navAlignFwdTol) ? navAlignFwdSpeed : 0.0f;
    spYawDeg = normalizeAngle(currentYaw + smoothBearing);
    if (!valid || snr < navAcqSnr)     return NAV_SEARCHING;
    if (fabsf(bearing) < navAlignTol)  return NAV_APPROACHING;
    return NAV_ALIGNING;
}

/* Aligned — fly forward while continuously tracking the bearing. */
static NavState handleApproaching(float bearing, float snr, bool valid, float currentYaw)
{
    spFwdVel = navFwdSpeed;
    spYawDeg = normalizeAngle(currentYaw + smoothBearing);
    if (!valid || snr < navAcqSnr)             return NAV_SEARCHING;
    if (fabsf(bearing) > navAlignTol * 2.0f)   return NAV_ALIGNING;
    if (snr > navArrSnr) {
        dwellStart = xTaskGetTickCount();
        return NAV_DWELLING;
    }
    return NAV_APPROACHING;
}

/* Arrived — hold position for dwell_ms then advance to the next waypoint. */
static NavState handleDwelling(void)
{
    spFwdVel = 0.0f;
    uint32_t elapsed = (xTaskGetTickCount() - dwellStart) * portTICK_RATE_MS;
    if (elapsed < (uint32_t)waypoints[waypointIndex].dwell_ms)
        return NAV_DWELLING;

    /* Dwell complete — advance waypoint */
    waypointIndex++;
    if (waypointIndex >= waypointCount)
        return NAV_COMPLETE;

    /* Frequency change will be detected at the next FFT frame, which
     * will call resetActiveState() automatically. */
    return NAV_SEARCHING;
}

/* ── Mode change ────────────────────────────────────────────────────────── */
static bool safeToNavigate(void)
{
    if (!pdDeckIsReady()) {
        DEBUG_PRINT("MODEMGR: NAVIGATE refused — PD deck not ready\n");
        return false;
    }
    return true;
}

static void startMission(void)
{
    waypointIndex = 0;
    navState      = NAV_SEARCHING;
    activeFreq    = 0.0f;   /* force frequency-change detection on first FFT frame */
    spYawDeg      = 0.0f;
    resetActiveState();
}

static void applyModeChange(DroneMode req)
{
    if (req == MODE_NAVIGATE) {
        if (!safeToNavigate()) { currentMode = MODE_MANUAL; return; }
        startMission();
    }
    currentMode = req;
    DEBUG_PRINT("MODEMGR: mode -> %d\n", (int)currentMode);
}

static void modeParamCallback(void) { applyModeChange(currentMode); }

/* ── Setpoint injection ─────────────────────────────────────────────────── */
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

/* ── Main task ──────────────────────────────────────────────────────────── */
#define MODE_TASK_STACKSIZE  (10 * configMINIMAL_STACK_SIZE)
#define MODE_TASK_PRIORITY   1

STATIC_MEM_TASK_ALLOC(modeTask, MODE_TASK_STACKSIZE);

static void modeTask(void *param)
{
    (void)param;
    systemWaitStart();
    TickType_t lastWake = xTaskGetTickCount();

    while (1) {

        /* 1. PD health check */
        if (pdDeckIsReady()) lastPdOkTick = xTaskGetTickCount();

        /* 2. PD timeout safety */
        if (currentMode != MODE_MANUAL) {
            uint32_t sinceMs = (xTaskGetTickCount() - lastPdOkTick)
                               * portTICK_RATE_MS;
            if (sinceMs > pdTimeoutMs) {
                DEBUG_PRINT("MODEMGR: PD timeout -> MANUAL\n");
                currentMode = MODE_MANUAL;
            }
        }

        /* 3. Current yaw — needed every tick */
        float currentYaw = 0.0f;
        {
            float roll = 0.0f, pitch = 0.0f;
            sensfusion6GetEulerRPY(&roll, &pitch, &currentYaw);
        }

        /* 4. Run FFT when a new hop is ready */
        bool newSpectrum = false;
        if (pdFftAnalyzerWindowReady())
            newSpectrum = pdFftAnalyzerRun();

        /* 5. Process new spectrum ─────────────────────────────────────────── */
        if (newSpectrum) {
            float targetFreq = (waypointIndex < waypointCount)
                               ? waypoints[waypointIndex].freq : 0.0f;

            /* Frequency change: new waypoint or mission start — reset all state.
             * We do NOT skip this frame: the stale spectrum from the old
             * frequency will have near-zero SNR at the new frequency, so the
             * bearing-valid gate naturally rejects it without a forced skip.
             * Skipping caused an unwanted SEARCHING scan-rotation on every
             * waypoint transition. */
            if (fabsf(targetFreq - activeFreq) > 0.5f) {
                DEBUG_PRINT("MODEMGR: freq %.0f -> %.0f Hz, re-init\n",
                            (double)activeFreq, (double)targetFreq);
                activeFreq = targetFreq;
                resetActiveState();
            }

            if (activeFreq > 0.0f) {

                /* ── FFT extraction ───────────────────────────────────── */
                float magnitudes[BA_SENSOR_COUNT];
                float totalMag = 0.0f, maxSnrLocal = 0.0f;
                for (int ch = 0; ch < BA_SENSOR_COUNT; ch++) {
                    PdFreqResult res;
                    pdFftAnalyzerGetFrequency(ch, activeFreq, 2.0f, &res);
                    magnitudes[ch]  = res.magnitude;
                    logChSnr[ch]    = (int16_t)(res.snr * 100.0f);
                    totalMag       += res.magnitude;
                    if (res.snr > maxSnrLocal) maxSnrLocal = res.snr;
                }
                logMaxSnr = maxSnrLocal;

                /* ── Bearing estimation ───────────────────────────────── */
                BearingAngleInput baIn;
                memcpy(baIn.magnitude, magnitudes, sizeof(baIn.magnitude));
                baIn.max_snr = maxSnrLocal;
                BearingAngleOutput baOut;
                bearingAngleControllerUpdate(&baIn, &baOut);

                bearingValid = baOut.valid && (totalMag > minTotalLight);
                if (bearingValid) {
                    bearingInvalidFrames = 0;
                    if (!bearingInitialized) {
                        /* First valid reading after reset — snap directly so the
                         * filter starts from the true bearing, not from zero. */
                        smoothBearing      = baOut.bearing_deg;
                        bearingInitialized = true;
                    } else {
                        float diff = normalizeAngle(baOut.bearing_deg - smoothBearing);
                        /* Convention: 0 = no smoothing (immediate), 1 = fully frozen.
                         * (1 - factor) is the weight given to the new measurement. */
                        smoothBearing = normalizeAngle(
                            smoothBearing + (1.0f - bearingSmoothFactor) * diff);
                    }
                } else {
                    if (bearingInvalidFrames < 255) bearingInvalidFrames++;
                }
                logBearingAngle = smoothBearing;

                /* ── WLS map update + gradient ────────────────────────── */
                float posX = 0.0f, posY = 0.0f;
                {
                    point_t pos;
                    estimatorKalmanGetEstimatedPos(&pos);
                    posX = pos.x; posY = pos.y;
                }
                if (currentMode != MODE_MANUAL)
                    wlsGradientControllerAddMapPoint(posX, posY, totalMag);

                WlsGradientOutput wOutMap;
                bool gradValid = wlsGradientControllerUpdateMap(posX, posY, &wOutMap);
                int  mapSz     = wlsGradientControllerGetMapSize();
                bool gradReady = gradValid
                                 && mapSz >= (int)fusionMinMapPoints
                                 && wOutMap.gradMagnitude >= gradientThreshold;

                logGradAngle = wOutMap.gradAngleDeg;
                logGradMag   = wOutMap.gradMagnitude;
                logMapSize   = mapSz;

                /* ── Heading fusion ───────────────────────────────────── */
                /* For the yaw SETPOINT we use the raw (unfiltered) bearing
                 * converted to a world-frame angle:
                 *   currentYaw + rawBearing = fixed world angle of the source
                 * This is stable because the source doesn't move; filtering a
                 * relative bearing instead causes per-frame overshoot and
                 * oscillation.  smoothBearing is kept separately for state
                 * decisions (alignment check, validity hysteresis) where
                 * noise-resistance matters more than instant response. */
                float absBearingYaw = bearingValid
                    ? normalizeAngle(baOut.bearing_deg + currentYaw)
                    : normalizeAngle(smoothBearing + currentYaw);
                float fusedYaw = absBearingYaw;
                float wB = 1.0f, wG = 0.0f;

                if (bearingValid && gradReady) {
                    float gradAngleWorld = normalizeAngle(
                        currentYaw + wOutMap.gradAngleDeg);
                    wB = fusionWBearing; wG = fusionWGradient;
                    fusedYaw = weightedCircularMean(absBearingYaw, wB,
                                                    gradAngleWorld, wG);
                } else if (!bearingValid &&
                           bearingInvalidFrames > bearingHoldFrames) {
                    fusedYaw = currentYaw;
                }
                logCmdYaw = fusedYaw; logWB = wB; logWG = wG;

                /* ── State machine update (FFT-rate) ──────────────────── */
                if (currentMode == MODE_NAVIGATE) {
                    switch (navState) {
                        case NAV_SEARCHING:
                            navState = handleSearching(maxSnrLocal, bearingValid);
                            break;
                        case NAV_ALIGNING:
                            navState = handleAligning(smoothBearing, maxSnrLocal,
                                                      bearingValid, currentYaw);
                            break;
                        case NAV_APPROACHING:
                            navState = handleApproaching(smoothBearing, maxSnrLocal,
                                                         bearingValid, currentYaw);
                            break;
                        default:
                            break;
                    }

                    /* ALIGNING and APPROACHING use the fused heading */
                    if (navState == NAV_ALIGNING || navState == NAV_APPROACHING)
                        spYawDeg = fusedYaw;
                }
            }
        }

        /* 6. 100 Hz navigation updates ─────────────────────────────────── */
        if (currentMode == MODE_NAVIGATE) {

            /* SEARCHING: only rotate to scan when we have no bearing at all.
             * On brief SNR dropouts (bearingInitialized=true) we hold the
             * last heading so the drone doesn't rotate away from the source
             * while the signal is temporarily weak. */
            if (navState == NAV_SEARCHING && !bearingInitialized)
                spYawDeg = normalizeAngle(spYawDeg + navYawRate * 0.01f);

            /* DWELLING: check dwell timer at full 100 Hz resolution */
            if (navState == NAV_DWELLING)
                navState = handleDwelling();

            injectSetpoint(spFwdVel, spYawDeg);
        }

        vTaskDelayUntil(&lastWake, M2T(10));
    }
}

/* ── Init ───────────────────────────────────────────────────────────────── */
void modeManagerInit(void)
{
    if (!pdFftAnalyzerInit()) DEBUG_PRINT("MODEMGR: FFT init FAILED\n");
    bearingAngleControllerInit();
    wlsGradientControllerInit();

    /* Load default mission */
    for (size_t i = 0; i < DEFAULT_MISSION_LEN; i++) {
        if (waypointCount < NAV_MAX_WAYPOINTS) {
            waypoints[waypointCount].freq     = DEFAULT_MISSION[i].freq;
            waypoints[waypointCount].dwell_ms = DEFAULT_MISSION[i].dwell_ms;
            waypointCount++;
        }
    }

    STATIC_MEM_TASK_CREATE(modeTask, modeTask, "modeTask", NULL,
                           MODE_TASK_PRIORITY);
    DEBUG_PRINT("MODEMGR: init OK (%d waypoints)\n", waypointCount);
}

DroneMode modeManagerGetMode(void) { return currentMode; }

/* ── Mission reset (param callback) ─────────────────────────────────────── */
static uint8_t navResetFlag = 0;
static void navResetCallback(void)
{
    if (navResetFlag) {
        waypointIndex = 0;
        navState      = NAV_IDLE;
        navResetFlag  = 0;
    }
}

/* ── PARAM ──────────────────────────────────────────────────────────────── */
PARAM_GROUP_START(nav)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, mode,        &currentMode,        modeParamCallback)
    PARAM_ADD(PARAM_FLOAT,               altTarget,   &navAltTarget)
    PARAM_ADD(PARAM_FLOAT,               maxVel,      &navMaxVel)
    PARAM_ADD(PARAM_FLOAT,               fwdSpeed,    &navFwdSpeed)
    PARAM_ADD(PARAM_UINT32,              pdTimeout,   &pdTimeoutMs)
    PARAM_ADD(PARAM_FLOAT,               wBearing,    &fusionWBearing)
    PARAM_ADD(PARAM_FLOAT,               wGradient,   &fusionWGradient)
    PARAM_ADD(PARAM_FLOAT,               smoothFact,  &bearingSmoothFactor)
    PARAM_ADD(PARAM_FLOAT,               gradThresh,  &gradientThreshold)
    PARAM_ADD(PARAM_FLOAT,               minLight,    &minTotalLight)
    PARAM_ADD(PARAM_UINT8,               minMapPts,   &fusionMinMapPoints)
    PARAM_ADD(PARAM_UINT8,               bearingHold, &bearingHoldFrames)
    PARAM_ADD(PARAM_FLOAT,               alignFwdTol, &navAlignFwdTol)
    PARAM_ADD(PARAM_FLOAT,               alignFwdSpd, &navAlignFwdSpeed)
PARAM_GROUP_STOP(nav)

/* wpNav group kept for Python compatibility (same param names as before) */
PARAM_GROUP_START(wpNav)
    PARAM_ADD(PARAM_FLOAT, alignTol, &navAlignTol)
    PARAM_ADD(PARAM_FLOAT, acqSnr,   &navAcqSnr)
    PARAM_ADD(PARAM_FLOAT, arrSnr,   &navArrSnr)
    PARAM_ADD(PARAM_FLOAT, yawRate,  &navYawRate)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, reset, &navResetFlag, navResetCallback)
PARAM_GROUP_STOP(wpNav)

/* ── LOG ────────────────────────────────────────────────────────────────── */
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
    LOG_ADD(LOG_INT16,  snr0,    &logChSnr[0])
    LOG_ADD(LOG_INT16,  snr1,    &logChSnr[1])
    LOG_ADD(LOG_INT16,  snr2,    &logChSnr[2])
    LOG_ADD(LOG_INT16,  snr3,    &logChSnr[3])
    LOG_ADD(LOG_INT16,  snr4,    &logChSnr[4])
    LOG_ADD(LOG_INT16,  snr5,    &logChSnr[5])
    LOG_ADD(LOG_INT16,  snr6,    &logChSnr[6])
    LOG_ADD(LOG_INT16,  snr7,    &logChSnr[7])
LOG_GROUP_STOP(nav)

/* wpNav log group kept for Python compatibility */
LOG_GROUP_START(wpNav)
    LOG_ADD(LOG_UINT8, state, &navState)
    LOG_ADD(LOG_UINT8, wpIdx, &waypointIndex)
LOG_GROUP_STOP(wpNav)
