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
 * tuned position controller handles the actual rotation.  spYawDeg is set
 * once per FFT frame from the raw bearing (world-frame: currentYaw + rawBearing).
 * Scan rotation in SEARCHING only runs when no bearing has been acquired yet;
 * brief SNR dropouts hold the last heading instead of rotating away.
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
    NAV_IDLE        = 0,
    NAV_SEARCHING   = 1,
    NAV_ALIGNING    = 2,
    NAV_APPROACHING = 3,
    NAV_DWELLING    = 4,
    NAV_RECOVERING  = 5,   /* SNR lost but gradient map reliable — keep moving */
    NAV_COMPLETE    = 6,
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

/* Per-channel magnitude IIR — applied after Welch publication.
 * 0 = no smoothing (pass-through), 1 = fully frozen.
 * Smoothed magnitudes feed the bearing estimator; smoothed SNR
 * is reconstructed from the same smoothed magnitude ÷ the
 * current-frame noise floor so the SNR gate stays calibrated. */
static float    magIirAlpha         = 0.5f;
static float    smoothMag[BA_SENSOR_COUNT];

/* ── Flight parameters ──────────────────────────────────────────────────── */
static float    navAltTarget      = 1.0f;
static float    navMaxVel         = 0.30f;
static float    navFwdSpeed       = 0.20f;
static float    navSearchFwdSpd   = 0.0f;  /* forward speed during SEARCHING (0 = spin in place) */
static float    navSearchRadius   = 0.8f;  /* max distance from search origin (m) before reversing */
static uint32_t pdTimeoutMs       = 500U;
static float    navDataFreq       = 0.0f;  /* explicit freq for DATA_GATHER mode (0 = use waypoints) */

/* Search-area state: captured on first entry to SEARCHING after each reset. */
static float searchOriginX   = 0.0f;
static float searchOriginY   = 0.0f;
static bool  searchOriginSet = false;

/* ── Default mission ────────────────────────────────────────────────────── */
static const NavWaypoint DEFAULT_MISSION[] = {
    { 150.0f, 1000.0f },
    { 170.0f, 1000.0f },
    // { 150.0f, 1000.0f },
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
static float   logGradR2       = 0.0f;   /* live R² — updated every FFT frame */
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
    memset(smoothMag, 0, sizeof(smoothMag));   /* clear IIR state on freq change */
    searchOriginSet = false;                   /* re-capture search origin on next SEARCHING entry */
    wlsGradientControllerClearMap();
    pdFftAnalyzerResetAccumulator();
    memset(logChSnr, 0, sizeof(logChSnr));
    logMaxSnr = 0.0f; logBearingAngle = 0.0f;
    logGradAngle = 0.0f; logGradMag = 0.0f;
    logMapSize = 0; logWB = 1.0f; logWG = 0.0f;
}

/* ── Navigation state handlers ──────────────────────────────────────────── */

/* No valid bearing — rotate while covering a bounded search area.
 * Captures the position on first entry as the search origin.
 * Moves forward at navSearchFwdSpd until navSearchRadius is reached,
 * then reverses back — combined with yaw rotation this traces a
 * bounded spiral/figure-of-eight pattern.
 * Set navSearchFwdSpd = 0 to spin in place (original behaviour).
 * spYawDeg is advanced in the 100 Hz loop. */
static NavState handleSearching(float snr, bool valid, float posX, float posY)
{
    if (valid && snr > navAcqSnr) {
        searchOriginSet = false;   /* reset so next SEARCHING re-anchors */
        return NAV_ALIGNING;
    }

    if (navSearchFwdSpd > 0.0f) {
        /* Anchor the search area on first entry */
        if (!searchOriginSet) {
            searchOriginX   = posX;
            searchOriginY   = posY;
            searchOriginSet = true;
        }
        /* Move outward until radius is hit, then reverse */
        float dx   = posX - searchOriginX;
        float dy   = posY - searchOriginY;
        float dist = sqrtf(dx * dx + dy * dy);
        spFwdVel = (dist < navSearchRadius) ? navSearchFwdSpd : -navSearchFwdSpd;
    } else {
        spFwdVel = 0.0f;   /* spin-in-place mode */
    }

    return NAV_SEARCHING;
}

/* Signal acquired but not aligned — rotate toward the fused heading.
 * No forward motion until the drone is tracking fusedYaw to within
 * navAlignTol degrees (see handleApproaching).
 * spYawDeg is set from fusedYaw after the switch statement. */
static NavState handleAligning(float headingErr, float snr, bool valid, bool gradReady)
{
    spFwdVel = 0.0f;
    if (!valid || snr < navAcqSnr) {
        /* SNR lost — if the spatial map is still reliable, move toward
         * the gradient direction to escape the occluded region. */
        return gradReady ? NAV_RECOVERING : NAV_SEARCHING;
    }
    if (headingErr < navAlignTol)  return NAV_APPROACHING;
    return NAV_ALIGNING;
}

/* Tracking the fused heading — fly forward.
 * Gate: drone must be within navAlignTol of fusedYaw to move forward.
 * Uses heading tracking error (|fusedYaw − currentYaw|) rather than raw
 * bearing so the gradient component of the fused command does not
 * spuriously trigger a return to ALIGNING. */
static NavState handleApproaching(float headingErr, float snr, bool valid, bool gradReady)
{
    spFwdVel = navFwdSpeed;
    if (!valid || snr < navAcqSnr) {
        /* SNR lost — if the spatial map is still reliable, switch to
         * gradient-only recovery instead of stopping to search. */
        return gradReady ? NAV_RECOVERING : NAV_SEARCHING;
    }
    if (headingErr > navAlignTol * 2.0f)  return NAV_ALIGNING;
    if (snr > navArrSnr) {
        dwellStart = xTaskGetTickCount();
        return NAV_DWELLING;
    }
    return NAV_APPROACHING;
}

/* Gradient-only recovery — bearing/SNR lost but spatial map is reliable.
 * Keeps flying forward on the gradient direction so the drone can exit
 * an occluded region (e.g. behind an obstacle) without stopping to scan.
 * Exits back to APPROACHING when SNR recovers; falls to SEARCHING only
 * when the gradient map itself becomes unreliable. */
static NavState handleRecovering(float headingErr, float snr, bool valid, bool gradReady)
{
    spFwdVel = navFwdSpeed;
    if (!gradReady)                       return NAV_SEARCHING;  /* map gone too */
    if (valid && snr >= navAcqSnr)        return NAV_APPROACHING; /* SNR back */
    if (headingErr > navAlignTol * 2.0f)  return NAV_ALIGNING;   /* re-align to gradient */
    return NAV_RECOVERING;
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

        /* 3. Current yaw from Kalman estimator — sensfusion6 is only updated
         * by the complementary estimator path; when the Kalman estimator is
         * active sensfusion6 is never fed sensor data and returns 0° always.
         * Extract yaw from the Kalman rotation matrix: R is row-major [3][3],
         * so R[1][0] = sin(yaw)*cos(pitch) and R[0][0] = cos(yaw)*cos(pitch),
         * giving yaw = atan2(R[1][0], R[0][0]) for all practical pitch angles. */
        float currentYaw = 0.0f;
        {
            float R[9];
            estimatorKalmanGetEstimatedRot(R);
            currentYaw = atan2f(R[3], R[0]) * 180.0f / (float)M_PI;
        }

        /* 4. Run FFT when a new hop is ready */
        bool newSpectrum = false;
        if (pdFftAnalyzerWindowReady())
            newSpectrum = pdFftAnalyzerRun();

        /* 5. Process new spectrum ─────────────────────────────────────────── */
        if (newSpectrum) {
            /* In DATA_GATHER mode, navDataFreq (set via param) overrides the
             * waypoint table so the Python script can select the frequency
             * to monitor without needing an app-memory write. */
            float targetFreq;
            if (currentMode == MODE_DATA_GATHER && navDataFreq > 0.0f) {
                targetFreq = navDataFreq;
            } else {
                targetFreq = (waypointIndex < waypointCount)
                             ? waypoints[waypointIndex].freq : 0.0f;
            }

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

                    /* Per-channel magnitude IIR: smooths all 8 channels with
                     * the same alpha so relative ratios (= bearing cue) are
                     * preserved while frame-to-frame noise is reduced.
                     * SNR is reconstructed as smoothedMag / noiseFl so the
                     * SNR gate stays correctly calibrated against the current
                     * noise floor rather than the smoothed one. */
                    smoothMag[ch] = magIirAlpha * smoothMag[ch]
                                  + (1.0f - magIirAlpha) * res.magnitude;
                    magnitudes[ch] = smoothMag[ch];
                    float noiseFl  = (res.snr > 0.0f) ? (res.magnitude / res.snr) : 1.0f;
                    float smSnr    = smoothMag[ch] / noiseFl;

                    logChSnr[ch]    = (int16_t)(smSnr * 100.0f);
                    totalMag       += smoothMag[ch];
                    if (smSnr > maxSnrLocal) maxSnrLocal = smSnr;
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
                         * filter starts from the true bearing, not from zero.
                         * Normalize to [-180°, 180°) so fabsf(smoothBearing) is
                         * a true angular error (baOut is [0°, 360°): 270° = right). */
                        smoothBearing      = normalizeAngle(baOut.bearing_deg);
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
                /* Store max SNR across all channels — empirically better than
                 * summed magnitude for map quality (from data-gathering runs). */
                if (currentMode != MODE_MANUAL)
                    wlsGradientControllerAddMapPoint(posX, posY, maxSnrLocal);

                WlsGradientOutput wOutMap;
                bool gradValid = wlsGradientControllerUpdateMap(posX, posY, &wOutMap);
                int  mapSz     = wlsGradientControllerGetMapSize();
                bool gradReady = gradValid
                                 && mapSz >= (int)fusionMinMapPoints
                                 && wOutMap.gradMagnitude >= gradientThreshold;

                logGradAngle = wOutMap.gradAngleDeg;
                logGradMag   = wOutMap.gradMagnitude;
                logGradR2    = wOutMap.r_squared;
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
                    /* Both sources available: weighted circular mean. */
                    float gradAngleWorld = normalizeAngle(wOutMap.gradAngleDeg);
                    wB = fusionWBearing; wG = fusionWGradient;
                    fusedYaw = weightedCircularMean(absBearingYaw, wB,
                                                    gradAngleWorld, wG);
                } else if (!bearingValid && gradReady) {
                    /* Bearing lost (obstacle / out of range) but the spatial
                     * map is reliable — navigate on gradient alone so the
                     * drone can move out of the occluded region. */
                    fusedYaw = normalizeAngle(wOutMap.gradAngleDeg);
                    wB = 0.0f; wG = 1.0f;
                } else if (!bearingValid &&
                           bearingInvalidFrames > bearingHoldFrames) {
                    fusedYaw = currentYaw;   /* nothing reliable — hold heading */
                }
                logCmdYaw = fusedYaw; logWB = wB; logWG = wG;

                /* ── State machine update (FFT-rate) ──────────────────── */
                if (currentMode == MODE_NAVIGATE) {
                    /* Heading tracking error — how far the drone's actual yaw
                     * deviates from the fused command.  Used instead of raw
                     * bearing so the gradient component of fusedYaw does not
                     * cause spurious ALIGNING re-entries. */
                    float headingErr = fabsf(normalizeAngle(fusedYaw - currentYaw));

                    switch (navState) {
                        case NAV_SEARCHING:
                            navState = handleSearching(maxSnrLocal, bearingValid,
                                                       posX, posY);
                            break;
                        case NAV_ALIGNING:
                            navState = handleAligning(headingErr, maxSnrLocal,
                                                      bearingValid, gradReady);
                            break;
                        case NAV_APPROACHING:
                            navState = handleApproaching(headingErr, maxSnrLocal,
                                                         bearingValid, gradReady);
                            break;
                        case NAV_RECOVERING:
                            navState = handleRecovering(headingErr, maxSnrLocal,
                                                        bearingValid, gradReady);
                            break;
                        default:
                            break;
                    }

                    /* All active states use the fused heading */
                    if (navState == NAV_ALIGNING   ||
                        navState == NAV_APPROACHING ||
                        navState == NAV_RECOVERING)
                        spYawDeg = fusedYaw;
                }
            }
        }

        /* 6. 100 Hz navigation updates ─────────────────────────────────── */
        if (currentMode == MODE_NAVIGATE) {

            /* SEARCHING: always rotate to scan.
             * Previously this was gated on !bearingInitialized so that brief
             * SNR dropouts would hold the last heading.  That logic is now
             * handled by NAV_RECOVERING (gradient still reliable → keep moving).
             * Any time we are genuinely in SEARCHING both bearing and gradient
             * are gone, so rotation is always the right action. */
            if (navState == NAV_SEARCHING)
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
    PARAM_ADD(PARAM_FLOAT,               searchSpd,   &navSearchFwdSpd)
    PARAM_ADD(PARAM_FLOAT,               searchR,     &navSearchRadius)
    PARAM_ADD(PARAM_UINT32,              pdTimeout,   &pdTimeoutMs)
    PARAM_ADD(PARAM_FLOAT,               wBearing,    &fusionWBearing)
    PARAM_ADD(PARAM_FLOAT,               wGradient,   &fusionWGradient)
    PARAM_ADD(PARAM_FLOAT,               smoothFact,  &bearingSmoothFactor)
    PARAM_ADD(PARAM_FLOAT,               gradThresh,  &gradientThreshold)
    PARAM_ADD(PARAM_FLOAT,               minLight,    &minTotalLight)
    PARAM_ADD(PARAM_UINT8,               minMapPts,   &fusionMinMapPoints)
    PARAM_ADD(PARAM_UINT8,               bearingHold, &bearingHoldFrames)
    PARAM_ADD(PARAM_FLOAT,               magAlpha,    &magIirAlpha)
    PARAM_ADD(PARAM_FLOAT,               dataFreq,    &navDataFreq)
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
    LOG_ADD(LOG_FLOAT,  gradR2,  &logGradR2)
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
