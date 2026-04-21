/**
 * waypoint_navigator.c
 *
 * Exact port of Teensy WaypointNavigator (waypoint_navigator.h).
 *
 * State machine transitions (identical to Teensy):
 *
 *   IDLE ──start()──▶ SEARCHING
 *                          │ SNR > acq_threshold
 *                          ▼
 *                      ALIGNING ◀────────────── lost signal
 *                          │ |bearing| < alignment_tol
 *                          ▼
 *                      APPROACHING ◀──────────── misaligned (2× tol)
 *                          │ SNR > approach_threshold
 *                          ▼
 *                       HOLDING
 *                          │ dwell_ms elapsed
 *                          ▼
 *                    TRANSITIONING ──more WPs──▶ SEARCHING
 *                          │ last WP
 *                          ▼
 *                       COMPLETE
 *
 * CrazyFlie setpoint mapping (replacing Teensy FlightCommand enum):
 *   CMD_ROTATE_RIGHT    → yaw_rate = +navYawRate  (positive = CW from above)
 *   CMD_ROTATE_LEFT     → yaw_rate = -navYawRate
 *   CMD_MOVE_FORWARD    → vx = +navFwdVel
 *   CMD_HOLD_POSITION   → vx = 0, yaw_rate = 0
 *   CMD_MISSION_COMPLETE→ vx = 0, yaw_rate = 0
 *
 * Bearing convention (matches Teensy handle_aligning):
 *   bearing = 0°   → source is directly ahead → aligned
 *   bearing > 0°   → source is to the left    → rotate CCW (CMD_ROTATE_RIGHT
 *                                                in Teensy = positive yaw here)
 *   bearing < 0°   → source is to the right   → rotate CW
 *
 * NOTE on angular_distance():
 *   Identical to Teensy: returns signed shortest path from angle1 to angle2.
 *   alignment_error = angular_distance(0°, bearing°)
 *   Positive error  → bearing is CCW from forward → rotate CCW to align
 */

#define DEBUG_MODULE "WPNAV"

#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "debug.h"

#include "waypoint_navigator.h"

/* Forward declaration — handleHolding calls handleTransitioning directly */
static WpNavSetpoint handleTransitioning(void);

/* ──────────────────────────────────────────────────────────────────────────
 * Tunable parameters — match Teensy defaults, adjustable from Python
 * ────────────────────────────────────────────────────────────────────────── */

/** ±degrees within which the drone is considered aligned with the target */
static float navAlignTol        = 15.0f;   /* Teensy default */

/** Minimum SNR to trust the bearing (acquisition gate) */
static float navAcqSnrThresh    =  5.0f;   /* Teensy default */

/** SNR threshold used to declare "arrived at waypoint" */
static float navApproachSnrThresh = 10.0f; /* Teensy default */

/** Forward velocity when approaching (m/s) */
static float navFwdVel          = 0.20f;

/** Yaw rate when searching or aligning (deg/s) */
static float navYawRate         = 30.0f;

/* ──────────────────────────────────────────────────────────────────────────
 * Mission state
 * ────────────────────────────────────────────────────────────────────────── */

static WpNavWaypoint  waypoints[WP_NAV_MAX_WAYPOINTS];
static int            waypointCount       = 0;
static int            currentIndex        = 0;
static bool           missionActive       = false;
static WpNavState     navState            = WP_NAV_IDLE;
static uint32_t       holdStartTick       = 0;

/* Unique frequency registry (mirrors Teensy extract_unique_frequencies) */
static float  uniqueFreqs[WP_NAV_MAX_FREQUENCIES];
static int    numUniqueFreqs = 0;

/* Latest setpoint (for LOG) */
static WpNavSetpoint latestSp;

/* ──────────────────────────────────────────────────────────────────────────
 * Helpers
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * angular_distance() — identical to Teensy implementation.
 * Returns signed shortest angular distance from angle1 to angle2 in degrees.
 * Result is in (-180, +180].
 */
static float angularDistance(float angle1, float angle2)
{
    float diff = angle2 - angle1;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/**
 * find_reading() — find the reading for a given frequency in the input array.
 * Returns NULL if not found. Mirrors Teensy find_frequency_index().
 */
static const WpNavFreqReading *findReading(const WpNavFreqReading *readings,
                                            int num, float freq_hz)
{
    for (int i = 0; i < num; i++) {
        if (fabsf(readings[i].frequency_hz - freq_hz) < 0.5f) {
            return &readings[i];
        }
    }
    return NULL;
}

/** extract_unique_frequencies — mirror of Teensy method */
static void extractUniqueFrequencies(void)
{
    numUniqueFreqs = 0;
    for (int i = 0; i < waypointCount; i++) {
        float f = waypoints[i].frequency_hz;
        bool found = false;
        for (int j = 0; j < numUniqueFreqs; j++) {
            if (fabsf(uniqueFreqs[j] - f) < 0.5f) { found = true; break; }
        }
        if (!found && numUniqueFreqs < WP_NAV_MAX_FREQUENCIES) {
            uniqueFreqs[numUniqueFreqs++] = f;
        }
    }
    DEBUG_PRINT("WPNAV: %d unique frequencies registered\n", numUniqueFreqs);
}

/* ──────────────────────────────────────────────────────────────────────────
 * State handlers — each returns a WpNavSetpoint
 * Direct port of Teensy handle_*() methods
 * ────────────────────────────────────────────────────────────────────────── */

static WpNavSetpoint makeSetpoint(float vx, float yaw, WpNavState st)
{
    WpNavSetpoint sp = { .vx = vx, .vy = 0.0f, .yaw_rate_deg = yaw, .state = st };
    return sp;
}

static WpNavSetpoint handleSearching(float bearing, float snr, bool valid)
{
    /* Teensy handle_searching: rotate right until SNR > acquisition_threshold */
    if (valid && snr > navAcqSnrThresh) {
        DEBUG_PRINT("WPNAV: target acquired (SNR=%.1f), aligning\n", (double)snr);
        navState = WP_NAV_ALIGNING;
        /* Fall through into aligning immediately (mirrors Teensy) */
        float err = angularDistance(0.0f, bearing);
        if (fabsf(err) < navAlignTol) {
            navState = WP_NAV_APPROACHING;
            return makeSetpoint(navFwdVel, 0.0f, WP_NAV_APPROACHING);
        }
        return makeSetpoint(0.0f, (err > 0.0f ? navYawRate : -navYawRate), WP_NAV_ALIGNING);
    }
    /* Rotate right (positive yaw = CCW in CrazyFlie convention) */
    return makeSetpoint(0.0f, navYawRate, WP_NAV_SEARCHING);
}

static WpNavSetpoint handleAligning(float bearing, float snr, bool valid)
{
    /* Teensy handle_aligning */
    if (!valid || snr < navAcqSnrThresh) {
        DEBUG_PRINT("WPNAV: lost signal, searching\n");
        navState = WP_NAV_SEARCHING;
        return makeSetpoint(0.0f, navYawRate, WP_NAV_SEARCHING);
    }

    /* alignment_error = angular_distance(0°, bearing) */
    float err = angularDistance(0.0f, bearing);

    if (fabsf(err) < navAlignTol) {
        DEBUG_PRINT("WPNAV: aligned (err=%.1f°), approaching\n", (double)err);
        navState = WP_NAV_APPROACHING;
        return makeSetpoint(navFwdVel, 0.0f, WP_NAV_APPROACHING);
    }

    /* Rotate toward target — positive error → rotate CCW (+yaw) */
    return makeSetpoint(0.0f, (err > 0.0f ? navYawRate : -navYawRate), WP_NAV_ALIGNING);
}

static WpNavSetpoint handleApproaching(float bearing, float snr, bool valid)
{
    /* Teensy handle_approaching */
    if (!valid || snr < navAcqSnrThresh) {
        DEBUG_PRINT("WPNAV: lost signal during approach, searching\n");
        navState = WP_NAV_SEARCHING;
        return makeSetpoint(0.0f, navYawRate, WP_NAV_SEARCHING);
    }

    float err = angularDistance(0.0f, bearing);

    /* Realign if misaligned beyond 2× tolerance */
    if (fabsf(err) > navAlignTol * 2.0f) {
        DEBUG_PRINT("WPNAV: misaligned (err=%.1f°), realigning\n", (double)err);
        navState = WP_NAV_ALIGNING;
        return makeSetpoint(0.0f, (err > 0.0f ? navYawRate : -navYawRate), WP_NAV_ALIGNING);
    }

    /* "Arrived" when SNR exceeds approach threshold */
    if (snr > navApproachSnrThresh) {
        DEBUG_PRINT("WPNAV: waypoint %d reached (SNR=%.1f)\n",
                    currentIndex, (double)snr);
        waypoints[currentIndex].reached = true;
        navState      = WP_NAV_HOLDING;
        holdStartTick = xTaskGetTickCount();
        return makeSetpoint(0.0f, 0.0f, WP_NAV_HOLDING);
    }

    return makeSetpoint(navFwdVel, 0.0f, WP_NAV_APPROACHING);
}

static WpNavSetpoint handleHolding(void)
{
    /* Teensy handle_holding */
    uint32_t elapsed = (xTaskGetTickCount() - holdStartTick) * portTICK_RATE_MS;

    if (elapsed >= (uint32_t)waypoints[currentIndex].dwell_ms) {
        DEBUG_PRINT("WPNAV: dwell complete, transitioning\n");
        navState = WP_NAV_TRANSITIONING;
        return handleTransitioning();   /* mirrors Teensy immediate call-through */
    }
    return makeSetpoint(0.0f, 0.0f, WP_NAV_HOLDING);
}

static WpNavSetpoint handleTransitioning(void)
{
    /* Teensy handle_transitioning */
    currentIndex++;
    if (currentIndex >= waypointCount) {
        DEBUG_PRINT("WPNAV: mission complete!\n");
        navState      = WP_NAV_COMPLETE;
        missionActive = false;
        return makeSetpoint(0.0f, 0.0f, WP_NAV_COMPLETE);
    }
    DEBUG_PRINT("WPNAV: moving to waypoint %d (%.0f Hz)\n",
                currentIndex, (double)waypoints[currentIndex].frequency_hz);
    navState = WP_NAV_SEARCHING;
    return makeSetpoint(0.0f, navYawRate, WP_NAV_SEARCHING);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────────────────────────── */

void waypointNavigatorInit(void)
{
    memset(waypoints,  0, sizeof(waypoints));
    memset(uniqueFreqs, 0, sizeof(uniqueFreqs));
    waypointCount  = 0;
    currentIndex   = 0;
    missionActive  = false;
    navState       = WP_NAV_IDLE;
    numUniqueFreqs = 0;
    memset(&latestSp, 0, sizeof(latestSp));
    DEBUG_PRINT("WPNAV: initialised\n");
}

bool waypointNavigatorTest(void) { return true; }

bool waypointNavigatorAddWaypoint(float frequency_hz, float dwell_ms)
{
    if (waypointCount >= WP_NAV_MAX_WAYPOINTS) {
        DEBUG_PRINT("WPNAV: waypoint list full\n");
        return false;
    }
    waypoints[waypointCount].frequency_hz = frequency_hz;
    waypoints[waypointCount].dwell_ms     = dwell_ms;
    waypoints[waypointCount].reached      = false;
    waypointCount++;
    DEBUG_PRINT("WPNAV: added WP %d @ %.0f Hz, dwell %.0f ms\n",
                waypointCount, (double)frequency_hz, (double)dwell_ms);
    return true;
}

void waypointNavigatorStartMission(void)
{
    if (waypointCount == 0) {
        DEBUG_PRINT("WPNAV: no waypoints — cannot start\n");
        return;
    }
    extractUniqueFrequencies();
    for (int i = 0; i < waypointCount; i++) waypoints[i].reached = false;
    currentIndex  = 0;
    missionActive = true;
    navState      = WP_NAV_SEARCHING;
    DEBUG_PRINT("WPNAV: mission started (%d WPs, %d freqs)\n",
                waypointCount, numUniqueFreqs);
}

void waypointNavigatorResetMission(void)
{
    currentIndex  = 0;
    missionActive = false;
    navState      = WP_NAV_IDLE;
    for (int i = 0; i < waypointCount; i++) waypoints[i].reached = false;
    DEBUG_PRINT("WPNAV: reset\n");
}

WpNavSetpoint waypointNavigatorUpdate(const WpNavFreqReading *readings,
                                       int                     num_readings)
{
    WpNavSetpoint idle = makeSetpoint(0.0f, 0.0f, navState);

    if (!missionActive || navState == WP_NAV_COMPLETE || navState == WP_NAV_IDLE) {
        return idle;
    }

    /* Find the reading for the current waypoint's frequency */
    const WpNavFreqReading *r = findReading(readings, num_readings,
                                             waypoints[currentIndex].frequency_hz);
    if (!r) {
        DEBUG_PRINT("WPNAV: no reading for %.0f Hz\n",
                    (double)waypoints[currentIndex].frequency_hz);
        return idle;
    }

    WpNavSetpoint sp;
    switch (navState) {
        case WP_NAV_SEARCHING:    sp = handleSearching(r->bearing_deg, r->max_snr, r->valid); break;
        case WP_NAV_ALIGNING:     sp = handleAligning (r->bearing_deg, r->max_snr, r->valid); break;
        case WP_NAV_APPROACHING:  sp = handleApproaching(r->bearing_deg, r->max_snr, r->valid); break;
        case WP_NAV_HOLDING:      sp = handleHolding(); break;
        case WP_NAV_TRANSITIONING:sp = handleTransitioning(); break;
        default:                  sp = idle; break;
    }

    latestSp = sp;
    return sp;
}

WpNavState waypointNavigatorGetState(void)         { return navState; }
int        waypointNavigatorGetCurrentIndex(void)  { return currentIndex; }
int        waypointNavigatorGetWaypointCount(void) { return waypointCount; }
bool       waypointNavigatorIsMissionComplete(void){ return navState == WP_NAV_COMPLETE; }
int        waypointNavigatorGetNumUniqueFreqs(void){ return numUniqueFreqs; }
float      waypointNavigatorGetUniqueFreq(int i)   { return (i < numUniqueFreqs) ? uniqueFreqs[i] : 0.0f; }
void       waypointNavigatorBuildFreqTable(void)   { extractUniqueFrequencies(); }

/* ──────────────────────────────────────────────────────────────────────────
 * PARAM
 * ────────────────────────────────────────────────────────────────────────── */

static uint8_t navResetFlag = 0;
static void navResetCallback(void) {
    if (navResetFlag) { waypointNavigatorResetMission(); navResetFlag = 0; }
}

PARAM_GROUP_START(wpNav)
    PARAM_ADD(PARAM_FLOAT, alignTol,    &navAlignTol)
    PARAM_ADD(PARAM_FLOAT, acqSnr,      &navAcqSnrThresh)
    PARAM_ADD(PARAM_FLOAT, arrSnr,      &navApproachSnrThresh)
    PARAM_ADD(PARAM_FLOAT, fwdVel,      &navFwdVel)
    PARAM_ADD(PARAM_FLOAT, yawRate,     &navYawRate)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, reset, &navResetFlag, navResetCallback)
PARAM_GROUP_STOP(wpNav)

/* ──────────────────────────────────────────────────────────────────────────
 * LOG
 * ────────────────────────────────────────────────────────────────────────── */

LOG_GROUP_START(wpNav)
    LOG_ADD(LOG_UINT8,  state,   &navState)
    LOG_ADD(LOG_UINT8,  wpIdx,   &currentIndex)
    LOG_ADD(LOG_FLOAT,  vx,      &latestSp.vx)
    LOG_ADD(LOG_FLOAT,  yawRate, &latestSp.yaw_rate_deg)
LOG_GROUP_STOP(wpNav)
