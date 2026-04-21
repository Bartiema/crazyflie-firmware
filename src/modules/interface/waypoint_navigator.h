/**
 * waypoint_navigator.h
 *
 * Exact port of the Teensy WaypointNavigator (waypoint_navigator.h).
 *
 * Key design difference from Teensy:
 *   - Teensy output: enum FlightCommand (ROTATE_LEFT, MOVE_FORWARD, etc.)
 *     → interpreted by a human or external controller
 *   - CrazyFlie output: direct velocity + yaw-rate setpoints injected into
 *     commanderSetSetpoint() at 100 Hz
 *
 * State machine (identical to Teensy):
 *   IDLE → SEARCHING → ALIGNING → APPROACHING → HOLDING → TRANSITIONING → COMPLETE
 *
 * Waypoints are frequency-based: each waypoint targets a light source
 * flickering at a specific Hz. The navigator uses the bearing computed by
 * BearingAngleController for that frequency to drive the drone toward it.
 *
 * Flow Deck v2 note:
 *   This navigator does NOT use absolute position. It only uses the bearing
 *   estimate and SNR from the photodiodes. The Flow Deck provides velocity
 *   stabilisation inside the CrazyFlie stabiliser — the navigator just sets
 *   velocity setpoints and the stabiliser handles the rest.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define WP_NAV_MAX_WAYPOINTS   10
#define WP_NAV_MAX_FREQUENCIES 10

/** Single waypoint: a frequency to seek and a time to dwell there. */
typedef struct {
    float    frequency_hz;   /**< Target light modulation frequency (Hz)        */
    float    dwell_ms;       /**< Time to hold at waypoint after arrival (ms)   */
    bool     reached;        /**< Set true once this waypoint is visited        */
} WpNavWaypoint;

/** Per-frequency sensor data supplied each update tick. */
typedef struct {
    float frequency_hz;   /**< Which frequency this reading corresponds to */
    float bearing_deg;    /**< Estimated bearing, 0–360° (0 = drone forward) */
    float max_snr;        /**< Maximum SNR across all 8 sensors for this freq */
    bool  valid;          /**< False if bearing could not be computed          */
} WpNavFreqReading;

/** Navigation state (identical enum to Teensy NavState). */
typedef enum {
    WP_NAV_IDLE         = 0,
    WP_NAV_SEARCHING    = 1,
    WP_NAV_ALIGNING     = 2,
    WP_NAV_APPROACHING  = 3,
    WP_NAV_HOLDING      = 4,
    WP_NAV_TRANSITIONING= 5,
    WP_NAV_COMPLETE     = 6,
} WpNavState;

/** Output setpoint for injection into commanderSetSetpoint(). */
typedef struct {
    float vx;             /**< Body-frame forward velocity (m/s)          */
    float vy;             /**< Body-frame left velocity (m/s) — unused     */
    float yaw_rate_deg;   /**< Yaw rate (deg/s), positive = CCW from above */
    WpNavState state;     /**< Current state machine state                 */
} WpNavSetpoint;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */
void waypointNavigatorInit(void);
bool waypointNavigatorTest(void);

/* ── Mission configuration (call before waypointNavigatorStartMission) ─── */
bool waypointNavigatorAddWaypoint(float frequency_hz, float dwell_ms);
void waypointNavigatorStartMission(void);
void waypointNavigatorResetMission(void);

/* ── Per-tick update — call at 100 Hz from mode_manager ────────────────── */
WpNavSetpoint waypointNavigatorUpdate(const WpNavFreqReading *readings,
                                       int                     num_readings);

/* ── State queries ─────────────────────────────────────────────────────── */
WpNavState waypointNavigatorGetState(void);
int        waypointNavigatorGetCurrentIndex(void);
int        waypointNavigatorGetWaypointCount(void);
bool       waypointNavigatorIsMissionComplete(void);

/** Get the list of unique frequencies needed (to configure FFT analysis). */
int    waypointNavigatorGetNumUniqueFreqs(void);
float  waypointNavigatorGetUniqueFreq(int index);

/** Build the unique-frequency table from added waypoints without starting
 *  the mission. Call this after AddWaypoint() so GetNumUniqueFreqs() is
 *  valid before the first StartMission(). */
void   waypointNavigatorBuildFreqTable(void);
