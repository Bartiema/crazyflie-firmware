/**
 * bearing_angle_controller.h
 *
 * BearingAngle gradient controller — frequency-selective light source tracker.
 *
 * Exact port of the Teensy AngleCalculator::calculate_weighted_angle_magnitude()
 * combined with the WaypointNavigator state machine.
 *
 * Algorithm:
 *   1. For each sensor i, extract FFT magnitude at the target frequency
 *   2. Apply optional per-sensor magnitude correction (calibration)
 *   3. Compute weighted circular mean:
 *        sum_x = Σ mag[i] * cos(θ[i])
 *        sum_y = Σ mag[i] * sin(θ[i])
 *        bearing = atan2(sum_y, sum_x)  →  degrees, normalised 0–360°
 *   4. SNR check gates whether the bearing is trusted
 */

#pragma once

#include <stdbool.h>

#define BA_SENSOR_COUNT  8

typedef struct {
    float magnitude[BA_SENSOR_COUNT]; /**< FFT magnitude at target freq, per channel */
    float snr[BA_SENSOR_COUNT];       /**< SNR per channel (from pd_fft_analyzer)    */
    float max_snr;                    /**< Max SNR across all channels               */
} BearingAngleInput;

typedef struct {
    float bearing_deg;   /**< Estimated bearing to source, 0–360° (0=forward)  */
    float bearing_rad;   /**< Same, in radians                                  */
    float total_weight;  /**< Sum of magnitudes used (confidence proxy)         */
    float vx;            /**< Body-frame velocity command (m/s) — forward       */
    float vy;            /**< Body-frame velocity command (m/s) — left          */
    float yaw_rate_deg;  /**< Yaw rate command (deg/s), positive = CCW          */
    bool  valid;         /**< False if total_weight is zero                     */
} BearingAngleOutput;

void bearingAngleControllerInit(void);
bool bearingAngleControllerTest(void);
void bearingAngleControllerUpdate(const BearingAngleInput  *in,
                                   BearingAngleOutput       *out);

/** Apply calibration correction factors to a magnitude array in-place. */
void bearingAngleApplyCalibration(float magnitudes[BA_SENSOR_COUNT]);

/** Reset calibration (set all correction factors to 1.0). */
void bearingAngleResetCalibration(void);
