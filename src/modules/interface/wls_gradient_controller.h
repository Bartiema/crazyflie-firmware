/**
 * wls_gradient_controller.h
 *
 * Weighted Least Squares spatial gradient estimator.
 *
 * Uses exact sensor X/Y positions from PCB layout (no radius approximation needed).
 * Sensor positions hard-coded from photodiode-expansion.kicad_pcb measurements.
 *
 * Two modes of operation:
 *   1. INSTANTANEOUS: single-frame gradient from 8 simultaneous sensor readings
 *      (same as the sensor-ring WLS used in simulation for the 8-sensor case)
 *   2. MAP-BASED: builds a spatial measurement map from flight history,
 *      fits a plane to nearby map points using inverse-distance² weights.
 *      This is the full blimp.cpp gradient estimator ported to firmware.
 *
 * The map-based mode gives better gradient accuracy when the drone has
 * covered enough spatial area. The mode_manager selects which to use
 * based on map size and gradient magnitude thresholds.
 */

#pragma once

#include <stdbool.h>

#define WLS_SENSOR_COUNT  8

typedef struct {
    float pd[WLS_SENSOR_COUNT];  /**< FFT magnitudes at target freq [0..∞) */
} WlsGradientInput;

typedef struct {
    float vx;              /**< Gradient ascent velocity, body X (m/s)     */
    float vy;              /**< Gradient ascent velocity, body Y (m/s)     */
    float gradX;           /**< Estimated ∂I/∂x — body frame               */
    float gradY;           /**< Estimated ∂I/∂y — body frame               */
    float gradMagnitude;   /**< |∇I|                                        */
    float gradAngleDeg;    /**< Direction of steepest ascent (0–360°)      */
    float r_squared;       /**< Weighted R² of the plane fit (map mode)    */
    bool  mapReady;        /**< True once map has MIN_MAP_POINTS entries    */
} WlsGradientOutput;

void wlsGradientControllerInit(void);
bool wlsGradientControllerTest(void);

/**
 * wlsGradientControllerUpdateInstantaneous()
 * Single-frame gradient estimate from the 8 sensor ring.
 * Available immediately — no spatial history needed.
 */
void wlsGradientControllerUpdateInstantaneous(const WlsGradientInput *in,
                                               WlsGradientOutput      *out);

/**
 * wlsGradientControllerAddMapPoint()
 * Record a new (x, y, intensity) measurement in the spatial map.
 * Call whenever the drone's position changes meaningfully.
 * x, y are in metres relative to takeoff point (Flow Deck odometry).
 * intensity = total_light or max FFT magnitude across all sensors.
 */
void wlsGradientControllerAddMapPoint(float x, float y, float intensity);

/**
 * wlsGradientControllerUpdateMap()
 * Estimate gradient from the accumulated spatial measurement map.
 * Uses inverse-distance² weighting around (cx, cy) — port of blimp.cpp
 * estimate_gradient() function.
 * Returns false (and zeros out) if map is too small or R² is too low.
 */
bool wlsGradientControllerUpdateMap(float cx, float cy,
                                     WlsGradientOutput *out);

/** Reset the spatial measurement map. */
void wlsGradientControllerClearMap(void);

/** Get number of map points accumulated so far. */
int wlsGradientControllerGetMapSize(void);
