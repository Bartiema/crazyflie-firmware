/**
 * mode_manager.h
 *
 * Runtime operational mode switch and main orchestration task.
 *
 * Modes:
 *   MODE_MANUAL      (0) — all control from cflib, no custom modules active
 *   MODE_DATA_GATHER (1) — manual flight + full FFT + bearing + WLS logged
 *   MODE_NAVIGATE    (2) — waypoint navigator active (BearingAngle-driven)
 *                          TODO: fused with WLS gradient once simulation
 *                                fusion code is integrated
 *
 * Architecture of the 100 Hz task:
 *
 *   pdDeckGetValues()
 *        │
 *        ▼
 *   pdFftAnalyzerPushSample()           ← every tick (200 Hz PD, 100 Hz task)
 *        │   (every PD_FFT_SIZE ticks)
 *        ▼
 *   pdFftAnalyzerRun()                  ← runs FFT on all 8 channels
 *        │
 *        ├──▶ bearingAngleControllerUpdate()  ← per unique frequency
 *        │
 *        ├──▶ wlsGradientControllerUpdate()   ← DATA_GATHER only (or fused)
 *        │
 *        └──▶ waypointNavigatorUpdate()        ← NAVIGATE mode
 *                  │
 *                  ▼
 *             commanderSetSetpoint()
 */

#pragma once

#include <stdint.h>

typedef enum {
    MODE_MANUAL      = 0,
    MODE_DATA_GATHER = 1,
    MODE_NAVIGATE    = 2,
} DroneMode;

void      modeManagerInit(void);
DroneMode modeManagerGetMode(void);
