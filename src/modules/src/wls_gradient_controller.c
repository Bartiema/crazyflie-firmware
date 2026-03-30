/**
 * wls_gradient_controller.c
 *
 * Two gradient estimation strategies, both ported from simulation sources:
 *
 * 1. INSTANTANEOUS (wlsGradientControllerUpdateInstantaneous)
 *    Fits a plane to the 8 simultaneous sensor readings at their known
 *    PCB positions. Available every FFT frame. Accurate when the light
 *    gradient varies spatially on the scale of the sensor ring (~37 mm
 *    diameter), which may be small relative to the light field.
 *
 * 2. MAP-BASED (wlsGradientControllerUpdateMap)
 *    Direct port of blimp.cpp estimate_gradient(). Accumulates (x,y,intensity)
 *    measurements as the drone flies. Fits a weighted plane to nearby points
 *    using inverse-distance² weights. Requires MIN_MAP_POINTS entries and
 *    weighted R² ≥ GRADIENT_R2_THRESHOLD before it is trusted.
 *    This is the primary gradient source for the fused controller.
 *
 * Sensor positions (exact, from photodiode-expansion.kicad_pcb):
 *   ch | drone_angle | px (m)     | py (m)
 *   ---+-------------+------------+------------
 *    0 |  232.7°     | -0.011383  | -0.014943
 *    1 |  277.8°     | +0.002509  | -0.018318
 *    2 |  142.7°     | -0.014828  | +0.011296
 *    3 |  187.7°     | -0.018502  | -0.002502
 *    4 |   53.6°     | +0.011010  | +0.014933
 *    5 |   97.8°     | -0.002509  | +0.018318
 *    6 |    7.7°     | +0.018502  | +0.002502
 *    7 |  322.7°     | +0.014828  | -0.011296
 *
 * Body frame: +X = forward, +Y = left. Radius ≈ 18.62 mm (PCB measured).
 */

#define DEBUG_MODULE "WLSCTRL"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "log.h"
#include "param.h"
#include "debug.h"
#include "static_mem.h"

#include "wls_gradient_controller.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Exact sensor positions from PCB file (metres, drone body frame)
 * ────────────────────────────────────────────────────────────────────────── */
static const float SENSOR_PX[WLS_SENSOR_COUNT] = {
    -0.011383f,  /* ch0 – J1 */
    +0.002509f,  /* ch1 – J2 */
    -0.014828f,  /* ch2 – J3 */
    -0.018502f,  /* ch3 – J4 */
    +0.011010f,  /* ch4 – J5 */
    -0.002509f,  /* ch5 – J6 */
    +0.018502f,  /* ch6 – J7 */
    +0.014828f,  /* ch7 – J8 */
};

static const float SENSOR_PY[WLS_SENSOR_COUNT] = {
    -0.014943f,  /* ch0 – J1 */
    -0.018318f,  /* ch1 – J2 */
    +0.011296f,  /* ch2 – J3 */
    -0.002502f,  /* ch3 – J4 */
    +0.014933f,  /* ch4 – J5 */
    +0.018318f,  /* ch5 – J6 */
    +0.002502f,  /* ch6 – J7 */
    -0.011296f,  /* ch7 – J8 */
};

/* ──────────────────────────────────────────────────────────────────────────
 * Tunable parameters
 * ────────────────────────────────────────────────────────────────────────── */

/** Step size for instantaneous gradient ascent (m/s) */
static float wlsStepSize      = 0.20f;

/** Instantaneous gradient magnitude below this → output zero velocity */
static float wlsMinMag        = 0.005f;

/** Weight exponent for instantaneous WLS: w[i] = pd[i]^wlsWeightExp */
static float wlsWeightExp     = 2.0f;

/** Map-based: minimum number of map points before gradient is trusted */
static uint8_t mapMinPoints   = 3;

/** Map-based: minimum weighted R² to accept the gradient estimate.
 *  Mirrors blimp.cpp GRADIENT_THRESHOLD used as R² gate. */
static float mapR2Threshold   = 0.50f;

/** Map-based: max distance (m) of a point from query position to include */
static float mapMaxDist       = 3.0f;

/** Map-based: min distance (m) — avoid self-weight at zero distance */
static float mapMinDist       = 0.05f;

/** Map-based: epsilon in weight denominator 1/(d²+eps), avoids 1/0 */
static float mapWEps          = 0.01f;

/** Map-based: only update a cell if new intensity exceeds old by this */
static float mapMinImprovement = 0.05f;

/** Map grid resolution in metres — one cell per GRID_RES × GRID_RES */
static float mapGridRes       = 0.20f;

/* ──────────────────────────────────────────────────────────────────────────
 * Spatial measurement map
 * Port of blimp.cpp measurement_map (std::map<GridCell,MeasurementPoint>)
 * Implemented as a fixed-size flat array (no heap allocation on embedded).
 * ────────────────────────────────────────────────────────────────────────── */

#define MAP_MAX_POINTS  256   /* max cells; 256 × 12 bytes = 3 KB */

typedef struct {
    float x, y;          /* world-frame position (m from takeoff) */
    float intensity;     /* light intensity (FFT magnitude or total) */
    int16_t gx, gy;      /* grid cell indices */
    bool  occupied;
} MapCell;

static MapCell mapCells[MAP_MAX_POINTS];
static int     mapSize = 0;

static StaticSemaphore_t mapMutexBuf;
static SemaphoreHandle_t mapMutex;

/* ── Latest outputs for LOG ──────────────────────────────────────────────── */
static WlsGradientOutput latestInstant;
static WlsGradientOutput latestMap;

/* ──────────────────────────────────────────────────────────────────────────
 * Helpers
 * ────────────────────────────────────────────────────────────────────────── */

/** Analytical 3×3 inverse using Cramer's rule. Port of blimp.cpp solve. */
static bool solve3x3(const float M[3][3], const float R[3], float sol[3])
{
    float det = M[0][0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
              - M[0][1]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
              + M[0][2]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);

    if (fabsf(det) < 1e-6f) return false;

    /* sol[0] = a (∂I/∂x), sol[1] = b (∂I/∂y), sol[2] = c (intercept) */
    float det_a = R[0]*(M[1][1]*M[2][2] - M[1][2]*M[2][1])
                - M[0][1]*(R[1]*M[2][2] - M[1][2]*R[2])
                + M[0][2]*(R[1]*M[2][1] - M[1][1]*R[2]);

    float det_b = M[0][0]*(R[1]*M[2][2] - M[1][2]*R[2])
                - R[0]*(M[1][0]*M[2][2] - M[1][2]*M[2][0])
                + M[0][2]*(M[1][0]*R[2] - R[1]*M[2][0]);

    float det_c = M[0][0]*(M[1][1]*R[2] - R[1]*M[2][1])
                - M[0][1]*(M[1][0]*R[2] - R[1]*M[2][0])
                + R[0]*(M[1][0]*M[2][1] - M[1][1]*M[2][0]);

    sol[0] = det_a / det;
    sol[1] = det_b / det;
    sol[2] = det_c / det;
    return true;
}

static int16_t toGrid(float v) { return (int16_t)floorf(v / mapGridRes); }

/* ──────────────────────────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────────────────────────── */

void wlsGradientControllerInit(void)
{
    mapMutex = xSemaphoreCreateMutexStatic(&mapMutexBuf);
    ASSERT(mapMutex != NULL);
    memset(mapCells,   0, sizeof(mapCells));
    memset(&latestInstant, 0, sizeof(latestInstant));
    memset(&latestMap,     0, sizeof(latestMap));
    mapSize = 0;
    DEBUG_PRINT("WLS gradient controller init OK\n");
    DEBUG_PRINT("  Sensor ring radius: ~18.62 mm (PCB measured)\n");
    DEBUG_PRINT("  Map capacity: %d cells @ %.2f m resolution\n",
                MAP_MAX_POINTS, (double)mapGridRes);
}

bool wlsGradientControllerTest(void)
{
    /* Instantaneous test: magnitudes proportional to cos(angle) → gradient +X */
    WlsGradientInput testIn;
    for (int i = 0; i < WLS_SENSOR_COUNT; i++) {
        float ang   = atan2f(SENSOR_PY[i], SENSOR_PX[i]);
        testIn.pd[i] = 0.5f + 0.5f * cosf(ang);
    }
    WlsGradientOutput testOut;
    wlsGradientControllerUpdateInstantaneous(&testIn, &testOut);

    if (testOut.gradMagnitude < wlsMinMag) {
        DEBUG_PRINT("WLS self-test FAIL: gradient too small\n");
        return false;
    }
    float err = fabsf(testOut.gradAngleDeg);
    if (err > 15.0f && err < 345.0f) {
        DEBUG_PRINT("WLS self-test FAIL: angle=%.1f° (expected ~0°)\n",
                    (double)testOut.gradAngleDeg);
        return false;
    }
    DEBUG_PRINT("WLS self-test PASS (angle=%.1f°, mag=%.4f)\n",
                (double)testOut.gradAngleDeg, (double)testOut.gradMagnitude);
    return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Instantaneous gradient from sensor ring
 * Model: intensity[i] ≈ β0 + β1·px[i] + β2·py[i]
 * Solved via WLS with w[i] = pd[i]^wlsWeightExp.
 * ────────────────────────────────────────────────────────────────────────── */
void wlsGradientControllerUpdateInstantaneous(const WlsGradientInput *in,
                                               WlsGradientOutput      *out)
{
    /* Weights */
    float w[WLS_SENSOR_COUNT];
    for (int i = 0; i < WLS_SENSOR_COUNT; i++) {
        w[i] = powf(fmaxf(in->pd[i], 0.0f), wlsWeightExp);
    }

    /* Build XᵀWX (3×3, symmetric) and XᵀWy (3×1)
     * X columns = [px, py, 1]  (same layout as blimp.cpp M matrix) */
    float sw=0, swx=0, swy=0, swz=0;
    float swxx=0, swyy=0, swxy=0, swxz=0, swyz=0;

    for (int i = 0; i < WLS_SENSOR_COUNT; i++) {
        float wi = w[i];
        float x  = SENSOR_PX[i], y = SENSOR_PY[i], z = in->pd[i];
        sw   += wi;
        swx  += wi*x;    swy  += wi*y;    swz  += wi*z;
        swxx += wi*x*x;  swyy += wi*y*y;  swxy += wi*x*y;
        swxz += wi*x*z;  swyz += wi*y*z;
    }

    /* Normal matrix layout matches blimp.cpp exactly:
     * M = [[swxx, swxy, swx],
     *      [swxy, swyy, swy],
     *      [swx,  swy,  sw ]]
     * R = [swxz, swyz, swz] */
    float M[3][3] = {
        {swxx, swxy, swx},
        {swxy, swyy, swy},
        {swx,  swy,  sw }
    };
    float R[3] = {swxz, swyz, swz};
    float sol[3];

    if (!solve3x3(M, R, sol)) {
        out->vx = out->vy = out->gradX = out->gradY = out->gradMagnitude = 0.0f;
        out->gradAngleDeg = 0.0f;
        out->r_squared    = 0.0f;
        out->mapReady     = false;
        latestInstant     = *out;
        return;
    }

    float gx  = sol[0];   /* ∂I/∂x */
    float gy  = sol[1];   /* ∂I/∂y */
    float mag = sqrtf(gx*gx + gy*gy);

    out->gradX         = gx;
    out->gradY         = gy;
    out->gradMagnitude = mag;
    out->r_squared     = 0.0f;   /* not computed for instantaneous mode */
    out->mapReady      = false;

    float ang = atan2f(gy, gx) * 180.0f / (float)M_PI;
    if (ang < 0.0f) ang += 360.0f;
    out->gradAngleDeg = ang;

    if (mag < wlsMinMag) {
        out->vx = out->vy = 0.0f;
    } else {
        out->vx = wlsStepSize * (gx / mag);
        out->vy = wlsStepSize * (gy / mag);
    }

    latestInstant = *out;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Spatial map management
 * Port of blimp.cpp update_map() and measurement_map
 * ────────────────────────────────────────────────────────────────────────── */
void wlsGradientControllerAddMapPoint(float x, float y, float intensity)
{
    if (xSemaphoreTake(mapMutex, M2T(5)) != pdTRUE) return;

    int16_t gx = toGrid(x), gy = toGrid(y);

    /* Search for existing cell */
    for (int i = 0; i < mapSize; i++) {
        if (mapCells[i].gx == gx && mapCells[i].gy == gy) {
            /* Update only if new intensity is meaningfully better */
            if (intensity > mapCells[i].intensity + mapMinImprovement) {
                mapCells[i].x         = x;
                mapCells[i].y         = y;
                mapCells[i].intensity = intensity;
            }
            xSemaphoreGive(mapMutex);
            return;
        }
    }

    /* New cell */
    if (mapSize < MAP_MAX_POINTS) {
        mapCells[mapSize].x         = x;
        mapCells[mapSize].y         = y;
        mapCells[mapSize].intensity = intensity;
        mapCells[mapSize].gx        = gx;
        mapCells[mapSize].gy        = gy;
        mapCells[mapSize].occupied  = true;
        mapSize++;
    } else {
        /* Map full: replace lowest-intensity cell (simple eviction policy) */
        int minIdx = 0;
        for (int i = 1; i < MAP_MAX_POINTS; i++) {
            if (mapCells[i].intensity < mapCells[minIdx].intensity) minIdx = i;
        }
        if (intensity > mapCells[minIdx].intensity) {
            mapCells[minIdx].x         = x;
            mapCells[minIdx].y         = y;
            mapCells[minIdx].intensity = intensity;
            mapCells[minIdx].gx        = gx;
            mapCells[minIdx].gy        = gy;
        }
    }

    xSemaphoreGive(mapMutex);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Map-based gradient estimator
 * Direct port of blimp.cpp estimate_gradient()
 *
 * Weighted plane fit:  intensity ≈ a·x + b·y + c
 * Weight per point:    w_i = 1 / (d_i² + W_EPS)   [inverse-distance²]
 * Quality gate:        weighted R² ≥ mapR2Threshold
 * ────────────────────────────────────────────────────────────────────────── */
bool wlsGradientControllerUpdateMap(float cx, float cy,
                                     WlsGradientOutput *out)
{
    out->vx = out->vy = out->gradX = out->gradY = out->gradMagnitude = 0.0f;
    out->gradAngleDeg = 0.0f;
    out->r_squared    = 0.0f;
    out->mapReady     = (mapSize >= (int)mapMinPoints);

    if (!out->mapReady) return false;

    if (xSemaphoreTake(mapMutex, M2T(10)) != pdTRUE) return false;

    /* Collect neighbours with inverse-distance² weights */
    float pts_x[MAP_MAX_POINTS], pts_y[MAP_MAX_POINTS], pts_z[MAP_MAX_POINTS];
    float weights[MAP_MAX_POINTS];
    int   n          = 0;
    float weight_sum = 0.0f;

    for (int i = 0; i < mapSize; i++) {
        float dx = cx - mapCells[i].x;
        float dy = cy - mapCells[i].y;
        float d  = sqrtf(dx*dx + dy*dy);
        if (d >= mapMinDist && d <= mapMaxDist) {
            float wi = 1.0f / (d*d + mapWEps);
            pts_x[n] = mapCells[i].x;
            pts_y[n] = mapCells[i].y;
            pts_z[n] = mapCells[i].intensity;
            weights[n] = wi;
            weight_sum += wi;
            n++;
        }
    }

    xSemaphoreGive(mapMutex);

    if (n < (int)mapMinPoints) return false;

    /* Normalise weights (mirrors blimp.cpp) */
    for (int i = 0; i < n; i++) weights[i] /= weight_sum;

    /* Build weighted normal equations — identical to blimp.cpp scalar sums */
    float sw=0, swx=0, swy=0, swz=0;
    float swxx=0, swyy=0, swxy=0, swxz=0, swyz=0;

    for (int i = 0; i < n; i++) {
        float wi = weights[i];
        float x  = pts_x[i], y = pts_y[i], z = pts_z[i];
        sw   += wi;
        swx  += wi*x;    swy  += wi*y;    swz  += wi*z;
        swxx += wi*x*x;  swyy += wi*y*y;  swxy += wi*x*y;
        swxz += wi*x*z;  swyz += wi*y*z;
    }

    float M[3][3] = {
        {swxx, swxy, swx},
        {swxy, swyy, swy},
        {swx,  swy,  sw }
    };
    float R[3]  = {swxz, swyz, swz};
    float sol[3];

    if (!solve3x3(M, R, sol)) return false;

    float gx = sol[0], gy = sol[1], c_coeff = sol[2];

    /* Weighted R² quality check — exact port of blimp.cpp */
    float mean_z_w = swz / sw;
    float ss_res = 0.0f, ss_tot = 0.0f;
    for (int i = 0; i < n; i++) {
        float wi   = weights[i];
        float z    = pts_z[i];
        float pred = gx*pts_x[i] + gy*pts_y[i] + c_coeff;
        ss_res += wi * (z - pred) * (z - pred);
        ss_tot += wi * (z - mean_z_w) * (z - mean_z_w);
    }
    float r2 = (ss_tot > 1e-10f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    out->r_squared = r2;

    if (r2 < mapR2Threshold) {
        DEBUG_PRINT("WLS map: poor fit R²=%.3f (threshold=%.2f)\n",
                    (double)r2, (double)mapR2Threshold);
        return false;
    }

    float mag = sqrtf(gx*gx + gy*gy);
    out->gradX         = gx;
    out->gradY         = gy;
    out->gradMagnitude = mag;

    float ang = atan2f(gy, gx) * 180.0f / (float)M_PI;
    if (ang < 0.0f) ang += 360.0f;
    out->gradAngleDeg = ang;

    if (mag < wlsMinMag) {
        out->vx = out->vy = 0.0f;
    } else {
        out->vx = wlsStepSize * (gx / mag);
        out->vy = wlsStepSize * (gy / mag);
    }

    latestMap = *out;
    return true;
}

void wlsGradientControllerClearMap(void)
{
    if (xSemaphoreTake(mapMutex, M2T(10)) == pdTRUE) {
        memset(mapCells, 0, sizeof(mapCells));
        mapSize = 0;
        xSemaphoreGive(mapMutex);
    }
    DEBUG_PRINT("WLS map cleared\n");
}

int wlsGradientControllerGetMapSize(void) { return mapSize; }

/* ── PARAM ─────────────────────────────────────────────────────────────── */
static uint8_t clearMapFlag = 0;
static void clearMapCallback(void) {
    if (clearMapFlag) { wlsGradientControllerClearMap(); clearMapFlag = 0; }
}

PARAM_GROUP_START(wlsCtrl)
    PARAM_ADD(PARAM_FLOAT,  stepSize,   &wlsStepSize)
    PARAM_ADD(PARAM_FLOAT,  minMag,     &wlsMinMag)
    PARAM_ADD(PARAM_FLOAT,  weightExp,  &wlsWeightExp)
    PARAM_ADD(PARAM_UINT8,  mapMinPts,  &mapMinPoints)
    PARAM_ADD(PARAM_FLOAT,  mapR2,      &mapR2Threshold)
    PARAM_ADD(PARAM_FLOAT,  mapMaxDist, &mapMaxDist)
    PARAM_ADD(PARAM_FLOAT,  mapGridRes, &mapGridRes)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, clearMap, &clearMapFlag, clearMapCallback)
PARAM_GROUP_STOP(wlsCtrl)

/* ── LOG ───────────────────────────────────────────────────────────────── */
LOG_GROUP_START(wlsCtrl)
    /* Instantaneous estimator */
    LOG_ADD(LOG_FLOAT, iGradX,   &latestInstant.gradX)
    LOG_ADD(LOG_FLOAT, iGradY,   &latestInstant.gradY)
    LOG_ADD(LOG_FLOAT, iGradMag, &latestInstant.gradMagnitude)
    LOG_ADD(LOG_FLOAT, iGradAng, &latestInstant.gradAngleDeg)
    /* Map-based estimator */
    LOG_ADD(LOG_FLOAT, mGradX,   &latestMap.gradX)
    LOG_ADD(LOG_FLOAT, mGradY,   &latestMap.gradY)
    LOG_ADD(LOG_FLOAT, mGradMag, &latestMap.gradMagnitude)
    LOG_ADD(LOG_FLOAT, mGradAng, &latestMap.gradAngleDeg)
    LOG_ADD(LOG_FLOAT, mR2,      &latestMap.r_squared)
    LOG_ADD(LOG_INT32, mapSize,  &mapSize)
LOG_GROUP_STOP(wlsCtrl)
