/**
 * bearing_angle_controller.c
 *
 * Exact port of Teensy AngleCalculator (angle_calculator.h).
 *
 * Sensor geometry confirmed from PCB file (photodiode-expansion.kicad_pcb):
 *   Board centre at PCB coords (139.001, 97.001).
 *
 *   ch | J(PCB) | drone angle | dist
 *   ---+--------+-------------+---------
 *    0 | J1     |  45°       | 18.785 mm
 *    1 | J2     |  0°        | 18.489 mm
 *    2 | J3     |  135°      | 18.641 mm
 *    3 | J4     |  90°       | 18.670 mm
 *    4 | J5     |  225°      | 18.553 mm
 *    5 | J6     |  180°      | 18.489 mm
 *    6 | J7     |  270°      | 18.670 mm 
 *    7 | J8     |  315°      | 18.641 mm
 *
 * Channel wiring (confirmed from both schematics, exact 5.08 mm ΔY per step):
 *   J1(sch) → PD_IN_0 → AMP_OUT_0 → ADC_CH0 → pdValues[0]
 *   J2(sch) → PD_IN_1 → AMP_OUT_1 → ADC_CH1 → pdValues[1]   ... etc.
 *
 * NOTE: ch6 (J7) is physically closest to drone forward (7.7°), not ch0.
 *       The user's stated "ch0 = 0°" refers to the intended nominal layout.
 *       These exact angles give better bearing accuracy than the 45° approximation.
 */

#define DEBUG_MODULE "BACTRL"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "log.h"
#include "param.h"
#include "debug.h"

#include "bearing_angle_controller.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Exact sensor angles from PCB layout (degrees, drone body frame)
 * Body frame: 0° = forward (+X), 90° = left (+Y), CCW positive
 * ────────────────────────────────────────────────────────────────────────── */
static const float SENSOR_ANGLES_DEG[BA_SENSOR_COUNT] = {
    45.0f,
    0.0f,
    135.0f,
    90.0f,
    225.0f,
    180.0f,
    270.0f,
    315.0f
};

static float sensorCos[BA_SENSOR_COUNT];
static float sensorSin[BA_SENSOR_COUNT];

/* ── Calibration ───────────────────────────────────────────────────────── */
static float   correctionFactors[BA_SENSOR_COUNT] = {
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};
static float   calMagSum[BA_SENSOR_COUNT];
static int     calSampleCount = 0;
static uint8_t calActive      = 0;
static uint8_t calMinSamples  = 50;

static BearingAngleOutput latestOut;

/* ── Init ──────────────────────────────────────────────────────────────── */
void bearingAngleControllerInit(void)
{
    for (int i = 0; i < BA_SENSOR_COUNT; i++) {
        float rad    = SENSOR_ANGLES_DEG[i] * (float)M_PI / 180.0f;
        sensorCos[i] = cosf(rad);
        sensorSin[i] = sinf(rad);
    }
    memset(&latestOut, 0, sizeof(latestOut));
    DEBUG_PRINT("BearingAngle init OK (exact PCB angles)\n");
}

bool bearingAngleControllerTest(void)
{
    /* ch6 is at 7.7° (forward) — light only on ch6 → bearing ≈ 7.7° */
    BearingAngleInput testIn = {{0}};
    testIn.magnitude[6] = 1.0f;
    testIn.max_snr = 10.0f;

    BearingAngleOutput testOut;
    bearingAngleControllerUpdate(&testIn, &testOut);

    if (!testOut.valid || fabsf(testOut.bearing_deg - 7.7f) > 5.0f) {
        DEBUG_PRINT("BearingAngle self-test FAIL (bearing=%.2f°)\n",
                    (double)testOut.bearing_deg);
        return false;
    }
    DEBUG_PRINT("BearingAngle self-test PASS (bearing=%.2f°)\n",
                (double)testOut.bearing_deg);
    return true;
}

/* ── Calibration helpers ───────────────────────────────────────────────── */
void bearingAngleApplyCalibration(float mag[BA_SENSOR_COUNT])
{
    for (int i = 0; i < BA_SENSOR_COUNT; i++) mag[i] *= correctionFactors[i];
}

void bearingAngleResetCalibration(void)
{
    for (int i = 0; i < BA_SENSOR_COUNT; i++) correctionFactors[i] = 1.0f;
}

static void runCalibration(const float mag[BA_SENSOR_COUNT])
{
    for (int i = 0; i < BA_SENSOR_COUNT; i++) calMagSum[i] += mag[i];
    calSampleCount++;

    if (calActive == 2 && calSampleCount >= calMinSamples) {
        float totalAvg = 0.0f, avg[BA_SENSOR_COUNT];
        for (int i = 0; i < BA_SENSOR_COUNT; i++) {
            avg[i] = calMagSum[i] / (float)calSampleCount;
            totalAvg += avg[i];
        }
        float refMag = totalAvg / BA_SENSOR_COUNT;
        for (int i = 0; i < BA_SENSOR_COUNT; i++) {
            correctionFactors[i] = (avg[i] > 0.0f) ? (refMag / avg[i]) : 1.0f;
        }
        DEBUG_PRINT("BearingAngle calibration done (%d samples, ref=%.4f)\n",
                    calSampleCount, (double)refMag);
        calActive = 0; calSampleCount = 0;
        memset(calMagSum, 0, sizeof(calMagSum));
    }
}

/* ── Core update (port of calculate_weighted_angle_internal) ───────────── */
void bearingAngleControllerUpdate(const BearingAngleInput *in,
                                   BearingAngleOutput      *out)
{
    float mag[BA_SENSOR_COUNT];
    memcpy(mag, in->magnitude, sizeof(mag));

    if (calActive >= 1) runCalibration(mag);
    bearingAngleApplyCalibration(mag);

    float sum_x = 0.0f, sum_y = 0.0f, weight_sum = 0.0f;
    for (int i = 0; i < BA_SENSOR_COUNT; i++) {
        if (mag[i] > 0.0f) {
            sum_x      += mag[i] * sensorCos[i];
            sum_y      += mag[i] * sensorSin[i];
            weight_sum += mag[i];
        }
    }

    out->total_weight = weight_sum;
    if (weight_sum <= 0.0f) {
        out->bearing_deg = out->bearing_rad = 0.0f;
        out->vx = out->vy = out->yaw_rate_deg = 0.0f;
        out->valid = false;
        latestOut  = *out;
        return;
    }

    float rad = atan2f(sum_y, sum_x);
    float deg = rad * 180.0f / (float)M_PI;
    if (deg < 0.0f) deg += 360.0f;

    out->bearing_deg  = deg;
    out->bearing_rad  = rad;
    out->valid        = true;
    out->vx = out->vy = out->yaw_rate_deg = 0.0f;
    latestOut = *out;
}

/* ── PARAM ─────────────────────────────────────────────────────────────── */
PARAM_GROUP_START(bearingCtrl)
    PARAM_ADD(PARAM_UINT8, calibrate,  &calActive)
    PARAM_ADD(PARAM_UINT8, calMinSamp, &calMinSamples)
    PARAM_ADD(PARAM_FLOAT, corr0, &correctionFactors[0])
    PARAM_ADD(PARAM_FLOAT, corr1, &correctionFactors[1])
    PARAM_ADD(PARAM_FLOAT, corr2, &correctionFactors[2])
    PARAM_ADD(PARAM_FLOAT, corr3, &correctionFactors[3])
    PARAM_ADD(PARAM_FLOAT, corr4, &correctionFactors[4])
    PARAM_ADD(PARAM_FLOAT, corr5, &correctionFactors[5])
    PARAM_ADD(PARAM_FLOAT, corr6, &correctionFactors[6])
    PARAM_ADD(PARAM_FLOAT, corr7, &correctionFactors[7])
PARAM_GROUP_STOP(bearingCtrl)

/* ── LOG ───────────────────────────────────────────────────────────────── */
LOG_GROUP_START(bearingCtrl)
    LOG_ADD(LOG_FLOAT, bearing, &latestOut.bearing_deg)
    LOG_ADD(LOG_FLOAT, weight,  &latestOut.total_weight)
    LOG_ADD(LOG_UINT8, valid,   &latestOut.valid)
LOG_GROUP_STOP(bearingCtrl)
