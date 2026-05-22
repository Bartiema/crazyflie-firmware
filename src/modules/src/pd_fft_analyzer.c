/**
 * pd_fft_analyzer.c
 *
 * FFT-based per-channel frequency magnitude analyzer.
 * Direct port of the Teensy FFTAnalyzer class (fft_analyzer.h).
 *
 * Porting notes:
 *   - arm_rfft_fast_f32 is used identically to the Teensy (CMSIS-DSP available on both)
 *   - Sample rate changed: 1600 Hz (Teensy) → 500 Hz (CrazyFlie, pushed from pdTask)
 *   - FFT size changed:    1024 (Teensy)     → 256  (CrazyFlie)
 *   - Frequency resolution: 1.5625 Hz/bin   → 1.953 Hz/bin
 *   - SNR calculation: identical algorithm (exclude target + harmonics ± 10 bins)
 *   - Hamming window: identical formula
 */

#define DEBUG_MODULE "PDFFT"

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "arm_math.h"   /* CMSIS-DSP — already in crazyflie-firmware/lib/CMSIS/DSP */

#include "debug.h"
#include "static_mem.h"
#include "param.h"

#include "pd_fft_analyzer.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Constants
 * ────────────────────────────────────────────────────────────────────────── */

#define PD_SAMPLE_RATE_HZ   500.0f
#define FREQ_RESOLUTION     (PD_SAMPLE_RATE_HZ / (float)PD_FFT_SIZE)  /* 1.953 Hz */
#define NOISE_EXCL_BINS     10    /* ±bins excluded around target and harmonics */
#define NUM_HARMONICS       4     /* exclude fundamental + 4 harmonics for SNR */

/* ──────────────────────────────────────────────────────────────────────────
 * Static storage
 * ────────────────────────────────────────────────────────────────────────── */

/* Circular sample buffers — one per channel */
static uint16_t sampleBuf[PD_FFT_CHANNELS][PD_FFT_SIZE];
static int   writeIdx    = 0;
static int   sampleCount = 0;   /* total samples pushed, capped at PD_FFT_SIZE */

/* EMA smoothing factor applied to the per-bin spectrum each hop.
 * 0 = no smoothing (pass-through), 1 = fully frozen.
 * alpha=0.7 gives ~2.4× SNR improvement (≈ N=5 block averages) while
 * publishing a fresh spectrum after every hop (~4 Hz). */
static float specEmaAlpha = 0.7f;

/* True once the spectrum[] array has been seeded with a real measurement.
 * First hop after init/reset copies directly; subsequent hops EMA-blend. */
static bool spectrumInitialized = false;

/* FFT working buffers */
static float fftIn[PD_FFT_SIZE];                          /* scratch: linearised + windowed */
static float fftOut[PD_FFT_SIZE];                         /* scratch: complex output        */
static float tempMag[PD_FFT_SIZE / 2];                    /* scratch: per-hop magnitude     */
static float spectrum[PD_FFT_CHANNELS][PD_FFT_SIZE / 2];  /* EMA-smoothed live spectra      */

/* Scratch buffer for median noise floor — avoids stack allocation */
static float noiseBins[PD_FFT_SIZE / 2];

/* Shared Hamming window (computed once at init, same formula as Teensy) */
static float hammingWindow[PD_FFT_SIZE];

/* CMSIS-DSP FFT instance */
static arm_rfft_fast_instance_f32 fftInstance;

/* Mutex protecting sampleBuf and spectrum from concurrent access */
static StaticSemaphore_t fftMutexBuf;
static SemaphoreHandle_t fftMutex;

static bool initialized = false;
static bool bufferReady = false;   /* true once first full window is available */
static int  samplesSinceRun = 0;   /* samples pushed since last pdFftAnalyzerRun() */

/* ──────────────────────────────────────────────────────────────────────────
 * Helpers
 * ────────────────────────────────────────────────────────────────────────── */

static int floatAscCmp(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Init
 * ────────────────────────────────────────────────────────────────────────── */

bool pdFftAnalyzerInit(void)
{
    fftMutex = xSemaphoreCreateMutexStatic(&fftMutexBuf);
    ASSERT(fftMutex != NULL);

    /* Initialise CMSIS-DSP FFT for PD_FFT_SIZE-point real FFT */
    if (arm_rfft_fast_init_f32(&fftInstance, PD_FFT_SIZE) != ARM_MATH_SUCCESS) {
        DEBUG_PRINT("pdFftAnalyzer: CMSIS arm_rfft_fast_init_f32 FAILED\n");
        return false;
    }

    /* Pre-compute Hamming window — identical formula to Teensy fft_analyzer.h */
    for (int i = 0; i < PD_FFT_SIZE; i++) {
        hammingWindow[i] = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * i
                                                  / (float)(PD_FFT_SIZE - 1));
    }

    memset(sampleBuf, 0, sizeof(sampleBuf));
    memset(spectrum,  0, sizeof(spectrum));
    writeIdx             = 0;
    sampleCount          = 0;
    spectrumInitialized  = false;
    bufferReady          = false;
    initialized          = true;

    DEBUG_PRINT("pdFftAnalyzer: init OK (FFT=%d, SR=%.0f Hz, res=%.4f Hz/bin)\n",
                PD_FFT_SIZE, (double)PD_SAMPLE_RATE_HZ, (double)FREQ_RESOLUTION);
    return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Sample ingestion — called at 500 Hz from pdTask
 * ────────────────────────────────────────────────────────────────────────── */

void pdFftAnalyzerPushSample(const uint16_t pd[PD_FFT_CHANNELS])
{
    if (!initialized) return;

    /* No mutex here — writeIdx is only written by this one caller (pdTask).
     * pdFftAnalyzerRun() takes the mutex before reading sampleBuf. */
    for (int ch = 0; ch < PD_FFT_CHANNELS; ch++) {
        sampleBuf[ch][writeIdx] = pd[ch];
    }
    writeIdx = (writeIdx + 1) % PD_FFT_SIZE;

    if (sampleCount < PD_FFT_SIZE) {
        sampleCount++;
        if (sampleCount == PD_FFT_SIZE) {
            bufferReady = true;
        }
    }

    samplesSinceRun++;
}

bool pdFftAnalyzerReady(void) { return bufferReady; }

bool pdFftAnalyzerWindowReady(void)
{
    return bufferReady && (samplesSinceRun >= PD_FFT_HOP_SIZE);
}

/* ──────────────────────────────────────────────────────────────────────────
 * FFT execution — call at a lower rate (e.g. every PD_FFT_SIZE samples = 1.28 s)
 * ────────────────────────────────────────────────────────────────────────── */

bool pdFftAnalyzerRun(void)
{
    if (!initialized || !bufferReady) return false;

    if (xSemaphoreTake(fftMutex, M2T(10)) != pdTRUE) {
        DEBUG_PRINT("pdFftAnalyzer: mutex timeout in Run\n");
        return false;
    }

    /* Snapshot the current write index so we linearise from the oldest sample */
    int startIdx = writeIdx;

    for (int ch = 0; ch < PD_FFT_CHANNELS; ch++) {

        /* ── Step 1: Linearise circular buffer (oldest→newest) ─────────── */
        int dst = 0;
        for (int i = startIdx; i < PD_FFT_SIZE; i++) fftIn[dst++] = (float)sampleBuf[ch][i] / 4095.0f;
        for (int i = 0;        i < startIdx;    i++) fftIn[dst++] = (float)sampleBuf[ch][i] / 4095.0f;

        /* ── Step 2: Remove DC (subtract mean) ─────────────────────────── */
        float mean = 0.0f;
        arm_mean_f32(fftIn, PD_FFT_SIZE, &mean);
        arm_offset_f32(fftIn, -mean, fftIn, PD_FFT_SIZE);

        /* ── Step 3: Apply Hamming window ──────────────────────────────── */
        arm_mult_f32(fftIn, hammingWindow, fftIn, PD_FFT_SIZE);

        /* ── Step 4: Real FFT ───────────────────────────────────────────── */
        arm_rfft_fast_f32(&fftInstance, fftIn, fftOut, 0);

        /* ── Step 5: Complex magnitude ──────────────────────────────────── */
        arm_cmplx_mag_f32(fftOut, tempMag, PD_FFT_SIZE / 2);

        /* ── Step 6: EMA blend into live spectrum ───────────────────────── */
        if (!spectrumInitialized) {
            /* First hop after init or reset — copy directly so spectrum[]
             * starts at the true value rather than (1−alpha)-attenuated. */
            memcpy(spectrum[ch], tempMag, (PD_FFT_SIZE / 2) * sizeof(float));
        } else {
            /* spectrum[k] = alpha·spectrum[k] + (1−alpha)·tempMag[k]
             * Same alpha for all 8 channels → inter-channel ratios
             * (bearing cue) are preserved; only common noise is suppressed. */
            arm_scale_f32(spectrum[ch], specEmaAlpha,        spectrum[ch], PD_FFT_SIZE / 2);
            arm_scale_f32(tempMag,      1.0f - specEmaAlpha, tempMag,      PD_FFT_SIZE / 2);
            arm_add_f32(spectrum[ch], tempMag, spectrum[ch], PD_FFT_SIZE / 2);
        }
    }

    spectrumInitialized = true;
    samplesSinceRun     = 0;

    xSemaphoreGive(fftMutex);
    return true;   /* fresh EMA spectrum available after every hop (~4 Hz) */
}

/* ──────────────────────────────────────────────────────────────────────────
 * Accumulator reset — call on waypoint / frequency transitions
 * ────────────────────────────────────────────────────────────────────────── */

void pdFftAnalyzerResetAccumulator(void)
{
    if (!initialized) return;
    if (xSemaphoreTake(fftMutex, M2T(10)) != pdTRUE) {
        DEBUG_PRINT("pdFftAnalyzer: mutex timeout in ResetAccumulator\n");
        return;
    }
    /* Zero the EMA buffer so stale frequency-A bins do not bleed into
     * the new frequency-B spectrum.  Next hop re-seeds from scratch. */
    memset(spectrum, 0, sizeof(spectrum));
    spectrumInitialized = false;
    samplesSinceRun     = 0;
    xSemaphoreGive(fftMutex);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Frequency extraction — mirrors Teensy analyze_frequency_signal()
 * ────────────────────────────────────────────────────────────────────────── */

void pdFftAnalyzerGetFrequency(int ch, float target_freq_hz,
                                float search_window,
                                PdFreqResult *result)
{
    if (!initialized || !bufferReady || ch < 0 || ch >= PD_FFT_CHANNELS) {
        result->magnitude = 0.0f;
        result->snr       = 0.0f;
        return;
    }

    if (xSemaphoreTake(fftMutex, M2T(5)) != pdTRUE) {
        result->magnitude = 0.0f;
        result->snr       = 0.0f;
        return;
    }

    const int half = PD_FFT_SIZE / 2;

    /* ── Find peak magnitude within search band ─────────────────────────── */
    int target_bin  = (int)roundf(target_freq_hz / FREQ_RESOLUTION);
    int window_bins = (int)roundf(search_window  / FREQ_RESOLUTION);
    int s_start = target_bin - window_bins;  if (s_start < 1)    s_start = 1;
    int s_end   = target_bin + window_bins;  if (s_end   >= half) s_end   = half - 1;

    float peak = 0.0f;
    for (int i = s_start; i <= s_end; i++) {
        if (spectrum[ch][i] > peak) peak = spectrum[ch][i];
    }

    /* ── Calculate noise floor (median) ────────────────────────────────────
     * Collect all non-excluded bins then take the median.  The median is
     * robust against single-frequency spikes (motor harmonics, mains
     * interference) that would inflate a mean-based estimate.              */
    int noiseBinCount = 0;

    for (int i = 1; i < half; i++) {
        bool excluded = false;
        for (int h = 0; h < NUM_HARMONICS + 1; h++) {
            int exc_center = (int)roundf(target_freq_hz * (h + 1) / FREQ_RESOLUTION);
            if (abs(i - exc_center) <= NOISE_EXCL_BINS) {
                excluded = true;
                break;
            }
        }
        if (!excluded) {
            noiseBins[noiseBinCount++] = spectrum[ch][i];
        }
    }

    float noise_avg = 1.0f;
    if (noiseBinCount > 0) {
        qsort(noiseBins, noiseBinCount, sizeof(float), floatAscCmp);
        noise_avg = (noiseBinCount & 1)
            ? noiseBins[noiseBinCount / 2]
            : 0.5f * (noiseBins[noiseBinCount / 2 - 1] + noiseBins[noiseBinCount / 2]);
    }

    result->magnitude = peak;
    result->snr       = (noise_avg > 0.0f) ? (peak / noise_avg) : 0.0f;

    xSemaphoreGive(fftMutex);
}

/* ──────────────────────────────────────────────────────────────────────────
 * Parameters
 * ────────────────────────────────────────────────────────────────────────── */

PARAM_GROUP_START(pdFft)
    /** EMA alpha applied to every spectral bin each hop (0=off, 1=frozen).
     *  Default 0.7 → ~2.4× SNR vs single-hop, still publishes at ~4 Hz. */
    PARAM_ADD(PARAM_FLOAT, specAlpha, &specEmaAlpha)
PARAM_GROUP_STOP(pdFft)
