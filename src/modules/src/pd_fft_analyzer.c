/**
 * pd_fft_analyzer.c
 *
 * FFT-based per-channel frequency magnitude analyzer.
 * Direct port of the Teensy FFTAnalyzer class (fft_analyzer.h).
 *
 * Porting notes:
 *   - arm_rfft_fast_f32 is used identically to the Teensy (CMSIS-DSP available on both)
 *   - Sample rate changed: 1600 Hz (Teensy) → 200 Hz (CrazyFlie)
 *   - FFT size changed:    1024 (Teensy)     → 256  (CrazyFlie)
 *   - Frequency resolution: 1.5625 Hz/bin   → 0.78125 Hz/bin (better!)
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

#include "pd_fft_analyzer.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Constants
 * ────────────────────────────────────────────────────────────────────────── */

#define PD_SAMPLE_RATE_HZ   200.0f
#define FREQ_RESOLUTION     (PD_SAMPLE_RATE_HZ / (float)PD_FFT_SIZE)  /* 0.78125 Hz */
#define NOISE_EXCL_BINS     10    /* ±bins excluded around target and harmonics */
#define NUM_HARMONICS       4     /* exclude fundamental + 4 harmonics for SNR */

/* ──────────────────────────────────────────────────────────────────────────
 * Static storage
 * ────────────────────────────────────────────────────────────────────────── */

/* Circular sample buffers — one per channel */
static float sampleBuf[PD_FFT_CHANNELS][PD_FFT_SIZE];
static int   writeIdx    = 0;
static int   sampleCount = 0;   /* total samples pushed, capped at PD_FFT_SIZE */

/* FFT working buffers */
static float fftIn[PD_FFT_SIZE];                        /* scratch: linearised + windowed */
static float fftOut[PD_FFT_SIZE];                       /* scratch: complex output        */
static float spectrum[PD_FFT_CHANNELS][PD_FFT_SIZE / 2]; /* magnitude spectra             */

/* Shared Hamming window (computed once at init, same formula as Teensy) */
static float hammingWindow[PD_FFT_SIZE];

/* CMSIS-DSP FFT instance */
static arm_rfft_fast_instance_f32 fftInstance;

/* Mutex protecting sampleBuf and spectrum from concurrent access */
static StaticSemaphore_t fftMutexBuf;
static SemaphoreHandle_t fftMutex;

static bool initialized = false;
static bool bufferReady = false;   /* true once first full window is available */

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
    writeIdx    = 0;
    sampleCount = 0;
    bufferReady = false;
    initialized = true;

    DEBUG_PRINT("pdFftAnalyzer: init OK (FFT=%d, SR=%.0f Hz, res=%.4f Hz/bin)\n",
                PD_FFT_SIZE, (double)PD_SAMPLE_RATE_HZ, (double)FREQ_RESOLUTION);
    return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Sample ingestion — called at 200 Hz from pdTask
 * ────────────────────────────────────────────────────────────────────────── */

void pdFftAnalyzerPushSample(const float pd[PD_FFT_CHANNELS])
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
}

bool pdFftAnalyzerReady(void) { return bufferReady; }

/* ──────────────────────────────────────────────────────────────────────────
 * FFT execution — call at a lower rate (e.g. every PD_FFT_SIZE samples = 1.28 s)
 * ────────────────────────────────────────────────────────────────────────── */

void pdFftAnalyzerRun(void)
{
    if (!initialized || !bufferReady) return;

    if (xSemaphoreTake(fftMutex, M2T(10)) != pdTRUE) {
        DEBUG_PRINT("pdFftAnalyzer: mutex timeout in Run\n");
        return;
    }

    /* Snapshot the current write index so we linearise from the oldest sample */
    int startIdx = writeIdx;

    for (int ch = 0; ch < PD_FFT_CHANNELS; ch++) {

        /* ── Step 1: Linearise circular buffer (oldest→newest) ─────────── */
        int dst = 0;
        for (int i = startIdx; i < PD_FFT_SIZE; i++) fftIn[dst++] = sampleBuf[ch][i];
        for (int i = 0;        i < startIdx;    i++) fftIn[dst++] = sampleBuf[ch][i];

        /* ── Step 2: Remove DC (subtract mean) — identical to Teensy ───── */
        float mean = 0.0f;
        arm_mean_f32(fftIn, PD_FFT_SIZE, &mean);
        arm_offset_f32(fftIn, -mean, fftIn, PD_FFT_SIZE);

        /* ── Step 3: Apply Hamming window ──────────────────────────────── */
        arm_mult_f32(fftIn, hammingWindow, fftIn, PD_FFT_SIZE);

        /* ── Step 4: Real FFT (in-place, forward) ──────────────────────── */
        arm_rfft_fast_f32(&fftInstance, fftIn, fftOut, 0);

        /* ── Step 5: Complex magnitude → spectrum[ch][0..FFT_SIZE/2-1] ─── */
        arm_cmplx_mag_f32(fftOut, spectrum[ch], PD_FFT_SIZE / 2);
    }

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

    /* ── Calculate noise floor — identical to Teensy ────────────────────── */
    /*    Exclude target + harmonics ± NOISE_EXCL_BINS each                  */
    float noise_sum  = 0.0f;
    int   noise_bins = 0;

    for (int i = 1; i < half; i++) {
        bool excluded = false;
        for (int h = 0; h < NUM_HARMONICS + 1; h++) {   /* fundamental + harmonics */
            int exc_center = (int)roundf(target_freq_hz * (h + 1) / FREQ_RESOLUTION);
            if (abs(i - exc_center) <= NOISE_EXCL_BINS) {
                excluded = true;
                break;
            }
        }
        if (!excluded) {
            noise_sum += spectrum[ch][i];
            noise_bins++;
        }
    }

    float noise_avg = (noise_bins > 0) ? (noise_sum / (float)noise_bins) : 1.0f;

    result->magnitude = peak;
    result->snr       = (noise_avg > 0.0f) ? (peak / noise_avg) : 0.0f;

    xSemaphoreGive(fftMutex);
}
