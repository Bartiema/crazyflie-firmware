/**
 * pd_fft_analyzer.h
 *
 * Per-channel FFT magnitude analyzer for the photodiode deck.
 *
 * This module mirrors the Teensy FFTAnalyzer class but adapted for the
 * CrazyFlie firmware environment:
 *   - Uses CMSIS-DSP arm_rfft_fast_f32 (already in the CF firmware tree)
 *   - Accumulates a circular sample buffer from pdDeckGetValues() at the
 *     PD sampling rate (200 Hz)
 *   - Runs FFT on demand when a full window is available
 *   - Extracts magnitude and SNR at a caller-specified target frequency
 *
 * Key differences from Teensy:
 *   - Teensy: 1024-point FFT at 1600 Hz sample rate → 1.5625 Hz/bin
 *   - CrazyFlie: 256-point FFT at 200 Hz sample rate → 0.78125 Hz/bin
 *     This gives adequate frequency resolution for separating 150/200 Hz targets
 *     while keeping RAM usage and compute time suitable for the STM32F405.
 *
 * RAM cost per channel: 256 * 4 bytes = 1 KB sample buffer
 *                       256 * 4 bytes = 1 KB FFT output buffer
 *                       256 * 4 bytes = 1 KB Hamming window (shared across all channels)
 * Total: ~17 KB for 8 channels + shared window.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** FFT window size. Must be a power of 2. 256 @ 200 Hz → 0.78 Hz/bin. */
#define PD_FFT_SIZE       256

/** Number of photodiode channels. */
#define PD_FFT_CHANNELS   8

/**
 * Result of a single-frequency analysis for one channel.
 */
typedef struct {
    float magnitude;   /**< FFT magnitude at target frequency          */
    float snr;         /**< Signal-to-noise ratio (signal / noise_avg) */
} PdFreqResult;

/**
 * pdFftAnalyzerInit()
 * Initialise CMSIS-DSP FFT instance and Hamming window. Call once at startup.
 * Returns false on CMSIS-DSP init failure.
 */
bool pdFftAnalyzerInit(void);

/**
 * pdFftAnalyzerPushSample()
 * Add one new ADC sample for all 8 channels to the circular buffers.
 * Call this at the PD sampling rate (200 Hz) from the sampling task.
 */
void pdFftAnalyzerPushSample(const float pd[PD_FFT_CHANNELS]);

/**
 * pdFftAnalyzerReady()
 * Returns true when the circular buffer contains at least PD_FFT_SIZE samples
 * (i.e., the first full window is available).
 */
bool pdFftAnalyzerReady(void);

/**
 * pdFftAnalyzerRun()
 * Run the FFT on all 8 channels using the current buffer contents.
 * This is the expensive call (~1–2 ms on STM32F405); call it only when
 * pdFftAnalyzerReady() returns true and a new window is due.
 * Thread-safe: takes an internal mutex.
 */
void pdFftAnalyzerRun(void);

/**
 * pdFftAnalyzerGetFrequency()
 * Extract the magnitude and SNR at @target_freq_hz for channel @ch.
 * Must be called after pdFftAnalyzerRun().
 *
 * SNR calculation mirrors Teensy: noise = mean of all bins excluding the
 * target frequency and its first 4 harmonics (±10 bins exclusion window).
 *
 * @param ch              Channel index [0, PD_FFT_CHANNELS)
 * @param target_freq_hz  Target frequency in Hz (e.g. 150.0f, 200.0f)
 * @param search_window   Half-width of search band in Hz (default 2.0 Hz)
 * @param result          Output magnitude and SNR
 */
void pdFftAnalyzerGetFrequency(int ch, float target_freq_hz,
                                float search_window,
                                PdFreqResult *result);
