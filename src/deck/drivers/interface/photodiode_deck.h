/**
 * photodiode_deck.h — Public API for the 8-channel photodiode expansion deck
 *
 * Hardware:  Custom CrazyFlie 2.1 expansion board
 * ADC:       Texas Instruments ADS7953SRHBT (12-bit, SPI Mode 0)
 * Amplifier: OPA4350EA/2K5 quad transimpedance amplifier (×2 ICs, 8 channels)
 * Filter:    RC low-pass per channel: 1600 Ω + 1 µF → f_c ≈ 100 Hz (anti-aliasing)
 * Reference: REF5025AIDR — 2.500 V precision reference (REFP pin)
 * CS pin:    DECK_GPIO_IO4  ← conflicts with micro-SD deck (cannot use both)
 *
 * Channel mapping (confirmed from schematic):
 *   PD_IN_N → OPA4350 TIA → RC filter → AMP_OUT_N → ADS7953 CH_N → pdValues[N]
 *   (1:1 direct mapping throughout — no channel reordering)
 *
 * Physical photodiode layout (8 sensors, equally spaced, 45° intervals):
 *   ch0:  0°  (forward,       +X body axis)
 *   ch1: 45°  (forward-left)
 *   ch2: 90°  (left,          +Y body axis)
 *   ch3: 135° (rear-left)
 *   ch4: 180° (rear,          -X body axis)
 *   ch5: 225° (rear-right)
 *   ch6: 270° (right,         -Y body axis)
 *   ch7: 315° (forward-right)
 *
 * NOTE: CrazyFlie body frame — +X = forward, +Y = left, angles measured
 *       counter-clockwise from forward when viewed from above.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PD_CHANNEL_COUNT   8

/**
 * pdDeckGetValues()
 *
 * Copy the most recent normalised photodiode readings into out[8].
 * Values are in [0.0, 1.0] (0V → 0.0, VREF=2.5V → 1.0).
 *
 * Thread-safe (mutex-protected). Max blocking: 5 ms.
 * Returns false if deck not ready or mutex timeout.
 */
bool pdDeckGetValues(float out[PD_CHANNEL_COUNT]);

/** Returns true once at least one conversion has completed. */
bool pdDeckIsReady(void);
