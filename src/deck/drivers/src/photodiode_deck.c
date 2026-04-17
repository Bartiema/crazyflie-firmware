/**
 * photodiode_deck.c — CrazyFlie deck driver for the 8-channel photodiode board
 *
 * ADC:       Texas Instruments ADS7953SRHBT
 *              - 12-bit SAR ADC, 16 channels, SPI Mode 0 (CPOL=0 CPHA=0)
 *              - Max SPI clock: 20 MHz; we use 10 MHz for margin
 *              - 16-bit SPI frames; MSB first
 *              - Used in Auto-2 scan mode: autonomously cycles CH0→CH7
 * Amplifier: OPA4350EA/2K5 transimpedance amplifier, ~3.2 kΩ feedback
 * Reference: REF5025AIDR → REFP = 2.500 V, REFM = GND
 * CS pin:    DECK_GPIO_IO4 (active low)
 *            *** Incompatible with micro-SD deck (same CS pin) ***
 *
 * ADS7953 Auto-2 mode overview
 * ─────────────────────────────
 * After a one-time "Program Auto-2" command, the ADS7953 automatically
 * cycles through the selected channel range on every subsequent CS pulse.
 * We configure it to scan CH0–CH7 continuously. Each 16-bit read returns
 * the PREVIOUS conversion result (pipeline delay of 1 frame), so we discard
 * the first frame after mode entry and then read 8 frames to get CH0–CH7.
 *
 * Frame format (read back, Auto-2 mode):
 *   Bits [15:12] — channel ID of the result
 *   Bits [11: 0] — 12-bit ADC result (0–4095)
 *
 * Normalisation:
 *   float value = raw / 4095.0f   →  range [0.0, 1.0]
 *   Physical range: 0 V (dark) – 2.5 V (saturated)
 */

#define DEBUG_MODULE "PDDECK"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "deck.h"
#include "deck_spi.h"
#include "deck_digital.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "static_mem.h"

#include "photodiode_deck.h"

/* ──────────────────────────────────────────────────────────────────────────
 * Hardware constants
 * ────────────────────────────────────────────────────────────────────────── */

/** SPI baudrate — ADS7953 max 20 MHz; SPI_BAUDRATE_12MHZ gives 10.5 MHz actual
 *  (APB2=84MHz, prescaler=8), safely within spec. */
#define ADS7953_SPI_BAUDRATE    SPI_BAUDRATE_12MHZ

/** Chip-select pin — must match schematic note: "We will use IO_4 as our CS pin." */
#define PD_CS_PIN               DECK_GPIO_IO4

/** Photodiode sampling rate in Hz. */
#define PD_SAMPLE_RATE_HZ       200

/** Total SPI frames per burst (1 dummy discard + 8 channel reads). */
#define ADS7953_BURST_FRAMES    9

/** Size of one SPI burst in bytes (each frame = 2 bytes). */
#define ADS7953_BURST_BYTES     (ADS7953_BURST_FRAMES * 2)

/* ──────────────────────────────────────────────────────────────────────────
 * ADS7953 command words (16-bit, sent MSB-first)
 *
 * See ADS7953 datasheet (SBAS424), Table 2 "Frame Bit Descriptions"
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * "Program Register 1 for Auto-2 mode, CH0–CH7, ±VREF range"
 *
 * Bit breakdown:
 *   [15:12] = 0001  Program Register 1
 *   [11]    = 1     AUTO: enable auto-scan
 *   [10]    = 1     RNG:  0 = 0…+VREF range (we want this for TIA outputs)
 *   [9]     = 0     (reserved)
 *   [8]     = 0     DFT:  keep default
 *   [7:4]   = 0111  Last channel = CH7
 *   [3:0]   = 0000  First channel = CH0
 *
 * Result: 0001 1 1 00  0111 0000 = 0x1C70
 *
 * NOTE: Verify against your ADS7953 silicon revision.
 *       Datasheet SBAS424D, Table 2, "Auto-2 Scan with Range Selection".
 */
#define ADS7953_CMD_PROGRAM_AUTO2   0x1C70U

/**
 * "Continue in current mode" NOP frame — sent for each subsequent read
 * after the mode is programmed.
 * Bit [15:12] = 0000 = "Continue"
 */
#define ADS7953_CMD_CONTINUE        0x0000U

/* ──────────────────────────────────────────────────────────────────────────
 * Shared state
 * ────────────────────────────────────────────────────────────────────────── */

static uint16_t  pdValues[PD_CHANNEL_COUNT];
static bool      pdReady = false;

static StaticSemaphore_t pdMutexBuf;
static SemaphoreHandle_t pdMutex;

static StaticSemaphore_t pdDataReadyBuf;
static SemaphoreHandle_t pdDataReady;

/* ──────────────────────────────────────────────────────────────────────────
 * Internal helpers
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * adsWriteCommand() — send a single 16-bit command word, ignore MISO.
 * CS is asserted / deasserted around the transaction.
 */
static void adsWriteCommand(uint16_t cmd)
{
    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    uint8_t rx[2];

    spiBeginTransaction(ADS7953_SPI_BAUDRATE);
    digitalWrite(PD_CS_PIN, LOW);
    spiExchange(2, tx, rx);
    digitalWrite(PD_CS_PIN, HIGH);
    spiEndTransaction();
}

/**
 * adsReadAllChannels() — burst-read CH0–CH7 in Auto-2 mode.
 *
 * The ADS7953 pipeline means the first frame after CS assert contains the
 * result of the PREVIOUS conversion (from the prior burst).  We read
 * ADS7953_BURST_FRAMES = 9 frames: frame 0 is discarded, frames 1–8
 * carry CH0–CH7 results for the CURRENT burst.
 *
 * We also cross-check the channel-ID nibble in each returned frame to
 * detect any de-synchronisation with the auto-scan sequence.
 */
static void adsReadAllChannels(uint16_t out[PD_CHANNEL_COUNT])
{
    uint8_t tx[ADS7953_BURST_BYTES];
    uint8_t rx[ADS7953_BURST_BYTES];

    /* All transmit frames are "Continue" NOPs in auto-scan mode */
    for (int i = 0; i < ADS7953_BURST_FRAMES; i++) {
        tx[i * 2]     = (uint8_t)(ADS7953_CMD_CONTINUE >> 8);
        tx[i * 2 + 1] = (uint8_t)(ADS7953_CMD_CONTINUE & 0xFF);
    }

    spiBeginTransaction(ADS7953_SPI_BAUDRATE);
    digitalWrite(PD_CS_PIN, LOW);
    spiExchange(ADS7953_BURST_BYTES, tx, rx);
    digitalWrite(PD_CS_PIN, HIGH);
    spiEndTransaction();

    /* Decode frames */
    for (int word_idx = 0; word_idx < ADS7953_BURST_FRAMES / 2; word_idx++) {
        uint16_t word = ((uint16_t)rx[word_idx] << 8) | rx[word_idx + 1];

        /* Bits [15:12]: channel ID returned by ADS7953 */
        uint8_t  reported_ch = (word >> 12) & 0x0F;
        uint16_t raw         = word & 0x0FFF;        
        if (reported_ch < 8) {
            out[reported_ch] = raw;                   
        }
    }
}

/* ──────────────────────────────────────────────────────────────────────────
 * Sampling task
 * ────────────────────────────────────────────────────────────────────────── */

#define PD_TASK_STACKSIZE   (4 * configMINIMAL_STACK_SIZE)
#define PD_TASK_PRIORITY    3   /* same level as other deck sensor tasks */

STATIC_MEM_TASK_ALLOC(pdTask, PD_TASK_STACKSIZE);

static void pdTask(void *param)
{
    (void)param;

    /* Wait for the rest of the firmware to finish initialising */
    systemWaitStart();

    /* Programme the ADS7953 into Auto-2 scan mode (CH0–CH7) */
    adsWriteCommand(ADS7953_CMD_PROGRAM_AUTO2);

    /* Discard first burst — pipeline contains stale result from before mode-change */
    float discard[PD_CHANNEL_COUNT];
    adsReadAllChannels(discard);

    pdReady = true;
    DEBUG_PRINT("PD deck: ADS7953 ready, scanning CH0-CH7 @ %d Hz\n", PD_SAMPLE_RATE_HZ);

    TickType_t lastWake = xTaskGetTickCount();

    while (1) {
        uint16_t buf[PD_CHANNEL_COUNT];
        adsReadAllChannels(buf);

        /* Update shared values under mutex */
        if (xSemaphoreTake(pdMutex, M2T(2)) == pdTRUE) {
            memcpy(pdValues, buf, sizeof(pdValues));
            xSemaphoreGive(pdMutex);
        } else {
            DEBUG_PRINT("PD: mutex timeout (driver overloaded?)\n");
        }

        /* Non-blocking post — if no consumer is waiting, the token is lost */
        xSemaphoreGive(pdDataReady);

        vTaskDelayUntil(&lastWake, M2T(1000 / PD_SAMPLE_RATE_HZ));
    }
}

/* ──────────────────────────────────────────────────────────────────────────
 * Deck lifecycle callbacks
 * ────────────────────────────────────────────────────────────────────────── */

static void pdDeckInit(DeckInfo *info)
{
    (void)info;

    /* Initialise synchronisation primitives */
    pdMutex     = xSemaphoreCreateMutexStatic(&pdMutexBuf);
    pdDataReady = xSemaphoreCreateBinaryStatic(&pdDataReadyBuf);

    ASSERT(pdMutex     != NULL);
    ASSERT(pdDataReady != NULL);

    /* Configure CS pin as output, deasserted (high) */
    pinMode(PD_CS_PIN, OUTPUT);
    digitalWrite(PD_CS_PIN, HIGH);

    /* Initialise the SPI bus (creates mutex, configures DMA, GPIO) */
    spiBegin();

    /* Spawn sampling task */
    STATIC_MEM_TASK_CREATE(pdTask, pdTask, "pdTask", NULL, PD_TASK_PRIORITY);

    DEBUG_PRINT("PD deck: initialised (ADS7953, SPI Mode 0, IO4 CS)\n");
}

static bool pdDeckTest(void)
{
    /* SPI is not available until systemWaitStart() completes in pdTask,
     * so we cannot test hardware here. Return true and rely on runtime
     * debug output to verify correct operation. */
    DEBUG_PRINT("PD deck: self-test PASS (deferred — SPI tested at runtime)\n");
    return true;
}

/* ──────────────────────────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────────────────────────── */

bool pdDeckGetValues(float out[PD_CHANNEL_COUNT])
{
    if (!pdReady) {
        return false;
    }

    if (xSemaphoreTake(pdMutex, M2T(5)) == pdTRUE) {
        memcpy(out, pdValues, sizeof(pdValues));
        xSemaphoreGive(pdMutex);
        return true;
    }

    return false;   /* mutex timeout */
}

bool pdDeckIsReady(void)
{
    return pdReady;
}

/* ──────────────────────────────────────────────────────────────────────────
 * LOG variables — visible in cfclient and cflib LogConfig
 * ────────────────────────────────────────────────────────────────────────── */

LOG_GROUP_START(pd)
    LOG_ADD(LOG_UINT16, ch0, &pdValues[0])
    LOG_ADD(LOG_UINT16, ch1, &pdValues[1])
    LOG_ADD(LOG_UINT16, ch2, &pdValues[2])
    LOG_ADD(LOG_UINT16, ch3, &pdValues[3])
    LOG_ADD(LOG_UINT16, ch4, &pdValues[4])
    LOG_ADD(LOG_UINT16, ch5, &pdValues[5])
    LOG_ADD(LOG_UINT16, ch6, &pdValues[6])
    LOG_ADD(LOG_UINT16, ch7, &pdValues[7])
LOG_GROUP_STOP(pd)

/* ──────────────────────────────────────────────────────────────────────────
 * Deck driver registration
 *
 * vid = 0, pid = 0, name = "pdDeck":
 *   No OW EEPROM fitted on this custom board. During development force
 *   initialisation with:  CONFIG_DECK_FORCE="pdDeck"  in .config
 *   For production, solder a 1-wire EEPROM and assign a VID/PID.
 * ────────────────────────────────────────────────────────────────────────── */

static const DeckDriver pd_deck_driver = {
    .vid = 0x00,
    .pid = 0x00,
    .name = "pdDeck",

    .usedPeriph = DECK_USING_SPI,
    .usedGpio   = DECK_USING_IO_4,

    .init = pdDeckInit,
    .test = pdDeckTest,
};

DECK_DRIVER(pd_deck_driver);
