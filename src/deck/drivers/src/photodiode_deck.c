/**
 * photodiode_deck.c — CrazyFlie deck driver for the 8-channel photodiode board
 *
 * ADC:  Texas Instruments ADS7953SRHBT — 12-bit SAR, 16 channels
 * TIA:  OPA4350EA/2K5, ~3.2 kΩ feedback
 * VREF: REF5025AIDR → 2.500 V
 * CS:   DECK_GPIO_IO4
 *
 * Coexists with the flow deck on the shared SPI bus via the deck_spi API.
 * The bus is Mode 3 by compile-time; the ADS7953 accepts Mode 3.
 *
 * Auto-2 program-register command word (CMD_PROGRAM_AUTO2):
 *   DI[15:12] = 1001  — select Auto-2 program register
 *   DI[11]    = 1     — ENABLE programming bits below (critical)
 *   DI[10]    = 0     — reserved
 *   DI[9:6]   = last channel to scan, 4 bits
 *   DI[5:0]   = 0     — GPIO config
 *   For last channel = 7 (CH7):  1001 1 0 0111 0 00000  = 0x99C0
 *
 *   NB: the Teensy reference used 0x9700, which has DI[11]=0 — the enable
 *   bit clear means the program register is NEVER actually written, so
 *   the ADC stays in its default state (scan all 16 channels). That made
 *   the ADC return channels 8..15 interleaved with 0..7, which our
 *   routing-by-nibble code then discarded — showing up as periodic
 *   dropouts to zero in the plotter.
 *
 * Auto-2 mode-start command word (CMD_AUTO2_START):
 *   DI[15:12] = 0011  — enter Auto-2 mode
 *   DI[11]    = 1     — enable programming bits below
 *   DI[10]    = 1     — reset channel counter to first channel
 *   DI[9:6]   = 0     — Range 1 (0..VREF), no power-down, no GPIO output
 *   Result: 0x3800
 *
 * Response word: [15:12]=channel ID, [11:0]=12-bit result
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
#include "debug.h"
#include "static_mem.h"
#include "sleepus.h"

#include "photodiode_deck.h"
#include "pd_fft_analyzer.h"
#include "mode_manager.h"

/* ── Configuration ───────────────────────────────────────────────────────── */

#define ADS7953_SPI_BAUDRATE    SPI_BAUDRATE_2MHZ
#define PD_CS_PIN               DECK_GPIO_IO4
#define PD_SAMPLE_RATE_HZ       500
#define ADS7953_INTERFRAME_US   5

/* Corrected Auto-2 program command: enable bit set, last channel = 7. */
#define CMD_PROGRAM_AUTO2       0x99C0U
#define CMD_AUTO2_START         0x3800U
#define CMD_CONTINUE            0x0000U

/* ── Shared state ────────────────────────────────────────────────────────── */

static uint16_t pdValues[PD_CHANNEL_COUNT];   /* raw 12-bit codes, 0..4095 */
static bool     pdReady = false;

static StaticSemaphore_t pdMutexBuf;
static SemaphoreHandle_t pdMutex;

static StaticSemaphore_t pdDataReadyBuf;
static SemaphoreHandle_t pdDataReady;

/* ── 16-bit ADC frame via the shared SPI API ────────────────────────────── */

static uint16_t adsTransfer(uint16_t cmd)
{
    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    uint8_t rx[2] = { 0, 0 };

    spiBeginTransaction(ADS7953_SPI_BAUDRATE);

    digitalWrite(PD_CS_PIN, LOW);
    spiExchange(2, tx, rx);
    digitalWrite(PD_CS_PIN, HIGH);

    spiEndTransaction();

    sleepus(ADS7953_INTERFRAME_US);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

/* ── Read one full sweep of 8 channels in Auto-2 mode ───────────────────── */

static void adsReadAllChannels(uint16_t out[PD_CHANNEL_COUNT])
{
    /* First frame of each burst is stale — discard. */
    adsTransfer(CMD_CONTINUE);

    for (int i = 0; i < PD_CHANNEL_COUNT; i++) {
        uint16_t word = adsTransfer(CMD_CONTINUE);
        uint8_t  id   = (word >> 12) & 0x0F;
        uint16_t raw  = word & 0x0FFF;
        if (id < PD_CHANNEL_COUNT) {
            out[id] = raw;
        }
    }
}

/* ── Sampling task ───────────────────────────────────────────────────────── */

#define PD_TASK_STACKSIZE   (4 * configMINIMAL_STACK_SIZE)
#define PD_TASK_PRIORITY    3

static void pdTask(void *param)
{
    (void)param;
    systemWaitStart();
    vTaskDelay(M2T(50));

    adsTransfer(CMD_PROGRAM_AUTO2);
    adsTransfer(CMD_AUTO2_START);
    adsTransfer(CMD_CONTINUE);
    adsTransfer(CMD_CONTINUE);

    pdReady = true;
    DEBUG_PRINT("PD deck: ready @ %d Hz (Auto-2, shared SPI, last-ch=7)\n",
                PD_SAMPLE_RATE_HZ);

    /* Persistent scratch — bad frames hold the previous value. */
    uint16_t buf[PD_CHANNEL_COUNT] = { 0 };

    TickType_t lastWake = xTaskGetTickCount();

    while (1) {
        adsReadAllChannels(buf);

        xSemaphoreTake(pdMutex, portMAX_DELAY);
        memcpy(pdValues, buf, sizeof(pdValues));
        xSemaphoreGive(pdMutex);

        pdFftAnalyzerPushSample(buf);
        xSemaphoreGive(pdDataReady);
        vTaskDelayUntil(&lastWake, M2T(1000 / PD_SAMPLE_RATE_HZ));
    }
}

/* ── Deck lifecycle ──────────────────────────────────────────────────────── */

static void pdDeckInit(DeckInfo *info)
{
    (void)info;

    pdMutex     = xSemaphoreCreateMutexStatic(&pdMutexBuf);
    pdDataReady = xSemaphoreCreateBinaryStatic(&pdDataReadyBuf);
    ASSERT(pdMutex     != NULL);
    ASSERT(pdDataReady != NULL);

    pinMode(PD_CS_PIN, OUTPUT);
    digitalWrite(PD_CS_PIN, HIGH);

    xTaskCreate(pdTask, "pdTask", PD_TASK_STACKSIZE, NULL, PD_TASK_PRIORITY, NULL);
    modeManagerInit();

    DEBUG_PRINT("PD deck: init (shared SPI API, Mode 3 bus default)\n");
}

static bool pdDeckTest(void) { return true; }

/* ── Public API ──────────────────────────────────────────────────────────── */

bool pdDeckGetValues(uint16_t out[PD_CHANNEL_COUNT])
{
    if (!pdReady) return false;
    if (xSemaphoreTake(pdMutex, M2T(5)) == pdTRUE) {
        memcpy(out, pdValues, sizeof(pdValues));
        xSemaphoreGive(pdMutex);
        return true;
    }
    return false;
}

bool pdDeckIsReady(void) { return pdReady; }

/* ── Log variables ───────────────────────────────────────────────────────── */

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

/* ── Driver registration ─────────────────────────────────────────────────── */

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