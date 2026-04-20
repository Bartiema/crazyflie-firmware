/**
 * photodiode_deck.c — CrazyFlie deck driver for the 8-channel photodiode board
 *
 * ADC:  Texas Instruments ADS7953SRHBT — 12-bit SAR, 16 channels
 * TIA:  OPA4350EA/2K5, ~3.2 kΩ feedback
 * VREF: REF5025AIDR → 2.500 V
 * CS:   DECK_GPIO_IO4
 *
 * Coexists with the flow deck on the shared SPI bus via the deck_spi API,
 * so the mutex, DMA and GPIO setup is handled for us. The shared bus is
 * compiled for Mode 3 (CPOL=1 CPHA=1) because the PMW3901 on the flow
 * deck needs it. The ADS7953 accepts Mode 3 as well — DIN is latched on
 * the SCK falling edge and DOUT changes on the SCK falling edge, which
 * matches the Mode 3 master behaviour. No mode flipping is needed.
 *
 * ADS7953 Auto-2 mode (same protocol as the Teensy reference):
 *   Init:
 *     TX 0x9700 — Program Auto-2 register, last channel = CH7
 *     TX 0x3800 — Enter Auto-2 mode, reset counter to CH0, Range 1 (0..VREF)
 *     TX 0x0000 — Flush frame 1 (pipeline)
 *     TX 0x0000 — Flush frame 2 (pipeline)
 *   Steady state — per sample period:
 *     TX 0x0000 once    → discard (first frame of burst is stale)
 *     TX 0x0000 × 8     → read CH0..CH7, routed by response-nibble ID
 *   Response word:
 *     [15:12] = channel ID
 *     [11:0]  = 12-bit result (0..4095)
 *
 * Task allocation notes:
 *   - Use xTaskCreate (not STATIC_MEM_TASK_ALLOC) so the task stack lives
 *     in SRAM, not CCM RAM. deck_spi.c's spiExchange uses DMA which cannot
 *     reach CCM. Our stack-local tx/rx buffers inherit the task stack's
 *     memory region, so they must be in DMA-reachable memory.
 *   - We do NOT call spiBegin() ourselves. The flow deck's init runs
 *     before ours (deck 0 before deck 1) and its pmw3901Init calls
 *     spiBegin(). If pdDeck is ever run without the flow deck, spiBegin()
 *     would need to be called — either from here or from the task.
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

/* ── Configuration ───────────────────────────────────────────────────────── */

#define ADS7953_SPI_BAUDRATE    SPI_BAUDRATE_2MHZ
#define PD_CS_PIN               DECK_GPIO_IO4
#define PD_SAMPLE_RATE_HZ       200
#define ADS7953_INTERFRAME_US   5

#define CMD_PROGRAM_AUTO2       0x9700U
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
    spiExchange(2, tx, rx);        /* single 16-bit burst, no inter-byte gap */
    digitalWrite(PD_CS_PIN, HIGH);

    spiEndTransaction();

    sleepus(ADS7953_INTERFRAME_US);

    return ((uint16_t)rx[0] << 8) | rx[1];
}

/* ── Read one full sweep of 8 channels in Auto-2 mode ───────────────────── */

static void adsReadAllChannels(uint16_t out[PD_CHANNEL_COUNT])
{
    /* First frame of each burst is stale from the previous burst — discard. */
    adsTransfer(CMD_CONTINUE);

    /* Read 8 frames. Route by channel-ID nibble in bits [15:12] so any
     * out-of-order returns still land in the right slot. */
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

    /* Let any flow-deck / PMW3901 startup traffic settle. */
    vTaskDelay(M2T(50));

    /* ADS7953 Auto-2 init sequence. */
    adsTransfer(CMD_PROGRAM_AUTO2);
    adsTransfer(CMD_AUTO2_START);
    adsTransfer(CMD_CONTINUE);
    adsTransfer(CMD_CONTINUE);

    pdReady = true;
    DEBUG_PRINT("PD deck: ready @ %d Hz (Auto-2, shared SPI bus)\n", PD_SAMPLE_RATE_HZ);

    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        uint16_t buf[PD_CHANNEL_COUNT] = { 0 };
        adsReadAllChannels(buf);

        if (xSemaphoreTake(pdMutex, M2T(2)) == pdTRUE) {
            memcpy(pdValues, buf, sizeof(pdValues));
            xSemaphoreGive(pdMutex);
        }

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

    /* CS idle high — ADS7953 is active-low. */
    pinMode(PD_CS_PIN, OUTPUT);
    digitalWrite(PD_CS_PIN, HIGH);

    /* xTaskCreate (not STATIC_MEM_TASK_ALLOC) so the stack lives in SRAM,
     * reachable by the DMA that deck_spi.c uses. */
    xTaskCreate(pdTask, "pdTask", PD_TASK_STACKSIZE, NULL, PD_TASK_PRIORITY, NULL);

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