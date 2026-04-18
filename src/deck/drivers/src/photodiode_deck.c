/**
 * photodiode_deck.c — CrazyFlie deck driver for the 8-channel photodiode board
 *
 * ADC:  Texas Instruments ADS7953SRHBT — 12-bit SAR, SPI Mode 0, 16 channels
 * TIA:  OPA4350EA/2K5, ~3.2 kΩ feedback
 * VREF: REF5025AIDR → 2.500 V
 *
 * SPI wiring (SPI1):
 *   SCK  → PA5 (AF5)   MISO → PA6 (AF5)   MOSI → PA7 (AF5)
 *   CS   → PC12 (GPIO output, active-low) — DECK_GPIO_IO4
 *
 * This driver owns SPI1 exclusively — no deck_spi.c, no DMA, no shared mutex.
 * We force Mode 0 (CPOL=0 CPHA=0) at init; the shared bus default is Mode 3.
 *
 * Read sequence (mirrors working Teensy code):
 *   For each channel:
 *     CS pulse 1: send CMD_READ_CH(ch)  → starts conversion, returns prev result (discard)
 *     5 µs delay
 *     CS pulse 2: send CMD_READ_CH(ch)  → returns result for ch
 *   Shift received word right by 1 (ADS7953 SDO is one clock late in Mode 0).
 */

#define DEBUG_MODULE "PDDECK"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "deck.h"
#include "system.h"
#include "log.h"
#include "debug.h"
#include "static_mem.h"
#include "stm32fxxx.h"
#include "sleepus.h"

#include "photodiode_deck.h"

/* ── Hardware constants ──────────────────────────────────────────────────── */

/* BR[2:0]=5 → APB2(84MHz)/64 = 1.3125 MHz — matches Teensy 1MHz test */
#define SPI_BR_DIV64        5u

#define PD_SAMPLE_RATE_HZ   200
#define ADS7953_INTERFRAME_US  5

/* Manual-mode channel select command: [15:12]=0001, [11:8]=channel */
#define ADS7953_CMD_READ_CH(ch)  (0x1000U | (((uint16_t)(ch) & 0x0FU) << 8))

/* CS pin: PC12 — BSRRL sets HIGH, BSRRH sets LOW */
#define PD_CS_HIGH()  (GPIOC->BSRRL = (1u << 12))
#define PD_CS_LOW()   (GPIOC->BSRRH = (1u << 12))

/* ── Shared state ────────────────────────────────────────────────────────── */

static float     pdValues[PD_CHANNEL_COUNT];
static bool      pdReady = false;

static StaticSemaphore_t pdMutexBuf;
static SemaphoreHandle_t pdMutex;

static StaticSemaphore_t pdDataReadyBuf;
static SemaphoreHandle_t pdDataReady;

/* ── Hardware init ───────────────────────────────────────────────────────── */

static void pdHwInit(void)
{
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    /* PA5=SCK, PA6=MISO, PA7=MOSI → AF5 (SPI1) */
    GPIOA->MODER   = (GPIOA->MODER   & ~(0x3Fu << 10)) | (0x2Au << 10);
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFFFu << 20)) | (0x555u << 20);
    GPIOA->OSPEEDR |= (0x3Fu << 10);
    GPIOA->PUPDR   &= ~(0x3Fu << 10);

    /* PC12 = CS → push-pull output, start HIGH (deasserted) */
    GPIOC->MODER   = (GPIOC->MODER   & ~(3u << 24)) | (1u << 24);
    GPIOC->OTYPER  &= ~(1u << 12);
    GPIOC->OSPEEDR |= (3u << 24);
    GPIOC->PUPDR   &= ~(3u << 24);
    PD_CS_HIGH();

    /* SPI1: Master, Mode 3 (CPOL=1 CPHA=1), 8-bit MSB-first, SW-NSS */
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    SPI1->CR1 = SPI_CR1_MSTR
              | (SPI_BR_DIV64 << 3)
              | SPI_CR1_SSM
              | SPI_CR1_SSI
              | SPI_CR1_CPOL
              | SPI_CR1_CPHA;
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* ── Polled SPI byte exchange ────────────────────────────────────────────── */

static void pdSpiExchange(size_t len, const uint8_t *tx, uint8_t *rx)
{
    /* Re-assert Mode 3 every transfer */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 |= (SPI_CR1_CPOL | SPI_CR1_CPHA);
    SPI1->CR1 |= SPI_CR1_SPE;

    for (size_t i = 0; i < len; i++) {
        while (!(SPI1->SR & SPI_SR_TXE));
        SPI1->DR = tx[i];
        while (!(SPI1->SR & SPI_SR_RXNE));
        rx[i] = (uint8_t)SPI1->DR;
    }
    while (SPI1->SR & SPI_SR_BSY);
}

/* ── ADS7953 per-channel read (mirrors Teensy sequence) ─────────────────── */

static void adsReadAllChannels(float out[PD_CHANNEL_COUNT])
{
    for (uint8_t ch = 0; ch < PD_CHANNEL_COUNT; ch++) {
        uint16_t cmd = ADS7953_CMD_READ_CH(ch);
        uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
        uint8_t rx[2];

        /* Pulse 1: send command, discard returned word (pipeline) */
        PD_CS_LOW();
        pdSpiExchange(2, tx, rx);
        PD_CS_HIGH();
        sleepus(ADS7953_INTERFRAME_US);

        /* Pulse 2: send same command, read actual result */
        PD_CS_LOW();
        pdSpiExchange(2, tx, rx);
        PD_CS_HIGH();
        sleepus(ADS7953_INTERFRAME_US);

        uint16_t word = ((uint16_t)rx[0] << 8) | rx[1];
        uint16_t raw  = word & 0x0FFF;
        out[ch] = (float)raw / 4095.0f;
    }
}

/* ── Sampling task ───────────────────────────────────────────────────────── */

#define PD_TASK_STACKSIZE   (4 * configMINIMAL_STACK_SIZE)
#define PD_TASK_PRIORITY    3

STATIC_MEM_TASK_ALLOC(pdTask, PD_TASK_STACKSIZE);

static void pdTask(void *param)
{
    (void)param;
    systemWaitStart();

    /* Prime pipeline — discard first round */
    float discard[PD_CHANNEL_COUNT];
    adsReadAllChannels(discard);

    pdReady = true;
    DEBUG_PRINT("PD deck: ready @ %d Hz\n", PD_SAMPLE_RATE_HZ);

    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        float buf[PD_CHANNEL_COUNT];
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
    ASSERT(pdMutex != NULL);
    ASSERT(pdDataReady != NULL);

    pdHwInit();  /* claim SPI1 and CS pin — no deck_spi.c involvement */

    STATIC_MEM_TASK_CREATE(pdTask, pdTask, "pdTask", NULL, PD_TASK_PRIORITY);
    DEBUG_PRINT("PD deck: init (baremetal SPI1, Mode 0, PC12 CS)\n");
}

static bool pdDeckTest(void) { return true; }

/* ── Public API ──────────────────────────────────────────────────────────── */

bool pdDeckGetValues(float out[PD_CHANNEL_COUNT])
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
    LOG_ADD(LOG_FLOAT, ch0, &pdValues[0])
    LOG_ADD(LOG_FLOAT, ch1, &pdValues[1])
    LOG_ADD(LOG_FLOAT, ch2, &pdValues[2])
    LOG_ADD(LOG_FLOAT, ch3, &pdValues[3])
    LOG_ADD(LOG_FLOAT, ch4, &pdValues[4])
    LOG_ADD(LOG_FLOAT, ch5, &pdValues[5])
    LOG_ADD(LOG_FLOAT, ch6, &pdValues[6])
    LOG_ADD(LOG_FLOAT, ch7, &pdValues[7])
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
