/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Standalone VCNL3020 bring-up test - INTERRUPT-DRIVEN on GPIO3, for
 * manually verifying the /INT wiring itself.
 *
 * Wiring: SDA, SCL, VDD, GND, PLUS /INT -> GPIO3 (open-drain, active LOW -
 * external ~10k pull-up to 3V3 recommended; the SoC's own internal pull-up
 * is also enabled below as a backup only).
 *
 * On every GPIO3 falling edge: read + clear the Interrupt Status Register
 * (0x8E, releasing /INT so future interrupts can fire), then read the
 * current proximity result (0x87/0x88) and compare it against
 * HIGH_THRESHOLD to decide CLOSED/OPEN - not from which 0x8E bit fired, so
 * this stays correct even if debounce/threshold register config changes
 * later. No periodic polling anywhere in this file - every print is
 * directly caused by a real GPIO3 edge.
 *
 * Prints exactly one thing, only when the decided status CHANGES from the
 * last one printed:
 *   VCNL3020: STATUS: OPEN
 *   VCNL3020: STATUS: CLOSED
 *
 * This is deliberately the signal for manually judging the wiring itself:
 *   - idle, nothing touching the sensor -> silent (no interrupts at all)
 *   - touch/cover -> exactly one "STATUS: CLOSED"
 *   - release -> exactly one "STATUS: OPEN"
 *   - repeated/rapid STATUS lines with nothing touched -> GPIO3/the /INT
 *     line itself is electrically unstable (floating, missing/weak
 *     pull-up, bad connection) - that is exactly what this build exists
 *     to expose, not a bug in the logic below.
 *
 * TUNE ME - HIGH_THRESHOLD: proximity >= this -> CLOSED, else OPEN, AND
 * this exact value is written into the sensor's own High Threshold
 * register (0x8C/0x8D) so /INT only asserts on a real crossing. Low
 * threshold is pinned to 0 (unreachable - proximity is unsigned) and never
 * used; the sensor's interrupt comparator needs some low-threshold value
 * programmed, this just keeps that side permanently inert. Calibrated from
 * real hardware data (2026-08-12): idle baseline measured rock-steady at
 * ~2140-2160 (about 20 counts of noise), and a hand near the sensor read
 * ~4857 rising to ~15000+ - a huge, clean gap. 3000 sits comfortably in the
 * middle. Adjust if your own testing shows it should be more/less
 * sensitive.
 *
 * TEMPORARY DIAGNOSTIC (added 2026-08-12, VCNL3020_RAW_DEBUG_ENABLE below) -
 * this project's STATUS lines were staying completely silent after boot
 * even when touching the sensor, unlike an earlier isolated test build that
 * printed clean RAW baselines and correct transitions. To tell apart "I2C/
 * sensor communication itself is broken in this project" from "I2C is fine,
 * GPIO3 interrupts specifically aren't firing/being serviced", this adds
 * back an on-demand RAW proximity print once a second, running ALONGSIDE
 * the interrupt-driven STATUS logic below (same task, same I2C bus, just
 * time-sliced against the interrupt wait - no separate task, no removal of
 * any interrupt code). Set VCNL3020_RAW_DEBUG_ENABLE to 0 once the GPIO3
 * issue is root-caused. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "qapi_gpio.h"

#define VCNL3020_I2C_ADDR              0x13
#define VCNL3020_I2C_INSTANCE          QAPI_I2C_INSTANCE_SE0_E

#define VCNL3020_REG_COMMAND           0x80
#define VCNL3020_REG_PROXIMITY_RATE    0x82
#define VCNL3020_REG_IR_LED_CURRENT    0x83
#define VCNL3020_REG_PROX_RESULT_MSB   0x87
#define VCNL3020_REG_PROX_RESULT_LSB   0x88
#define VCNL3020_REG_ICR               0x89
#define VCNL3020_REG_LOW_THR_HI        0x8A
#define VCNL3020_REG_LOW_THR_LO        0x8B
#define VCNL3020_REG_HIGH_THR_HI       0x8C
#define VCNL3020_REG_HIGH_THR_LO       0x8D
#define VCNL3020_REG_ISR               0x8E

#define VCNL3020_CMD_SELFTIMED_EN_BIT  (1U << 0)
#define VCNL3020_CMD_PROX_EN_BIT       (1U << 1)

/* Interrupt Control Register (0x89) bits/fields. */
#define VCNL3020_ICR_THRES_EN_BIT      (1U << 1)  /* write 1: enable threshold-crossing interrupt generation */
#define VCNL3020_ICR_COUNT_EXCEED_SHIFT 5U         /* bits 7:5: consecutive out-of-window samples before /INT asserts */
#define VCNL3020_INT_COUNT_EXCEED      1U          /* raw field value 1 -> 2 consecutive samples (hardware debounce) */

/* Interrupt Status Register (0x8E) bits - read+cleared to release /INT. */
#define VCNL3020_ISR_TH_HI_BIT         (1U << 0)
#define VCNL3020_ISR_TH_LOW_BIT        (1U << 1)

#define VCNL3020_LED_CURRENT_MA        20U  /* register 0x83, 10 mA/step */
#define VCNL3020_PROX_RATE_SELECT      0U   /* 1.95 measurements/s */

/* See this file's top comment for how this value was picked. */
#define HIGH_THRESHOLD                 3000U
#define LOW_THRESHOLD                  0U

/* /INT wiring under test - see this file's top comment. */
#define VCNL3020_INT_GPIO_ID           QAPI_GPIO_ID3_E

/* xTaskNotify() bit set from vcnl3020_gpio_isr(). */
#define VCNL3020_NOTIFY_BIT            (1U << 0)

/* TEMPORARY diagnostic - see this file's top comment. 1 = also print
 * "RAW: NNNN" once a second from an on-demand I2C read, independent of
 * whether GPIO3 interrupts are firing. Set to 0 to go back to purely
 * interrupt-driven/silent-until-a-real-edge behavior. */
#define VCNL3020_RAW_DEBUG_ENABLE      1
#define VCNL3020_RAW_DEBUG_PERIOD_MS   1000U

#define info_printf(msg, ...) printf("VCNL3020: " msg, ##__VA_ARGS__)

static TaskHandle_t s_task_handle;

static int vcnl3020_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    qapi_I2CM_Transfer_Config_t xcfg = {
        .BusFreqKHz = 100,
        .SlaveAddress = VCNL3020_I2C_ADDR,
        .SlaveMaxClockStretchUs = 0,
        .Delay = 0,
        .NoiseReject = 0,
    };
    qapi_I2CM_Descriptor_t wr = {
        .Buffer = buf,
        .Length = 2,
        .Transferred = 0,
        .Flags = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_WRITE,
    };

    return (qapi_I2CM_Transfer(VCNL3020_I2C_INSTANCE, &xcfg, &wr, 1, NULL, NULL) == QAPI_OK) ? 0 : -1;
}

static int vcnl3020_read_reg(uint8_t reg, uint8_t *val)
{
    qapi_I2CM_Transfer_Config_t xcfg = {
        .BusFreqKHz = 100,
        .SlaveAddress = VCNL3020_I2C_ADDR,
        .SlaveMaxClockStretchUs = 0,
        .Delay = 0,
        .NoiseReject = 0,
    };
    qapi_I2CM_Descriptor_t wr = {
        .Buffer = &reg,
        .Length = 1,
        .Transferred = 0,
        .Flags = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_WRITE,
    };
    qapi_I2CM_Descriptor_t rd = {
        .Buffer = val,
        .Length = 1,
        .Transferred = 0,
        .Flags = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_READ,
    };

    if (qapi_I2CM_Transfer(VCNL3020_I2C_INSTANCE, &xcfg, &wr, 1, NULL, NULL) != QAPI_OK)
        return -1;
    if (qapi_I2CM_Transfer(VCNL3020_I2C_INSTANCE, &xcfg, &rd, 1, NULL, NULL) != QAPI_OK)
        return -1;
    return 0;
}

static int vcnl3020_read_result(uint16_t *out_proximity)
{
    uint8_t msb, lsb;

    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_MSB, &msb) != 0 ||
        vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_LSB, &lsb) != 0)
        return -1;

    *out_proximity = ((uint16_t)msb << 8) | lsb;
    return 0;
}

/* Runs in GPIO interrupt-controller context - bare minimum only (no I2C,
 * no printf).
 *
 * EDGE_FALLING, not level-triggered, and that choice is load-bearing: the
 * VCNL3020's /INT stays asserted low until the owning task clears
 * VCNL3020_REG_ISR over I2C, and a level trigger left enabled during that
 * window would re-fire the instant this ISR returns - forever, without
 * ever letting the notified task run. EDGE_FALLING sidesteps that: it
 * fires exactly once per high-to-low transition regardless of how long the
 * line then stays low, so nothing needs masking/unmasking - the interrupt
 * just stays enabled permanently after the one
 * qapi_GPIO_Enable_Interrupt() call below. */
static void vcnl3020_gpio_isr(qapi_GPIO_CB_Data_t data)
{
    BaseType_t woken = pdFALSE;

    (void)data;

    if (s_task_handle != NULL) {
        xTaskNotifyFromISR(s_task_handle, VCNL3020_NOTIFY_BIT, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* I2C open + LED current + proximity rate + static threshold registers +
 * interrupt control + self-timed enable + GPIO3 interrupt arm. */
static int vcnl3020_configure(void)
{
    qapi_I2CM_Config_t i2c_cfg = { .Blocking = 1, .Dma = 0 };
    qapi_GPIO_Config_t gpio_cfg;
    uint8_t icr = VCNL3020_ICR_THRES_EN_BIT |
                  (uint8_t)(VCNL3020_INT_COUNT_EXCEED << VCNL3020_ICR_COUNT_EXCEED_SHIFT);

    if (qapi_I2CM_Open(VCNL3020_I2C_INSTANCE, &i2c_cfg) != QAPI_OK) {
        info_printf("I2C open failed\n");
        return -1;
    }

    if (vcnl3020_write_reg(VCNL3020_REG_IR_LED_CURRENT, (uint8_t)(VCNL3020_LED_CURRENT_MA / 10U)) != 0) {
        info_printf("LED current write failed\n");
        return -1;
    }
    if (vcnl3020_write_reg(VCNL3020_REG_PROXIMITY_RATE, VCNL3020_PROX_RATE_SELECT) != 0) {
        info_printf("proximity rate write failed\n");
        return -1;
    }

    /* Registers 0x8A-0x8D: static LOW=0/HIGH=HIGH_THRESHOLD, written once,
     * never re-armed. */
    if (vcnl3020_write_reg(VCNL3020_REG_LOW_THR_HI, (uint8_t)(LOW_THRESHOLD >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_LOW_THR_LO, (uint8_t)(LOW_THRESHOLD & 0xFF)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_HI, (uint8_t)(HIGH_THRESHOLD >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_LO, (uint8_t)(HIGH_THRESHOLD & 0xFF)) != 0) {
        info_printf("threshold write failed\n");
        return -1;
    }

    /* Register 0x89: threshold interrupt enabled, debounced by
     * VCNL3020_INT_COUNT_EXCEED consecutive out-of-window samples,
     * INT_PROX_READY_EN deliberately left OFF - no interrupt on every
     * single measurement, threshold crossings only. */
    if (vcnl3020_write_reg(VCNL3020_REG_ICR, icr) != 0) {
        info_printf("interrupt control write failed\n");
        return -1;
    }

    /* selftimed_en MUST be set before prox_en per the datasheet, hence two
     * separate writes. */
    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND, VCNL3020_CMD_SELFTIMED_EN_BIT) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_COMMAND,
            VCNL3020_CMD_SELFTIMED_EN_BIT | VCNL3020_CMD_PROX_EN_BIT) != 0) {
        info_printf("self-timed enable failed\n");
        return -1;
    }

    /* GPIO3 armed LAST, after selftimed_en/prox_en are already set. */
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.Dir = QAPI_GPIO_INPUT_E;
    gpio_cfg.Pull = QAPI_GPIO_PULL_UP_E; /* backup for the recommended external pull-up - see this file's top comment */
    gpio_cfg.Drive = QAPI_GPIO_DRIVE_LOW_E; /* irrelevant for an input, kept in a defined state */
    qapi_GPIO_Config(VCNL3020_INT_GPIO_ID, &gpio_cfg);

    qapi_GPIO_Enable_Interrupt(VCNL3020_INT_GPIO_ID, QAPI_GPIO_TRIGGER_EDGE_FALLING_E, vcnl3020_gpio_isr, 0);

    info_printf("armed: self-timed + threshold interrupt on GPIO%d, HIGH=%u, LED=%umA\n",
        (int)VCNL3020_INT_GPIO_ID, (unsigned)HIGH_THRESHOLD, (unsigned)VCNL3020_LED_CURRENT_MA);
    return 0;
}

static void vcnl3020_test_task(void *arg)
{
    uint8_t last_state = 0xFFU; /* neither OPEN(0) nor CLOSED(1) - forces the first real transition to print */

    (void)arg;

    /* s_task_handle must be valid before qapi_GPIO_Enable_Interrupt() runs
     * inside vcnl3020_configure() - the ISR notifies it. */
    s_task_handle = xTaskGetCurrentTaskHandle();

    while (vcnl3020_configure() != 0) {
        info_printf("retrying in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* Interrupt-driven STATUS logic below is unchanged from before - every
     * STATUS line is still caused by a real GPIO3 falling edge (or genuine
     * electrical chatter on that line). The only addition is the
     * VCNL3020_RAW_DEBUG_ENABLE branch: when the wait below times out
     * (i.e. no interrupt notification arrived within
     * VCNL3020_RAW_DEBUG_PERIOD_MS), it does a plain on-demand I2C read and
     * prints RAW - see this file's top comment. With
     * VCNL3020_RAW_DEBUG_ENABLE set to 0 the wait goes back to
     * portMAX_DELAY and this is purely interrupt-driven/silent again. */
    for (;;) {
        uint32_t notified;
        uint8_t isr = 0;
        uint16_t proximity;

#if VCNL3020_RAW_DEBUG_ENABLE
        BaseType_t got_notify = xTaskNotifyWait(0, VCNL3020_NOTIFY_BIT, &notified,
            pdMS_TO_TICKS(VCNL3020_RAW_DEBUG_PERIOD_MS));
        if (got_notify == pdFALSE) {
            /* Timed out - no GPIO3 interrupt notification arrived. Plain
             * on-demand I2C read, completely independent of the ISR/notify
             * path above and below. */
            uint16_t raw;

            if (vcnl3020_read_result(&raw) == 0)
                info_printf("RAW: %u\n", (unsigned)raw);
            continue;
        }
#else
        xTaskNotifyWait(0, VCNL3020_NOTIFY_BIT, &notified, portMAX_DELAY);
#endif

        /* Read + clear the Interrupt Status Register - releases the
         * open-drain /INT line so future interrupts can fire. */
        if (vcnl3020_read_reg(VCNL3020_REG_ISR, &isr) != 0)
            continue;
        isr &= (VCNL3020_ISR_TH_HI_BIT | VCNL3020_ISR_TH_LOW_BIT);
        if (isr != 0 && vcnl3020_write_reg(VCNL3020_REG_ISR, isr) != 0)
            continue;

        if (vcnl3020_read_result(&proximity) == 0) {
            uint8_t state = (proximity >= HIGH_THRESHOLD) ? 1U : 0U;

            if (state != last_state) {
                last_state = state;
                info_printf("STATUS: %s\n", state ? "CLOSED" : "OPEN");
            }
        }
    }
}

void vcnl3020_test_start(void)
{
    if (s_task_handle != NULL)
        return;

    if (nt_qurt_thread_create(vcnl3020_test_task, "vcnl3020_test", 2048, NULL, 5, &s_task_handle) != pdPASS) {
        info_printf("task create failed\n");
    }
}