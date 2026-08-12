/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Vishay VCNL3020 IR proximity sensor - I2C, INTERRUPT-DRIVEN. See
 * vcnl3020.h for the calling contract and wiring notes.
 *
 * ROOT CAUSE OF THE EARLIER ~95ms INTERRUPT STORM (this file's own prior
 * version) - found by building an isolated standalone test (demo/
 * vcnl3020_test_demo, no WLAN/MQTT/DTIM at all) with the SAME sensor and
 * the SAME GPIO3 wiring, which came up completely clean: silent at idle,
 * exactly one STATUS transition per real touch/release, silent again with
 * /INT physically unplugged. Two things were different between that
 * working version and this file's prior one, and both are now fixed here:
 *
 *   1. THE ACTUAL CULPRIT: this file used to call vcnl3020_i2c_reopen()
 *      (qapi_I2CM_Close() + qapi_I2CM_Open()) before EVERY SINGLE register
 *      access - a workaround carried over from this SDK's SHT40 sensor
 *      code for a real, previously-observed issue (the I2C peripheral
 *      going stale after a long WLAN BMPS/DTIM10 idle period). Applied
 *      unconditionally on every interrupt-servicing pass, though, it
 *      means every GPIO3 edge - including a spurious one - immediately
 *      triggered an I2C peripheral close+reopen, which itself is a
 *      plausible source of brief SDA/SCL-adjacent electrical transients.
 *      This lines up with every piece of prior evidence: the original
 *      storm's cadence tracked I2C activity, not any plausible touch
 *      cadence; a later calibration test's rapid on-demand I2C polling
 *      produced near-identical interrupt-count-per-I2C-transaction
 *      bursts; and the storm went completely silent the instant that
 *      polling stopped. In short: a chatter -> reopen -> more chatter
 *      feedback loop, self-inflicted by this file, not the sensor or the
 *      GPIO3 wiring. Fixed here: I2C is opened ONCE in vcnl3020_init(),
 *      never preemptively reopened - vcnl3020_i2c_reopen() now only runs
 *      as a one-shot recovery retry on an actual I2C failure (preserving
 *      the legitimate long-idle-BMPS recovery case without the
 *      per-access churn).
 *   2. Unnecessary complexity: this file also used to dynamically re-arm
 *      a single "live" threshold boundary per transition (Schmitt-trigger
 *      style, see the old vcnl3020_arm_next_transition()). The working
 *      standalone demo does none of that - static LOW=0/HIGH=
 *      VCNL3020_THRESHOLD_HIGH, written once, with the host deciding
 *      CLOSED/OPEN by comparing the raw proximity value against
 *      VCNL3020_THRESHOLD_HIGH directly (not from which Interrupt Status
 *      Register bit fired). Simpler, and now proven correct on this exact
 *      hardware/threshold - ported as-is.
 *
 * Architecture otherwise unchanged: the sensor free-runs self-timed
 * proximity measurements in the background (VCNL3020_PROX_RATE_SELECT)
 * and compares each result against the threshold registers entirely in
 * its own hardware, asserting its open-drain /INT pin only when a
 * measurement crosses VCNL3020_THRESHOLD_HIGH. The host does zero polling
 * of the sensor - vcnl3020_gpio_isr() below just notifies the owning task
 * (see vcnl3020_init()) and returns; that task calls
 * vcnl3020_service_interrupt() to do the actual I2C work, entirely
 * outside of interrupt context.
 *
 * Register map (Vishay datasheet: https://www.vishay.com/docs/84150/
 * vcnl3020.pdf):
 *
 *   0x80  Command Register (#0)             - selftimed_en/prox_en control bits
 *   0x82  Proximity Rate Register (#2)      - self-timed measurement rate
 *   0x83  IR LED Current Register (#3)      - drive current, 10 mA/step, 0-20 (0-200 mA)
 *   0x87  Proximity Result MSB (#7)  \_ 16-bit result, MSB-first
 *   0x88  Proximity Result LSB (#8)  /
 *   0x89  Interrupt Control Register (#9)   - threshold-interrupt enable + debounce
 *   0x8A  Low Threshold, high byte (#10) \_ 16-bit, MSB-first
 *   0x8B  Low Threshold, low byte (#11)  /
 *   0x8C  High Threshold, high byte (#12) \_ 16-bit, MSB-first
 *   0x8D  High Threshold, low byte (#13)  /
 *   0x8E  Interrupt Status Register (#14)   - read to see which threshold
 *                                              fired, write 1 to the same
 *                                              bit(s) to clear/acknowledge
 *                                              (still done here to release
 *                                              /INT, just not used to
 *                                              decide CLOSED/OPEN anymore)
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "qapi_gpio.h"

#include "vcnl3020.h"

/* Fixed 7-bit I2C slave address for every VCNL3020 - no address strap. */
#define VCNL3020_I2C_ADDR              0x13

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

/* Command Register (0x80) bits. */
#define VCNL3020_CMD_SELFTIMED_EN_BIT  (1U << 0)  /* write 1: enable the state machine/LP oscillator for autonomous measurement */
#define VCNL3020_CMD_PROX_EN_BIT       (1U << 1)  /* write 1: enable periodic proximity measurement (used with selftimed_en) */

/* Interrupt Control Register (0x89) bits/fields. */
#define VCNL3020_ICR_THRES_EN_BIT      (1U << 1)  /* write 1: enable threshold-crossing interrupt generation */
#define VCNL3020_ICR_COUNT_EXCEED_SHIFT 5U        /* bits 7:5: consecutive-out-of-window samples required before /INT asserts, see VCNL3020_INT_COUNT_EXCEED */

/* Interrupt Status Register (0x8E) bits - read+cleared to release /INT,
 * no longer used to decide CLOSED/OPEN (see this file's top comment). */
#define VCNL3020_ISR_TH_HI_BIT         (1U << 0)
#define VCNL3020_ISR_TH_LOW_BIT        (1U << 1)

#define info_printf(msg, ...) printf("VCNL3020: " msg, ##__VA_ARGS__)

static TaskHandle_t s_notify_task;

/* ------------------------------------------------------------------ */
/*  Low-level register access                                          */
/* ------------------------------------------------------------------ */

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
    /* Two separate START+STOP transfers (address-write, then data-read)
     * rather than one repeated-start transfer - matches the pattern
     * already proven against this SDK's I2C driver/instance. */
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

static int vcnl3020_read_result_once(uint16_t *out_proximity)
{
    uint8_t msb, lsb;

    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_MSB, &msb) != 0 ||
        vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_LSB, &lsb) != 0)
        return -1;

    *out_proximity = ((uint16_t)msb << 8) | lsb;
    return 0;
}

/* Close+reopen the host I2C instance. Host-controller-side workaround for
 * a REAL, previously-observed issue (I2C peripheral state going stale
 * after a long WLAN BMPS/DTIM10 idle period - same root cause documented
 * for this SDK's SHT40 sensor code). Deliberately NOT called before every
 * access anymore - see this file's top comment for why that was actually
 * the cause of the interrupt storm this file used to have. Now only
 * invoked as a one-shot recovery retry when an access has already failed
 * once (see vcnl3020_i2c_retry() below) - never preemptively. */
static int vcnl3020_i2c_reopen(void)
{
    /* qbool_t: 1 = Blocking mode / DMA on, 0 = Nonblocking / FIFO - using
     * the literal values (rather than TRUE/FALSE) avoids depending on
     * whichever transitive header happens to define those macros; this
     * file's include set doesn't pull one in (confirmed by an actual
     * build failure, "'TRUE' undeclared", earlier in this file's history
     * when it used TRUE/FALSE here). */
    qapi_I2CM_Config_t cfg = { .Blocking = 1, .Dma = 0 };
    qapi_Status_t status;

    qapi_I2CM_Close(VCNL3020_I2C_INSTANCE);
    status = qapi_I2CM_Open(VCNL3020_I2C_INSTANCE, &cfg);
    if (status != QAPI_OK) {
        info_printf("I2C open failed, status=%d\n", (int)status);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  GPIO interrupt (VCNL3020 /INT)                                     */
/* ------------------------------------------------------------------ */

/* Runs in GPIO interrupt-controller context - bare minimum only (no I2C,
 * no printf).
 *
 * EDGE_FALLING, not level-triggered, and that choice is load-bearing:
 * the VCNL3020's /INT stays asserted low until the owning task clears
 * VCNL3020_REG_ISR over I2C, and a level trigger left enabled during that
 * window would re-fire the instant this ISR returns - forever, without
 * ever letting the notified task run (an interrupt-storm lockup,
 * reproduced on hardware by this SDK's contact_sensor_task.c during its
 * own development). Masking the level interrupt here via
 * qapi_GPIO_Disable_Interrupt() is illegal from actual ISR context on
 * this RTOS port (that API takes a critical section FreeRTOS's ARM_CM4F
 * port asserts on outside task context - also reproduced on hardware).
 * EDGE_FALLING sidesteps the problem entirely: it fires exactly once per
 * high-to-low transition regardless of how long the line then stays low,
 * so nothing needs masking/unmasking - the interrupt just stays enabled
 * permanently after the one qapi_GPIO_Enable_Interrupt() call in
 * vcnl3020_init(). */
static void vcnl3020_gpio_isr(qapi_GPIO_CB_Data_t data)
{
    BaseType_t woken = pdFALSE;

    (void)data;

    if (s_notify_task != NULL) {
        xTaskNotifyFromISR(s_notify_task, VCNL3020_NOTIFY_BIT, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int vcnl3020_init(void)
{
    qapi_GPIO_Config_t gpio_cfg;
    uint8_t led_reg = (uint8_t)(VCNL3020_LED_CURRENT_MA / 10U);
    uint8_t icr = VCNL3020_ICR_THRES_EN_BIT |
                  (uint8_t)(VCNL3020_INT_COUNT_EXCEED << VCNL3020_ICR_COUNT_EXCEED_SHIFT);

    /* Whichever task calls this owns servicing VCNL3020_NOTIFY_BIT from
     * here on - see vcnl3020.h. Captured before anything below can
     * possibly race a real interrupt. */
    s_notify_task = xTaskGetCurrentTaskHandle();

    /* Opened ONCE here - not reopened before every access anymore, see
     * this file's top comment. */
    if (vcnl3020_i2c_reopen() != 0)
        return -1;

    /* Register 0x83: IR LED drive current, 10 mA per LSB. */
    if (vcnl3020_write_reg(VCNL3020_REG_IR_LED_CURRENT, led_reg) != 0) {
        info_printf("LED current write failed\n");
        return -1;
    }

    /* Register 0x82: self-timed measurement rate. */
    if (vcnl3020_write_reg(VCNL3020_REG_PROXIMITY_RATE, VCNL3020_PROX_RATE_SELECT) != 0) {
        info_printf("proximity rate write failed\n");
        return -1;
    }

    /* Registers 0x8A-0x8D: static LOW=VCNL3020_THRESHOLD_LOW(0)/
     * HIGH=VCNL3020_THRESHOLD_HIGH - written once, never re-armed. See
     * this file's top comment for why the old dynamic re-arming scheme
     * was dropped. */
    if (vcnl3020_write_reg(VCNL3020_REG_LOW_THR_HI, (uint8_t)(VCNL3020_THRESHOLD_LOW >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_LOW_THR_LO, (uint8_t)(VCNL3020_THRESHOLD_LOW & 0xFF)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_HI, (uint8_t)(VCNL3020_THRESHOLD_HIGH >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_LO, (uint8_t)(VCNL3020_THRESHOLD_HIGH & 0xFF)) != 0) {
        info_printf("threshold write failed\n");
        return -1;
    }

    /* Register 0x89: threshold interrupt enabled, debounced by
     * VCNL3020_INT_COUNT_EXCEED consecutive out-of-window samples,
     * INT_PROX_READY_EN deliberately left OFF (bit not set) - no
     * interrupt on every single measurement, threshold crossings only. */
    if (vcnl3020_write_reg(VCNL3020_REG_ICR, icr) != 0) {
        info_printf("interrupt control write failed\n");
        return -1;
    }

    /* Register 0x80: selftimed_en MUST be set before prox_en per the
     * datasheet, hence two separate writes rather than one combined one. */
    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND, VCNL3020_CMD_SELFTIMED_EN_BIT) != 0) {
        info_printf("selftimed_en write failed\n");
        return -1;
    }
    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND,
            VCNL3020_CMD_SELFTIMED_EN_BIT | VCNL3020_CMD_PROX_EN_BIT) != 0) {
        info_printf("prox_en write failed\n");
        return -1;
    }

    /* qapi_GPIO_Init() is declared in qapi_gpio.h but is NOT linked into
     * this image (confirmed by contact_sensor_task.c: an actual link
     * failure, "undefined reference to qapi_GPIO_Init", when it called
     * it) - board bring-up already handles whatever it would have done.
     * Do not call it. GPIO3 registered LAST, after selftimed_en/prox_en
     * are already set - matches the proven-working standalone demo's
     * ordering. */
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.Dir = QAPI_GPIO_INPUT_E;
    gpio_cfg.Pull = QAPI_GPIO_PULL_UP_E; /* backup for the recommended external pull-up - see VCNL3020_INT_GPIO_ID's comment */
    gpio_cfg.Drive = QAPI_GPIO_DRIVE_LOW_E; /* irrelevant for an input, kept in a defined state */
    qapi_GPIO_Config(VCNL3020_INT_GPIO_ID, &gpio_cfg);

    qapi_GPIO_Enable_Interrupt(VCNL3020_INT_GPIO_ID, QAPI_GPIO_TRIGGER_EDGE_FALLING_E, vcnl3020_gpio_isr, 0);

    info_printf("armed: self-timed + threshold interrupt on GPIO%d, HIGH=%u, LED=%umA\n",
        (int)VCNL3020_INT_GPIO_ID, (unsigned)VCNL3020_THRESHOLD_HIGH, (unsigned)VCNL3020_LED_CURRENT_MA);

    return 0;
}

int vcnl3020_service_interrupt(uint16_t *out_proximity)
{
    uint8_t isr = 0;

    /* Read + clear the Interrupt Status Register - releases the
     * open-drain /INT line so future interrupts can fire. Content not
     * used to decide CLOSED/OPEN anymore - see this file's top comment. */
    if (vcnl3020_read_reg(VCNL3020_REG_ISR, &isr) != 0) {
        if (vcnl3020_i2c_reopen() != 0)
            return -1;
        if (vcnl3020_read_reg(VCNL3020_REG_ISR, &isr) != 0)
            return -1;
    }

    isr &= (VCNL3020_ISR_TH_HI_BIT | VCNL3020_ISR_TH_LOW_BIT);
    if (isr != 0 && vcnl3020_write_reg(VCNL3020_REG_ISR, isr) != 0)
        return -1;

    if (vcnl3020_read_result_once(out_proximity) != 0) {
        if (vcnl3020_i2c_reopen() != 0)
            return -1;
        if (vcnl3020_read_result_once(out_proximity) != 0)
            return -1;
    }

    return 0;
}

int vcnl3020_read_result(uint16_t *out_proximity)
{
    if (vcnl3020_read_result_once(out_proximity) == 0)
        return 0;

    /* One recovery attempt - the I2C peripheral may have gone stale after
     * a long BMPS/DTIM10 idle gap (the periodic MQTT-tick caller of this
     * function can go up to MQTT_PUBLISH_INTERVAL_MS, ~10 min, between
     * calls). NOT done preemptively - see this file's top comment. */
    if (vcnl3020_i2c_reopen() != 0)
        return -1;

    return vcnl3020_read_result_once(out_proximity);
}
