/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* VCNL3020 bring-up test - INTERRUPT-DRIVEN on GPIO3, for manually
 * verifying the /INT wiring itself, now also publishing every STATUS
 * transition to MQTT (see wifi_mqtt.c/.h - same broker/WLAN config as
 * demo/powertest_demo's contact_sensor_task.c/mqtt_printf_task.c). This
 * file's own GPIO3/I2C/threshold logic is untouched by that addition - the
 * only new line is the vcnl3020_mqtt_publish_status() call right at the
 * existing STATUS-print site below, which just queues the event for
 * wifi_mqtt.c's own task (non-blocking, safe to call before WLAN/MQTT are
 * even up).
 *
 * BMPS I2C-reopen retry (added 2026-08-14, see vcnl3020_i2c_reopen()/
 * vcnl3020_service_interrupt()) - once wifi_mqtt.c's DTIM10/BMPS engages,
 * the host I2C peripheral can go stale after an idle gap (same issue
 * already documented/fixed for this sensor in demo/powertest_demo/src/
 * vcnl3020.c); without a retry, a GPIO3 edge landing on a failed I2C access
 * would silently drop the event AND leave the sensor's Interrupt Status
 * Register uncleared, latching /INT low and killing all future
 * STATUS/MQTT updates for good (the EDGE_FALLING trigger can never re-fire
 * on a line that never goes back high).
 *
 * Behavior under BMPS is otherwise unchanged: a real STATUS transition (or
 * the MQTT heartbeat) is just a normal socket send, which wakes the radio
 * briefly and returns to the DTIM10 duty cycle on its own - no action
 * needed here for that part.
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

#include "wifi_mqtt.h"

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

static int vcnl3020_read_result_once(uint16_t *out_proximity)
{
    uint8_t msb, lsb;

    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_MSB, &msb) != 0 ||
        vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_LSB, &lsb) != 0)
        return -1;

    *out_proximity = ((uint16_t)msb << 8) | lsb;
    return 0;
}

/* Close+reopen the host I2C MASTER instance - purely a host-side driver
 * reset, it does NOT touch the VCNL3020's own internal state (self-timed
 * measurement/threshold comparison keeps running on the sensor
 * regardless). Workaround for a real, previously-observed issue: the
 * QCC730 I2C host peripheral's state goes stale after a WLAN BMPS/DTIM10
 * idle period (same root cause already documented/fixed for this same
 * sensor in demo/powertest_demo/src/vcnl3020.c and for the SHT40 sensor
 * elsewhere in this SDK). Only invoked as a one-shot recovery retry when an
 * access has already failed once (see vcnl3020_service_interrupt() below) -
 * never preemptively, so it costs nothing while WLAN/BMPS isn't involved
 * (e.g. the standalone build with no WLAN at all). */
static int vcnl3020_i2c_reopen(void)
{
    qapi_I2CM_Config_t cfg = { .Blocking = 1, .Dma = 0 };

    qapi_I2CM_Close(VCNL3020_I2C_INSTANCE);
    if (qapi_I2CM_Open(VCNL3020_I2C_INSTANCE, &cfg) != QAPI_OK) {
        info_printf("I2C reopen failed\n");
        return -1;
    }
    return 0;
}

/* Read+clear the Interrupt Status Register (releasing /INT) and read the
 * fresh proximity result - each with ONE reopen-and-retry if the I2C
 * transfer itself fails (see vcnl3020_i2c_reopen()'s comment). Without this
 * retry, an I2C failure right when BMPS engages would silently drop the
 * event AND leave the ISR register uncleared, which latches the open-drain
 * /INT line low forever - since this file arms GPIO3 as EDGE_FALLING (see
 * vcnl3020_gpio_isr()'s comment), a line that never returns high can never
 * produce another falling edge, permanently stopping all future STATUS
 * updates and MQTT publishes. Returns 0 on success, -1 on an unrecoverable
 * I2C failure (caller should just wait for the next event, no sensor
 * reconfigure needed - only the host I2C peripheral state was reset, the
 * sensor's own threshold/self-timed config is untouched). */
static int vcnl3020_service_interrupt(uint16_t *out_proximity)
{
    uint8_t isr = 0;

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

    /* PURE event-driven - no periodic sensor I2C access anywhere in this
     * loop (removed 2026-08-14, previously VCNL3020_POLL_FALLBACK_PERIOD_MS).
     * That poll was added after hardware testing seemed to show GPIO3 edges
     * going unserviced once wifi_mqtt.c's DTIM10/BMPS engaged, but it was
     * itself paying for that: every poll cycle forced a full "wakebmps"
     * radio wake (documented on this exact SoC), which is precisely the
     * per-cycle radio-wake cost DTIM10/BMPS exists to avoid - a timer-driven
     * "backstop" is a contradiction of "signal only on a real state change".
     * This SDK already has a proven, hardware-confirmed precedent for this
     * exact situation (VCNL3020 interrupt-driven contact sensing WITH
     * DTIM10/BMPS engaged, same SoC): demo/powertest_demo/src/vcnl3020.c +
     * mqtt_printf_task.c. There, an apparently similar "interrupt not
     * behaving" symptom was root-caused NOT to a BMPS/GPIO hardware
     * limitation but to periodic/per-access I2C churn itself generating
     * electrical crosstalk near GPIO3 (see vcnl3020.c's top-of-file
     * comment) - i.e. self-inflicted by exactly the kind of periodic I2C
     * poll this loop used to run. The fix there was the same one applied
     * here: zero periodic sensor I2C access, block indefinitely
     * (portMAX_DELAY, no timeout) and let the sensor's own hardware
     * threshold comparator be the only thing that ever wakes this task. If
     * real GPIO3 edges do turn out to still be missed under BMPS on this
     * specific board (a separate, unconfirmed possibility - the "3 tests"
     * that motivated the removed poll ran at a much noisier 2s/5s poll
     * cadence than reproduced), that would show up as touches with no
     * "GPIO3 notified" line at all with this build. */
    for (;;) {
        uint32_t notified;
        uint16_t proximity;

        xTaskNotifyWait(0, VCNL3020_NOTIFY_BIT, &notified, portMAX_DELAY);

        /* Real GPIO3 edge - read + clear the Interrupt Status Register
         * (releases /INT) and read the fresh proximity result, retrying
         * once via an I2C reopen on failure. */
        info_printf("GPIO3 notified\n");

        if (vcnl3020_service_interrupt(&proximity) != 0) {
            info_printf("interrupt service failed (I2C), will retry\n");
            continue;
        }

        {
            uint8_t state = (proximity >= HIGH_THRESHOLD) ? 1U : 0U;

            if (state != last_state) {
                last_state = state;
                info_printf("STATUS: %s (raw=%u, via GPIO3)\n", state ? "CLOSED" : "OPEN",
                    (unsigned)proximity);
                /* Non-blocking - hands off to wifi_mqtt.c's own task, see
                 * vcnl3020_mqtt_publish_status()'s header comment. Queued
                 * even before WLAN/MQTT come up; safe to call unconditionally. */
                vcnl3020_mqtt_publish_status(state);
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