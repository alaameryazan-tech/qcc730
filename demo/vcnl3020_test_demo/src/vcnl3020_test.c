/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* VCNL3020 bring-up test - publishes every STATUS transition to MQTT (see
 * wifi_mqtt.c/.h - same broker/WLAN config as demo/powertest_demo's
 * contact_sensor_task.c/mqtt_printf_task.c).
 *
 * ARCHITECTURE (rewritten 2026-08-17 - GPIO3 removed entirely):
 *
 *   This file no longer owns any GPIO interrupt. All detection is a plain
 *   on-demand I2C read (vcnl3020_poll_and_clear()), triggered from
 *   wifi_mqtt.c via vcnl3020_test_poll_now() at two different times:
 *
 *     1. A plain periodic timer, ONLY while WLAN is connected and BMPS has
 *        not engaged yet (the ~15s full-power window right after connect,
 *        see VCNL3020_BMPS_ENTER_DELAY_MS in wifi_mqtt.c) - the radio isn't
 *        duty-cycling in that window regardless, so a plain poll costs
 *        nothing extra there (unlike polling DURING BMPS, which forces its
 *        own radio wake every cycle - see wifi_mqtt.c's history for why
 *        that was rejected as the steady-state mechanism).
 *     2. wifi_mqtt.c's bmps_wake_probe_cb(), on a real
 *        PWR_EVT_WMAC_POST_AWAKE event - which now fires promptly on a real
 *        touch/release once BMPS is active, because the VCNL3020's /INT is
 *        physically wired to chip pin 22 (EXT_WAKEUP_INTR_N) in addition to
 *        (now former) GPIO3. See wifi_mqtt.c's top-of-file comment for the
 *        full mechanism (init_aon_ext_wakeup_int() / aon_a2f_assert_isr_
 *        handler() / EXIT_REASON_EXT_INT).
 *
 *   Why GPIO3 was removed rather than kept alongside pin 22: GPIO3's
 *   EDGE_FALLING interrupt cannot fire at all while BMPS "Sleep" is active
 *   (confirmed via the QCC730M-1 datasheet's power-mode table - CPU is OFF
 *   in that mode), so it only ever covered the pre-BMPS window - exactly
 *   the window the plain timer poll above now covers instead, without
 *   needing a second physical wire or the GPIO controller at all.
 *
 *   vcnl3020_poll_and_clear() still reads+clears the sensor's Interrupt
 *   Status Register (0x8E) on every call, even though nothing here is
 *   edge-triggered off it anymore: the VCNL3020 does NOT release its
 *   open-drain /INT line on its own once the underlying condition clears -
 *   only an explicit host write-1-to-clear does that. Skipping this would
 *   let /INT latch low after the very first threshold crossing and never
 *   produce another edge, which would silently break the pin-22 path
 *   (mechanism 2 above) after one use, even though plain polling (which
 *   reads the raw result registers directly, unaffected by ISR/INT state)
 *   would keep working. Same one-shot I2C-reopen-and-retry pattern as
 *   before if a transfer fails (see vcnl3020_i2c_reopen()'s comment) - the
 *   QCC730 I2C host peripheral goes stale after a WLAN BMPS/DTIM10 idle
 *   period (same root cause already documented/fixed for this sensor in
 *   demo/powertest_demo/src/vcnl3020.c).
 *
 * Wiring: SDA, SCL, VDD, GND, PLUS /INT -> chip pin 22 (EXT_WAKEUP_INTR_N),
 * open-drain, active LOW - external ~10k pull-up to 3V3 recommended.
 *
 * Prints exactly one thing, only when the decided status CHANGES from the
 * last one printed:
 *   VCNL3020: STATUS: OPEN
 *   VCNL3020: STATUS: CLOSED
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
 * sensitive. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"

#include "wifi_mqtt.h"

/* modules/hal/qcc730/qtmr/timer.h - not pulled in by this file's minimal
 * include set, declared the same ad-hoc way vcnl3020_test_poll_now() is
 * declared over in wifi_mqtt.c (no dedicated header). TEMP DEBUG
 * (2026-08-17): only needed for the timestamp added to the STATUS log line
 * below, to line up against wifi_mqtt.c's own timestamps while chasing the
 * "~2 min before MQTTX sees it" report. */
extern uint32_t hres_timer_curr_time_ms(void);

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
/* raw field value -> consecutive out-of-window samples before /INT asserts:
 * value N -> 2^N samples at VCNL3020_PROX_RATE_SELECT's rate (1.95/s here),
 * so value 2 -> 4 samples -> ~2.05s. Raised 1->2 (2026-08-18) after hardware
 * logs showed a single slow hand withdrawal forcing THREE separate pin-22
 * BMPS wakes ~1-2s apart (935ms/1415ms/1954ms gaps - each matching the old
 * value-1 = 2-sample = ~1.03s debounce) before the release finally
 * registered - because LOW_THRESHOLD is pinned to 0 (unreachable, see this
 * file's top comment), the comparator can only ever re-arm on the HIGH side,
 * so a value hovering in/out of the debounce window near HIGH_THRESHOLD
 * re-triggers /INT - and therefore a real forced radio wake via pin 22 -
 * every time, even though OPEN/CLOSED never actually changed in between.
 * Doubling the debounce window cuts that chatter roughly in half at the
 * cost of ~1s more worst-case release latency (touch/CLOSED detection is
 * unaffected in practice - a real touch jumps straight from ~2150 to
 * 4857+, clearing 4 consecutive samples immediately either way). Raise
 * further (3 -> 8 samples -> ~4.1s) if hardware logs still show multi-wake
 * chatter after this change. */
#define VCNL3020_INT_COUNT_EXCEED      2U

/* Interrupt Status Register (0x8E) bits - read+cleared to release /INT. */
#define VCNL3020_ISR_TH_HI_BIT         (1U << 0)
#define VCNL3020_ISR_TH_LOW_BIT        (1U << 1)

#define VCNL3020_LED_CURRENT_MA        20U  /* register 0x83, 10 mA/step */
#define VCNL3020_PROX_RATE_SELECT      0U   /* 1.95 measurements/s */

/* See this file's top comment for how this value was picked. */
#define HIGH_THRESHOLD                 3000U
#define LOW_THRESHOLD                  0U

/* xTaskNotify() bit set from vcnl3020_test_poll_now() - see that
 * function's comment. Single trigger type now (pre-BMPS timer poll and
 * pin-22 BMPS-wake both call the same function), no need to distinguish
 * sources. */
#define VCNL3020_POLL_NOW_BIT          (1U << 0)

/* Master log switch - off by default to save the UART/CPU-active time
 * every printf() costs. Flip to 1 to get every line back for debugging -
 * no call sites change either way.
 *
 * TEMP DEBUG (2026-08-17): flipped on together with wifi_mqtt.c's own
 * switch to chase the "publishes locally but MQTTX doesn't see it for ~2
 * min" report - see wifi_mqtt.c's matching comment. Revert to 0 once
 * resolved. */
#define VCNL3020_LOG_ENABLE            1

#if VCNL3020_LOG_ENABLE
#define info_printf(msg, ...) printf("VCNL3020: " msg, ##__VA_ARGS__)
#else
#define info_printf(msg, ...) do {} while (0)
#endif

static TaskHandle_t s_task_handle;
/* Last OPEN(0)/CLOSED(1) status actually printed/published. 0xFF = unknown,
 * forces the first real reading through. */
static uint8_t s_last_state = 0xFFU;

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
 * access has already failed once. */
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
 * transfer itself fails (see vcnl3020_i2c_reopen()'s comment). See this
 * file's top comment for why the ISR clear still matters even though
 * nothing here is edge-triggered anymore (it's what lets pin 22 see a
 * fresh edge on the NEXT transition). Returns 0 on success, -1 on an
 * unrecoverable I2C failure. */
static int vcnl3020_poll_and_clear(uint16_t *out_proximity)
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

/* Called from wifi_mqtt.c at two different times - see this file's top
 * comment: the pre-BMPS periodic poll timer, and bmps_wake_probe_cb()'s
 * PWR_EVT_WMAC_POST_AWAKE handler (pin-22 BMPS wake). Plain xTaskNotify(),
 * safe to call from any task/callback context. */
void vcnl3020_test_poll_now(void)
{
    if (s_task_handle != NULL)
        xTaskNotify(s_task_handle, VCNL3020_POLL_NOW_BIT, eSetBits);
}

/* I2C open + LED current + proximity rate + static threshold registers +
 * interrupt control + self-timed enable. No GPIO/interrupt-controller setup
 * here anymore - see this file's top comment. */
static int vcnl3020_configure(void)
{
    qapi_I2CM_Config_t i2c_cfg = { .Blocking = 1, .Dma = 0 };
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
     * never re-armed. Still needed even though this file no longer owns a
     * GPIO interrupt: this is what makes the sensor assert /INT at all, and
     * pin 22 (wired externally, armed by wifi_mqtt.c's side) depends on
     * that real electrical edge. */
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

    info_printf("armed: self-timed + threshold interrupt, HIGH=%u, LED=%umA (poll-driven, see wifi_mqtt.c)\n",
        (unsigned)HIGH_THRESHOLD, (unsigned)VCNL3020_LED_CURRENT_MA);
    return 0;
}

static void vcnl3020_test_task(void *arg)
{
    (void)arg;

    /* s_task_handle must be valid before wifi_mqtt.c can ever call
     * vcnl3020_test_poll_now() - vcnl3020_mqtt_start()/wifi_mqtt_task()
     * always run after vcnl3020_test_start() in vcnl3020_test_demo_main.c,
     * but set this before vcnl3020_configure() regardless, defensively. */
    s_task_handle = xTaskGetCurrentTaskHandle();

    while (vcnl3020_configure() != 0) {
        info_printf("retrying in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* Blocks indefinitely - every wake here is a real trigger from
     * wifi_mqtt.c (pre-BMPS timer poll or pin-22 BMPS wake), never an
     * independent timer owned by this file. See this file's top comment. */
    for (;;) {
        uint32_t notified;
        uint16_t proximity;

        xTaskNotifyWait(0, VCNL3020_POLL_NOW_BIT, &notified, portMAX_DELAY);

        if (vcnl3020_poll_and_clear(&proximity) != 0) {
            info_printf("poll read failed (I2C), will retry next trigger\n");
            continue;
        }

        {
            uint8_t state = (proximity >= HIGH_THRESHOLD) ? 1U : 0U;

            if (state != s_last_state) {
                s_last_state = state;
                info_printf("STATUS: %s (raw=%u) at %lu ms\n", state ? "CLOSED" : "OPEN",
                    (unsigned)proximity, (unsigned long)hres_timer_curr_time_ms());
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
