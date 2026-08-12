/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Vishay VCNL3020 IR proximity sensor - I2C, INTERRUPT-DRIVEN.
 *
 * This is a direct port of the design proven out in the standalone
 * demo/vcnl3020_test_demo (no WLAN/MQTT/DTIM at all) - confirmed on
 * hardware there: silent at idle, one clean STATUS transition per real
 * touch/release, silent again with /INT physically disconnected (no
 * floating-pin chatter). Two things changed from THIS file's own earlier
 * attempt, both root-caused by comparing against that working demo - see
 * vcnl3020.c's top-of-file comment for the full writeup:
 *   1. No more per-I2C-access "close+reopen" (vcnl3020_i2c_reopen() used
 *      to run before every single register access) - that pattern is the
 *      leading suspect for the original ~95ms interrupt storm, not real
 *      touch events or board wiring. I2C is now opened once in
 *      vcnl3020_init() and only reopened as an on-failure recovery retry.
 *   2. No more dynamic per-transition threshold re-arming
 *      (vcnl3020_arm_next_transition()/s_armed_low). LOW is pinned to 0
 *      and HIGH=VCNL3020_THRESHOLD_HIGH, both static, written once. The
 *      STATUS decision compares the raw proximity value against
 *      VCNL3020_THRESHOLD_HIGH directly, the same way the working
 *      standalone demo does - not from which Interrupt Status Register
 *      bit fired.
 *
 * Wiring: SDA, SCL, VDD, GND as before, PLUS /INT -> VCNL3020_INT_GPIO_ID
 * with a pull-up (external ~10k to 3V3 recommended, matches an open-drain
 * interrupt line - see that macro's comment for why EXT_WAKEUP is NOT used
 * here despite being the originally-proposed pin).
 */

#ifndef VCNL3020_H
#define VCNL3020_H

#include <stdint.h>

#include "qapi_gpio.h"

/* IR LED drive current used for every self-timed measurement, in mA.
 * Register 0x83 takes this in 10 mA steps (0-200 mA, i.e. 0-20 decimal).
 * Higher = more reflected IR = larger raw counts/longer range, at the cost
 * of more current per conversion (the sensor is now converting
 * continuously in the background at VCNL3020_PROX_RATE_SELECT's rate,
 * regardless of host activity, so this directly sets the sensor's own
 * average current draw). Must be a multiple of 10 between 0 and 200. */
#define VCNL3020_LED_CURRENT_MA        20U

/* qapi_I2CM_Transfer() I2C instance the sensor is wired to. */
#define VCNL3020_I2C_INSTANCE          QAPI_I2C_INSTANCE_SE0_E

/* Proximity Rate register (0x82) value - how often the sensor measures on
 * its own; raw index 0-7 (low 3 bits only):
 *   0: 1.95 measurements/s (~513ms period, used here - lowest power)
 *   1: 3.90625/s   2: 7.8125/s   3: 15.625/s
 *   4: 31.25/s     5: 62.5/s     6: 125/s      7: 250/s
 * Governs both how quickly a real crossing is noticed (worst case ~1
 * period later) and the sensor's own average current, since it measures
 * continuously in the background regardless of whether the host is awake.
 * Appropriate for slow-changing proximity events per the datasheet default. */
#define VCNL3020_PROX_RATE_SELECT      0U

/* Interrupt Control Register (0x89) int_count_exceed field (bits 7:5) -
 * how many CONSECUTIVE self-timed measurements must land outside the
 * armed threshold before /INT actually asserts, i.e. hardware debounce
 * against a single noisy sample. Raw 3-bit field value -> consecutive
 * count required:
 *   0: 1   1: 2   2: 4   3: 8   4: 16   5: 32   6: 64   7: 128
 * Default here is 1 (2 consecutive samples, ~1s worst-case extra latency
 * at the default VCNL3020_PROX_RATE_SELECT). */
#define VCNL3020_INT_COUNT_EXCEED      1U

/* CONFIRMED WORKING on hardware via demo/vcnl3020_test_demo: proximity >=
 * this -> CLOSED (object detected), else OPEN. Idle baseline measured
 * ~2150-2160; this threshold cleanly separated touched/covered from clear
 * across repeated tests in that standalone demo. LOW is intentionally
 * fixed at 0 (unreachable - proximity is unsigned) and never changed; the
 * sensor's interrupt comparator needs some low-threshold value
 * programmed, this just keeps that side permanently inert - only
 * VCNL3020_THRESHOLD_HIGH is meaningful here. */
#define VCNL3020_THRESHOLD_HIGH        3000U
#define VCNL3020_THRESHOLD_LOW         0U

/* VCNL3020 /INT is OPEN-DRAIN, active LOW - needs a pull-up (external
 * ~10k to 3.3V recommended; QAPI_GPIO_PULL_UP_E in vcnl3020_init() enables
 * the SoC's own, usually weaker, internal pull-up as a backup only).
 *
 * NOTE: the originally-proposed wiring target was the SoC's EXT_WAKEUP pin
 * (chip pin 22). That pin is NOT usable here: it is a dedicated PMU/AON
 * pin wired directly into this SDK's own deep-sleep (IMPS) wake sequencer
 * (see PMU_AON_LIC_INT_* registers in modules/hal/inc/qcc730v2.h and the
 * low-level handling in modules/core/system/sys_src/nt_socpm_sleep.c /
 * wifi_fw_ext_intr.c) - there is no qapi_GPIO_Enable_Interrupt()-style
 * application callback hook exposed for it, and this demo runs BMPS/
 * DTIM10 (ENABLE_IMPS_DEEPSLEEP 0 in powertest_demo.c), not IMPS deep
 * sleep, so that wake path is never even entered. Under BMPS the CPU is
 * always running FreeRTOS (idle-task WFI at most - see the "wakebmps"
 * clarification this repo's contact_sensor_task.c already worked out on
 * hardware), so a plain numbered GPIO with qapi_GPIO_Enable_Interrupt() is
 * both necessary and sufficient. GPIO3 is confirmed working (see this
 * file's top comment) and free of I2C/JTAG conflicts on this DevKit -
 * physically wire /INT (and its pull-up) to the GPIO3 header pin, NOT
 * EXT_WAKEUP. */
#define VCNL3020_INT_GPIO_ID           QAPI_GPIO_ID3_E

/* xTaskNotify() bit this driver sets (via xTaskNotifyFromISR(), from
 * vcnl3020_gpio_isr()) on whichever task called vcnl3020_init() - that
 * task owns responding to it (typically by calling
 * vcnl3020_service_interrupt() and publishing the result), NOT this
 * driver. Pick a bit that doesn't collide with any other notify bit the
 * calling task already uses. */
#define VCNL3020_NOTIFY_BIT            (1U << 0)

/* One-time setup, call from the task that will own sensor events (it is
 * captured internally via xTaskGetCurrentTaskHandle() as the
 * VCNL3020_NOTIFY_BIT target - see above):
 *   - opens I2C (once - see this file's top comment)
 *   - programs LED current (0x83) and proximity rate (0x82)
 *   - programs the static LOW=0/HIGH=VCNL3020_THRESHOLD_HIGH threshold
 *     registers (0x8A-0x8D)
 *   - programs the Interrupt Control Register (0x89): INT_THRES_EN with
 *     the debounce count above, INT_PROX_READY_EN left OFF (no
 *     interrupt-per-measurement, threshold crossings only)
 *   - enables self-timed continuous measurement (Command Register 0x80:
 *     selftimed_en, THEN prox_en in a second write, per datasheet ordering)
 *   - configures VCNL3020_INT_GPIO_ID as an edge-triggered input interrupt
 * Safe to call again later to recover from a service failure (see
 * vcnl3020_service_interrupt()). Returns 0 on success, -1 on I2C failure. */
int vcnl3020_init(void);

/* Call after waking on VCNL3020_NOTIFY_BIT. Reads + clears the sensor's
 * Interrupt Status Register (0x8E, releasing the open-drain /INT line so
 * future interrupts can fire) and reads the current proximity result
 * (0x87/0x88) into *out_proximity - unconditionally, regardless of which
 * (if any) bit 0x8E reported; the caller decides CLOSED/OPEN by comparing
 * *out_proximity against VCNL3020_THRESHOLD_HIGH itself (matching the
 * proven standalone-demo design - see this file's top comment).
 *
 * Returns 0 on success, -1 on I2C failure (caller should call
 * vcnl3020_init() again to recover - this function already attempts one
 * internal close+reopen retry on failure before giving up). */
int vcnl3020_service_interrupt(uint16_t *out_proximity);

/* On-demand direct read of the latest self-timed result (0x87/0x88) -
 * does NOT touch the Interrupt Status Register. Safe to call any time
 * (e.g. from a periodic MQTT-keepalive tick, independent of interrupt
 * servicing) since the sensor is continuously self-timing regardless of
 * whether the host is reading it. Returns 0 on success, -1 on I2C failure. */
int vcnl3020_read_result(uint16_t *out_proximity);

#endif /* VCNL3020_H */
