/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Standalone VCNL3020 bring-up test - PURE I2C POLLING, no /INT wiring at
 * all.
 *
 * Wiring: SDA, SCL, VDD, GND only - nothing connected to GPIO3 or any
 * other pin. Deliberately reverted 2026-08-12 from an interrupt-driven
 * design: extensive testing traced a continuous, non-touch-related
 * interrupt storm to the physical /INT connection itself (the storm
 * stopped instantly when that wire was unplugged, with everything else -
 * sensor, I2C, threshold config - already independently confirmed
 * correct). Rather than keep chasing a flaky physical connection, this
 * version sidesteps GPIO3 entirely: the sensor still free-runs self-timed
 * measurements in the background, this task just reads the result
 * register directly on a plain timer - no interrupt, no threshold
 * register, no ISR, nothing that can storm.
 *
 * Prints exactly one thing, only when it changes:
 *   VCNL3020: STATUS: OPEN
 *   VCNL3020: STATUS: CLOSED
 *
 * TUNE ME - VCNL3020_CLOSED_THRESHOLD: proximity >= this -> CLOSED, else
 * OPEN. Calibrated from real hardware data (2026-08-12): idle baseline
 * measured rock-steady at ~2140-2160 (about 20 counts of noise), and a
 * hand near the sensor read ~4857 rising to ~15000+ - a huge, clean gap.
 * 3000 sits comfortably in the middle: far enough above the ~2160 baseline
 * ceiling to never false-trigger on noise, and far enough below the
 * ~4857+ "something's there" floor to stay easily sensitive. Adjust if
 * your own testing shows it should be more/less sensitive. */

#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"

#define VCNL3020_I2C_ADDR              0x13
#define VCNL3020_I2C_INSTANCE          QAPI_I2C_INSTANCE_SE0_E

#define VCNL3020_REG_COMMAND           0x80
#define VCNL3020_REG_PROXIMITY_RATE    0x82
#define VCNL3020_REG_IR_LED_CURRENT    0x83
#define VCNL3020_REG_PROX_RESULT_MSB   0x87
#define VCNL3020_REG_PROX_RESULT_LSB   0x88

#define VCNL3020_CMD_SELFTIMED_EN_BIT  (1U << 0)
#define VCNL3020_CMD_PROX_EN_BIT       (1U << 1)

#define VCNL3020_LED_CURRENT_MA        20U  /* register 0x83, 10 mA/step */
#define VCNL3020_PROX_RATE_SELECT      0U   /* 1.95 measurements/s */

/* See this file's top comment for how this value was picked. */
#define VCNL3020_CLOSED_THRESHOLD      3000U

#define VCNL3020_POLL_PERIOD_MS        300U /* no power budget to protect in this image */

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

/* I2C open + LED current + proximity rate + self-timed enable. No
 * threshold registers, no interrupt control register, no GPIO - this
 * version never touches any of that. */
static int vcnl3020_configure(void)
{
    qapi_I2CM_Config_t i2c_cfg = { .Blocking = 1, .Dma = 0 };

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

    /* selftimed_en MUST be set before prox_en per the datasheet, hence
     * two separate writes. */
    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND, VCNL3020_CMD_SELFTIMED_EN_BIT) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_COMMAND,
            VCNL3020_CMD_SELFTIMED_EN_BIT | VCNL3020_CMD_PROX_EN_BIT) != 0) {
        info_printf("self-timed enable failed\n");
        return -1;
    }

    info_printf("self-timed measurement running, threshold=%u, LED=%umA, polling every %ums\n",
        (unsigned)VCNL3020_CLOSED_THRESHOLD, (unsigned)VCNL3020_LED_CURRENT_MA,
        (unsigned)VCNL3020_POLL_PERIOD_MS);
    return 0;
}

static void vcnl3020_test_task(void *arg)
{
    uint8_t last_state = 0xFFU; /* neither OPEN(0) nor CLOSED(1) - forces the first read to print */

    (void)arg;

    while (vcnl3020_configure() != 0) {
        info_printf("retrying in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    for (;;) {
        uint16_t proximity;

        if (vcnl3020_read_result(&proximity) == 0) {
            uint8_t state = (proximity >= VCNL3020_CLOSED_THRESHOLD) ? 1U : 0U;

            if (state != last_state) {
                last_state = state;
                info_printf("STATUS: %s\n", state ? "CLOSED" : "OPEN");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(VCNL3020_POLL_PERIOD_MS));
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