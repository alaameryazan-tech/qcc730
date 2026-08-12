/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Standalone VCNL3020 bring-up test - entry point.
 *
 * See vcnl3020_test.c for the actual sensor logic (pure I2C polling, no
 * /INT/GPIO wiring at all - see that file's top comment for why). This
 * file only wires app_init()/app_main() per this SDK's demo entry-point
 * convention (build/freertos/common/application_code/main.c's
 * qccsdk_start_app_task(): app_init() runs first, then a dedicated
 * "qmain" task calls app_main() and is deleted when it returns - no
 * busy-loop needed here since all the real work runs in the task
 * vcnl3020_test_start() spawns).
 *
 * Deliberately no WLAN/MQTT/DTIM/BMPS anywhere in this image - see
 * prj.conf. This exists purely to test the VCNL3020 sensor in isolation. */

#include <string.h>
#include <stdint.h>

#ifdef FERMION_SILICON
extern uint32_t UART_Send_direct(char *txbuf, uint32_t buflen);
#define UART_SEND_DIRECT(str) UART_Send_direct((str), strlen(str))
#else
#define UART_SEND_DIRECT(str)
#endif

extern void vcnl3020_test_start(void);

void app_init(void)
{
    UART_SEND_DIRECT("app_init entry\r\n");
    vcnl3020_test_start();
    UART_SEND_DIRECT("app_init over\r\n");
}

void app_main(void)
{
    UART_SEND_DIRECT("app_main entry\r\n");
    UART_SEND_DIRECT("VCNL3020 bring-up test running - watch for STATUS: OPEN/CLOSED lines\r\n");
    UART_SEND_DIRECT("app_main over\r\n");
}