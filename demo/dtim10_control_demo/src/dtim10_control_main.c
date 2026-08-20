/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* DTIM10 power-control demo - entry point.
 *
 * See wifi_mqtt.c for the whole implementation (WLAN connect, MQTT connect,
 * DTIM10/BMPS engage) - this file only wires app_init()/app_main() per this
 * SDK's demo entry-point convention (build/freertos/common/application_code/
 * main.c's qccsdk_start_app_task(): app_init() runs first, then a dedicated
 * "qmain" task calls app_main() and is deleted when it returns - no
 * busy-loop needed here since all the real work runs in the task
 * dtim10_control_start() spawns).
 *
 * Added 2026-08-20 as a control build to isolate demo/vcnl3020_test_demo's
 * ~30-40s power-spike question from its sensor code - see wifi_mqtt.c's top
 * comment for the full reasoning. */

#include <string.h>
#include <stdint.h>

#include "wifi_mqtt.h"

#ifdef FERMION_SILICON
extern uint32_t UART_Send_direct(char *txbuf, uint32_t buflen);
#define UART_SEND_DIRECT(str) UART_Send_direct((str), strlen(str))
#else
#define UART_SEND_DIRECT(str)
#endif

void app_init(void)
{
    UART_SEND_DIRECT("app_init entry\r\n");
    dtim10_control_start();
    UART_SEND_DIRECT("app_init over\r\n");
}

void app_main(void)
{
    UART_SEND_DIRECT("app_main entry\r\n");
    UART_SEND_DIRECT("DTIM10 control demo running - no sensor, WLAN+MQTT+DTIM10 only\r\n");
    UART_SEND_DIRECT("app_main over\r\n");
}
