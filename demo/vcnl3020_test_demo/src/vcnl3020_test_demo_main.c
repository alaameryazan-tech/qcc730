/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* VCNL3020 bring-up test - entry point.
 *
 * See vcnl3020_test.c for the actual sensor logic (interrupt-driven on
 * GPIO3 - see that file's top comment for the wiring and how to read the
 * STATUS lines) and wifi_mqtt.c for the WLAN+MQTT publisher (same broker/
 * WLAN config as demo/powertest_demo) that reports every STATUS
 * transition. This file only wires app_init()/app_main() per this SDK's
 * demo entry-point
 * convention (build/freertos/common/application_code/main.c's
 * qccsdk_start_app_task(): app_init() runs first, then a dedicated
 * "qmain" task calls app_main() and is deleted when it returns - no
 * busy-loop needed here since all the real work runs in the tasks
 * vcnl3020_test_start()/vcnl3020_mqtt_start() spawn).
 *
 * WLAN/MQTT added 2026-08-14, DTIM10/BMPS (engages 15s after WLAN
 * associates) added the same day - see prj.conf/BUILD.gn and wifi_mqtt.c's
 * top comment for the full config. */

#include <string.h>
#include <stdint.h>

#include "wifi_mqtt.h"

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
    /* Independent of the sensor task above - starts its own WLAN-connect
     * task and only begins publishing once a real STATUS transition comes
     * in via vcnl3020_mqtt_publish_status() (see vcnl3020_test.c). */
    vcnl3020_mqtt_start();
    UART_SEND_DIRECT("app_init over\r\n");
}

void app_main(void)
{
    UART_SEND_DIRECT("app_main entry\r\n");
    UART_SEND_DIRECT("VCNL3020 bring-up test running - watch for STATUS: OPEN/CLOSED lines and MQTT publishes\r\n");
    UART_SEND_DIRECT("app_main over\r\n");
}