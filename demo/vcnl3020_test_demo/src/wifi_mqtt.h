/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef VCNL3020_TEST_WIFI_MQTT_H
#define VCNL3020_TEST_WIFI_MQTT_H

#include <stdint.h>

/* Starts the minimal WLAN connect + persistent MQTT publisher added ONLY to
 * report VCNL3020 STATUS transitions (see vcnl3020_test.c) to MQTT topic
 * "sensor/proximity" on broker 139.162.181.126:1883 - nothing else. Call
 * once, e.g. from app_init(). Safe to call more than once (only starts the
 * task the first time). DTIM10/BMPS power-save engages 15s after WLAN
 * associates and is disengaged again before any reconnect attempt - see
 * wifi_mqtt.c's top comment for the full design; does not touch the
 * VCNL3020/GPIO3 sensor logic in any way. */
void vcnl3020_mqtt_start(void);

/* Call ONLY on a real STATUS transition (OPEN<->CLOSED) - never
 * periodically. is_closed: 0 publishes "OPEN", nonzero publishes "CLOSED".
 * Non-blocking (queues the publish for wifi_mqtt.c's own task): the caller
 * is servicing a genuine GPIO3 interrupt at the call site and must not
 * block on network I/O. Safe to call before vcnl3020_mqtt_start()/WLAN
 * connects - queued events are published once the MQTT session comes up.
 * If the queue is momentarily full the event is dropped (logged), never
 * blocking the caller. */
void vcnl3020_mqtt_publish_status(uint8_t is_closed);

#endif /* VCNL3020_TEST_WIFI_MQTT_H */
