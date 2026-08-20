/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef DTIM10_CONTROL_WIFI_MQTT_H
#define DTIM10_CONTROL_WIFI_MQTT_H

/* Starts the minimal WLAN connect + persistent MQTT client. Call once, e.g.
 * from app_init(). Safe to call more than once (only starts the task the
 * first time). DTIM10/BMPS power-save engages 15s after WLAN associates and
 * is disengaged again before any reconnect attempt - see wifi_mqtt.c's top
 * comment for the full design. This is the ENTIRE job of this demo - see
 * that file's top comment for why nothing else is here on purpose. */
void dtim10_control_start(void);

#endif /* DTIM10_CONTROL_WIFI_MQTT_H */
