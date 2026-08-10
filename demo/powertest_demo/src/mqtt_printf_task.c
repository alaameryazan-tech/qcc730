/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* SHT40 + MQTT publisher for powertest_demo.
 *
 * Reuses the exact same MQTT setup (broker, topic, QoS, reconnect handling)
 * as the qcli_demo SHT40 publisher (sht40_task.c) and the SHT40 I2C read
 * sequence from that same file, but wired into the DTIM10 power-test flow:
 * publishes one reading immediately after connecting, then goes idle in
 * DTIM10 sleep and publishes again every MQTT_PUBLISH_INTERVAL_MS (10 min
 * by default) so current draw can be measured under DTIM10 with only
 * infrequent, predictable wake bursts for real sensor data.
 *
 * Started from powertest_demo.c once WLAN connects (see
 * mqtt_printf_task_start()); does not touch WLAN/listen-interval setup.
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "core_mqtt.h"
#include "plaintext_posix.h"
#include "nt_timer.h"

/* Same broker/topic/QoS/reconnect handling as the SHT40 MQTT publisher. */
#define MQTT_BROKER                 "139.162.181.126"
#define MQTT_PORT                   1883
#define MQTT_CLIENT_ID              "sht40_qcc7030"
#define MQTT_USERNAME               "AABBCCDD"
#define MQTT_PASSWORD               "KXL8DHPsXggvRELD"
#define MQTT_TOPIC_PUB              "sensor/sht40"
#define MQTT_TOPIC_CMD              "sensor/cmd"
/* Status/availability topic: retained "connected" published right after
 * we connect, non-retained Last Will published by the BROKER (not us) if
 * we ever drop off ungracefully (power loss, out of range, keepalive
 * timeout) without a clean MQTT disconnect. */
#define MQTT_TOPIC_STATUS           "sensor/status"
#define MQTT_LWT_MESSAGE            "qcc disconnected"
#define MQTT_ONLINE_MESSAGE         "qcc connected"
#define MQTT_NETWORK_BUF_SIZE       1024U
/* 10 min instead of the MQTT-typical 60s: fewer PINGREQ/PINGRESP bursts
 * means fewer interruptions to DTIM10 idle sleep, reducing average current
 * draw. Broker will wait ~1.5x this (~15 min) before considering the
 * session dead if nothing at all is heard from us. */
#define MQTT_KEEPALIVE_SEC          600U
#define MQTT_CONNACK_TIMEOUT_MS     5000U
#define MQTT_TRANSPORT_TIMEOUT_MS   1000U

/* How often a reading is measured and published once connected: once
 * immediately on (re)connect, then on this interval. 10 min matches the
 * keepalive above, so publishes also double as the traffic keeping the
 * session alive - no separate idle PINGREQ needed most cycles. */
#define MQTT_PUBLISH_INTERVAL_MS    600000U

/* Set to 0 to establish WLAN + MQTT and then stay fully silent in DTIM10
 * sleep (no periodic publish at all) - useful for a clean baseline
 * sleep-current measurement with no traffic whatsoever. Default 1: publish
 * an SHT40 reading on connect and every MQTT_PUBLISH_INTERVAL_MS after.
 * Either way, MQTT_ProcessLoop() still runs to service the connection, and
 * the on-demand "measure" command on sensor/cmd still works. */
#define MQTT_AUTO_PUBLISH 1

/* How often the task wakes to call MQTT_ProcessLoop(). Kept well under
 * MQTT_PUBLISH_INTERVAL_MS/keepalive but still coarse (10s) on purpose:
 * confirmed by log timestamps that a fast poll here (500ms) forces the
 * board to wake ~2x more often than DTIM10's own ~1024ms cycle, which
 * previously kept the DTIM10 current target out of reach - the polling
 * loop itself, not the radio, was keeping the CPU busy. */
#define MQTT_PROCESS_INTERVAL_MS 10000U

#define info_printf(msg, ...) printf("MQTT_TEST: " msg, ##__VA_ARGS__)

/* Set by powertest_demo.c once the WLAN CONNECT event / 4-way handshake
 * completes; also used to unwind this task's loops on disconnect. */
extern uint8_t g_wifi_ready;

static PlaintextParams_t s_net_params;
static NetworkContext_t  s_net_ctx;
static MQTTContext_t     s_mqtt_ctx;
static uint8_t           s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static volatile int      s_publish_now;
static TaskHandle_t      s_task_handle;
static volatile uint8_t  s_started;

/* ------------------------------------------------------------------ */
/*  SHT40 / I2C - ported from qcli_demo/src/sht40_task.c                */
/* ------------------------------------------------------------------ */

#define SHT40_I2C_ADDR              0x44
#define SHT40_CMD_MEAS              0xFD

static uint8_t s_sht40_ready;

static uint8_t sht40_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0xFF;
    uint32_t index;
    uint8_t bit;

    for (index = 0; index < len; index++) {
        crc ^= data[index];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static int sht40_open(void)
{
    qapi_I2CM_Config_t cfg = { .Blocking = TRUE, .Dma = FALSE };
    qapi_Status_t status;

    /* Close first in case the instance is already open - qapi_I2CM_Open()
     * fails with an "already open" error otherwise (confirmed by log:
     * status=-30024 when called a second time without this). Closing an
     * instance that isn't currently open is expected to just fail too;
     * its return value is intentionally ignored so this function is safe
     * to call more than once, e.g. before every periodic measurement. */
    qapi_I2CM_Close(QAPI_I2C_INSTANCE_SE0_E);

    status = qapi_I2CM_Open(QAPI_I2C_INSTANCE_SE0_E, &cfg);

    info_printf("SHT40 I2C open status=%d\n", status);
    if (status != QAPI_OK) {
        s_sht40_ready = 0;
        return -1;
    }

    s_sht40_ready = 1;
    return 0;
}

static int sht40_measure(int *temp_x10, int *rh_x10)
{
    uint8_t cmd = SHT40_CMD_MEAS;
    uint8_t rx[6];
    qapi_I2CM_Transfer_Config_t xcfg = {
        .BusFreqKHz = 400,
        .SlaveAddress = SHT40_I2C_ADDR,
        .SlaveMaxClockStretchUs = 0,
        .Delay = 0,
        .NoiseReject = 0,
    };
    qapi_I2CM_Descriptor_t wr = {
        .Buffer = &cmd,
        .Length = 1,
        .Transferred = 0,
        .Flags = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_WRITE,
    };
    qapi_I2CM_Descriptor_t rd = {
        .Buffer = rx,
        .Length = 6,
        .Transferred = 0,
        .Flags = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_READ,
    };
    uint16_t t_raw;
    uint16_t rh_raw;

    if (!s_sht40_ready) {
        info_printf("SHT40 read skipped, I2C not ready\n");
        return -1;
    }

    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &wr, 1, NULL, NULL) != QAPI_OK)
        return -1;

    vTaskDelay(pdMS_TO_TICKS(10));

    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &rd, 1, NULL, NULL) != QAPI_OK)
        return -1;

    if (sht40_crc8(rx, 2) != rx[2] || sht40_crc8(&rx[3], 2) != rx[5]) {
        info_printf("SHT40 CRC check failed: %02x %02x %02x %02x %02x %02x\n",
            rx[0], rx[1], rx[2], rx[3], rx[4], rx[5]);
        return -1;
    }

    t_raw = ((uint16_t)rx[0] << 8) | rx[1];
    rh_raw = ((uint16_t)rx[3] << 8) | rx[4];
    *temp_x10 = -450 + (int)(1750 * (uint32_t)t_raw / 65535);
    *rh_x10 = -60 + (int)(1250 * (uint32_t)rh_raw / 65535);

    return 0;
}

/* ------------------------------------------------------------------ */
/*  MQTT transport / callbacks                                          */
/* ------------------------------------------------------------------ */

static uint32_t mqtt_get_time_ms(void)
{
    return hres_timer_curr_time_ms();
}

static void mqtt_event_cb(MQTTContext_t *ctx, MQTTPacketInfo_t *pkt, MQTTDeserializedInfo_t *info)
{
    MQTTPublishInfo_t *pub;

    (void)ctx;

    if ((pkt->type & 0xF0U) != MQTT_PACKET_TYPE_PUBLISH)
        return;

    pub = info->pPublishInfo;
    if (pub == NULL)
        return;

    /* "measure" on sensor/cmd forces an immediate publish, same as the
     * SHT40 publisher's command handling. */
    if (pub->topicNameLength != (sizeof(MQTT_TOPIC_CMD) - 1) ||
        strncmp(MQTT_TOPIC_CMD, pub->pTopicName, pub->topicNameLength) != 0)
        return;

    if (pub->payloadLength == (sizeof("measure") - 1) &&
        strncmp("measure", (const char *)pub->pPayload, pub->payloadLength) == 0)
        s_publish_now = 1;
}

static int mqtt_connect_and_subscribe(void)
{
    TransportInterface_t transport;
    MQTTFixedBuffer_t netbuf;
    MQTTConnectInfo_t conninfo;
    MQTTPublishInfo_t willInfo;
    MQTTSubscribeInfo_t sub;
    ServerInfo_t server;
    bool session_present;
    uint16_t sub_id;

    s_net_params.socketDescriptor = -1;
    s_net_ctx.pParams = &s_net_params;

    server.pHostName = MQTT_BROKER;
    server.hostNameLength = sizeof(MQTT_BROKER) - 1;
    server.port = MQTT_PORT;

    if (Plaintext_Connect(&s_net_ctx, &server, MQTT_TRANSPORT_TIMEOUT_MS, MQTT_TRANSPORT_TIMEOUT_MS) != SOCKETS_SUCCESS)
        return -1;

    transport.pNetworkContext = &s_net_ctx;
    transport.send = Plaintext_Send;
    transport.recv = Plaintext_Recv;
    transport.writev = NULL;

    netbuf.pBuffer = s_mqtt_buf;
    netbuf.size = sizeof(s_mqtt_buf);

    if (MQTT_Init(&s_mqtt_ctx, &transport, mqtt_get_time_ms, mqtt_event_cb, &netbuf) != MQTTSuccess) {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    memset(&conninfo, 0, sizeof(conninfo));
    conninfo.cleanSession = true;
    conninfo.keepAliveSeconds = MQTT_KEEPALIVE_SEC;
    conninfo.pClientIdentifier = MQTT_CLIENT_ID;
    conninfo.clientIdentifierLength = sizeof(MQTT_CLIENT_ID) - 1;
    conninfo.pUserName = MQTT_USERNAME;
    conninfo.userNameLength = sizeof(MQTT_USERNAME) - 1;
    conninfo.pPassword = MQTT_PASSWORD;
    conninfo.passwordLength = sizeof(MQTT_PASSWORD) - 1;

    /* Last Will: the BROKER publishes this on our behalf, NOT retained, only
     * if we drop off ungracefully (no clean MQTT DISCONNECT) - e.g. power
     * loss, out of range, or keepalive timeout. Non-retained: a client that
     * subscribes after the fact sees nothing until the next live event,
     * rather than getting a possibly-stale "disconnected" forever. */
    memset(&willInfo, 0, sizeof(willInfo));
    willInfo.qos = MQTTQoS0;
    willInfo.retain = false;
    willInfo.pTopicName = MQTT_TOPIC_STATUS;
    willInfo.topicNameLength = sizeof(MQTT_TOPIC_STATUS) - 1;
    willInfo.pPayload = MQTT_LWT_MESSAGE;
    willInfo.payloadLength = sizeof(MQTT_LWT_MESSAGE) - 1;

    if (MQTT_Connect(&s_mqtt_ctx, &conninfo, &willInfo, MQTT_CONNACK_TIMEOUT_MS, &session_present) != MQTTSuccess) {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    /* Publish our own retained "connected" status right away, so the topic
     * reflects reality again after any earlier LWT "disconnected" and
     * doesn't just sit stuck on the last will forever. */
    {
        MQTTPublishInfo_t onlineInfo;

        memset(&onlineInfo, 0, sizeof(onlineInfo));
        onlineInfo.qos = MQTTQoS0;
        onlineInfo.retain = true;
        onlineInfo.pTopicName = MQTT_TOPIC_STATUS;
        onlineInfo.topicNameLength = sizeof(MQTT_TOPIC_STATUS) - 1;
        onlineInfo.pPayload = MQTT_ONLINE_MESSAGE;
        onlineInfo.payloadLength = sizeof(MQTT_ONLINE_MESSAGE) - 1;
        MQTT_Publish(&s_mqtt_ctx, &onlineInfo, 0);
    }

    sub.qos = MQTTQoS0;
    sub.pTopicFilter = MQTT_TOPIC_CMD;
    sub.topicFilterLength = sizeof(MQTT_TOPIC_CMD) - 1;
    sub_id = MQTT_GetPacketId(&s_mqtt_ctx);
    if (MQTT_Subscribe(&s_mqtt_ctx, &sub, 1, sub_id) != MQTTSuccess) {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    {
        MQTTStatus_t process_status = MQTT_ProcessLoop(&s_mqtt_ctx);
        if (process_status != MQTTSuccess && process_status != MQTTNeedMoreBytes) {
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }

    info_printf("connected to %s:%d, subscribed to %s\n", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_CMD);
    return 0;
}

static int mqtt_publish(int temp_x10, int rh_x10)
{
    char msg[48];
    int len;
    MQTTPublishInfo_t pub;

    len = snprintf(msg, sizeof(msg), "Temp=%d.%dC Feuchte=%d.%d%%",
                   temp_x10 / 10, temp_x10 % 10, rh_x10 / 10, rh_x10 % 10);
    if (len <= 0 || len >= (int)sizeof(msg))
        return -1;

    memset(&pub, 0, sizeof(pub));
    pub.qos = MQTTQoS0;
    pub.retain = false;
    pub.dup = false;
    pub.pTopicName = MQTT_TOPIC_PUB;
    pub.topicNameLength = sizeof(MQTT_TOPIC_PUB) - 1;
    pub.pPayload = msg;
    pub.payloadLength = (size_t)len;

    return MQTT_Publish(&s_mqtt_ctx, &pub, 0) == MQTTSuccess ? 0 : -1;
}

/* Take one SHT40 reading and publish it. Used both right after connecting
 * and on the periodic MQTT_PUBLISH_INTERVAL_MS tick.
 *
 * Return values distinguish WHERE it failed, since only one of these is
 * actually a connection problem:
 *    0  measured and published successfully
 *    1  measurement failed (I2C/sensor hiccup) - NOT a connection problem;
 *       caller should just log and retry next cycle, leaving the MQTT
 *       session untouched
 *   -1  measurement succeeded but MQTT_Publish() itself failed - a real
 *       connection problem, caller should reconnect
 *
 * Conflating these (treating a sensor hiccup as if the MQTT session had
 * died) was causing unnecessary reconnect storms - each full TCP+MQTT
 * handshake burns far more radio time than a skipped reading ever would,
 * which showed up as sustained elevated current instead of a quick
 * recovery back to idle. */
static int sht40_measure_and_publish(void)
{
    int temp_x10 = 0;
    int rh_x10 = 0;

    /* Re-open (via sht40_open(), which now closes first) immediately
     * before every measurement, not just once at task boot. Every
     * periodic reading taken while BMPS/DTIM10 was actively cycling
     * failed consistently (100% of attempts, per log), while the one
     * reading taken before BMPS engaged succeeded - pointing at the I2C
     * peripheral's state going stale under BMPS. sht40_open() now closes
     * the instance first, so calling it again here is safe (an earlier
     * attempt without the close-first pairing broke the first
     * measurement too - reverted, this is the corrected version). */
    sht40_open();

    if (sht40_measure(&temp_x10, &rh_x10) != 0) {
        info_printf("SHT40 measurement failed, will retry next cycle\n");
        return 1;
    }

    info_printf("Temp=%d.%dC Feuchte=%d.%d%%\n", temp_x10 / 10, temp_x10 % 10, rh_x10 / 10, rh_x10 % 10);
    return mqtt_publish(temp_x10, rh_x10) == 0 ? 0 : -1;
}

static void mqtt_printf_task(void *arg)
{
    TickType_t last_pub;

    (void)arg;

    sht40_open();

    /* Let DHCP / network settle after WLAN connect before opening the
     * MQTT socket. */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        info_printf("connecting to %s:%d...\n", MQTT_BROKER, MQTT_PORT);
        while (g_wifi_ready && (mqtt_connect_and_subscribe() != 0)) {
            info_printf("connect failed, retry in 5s\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        if (!g_wifi_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

#if MQTT_AUTO_PUBLISH
        /* Send a reading immediately on (re)connect, before settling into
         * the periodic interval below. A measurement hiccup (return 1) is
         * already logged inside the helper and doesn't affect the fresh
         * connection; only an actual publish failure (-1) is worth an
         * extra note here. */
        if (sht40_measure_and_publish() < 0)
            info_printf("initial publish failed\n");
#endif /* MQTT_AUTO_PUBLISH */

        last_pub = xTaskGetTickCount();
        s_publish_now = 0;

        for (;;) {
            MQTTStatus_t st;

            if (!g_wifi_ready) {
                Plaintext_Disconnect(&s_net_ctx);
                break;
            }

            st = MQTT_ProcessLoop(&s_mqtt_ctx);
            if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                info_printf("session lost (%d), reconnecting\n", (int)st);
                Plaintext_Disconnect(&s_net_ctx);
                break;
            }

#if MQTT_AUTO_PUBLISH
            {
                TickType_t now = xTaskGetTickCount();
                uint32_t elapsed = (uint32_t)((now - last_pub) * portTICK_PERIOD_MS);
                if (elapsed >= MQTT_PUBLISH_INTERVAL_MS)
                    s_publish_now = 1;
            }
#endif /* MQTT_AUTO_PUBLISH */

            if (s_publish_now) {
                /* Only an actual publish failure (-1) is a connection
                 * problem worth reconnecting over. A measurement-only
                 * failure (1) is already logged inside the helper; leave
                 * the MQTT session alone and just try again next cycle. */
                if (sht40_measure_and_publish() < 0) {
                    info_printf("publish failed, reconnecting\n");
                    Plaintext_Disconnect(&s_net_ctx);
                    s_publish_now = 0;
                    break;
                }
                last_pub = xTaskGetTickCount();
                s_publish_now = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(MQTT_PROCESS_INTERVAL_MS));
        }

        /* Brief pause before reconnect, same as the SHT40 publisher. */
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* Call once WLAN is connected (g_wifi_ready == 1) to start the SHT40
 * publisher. Safe to call more than once - only starts the task the first
 * time. */
void mqtt_printf_task_start(void)
{
    if (s_started)
        return;

    if (nt_qurt_thread_create(mqtt_printf_task, "mqtt_printf_task", 4096, NULL, 5, &s_task_handle) == pdPASS) {
        s_started = 1;
    } else {
        info_printf("task create fail\n");
    }
}