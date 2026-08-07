/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Minimal MQTT test publisher for powertest_demo.
 *
 * Reuses the exact same MQTT setup (broker, topic, QoS, publish interval,
 * reconnect handling) as the SHT40 MQTT demo (see sht40_task.c under
 * qcli_demo, and the disabled ENABLE_SHT40_MQTT block in powertest_demo.c),
 * but publishes a plain printf-style counter/uptime message instead of a
 * real sensor reading. Used to verify MQTT connectivity and message
 * delivery timing while measuring current draw under DTIM10 sleep, without
 * needing the SHT40 sensor connected.
 *
 * Started from powertest_demo.c once WLAN connects (see
 * mqtt_printf_task_start()); does not touch WLAN/listen-interval setup.
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "core_mqtt.h"
#include "plaintext_posix.h"
#include "nt_timer.h"

/* Same broker/topic/QoS/timing as the SHT40 MQTT publisher - only the
 * payload source differs. */
#define MQTT_BROKER                 "139.162.181.126"
#define MQTT_PORT                   1883
#define MQTT_CLIENT_ID              "sht40_qcc7030"
#define MQTT_USERNAME               "AABBCCDD"
#define MQTT_PASSWORD               "KXL8DHPsXggvRELD"
#define MQTT_TOPIC_PUB              "sensor/sht40"
#define MQTT_TOPIC_CMD              "sensor/cmd"
#define MQTT_NETWORK_BUF_SIZE       1024U
#define MQTT_KEEPALIVE_SEC          60U
#define MQTT_CONNACK_TIMEOUT_MS     5000U
#define MQTT_TRANSPORT_TIMEOUT_MS   1000U
#define MQTT_PUBLISH_INTERVAL_MS    30000U

/* Set to 1 to publish a counter/uptime message every MQTT_PUBLISH_INTERVAL_MS
 * once connected (useful for testing delivery timing under DTIM10). Set to
 * 0 (default) to just establish WLAN + the MQTT connection and then stay
 * fully idle in DTIM10 sleep - no periodic traffic at all - for a clean
 * baseline sleep-current measurement. Either way, MQTT_ProcessLoop() still
 * runs to service the connection (keepalive pings, incoming data), and the
 * on-demand "measure" command on sensor/cmd still works for a manual
 * one-off publish. */
#define MQTT_AUTO_PUBLISH 0

/* How often the task wakes to call MQTT_ProcessLoop(). Confirmed by log
 * timestamps that a fast poll here (500ms) forces the board to wake ~2x
 * more often than DTIM10's own ~1024ms cycle, which is why the DTIM10
 * current target wasn't being hit - the polling loop itself, not the
 * radio, was keeping the CPU busy. When idle (MQTT_AUTO_PUBLISH=0), poll
 * far less often so DTIM10's own cadence dominates; MQTT keepalive (60s)
 * has a wide safety margin against this. When actively testing publish
 * timing (MQTT_AUTO_PUBLISH=1), keep it tight so publishes fire promptly. */
#if MQTT_AUTO_PUBLISH
#define MQTT_PROCESS_INTERVAL_MS 500U
#else
#define MQTT_PROCESS_INTERVAL_MS 10000U
#endif

#define info_printf(msg, ...) printf("MQTT_TEST: " msg, ##__VA_ARGS__)

/* Set by powertest_demo.c once the WLAN CONNECT event / 4-way handshake
 * completes; also used to unwind this task's loops on disconnect. */
extern uint8_t g_wifi_ready;

static PlaintextParams_t s_net_params;
static NetworkContext_t  s_net_ctx;
static MQTTContext_t     s_mqtt_ctx;
static uint8_t           s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static volatile int      s_publish_now;
static uint32_t          s_counter;
static TaskHandle_t      s_task_handle;
static volatile uint8_t  s_started;

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

    if (MQTT_Connect(&s_mqtt_ctx, &conninfo, NULL, MQTT_CONNACK_TIMEOUT_MS, &session_present) != MQTTSuccess) {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
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

static int mqtt_publish(uint32_t counter)
{
    char msg[64];
    int len;
    MQTTPublishInfo_t pub;

    /* Plain printf-style placeholder payload: just an incrementing counter
     * and uptime, no sensor reading involved. */
    len = snprintf(msg, sizeof(msg), "test #%lu uptime=%lums",
                   (unsigned long)counter, (unsigned long)mqtt_get_time_ms());
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

static void mqtt_printf_task(void *arg)
{
    TickType_t last_pub;

    (void)arg;

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
                s_counter++;
                info_printf("publish #%lu\n", (unsigned long)s_counter);
                if (mqtt_publish(s_counter) != 0) {
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

/* Call once WLAN is connected (g_wifi_ready == 1) to start the periodic
 * test publisher. Safe to call more than once - only starts the task the
 * first time. */
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