/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "nt_osal.h"
#include "safeAPI.h"
#include "timer.h"
#include "core_mqtt.h"
#include "plaintext_posix.h"

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */

#define SHT40_I2C_ADDR           0x44
#define SHT40_CMD_MEAS           0xFD

#define MQTT_BROKER              "139.162.181.126"
#define MQTT_PORT                1883
#define MQTT_CLIENT_ID           "sht40_qcc7030"
#define MQTT_USERNAME            "AABBCCDD"
#define MQTT_PASSWORD            "KXL8DHPsXggvRELD"
#define MQTT_TOPIC_PUB           "sensor/sht40"
#define MQTT_TOPIC_CMD           "sensor/cmd"
#define MQTT_NETWORK_BUF_SIZE    1024U
#define MQTT_KEEPALIVE_SEC       60U
#define MQTT_CONNACK_TIMEOUT_MS  5000U
#define MQTT_TRANSPORT_TIMEOUT_MS 1000U
#define MQTT_MEASURE_INTERVAL_MS 5000U
#define MQTT_PROCESS_INTERVAL_MS 500U

/* ------------------------------------------------------------------ */
/*  Shared WiFi-ready flag (owned by qcli_demo_main.c)                */
/* ------------------------------------------------------------------ */

extern volatile qbool_t g_wifi_auto_connected;

/* ------------------------------------------------------------------ */
/*  Static state                                                        */
/* ------------------------------------------------------------------ */

static PlaintextParams_t s_net_params;
static NetworkContext_t  s_net_ctx;
static MQTTContext_t     s_mqtt_ctx;
static uint8_t           s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static volatile int      s_measure_now;

/* ------------------------------------------------------------------ */
/*  I2C / SHT40                                                        */
/* ------------------------------------------------------------------ */

static void sht40_open(void)
{
    qapi_I2CM_Config_t cfg = { .Blocking = TRUE, .Dma = FALSE };
    qapi_I2CM_Open(QAPI_I2C_INSTANCE_SE0_E, &cfg);
}

static int sht40_measure(int *temp_x10, int *rh_x10)
{
    uint8_t cmd = SHT40_CMD_MEAS;
    uint8_t rx[6];

    qapi_I2CM_Transfer_Config_t xcfg = {
        .BusFreqKHz             = 400,
        .SlaveAddress           = SHT40_I2C_ADDR,
        .SlaveMaxClockStretchUs = 0,
        .Delay                  = 0,
        .NoiseReject            = 0,
    };

    qapi_I2CM_Descriptor_t wr = {
        .Buffer      = &cmd,
        .Length      = 1,
        .Transferred = 0,
        .Flags       = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_WRITE,
    };
    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &wr, 1, NULL, NULL) != QAPI_OK)
        return -1;

    vTaskDelay(pdMS_TO_TICKS(10));

    qapi_I2CM_Descriptor_t rd = {
        .Buffer      = rx,
        .Length      = 6,
        .Transferred = 0,
        .Flags       = QAPI_I2C_FLAG_START | QAPI_I2C_FLAG_STOP | QAPI_I2C_FLAG_READ,
    };
    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &rd, 1, NULL, NULL) != QAPI_OK)
        return -1;

    uint16_t t_raw  = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t rh_raw = ((uint16_t)rx[3] << 8) | rx[4];

    *temp_x10 = -450 + (int)(1750 * (uint32_t)t_raw  / 65535);
    *rh_x10   =  -60 + (int)(1250 * (uint32_t)rh_raw / 65535);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  MQTT transport / callbacks                                          */
/* ------------------------------------------------------------------ */

static uint32_t mqtt_get_time_ms(void)
{
    return hres_timer_curr_time_ms();
}

static void mqtt_event_cb(MQTTContext_t *ctx,
                          MQTTPacketInfo_t *pkt,
                          MQTTDeserializedInfo_t *info)
{
    (void)ctx;

    if ((pkt->type & 0xF0U) != MQTT_PACKET_TYPE_PUBLISH)
        return;

    MQTTPublishInfo_t *pub = info->pPublishInfo;
    if (!pub)
        return;

    /* Check topic == "sensor/cmd" */
    if (pub->topicNameLength != (sizeof(MQTT_TOPIC_CMD) - 1) ||
        strncmp(MQTT_TOPIC_CMD, pub->pTopicName, pub->topicNameLength) != 0)
        return;

    /* Check payload == "measure" */
    if (pub->payloadLength == (sizeof("measure") - 1) &&
        strncmp("measure", (const char *)pub->pPayload, pub->payloadLength) == 0)
    {
        s_measure_now = 1;
    }
}

static int mqtt_connect_and_subscribe(void)
{
    s_net_params.socketDescriptor = -1;
    s_net_ctx.pParams = &s_net_params;

    ServerInfo_t server = {
        .pHostName      = MQTT_BROKER,
        .hostNameLength = sizeof(MQTT_BROKER) - 1,
        .port           = MQTT_PORT,
    };

    if (Plaintext_Connect(&s_net_ctx, &server,
                          MQTT_TRANSPORT_TIMEOUT_MS,
                          MQTT_TRANSPORT_TIMEOUT_MS) != SOCKETS_SUCCESS)
        return -1;

    TransportInterface_t transport = {
        .pNetworkContext = &s_net_ctx,
        .send            = Plaintext_Send,
        .recv            = Plaintext_Recv,
        .writev          = NULL,
    };
    MQTTFixedBuffer_t netbuf = {
        .pBuffer = s_mqtt_buf,
        .size    = sizeof(s_mqtt_buf),
    };

    if (MQTT_Init(&s_mqtt_ctx, &transport, mqtt_get_time_ms,
                  mqtt_event_cb, &netbuf) != MQTTSuccess)
    {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    MQTTConnectInfo_t conninfo = {
        .cleanSession           = true,
        .keepAliveSeconds       = MQTT_KEEPALIVE_SEC,
        .pClientIdentifier      = MQTT_CLIENT_ID,
        .clientIdentifierLength = sizeof(MQTT_CLIENT_ID) - 1,
        .pUserName              = MQTT_USERNAME,
        .userNameLength         = sizeof(MQTT_USERNAME) - 1,
        .pPassword              = MQTT_PASSWORD,
        .passwordLength         = sizeof(MQTT_PASSWORD) - 1,
    };
    bool session_present;
    if (MQTT_Connect(&s_mqtt_ctx, &conninfo, NULL,
                     MQTT_CONNACK_TIMEOUT_MS, &session_present) != MQTTSuccess)
    {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    MQTTSubscribeInfo_t sub = {
        .qos              = MQTTQoS0,
        .pTopicFilter     = MQTT_TOPIC_CMD,
        .topicFilterLength = sizeof(MQTT_TOPIC_CMD) - 1,
    };
    uint16_t sub_id = MQTT_GetPacketId(&s_mqtt_ctx);
    if (MQTT_Subscribe(&s_mqtt_ctx, &sub, 1, sub_id) != MQTTSuccess)
    {
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    /* Consume SUBACK */
    MQTT_ProcessLoop(&s_mqtt_ctx);

    printf("MQTT: connected to %s:%d, subscribed to %s\r\n",
           MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_CMD);
    return 0;
}

static int mqtt_publish(int temp_x10, int rh_x10)
{
    char msg[48];
    int len = snprintf(msg, sizeof(msg), "Temp=%d.%dC Feuchte=%d.%d%%",
                       temp_x10 / 10, temp_x10 % 10,
                       rh_x10  / 10, rh_x10  % 10);
    if (len <= 0 || len >= (int)sizeof(msg))
        return -1;

    MQTTPublishInfo_t pub = {
        .qos             = MQTTQoS0,
        .retain          = false,
        .dup             = false,
        .pTopicName      = MQTT_TOPIC_PUB,
        .topicNameLength = sizeof(MQTT_TOPIC_PUB) - 1,
        .pPayload        = msg,
        .payloadLength   = (size_t)len,
    };
    return MQTT_Publish(&s_mqtt_ctx, &pub, 0) == MQTTSuccess ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/*  Task entry                                                          */
/* ------------------------------------------------------------------ */

void sht40_task_main(void *arg)
{
    (void)arg;

    sht40_open();

    /* Wait for WiFi association */
    while (!g_wifi_auto_connected)
        vTaskDelay(pdMS_TO_TICKS(500));

    /* Give DHCP time to complete after WiFi link is up */
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        printf("MQTT: connecting to %s:%d...\r\n", MQTT_BROKER, MQTT_PORT);
        while (mqtt_connect_and_subscribe() != 0) {
            printf("MQTT: connect failed, retry in 5s\r\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        TickType_t last_pub = xTaskGetTickCount();
        s_measure_now = 0;

        /* Process loop: run every 500ms, publish every 5s or on demand */
        for (;;) {
            MQTTStatus_t st = MQTT_ProcessLoop(&s_mqtt_ctx);
            if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                printf("MQTT: session lost (%d), reconnecting\r\n", (int)st);
                Plaintext_Disconnect(&s_net_ctx);
                break;
            }

            TickType_t now   = xTaskGetTickCount();
            uint32_t elapsed = (uint32_t)((now - last_pub) * portTICK_PERIOD_MS);
            int do_measure   = s_measure_now || (elapsed >= MQTT_MEASURE_INTERVAL_MS);

            if (do_measure) {
                int temp_x10 = 0, rh_x10 = 0;
                if (sht40_measure(&temp_x10, &rh_x10) == 0) {
                    printf("Temp=%d.%dC Feuchte=%d.%d%%\r\n",
                           temp_x10 / 10, temp_x10 % 10,
                           rh_x10  / 10, rh_x10  % 10);
                    if (mqtt_publish(temp_x10, rh_x10) != 0) {
                        printf("MQTT: publish failed, reconnecting\r\n");
                        Plaintext_Disconnect(&s_net_ctx);
                        s_measure_now = 0;
                        break;
                    }
                }
                last_pub      = xTaskGetTickCount();
                s_measure_now = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(MQTT_PROCESS_INTERVAL_MS));
        }

        /* Brief pause before reconnect */
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
