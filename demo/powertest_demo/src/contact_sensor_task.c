/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Vishay VCNL3020 IR proximity sensor, used as a "contact" (open/closed)
 * sensor over I2C, published to MQTT and logged to the console on every
 * OPEN<->CLOSED transition - NOT on every poll. The sensor is still
 * sampled every CONTACT_SENSOR_READ_INTERVAL_MS internally (needed for the
 * median filter/hysteresis to work), it just stays quiet as long as the
 * status doesn't change.
 *
 * I2C read/write pattern (open, close-before-reopen, two separate
 * START+STOP transfers for a register read) is copied from the working
 * SHT40 driver in mqtt_printf_task.c in this same demo - that pattern is
 * already confirmed to work with the QAPI I2C driver/instance used here.
 * The MQTT connect/publish/reconnect logic below is the same trimmed down
 * to publish-only (no subscribe) - same broker/credentials as that task.
 *
 * The I2C side is independent of WLAN and keeps sampling/printing/tracking
 * status changes even with no WiFi at all; only the MQTT publish is gated
 * on g_wifi_ready. Started unconditionally from Initialize_powertest_Demo()
 * in powertest_demo.c. */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "nt_timer.h"
#include "core_mqtt.h"
#include "plaintext_posix.h"

/* Set by powertest_demo.c once the WLAN CONNECT event / 4-way handshake
 * completes; also used to unwind the MQTT connection on disconnect. */
extern uint8_t g_wifi_ready;

/* Same broker/credentials as mqtt_printf_task.c's SHT40 publisher - a
 * different MQTT_CLIENT_ID so the two can hold separate broker sessions if
 * both happen to be running at once. */
#define MQTT_BROKER               "139.162.181.126"
#define MQTT_PORT                 1883
#define MQTT_CLIENT_ID            "contact_sensor_qcc7030"
#define MQTT_USERNAME             "AABBCCDD"
#define MQTT_PASSWORD             "KXL8DHPsXggvRELD"
/* Payload is just "OPEN" or "CLOSED", published retained so an MQTTX
 * client that (re)subscribes later immediately sees the current status
 * instead of waiting for the next transition. */
#define MQTT_TOPIC_PUB            "sensor/contact"
/* Retained "connected"/non-retained LWT "disconnected", same pattern as
 * mqtt_printf_task.c's status topic. */
#define MQTT_TOPIC_STATUS         "sensor/contact/status"
#define MQTT_LWT_MESSAGE          "qcc disconnected"
#define MQTT_ONLINE_MESSAGE       "qcc connected"
#define MQTT_NETWORK_BUF_SIZE     512U
#define MQTT_KEEPALIVE_SEC        60U
#define MQTT_CONNACK_TIMEOUT_MS   5000U
#define MQTT_TRANSPORT_TIMEOUT_MS 1000U
/* Minimum gap between MQTT (re)connect attempts, so a broker/network
 * outage doesn't get hammered with an attempt every single 500ms I2C poll. */
#define MQTT_RECONNECT_INTERVAL_MS 5000U

/* ------------------------------------------------------------------ */
/*  VCNL3020 register map                                              */
/*  (Vishay VCNL3020 datasheet: https://www.vishay.com/docs/84150/vcnl3020.pdf) */
/* ------------------------------------------------------------------ */

/* Fixed 7-bit I2C slave address for every VCNL3020 - there is no address
 * pin/strap to change it, so if you have more than one on a bus you need
 * an I2C mux/expander. */
#define VCNL3020_I2C_ADDR              0x13

#define VCNL3020_REG_COMMAND           0x80  /* control + status */
#define VCNL3020_REG_IR_LED_CURRENT    0x83  /* IR LED drive current, 10 mA/step, 0-20 (0-200 mA) */
#define VCNL3020_REG_PROX_RESULT_MSB   0x87
#define VCNL3020_REG_PROX_RESULT_LSB   0x88

#define VCNL3020_CMD_PROX_OD_BIT       (1U << 3)  /* write 1: trigger one on-demand proximity measurement */
#define VCNL3020_CMD_PROX_DATA_RDY_BIT (1U << 5)  /* read-only: 1 once the result registers hold fresh data */

/* IR LED drive current: 0-20 in 10 mA steps (0 = 0 mA, 20 = 200 mA/max
 * range). Higher current = more reflected IR = larger raw counts and
 * longer detection range, at the cost of more current draw per
 * measurement. 20 (max) is used here since a contact sensor only measures
 * on demand, not continuously - tune down if your target is always very
 * close (a few mm) and you are getting saturated/near-identical readings
 * for both open and closed. */
#define CONTACT_SENSOR_LED_CURRENT_STEP 20

/* How often a reading is taken and printed. */
#define CONTACT_SENSOR_READ_INTERVAL_MS 500U

/* TUNE ME: raw proximity counts (0-65535 as returned by this sensor) that
 * bound a hysteresis band, evaluated against a 3-sample MEDIAN (see
 * contact_median3() below), not the raw instantaneous reading:
 *   median >= HIGH  -> report CLOSED
 *   median <  LOW    -> report OPEN
 *   in between        -> keep whatever status was already reported
 * (prevents flicker right at the boundary).
 *
 * IMPORTANT: LOW must be safely ABOVE your sensor's normal resting/open
 * baseline, not below it - if LOW sits at or under the open-state noise
 * floor, the median can never drop below LOW again once something has
 * pushed it CLOSED, and status latches CLOSED forever (this was exactly
 * the bug in the previous LOW=2000 value here: measured open baseline was
 * ~2350-2665, i.e. already above LOW, so it could close but never reopen).
 *
 * Current values assume an open baseline in the low thousands and a
 * closed/near reading in the tens of thousands (typical for this sensor at
 * a few cm, per observed logs) - re-check against YOUR printed
 * `proximity=`/`median=` values with the contact physically open, then
 * physically closed, and adjust: LOW should sit above the highest value
 * you see at rest, HIGH below the lowest value you see when closed. */
#define CONTACT_SENSOR_PROXIMITY_THRESHOLD_LOW  3000U
#define CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH 5000U

#define info_printf(msg, ...) printf("CONTACT_SENSOR: " msg, ##__VA_ARGS__)

static uint8_t     s_sensor_ready;
static uint8_t     s_contact_closed; /* last reported status, for hysteresis */
static TaskHandle_t s_task_handle;
static volatile uint8_t s_started;

/* ------------------------------------------------------------------ */
/*  MQTT (publish-only)                                                */
/* ------------------------------------------------------------------ */

static PlaintextParams_t s_net_params;
static NetworkContext_t  s_net_ctx;
static MQTTContext_t     s_mqtt_ctx;
static uint8_t           s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static uint8_t           s_mqtt_connected;
static uint8_t           s_mqtt_needs_publish; /* current status not yet sent on this connection */
static uint32_t          s_mqtt_next_attempt_ms;

static uint32_t mqtt_get_time_ms(void)
{
    return hres_timer_curr_time_ms();
}

/* No subscriptions on this client, so nothing to do with incoming
 * packets - MQTT_Init() still requires a valid callback pointer. */
static void mqtt_event_cb(MQTTContext_t *ctx, MQTTPacketInfo_t *pkt, MQTTDeserializedInfo_t *info)
{
    (void)ctx;
    (void)pkt;
    (void)info;
}

static int mqtt_connect(void)
{
    TransportInterface_t transport;
    MQTTFixedBuffer_t netbuf;
    MQTTConnectInfo_t conninfo;
    MQTTPublishInfo_t willInfo;
    MQTTPublishInfo_t onlineInfo;
    ServerInfo_t server;
    bool session_present;

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

    memset(&onlineInfo, 0, sizeof(onlineInfo));
    onlineInfo.qos = MQTTQoS0;
    onlineInfo.retain = true;
    onlineInfo.pTopicName = MQTT_TOPIC_STATUS;
    onlineInfo.topicNameLength = sizeof(MQTT_TOPIC_STATUS) - 1;
    onlineInfo.pPayload = MQTT_ONLINE_MESSAGE;
    onlineInfo.payloadLength = sizeof(MQTT_ONLINE_MESSAGE) - 1;
    MQTT_Publish(&s_mqtt_ctx, &onlineInfo, 0);

    info_printf("MQTT connected to %s:%d, publishing to %s\n", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_PUB);
    return 0;
}

static void mqtt_teardown(void)
{
    Plaintext_Disconnect(&s_net_ctx);
    s_mqtt_connected = 0;
    /* Force a fresh publish of the current status once reconnected, since
     * a brand new connection has no idea what we last sent. */
    s_mqtt_needs_publish = 1;
}

static int mqtt_publish_status(uint8_t closed)
{
    MQTTPublishInfo_t pub;
    const char *payload = closed ? "CLOSED" : "OPEN";

    memset(&pub, 0, sizeof(pub));
    pub.qos = MQTTQoS0;
    pub.retain = true; /* late subscribers immediately see current status */
    pub.dup = false;
    pub.pTopicName = MQTT_TOPIC_PUB;
    pub.topicNameLength = sizeof(MQTT_TOPIC_PUB) - 1;
    pub.pPayload = payload;
    pub.payloadLength = strlen(payload);

    return (MQTT_Publish(&s_mqtt_ctx, &pub, 0) == MQTTSuccess) ? 0 : -1;
}

/* Called once per task loop iteration: services the existing connection,
 * (re)connects if needed (rate-limited to MQTT_RECONNECT_INTERVAL_MS), and
 * flushes a pending status publish. Never blocks the I2C polling cadence
 * for more than one MQTT_TRANSPORT_TIMEOUT_MS. */
static void mqtt_service(uint32_t now_ms)
{
    if (!g_wifi_ready) {
        if (s_mqtt_connected)
            mqtt_teardown();
        return;
    }

    if (!s_mqtt_connected) {
        if (now_ms < s_mqtt_next_attempt_ms)
            return;
        if (mqtt_connect() == 0) {
            s_mqtt_connected = 1;
            s_mqtt_needs_publish = 1; /* push current status right away */
        } else {
            s_mqtt_next_attempt_ms = now_ms + MQTT_RECONNECT_INTERVAL_MS;
        }
        return;
    }

    {
        MQTTStatus_t st = MQTT_ProcessLoop(&s_mqtt_ctx);
        if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
            info_printf("MQTT session lost (%d), reconnecting\n", (int)st);
            mqtt_teardown();
            s_mqtt_next_attempt_ms = now_ms + MQTT_RECONNECT_INTERVAL_MS;
            return;
        }
    }

    if (s_mqtt_needs_publish) {
        if (mqtt_publish_status(s_contact_closed) == 0) {
            s_mqtt_needs_publish = 0;
        } else {
            info_printf("MQTT publish failed, reconnecting\n");
            mqtt_teardown();
            s_mqtt_next_attempt_ms = now_ms + MQTT_RECONNECT_INTERVAL_MS;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  I2C register access                                                */
/* ------------------------------------------------------------------ */

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

    return (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &wr, 1, NULL, NULL) == QAPI_OK) ? 0 : -1;
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
    /* Two separate START+STOP transfers (address-write, then data-read)
     * rather than one repeated-start transfer, matching the pattern
     * already proven to work against this I2C instance/driver. */
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

    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &wr, 1, NULL, NULL) != QAPI_OK)
        return -1;

    if (qapi_I2CM_Transfer(QAPI_I2C_INSTANCE_SE0_E, &xcfg, &rd, 1, NULL, NULL) != QAPI_OK)
        return -1;

    return 0;
}

static int vcnl3020_open(void)
{
    qapi_I2CM_Config_t cfg = { .Blocking = TRUE, .Dma = FALSE };
    qapi_Status_t status;

    /* Close first in case the instance is already open (same reasoning as
     * sht40_open() elsewhere in this demo: qapi_I2CM_Open() fails if
     * called on an already-open instance, and closing an instance that
     * isn't open is expected to just fail harmlessly). */
    qapi_I2CM_Close(QAPI_I2C_INSTANCE_SE0_E);

    status = qapi_I2CM_Open(QAPI_I2C_INSTANCE_SE0_E, &cfg);
    if (status != QAPI_OK) {
        info_printf("I2C open failed, status=%d\n", (int)status);
        s_sensor_ready = 0;
        return -1;
    }

    if (vcnl3020_write_reg(VCNL3020_REG_IR_LED_CURRENT, CONTACT_SENSOR_LED_CURRENT_STEP) != 0) {
        info_printf("VCNL3020 LED current write failed\n");
        s_sensor_ready = 0;
        return -1;
    }

    s_sensor_ready = 1;
    return 0;
}

/* Trigger one on-demand proximity measurement and read the 16-bit raw
 * result. Blocks (via vTaskDelay polling) until the sensor reports the
 * result ready or ~200 ms elapses. */
static int vcnl3020_read_proximity(uint16_t *proximity)
{
    uint8_t cmd_reg = 0;
    uint8_t msb, lsb;
    int retries = 20; /* 20 x 10 ms = 200 ms worst case */

    if (!s_sensor_ready)
        return -1;

    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND, VCNL3020_CMD_PROX_OD_BIT) != 0)
        return -1;

    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (vcnl3020_read_reg(VCNL3020_REG_COMMAND, &cmd_reg) != 0)
            return -1;
    } while (!(cmd_reg & VCNL3020_CMD_PROX_DATA_RDY_BIT) && --retries > 0);

    if (!(cmd_reg & VCNL3020_CMD_PROX_DATA_RDY_BIT)) {
        info_printf("proximity measurement timed out\n");
        return -1;
    }

    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_MSB, &msb) != 0)
        return -1;
    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_LSB, &lsb) != 0)
        return -1;

    *proximity = ((uint16_t)msb << 8) | lsb;
    return 0;
}

/* Median of the last 3 raw readings. A median rejects a single spurious
 * outlier sample (e.g. one 0xFFFF glitch from ambient IR or a marginal I2C
 * transfer) without smearing out a real, sustained proximity change the
 * way a running average would - a genuine open<->closed transition still
 * shows up within 2 samples (1s at the default read interval). */
static uint16_t contact_median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

/* Hysteresis: only flips s_contact_closed when the median clearly crosses
 * into the other band, otherwise holds the last reported status. */
static const char *contact_classify(uint16_t median)
{
    if (median >= CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH) {
        s_contact_closed = 1;
    } else if (median < CONTACT_SENSOR_PROXIMITY_THRESHOLD_LOW) {
        s_contact_closed = 0;
    }
    return s_contact_closed ? "CLOSED" : "OPEN";
}

/* ------------------------------------------------------------------ */
/*  Task                                                                */
/* ------------------------------------------------------------------ */

static void contact_sensor_task(void *arg)
{
    uint16_t hist[3] = { 0, 0, 0 };
    uint8_t hist_count = 0;
    /* Sentinel (neither 0 nor 1) so the very first classification always
     * prints once at boot, establishing a known starting line in the log -
     * after that, only actual OPEN<->CLOSED transitions print. */
    uint8_t last_printed_closed = 0xFF;

    (void)arg;

    while (vcnl3020_open() != 0) {
        info_printf("retrying I2C open in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    info_printf("VCNL3020 ready on I2C addr 0x%02X\n", VCNL3020_I2C_ADDR);

    for (;;) {
        uint16_t proximity = 0;
        /* Uptime-since-boot timestamp in ms - this board has no RTC/wall
         * clock set up by default, so a monotonic uptime stamp (same
         * clock source the MQTT task uses for its own timestamps) is what
         * is actually available here. */
        uint32_t now_ms = hres_timer_curr_time_ms();

        mqtt_service(now_ms);

        if (vcnl3020_read_proximity(&proximity) == 0) {
            uint16_t median;
            const char *status;

            hist[0] = hist[1];
            hist[1] = hist[2];
            hist[2] = proximity;
            if (hist_count < 3)
                hist_count++;
            /* Pad with the newest sample until 3 real readings are in -
             * median of (x, x, new) still favors the new one on boot. */
            median = contact_median3(hist_count < 3 ? proximity : hist[0], hist[1], hist[2]);
            status = contact_classify(median); /* updates s_contact_closed */

            if (s_contact_closed != last_printed_closed) {
                info_printf("[%lu.%03lus] status changed: %s (raw=%u median=%u)\n",
                    (unsigned long)(now_ms / 1000), (unsigned long)(now_ms % 1000),
                    status, (unsigned)proximity, (unsigned)median);
                last_printed_closed = s_contact_closed;
                s_mqtt_needs_publish = 1; /* mqtt_service() sends it once connected */
            }
        } else {
            info_printf("[%lu.%03lus] read failed, re-opening I2C\n",
                (unsigned long)(now_ms / 1000), (unsigned long)(now_ms % 1000));
            /* Re-open in case the I2C peripheral wedged - same recovery
             * pattern the SHT40 task uses on every cycle. */
            vcnl3020_open();
        }

        vTaskDelay(pdMS_TO_TICKS(CONTACT_SENSOR_READ_INTERVAL_MS));
    }
}

/* Safe to call more than once - only starts the task the first time. */
void contact_sensor_task_start(void)
{
    if (s_started)
        return;

    if (nt_qurt_thread_create(contact_sensor_task, "contact_sensor_task", 4096, NULL, 5, &s_task_handle) == pdPASS) {
        s_started = 1;
    } else {
        info_printf("task create fail\n");
    }
}
