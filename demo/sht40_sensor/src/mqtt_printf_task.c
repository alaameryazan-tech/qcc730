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
 * DTIM10 sleep and publishes again every MQTT_PUBLISH_INTERVAL_MS (5 min
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

/* TEMP DIAGNOSTIC - point at a local mosquitto on the same WiFi/subnet as
 * the board (192.168.80.163, allow_anonymous) instead of the VPS, to check
 * whether the ~15-20 min disconnect is broker/WAN-path-related or purely
 * device-side: same-subnet traffic never crosses the router's WAN NAT, so
 * this also rules out router NAT/conntrack aging, not just the remote
 * broker itself. Flip MQTT_USE_LOCAL_TEST_BROKER back to 0 (restores the
 * VPS + username/password) once the test is done. */
#define MQTT_USE_LOCAL_TEST_BROKER  1

/* Same broker/topic/QoS/reconnect handling as the SHT40 MQTT publisher. */
#if MQTT_USE_LOCAL_TEST_BROKER
#define MQTT_BROKER                 "192.168.80.163"
#define MQTT_USERNAME               NULL
#define MQTT_PASSWORD               NULL
#else
#define MQTT_BROKER                 "139.162.181.126"
#define MQTT_USERNAME               "AABBCCDD"
#define MQTT_PASSWORD               "KXL8DHPsXggvRELD"
#endif
#define MQTT_PORT                   1883
#define MQTT_CLIENT_ID              "sht40_qcc7030"
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
 * immediately on (re)connect, then on this interval. Shorter than the
 * keepalive above, so publishes still double as the traffic keeping the
 * session alive - no separate idle PINGREQ needed most cycles. */
/* Also doubles as the ONLY wake interval for the whole task once
 * connected/idle - see mqtt_printf_task(). Every call to MQTT_ProcessLoop()
 * needs the radio to come fully active (not just DTIM10's passive listen
 * wake) to service the socket, which draws far more current than idle -
 * confirmed by measurement: even a 10s poll (an earlier, more cautious
 * choice than strictly necessary) was producing a periodic ~30uA->170uA
 * spike every single cycle, all day, whether or not there was anything to
 * do. Neither this nor MQTT_KEEPALIVE_SEC need anywhere near that kind of
 * precision, so the task now sleeps for this entire interval in one shot -
 * a single wake per cycle, the minimum possible for this workload. */
#define MQTT_PUBLISH_INTERVAL_MS    300000U

/* How long to keep polling MQTT_ProcessLoop() after a QoS2 publish, waiting
 * for its PUBCOMP, before giving up and reconnecting - see the QoS2 handling
 * in mqtt_printf_task(). Generous relative to a normal LAN/WAN round trip,
 * but still tiny next to MQTT_PUBLISH_INTERVAL_MS. */
#define MQTT_PUBACK_WAIT_TIMEOUT_MS   5000U
#define MQTT_PUBACK_POLL_INTERVAL_MS   200U

/* Set to 0 to establish WLAN + MQTT and then stay fully silent in DTIM10
 * sleep (no periodic publish at all) - useful for a clean baseline
 * sleep-current measurement with no traffic whatsoever. Default 1: publish
 * an SHT40 reading on connect and every MQTT_PUBLISH_INTERVAL_MS after.
 * Either way, MQTT_ProcessLoop() still runs to service the connection
 * (keepalive, incoming data), and the on-demand "measure" command on
 * sensor/cmd still works, though it may take up to a full
 * MQTT_PUBLISH_INTERVAL_MS to be noticed and acted on - acceptable given
 * the priority here is minimum wake count/power, not responsiveness. */
#define MQTT_AUTO_PUBLISH 1

#define info_printf(msg, ...) printf("MQTT_TEST: " msg, ##__VA_ARGS__)

/* Set by powertest_demo.c once the WLAN CONNECT event / 4-way handshake
 * completes; also used to unwind this task's loops on disconnect. */
extern uint8_t g_wifi_ready;

/* Set true here once mqtt_connect_and_subscribe() first succeeds (CONNACK
 * received); powertest_demo.c's powertest_pm_enable_task() polls this to
 * engage BMPS/DTIM10 sleep as soon as it's safe, instead of guessing with a
 * fixed delay. Left true across later reconnects - only meant to gate that
 * one first entry into DTIM10 sleep, not to track live connection state. */
volatile uint8_t g_mqtt_connected;

static PlaintextParams_t s_net_params;
static NetworkContext_t  s_net_ctx;
static MQTTContext_t     s_mqtt_ctx;
static uint8_t           s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static volatile int      s_publish_now;

/* Outgoing-publish ack bookkeeping, needed by MQTT_InitStatefulQoS() to
 * allow QoS2 publishes (see mqtt_publish()). Only ever one reading in
 * flight at a time, but a couple of spare slots costs nothing. */
#define MQTT_OUTGOING_PUBLISH_RECORDS 2U
static MQTTPubAckInfo_t  s_outgoing_pub_records[MQTT_OUTGOING_PUBLISH_RECORDS];

/* Set when a QoS2 sensor-reading publish is sent, cleared when its PUBCOMP
 * (the final "exactly once, confirmed delivered" step of the QoS2
 * handshake) is seen in mqtt_event_cb(). Checked at the start of the next
 * cycle - see mqtt_publish()/mqtt_printf_task() for why this matters over
 * trusting MQTT_Publish()'s return value alone. */
static volatile uint16_t s_last_pub_packet_id;
static volatile uint8_t  s_awaiting_pub_ack;
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

    /* QoS2 4-packet handshake for our sensor-reading publish: broker sends
     * PUBREC first (received, not yet fully settled), we reply PUBREL
     * (handled internally by MQTT_ProcessLoop()), broker then sends
     * PUBCOMP - THAT's the actual "exactly once, confirmed delivered"
     * signal, not PUBREC. Either way this is a genuine broker-side
     * confirmation, unlike MQTT_Publish()'s return value which only means
     * the local transport accepted the write. See mqtt_publish(). */
    if (pkt->type == MQTT_PACKET_TYPE_PUBCOMP) {
        if (s_awaiting_pub_ack && info->packetIdentifier == s_last_pub_packet_id)
            s_awaiting_pub_ack = 0;
        return;
    }

    /* PUBREC (and anything else that isn't a PUBLISH) falls through to
     * here and is discarded by the mask check below - intentional, we
     * only care about the final PUBCOMP above. */
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

/* Retained keep-alive/liveness publish on MQTT_TOPIC_STATUS, separate from
 * the sensor reading on MQTT_TOPIC_PUB - called once right after connecting
 * (mqtt_connect_and_subscribe()) AND once every MQTT_PUBLISH_INTERVAL_MS
 * cycle thereafter (mqtt_printf_task()), so the retained "connected" value
 * carries a recent timestamp instead of just whatever it was when the
 * session first came up, possibly hours ago. Best-effort (QoS0, like the
 * one-shot online publish this replaces) - a dropped keep-alive isn't worth
 * reconnecting over by itself; MQTT_ProcessLoop()'s own keepalive and the
 * QoS2 sensor publish's ack-wait (see mqtt_publish()) are what actually
 * detect a dead session. */
static void mqtt_publish_online_status(void)
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

    {
        SocketStatus_t sock_status = Plaintext_Connect(&s_net_ctx, &server,
            MQTT_TRANSPORT_TIMEOUT_MS, MQTT_TRANSPORT_TIMEOUT_MS);
        if (sock_status != SOCKETS_SUCCESS) {
            /* Distinguishes "couldn't even open the TCP socket to the
             * broker" (this) from the MQTT-level failures below - see the
             * SocketStatus_t values in sockets_posix.h for what each number
             * means (DNS failure, connect refused/timed out, etc). Was
             * previously silent, indistinguishable from every other
             * "connect failed" cause. */
            info_printf("Plaintext_Connect failed, status=%d\n", (int)sock_status);
            return -1;
        }
    }

    transport.pNetworkContext = &s_net_ctx;
    transport.send = Plaintext_Send;
    transport.recv = Plaintext_Recv;
    transport.writev = NULL;

    netbuf.pBuffer = s_mqtt_buf;
    netbuf.size = sizeof(s_mqtt_buf);

    {
        MQTTStatus_t mqtt_status = MQTT_Init(&s_mqtt_ctx, &transport, mqtt_get_time_ms, mqtt_event_cb, &netbuf);
        if (mqtt_status != MQTTSuccess) {
            info_printf("MQTT_Init failed, status=%d\n", (int)mqtt_status);
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }

    /* Needed for QoS2 publishes (see mqtt_publish()) - tracks unacked
     * outgoing PUBLISHes so a lost one can actually be detected instead of
     * trusting MQTT_Publish()'s return value alone. No incoming QoS>0
     * records needed, our subscribe is QoS0. */
    {
        MQTTStatus_t mqtt_status = MQTT_InitStatefulQoS(&s_mqtt_ctx, s_outgoing_pub_records,
            MQTT_OUTGOING_PUBLISH_RECORDS, NULL, 0);
        if (mqtt_status != MQTTSuccess) {
            info_printf("MQTT_InitStatefulQoS failed, status=%d\n", (int)mqtt_status);
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }
    s_awaiting_pub_ack = 0;

    memset(&conninfo, 0, sizeof(conninfo));
    conninfo.cleanSession = true;
    conninfo.keepAliveSeconds = MQTT_KEEPALIVE_SEC;
    conninfo.pClientIdentifier = MQTT_CLIENT_ID;
    conninfo.clientIdentifierLength = sizeof(MQTT_CLIENT_ID) - 1;
    /* MQTT_USERNAME/PASSWORD are NULL in the MQTT_USE_LOCAL_TEST_BROKER
     * (anonymous) build above - sizeof(MQTT_USERNAME) would be sizeof(a
     * pointer) in that case, not a string length, so length must be 0
     * whenever the pointer itself is NULL. */
    conninfo.pUserName = MQTT_USERNAME;
    conninfo.userNameLength = MQTT_USERNAME ? (sizeof(MQTT_USERNAME) - 1) : 0;
    conninfo.pPassword = MQTT_PASSWORD;
    conninfo.passwordLength = MQTT_PASSWORD ? (sizeof(MQTT_PASSWORD) - 1) : 0;

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

    {
        MQTTStatus_t mqtt_status = MQTT_Connect(&s_mqtt_ctx, &conninfo, &willInfo,
            MQTT_CONNACK_TIMEOUT_MS, &session_present);
        if (mqtt_status != MQTTSuccess) {
            info_printf("MQTT_Connect failed, status=%d\n", (int)mqtt_status);
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }

    /* Publish our own retained "connected" status right away, so the topic
     * reflects reality again after any earlier LWT "disconnected" and
     * doesn't just sit stuck on the last will forever. Re-published every
     * cycle too - see mqtt_publish_online_status()'s other call site in
     * mqtt_printf_task(). */
    mqtt_publish_online_status();

    sub.qos = MQTTQoS0;
    sub.pTopicFilter = MQTT_TOPIC_CMD;
    sub.topicFilterLength = sizeof(MQTT_TOPIC_CMD) - 1;
    sub_id = MQTT_GetPacketId(&s_mqtt_ctx);
    {
        MQTTStatus_t mqtt_status = MQTT_Subscribe(&s_mqtt_ctx, &sub, 1, sub_id);
        if (mqtt_status != MQTTSuccess) {
            info_printf("MQTT_Subscribe failed, status=%d\n", (int)mqtt_status);
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }

    {
        MQTTStatus_t process_status = MQTT_ProcessLoop(&s_mqtt_ctx);
        if (process_status != MQTTSuccess && process_status != MQTTNeedMoreBytes) {
            info_printf("post-subscribe MQTT_ProcessLoop failed, status=%d\n", (int)process_status);
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
    uint16_t packet_id;

    len = snprintf(msg, sizeof(msg), "Temp=%d.%dC Feuchte=%d.%d%%",
                   temp_x10 / 10, temp_x10 % 10, rh_x10 / 10, rh_x10 % 10);
    if (len <= 0 || len >= (int)sizeof(msg))
        return -1;

    memset(&pub, 0, sizeof(pub));
    /* QoS2, not QoS0: a QoS0 MQTT_Publish() returning success only means
     * the local transport accepted the write, not that the broker actually
     * got it - under a degraded DTIM10 link that gap let readings vanish
     * silently (device log said "success", nothing showed up on the
     * broker). QoS2's PUBCOMP is a genuine broker-side "exactly once,
     * confirmed delivered" signal, so a missing one is a real sign
     * something's wrong - see s_awaiting_pub_ack, mqtt_event_cb(), and its
     * check in mqtt_printf_task(). */
    pub.qos = MQTTQoS2;
    pub.retain = false;
    pub.dup = false;
    pub.pTopicName = MQTT_TOPIC_PUB;
    pub.topicNameLength = sizeof(MQTT_TOPIC_PUB) - 1;
    pub.pPayload = msg;
    pub.payloadLength = (size_t)len;

    packet_id = MQTT_GetPacketId(&s_mqtt_ctx);
    if (MQTT_Publish(&s_mqtt_ctx, &pub, packet_id) != MQTTSuccess)
        return -1;

    s_last_pub_packet_id = packet_id;
    s_awaiting_pub_ack = 1;
    return 0;
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

        /* Reaching here means mqtt_connect_and_subscribe() just succeeded
         * (the while loop above only exits early otherwise). Unblocks
         * powertest_pm_enable_task()'s wait for g_mqtt_connected. */
        g_mqtt_connected = 1;

        /* No separate "initial publish" here: the inner loop's first pass
         * (immediately below, before its vTaskDelay) already publishes
         * right after connecting - having both was the bug that caused
         * two back-to-back messages at startup. */
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

            /* Previous cycle's QoS2 reading never reached PUBCOMP, even
             * after a full MQTT_PUBLISH_INTERVAL_MS plus the ProcessLoop()
             * call just above to pick it up - the session looked alive (no
             * error from ProcessLoop) but the broker never actually
             * confirmed that message. Reconnect rather than silently
             * publishing the next reading on a session already known to be
             * dropping data. */
            if (s_awaiting_pub_ack) {
                info_printf("previous reading never acked by broker, reconnecting\n");
                Plaintext_Disconnect(&s_net_ctx);
                s_publish_now = 0;
                break;
            }

#if MQTT_AUTO_PUBLISH
            /* Keep-alive: refresh the retained "connected" status every
             * cycle (see mqtt_publish_online_status()) - separate from the
             * sensor reading below, so MQTT_TOPIC_STATUS always carries a
             * recent timestamp even across many quiet cycles. Inside this
             * #if (not unconditional) so MQTT_AUTO_PUBLISH=0's "stay fully
             * silent, no traffic whatsoever" baseline test mode (see its
             * own comment) still means what it says. */
            mqtt_publish_online_status();

            /* No elapsed-time bookkeeping needed: every wake here already
             * IS the scheduled publish point, since the vTaskDelay below
             * sleeps for exactly one full MQTT_PUBLISH_INTERVAL_MS - this
             * is the ONLY wake in the whole cycle, done deliberately to
             * minimize how often the radio has to come active. An
             * on-demand "measure" command that arrived on sensor/cmd
             * during the sleep (buffered by the socket, picked up by
             * MQTT_ProcessLoop() just above) folds into this same cycle
             * rather than causing an early wake - fine given the
             * priority here is minimum wake count, not responsiveness. */
            {
                int result = sht40_measure_and_publish();

                if (result < 0) {
                    info_printf("publish failed, reconnecting\n");
                    Plaintext_Disconnect(&s_net_ctx);
                    s_publish_now = 0;
                    break;
                }
                /* result == 1 (measurement-only hiccup, already logged
                 * inside the helper): MQTT_ProcessLoop() above already
                 * confirmed the session is fine. There's no faster retry
                 * anymore - a bad reading now costs a full
                 * MQTT_PUBLISH_INTERVAL_MS (5 min) wait before the next
                 * attempt, traded deliberately for the lower wake
                 * frequency. */

                /* result == 0: a QoS2 publish just went out (see
                 * mqtt_publish()), which needs PUBREC/PUBREL/PUBCOMP round
                 * trips to actually settle - one MQTT_ProcessLoop() call
                 * isn't enough time for that multi-step handshake to
                 * finish, especially right after the radio wakes from a
                 * long BMPS/DTIM10 sleep. Poll for a few seconds here so a
                 * healthy link gets a fair chance to complete it before we
                 * sleep - the alternative was declaring readings "never
                 * acked" (and reconnecting) when really they just hadn't
                 * had time to be acked yet. If it's still not settled after
                 * this, the link is genuinely bad and reconnecting now
                 * beats silently burning a full MQTT_PUBLISH_INTERVAL_MS
                 * asleep unable to deliver anything. */
                if (s_awaiting_pub_ack) {
                    uint32_t waited_ms = 0;
                    uint8_t  session_lost = 0;

                    while (s_awaiting_pub_ack && !session_lost && waited_ms < MQTT_PUBACK_WAIT_TIMEOUT_MS) {
                        vTaskDelay(pdMS_TO_TICKS(MQTT_PUBACK_POLL_INTERVAL_MS));
                        waited_ms += MQTT_PUBACK_POLL_INTERVAL_MS;

                        /* A non-success status here isn't necessarily about
                         * our PUBCOMP specifically - MQTTKeepAliveTimeout
                         * (10) in particular means coreMQTT's own internal
                         * PINGREQ never got a PINGRESP within its fixed
                         * MQTT_PINGRESP_TIMEOUT_MS (5000ms, core_mqtt_config.h)
                         * - a real broker round-trip stall, whatever the
                         * cause. Either way, ProcessLoop() itself is saying
                         * the session's bad, so treat it as such. */
                        st = MQTT_ProcessLoop(&s_mqtt_ctx);
                        if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                            info_printf("MQTT_ProcessLoop failed while settling (status=%d), reconnecting\n", (int)st);
                            session_lost = 1;
                        }
                    }

                    if (session_lost || s_awaiting_pub_ack) {
                        if (!session_lost) {
                            info_printf("reading still not acked after %ums, reconnecting\n",
                                (unsigned)waited_ms);
                        }
                        s_awaiting_pub_ack = 0;
                        Plaintext_Disconnect(&s_net_ctx);
                        s_publish_now = 0;
                        break;
                    }
                }
            }
#endif /* MQTT_AUTO_PUBLISH */
            s_publish_now = 0;

            /* Single wake per cycle - the minimum possible for this
             * workload. MQTT_PUBLISH_INTERVAL_MS doubles as the
             * MQTT_ProcessLoop() servicing cadence too (keepalive
             * PINGREQ, if the publish above didn't already reset it) -
             * no reason to wake more often than the publish schedule
             * itself requires. */
            vTaskDelay(pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL_MS));
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