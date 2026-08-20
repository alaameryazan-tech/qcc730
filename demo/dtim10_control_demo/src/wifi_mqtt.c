/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* CONTROL BUILD (2026-08-20) - the ONLY job of this whole demo is: connect
 * to WLAN, connect to MQTT, engage DTIM10/BMPS 15s later, then idle. No
 * sensor, no GPIO, no I2C, no pin-22/EXT_WAKEUP handling, no per-event
 * publish - nothing but WLAN+MQTT+DTIM10.
 *
 * Exists to answer one question directly on hardware: does a build with
 * ZERO sensor code show the same ~30-40s power-spike cadence that
 * demo/vcnl3020_test_demo shows, on the same AP/network/DTIM10 config? A
 * hardware capture on that demo (2026-08-20, see wifi_mqtt.c's
 * VCNL3020_MQTT_LOG_ENABLE comment there) already showed 13/13 steady-state
 * BMPS wakes over 9+ minutes were exit_reason=2 (EXIT_REASON_TIM_UC / beacon
 * -miss recovery, see the WLAN firmware's own "Exit: reason=2 run_bmiss=.."
 * log lines) with ZERO sensor/I2C activity on any of them - i.e. the wake
 * pattern already didn't depend on the sensor code. This demo removes that
 * code entirely so the comparison has nothing left to argue about: same
 * WLAN config (DTIM10, same AP/credentials, same BMPS idle-timeout - see
 * dtim10_bmps_enable()), same measurement conditions, zero sensor.
 *
 * WLAN bring-up (station mode, WPA2-PSK/AES, DHCP) and MQTT connect
 * mechanics are copied as-is from demo/vcnl3020_test_demo/src/wifi_mqtt.c
 * (which itself matches demo/mqtt_demo/src/mqtt_demo.c's shell/console-free
 * pattern) - same broker/account, same SSID/passphrase (this SDK's standard
 * dev AP), same DTIM10 listen interval and BMPS idle-timeout (none - see
 * dtim10_bmps_enable()) as that demo's current (2026-08-20) fixed config.
 * The only MQTT traffic this demo ever sends is one retained "online"
 * publish right after connecting (so it shows up in MQTTX like every other
 * demo on this broker) and the periodic keepalive - nothing event-driven,
 * because there is no event source. */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_wlan.h"
#include "qurt_internal.h"
#include "safeAPI.h"
#include "ip_addr.h"
#include "netif.h"
#include "dhcp.h"
#include "network_al.h"

#include "core_mqtt.h"
#include "plaintext_posix.h"
#include "nt_timer.h"

/* DTIM10/BMPS control - qapi_pm_enable()/qapi_bmps_cfg() from
 * qapi_lowpower.h, WMI_BMPS_IGNORE_BCMC + wmi_cmd_send() from wmi.h/
 * wmi_api.h - same headers demo/vcnl3020_test_demo/src/wifi_mqtt.c uses for
 * the same calls. */
#include "qapi_lowpower.h"
#include "wmi.h"
#include "wmi_api.h"
#include "wlan_power.h" /* EXIT_REASON_* - see bmps_wake_probe_cb() */

#include "wifi_mqtt.h"

#ifndef DEV_STA_ID
#define DEV_STA_ID 1
#endif

/* Same dev AP as demo/vcnl3020_test_demo and demo/mqtt_demo - this SDK's
 * standard test network. */
#define WIFI_SSID                   "FRITZ!Box7590AX"
#define WIFI_PASSPHRASE              "Tech91847"

/* Same broker/account as demo/vcnl3020_test_demo/src/wifi_mqtt.c - distinct
 * client ID/topic so both can run (at different times) without colliding on
 * the broker session. */
#define MQTT_BROKER                  "139.162.181.126"
#define MQTT_PORT                    1883
#define MQTT_USERNAME                "AABBCCDD"
#define MQTT_PASSWORD                "KXL8DHPsXggvRELD"
#define MQTT_CLIENT_ID               "dtim10_control_qcc7030"
#define MQTT_TOPIC_STATUS            "sensor/dtim10_control/status"
#define MQTT_ONLINE_MESSAGE          "qcc connected (no sensor - DTIM10 control build)"
#define MQTT_NETWORK_BUF_SIZE        256U
/* 600s (10 min), same value/reasoning as demo/vcnl3020_test_demo/src/
 * wifi_mqtt.c: fewer PINGREQ/PINGRESP bursts means fewer interruptions to
 * DTIM10/BMPS idle sleep. Also doubles as this task's ONLY scheduled wake
 * interval - there is no event source to preempt it, unlike the VCNL3020
 * demo's publish queue. */
#define MQTT_KEEPALIVE_SEC           600U
#define MQTT_CONNACK_TIMEOUT_MS      5000U
#define MQTT_TRANSPORT_TIMEOUT_MS    1000U
#define MQTT_HEARTBEAT_INTERVAL_MS   600000U

/* How long wlan_associate() waits for a CONNECT event before re-issuing
 * qapi_WLAN_Commit(). */
#define WLAN_CONNECT_WAIT_MS         15000U

/* Listen interval = 1000 TU = 10 x 100 TU (default AP beacon interval) -
 * "DTIM10", the value demo/vcnl3020_test_demo/src/wifi_mqtt.c is currently
 * (2026-08-20) set to and the only one confirmed clean (zero beacon-miss
 * escalations over a 20+ min run) on this AP - see that file's
 * VCNL3020_DTIM10_LISTEN_INTERVAL_TU comment for the full DTIM10/13/15
 * history. Set before every qapi_WLAN_Commit(), see wlan_associate(). */
#define DTIM10_LISTEN_INTERVAL_TU    1000U
/* Delay after WLAN associates before engaging DTIM10/BMPS - same value as
 * demo/vcnl3020_test_demo/src/wifi_mqtt.c's VCNL3020_BMPS_ENTER_DELAY_MS. */
#define BMPS_ENTER_DELAY_MS          15000U

/* Master log switch - OFF by default, same reasoning as demo/
 * vcnl3020_test_demo/src/wifi_mqtt.c's VCNL3020_MQTT_LOG_ENABLE: every
 * printf() costs real UART/CPU-active time on every BMPS wake, which is
 * exactly the kind of overhead this control build exists to NOT have. Turn
 * on temporarily to watch exit_reason on each bmps_wake line (see
 * bmps_wake_probe_cb()) if you want to compare wake cadence/exit_reason
 * directly against the VCNL3020 demo's own (2026-08-20) hardware capture. */
#define LOG_ENABLE                   0

#if LOG_ENABLE
#define info_printf(msg, ...) printf("DTIM10_CTRL: " msg, ##__VA_ARGS__)
#else
#define info_printf(msg, ...) do {} while (0)
#endif

/* Always-on - real failure conditions only, never gated by LOG_ENABLE. */
#define err_printf(msg, ...) printf("DTIM10_CTRL: " msg, ##__VA_ARGS__)

static qbool_t s_wlan_enabled;
static qbool_t s_wifi_connected;
static qbool_t s_bmps_active;
static TaskHandle_t s_task_handle;
static PlaintextParams_t s_net_params;
static NetworkContext_t s_net_ctx;
static MQTTContext_t s_mqtt_ctx;
static uint8_t s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];

/* ------------------------------------------------------------------ */
/*  WLAN - minimal station-mode connect, no shell/console involved      */
/* ------------------------------------------------------------------ */

static void wlan_event_cb(uint8_t deviceId, uint32_t cbId, void *appCtx, void *payload, uint32_t payloadLength)
{
    qapi_WLAN_Join_Comp_Evt_t *cxnInfo = (qapi_WLAN_Join_Comp_Evt_t *)payload;

    (void)deviceId;
    (void)appCtx;
    (void)payloadLength;

    if (cbId != QAPI_WLAN_CONNECT_CB_E || cxnInfo == NULL)
        return;

    if (cxnInfo->evt_hdr.status == QAPI_OK) {
        s_wifi_connected = TRUE;
        info_printf("WiFi connected\n");
    } else {
        s_wifi_connected = FALSE;
        err_printf("WiFi disconnected, reason=%d\n", cxnInfo->reason_code);
    }
}

static qapi_Status_t wlan_enable_once(void)
{
    if (s_wlan_enabled)
        return QAPI_OK;

    qapi_WLAN_Set_Callback(wlan_event_cb, NULL);
    if (qapi_WLAN_Enable(true) != QAPI_OK) {
        err_printf("WLAN enable failed\n");
        return QAPI_ERROR;
    }
    s_wlan_enabled = TRUE;
    return QAPI_OK;
}

static qapi_Status_t wlan_associate(void)
{
    qapi_WLAN_DEV_Mode_e devMode = DEV_MODE_STATION_E;
    qapi_WLAN_Auth_Mode_e authMode = QAPI_WLAN_AUTH_WPA2_PSK_E;
    qapi_WLAN_Crypt_Type_e cipherType = QAPI_WLAN_CRYPT_AES_CRYPT_E;
    qapi_WLAN_Listen_Interval_Params_t listen_interval;
    const char *ssid = WIFI_SSID;
    const char *passphrase = WIFI_PASSPHRASE;

    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_OPERATION_MODE, &devMode, sizeof(devMode), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_AUTH_MODE, (void *)&authMode, sizeof(authMode), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_ENCRYPTION_TYPE, (void *)&cipherType, sizeof(cipherType), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_PASSPHRASE, (void *)passphrase, strlen(passphrase), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_SSID, (void *)ssid, strlen(ssid), FALSE);

    listen_interval.time = DTIM10_LISTEN_INTERVAL_TU;
    listen_interval.round_type = 0;
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_STA_LISTEN_INTERVAL_IN_TU, &listen_interval, sizeof(listen_interval), FALSE);

    return qapi_WLAN_Commit(DEV_STA_ID);
}

/* ------------------------------------------------------------------ */
/*  DTIM10/BMPS - engaged BMPS_ENTER_DELAY_MS after WLAN associates,     */
/*  disengaged before any reconnect attempt.                            */
/* ------------------------------------------------------------------ */

static void dtim10_bmps_enable(void)
{
    WMI_BMPS_IGNORE_BCMC ignore_bcmc;

    if (s_bmps_active)
        return;

    qapi_pm_enable(1);

    memset(&ignore_bcmc, 0, sizeof(ignore_bcmc));
    ignore_bcmc.enable = 1;
    wmi_cmd_send(WMI_BMPS_IGNORE_BCMC_CMDID, &ignore_bcmc, sizeof(ignore_bcmc));

    /* idle_timeout=0 - deliberately does NOT override the WMAC firmware's
     * own default idle-before-sleep timing (qapi_bmps_cfg() only sends
     * WMI_STA_IDLE_TIMER_CMDID when idle_timeout is nonzero - see
     * qapi_lowpower.c). Matches demo/vcnl3020_test_demo/src/wifi_mqtt.c's
     * current (2026-08-20) fixed config exactly - see that file's
     * VCNL3020_BMPS_IDLE_TIMEOUT_MS comment for why an explicit override
     * was removed. */
    qapi_bmps_cfg(1, 0);

    s_bmps_active = TRUE;
    info_printf("DTIM10/BMPS engaged\n");
}

static void dtim10_bmps_disable(void)
{
    if (!s_bmps_active)
        return;

    qapi_bmps_cfg(0, 0);
    qapi_pm_enable(0);

    s_bmps_active = FALSE;
    info_printf("DTIM10/BMPS disengaged (reconnecting)\n");
}

/* Optional diagnostic only (LOG_ENABLE) - logs exit_reason on every WMAC
 * wake, same instrumentation demo/vcnl3020_test_demo/src/wifi_mqtt.c used
 * for its 2026-08-20 hardware capture (13/13 exit_reason=2). No sensor poll
 * call here - there is nothing to poll in this build. */
static void bmps_wake_probe_cb(uint8_t evt, const void *p_args)
{
    const char *name = "?";

    (void)p_args;

    if (evt & PWR_EVT_WMAC_PRE_SLEEP) {
        name = "PRE_SLEEP";
    } else if (evt & PWR_EVT_WMAC_POST_AWAKE) {
        name = "POST_AWAKE";
    } else if (evt & PWR_EVT_WMAC_SLEEP_ABORT) {
        name = "SLEEP_ABORT";
    }

    if (evt & PWR_EVT_WMAC_POST_AWAKE) {
        uint8_t exit_reason = 0xFFU;

        qapi_bmps_get_exit_reason(&exit_reason);
        info_printf("bmps_wake: %s at %lu ms, exit_reason=%u\n",
            name, (unsigned long)hres_timer_curr_time_ms(), (unsigned)exit_reason);
    } else {
        info_printf("bmps_wake: %s at %lu ms\n", name, (unsigned long)hres_timer_curr_time_ms());
    }
}

static qapi_Status_t wlan_start_dhcp(void)
{
    struct netif *netif;
    uint8_t netid = nt_get_netifidx_by_devmode(STA_DEVICE);

    netif = netif_get_by_index(netid);
    if (netif == NULL) {
        err_printf("netif not found\n");
        return QAPI_ERR_INVALID_PARAM;
    }

    netif_set_addr(netif, IP4_ADDR_ANY4, IP4_ADDR_ANY4, IP4_ADDR_ANY4);
    return (dhcp_start(netif) == QAPI_OK) ? QAPI_OK : QAPI_ERROR;
}

static qbool_t wlan_dhcp_bound(void)
{
    struct netif *netif;
    struct dhcp *dhcp;
    uint8_t netid = nt_get_netifidx_by_devmode(STA_DEVICE);

    netif = netif_get_by_index(netid);
    if (netif == NULL)
        return FALSE;

    dhcp = netif_dhcp_data(netif);
    return (dhcp != NULL && dhcp->state == DHCP_STATE_BOUND) ? TRUE : FALSE;
}

/* ------------------------------------------------------------------ */
/*  MQTT - connect once, publish "online" once, then idle keepalive     */
/* ------------------------------------------------------------------ */

static uint32_t mqtt_get_time_ms(void)
{
    return hres_timer_curr_time_ms();
}

static void mqtt_event_cb(MQTTContext_t *ctx, MQTTPacketInfo_t *pkt, MQTTDeserializedInfo_t *info)
{
    (void)ctx;
    (void)info;

    if ((pkt->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
        info_printf("unexpected incoming publish (not subscribed to anything), ignored\n");
}

static const char *sockets_strerror(SocketStatus_t status)
{
    switch (status) {
    case SOCKETS_SUCCESS:             return "success";
    case SOCKETS_INVALID_PARAMETER:   return "invalid parameter";
    case SOCKETS_INSUFFICIENT_MEMORY: return "insufficient memory";
    case SOCKETS_API_ERROR:           return "socket API error";
    case SOCKETS_DNS_FAILURE:         return "DNS/hostname resolution failed";
    case SOCKETS_CONNECT_FAILURE:     return "TCP connect failed (broker unreachable/refused)";
    default:                          return "unknown";
    }
}

static int mqtt_connect(void)
{
    TransportInterface_t transport;
    MQTTFixedBuffer_t netbuf;
    MQTTConnectInfo_t conninfo;
    ServerInfo_t server;
    bool session_present;
    SocketStatus_t sock_status;
    MQTTStatus_t mqtt_status;
    MQTTPublishInfo_t onlineInfo;

    s_net_params.socketDescriptor = -1;
    s_net_ctx.pParams = &s_net_params;

    server.pHostName = MQTT_BROKER;
    server.hostNameLength = sizeof(MQTT_BROKER) - 1;
    server.port = MQTT_PORT;

    sock_status = Plaintext_Connect(&s_net_ctx, &server, MQTT_TRANSPORT_TIMEOUT_MS, MQTT_TRANSPORT_TIMEOUT_MS);
    if (sock_status != SOCKETS_SUCCESS) {
        err_printf("TCP connect to %s:%d failed: %s (status=%d)\n",
            MQTT_BROKER, MQTT_PORT, sockets_strerror(sock_status), (int)sock_status);
        return -1;
    }

    transport.pNetworkContext = &s_net_ctx;
    transport.send = Plaintext_Send;
    transport.recv = Plaintext_Recv;
    transport.writev = NULL;

    netbuf.pBuffer = s_mqtt_buf;
    netbuf.size = sizeof(s_mqtt_buf);

    mqtt_status = MQTT_Init(&s_mqtt_ctx, &transport, mqtt_get_time_ms, mqtt_event_cb, &netbuf);
    if (mqtt_status != MQTTSuccess) {
        err_printf("MQTT_Init failed: %s\n", MQTT_Status_strerror(mqtt_status));
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

    mqtt_status = MQTT_Connect(&s_mqtt_ctx, &conninfo, NULL, MQTT_CONNACK_TIMEOUT_MS, &session_present);
    if (mqtt_status != MQTTSuccess) {
        err_printf("MQTT_Connect (CONNECT/CONNACK) failed: %s - check broker username/password\n",
            MQTT_Status_strerror(mqtt_status));
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    /* One retained "online" publish so this shows up in MQTTX like every
     * other demo on this broker - NOT repeated, there is no periodic
     * status to re-send in a build with no sensor. */
    memset(&onlineInfo, 0, sizeof(onlineInfo));
    onlineInfo.qos = MQTTQoS0;
    onlineInfo.retain = true;
    onlineInfo.pTopicName = MQTT_TOPIC_STATUS;
    onlineInfo.topicNameLength = sizeof(MQTT_TOPIC_STATUS) - 1;
    onlineInfo.pPayload = MQTT_ONLINE_MESSAGE;
    onlineInfo.payloadLength = sizeof(MQTT_ONLINE_MESSAGE) - 1;
    MQTT_Publish(&s_mqtt_ctx, &onlineInfo, 0);

    info_printf("connected to %s:%d, published to %s\n", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_STATUS);
    return 0;
}

static qbool_t wlan_wait_connected(void)
{
    uint32_t waited_ms = 0;

    while (!s_wifi_connected && waited_ms < WLAN_CONNECT_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(200));
        waited_ms += 200;
    }
    return s_wifi_connected;
}

static void dtim10_control_task(void *arg)
{
    (void)arg;

    if (wlan_enable_once() != QAPI_OK) {
        err_printf("cannot continue without WLAN enabled\n");
        vTaskDelete(NULL);
    }

    /* Optional (LOG_ENABLE) - see bmps_wake_probe_cb()'s comment. Harmless
     * to register before BMPS ever engages. */
    qapi_bmps_sleep_wakeup_cb(bmps_wake_probe_cb, 1);

    /* Silence the WLAN firmware's own BMPS debug log ("P"/"B"/"wakebmps"/
     * "H"/"Exit: reason=..." lines) - same call demo/vcnl3020_test_demo/
     * src/wifi_mqtt.c makes. */
    qapi_bmps_log_enable(0);

    for (;;) {
        uint32_t bmps_enable_at_ms;

        /* ---- 1. WLAN: connect, wait until fully associated ---- */
        while (!s_wifi_connected) {
            if (wlan_associate() != QAPI_OK) {
                err_printf("WLAN associate failed, retry in 5s\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            if (!wlan_wait_connected())
                err_printf("WLAN connect timed out, retrying\n");
        }
        bmps_enable_at_ms = hres_timer_curr_time_ms() + BMPS_ENTER_DELAY_MS;

        /* ---- DHCP (skip if a lease from an earlier cycle is still bound) ---- */
        if (!wlan_dhcp_bound()) {
            while (s_wifi_connected && wlan_start_dhcp() != QAPI_OK) {
                err_printf("DHCP start failed, retry in 5s\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            while (s_wifi_connected && !wlan_dhcp_bound())
                vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!s_wifi_connected)
            continue;

        /* ---- 2. MQTT: connect (publishes "online" once inline) ---- */
        while (mqtt_connect() != 0) {
            if (!s_wifi_connected)
                break;
            err_printf("MQTT connect failed, retry in 5s\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        if (!s_wifi_connected)
            continue;

        /* ---- 3. Steady state: engage BMPS on schedule, idle keepalive ---- */
        for (;;) {
            uint32_t wait_ms = MQTT_HEARTBEAT_INTERVAL_MS;
            MQTTStatus_t st;

            if (!s_wifi_connected)
                break;

            /* Same early-engage fix as demo/vcnl3020_test_demo/src/
             * wifi_mqtt.c: cap the wait to however long is left until
             * bmps_enable_at_ms so BMPS engages promptly ~15s after
             * connect instead of only at the next heartbeat. */
            if (!s_bmps_active) {
                uint32_t now_ms = hres_timer_curr_time_ms();

                if (now_ms >= bmps_enable_at_ms) {
                    dtim10_bmps_enable();
                } else {
                    uint32_t remaining_ms = bmps_enable_at_ms - now_ms;

                    if (remaining_ms < wait_ms)
                        wait_ms = remaining_ms;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(wait_ms));

            if (!s_wifi_connected)
                break;
            if (!s_bmps_active)
                continue; /* woke early only to engage BMPS above, not a real heartbeat tick */

            st = MQTT_ProcessLoop(&s_mqtt_ctx);
            if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                err_printf("session lost (%d), reconnecting\n", (int)st);
                break;
            }
            info_printf("heartbeat\n");
        }

        dtim10_bmps_disable();
        Plaintext_Disconnect(&s_net_ctx);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void dtim10_control_start(void)
{
    if (s_task_handle != NULL)
        return;

    if (nt_qurt_thread_create(dtim10_control_task, "dtim10_control", 4096, NULL, 5, &s_task_handle) != pdPASS) {
        err_printf("task create failed\n");
    }
}
