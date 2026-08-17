/*
 * Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Minimal WLAN connect + MQTT publisher, added ONLY to report VCNL3020
 * STATUS transitions (see vcnl3020_test.c) to an MQTT broker - nothing
 * else. Threshold config and STATUS detection in vcnl3020_test.c are
 * UNTOUCHED; this file is purely additive, but it does now drive WHEN
 * vcnl3020_test.c checks the sensor (see the pre-BMPS poll and pin-22
 * sections below) - vcnl3020_test.c no longer owns any GPIO interrupt
 * itself. The two sides are decoupled via vcnl3020_mqtt_publish_status()'s queue
 * (see wifi_mqtt.h) so the sensor task never blocks on WLAN/MQTT I/O -
 * vcnl3020_test.c's only change is one non-blocking call at its existing
 * STATUS-print site.
 *
 * WLAN bring-up (station mode, WPA2-PSK/AES, DHCP) is the same minimal,
 * shell/console-free pattern as demo/mqtt_demo/src/mqtt_demo.c - same
 * SSID/passphrase too (this SDK's standard dev AP - see that file's
 * WLAN_AP_SSID/WLAN_AP_PASSPHRASE in mqtt_demo.h). Deliberately does NOT
 * pull in demo/common/net/src/net_shell.c (CONFIG_NET_SHELL) just for its
 * DHCP-client wrapper - that drags in a whole console-shell source bundle
 * (iperf/ssl_demo/ping/prefix/rvr_lite) unrelated to this file's one job.
 * dhcp_start() is called directly against the STA netif instead, exactly
 * like mqtt_demo.c does.
 *
 * MQTT connect settings (broker/credentials) match demo/powertest_demo/src/
 * mqtt_printf_task.c's - same broker, so the same account; client ID/topic
 * differ since this is a different device/purpose. Persistent session
 * (connect once, then publish-on-event, strictly - MQTT_TOPIC_PUB only ever
 * gets a message on a real OPEN<->CLOSED transition, never a repeat of the
 * unchanged status; see mqtt_publish_status()'s one call site). A
 * MQTT_HEARTBEAT_INTERVAL_MS (5s) tick drives the MQTT_ProcessLoop()
 * keepalive and prints a CONSOLE-ONLY liveness line so "still alive but
 * nothing changed" stays distinguishable from a hang on the console;
 * deliberately does NOT touch MQTT_TOPIC_PUB (re-publishing the last status
 * every tick would just spam MQTTX with the same value - see
 * wifi_mqtt_task()'s heartbeat branch).
 *
 * DTIM10/BMPS power-save:
 *
 *   - Listen interval is set to DTIM10 (VCNL3020_DTIM10_LISTEN_INTERVAL_TU
 *     = 1000 TU = 10 beacons) BEFORE every qapi_WLAN_Commit() - see
 *     wlan_associate() - same value demo/powertest_demo/src/powertest_demo.c
 *     and demo/ambient_power_demo/src/ambient_power_demo.c use. This only
 *     affects the *association* (what listen interval the AP grants us); it
 *     does NOT by itself put the radio into any low-power state.
 *   - Actually ENGAGING BMPS (dtim10_bmps_enable(): qapi_pm_enable(1) +
 *     WMI_BMPS_IGNORE_BCMC + qapi_bmps_cfg(1, ...) - same calls
 *     demo/ambient_power_demo/src/ambient_power_demo.c's pm_enable() uses)
 *     happens VCNL3020_BMPS_ENTER_DELAY_MS (15s) after WLAN associates - see
 *     wifi_mqtt_task()'s steady-state loop. MQTT connects during that
 *     15s full-power window same as always; once BMPS engages, publishing
 *     (a real STATUS transition) and the heartbeat's MQTT_ProcessLoop() both
 *     keep working exactly as before - a socket send just wakes the radio
 *     briefly for that one transmission and the firmware returns to its
 *     DTIM10 duty cycle on its own, no action needed from this code.
 *   - If WLAN or MQTT ever drops, dtim10_bmps_disable() runs BEFORE any
 *     reconnect attempt, and BMPS only re-engages once a fresh WLAN
 *     association has again been up for VCNL3020_BMPS_ENTER_DELAY_MS - the
 *     same rule as the initial connect, not a special case.
 *
 * Pin-22 (EXT_WAKEUP_INTR_N) interrupt wake during BMPS (added 2026-08-17,
 * see bmps_wake_probe_cb()): the VCNL3020's /INT is physically wired to
 * chip pin 22 (only - GPIO3 is no longer used, see vcnl3020_test.c's top
 * comment). init_aon_ext_wakeup_int() arms that pin
 * automatically at boot (FIRMWARE_APPS_INFORMED_WAKE is unconditionally
 * defined for this chip - see build/freertos/common/application_code/
 * main.c), and its ISR has a dedicated BMPS-wake path
 * (SUPPORT_SWTMR_TO_WKUP_FROM_BMPS in wifi_fw_ext_intr.c, also
 * unconditionally defined here) that calls
 * nt_bmps_wakeup_callback(EXIT_REASON_EXT_INT) whenever pin 22 asserts
 * while genuinely in BMPS "Sleep". Confirmed on hardware: a real touch now
 * produces qapi_bmps_get_exit_reason() == 7, not the reason=2 of an
 * ordinary DTIM wake - i.e. a real GPIO edge now forces its own BMPS exit,
 * independent of the natural DTIM cadence. bmps_wake_probe_cb()'s
 * PWR_EVT_WMAC_POST_AWAKE handler is the ONLY trigger while BMPS is active -
 * it only ever rides a wake the WLAN firmware was already doing on its own
 * (DTIM timer or pin 22), no independent timer while BMPS is on. There IS a
 * separate, independent poll timer for the pre-BMPS window specifically
 * (see prebmps_poll_start()/VCNL3020_PREBMPS_POLL_INTERVAL_MS below) - that
 * one is fine because the radio isn't duty-cycling in that window anyway,
 * so it doesn't force anything extra the way a poll DURING BMPS would. */

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

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
 * wmi_api.h - same headers demo/ambient_power_demo/src/ambient_power_demo.c
 * uses for the same calls. */
#include "qapi_lowpower.h"
#include "wmi.h"
#include "wmi_api.h"

#include "wifi_mqtt.h"

/* vcnl3020_test.c - poll trigger target, called both from
 * bmps_wake_probe_cb() and prebmps_poll_timer_cb() below. No dedicated
 * header in this project (see vcnl3020_test_start()'s own extern in
 * vcnl3020_test_demo_main.c), so declared the same way here. */
extern void vcnl3020_test_poll_now(void);

#ifndef DEV_STA_ID
#define DEV_STA_ID 1
#endif

/* Same dev AP as demo/mqtt_demo/src/mqtt_demo.h - this SDK's standard test
 * network. */
#define WIFI_SSID                   "FRITZ!Box7590AX"
#define WIFI_PASSPHRASE              "Tech91847"

/* Same broker/account as demo/powertest_demo/src/mqtt_printf_task.c. */
#define MQTT_BROKER                  "139.162.181.126"
#define MQTT_PORT                    1883
#define MQTT_USERNAME                "AABBCCDD"
#define MQTT_PASSWORD                "KXL8DHPsXggvRELD"
#define MQTT_CLIENT_ID               "vcnl3020_test_qcc7030"
#define MQTT_TOPIC_PUB               "sensor/proximity"
#define MQTT_NETWORK_BUF_SIZE        256U
/* 600s (10 min) instead of the MQTT-typical 60s - same value/reasoning as
 * demo/powertest_demo/src/mqtt_printf_task.c: fewer PINGREQ/PINGRESP
 * bursts means fewer interruptions to DTIM10/BMPS idle sleep, directly
 * reducing average current. Broker waits ~1.5x this (~15 min) before
 * considering the session dead if nothing at all is heard from us -
 * MQTT_HEARTBEAT_INTERVAL_MS below is matched 1:1 to this so the tick's
 * own traffic doubles as the keepalive, no separate idle PINGREQ needed
 * most cycles. */
#define MQTT_KEEPALIVE_SEC           600U
#define MQTT_CONNACK_TIMEOUT_MS      5000U
#define MQTT_TRANSPORT_TIMEOUT_MS    1000U
/* Heartbeat tick while waiting for a real STATUS event - also prints a
 * console-only liveness line every tick (see s_last_status and the
 * heartbeat branch in wifi_mqtt_task() below), so a missing heartbeat line
 * is a fast signal that something (task/WLAN/MQTT) actually stalled.
 * Deliberately MQTT-silent otherwise: this tick must NOT publish to
 * MQTT_TOPIC_PUB - that topic is only ever updated on a genuine
 * OPEN<->CLOSED transition (see mqtt_publish_status()'s one call site),
 * never re-sent just because this timer fired.
 *
 * Raised 5000->600000 (2026-08-14, matches MQTT_KEEPALIVE_SEC 1:1, same
 * pattern as demo/powertest_demo/src/mqtt_printf_task.c's
 * MQTT_PUBLISH_INTERVAL_MS): each tick's MQTT_ProcessLoop() call was found
 * to force a full radio wake ("wakebmps") under BMPS regardless of whether
 * there was anything to do, so a 5s tick meant a forced wake every 5s all
 * day - confirmed the dominant remaining power cost once the sensor side
 * was fixed. A real STATUS-change publish is NOT gated by this - it still
 * happens immediately via the publish queue (see wifi_mqtt_task()'s
 * xQueueReceive() below), this only affects the idle/no-change case. */
#define MQTT_HEARTBEAT_INTERVAL_MS   600000U
/* Depth of the OPEN/CLOSED event queue - a handful of transitions' worth of
 * slack in case the MQTT task is briefly mid-(re)connect; publishes are
 * cheap enough that this should never realistically fill under normal
 * touch/release cadence. */
#define MQTT_PUBLISH_QUEUE_DEPTH     8U

/* How long wlan_associate() waits for a CONNECT event before re-issuing
 * qapi_WLAN_Commit() - bounds a silent hang if one attempt never completes,
 * without needing a separate roaming timer. */
#define VCNL3020_WLAN_CONNECT_WAIT_MS       15000U

/* DTIM10: listen interval = 1000 TU = 10 x 100 TU (default AP beacon
 * interval), i.e. wake every 10th beacon instead of every one - same value
 * demo/powertest_demo and demo/ambient_power_demo use for the same thing.
 * Set before every qapi_WLAN_Commit(), see wlan_associate(). */
#define VCNL3020_DTIM10_LISTEN_INTERVAL_TU  1000U
/* BMPS idle-timeout passed to qapi_bmps_cfg() once BMPS engages. Was 50 -
 * ambient_power_demo.c's tuned value - but on hardware here (2026-08-14)
 * that value showed a real beacon-miss/reconnect storm right after BMPS
 * engaged (climbing run_bmiss/bmiss counters in the console log, followed
 * by a fresh WPA 4-way handshake, i.e. an actual disconnect+reconnect) -
 * plausibly this AP not tolerating BMPS re-entering that aggressively.
 * Reverted to 200 (ambient_power_demo.c's own comment: "idle time is set
 * to 200ms on previous demo, reduce to 50ms here" - i.e. 200 was the more
 * conservative, presumably more broadly-compatible starting point before
 * that demo's own tuning). Re-lower once confirmed stable if the extra
 * 150ms of idle-before-sleep matters for your power budget. */
#define VCNL3020_BMPS_IDLE_TIMEOUT_MS       200U
/* Delay after WLAN associates before engaging DTIM10/BMPS - gives a fixed
 * full-power window for WLAN+MQTT to fully establish before the radio
 * starts duty-cycling. */
#define VCNL3020_BMPS_ENTER_DELAY_MS        15000U

#define info_printf(msg, ...) printf("VCNL3020_MQTT: " msg, ##__VA_ARGS__)

static qbool_t s_wlan_enabled;    /* qapi_WLAN_Enable() done - once, ever */
static qbool_t s_wifi_connected;  /* set/cleared by wlan_event_cb on CONNECT/DISCONNECT */
static qbool_t s_bmps_active;     /* whether DTIM10/BMPS is currently engaged - guards dtim10_bmps_enable/disable */
static uint8_t s_last_status = 0xFFU; /* last known OPEN(0)/CLOSED(1) - 0xFF = unknown until the first real event, see mqtt_heartbeat() */
static QueueHandle_t s_publish_queue;
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
        info_printf("WiFi disconnected, reason=%d\n", cxnInfo->reason_code);
    }
}

/* Runs exactly once, ever - qapi_WLAN_Enable() is not meant to be re-run on
 * every reconnect (matches demo/powertest_demo/src/powertest_demo.c's
 * function_reconnect_cb(), which only re-applies Set_Param+Commit, never
 * re-enables). */
static qapi_Status_t wlan_enable_once(void)
{
    if (s_wlan_enabled)
        return QAPI_OK;

    qapi_WLAN_Set_Callback(wlan_event_cb, NULL);
    if (qapi_WLAN_Enable(true) != QAPI_OK) {
        info_printf("WLAN enable failed\n");
        return QAPI_ERROR;
    }
    s_wlan_enabled = TRUE;
    return QAPI_OK;
}

/* Applies station-mode/security/SSID params and commits - safe to call
 * repeatedly, both for the initial connect and every reconnect attempt. */
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

    /* DTIM10 - MUST be set before qapi_WLAN_Commit() so it applies to this
     * association attempt; firmware snaps to the closest DTIM the AP
     * actually advertises. This only affects what listen interval gets
     * granted - BMPS itself is engaged separately, VCNL3020_BMPS_ENTER_
     * DELAY_MS after this association completes (see wifi_mqtt_task()). */
    listen_interval.time = VCNL3020_DTIM10_LISTEN_INTERVAL_TU;
    listen_interval.round_type = 0;
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_STA_LISTEN_INTERVAL_IN_TU, &listen_interval, sizeof(listen_interval), FALSE);

    return qapi_WLAN_Commit(DEV_STA_ID);
}

/* Pre-BMPS periodic poll (added 2026-08-17) - covers the window between
 * WLAN connect and BMPS actually engaging (VCNL3020_BMPS_ENTER_DELAY_MS,
 * ~15s), now that GPIO3 is no longer used at all (see vcnl3020_test.c's top
 * comment). The radio is NOT duty-cycling in this window regardless of
 * whether this timer exists, so a plain poll here costs nothing extra -
 * unlike polling DURING BMPS, which forces its own radio wake every cycle
 * (see this file's history/git log for why that was rejected as the
 * steady-state mechanism). Created lazily, started when WLAN connects
 * (see wifi_mqtt_task()), stopped the moment BMPS actually engages (see
 * dtim10_bmps_enable() below) or the connection drops (see
 * wifi_mqtt_task()'s cleanup path) - pin 22 (bmps_wake_probe_cb()) is the
 * only trigger once BMPS is active. */
#define VCNL3020_PREBMPS_POLL_INTERVAL_MS   500U

static TimerHandle_t s_prebmps_poll_timer;

static void prebmps_poll_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    vcnl3020_test_poll_now();
}

static void prebmps_poll_start(void)
{
    if (s_prebmps_poll_timer == NULL) {
        s_prebmps_poll_timer = nt_create_timer(prebmps_poll_timer_cb, NULL,
            pdMS_TO_TICKS(VCNL3020_PREBMPS_POLL_INTERVAL_MS), pdTRUE);
        if (s_prebmps_poll_timer == NULL) {
            info_printf("pre-BMPS poll timer create failed\n");
            return;
        }
    }
    nt_start_timer(s_prebmps_poll_timer);
}

static void prebmps_poll_stop(void)
{
    if (s_prebmps_poll_timer != NULL)
        nt_stop_timer(s_prebmps_poll_timer);
}

/* ------------------------------------------------------------------ */
/*  DTIM10/BMPS - engaged VCNL3020_BMPS_ENTER_DELAY_MS after WLAN        */
/*  associates, disengaged before any reconnect attempt. See this file's */
/*  top comment for the full rule and why nothing here runs on a         */
/*  per-publish basis.                                                   */
/* ------------------------------------------------------------------ */

static void dtim10_bmps_enable(void)
{
    WMI_BMPS_IGNORE_BCMC ignore_bcmc;

    if (s_bmps_active)
        return;

    qapi_pm_enable(1);

    /* Ignore broadcast/multicast traffic while in BMPS - without this,
     * irrelevant BCMC frames on the LAN would themselves wake the radio,
     * working against the "sparse, DTIM10-only wakebmps" goal this exists
     * for. Same call demo/ambient_power_demo/src/ambient_power_demo.c's
     * pm_enable() uses. */
    memset(&ignore_bcmc, 0, sizeof(ignore_bcmc));
    ignore_bcmc.enable = 1;
    wmi_cmd_send(WMI_BMPS_IGNORE_BCMC_CMDID, &ignore_bcmc, sizeof(ignore_bcmc));

    qapi_bmps_cfg(1, VCNL3020_BMPS_IDLE_TIMEOUT_MS);

    /* Pin 22 (bmps_wake_probe_cb()) takes over from here - no reason to
     * keep forcing a plain poll now that BMPS is actually active. */
    prebmps_poll_stop();

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

/* Pin-22 (EXT_WAKEUP_INTR_N) BMPS-wake piggyback (2026-08-17) - qapi_bmps_
 * sleep_wakeup_cb() (drivers/lowpower/qapi_lowpower.c) registers this
 * against PWR_EVT_WMAC_PRE_SLEEP / PWR_EVT_WMAC_POST_AWAKE /
 * PWR_EVT_WMAC_SLEEP_ABORT (wifi_fw_pwr_cb_infra.h) - real WMAC power
 * events, not a timer this file invented. The VCNL3020's /INT is physically
 * wired to chip pin 22 (EXT_WAKEUP_INTR_N) - GPIO3 is no longer used at all
 * (see vcnl3020_test.c's top comment for why). Confirmed on hardware: a
 * real touch/release makes BMPS exit via qapi_bmps_get_exit_reason() == 7
 * (EXIT_REASON_EXT_INT, see aon_a2f_assert_isr_handler()'s
 * SUPPORT_SWTMR_TO_WKUP_FROM_BMPS branch in wifi_fw_ext_intr.c), not just
 * the natural DTIM-timer wake (reason 2) - so POST_AWAKE fires promptly on
 * a real event instead of only every ~30-45s. This is the ONLY trigger
 * while BMPS is active - see prebmps_poll_timer_cb() for the separate
 * pre-BMPS mechanism. */
static void bmps_wake_probe_cb(uint8_t evt, const void *p_args)
{
    const char *name = "?";

    (void)p_args;

    if (evt & PWR_EVT_WMAC_PRE_SLEEP) {
        name = "PRE_SLEEP";
    } else if (evt & PWR_EVT_WMAC_POST_AWAKE) {
        name = "POST_AWAKE";
        vcnl3020_test_poll_now();
    } else if (evt & PWR_EVT_WMAC_SLEEP_ABORT) {
        name = "SLEEP_ABORT";
    }

    info_printf("bmps_wake: %s at %lu ms\n", name, (unsigned long)hres_timer_curr_time_ms());
}

static qapi_Status_t wlan_start_dhcp(void)
{
    struct netif *netif;
    uint8_t netid = nt_get_netifidx_by_devmode(STA_DEVICE);

    netif = netif_get_by_index(netid);
    if (netif == NULL) {
        info_printf("netif not found\n");
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
/*  MQTT - persistent connection, publish-on-event + idle keepalive     */
/* ------------------------------------------------------------------ */

static uint32_t mqtt_get_time_ms(void)
{
    return hres_timer_curr_time_ms();
}

/* MQTT_Init() requires a non-NULL event callback unconditionally (rejects
 * NULL with MQTTBadParameter, regardless of network state - see
 * core_mqtt.c) even though this client only ever publishes and never
 * subscribes to anything, so there's normally nothing incoming to handle.
 * Only logs the unexpected case where the broker sends something anyway. */
static void mqtt_event_cb(MQTTContext_t *ctx, MQTTPacketInfo_t *pkt, MQTTDeserializedInfo_t *info)
{
    (void)ctx;
    (void)info;

    if ((pkt->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
        info_printf("unexpected incoming publish (not subscribed to anything), ignored\n");
}

/* Human-readable SocketStatus_t - core_mqtt.h has MQTT_Status_strerror()
 * for MQTTStatus_t, but the transport layer (sockets_posix.h) has no
 * equivalent, so this fills that gap for the diagnostics below. */
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

    s_net_params.socketDescriptor = -1;
    s_net_ctx.pParams = &s_net_params;

    server.pHostName = MQTT_BROKER;
    server.hostNameLength = sizeof(MQTT_BROKER) - 1;
    server.port = MQTT_PORT;

    sock_status = Plaintext_Connect(&s_net_ctx, &server, MQTT_TRANSPORT_TIMEOUT_MS, MQTT_TRANSPORT_TIMEOUT_MS);
    if (sock_status != SOCKETS_SUCCESS) {
        info_printf("TCP connect to %s:%d failed: %s (status=%d)\n",
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
        info_printf("MQTT_Init failed: %s\n", MQTT_Status_strerror(mqtt_status));
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
        info_printf("MQTT_Connect (CONNECT/CONNACK) failed: %s - check broker username/password\n",
            MQTT_Status_strerror(mqtt_status));
        Plaintext_Disconnect(&s_net_ctx);
        return -1;
    }

    info_printf("connected to %s:%d\n", MQTT_BROKER, MQTT_PORT);
    return 0;
}

static int mqtt_publish_status(uint8_t is_closed)
{
    MQTTPublishInfo_t pub;

    memset(&pub, 0, sizeof(pub));
    pub.qos = MQTTQoS0;
    pub.pTopicName = MQTT_TOPIC_PUB;
    pub.topicNameLength = sizeof(MQTT_TOPIC_PUB) - 1;
    pub.pPayload = is_closed ? "CLOSED" : "OPEN";
    pub.payloadLength = is_closed ? (sizeof("CLOSED") - 1) : (sizeof("OPEN") - 1);

    if (MQTT_Publish(&s_mqtt_ctx, &pub, 0) != MQTTSuccess)
        return -1;

    info_printf("published %s to %s\n", (const char *)pub.pPayload, MQTT_TOPIC_PUB);
    return 0;
}

/* Blocks until s_wifi_connected (the CONNECT event) or
 * VCNL3020_WLAN_CONNECT_WAIT_MS elapses, whichever first. Returns TRUE if
 * connected, FALSE on timeout (caller re-issues wlan_associate()). */
static qbool_t wlan_wait_connected(void)
{
    uint32_t waited_ms = 0;

    while (!s_wifi_connected && waited_ms < VCNL3020_WLAN_CONNECT_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(200));
        waited_ms += 200;
    }
    return s_wifi_connected;
}

static void wifi_mqtt_task(void *arg)
{
    (void)arg;

    if (wlan_enable_once() != QAPI_OK) {
        /* Nothing recoverable to do here - qapi_WLAN_Enable() itself
         * failed, not just one association attempt. */
        info_printf("cannot continue without WLAN enabled\n");
        vTaskDelete(NULL);
    }

    /* Registered once, for the task's whole lifetime - see
     * bmps_wake_probe_cb()'s comment. Harmless to register before BMPS
     * ever engages; it simply won't fire until it does. */
    qapi_bmps_sleep_wakeup_cb(bmps_wake_probe_cb, 1);

    /* Outer loop = one full (re)connect cycle: WLAN association, then DHCP
     * (only if not already bound), then MQTT connect. Re-entered from the
     * bottom whenever the steady-state loop below detects WLAN or MQTT
     * dropped. */
    for (;;) {
        uint32_t bmps_enable_at_ms;

        /* ---- 1. WLAN: connect, wait until fully associated ---- */
        while (!s_wifi_connected) {
            if (wlan_associate() != QAPI_OK) {
                info_printf("WLAN associate failed, retry in 5s\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
            if (!wlan_wait_connected())
                info_printf("WLAN connect timed out, retrying\n");
        }
        info_printf("WLAN connected - DTIM10/BMPS engages in %us\n",
            (unsigned)(VCNL3020_BMPS_ENTER_DELAY_MS / 1000U));
        bmps_enable_at_ms = hres_timer_curr_time_ms() + VCNL3020_BMPS_ENTER_DELAY_MS;

        /* Covers the sensor until pin 22 takes over once BMPS actually
         * engages - see prebmps_poll_start()'s comment. */
        prebmps_poll_start();

        /* ---- DHCP (skip if a lease from an earlier cycle is still bound) ---- */
        if (!wlan_dhcp_bound()) {
            while (s_wifi_connected && wlan_start_dhcp() != QAPI_OK) {
                info_printf("DHCP start failed, retry in 5s\n");
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            while (s_wifi_connected && !wlan_dhcp_bound())
                vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!s_wifi_connected)
            continue; /* dropped mid-DHCP - restart this cycle from WLAN */
        info_printf("DHCP bound\n");

        /* ---- 2. MQTT: connect, wait until fully connected ---- */
        info_printf("connecting to MQTT broker %s:%d...\n", MQTT_BROKER, MQTT_PORT);
        while (mqtt_connect() != 0) {
            if (!s_wifi_connected)
                break; /* WLAN dropped mid-retry - restart this cycle from WLAN */
            info_printf("MQTT connect failed, retry in 5s\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        if (!s_wifi_connected)
            continue;

        /* ---- 3. Steady state: publish-on-event + 5s heartbeat ---- */
        for (;;) {
            uint8_t is_closed;
            uint32_t wait_ms = MQTT_HEARTBEAT_INTERVAL_MS;

            if (!s_wifi_connected) {
                info_printf("WLAN dropped, reconnecting\n");
                break;
            }

            /* BUG FIX 2026-08-14: this check used to only ever get
             * re-evaluated at the top of THIS loop, which is otherwise
             * blocked inside xQueueReceive() below for up to
             * MQTT_HEARTBEAT_INTERVAL_MS (10 min) at a time. With nothing
             * to publish, that meant BMPS could sit un-engaged for up to
             * 10 minutes after bmps_enable_at_ms actually passed - only a
             * real STATUS-change event (which wakes xQueueReceive() early)
             * made it re-check sooner, which is why entry looked
             * "flaky"/slow instead of a reliable ~15s. Fix: while BMPS
             * isn't active yet, cap the wait to however long is actually
             * left until bmps_enable_at_ms, so this loop wakes up and
             * re-checks right on schedule instead of waiting for the next
             * unrelated event. Once BMPS is active this has no effect -
             * wait_ms stays at the full heartbeat interval. */
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

            if (xQueueReceive(s_publish_queue, &is_closed, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
                s_last_status = is_closed;
                if (mqtt_publish_status(is_closed) != 0) {
                    info_printf("publish failed, reconnecting\n");
                    break;
                }
            } else {
                /* Heartbeat tick (queue wait above timed out - nothing
                 * changed in the last MQTT_HEARTBEAT_INTERVAL_MS).
                 * CONSOLE-ONLY liveness line + the MQTT_ProcessLoop()
                 * keepalive - deliberately does NOT republish
                 * s_last_status to MQTT_TOPIC_PUB: MQTTX is a dashboard
                 * of the sensor's actual state, so a message must mean
                 * a real OPEN<->CLOSED transition just happened, not
                 * "still the same as 5s ago" - re-publishing the
                 * unchanged status every tick was exactly the earlier
                 * bug (MQTTX showing a constant stream of CLOSED while
                 * the contact was open the whole time). */
                MQTTStatus_t st = MQTT_ProcessLoop(&s_mqtt_ctx);

                if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                    info_printf("session lost (%d), reconnecting\n", (int)st);
                    break;
                }

                info_printf("heartbeat: alive, BMPS=%s, last_status=%s\n",
                    s_bmps_active ? "on" : "off",
                    (s_last_status == 0xFFU) ? "unknown" : (s_last_status ? "CLOSED" : "OPEN"));
            }
        }

        /* Something dropped (WLAN or MQTT) - leave power-save BEFORE
         * attempting any recovery: never stay in DTIM10/BMPS mid-reconnect.
         * Also stop the pre-BMPS poll unconditionally - covers the case
         * where the connection dropped before BMPS ever got a chance to
         * engage (dtim10_bmps_enable() never ran, so never stopped it). */
        dtim10_bmps_disable();
        prebmps_poll_stop();
        Plaintext_Disconnect(&s_net_ctx);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void vcnl3020_mqtt_publish_status(uint8_t is_closed)
{
    if (s_publish_queue == NULL)
        return; /* vcnl3020_mqtt_start() not called yet - drop, nothing to hand this off to */

    if (xQueueSend(s_publish_queue, &is_closed, 0) != pdTRUE)
        info_printf("publish queue full, dropped one STATUS event\n");
}

void vcnl3020_mqtt_start(void)
{
    if (s_task_handle != NULL)
        return;

    s_publish_queue = xQueueCreate(MQTT_PUBLISH_QUEUE_DEPTH, sizeof(uint8_t));
    if (s_publish_queue == NULL) {
        info_printf("queue create failed\n");
        return;
    }

    if (nt_qurt_thread_create(wifi_mqtt_task, "vcnl3020_mqtt", 4096, NULL, 5, &s_task_handle) != pdPASS) {
        info_printf("task create failed\n");
    }
}
