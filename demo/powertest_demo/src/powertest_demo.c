/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <string.h>
#include <stdint.h>
#include "wifi_cmn.h"
#include "qapi_wlan.h"
#include "qapi_console.h"

#include "qurt_internal.h"
#include "qurt_mutex.h"

#include "qapi_net_status.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "ip_addr.h"
#include "netifapi.h"
#include "data_path.h"
#if NT_FN_DHCPS_V4
#include "lwip/apps/nt_dhcps.h"
#include "dns.h"
#include "ip_addr.h"
#endif /* NT_FN_DHCPS_V4 */
#if NT_FN_DHCP6
#include "dhcp6.h"
#endif
#if LWIP_AUTOIP
#include "autoip.h"
#endif
#if LWIP_DNS
#include "dns.h"
#endif
#include "network_al.h"
#include "wifi_fw_pmu_ts_cfg.h"
#include "iperf.h"
#include "tcpip.h"
#include "nt_timer.h"
#include "wmi.h"
#include "wlan_drv.h"

/* SHT40 + MQTT sensor task from the qcli_demo project. Not part of the
 * support sleep-mode test; disabled so only the pure power-test path runs. */
#define ENABLE_SHT40_MQTT 0

#if ENABLE_SHT40_MQTT
#include "qapi_i2c.h"
#include "core_mqtt.h"
#include "plaintext_posix.h"
#endif

#define DEFAULT_NETIF_IDX netif_get_index(netif_default) /* Default netif idx */

#ifdef FERMION_SILICON
extern uint32_t UART_Send_direct(char *txbuf, uint32_t buflen);
#define UART_SEND_DIRECT(str) UART_Send_direct((str), strlen(str))
#else
#define UART_SEND_DIRECT(str)
#endif

#define info_printf(msg, ...) printf("WLAN: " msg, ##__VA_ARGS__)

#ifndef DEV_STA_ID
#define DEV_STA_ID 1
#endif
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define POWERTEST_WIFI_SSID       "FRITZ!Box7590AX"
#define POWERTEST_WIFI_PASSPHRASE "Tech91847"

/* Delay before engaging BMPS/DTIM10 sleep, so WLAN + the MQTT test
 * publisher's first connect/publish have a full-power window to finish
 * establishing before the radio starts duty-cycling on the DTIM10
 * interval. pm_enable() still always fires after this delay regardless of
 * whether MQTT succeeded, so the core DTIM10 sleep-test path is unaffected
 * if the broker is unreachable. */
#define POWERTEST_BMPS_ENTER_DELAY_MS 30000U

/* Automatically enter IMPS deep sleep a short while after connecting, so the
 * board can be measured with no USB/console attached. Set to 0 to fall back
 * to BMPS-only sleep (e.g. testable via the "lowpower" shell manually).
 * Temporarily disabled: IMPS was engaging ~5s after connect, right in the
 * middle of the MQTT test publisher's TCP connect attempt, and blocking it
 * forever. DTIM10 (via BMPS) still applies with this at 0. Re-enable once
 * MQTT connectivity under DTIM10 is confirmed, with IMPS entry delayed
 * enough to let the first MQTT connect/publish complete first. */
#define ENABLE_IMPS_DEEPSLEEP 0

#if ENABLE_IMPS_DEEPSLEEP
/* Time after connect before entering deep sleep - unplug USB within this
 * window. */
#define POWERTEST_IMPS_ENTER_DELAY_MS 5000U
/* Passed to qapi_imps_enter_sleep(): how long to wait for WLAN reconnect
 * after waking, and how long to stay in deep sleep, both in ms. */
#define POWERTEST_IMPS_WAIT_MS        5000U
#define POWERTEST_IMPS_SLEEP_MS       30000U
#endif /* ENABLE_IMPS_DEEPSLEEP */

#if ENABLE_SHT40_MQTT
#define SHT40_I2C_ADDR              0x44
#define SHT40_CMD_MEAS              0xFD
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
#define MQTT_MEASURE_INTERVAL_MS    5000U
#define MQTT_PROCESS_INTERVAL_MS    500U
#endif /* ENABLE_SHT40_MQTT */

/**********************************************************************/

/**
Data structure used by the api layer to pass lowpower configurations to the driver.
*/
typedef union {
    WMI_IMPS_CFG imps_cfg;
    /**< IMPS cfg, used in qapi_imps_cfg. */
    struct {
        WMI_BMPS_IDLE_TIME bmps_idle_time;
        /**< The idle timeout in ms, used in qapi_bmps_cfg. */
        WMI_BMPS_ENABLE bmps_enable;
        /**< To enable/disable BMPS, used in qapi_bmps_cfg. */
        WMI_BMPS_LOG_ENABLE bmps_log_enable;
        /**< To enable/disable BMPS Log, used in qapi_bmps_log_enable. */
    } bmps_cfg;
    /**< BMPS cfg, used in qapi_bmps_cfg. */
    WMI_BMPS_IGNORE_BCMC bmps_ignore_bcmc;
    /**< To config ignore group-cast traffic during BMPS. */
    WMI_BMPS_TIMING_CFG bmps_timing;
    /**< Internal timing parameters in BMPS. */
    WMI_SLP_CLK_CAL_CFG slp_clk_cal;
    /**< Enable/disable 32k clock calibration in sleep mode. */
    WMI_SLP_CLK_CAL_ACT slp_clk_cal_act;
    /**< Enable/disable 32k clock calibration in active mode. */
    uint32_t force_dtim;
    /**< Force dtim period */
} lpr_wmi_t;

typedef struct wifi_shell_cxt_s {
    qurt_mutex_t    wifi_shell_cxt_mutex;
    int32_t         scan_mode;
    qapi_WLAN_Auth_Mode_e auth;
    qapi_WLAN_Phy_Mode_e phy_mode;
    qapi_WLAN_11n_HT_Config_e htcfg;
    qbool_t         connected;
    char            ssid[__QAPI_WLAN_MAX_SSID_LEN+1];
    int32_t         ssid_length;
    uint8           bssid[6];
    uint16_t        channel_frequency;
    uint8_t         active_device;
    uint8_t         wlan_enabled;
} wifi_shell_cxt_t;

/**
Data structure used by the api layer to store the connection info.
*/
typedef struct wifi_demo_cxt_s {
    char    ssid[__QAPI_WLAN_MAX_SSID_LEN+1];
    uint32_t ssid_len;
    uint8_t passphrase[WMI_PASSPHRASE_LEN+1];
    uint8_t passphrase_len;
    uint8_t dot11AuthMode;
    uint8_t authMode;
    uint8_t pairwiseCryptoType;
    uint8_t groupCryptoType;
} wifi_demo_cxt_t;

QAPI_Console_Group_Handle_t powertest_shell_cmd_group_handle;
TimerHandle_t iperf_timer;
TimerHandle_t roaming_timer;
TaskHandle_t net_send_task_handle;
THROUGHPUT_CXT *dtim_iperf_tCxt = NULL;
static wifi_shell_cxt_t *pg_wifi_shell_cxt;
static wifi_shell_cxt_t g_wifi_shell_cxt;
static wifi_demo_cxt_t pg_wifi_demo_cxt;
#if ENABLE_SHT40_MQTT
static TaskHandle_t mqtt_task_handle;
static PlaintextParams_t s_net_params;
static NetworkContext_t s_net_ctx;
static MQTTContext_t s_mqtt_ctx;
static uint8_t s_mqtt_buf[MQTT_NETWORK_BUF_SIZE];
static volatile int s_measure_now;
static volatile uint8_t s_mqtt_started;
static uint8_t s_sht40_ready;
#endif /* ENABLE_SHT40_MQTT */
static uint8_t s_pm_enabled;
#if ENABLE_IMPS_DEEPSLEEP
static uint8_t s_imps_started;
#endif /* ENABLE_IMPS_DEEPSLEEP */
uint8_t g_wifi_ready = 0;
uint8_t log_enable = 1;
extern lpr_wmi_t g_lowpower_wmi;
extern wlan_qapi_cxt_t *gp_wlan_qapi_cxt;

extern qapi_Status_t qapi_pm_enable(uint8_t enable);
extern qapi_Status_t wmi_cmd_send(WMI_COMMAND_ID cmd_id, void *p_data, uint32_t data_len);
/* Starts the periodic MQTT test-message publisher (mqtt_printf_task.c);
 * used to verify MQTT connectivity/timing under DTIM10 sleep without the
 * SHT40 sensor. See mqtt_printf_task_start() call in the CONNECT handler
 * below. */
extern void mqtt_printf_task_start(void);
/* Starts the DHCPv4 client on the STA netif (net_shell.c) - equivalent to
 * running "dhcpv4c wlan1 new" on the console. Without this the STA netif
 * stays on its unconfigured 127.0.0.1 default and every outbound
 * connection (MQTT included) fails, since there's no real IP/gateway.
 * Must run automatically since USB/console may be unplugged during the
 * actual power test. */
extern qapi_Status_t net_shell_start_dhcp_client(uint8_t devid);
#if ENABLE_IMPS_DEEPSLEEP
extern qapi_Status_t qapi_imps_enter_sleep(uint8_t enable, uint32_t wait_time, uint32_t sleep_time);
#endif /* ENABLE_IMPS_DEEPSLEEP */
extern void qurt_thread_sleep(uint32 duration);

void pm_enable(void);
uint8_t function_reconnect_cb(void);
static TaskHandle_t pm_enable_task_handle;
static uint8_t s_pm_enable_task_started;
static void powertest_pm_enable_task(void *arg);
#if ENABLE_IMPS_DEEPSLEEP
static TaskHandle_t imps_deepsleep_task_handle;
static void powertest_imps_deepsleep_task(void *arg);
#endif /* ENABLE_IMPS_DEEPSLEEP */

#if ENABLE_SHT40_MQTT
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
    qapi_Status_t status = qapi_I2CM_Open(QAPI_I2C_INSTANCE_SE0_E, &cfg);

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

    if (pub->topicNameLength != (sizeof(MQTT_TOPIC_CMD) - 1) ||
        strncmp(MQTT_TOPIC_CMD, pub->pTopicName, pub->topicNameLength) != 0)
        return;

    if (pub->payloadLength == (sizeof("measure") - 1) &&
        strncmp("measure", (const char *)pub->pPayload, pub->payloadLength) == 0)
        s_measure_now = 1;
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
        info_printf("MQTT_ProcessLoop after subscribe status=%d\n", (int)process_status);
        if (process_status != MQTTSuccess && process_status != MQTTNeedMoreBytes) {
            Plaintext_Disconnect(&s_net_ctx);
            return -1;
        }
    }

    info_printf("MQTT connected to %s:%d, subscribed to %s\n", MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_CMD);
    return 0;
}

static int mqtt_publish(int temp_x10, int rh_x10)
{
    char msg[48];
    int len;
    MQTTPublishInfo_t pub;

    len = snprintf(msg, sizeof(msg), "Temp=%d.%dC Feuchte=%d.%d%%", temp_x10 / 10, temp_x10 % 10, rh_x10 / 10, rh_x10 % 10);
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

static void mqtt_sensor_task(void *arg)
{
    TickType_t last_pub;

    (void)arg;

    while (sht40_open() != 0) {
        info_printf("SHT40 open failed, retry in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    for (;;) {
        info_printf("MQTT connecting to %s:%d...\n", MQTT_BROKER, MQTT_PORT);
        while (g_wifi_ready && (mqtt_connect_and_subscribe() != 0)) {
            info_printf("MQTT connect failed, retry in 5s\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }

        if (!g_wifi_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        last_pub = xTaskGetTickCount();
        s_measure_now = 0;

        for (;;) {
            MQTTStatus_t st;
            TickType_t now;
            uint32_t elapsed;
            int do_measure;

            if (!g_wifi_ready) {
                Plaintext_Disconnect(&s_net_ctx);
                break;
            }

            st = MQTT_ProcessLoop(&s_mqtt_ctx);
            if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
                info_printf("MQTT session lost (%d), reconnecting\n", (int)st);
                Plaintext_Disconnect(&s_net_ctx);
                break;
            }

            now = xTaskGetTickCount();
            elapsed = (uint32_t)((now - last_pub) * portTICK_PERIOD_MS);
            do_measure = s_measure_now || (elapsed >= MQTT_MEASURE_INTERVAL_MS);

            if (do_measure) {
                int temp_x10 = 0;
                int rh_x10 = 0;

                if (sht40_measure(&temp_x10, &rh_x10) == 0) {
                    info_printf("Temp=%d.%dC Feuchte=%d.%d%%\n", temp_x10 / 10, temp_x10 % 10, rh_x10 / 10, rh_x10 % 10);
                    if (mqtt_publish(temp_x10, rh_x10) != 0) {
                        info_printf("MQTT publish failed, reconnecting\n");
                        Plaintext_Disconnect(&s_net_ctx);
                        s_measure_now = 0;
                        break;
                    }
                }

                last_pub = xTaskGetTickCount();
                s_measure_now = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(MQTT_PROCESS_INTERVAL_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
#endif /* ENABLE_SHT40_MQTT */

uint8_t get_demo_active_device()
{
	return DEV_STA_ID;
}

static void wlan_shell_event_handler(__unused uint8_t deviceId, uint32_t cbId, void __unused *pApplicationContext, void *payload, uint32_t payload_Length)
{
    wifi_shell_cxt_t *p_cxt = pg_wifi_shell_cxt;

    switch(cbId) {
    case QAPI_WLAN_SCAN_COMPLETE_CB_E: {
        if (!payload || !payload_Length) {
            info_printf("QAPI_WLAN_SCAN_COMPLETE_CB_E event error\n");
            break;
        }

        qapi_WLAN_Scan_Comp_Evt_t *p_scan_compl_evt = (qapi_WLAN_Scan_Comp_Evt_t*)payload;
        info_printf("Received Scan complete event, found bss count:%d\n", p_scan_compl_evt->num_bss_cur);
        break;
    }
    case QAPI_WLAN_CONNECT_CB_E: {
        qapi_WLAN_Join_Comp_Evt_t *cxnInfo  = (qapi_WLAN_Join_Comp_Evt_t *)(payload);
        uint8_t * mac = cxnInfo->bssid;
		if(cxnInfo->ssid_Length) {
			memscpy(p_cxt->ssid, cxnInfo->ssid_Length, cxnInfo->ssid, cxnInfo->ssid_Length);
			p_cxt->ssid[cxnInfo->ssid_Length] = 0;
			p_cxt->ssid_length = cxnInfo->ssid_Length;
			memscpy(p_cxt->bssid, 6, cxnInfo->bssid, 6);
		}
        p_cxt->channel_frequency = cxnInfo->channel_frequency;
        if(cxnInfo->evt_hdr.status == QAPI_OK){
            qapi_WLAN_Auth_Mode_e e_wpa_ver = p_cxt->auth;
			if(cxnInfo->bss_Connection_Status)
				p_cxt->connected = true;
            info_printf("devid - %d %d CONNECTED MAC addr %02x:%02x:%02x:%02x:%02x:%02x\n",
                DEV_STA_ID, cxnInfo->bss_Connection_Status, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            if (e_wpa_ver==QAPI_WLAN_AUTH_WPA_PSK_E || e_wpa_ver==QAPI_WLAN_AUTH_WPA2_PSK_E) {
                info_printf("4 way handshake success for device=1\n");
            }
            g_wifi_ready = 1;
            /* Bring up a real IP address before anything tries to use the
             * network - association alone doesn't get one. */
            net_shell_start_dhcp_client(STA_DEVICE);
            /* Start the MQTT test publisher (plain counter/uptime message
             * every 5s, no SHT40 needed) so power/timing can be measured
             * under DTIM10 sleep. */
            mqtt_printf_task_start();
#if ENABLE_SHT40_MQTT
            if (!s_mqtt_started) {
                if (nt_qurt_thread_create(mqtt_sensor_task, "mqtt_sensor_task", STA_TASK_STACK_SIZE, NULL, 5, &mqtt_task_handle) == pdPASS) {
                    s_mqtt_started = 1;
                } else {
                    info_printf("mqtt_sensor_task create fail\n");
                }
            }
#endif /* ENABLE_SHT40_MQTT */
            if (!s_pm_enable_task_started) {
                if (nt_qurt_thread_create(powertest_pm_enable_task, "powertest_pm_enable",
                        STA_TASK_STACK_SIZE, NULL, 5, &pm_enable_task_handle) == pdPASS) {
                    s_pm_enable_task_started = 1;
                } else {
                    info_printf("powertest_pm_enable create fail\n");
                }
            }
#if ENABLE_IMPS_DEEPSLEEP
            if (!s_imps_started) {
                if (nt_qurt_thread_create(powertest_imps_deepsleep_task, "powertest_imps_sleep",
                        STA_TASK_STACK_SIZE, NULL, 5, &imps_deepsleep_task_handle) == pdPASS) {
                    s_imps_started = 1;
                } else {
                    info_printf("powertest_imps_sleep create fail\n");
                }
            }
#endif /* ENABLE_IMPS_DEEPSLEEP */
            if(roaming_timer != NULL) {
                nt_stop_timer(roaming_timer);
            }
        } else {
			info_printf("WiFi disconnect reason code is %d\n", cxnInfo->reason_code);
			if(cxnInfo->bss_Connection_Status) {
				p_cxt->connected = false;
                g_wifi_ready = 0;
                qapi_WLAN_Disconnect(get_demo_active_device());
				info_printf("devId %d Disconnected MAC addr %02x:%02x:%02x:%02x:%02x:%02x \n",
					DEV_STA_ID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                if(roaming_timer != NULL) {
                    nt_start_timer(roaming_timer);
                }
			} else {
				info_printf("REF_STA Disconnected MAC addr %02x:%02x:%02x:%02x:%02x:%02x devId %d\r\n",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], p_cxt->active_device);
			}
        }
        info_printf("channel_frequency=%d\n", cxnInfo->channel_frequency);
        info_printf("ssid = %s\n", p_cxt->ssid);
        info_printf("assoc_id=%d\n", cxnInfo->assoc_id);
        info_printf("host_initiated=%d\n", cxnInfo->host_initiated);
        break;
    }
    case QAPI_WLAN_DISCONNECT_CB_E: {
        qapi_WLAN_Join_Comp_Evt_t *cxnInfo = (qapi_WLAN_Join_Comp_Evt_t *)(payload);
		if(cxnInfo->bss_Connection_Status) {
            p_cxt->connected = false;
        }

        if(p_cxt->ssid_length)
            info_printf("devId %d disconnected from ssid = %s\n", p_cxt->active_device, p_cxt->ssid);
        break;
    }
	case QAPI_WLAN_CHANNEL_SWITCH_CB_E: {
		qapi_WLAN_Chan_Switch_Evt_t *ecsa = (qapi_WLAN_Chan_Switch_Evt_t *)payload;
		if(ecsa->evt_hdr.status == QAPI_OK) {
			p_cxt->channel_frequency = ecsa->freq;
			info_printf("devId %d channel switch to %d success\n", p_cxt->active_device, ecsa->freq);
		} else {
			info_printf("devId %d channel switch fail, reason %d\n", p_cxt->active_device, ecsa->reason);
		}
		break;
	}
    }
}

void pm_enable()
{
    if (s_pm_enabled) {
        info_printf("===== sleep already enabled =====\r\n");
        return;
    }

    info_printf("=====  sleep  =====\r\n");
    qapi_pm_enable(1);

    WMI_BMPS_IGNORE_BCMC *pdata = (WMI_BMPS_IGNORE_BCMC *)&g_lowpower_wmi;
    memset(pdata, 0, sizeof(*pdata));
    pdata->enable = 1;
    wmi_cmd_send(WMI_BMPS_IGNORE_BCMC_CMDID, pdata, sizeof(*pdata));

    WMI_BMPS_ENABLE *pdata2 = (WMI_BMPS_ENABLE *)&g_lowpower_wmi;
    memset(pdata2, 0, sizeof(*pdata2));
    pdata2->enable = 1;
    wmi_cmd_send(WMI_BMPS_ENABLE_CMDID, pdata2, sizeof(*pdata2));

    s_pm_enabled = 1;
}

static void powertest_pm_enable_task(void *arg)
{
    (void)arg;

    info_printf("delaying BMPS/DTIM10 sleep by %u ms so WLAN+MQTT can fully establish first\n",
        (unsigned)POWERTEST_BMPS_ENTER_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(POWERTEST_BMPS_ENTER_DELAY_MS));

    pm_enable();

    vTaskDelete(NULL);
}

#if ENABLE_IMPS_DEEPSLEEP
static void powertest_imps_deepsleep_task(void *arg)
{
    (void)arg;

    info_printf("entering deep sleep (IMPS) in %u ms - unplug USB now if needed\n",
        (unsigned)POWERTEST_IMPS_ENTER_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(POWERTEST_IMPS_ENTER_DELAY_MS));

    info_printf("===== deep sleep (IMPS) =====\n");
    qapi_imps_enter_sleep(1, POWERTEST_IMPS_WAIT_MS, POWERTEST_IMPS_SLEEP_MS);

    vTaskDelete(NULL);
}
#endif /* ENABLE_IMPS_DEEPSLEEP */

static TaskHandle_t wifi_auto_connect_task_handle;

static void powertest_wifi_auto_connect_task(void *arg)
{
    qapi_WLAN_DEV_Mode_e devMode = DEV_MODE_STATION_E;
    qapi_WLAN_Auth_Mode_e auth = QAPI_WLAN_AUTH_WPA2_PSK_E;
    qapi_WLAN_Crypt_Type_e cipher = QAPI_WLAN_CRYPT_AES_CRYPT_E;
    const char *ssid = POWERTEST_WIFI_SSID;
    const char *passphrase = POWERTEST_WIFI_PASSPHRASE;

    (void)arg;

    pg_wifi_shell_cxt = &g_wifi_shell_cxt;
    memset(&g_wifi_shell_cxt, 0, sizeof(wifi_shell_cxt_t));
    pg_wifi_shell_cxt->auth = auth;

    gp_wlan_qapi_cxt->qapi_event_handler = NULL;
    qapi_WLAN_Set_Callback(wlan_shell_event_handler, NULL);
    qapi_WLAN_Enable(true);
    pg_wifi_shell_cxt->wlan_enabled = 1;

    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_OPERATION_MODE,
        &devMode, sizeof(devMode), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_AUTH_MODE,
        (void *)&auth, sizeof(auth), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_ENCRYPTION_TYPE,
        (void *)&cipher, sizeof(cipher), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
        __QAPI_WLAN_PARAM_GROUP_SECURITY_PASSPHRASE,
        (void *)passphrase, strlen(passphrase), FALSE);
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_SSID,
        (void *)ssid, strlen(ssid), FALSE);

    /* Request DTIM 10 listen interval for lowest sleep-mode current draw.
     * time = 1000 TU = 10 x 100 TU (default AP beacon interval), so the
     * device wakes every 10th beacon/DTIM instead of every beacon.
     * Trade-off: buffered-data response latency rises to ~10 beacon
     * intervals (~1s), but sleep current drops (~45uA @ DTIM5 -> ~39uA
     * target @ DTIM10 per datasheet). Must be set before qapi_WLAN_Commit()
     * so it applies to this association attempt; firmware snaps to the
     * closest DTIM the AP actually advertises.
     */
    qapi_WLAN_Listen_Interval_Params_t listen_interval;
    listen_interval.time = 1000;
    listen_interval.round_type = 0;
    qapi_WLAN_Set_Param(DEV_STA_ID, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
        __QAPI_WLAN_PARAM_GROUP_WIRELESS_STA_LISTEN_INTERVAL_IN_TU,
        &listen_interval, sizeof(listen_interval), FALSE);

    info_printf("auto-connecting to ssid %s\n", ssid);
    qapi_WLAN_Commit(DEV_STA_ID);

    /* Save connection info so function_reconnect_cb() (driven by
     * roaming_timer below) can re-commit after an unexpected disconnect -
     * mirrors what iperf_for_powertest() does, but wired into the actual
     * boot-time auto-connect path instead of only firing as a side effect
     * of the iperf shell command. Without this, roaming_timer stays NULL
     * and a dropped AP association is never retried. */
    memscpy(pg_wifi_demo_cxt.ssid, __QAPI_WLAN_MAX_SSID_LEN+1, gp_wlan_qapi_cxt->connect_cmd.ssid, __QAPI_WLAN_MAX_SSID_LEN+1);
    pg_wifi_demo_cxt.ssid_len = gp_wlan_qapi_cxt->connect_cmd.ssidLength;
    memscpy(pg_wifi_demo_cxt.passphrase, WMI_PASSPHRASE_LEN+1, gp_wlan_qapi_cxt->passphrase_cmd.passphrase, WMI_PASSPHRASE_LEN+1);
    pg_wifi_demo_cxt.passphrase_len = gp_wlan_qapi_cxt->passphrase_cmd.passphrase_len;
    pg_wifi_demo_cxt.authMode = gp_wlan_qapi_cxt->connect_cmd.authMode;
    pg_wifi_demo_cxt.dot11AuthMode = gp_wlan_qapi_cxt->connect_cmd.dot11AuthMode;
    pg_wifi_demo_cxt.groupCryptoType = gp_wlan_qapi_cxt->connect_cmd.groupCryptoType;
    pg_wifi_demo_cxt.pairwiseCryptoType = gp_wlan_qapi_cxt->connect_cmd.pairwiseCryptoType;

    /* Retry every 10 min while disconnected; wlan_shell_event_handler()
     * starts this on DISCONNECT and stops it once CONNECT succeeds again.
     * Widened from 5s: a fast retry cadence here was the leading suspect
     * for periodic current spikes during idle power measurement (its
     * period matched the observed spike interval almost exactly). Slower
     * reconnect attempts trade off slower recovery from an AP outage for
     * much less average power impact if this timer is ever active. */
    roaming_timer = nt_create_timer(function_reconnect_cb, NULL, NT_MS_TO_TICKS(600000), TRUE);

    vTaskDelete(NULL);
}

void function_net_send_cb()
{
    xTaskNotify(net_send_task_handle, 1, eSetBits);
}

uint8_t function_reconnect_cb()
{
    uint8_t ret;
    uint8_t deviceId = DEV_STA_ID;

    if(!g_wifi_ready) {
        qapi_WLAN_Set_Param (deviceId, __QAPI_WLAN_PARAM_GROUP_WIRELESS_SECURITY,
            __QAPI_WLAN_PARAM_GROUP_SECURITY_PASSPHRASE,
            (void *)pg_wifi_demo_cxt.passphrase, pg_wifi_demo_cxt.passphrase_len, FALSE);

        /* Set auth mode and encryption type */
        gp_wlan_qapi_cxt->connect_cmd.authMode = pg_wifi_demo_cxt.authMode;
        gp_wlan_qapi_cxt->connect_cmd.dot11AuthMode = pg_wifi_demo_cxt.dot11AuthMode;
        gp_wlan_qapi_cxt->connect_cmd.pairwiseCryptoType = pg_wifi_demo_cxt.pairwiseCryptoType;
        gp_wlan_qapi_cxt->connect_cmd.groupCryptoType = pg_wifi_demo_cxt.groupCryptoType;

        qapi_WLAN_Set_Param (0, __QAPI_WLAN_PARAM_GROUP_WIRELESS,
            __QAPI_WLAN_PARAM_GROUP_WIRELESS_SSID,
            (void *)pg_wifi_demo_cxt.ssid, pg_wifi_demo_cxt.ssid_len, FALSE);

        ret = qapi_WLAN_Commit(deviceId);
        info_printf("connect to ssid return %d\n", ret);
    }
    return ret;
}

void function_net_send_task()
{
    BaseType_t xResult;
	uint32_t notified_value = 0;
    uint32_t send_bytes;
    uint32_t send_times = 0;

    while(1) {
        xResult = xTaskNotifyWait( pdFALSE, ULONG_MAX, &notified_value, portMAX_DELAY);
        if (xResult == pdPASS) {
            if (notified_value && g_wifi_ready) {
                do {
                    if (dtim_iperf_tCxt->buffer == NULL) {
                        while ((dtim_iperf_tCxt->buffer = malloc(dtim_iperf_tCxt->params.tx_params.packet_size)) == NULL) {
                            qurt_thread_sleep(100);
                        }
                    }
                    pattern(dtim_iperf_tCxt->buffer, dtim_iperf_tCxt->params.tx_params.packet_size);
                    send_bytes =
                        send(dtim_iperf_tCxt->sock_peer, dtim_iperf_tCxt->buffer, dtim_iperf_tCxt->params.tx_params.packet_size, 0);
                    send_times++;
                    if(log_enable)
                        info_printf("===== sent %u bytes for %u times =====\r\n", send_bytes, send_times);
                    if ((dtim_iperf_tCxt->params.tx_params.interval_us > 0) && (send_times < dtim_iperf_tCxt->params.tx_params.packet_number)) {
                        qurt_thread_sleep(dtim_iperf_tCxt->params.tx_params.interval_us);
                    }
                } while (send_times < dtim_iperf_tCxt->params.tx_params.packet_number);
                send_times = 0;
            }
        }
    }

    vTaskDelete(NULL);
}

qapi_Status_t iperf_for_powertest(uint32_t Parameter_Count, QAPI_Console_Parameter_t *Parameter_List)
{
    BaseType_t xResult = pdFAIL;
    uint32_t notified_value = 0;
    unsigned int index = 0;
    TickType_t dtim_time = 0;
    TickType_t roaming_time = 5000;
    char *receiver_ip;
    unsigned int ipAddress = 0;
    unsigned int pktSize = 0;
    g_wifi_ready = 1;

    /* Wifi shell init*/
    pg_wifi_shell_cxt = &g_wifi_shell_cxt;
    memset(&g_wifi_shell_cxt, 0, sizeof(wifi_shell_cxt_t));
    pg_wifi_shell_cxt->auth = QAPI_WLAN_AUTH_NONE_E;

    gp_wlan_qapi_cxt->qapi_event_handler = NULL;
    qapi_WLAN_Set_Callback(wlan_shell_event_handler, NULL);

    /* Save connection info for potential reconnection */
    memscpy(pg_wifi_demo_cxt.ssid, __QAPI_WLAN_MAX_SSID_LEN+1, gp_wlan_qapi_cxt->connect_cmd.ssid, __QAPI_WLAN_MAX_SSID_LEN+1);
    pg_wifi_demo_cxt.ssid_len = gp_wlan_qapi_cxt->connect_cmd.ssidLength;
    memscpy(pg_wifi_demo_cxt.passphrase, WMI_PASSPHRASE_LEN+1, gp_wlan_qapi_cxt->passphrase_cmd.passphrase, WMI_PASSPHRASE_LEN+1);
    pg_wifi_demo_cxt.passphrase_len = gp_wlan_qapi_cxt->passphrase_cmd.passphrase_len;
    pg_wifi_demo_cxt.authMode = gp_wlan_qapi_cxt->connect_cmd.authMode;
    pg_wifi_demo_cxt.dot11AuthMode = gp_wlan_qapi_cxt->connect_cmd.dot11AuthMode;
    pg_wifi_demo_cxt.groupCryptoType = gp_wlan_qapi_cxt->connect_cmd.groupCryptoType;
    pg_wifi_demo_cxt.pairwiseCryptoType = gp_wlan_qapi_cxt->connect_cmd.pairwiseCryptoType;

    /* If WiFi was already configured/connected before this command ran,
     * that connect went through whatever callback was registered at the
     * time, not wlan_shell_event_handler above, so pm_enable() never fired.
     * Re-commit here now that our handler is active, so the resulting
     * CONNECT event is guaranteed to route through it. */
    if (pg_wifi_demo_cxt.ssid_len) {
        qapi_WLAN_Commit(DEV_STA_ID);
    }

    if (nt_qurt_thread_create(function_net_send_task, "net_send_task", STA_TASK_STACK_SIZE, NULL, 5, &net_send_task_handle) != pdPASS) {
        info_printf("net_send_task create fail\n");
        goto ERROR_1;
    }

    dtim_iperf_tCxt = malloc(sizeof(THROUGHPUT_CXT));
    if (dtim_iperf_tCxt == NULL) {
        info_printf("Memory alloc failed\n");
        return QAPI_ERR_NO_MEMORY;
    }
    memset(dtim_iperf_tCxt, 0, sizeof(THROUGHPUT_CXT));

    dtim_iperf_tCxt->protocol = TCP;
    dtim_iperf_tCxt->params.tx_params.port = IPERF_DEFAULT_PORT;
    while (index < Parameter_Count) {
        if (0 == strcmp(Parameter_List[index].String_Value, "-u")) {
            index++;
            dtim_iperf_tCxt->protocol = UDP;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-d")) {
            index++;
            dtim_time = Parameter_List[index].Integer_Value;
            index++;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-p")) {
            index++;
            dtim_iperf_tCxt->params.tx_params.port = Parameter_List[index].Integer_Value;
            index++;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-c")) {
            index++;
            receiver_ip = Parameter_List[index].String_Value;
            if (inet_pton(AF_INET, receiver_ip, &ipAddress) != 1) {
                info_printf("Incorrect IP address %s\n", receiver_ip);
                return QAPI_ERR_INVALID_PARAM;
            }
            index++;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-l")) {
            index++;
            pktSize = Parameter_List[index].Integer_Value;
            index++;
            pktSize = pktSize < 12 ? 12 : pktSize;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-n")) {
            index++;
            dtim_iperf_tCxt->params.tx_params.packet_number = Parameter_List[index].Integer_Value;
            index++;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-i")) {
            index++;
            dtim_iperf_tCxt->params.tx_params.interval_us = Parameter_List[index].Integer_Value;
            index++;
        } else if (0 == strcmp(Parameter_List[index].String_Value, "-log")) {
            index++;
            log_enable = Parameter_List[index].Integer_Value;
            index++;
        } else {
            index++;
        }
    }

    if (pktSize > 0) {
        if (dtim_iperf_tCxt->protocol == TCP) {
            dtim_iperf_tCxt->params.tx_params.packet_size = min(pktSize, IPERF_MAX_PACKET_SIZE_TCP);
        } else {
            dtim_iperf_tCxt->params.tx_params.packet_size = min(pktSize, IPERF_MAX_PACKET_SIZE_UDP);
        }
    } else {
        if (dtim_iperf_tCxt->protocol == TCP) {
            dtim_iperf_tCxt->params.tx_params.packet_size = IPERF_MAX_PACKET_SIZE_TCP;
        } else {
            dtim_iperf_tCxt->params.tx_params.packet_size = IPERF_MAX_PACKET_SIZE_UDP;
        }
    }

    iperf_timer = nt_create_timer(function_net_send_cb, NULL, NT_MS_TO_TICKS(dtim_time), TRUE);
    roaming_timer = nt_create_timer(function_reconnect_cb, NULL, NT_MS_TO_TICKS(roaming_time), TRUE);

    if (dtim_iperf_tCxt->protocol == TCP) {
        if ((dtim_iperf_tCxt->sock_peer = socket(AF_INET, SOCK_STREAM, 0)) == A_ERROR) {
            info_printf("Socket creation failed\n");
            goto ERROR_1;
        }
    } else {
        if ((dtim_iperf_tCxt->sock_peer = socket(AF_INET, SOCK_DGRAM, 0)) == A_ERROR) {
            info_printf("Socket creation failed\n");
            goto ERROR_1;
        }
    }

    struct sockaddr_in si_other;
    memset((char *)&si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(dtim_iperf_tCxt->params.tx_params.port);
    dtim_iperf_tCxt->params.tx_params.ip_address = ipAddress;
    si_other.sin_addr.s_addr = dtim_iperf_tCxt->params.tx_params.ip_address;

    if (connect(dtim_iperf_tCxt->sock_peer, (struct sockaddr *)&si_other, sizeof(si_other)) == -1) {
        info_printf("Connection failed\n");
        goto ERROR_1;
    }

    nt_start_timer(iperf_timer);

ERROR_1:

    return QAPI_OK;
}

const QAPI_Console_Command_t Powertest_Command_List[] = {
    /* cmd_function                     cmd_string      usage_string              description */
    {iperf_for_powertest, "iperf_for_powertest", "\n\niperf_for_powertest", "iperf in dtim sleep"},
};

const QAPI_Console_Command_Group_t Powertest_Command_Group = {
    "Powertest", /* Power Test */
    sizeof(Powertest_Command_List) / sizeof(QAPI_Console_Command_t),
    Powertest_Command_List,
};

void Initialize_powertest_Demo(void)
{
    powertest_shell_cmd_group_handle = QAPI_Console_Register_Command_Group(NULL, &Powertest_Command_Group);
    if (powertest_shell_cmd_group_handle) {
        info_printf("Powertest Registered \n");
    }

    if (nt_qurt_thread_create(powertest_wifi_auto_connect_task, "powertest_wifi_connect",
            STA_TASK_STACK_SIZE, NULL, 5, &wifi_auto_connect_task_handle) != pdPASS) {
        info_printf("powertest_wifi_connect create fail\n");
    }
}