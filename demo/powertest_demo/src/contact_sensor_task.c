/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* Vishay VCNL3020 IR proximity sensor, used as a "contact" (open/closed)
 * sensor over I2C - INTERRUPT-DRIVEN. This replaces an earlier 500ms-poll
 * version of this file (measured ~301uA average, dominated by the CPU
 * waking every 500ms whether or not the contact had actually changed).
 *
 * Architecture:
 *  - The VCNL3020 free-runs self-timed proximity measurements in the
 *    background (vcnl3020_configure_selftimed(), CONTACT_SENSOR_PROX_RATE_
 *    SELECT) AND compares each result against LOW/HIGH threshold registers
 *    entirely in its own hardware, asserting its open-drain /INT pin only
 *    when a measurement crosses outside that window. The host does zero
 *    polling of the sensor.
 *  - /INT is wired to a QCC730 GPIO configured as a level-triggered input
 *    interrupt. The ISR (vcnl3020_gpio_isr()) does the bare minimum -
 *    notify a task - and returns; all I2C/MQTT work happens in
 *    contact_sensor_task(), which spends essentially all of its time
 *    blocked on xTaskNotifyWait() with NO periodic timer driving it.
 *  - The only other thing that can wake the task is a rare (10 min)
 *    MQTT-keepalive heartbeat timer - unrelated to the sensor, see
 *    MQTT_IDLE_SERVICE_INTERVAL_MS - and a one-shot notification from
 *    powertest_demo.c when WLAN connects/disconnects, so MQTT reacts to
 *    real network state changes instead of waiting for the next timer.
 *
 * IMPORTANT clarification on "waking the CPU from DTIM10 sleep": DTIM10 /
 * BMPS (see ENABLE_DTIM_BMPS_TASK in powertest_demo.c) is a WiFi-RADIO
 * power-save mode - it duty-cycles the RF front end between beacon/DTIM
 * windows, it does NOT stop the Cortex-M4 core itself. Only this SDK's
 * explicit deep-sleep path (qapi_deepsleep_enter()/qapi_imps_enter_sleep(),
 * NOT used here per the request to stay reachable) powers the CPU down far
 * enough to need an explicit "wakeup source" (see wkup_src in
 * qapi_deepsleep_enter()'s header doc). Under BMPS the CPU is always
 * running FreeRTOS (idle-task WFI at most), so a plain qapi_GPIO_Enable_
 * Interrupt() is both necessary AND sufficient - no special "wake from
 * DTIM10" API exists or is needed. What was actually costing ~301uA
 * previously was simply the CPU (and, seemingly, the radio along with it
 * on this SoC - the previous 500ms-poll build's log showed a "wakebmps"
 * line on every single poll cycle) never being allowed to go idle at all;
 * removing the periodic timer is what fixes that, independent of any
 * particular "wakeup source" mechanism.
 *
 * Register bit definitions (VCNL3020_CMD_..., VCNL3020_ICR_..., VCNL3020_ISR_...)
 * are verified against the upstream Linux kernel driver (drivers/iio/
 * proximity/vcnl3020.c), not just the datasheet. One correction versus
 * the original ask: VCNL3020 (proximity-only, no ALS channel) has no
 * "INT_THRES_SEL" field in its Interrupt Control Register - that field
 * exists on Vishay's combined ALS+proximity parts (e.g. VCNL4010) to
 * choose which channel drives the interrupt; VCNL3020 only ever has
 * proximity to select, so the kernel driver (and this file) only sets
 * INT_THRES_EN.
 *
 * I2C access pattern (vcnl3020_i2c_reopen() before every access) and the
 * reason for it (QCC730 I2C host controller goes stale under BMPS/DTIM10,
 * unrelated to the VCNL3020 itself) carries over unchanged from the
 * polling version - see that function's comment below.
 *
 * MQTT connect/publish/reconnect is the SHT40 publisher's pattern
 * (mqtt_printf_task.c) trimmed to publish-only (no subscribe) - same
 * broker/credentials as that task. */

#include <stdint.h>
#include <stdio.h>
#include <limits.h> /* ULONG_MAX, for the xTaskNotifyWait() clear-mask below */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "qapi_i2c.h"
#include "qapi_gpio.h"
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
/* 10 min - the ONLY periodic (non-event-driven) wake left in this whole
 * file. Purely to service MQTT keepalive/incoming traffic; entirely
 * unrelated to the sensor. See MQTT_IDLE_SERVICE_INTERVAL_MS below. */
#define MQTT_KEEPALIVE_SEC        600U
#define MQTT_CONNACK_TIMEOUT_MS   5000U
#define MQTT_TRANSPORT_TIMEOUT_MS 1000U
/* Minimum gap between MQTT (re)connect attempts. */
#define MQTT_RECONNECT_INTERVAL_MS 5000U
/* How often MQTT_ProcessLoop() runs when there is no status-change publish
 * pending - matches MQTT_KEEPALIVE_SEC 1:1 (same as mqtt_printf_task.c) so
 * this call doubles as the keepalive traffic itself, no separate idle
 * PINGREQ needed. Driven by mqtt_heartbeat_timer_cb() below, NOT by any
 * sensor activity - a real contact-state-change publish happens
 * immediately, independent of this timer, from the GPIO ISR path. */
#define MQTT_IDLE_SERVICE_INTERVAL_MS 600000U

/* ------------------------------------------------------------------ */
/*  VCNL3020 register map                                              */
/*  (Vishay datasheet: https://www.vishay.com/docs/84150/vcnl3020.pdf; */
/*   bit-exact values cross-checked against drivers/iio/proximity/     */
/*   vcnl3020.c in the upstream Linux kernel.)                         */
/* ------------------------------------------------------------------ */

/* Fixed 7-bit I2C slave address for every VCNL3020 - there is no address
 * pin/strap to change it, so if you have more than one on a bus you need
 * an I2C mux/expander. */
#define VCNL3020_I2C_ADDR              0x13

#define VCNL3020_REG_COMMAND           0x80  /* #0: control + status */
#define VCNL3020_REG_PROXIMITY_RATE    0x82  /* #2: self-timed measurement rate */
#define VCNL3020_REG_IR_LED_CURRENT    0x83  /* #3: IR LED drive current, 10 mA/step, 0-20 (0-200 mA) */
#define VCNL3020_REG_ICR               0x89  /* #9: Interrupt Control Register */
#define VCNL3020_REG_PROX_RESULT_MSB   0x87  /* #7 */
#define VCNL3020_REG_PROX_RESULT_LSB   0x88  /* #8 */
#define VCNL3020_REG_LOW_THR_HI        0x8A  /* #10: low threshold, high byte */
#define VCNL3020_REG_LOW_THR_LO        0x8B  /* #11: low threshold, low byte */
#define VCNL3020_REG_HIGH_THR_HI       0x8C  /* #12: high threshold, high byte */
#define VCNL3020_REG_HIGH_THR_LO       0x8D  /* #13: high threshold, low byte */
#define VCNL3020_REG_ISR               0x8E  /* #14: Interrupt Status Register */

/* Command register (0x80) bits. */
#define VCNL3020_CMD_SELFTIMED_EN_BIT  (1U << 0)  /* write 1: enable the state machine/LP oscillator for autonomous measurement */
#define VCNL3020_CMD_PROX_EN_BIT       (1U << 1)  /* write 1: enable periodic proximity measurement (used with selftimed_en) */

/* Interrupt Control Register (0x89) bits. VCNL3020 has no ALS channel, so
 * (unlike e.g. VCNL4010) there is no "INT_THRES_SEL" field to set - only
 * proximity can ever be the threshold source, so enabling threshold
 * interrupts at all (this one bit) is the whole story. */
#define VCNL3020_ICR_THRES_EN_BIT      (1U << 1)  /* write 1: enable threshold-crossing interrupt generation */

/* Interrupt Status Register (0x8E) bits - read to see which threshold(s)
 * fired, write the same bit(s) back to clear/acknowledge (releases the
 * open-drain /INT line back high). Both can only ever be set one at a
 * time in practice (a single measurement is either >= HIGH or < LOW, never
 * both), but the code below handles both defensively. */
#define VCNL3020_ISR_TH_HI_BIT         (1U << 0)  /* proximity rose to/above HIGH threshold -> CLOSED */
#define VCNL3020_ISR_TH_LOW_BIT        (1U << 1)  /* proximity fell below LOW threshold -> OPEN */

/* Proximity Rate register (0x82) values - how often the sensor measures on
 * its own; write the raw index 0-7 (low 3 bits only):
 *   0: 1.95 measurements/s (~513ms period, used here - lowest power)
 *   1: 3.90625/s   2: 7.8125/s   3: 15.625/s
 *   4: 31.25/s     5: 62.5/s     6: 125/s      7: 250/s
 * This governs how quickly a real open<->closed transition is noticed
 * (worst case ~1 period later) and directly sets the sensor's own average
 * current draw, since it is now measuring continuously in the background
 * regardless of whether the host is even awake. */
#define CONTACT_SENSOR_PROX_RATE_SELECT 0

/* IR LED drive current: 0-20 in 10 mA steps (0 mA - 200 mA/max range).
 * Higher = more reflected IR = larger raw counts/longer range, at the
 * cost of more current per self-timed conversion. Tune down if open vs
 * closed readings are saturating together (see the threshold comment
 * below for how to pick good values). */
#define CONTACT_SENSOR_LED_CURRENT_STEP 20

/* TUNE ME: raw proximity counts (0-65535) that the SENSOR's OWN hardware
 * compares each self-timed measurement against - this replaces the
 * previous software median-filter+hysteresis entirely; the VCNL3020 does
 * the debouncing/hysteresis itself now (a result has to land outside the
 * [LOW, HIGH) window to (re)trigger an interrupt at all, and once
 * CLOSED/OPEN it won't re-fire on the same side again).
 *   result >= HIGH -> /INT asserted, ISR TH_HI bit set -> report CLOSED
 *   result <  LOW   -> /INT asserted, ISR TH_LOW bit set -> report OPEN
 *   LOW <= result < HIGH -> inside the dead zone, no interrupt at all
 * Values below carried over already-tuned from this sensor's earlier
 * polling-mode testing (open baseline ~2350-2665, closed readings in the
 * tens of thousands) - re-tune the same way if you change hardware:
 * temporarily log raw proximity (e.g. via a one-off on-demand read) with
 * the contact physically open, then closed, and set LOW just above the
 * open ceiling, HIGH comfortably below the closed floor. */
#define CONTACT_SENSOR_PROXIMITY_THRESHOLD_LOW  3000U
#define CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH 5000U

/* VCNL3020 /INT is OPEN-DRAIN, active LOW, and stays asserted low until
 * the host clears VCNL3020_REG_ISR - it needs a pull-up (external ~10k to
 * 3.3V per Vishay's app note "Designing the VCNL3020 Into an Application"
 * is recommended; QAPI_GPIO_PULL_UP_E below enables the SoC's own,
 * usually weaker, internal pull-up as a backup/fallback, not a
 * replacement for a proper external one if you're seeing missed/flaky
 * interrupts).
 *
 * GPIO3 (silkscreen Pin 14 on the caller's board, not used by I2C or
 * JTAG) is used here per the request - I could not independently verify
 * the GPIO-number-to-silkscreen-pin mapping for your exact board variant
 * from SDK sources alone (no pinout table found under boards/mqm730i/),
 * so double-check against your board's schematic/pinout doc before
 * relying on it; the enum value QAPI_GPIO_ID3_E ("GPIO 3") itself is real
 * and defined in qapi_gpio.h regardless. */
#define CONTACT_SENSOR_INT_GPIO_ID QAPI_GPIO_ID3_E

/* Task notification bits (xTaskNotifyWait) - what woke contact_sensor_task
 * and why, so its one blocking wait can distinguish a real sensor event
 * from the unrelated MQTT heartbeat/WiFi-state notifications. */
#define CONTACT_SENSOR_NOTIFY_GPIO_BIT      (1U << 0) /* VCNL3020 /INT fired */
#define CONTACT_SENSOR_NOTIFY_MQTT_HEARTBEAT_BIT (1U << 1) /* periodic MQTT keepalive due */
#define CONTACT_SENSOR_NOTIFY_WIFI_BIT       (1U << 2) /* g_wifi_ready just changed - see contact_sensor_notify_wifi_changed() */

/* Minimum gap between full GPIO-triggered I2C service passes (reopen +
 * read/clear ISR + possible re-arm). NOT a "missed event" risk: the
 * VCNL3020 re-asserts /INT on every self-timed sample (~513ms) that
 * remains outside the armed threshold, including while genuinely
 * saturated at 0xFFFF - there is no register value that reliably means
 * "unreachable" on the high side (0xFFFF IS the ADC's own max, and real
 * close-up readings hit it), so vcnl3020_arm_next_transition() alone
 * cannot fully silence repeated same-side re-fires the way it can on the
 * low side (0 truly is unreachable there). This gate bounds how often
 * that repeated re-fire is allowed to cost a real I2C wake, without
 * delaying a genuine, isolated transition at all (it only ever throttles
 * a BURST of same-cause events, not the first one in a while - see
 * contact_sensor_task()'s NOTIFY_GPIO_BIT handling and s_gpio_catchup_timer). */
#define CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS 3000U

#define info_printf(msg, ...) printf("CONTACT_SENSOR: " msg, ##__VA_ARGS__)

static uint8_t     s_sensor_ready;
static uint8_t     s_contact_closed; /* current reported status */
static TaskHandle_t s_task_handle;
static volatile uint8_t s_started;
static uint32_t    s_next_gpio_process_ms; /* earliest time we'll do another full I2C service pass */
static TimerHandle_t s_gpio_catchup_timer; /* one-shot: guarantees a deferred event still gets serviced */

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
static uint32_t          s_mqtt_next_idle_service_ms; /* next allowed no-op ProcessLoop() call */

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

/* Called from contact_sensor_task() whenever it wakes for ANY reason (a
 * real GPIO interrupt, the WiFi-state notification, or the MQTT heartbeat
 * timer) - but only actually touches the radio when there's a real reason
 * to:
 *   - not connected yet: (re)connect, rate-limited to
 *     MQTT_RECONNECT_INTERVAL_MS.
 *   - connected, status changed since the last publish (s_mqtt_needs_publish,
 *     set by the GPIO ISR path): service + publish immediately.
 *   - connected, nothing changed: only service (MQTT_ProcessLoop(), which
 *     doubles as keepalive traffic) once every MQTT_IDLE_SERVICE_INTERVAL_MS. */
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
            s_mqtt_next_idle_service_ms = now_ms + MQTT_IDLE_SERVICE_INTERVAL_MS;
        } else {
            s_mqtt_next_attempt_ms = now_ms + MQTT_RECONNECT_INTERVAL_MS;
        }
        return;
    }

    if (!s_mqtt_needs_publish && now_ms < s_mqtt_next_idle_service_ms)
        return; /* nothing urgent, not due for a heartbeat - leave the radio alone */

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
            return;
        }
    }

    s_mqtt_next_idle_service_ms = now_ms + MQTT_IDLE_SERVICE_INTERVAL_MS;
}

/* Call from powertest_demo.c's WLAN event handler on CONNECT and
 * DISCONNECT, so mqtt_service() reacts to real network state changes
 * immediately instead of waiting for the next (rare) heartbeat. Safe to
 * call before the task exists - just a no-op notify to a NULL handle
 * guarded below. Uses the plain (non-ISR) xTaskNotify() since the WLAN
 * event handler runs in normal task context, not an interrupt. */
void contact_sensor_notify_wifi_changed(void)
{
    if (s_task_handle != NULL)
        xTaskNotify(s_task_handle, CONTACT_SENSOR_NOTIFY_WIFI_BIT, eSetBits);
}

/* FreeRTOS software timer callback (runs in the Timer Service Task's
 * normal context, not an ISR - same pattern already used for e.g.
 * roaming_timer/function_reconnect_cb in powertest_demo.c) - just wakes
 * contact_sensor_task to run mqtt_service() once. */
static void mqtt_heartbeat_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (s_task_handle != NULL)
        xTaskNotify(s_task_handle, CONTACT_SENSOR_NOTIFY_MQTT_HEARTBEAT_BIT, eSetBits);
}

/* One-shot timer, (re)started only when a GPIO-triggered service pass is
 * deferred (see CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS). Fires exactly once,
 * CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS after the deferral, and just
 * re-notifies the task - guarantees a deferred event still eventually
 * gets serviced (clearing the sensor's ISR register, re-arming, and
 * picking up any real status change) even if the physical /INT line is
 * stuck low the whole time and produces no further edges of its own
 * (EDGE_FALLING can't fire again on a line that never goes back high). */
static void gpio_catchup_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    if (s_task_handle != NULL)
        xTaskNotify(s_task_handle, CONTACT_SENSOR_NOTIFY_GPIO_BIT, eSetBits);
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

/* Close+reopen the QCC730's I2C MASTER instance - purely a host-side
 * driver operation, it does NOT touch the VCNL3020's own internal state
 * (self-timed measurement/threshold comparison keeps running on the
 * sensor regardless). Needed before every single I2C access while
 * DTIM10/BMPS is active: the host I2C peripheral's state goes stale under
 * BMPS otherwise (same root cause already documented for the SHT40 sensor
 * in mqtt_printf_task.c's sht40_measure_and_publish()). With this file now
 * interrupt-driven, this only runs when there's an actual event to
 * service - not on a timer. Safe to call repeatedly - qapi_I2CM_Close() on
 * an already-closed instance is expected to just fail harmlessly. */
static int vcnl3020_i2c_reopen(void)
{
    qapi_I2CM_Config_t cfg = { .Blocking = TRUE, .Dma = FALSE };
    qapi_Status_t status;

    qapi_I2CM_Close(QAPI_I2C_INSTANCE_SE0_E);
    status = qapi_I2CM_Open(QAPI_I2C_INSTANCE_SE0_E, &cfg);
    if (status != QAPI_OK) {
        info_printf("I2C open failed, status=%d\n", (int)status);
        return -1;
    }
    return 0;
}

/* Reprogram the threshold registers so ONLY the boundary relevant to
 * leaving the CURRENT reported state is reachable - the other boundary is
 * pushed to an unreachable extreme (0 or 0xFFFF).
 *
 * Why this exists: the VCNL3020 turned out to re-assert its threshold
 * interrupt on EVERY self-timed sample that lands outside [LOW, HIGH),
 * not just once at the moment of crossing - confirmed on hardware as
 * steady, high-rate interrupt traffic (and correspondingly high average
 * power) for as long as the contact stayed in one state, which stopped
 * the instant the physical /INT wire was disconnected. Vishay's
 * datasheet markets LOW/HIGH as "hysteresis", but with no host-side logic
 * that only describes the width of the dead zone, not one-shot edge
 * behavior. Dynamically retargeting which single boundary is "live" -
 * i.e. treating this pair of registers as a Schmitt trigger in software -
 * is what actually turns it into one interrupt per real transition:
 *   currently OPEN:   arm only "goes >= HIGH" (LOW pinned to 0, unreachable)
 *   currently CLOSED: arm only "goes <  LOW"  (HIGH pinned to 0xFFFF, unreachable)
 * Called once at startup/recovery (assuming OPEN until the boot read or
 * first interrupt says otherwise) and again every time contact_sensor_task()
 * handles a real transition, to arm the now-opposite boundary. */
static int vcnl3020_arm_next_transition(uint8_t closed)
{
    uint16_t low = closed ? CONTACT_SENSOR_PROXIMITY_THRESHOLD_LOW : 0x0000U;
    uint16_t high = closed ? 0xFFFFU : CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH;

    /* Threshold registers are 16-bit, MSB-first (matches the result
     * registers' own byte order). */
    if (vcnl3020_write_reg(VCNL3020_REG_LOW_THR_HI, (uint8_t)(low >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_LOW_THR_LO, (uint8_t)(low & 0xFF)) != 0) {
        info_printf("VCNL3020 low threshold write failed\n");
        return -1;
    }

    if (vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_HI, (uint8_t)(high >> 8)) != 0 ||
        vcnl3020_write_reg(VCNL3020_REG_HIGH_THR_LO, (uint8_t)(high & 0xFF)) != 0) {
        info_printf("VCNL3020 high threshold write failed\n");
        return -1;
    }

    return 0;
}

/* One-time (or recovery-time) sensor configuration: LED current, sample
 * rate, threshold registers (assumes OPEN - corrected right after by the
 * boot read in contact_sensor_task(), or by s_contact_closed already
 * holding the last known state on a recovery call), arm threshold-
 * interrupt generation, then arm self-timed continuous measurement last
 * (selftimed_en | prox_en) so every other register is already correct
 * before the sensor starts free-running. Deliberately never called on a
 * timer - only at startup and after a genuine read/interrupt-handling
 * failure (see contact_sensor_task() below); vcnl3020_i2c_reopen() alone
 * handles the routine per-event BMPS workaround. */
static int vcnl3020_configure_selftimed(void)
{
    if (vcnl3020_write_reg(VCNL3020_REG_IR_LED_CURRENT, CONTACT_SENSOR_LED_CURRENT_STEP) != 0) {
        info_printf("VCNL3020 LED current write failed\n");
        return -1;
    }

    if (vcnl3020_write_reg(VCNL3020_REG_PROXIMITY_RATE, CONTACT_SENSOR_PROX_RATE_SELECT) != 0) {
        info_printf("VCNL3020 proximity rate write failed\n");
        return -1;
    }

    if (vcnl3020_arm_next_transition(s_contact_closed) != 0)
        return -1;

    if (vcnl3020_write_reg(VCNL3020_REG_ICR, VCNL3020_ICR_THRES_EN_BIT) != 0) {
        info_printf("VCNL3020 interrupt control write failed\n");
        return -1;
    }

    if (vcnl3020_write_reg(VCNL3020_REG_COMMAND,
            VCNL3020_CMD_SELFTIMED_EN_BIT | VCNL3020_CMD_PROX_EN_BIT) != 0) {
        info_printf("VCNL3020 self-timed enable write failed\n");
        return -1;
    }

    return 0;
}

/* Full (re)init: reopen the I2C bus and (re)arm self-timed + threshold-
 * interrupt mode. Used at task startup and to recover from a real
 * interrupt-handling failure. */
static int vcnl3020_open(void)
{
    if (vcnl3020_i2c_reopen() != 0) {
        s_sensor_ready = 0;
        return -1;
    }

    if (vcnl3020_configure_selftimed() != 0) {
        s_sensor_ready = 0;
        return -1;
    }

    s_sensor_ready = 1;
    return 0;
}

/* Direct read of the latest self-timed result - no "data ready" check
 * needed here: by the time /INT has asserted, the sensor has by
 * definition already completed the measurement that triggered the
 * threshold comparison, so a fresh result is guaranteed to already be
 * sitting in these two registers. */
static int vcnl3020_read_result(uint16_t *proximity)
{
    uint8_t msb, lsb;

    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_MSB, &msb) != 0)
        return -1;
    if (vcnl3020_read_reg(VCNL3020_REG_PROX_RESULT_LSB, &lsb) != 0)
        return -1;

    *proximity = ((uint16_t)msb << 8) | lsb;
    return 0;
}

/* Read+clear the Interrupt Status Register and report which threshold(s)
 * fired via *isr_bits (VCNL3020_ISR_TH_HI_BIT / VCNL3020_ISR_TH_LOW_BIT).
 * Returns 0 on success (even if, rarely, no bit turns out to be set - a
 * possible spurious/shared-line wake), -1 on a real I2C failure. */
static int vcnl3020_handle_interrupt(uint8_t *isr_bits)
{
    uint8_t isr;

    if (vcnl3020_read_reg(VCNL3020_REG_ISR, &isr) != 0)
        return -1;

    *isr_bits = isr & (VCNL3020_ISR_TH_HI_BIT | VCNL3020_ISR_TH_LOW_BIT);

    if (*isr_bits != 0) {
        /* Write-1-to-clear only the bit(s) that were actually set, so the
         * open-drain /INT line is released back high and we don't
         * accidentally disturb any other status bit. */
        if (vcnl3020_write_reg(VCNL3020_REG_ISR, *isr_bits) != 0)
            return -1;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/*  GPIO interrupt (VCNL3020 /INT)                                     */
/* ------------------------------------------------------------------ */

/* Runs in GPIO interrupt-controller context - keep this to the bare
 * minimum (no I2C, no printf, no MQTT, and - critically, see below - no
 * qapi_GPIO_Enable/Disable_Interrupt() either): just hand off to the task
 * via an ISR-safe notification, same idiom already used elsewhere in this
 * SDK for a driver callback waking a task (see spi_rx_data_indication() in
 * demo/ambient_power_demo/src/ambient_power_demo.c).
 *
 * Trigger type is EDGE_FALLING (see qapi_GPIO_Enable_Interrupt() call
 * below), not level-triggered, and that choice is load-bearing, not
 * cosmetic: an earlier LEVEL_LOW version of this file needed to mask the
 * GPIO interrupt right here (the VCNL3020's /INT stays asserted low until
 * the task clears VCNL3020_REG_ISR over I2C, and a level trigger left
 * enabled during that window re-fires the instant this ISR returns,
 * forever, without ever letting the notified task run - a real
 * interrupt-storm lockup, reproduced on hardware). The natural fix,
 * calling qapi_GPIO_Disable_Interrupt() here to mask it, turned out to be
 * ILLEGAL: that API takes an internal critical section, which FreeRTOS's
 * ARM_CM4F port asserts on if called from actual interrupt context
 * (`Assert @vPortEnterCritical` - also reproduced on hardware) - only
 * "...FromISR()" APIs are legal here, and qapi_GPIO_Disable_Interrupt()
 * is not one. EDGE_FALLING sidesteps the entire problem: it fires exactly
 * once per high-to-low transition regardless of how long the line then
 * stays low, so there is nothing to mask/unmask at all - the interrupt
 * can just stay enabled permanently after the one qapi_GPIO_Enable_
 * Interrupt() call in contact_sensor_task(). */
static void vcnl3020_gpio_isr(qapi_GPIO_CB_Data_t data)
{
    BaseType_t woken = pdFALSE;

    (void)data;

    if (s_task_handle != NULL) {
        xTaskNotifyFromISR(s_task_handle, CONTACT_SENSOR_NOTIFY_GPIO_BIT, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* ------------------------------------------------------------------ */
/*  Task                                                                */
/* ------------------------------------------------------------------ */

static void contact_sensor_task(void *arg)
{
    qapi_GPIO_Config_t gpio_cfg;
    TimerHandle_t heartbeat_timer;

    (void)arg;

    while (vcnl3020_open() != 0) {
        info_printf("retrying I2C open in 5s\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    /* qapi_GPIO_Init() is declared in qapi_gpio.h but is NOT linked into
     * this image (confirmed by an actual link failure: "undefined
     * reference to qapi_GPIO_Init" when this file called it) - board
     * bring-up evidently already handles whatever it would have done, and
     * no other demo in this SDK calls it either. Do not call it. */

    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.Dir = QAPI_GPIO_INPUT_E;
    gpio_cfg.Pull = QAPI_GPIO_PULL_UP_E; /* backup for the recommended external pull-up - see CONTACT_SENSOR_INT_GPIO_ID comment */
    gpio_cfg.Drive = QAPI_GPIO_DRIVE_LOW_E; /* irrelevant for an input, but keep it in a defined state */
    qapi_GPIO_Config(CONTACT_SENSOR_INT_GPIO_ID, &gpio_cfg);

    /* EDGE_FALLING, not level - see the long comment on vcnl3020_gpio_isr()
     * for why: a level trigger needs the interrupt masked while /INT sits
     * low waiting for the task to service it, and the only way to do that
     * (qapi_GPIO_Disable_Interrupt()) is illegal from actual ISR context
     * on this RTOS port (reproduced on hardware: both an interrupt-storm
     * lockup with level+no masking, and a vPortEnterCritical assert with
     * level+masking-from-ISR). Registered once here and left enabled
     * permanently - no re-arming needed anywhere else in this file. */
    qapi_GPIO_Enable_Interrupt(CONTACT_SENSOR_INT_GPIO_ID, QAPI_GPIO_TRIGGER_EDGE_FALLING_E, vcnl3020_gpio_isr, 0);

    info_printf("VCNL3020 armed: self-timed + threshold interrupt on GPIO%d, LOW=%u HIGH=%u\n",
        (int)CONTACT_SENSOR_INT_GPIO_ID, (unsigned)CONTACT_SENSOR_PROXIMITY_THRESHOLD_LOW,
        (unsigned)CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH);

    /* One-time-only boot read (NOT a recurring poll): give the sensor one
     * full self-timed period to complete its first conversion, then read
     * the raw result directly, purely to log/publish a known starting
     * status. Without this, if the initial proximity happens to land
     * inside the [LOW, HIGH) dead zone, no interrupt fires until the
     * first real crossing and the log has no starting line at all - a
     * simple documented trade-off of hardware-threshold hysteresis. */
    {
        uint16_t boot_proximity = 0;

        vTaskDelay(pdMS_TO_TICKS(600));
        vcnl3020_i2c_reopen();
        if (vcnl3020_read_result(&boot_proximity) == 0) {
            s_contact_closed = (boot_proximity >= CONTACT_SENSOR_PROXIMITY_THRESHOLD_HIGH) ? 1 : 0;
            info_printf("initial status: %s (raw=%u)\n", s_contact_closed ? "CLOSED" : "OPEN", (unsigned)boot_proximity);
            s_mqtt_needs_publish = 1;
            /* vcnl3020_configure_selftimed() armed assuming OPEN (s_contact_closed
             * was still 0 at that point) - correct the armed boundary now
             * that the real starting state is known, so a sensor that
             * boots already CLOSED doesn't immediately re-fire on its own
             * still-CLOSED next sample (the exact high-power bug this
             * whole mechanism exists to prevent). */
            vcnl3020_arm_next_transition(s_contact_closed);
        } else {
            info_printf("initial read failed, waiting for first interrupt\n");
        }
    }

    /* The only periodic (non-event-driven) activity in this entire task -
     * an MQTT keepalive heartbeat, unrelated to the sensor. Everything
     * else below is purely interrupt/notification-driven. */
    heartbeat_timer = nt_create_timer(mqtt_heartbeat_timer_cb, NULL,
        pdMS_TO_TICKS(MQTT_IDLE_SERVICE_INTERVAL_MS), pdTRUE);
    if (heartbeat_timer != NULL) {
        nt_start_timer(heartbeat_timer);
    } else {
        info_printf("MQTT heartbeat timer create failed\n");
    }

    /* One-shot, NOT started here - see CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS
     * and gpio_catchup_timer_cb(). Only (re)started, from scratch each
     * time, when the GPIO handling below actually needs to defer a
     * same-cause re-fire. */
    s_gpio_catchup_timer = nt_create_timer(gpio_catchup_timer_cb, NULL,
        pdMS_TO_TICKS(CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS), pdFALSE);
    if (s_gpio_catchup_timer == NULL) {
        info_printf("GPIO catch-up timer create failed\n");
    }

    /* Kick an initial MQTT connect attempt now, in case WiFi is already
     * up by the time we get here (Initialize_powertest_Demo() starts this
     * task and the WLAN connect task independently, so either could run
     * first). */
    mqtt_service(hres_timer_curr_time_ms());

    for (;;) {
        uint32_t notified = 0;
        uint32_t now_ms;

        /* THE entire point of this design: block indefinitely with no
         * timeout at all. The CPU is free to go fully idle between real
         * events - no periodic sensor poll exists anywhere in this file
         * anymore. */
        xTaskNotifyWait(0, ULONG_MAX, &notified, portMAX_DELAY);

        now_ms = hres_timer_curr_time_ms();

        if ((notified & CONTACT_SENSOR_NOTIFY_GPIO_BIT) && now_ms < s_next_gpio_process_ms) {
            /* Defer: we very recently did a full service pass and this is
             * (almost certainly) the sensor re-firing on the same,
             * still-unchanged side rather than a second genuine
             * transition - see CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS. Do NOT
             * touch I2C at all here (that round trip is exactly the cost
             * being throttled); just guarantee we come back and actually
             * service it once the gap has elapsed, in case nothing else
             * wakes us before then. nt_start_timer() on an already-running
             * one-shot timer restarts its countdown, which is fine here -
             * worst case this just pushes the eventual catch-up out by
             * however much more bounce/re-fire keeps arriving, exactly as
             * intended. */
            if (s_gpio_catchup_timer != NULL)
                nt_start_timer(s_gpio_catchup_timer);
            notified &= ~CONTACT_SENSOR_NOTIFY_GPIO_BIT;
        }

        if (notified & CONTACT_SENSOR_NOTIFY_GPIO_BIT) {
            uint8_t isr_bits = 0;

            s_next_gpio_process_ms = now_ms + CONTACT_SENSOR_MIN_INTERRUPT_GAP_MS;

            /* Host-side I2C bus workaround only - see vcnl3020_i2c_reopen()
             * comment. Does not disturb the sensor's own state. */
            vcnl3020_i2c_reopen();

            if (vcnl3020_handle_interrupt(&isr_bits) == 0 && isr_bits != 0) {
                /* The sensor's own hardware comparator already made the
                 * open/closed decision (that is what triggered /INT in
                 * the first place) - just reflect it, no re-derivation
                 * against the threshold constants needed. */
                uint8_t new_closed = (isr_bits & VCNL3020_ISR_TH_HI_BIT) ? 1 :
                                      (isr_bits & VCNL3020_ISR_TH_LOW_BIT) ? 0 : s_contact_closed;

                /* Only treat this as a real event if the status actually
                 * flips. A stale/duplicate report (e.g. two ISR bits set
                 * at once from a noisy/bouncing /INT release producing
                 * more than one GPIO edge for what was electrically one
                 * transition, or a genuinely spurious re-read of the same
                 * side) must still have been cleared above (so the line
                 * stays serviceable), but should NOT re-print, re-publish,
                 * or needlessly rewrite the threshold registers again -
                 * that redundant work was itself shown to add measurable
                 * average current. */
                if (new_closed != s_contact_closed) {
                    uint16_t proximity = 0;

                    /* Best-effort - if this particular read fails we still
                     * trust the ISR bit itself for the status decision, we
                     * just won't have a raw value to log. */
                    (void)vcnl3020_read_result(&proximity);

                    s_contact_closed = new_closed;

                    info_printf("[%lu.%03lus] status changed: %s (raw=%u)\n",
                        (unsigned long)(now_ms / 1000), (unsigned long)(now_ms % 1000),
                        s_contact_closed ? "CLOSED" : "OPEN", (unsigned)proximity);
                    s_mqtt_needs_publish = 1;

                    /* Re-target which single threshold is "live" for the
                     * new state, so the sensor's own next same-side sample
                     * can no longer re-trip the interrupt - only the
                     * opposite-direction crossing can now fire. See
                     * vcnl3020_arm_next_transition(). */
                    vcnl3020_arm_next_transition(s_contact_closed);
                }
            } else if (isr_bits == 0) {
                /* GPIO fired but the sensor's ISR register was already
                 * clear - most likely we raced a previous event's clear
                 * with the line not yet released, or a genuinely spurious
                 * wake. Not an error, nothing to report. */
            } else {
                info_printf("[%lu.%03lus] interrupt read failed, reconfiguring sensor\n",
                    (unsigned long)(now_ms / 1000), (unsigned long)(now_ms % 1000));
                vcnl3020_open(); /* also re-arms thresholds for the current s_contact_closed via vcnl3020_configure_selftimed() */
            }
            /* No GPIO-level re-arm needed here (unlike an earlier
             * level-triggered version of this file): EDGE_FALLING was
             * registered once in contact_sensor_task() above and stays
             * enabled permanently - see vcnl3020_gpio_isr()'s comment for
             * why. The SENSOR's threshold registers, however, DO need
             * re-arming every time, handled above/via vcnl3020_open(). */
        }

        if (notified & (CONTACT_SENSOR_NOTIFY_MQTT_HEARTBEAT_BIT | CONTACT_SENSOR_NOTIFY_WIFI_BIT)) {
            mqtt_service(now_ms);
        } else if (notified & CONTACT_SENSOR_NOTIFY_GPIO_BIT) {
            /* A real status change also needs mqtt_service() run to
             * actually flush s_mqtt_needs_publish - do it here rather
             * than waiting for the next heartbeat. */
            mqtt_service(now_ms);
        }
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