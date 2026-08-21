/*
 * my_mqtt_demo - entry point.
 *
 * Per this SDK's demo convention (build/freertos/common/application_code/
 * main.c's qccsdk_start_app_task()): app_init() runs first, then a
 * dedicated "qmain" task calls app_main() and is deleted when it returns.
 */

#include <string.h>
#include <stdint.h>

#ifdef FERMION_SILICON
extern uint32_t UART_Send_direct(char *txbuf, uint32_t buflen);
#define UART_SEND_DIRECT(str) UART_Send_direct((str), strlen(str))
#else
#define UART_SEND_DIRECT(str)
#endif

void app_init(void)
{
    UART_SEND_DIRECT("app_init entry\r\n");
    /* WLAN/MQTT startup call goes here once wifi_mqtt.c exists (Phase 3/4). */
    UART_SEND_DIRECT("app_init over\r\n");
}

void app_main(void)
{
    UART_SEND_DIRECT("app_main entry\r\n");
    UART_SEND_DIRECT("my_mqtt_demo running\r\n");
    UART_SEND_DIRECT("app_main over\r\n");
}
