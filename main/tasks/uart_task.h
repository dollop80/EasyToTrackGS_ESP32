#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#define BT_CONNECTED_GPIO  CONFIG_CONNECTED_LED_GPIO
#define BT_UART_TX_GPIO    CONFIG_UART_TX_GPIO
#define BT_UART_RX_GPIO    CONFIG_UART_RX_GPIO

#define BT_UART_BITRATE    CONFIG_UART_BITRATE

#define BT_UART_RX_BUF_SZ (1024 * CONFIG_UART_RX_BUFF_SIZE)
#define BT_UART_TX_BUF_SZ (1024 * CONFIG_UART_TX_BUFF_SIZE)

void initUart(uint8_t baud);
void uart1_event_task(void *pvParameters);
void uart2_event_task(void *pvParameters);
uint32_t uart_baud_detect(uart_port_t uart_num);