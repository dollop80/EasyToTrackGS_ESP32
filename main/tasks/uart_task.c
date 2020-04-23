#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver/uart.h"
#include "uart_task.h"
#include "protocol_detection.h"
#include "tracker.h"
#include "sdkconfig.h"
#include "define.h"
#include "../telem.h"

#define BUF_SIZE (1024/4)

#define RD_U1_BUF_SIZE (BUF_SIZE)
#define RD_U2_BUF_SIZE (BUF_SIZE)

extern TaskHandle_t xHandleUart1;
extern TaskHandle_t xHandleUart2;
extern int16_t gBThandle;
extern int16_t gWFsock;
extern bool gFirstU2byte;

static QueueHandle_t uart2_queue;
static QueueHandle_t uart1_queue;

// Receive buffer to collect incoming data
uint8_t rxbuf[256];
// Register to collect data length
uint16_t urxlen;

uint8_t gTelemPhase = 0;

#define U2RXD 33
#define U2TXD 32

static const char TAG[] = "ETT-UART";

void initUart(uint8_t baud)
{
	int baudrate = 9600;
	switch (baud)
	{
		case 1:
		baudrate = 9600;
		break;
		
		case 2:
		baudrate = 19200;
		break;

		case 3:
		baudrate = 38400;
		break;

		case 4:
		baudrate = 57600;
		break;

		case 5:
		baudrate = 115200;
		break;
		
		default:
		baudrate = 57600;
		break;
	}
		
    //Configure UART1
    uart_config_t uart1_config = {
        .baud_rate = BT_UART_BITRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, BT_UART_TX_GPIO, BT_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BT_UART_RX_BUF_SZ*2, BT_UART_TX_BUF_SZ*2, 20, &uart1_queue, 0));
	
    //Configure UART2
    uart_config_t uart2_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BT_UART_RX_BUF_SZ*2, BT_UART_RX_BUF_SZ*2, 20, &uart2_queue, 0));

#if 0
	uint32_t baud = uart_baud_detect(2);
	for(int i = 0; i<4; i++)
	{
		ESP_LOGI(TAG, "baud is %d", baud);
		vTaskDelay(500);
	}
	ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_2));
	uart2_config.baud_rate = baud;
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BT_UART_RX_BUF_SZ*2, BT_UART_RX_BUF_SZ*2, 20, &uart2_queue, 0));
#endif	

}


uint32_t uart_baud_detect(uart_port_t uart_num)
{
	
    int low_period = 0;
    int high_period = 0;
    uint32_t intena_reg = READ_PERI_REG(UART_INT_ENA_REG(uart_num)); //UART[uart_num]->int_ena.val;
    //Disable the interruput.
    WRITE_PERI_REG(UART_INT_RAW_REG(uart_num), 0);//->int_ena.val = 0;
    WRITE_PERI_REG(UART_INT_CLR_REG(uart_num), 0);//UART[uart_num]->int_clr.val = ~0;
    //Filter
    SET_PERI_REG_MASK(UART_AUTOBAUD_REG(uart_num), 4);//UART[uart_num]->auto_baud.glitch_filt = 4;
	
    //Clear the previous result
    SET_PERI_REG_MASK(UART_AUTOBAUD_REG(uart_num), (UART_GLITCH_FILT << UART_GLITCH_FILT_S | UART_AUTOBAUD_EN));
    SET_PERI_REG_MASK(UART_AUTOBAUD_REG(uart_num), 0x08 << UART_GLITCH_FILT_S | UART_AUTOBAUD_EN);
    while(GET_PERI_REG_MASK(UART_RXD_CNT_REG(uart_num), UART_RXD_EDGE_CNT) < 100) {
       vTaskDelay(1);
    }
    low_period = GET_PERI_REG_MASK(UART_LOWPULSE_REG(uart_num), UART_LOWPULSE_MIN_CNT);//UART[uart_num]->lowpulse.min_cnt;
    high_period = GET_PERI_REG_MASK(UART_HIGHPULSE_REG(uart_num), UART_HIGHPULSE_MIN_CNT);//UART[uart_num]->highpulse.min_cnt;
    // disable the baudrate detection
    SET_PERI_REG_MASK(UART_AUTOBAUD_REG(uart_num), (UART_GLITCH_FILT << UART_GLITCH_FILT_S | UART_AUTOBAUD_EN)); //UART[uart_num]->auto_baud.en = 0;
    //Reset the fifo;
    //uart_reset_rx_fifo(uart_num);
    WRITE_PERI_REG(UART_INT_ENA_REG(uart_num), intena_reg);//UART[uart_num]->int_ena.val = intena_reg;
    ///Set the clock divider reg
    ///UART[uart_num]->clk_div.div_int = (low_period > high_period) ? high_period : low_period;

    ///Return the divider. baud = APB / divider;
    //return (low_period > high_period) ? high_period : low_period;
	int32_t divisor = (low_period > high_period) ? high_period : low_period;
	int32_t baudrate = UART_CLK_FREQ / divisor;

    static const int default_rates[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 256000, 460800, 921600, 1843200, 3686400};

    size_t i;
    for (i = 1; i < sizeof(default_rates) / sizeof(default_rates[0]) - 1; i++)	// find the nearest real baudrate
    {
        if (baudrate <= default_rates[i])
        {
            if (baudrate - default_rates[i - 1] < default_rates[i] - baudrate) {
                i--;
            }
            break;
        }
    }

    return default_rates[i];
}




void uart2_event_task(void *pvParameters)
{
	uint8_t res;
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_U2_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_U2_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_2);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_2, dtmp, event.size, portMAX_DELAY);
					gFirstU2byte = true;
                    for(int i = 0; i < event.size; i++)
					{
						if(!getProtocol())
						{
							enableProtocolDetection();
							protocolDetectionParser(dtmp[i]);
						}
						else
						{
							res = telem_parse(getProtocol(), dtmp[i]);
							if(res)
							{
								process_gps();
								gTelemPhase++;
								//ESP_LOGI(TAG, "[OK], %d",getProtocol());		
							}								
						}
					}
					//ESP_LOGI(TAG, "[DATA EVT]:");
                    //uart_write_bytes(UART_NUM_2, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo2 overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring2 buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_2);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart2 rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart2 parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart2 frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart2 event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}



void uart1_event_task(void *pvParameters)
{	 
	esp_err_t res;
	uint8_t unpack_res;
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_U1_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_U1_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", UART_NUM_2);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_1, dtmp, event.size, portMAX_DELAY);
					unpack_res = decode_packet_for_host(dtmp, event.size);
						
					if(gBThandle > 0)
					{					
						if(!unpack_res)
						{
							res = esp_spp_write(gBThandle, event.size, dtmp);
							if(res != ESP_OK) {
								ESP_LOGE(TAG, "Error sending SPP: errno %d", res);
								break;
							}
						}
						else if(getProtocol() == TP_MSV)
						{
							res = esp_spp_write(gBThandle, event.size, dtmp);
							if(res != ESP_OK) {
								ESP_LOGE(TAG, "Error sending SPP: errno %d", res);
								break;
							}
						}
					} 
					else if (gWFsock != -1)
					{
						if(!unpack_res)
						{
							res = send(gWFsock, dtmp, event.size, 0);
							if (res < 0) {
								ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
								break;
							}
						}
						else if(getProtocol() == TP_MSV)
						{
							res = send(gWFsock, dtmp, event.size, 0);
							if (res < 0) {
								ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
								break;
							}
						}						
					}
					
                    for(int i = 0; i < event.size; i++)
					{

					}
					//ESP_LOGI(TAG, "[DATA EVT], size %d=", event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo1 overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer1 full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_1);
                    xQueueReset(uart1_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart1 rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart1 parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart1 frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart1 event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}