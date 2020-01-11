/*
Copyright (c) 2017-2019 Tony Pottier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

@file main.c
@author Tony Pottier
@brief Entry point for the ESP32 application.
@see https://idyl.io
@see https://github.com/tonyp7/esp32-wifi-manager
*/

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
#include "components/wifi-manager/src/wifi_manager.h"
#include "components/avr-isp/avr_isp_helper.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "mbedtls/md5.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_spiffs.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "components/u8g2/csrc/u8g2.h"
#include "u8g2_esp32_hal.h"
#include "sdkconfig.h"

#include "define.h"

#include "telem.h"
#include "components/protocols/simple_bin.h"
#include "protocol_detection.h"
#include "tracker.h"

#define CONFIG_EXAMPLE_IPV4
#define PORT 23

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/


#define BT_DEV_NAME_PREFIX CONFIG_DEV_NAME_PREFIX
#define BT_DEV_NAME_PREFIX_LEN (sizeof(BT_DEV_NAME_PREFIX) - 1)

#define BT_CONNECTED_GPIO  CONFIG_CONNECTED_LED_GPIO
#define BT_UART_TX_GPIO    CONFIG_UART_TX_GPIO
#define BT_UART_RX_GPIO    CONFIG_UART_RX_GPIO

#define BT_LED_CONNECTED    0
#define BT_LED_DISCONNECTED 1

#define BT_UART_BITRATE    CONFIG_UART_BITRATE

#define BT_UART_RX_BUF_SZ (1024 * CONFIG_UART_RX_BUFF_SIZE)
#define BT_UART_TX_BUF_SZ (1024 * CONFIG_UART_TX_BUFF_SIZE)

#define EXAMPLE_ESP_WIFI_SSID "miau"
#define EXAMPLE_ESP_WIFI_PASS "happiness"
#define EXAMPLE_MAX_STA_CONN 2

#define BUF_SIZE (1024/4)

#define RD_U1_BUF_SIZE (BUF_SIZE)
#define RD_U2_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart2_queue;
static QueueHandle_t uart1_queue;

// Receive buffer to collect incoming data
uint8_t rxbuf[256];
uint8_t txbuf[256];
// Register to collect data length
uint16_t urxlen;

// SDA - GPIO21
#define PIN_SDA 21

// SCL - GPIO22
#define PIN_SCL 22

#define U2RXD 33
#define U2TXD 32

TELEM_DATA telem_data;
TO_HOST_DATA to_host_data;
FROM_HOST_DATA servo_settings_data;
AZ_ELEV_DATA az_elev_data;

const char * name = "/spiffs/GroundSt.hex";
int foundSig = -1;
uint8_t fuses [5];
esp_err_t ret;
spi_device_handle_t spi;


uint8_t serialBuffer[200]; // this hold the imcoming string from serial O string

/* @brief tag used for ESP serial console messages */
static const char TAG[] = "ETT";

static void tcp_server_task(void *pvParameters);

#define SPP_BUFF_SZ 128
//static uint8_t spp_buff[SPP_BUFF_SZ];

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

u8g2_t u8g2; // a structure which will contain all the data for one display

TaskHandle_t xHandleTCP = NULL;
TaskHandle_t xHandleMonitor = NULL;
TaskHandle_t xHandleOled = NULL;
TaskHandle_t xHandleUart1 = NULL;
TaskHandle_t xHandleUart2 = NULL;
TaskHandle_t xHandleSendToHost = NULL;
TaskHandle_t xHandleTracker = NULL;

int16_t handle;
int16_t sock = -1;

typedef enum {BT, WIFI} connection_type;
typedef enum {B9600, B19200, B38400, B57600, B115200} uart_baudrate;
typedef enum {NO, AP, STA} wifi_mode;
typedef enum {TRACKING_V, TRACKING_T, MANUAL, SETUP} tracker_mode;

wifi_mode wifi_m = NO;
tracker_mode tracker_m = TRACKING_V;

//bool manual_tracker_mode = false;
uint32_t upTimeS = 0;
uint16_t upTimeM = 0;
uint8_t upTimeSs = 0;
bool firstU2byte = false;

uint16_t gTelAzimuth;
uint8_t gTelElevation;

struct g_settings
{
	connection_type con_t;
	//telem_type tel_t;
	uart_baudrate uart_b;
};


//
// Define UART2 interrupt subroutine to ackowledge interrupt
//
/*
static void IRAM_ATTR uart2_intr_handle(void *arg)
{
  uint16_t rx_fifo_len, status;
  uint16_t i=0;
  
  status = UART2.int_st.val; // read UART interrupt Status
  rx_fifo_len = UART2.status.rxfifo_cnt; // read number of bytes in UART buffer
  
  while(rx_fifo_len){
   rxbuf[i++] = UART2.fifo.rw_byte; // read all bytes
   rx_fifo_len--;
 }
  
 // after reading bytes from buffer clear UART interrupt status
 uart_clear_intr_status(UART_NUM_2, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);

// a test code or debug code to indicate UART receives successfully,
// you can redirect received byte as echo also
 uart_write_bytes(UART_NUM_2, (const char*) "RX Done", 7);

}
*/

void initSPI()
{
	ESP_LOGI(TAG, "Initializing SPI");
	
	spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=4
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*100,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the programmer to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
	
	gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_NUM_RST, 1);
}

void initSPIFS()
{
	ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = false
    };
	// Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
	
	size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

void runProgrammer()
{
	ESP_LOGI(TAG, "Starting Programmer");
	
	target_poweron(spi);
	
	foundSig = getSignature (spi);
	getFuseBytes (spi, fuses);
	printf("\n\rFuses: ");
	  for(int i = 0; i < 5; i++)
		  printf(" 0x%02X ",fuses[i]);
	  // don't have signature? don't proceed
	  if (foundSig == -1)
		{
			stopProgramming (spi);
			return;
		}  // end of no signature	
	
	// verify that MCU flash is up to date
	if (readHexFile(spi, name, verifyFlash))
	{
      printf("\n\rStarting MCU flash update process\n\r");
	  bool ok = writeFlashContents (spi, name);
	  if(ok)
		ESP_LOGI(TAG, "DONE!!!!!");
	  else
		ESP_LOGI(TAG, "FAILED!!!!!"); 
	}
	else
	{
		printf("\n\rMCU flash is up to date\n\r");
	}
	stopProgramming (spi);

	gpio_set_level(PIN_NUM_RST, 1);  
}



static void sendHostHomeMessageToGS()
{
	    uint8_t encodedbuff[11];
        uint8_t bufftosend[11+6];
		uint8_t cnt = 11;
        uint8_t type = 0x07;//(uint8_t)(GsProtoIDs.HOST2GS_HOST_HOME_ID);
							
	    encodedbuff[0] = (uint8_t)(gTelAzimuth /*to_host_data.Track_azimuth*/);
        encodedbuff[1] = (uint8_t)(gTelAzimuth /*to_host_data.Track_azimuth*/>>8);
        encodedbuff[2] = (uint8_t) gTelElevation /*to_host_data.Track_elevation*/;
        encodedbuff[3] = (uint8_t)((int32_t)(to_host_data.GPS_lat*100000.0) & 0xff);
        encodedbuff[4] = (uint8_t)((int32_t)(to_host_data.GPS_lat*100000.0)>>8 & 0xff);
        encodedbuff[5] = (uint8_t)((int32_t)(to_host_data.GPS_lat*100000.0)>>16 & 0xff);
        encodedbuff[6] = (uint8_t)((int32_t)(to_host_data.GPS_lat*100000.0)>>24 & 0xff);
        encodedbuff[7] = (uint8_t)((int32_t)(to_host_data.GPS_lon*100000.0) & 0xff);
        encodedbuff[8] = (uint8_t)((int32_t)(to_host_data.GPS_lon*100000.0)>>8 & 0xff);
        encodedbuff[9] = (uint8_t)((int32_t)(to_host_data.GPS_lon*100000.0)>>16 & 0xff);
        encodedbuff[10] = (uint8_t)((int32_t)(to_host_data.GPS_lon*100000.0)>>24 & 0xff);
			
		uint8_t len = packPacket(type, bufftosend, encodedbuff, cnt);		
		uart_write_bytes(UART_NUM_1, (const char *) bufftosend, len);
}

/*
static void sendAzimuthManualPacketToSocket(bool mode)
{
        //mode = 0 - switch to automatic mode if no errors
        //mode = 1 - forced manual control
        uint8_t encodedbuff[10];
        uint8_t bufftosend[16];
        uint8_t cnt = 4;
        uint8_t type = 0x04;//(uint8_t)(GsProtoIDs.FROMHOST_MANUAL_AZIMUTH_MSG_ID);
        uint8_t _mode = 0;
        if(mode)
            _mode = 1;
							
        encodedbuff[1]=(char)to_host_data.Track_azimuth;
        encodedbuff[0]=(char)(to_host_data.Track_azimuth>>8);
        encodedbuff[2]=(char)to_host_data.Track_elevation;
        encodedbuff[3]=_mode;
			
		uint8_t len = packPacket(type, bufftosend, encodedbuff, cnt);		
		uart_write_bytes(UART_NUM_1, (const char *) bufftosend, len);
}
*/

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


static void decode_packet_and_send_to_gs(const char * rx_buffer, int len)
{
	uint8_t mlen = 0;
	static _sb_proto deck_pack;
	for(int i = 0; i < len; i++)
	{
		mlen = parseChar(&deck_pack, rx_buffer[i]);
		if(mlen)
		{
			if(deck_pack.msg[0] == 0x04 && deck_pack.msg[4] == 0x01) //FROMHOST_MANUAL_AZIMUTH_MSG_ID and force manual mode 
			{
				tracker_m = MANUAL;//manual_tracker_mode = true;
				to_host_data.Track_azimuth = deck_pack.msg[2] + ((uint16_t)deck_pack.msg[1] << 8);
				to_host_data.Track_elevation = deck_pack.msg[3];
			}
			else if(deck_pack.msg[0] == 0x04 && deck_pack.msg[4] == 0x00)
			{
				//manual_tracker_mode = false;
				if(getProtocol() == TP_MSV)
					tracker_m = TRACKING_V;
				else if(getProtocol())
					tracker_m = TRACKING_T;
			}
			
			else if(deck_pack.msg[0] == 0x00)
			{
				if(deck_pack.msg[9] == 1)
				{
					 tracker_m = SETUP;
				}
				else
				{
					if(getProtocol() == TP_MSV)
						tracker_m = TRACKING_V;
					else if(getProtocol())
						tracker_m = TRACKING_T;		 
				}
			}
			
			
			if(tracker_m == MANUAL /*manual_tracker_mode*/)
			{
				if(deck_pack.msg[0] == 0x02)
				{
					to_host_data.Track_azimuth = deck_pack.msg[1] + ((uint16_t)deck_pack.msg[2] << 8);
					to_host_data.Track_elevation = deck_pack.msg[3];					
				}
			}
			

		}
	}
	
	uart_write_bytes(UART_NUM_1, (const char *)rx_buffer, len);
}

static bool decode_packet_for_host(uint8_t * rx_buffer, int len)
{
	uint8_t mlen = 0;
	static _sb_proto deck_pack;
	for(int i = 0; i < len; i++)
	{
		mlen = parseChar(&deck_pack, rx_buffer[i]);
		if(mlen)
		{

			if(deck_pack.msg[0] == 0x00)
			{		
					to_host_data.Track_azimuth = ((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
					to_host_data.Track_elevation = ((deck_pack.msg[4]&0xff)<<8)+(deck_pack.msg[3]&0xff);		
				if(getProtocol() == TP_MSV)
				{
					to_host_data.Track_azimuth = ((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
					to_host_data.Track_elevation = ((deck_pack.msg[4]&0xff)<<8)+(deck_pack.msg[3]&0xff);
					to_host_data.ID = deck_pack.msg[47];
				}
				else
				{
					//gTelAzimuth = ((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
					//gTelElevation = ((deck_pack.msg[4]&0xff)<<8)+(deck_pack.msg[3]&0xff);
				}
				
				to_host_data.GS_Version = deck_pack.msg[44];
				
				if(deck_pack.msg[41] < 60 && !getProtocol()) //Video telemetry error count
					setProtocol(TP_MSV);
				return true;
			}
			
			else if(deck_pack.msg[0] == 0x02)
			{
				az_elev_data.Track_azimuth =((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
				az_elev_data.Track_elevation = (char)(deck_pack.msg[3]&0xff);
				gTelAzimuth/*to_host_data.Track_azimuth*/ = az_elev_data.Track_azimuth;
				gTelElevation/*to_host_data.Track_elevation*/ = az_elev_data.Track_elevation;
			}
			
		/*
			if(manual_tracker_mode)
			{
				if(deck_pack.msg[0] == 0x02)
				{
					to_host_data.Track_azimuth = deck_pack.msg[1] + ((uint16_t)deck_pack.msg[2] << 8);
					to_host_data.Track_elevation = deck_pack.msg[3];					
				}
			}
		*/
		}
	}
	return false;
}

static void uart2_event_task(void *pvParameters)
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
					firstU2byte = true;
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


static void tracker_task(void *pvParameters)
{
	while(1)
	{
		if(telem_data.GPS_mode > 2 && (tracker_m!=MANUAL /*!manual_tracker_mode*/))
		{
			calc_los();
			gTelAzimuth = CalcTrackAzimut();
			gTelElevation = CalcTrackElevation();
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
				 
		if(xHandleTCP == NULL)
			xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
	
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}




void init_oled_i2c(void) {

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);
    
	//Set the OLED type
	//u8g2_Setup_ssd1306_i2c_128x32_univision_f(
	u8g2_Setup_sh1106_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	
	u8g2_SetFlipMode(&u8g2, 1); // send init sequence to the display, display is in sleep mode after this,

	//ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	//ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
	
	//ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_DrawBox(&u8g2, 14, 55, 0, 6);
	u8g2_DrawFrame(&u8g2, 14,55,100,6);
	
	//ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	//ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 1,17,"EasyToTrack");
	
	//ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
	//ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 3,37,"Ground Station");
	
	//ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	//ESP_LOGI(TAG, "All done!");
	for (int i = 1; i < 5; i++)
	{
		vTaskDelay( pdMS_TO_TICKS(600) );
		u8g2_DrawBox(&u8g2, 14, 55, i*25, 6);
		u8g2_SendBuffer(&u8g2);
	}
}

static inline char hex_digit(uint8_t v)
{
    return v < 10 ? '0' + v : 'A' + v - 10;
}

static inline char byte_signature(uint8_t v)
{
    return hex_digit((v & 0xf) ^ (v >> 4));
}

#define BT_MAC_LEN 6

static void bt_set_device_name(void)
{
    char dev_name[BT_DEV_NAME_PREFIX_LEN + BT_MAC_LEN + 1] = BT_DEV_NAME_PREFIX;
    const uint8_t * mac = esp_bt_dev_get_address();
    int i;
    for (i = 0; i < BT_MAC_LEN; ++i) {
        dev_name[BT_DEV_NAME_PREFIX_LEN + i] = byte_signature(mac[i]);
    }
    dev_name[BT_DEV_NAME_PREFIX_LEN + BT_MAC_LEN] = 0;
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(dev_name));
    ESP_LOGI(SPP_TAG, "Device name is %s", dev_name);
}

/*
static void uart1_task(void *arg)
{
	uint8_t data[32];
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data to BT
		
		if(len && (handle > 0)){
			esp_spp_write(handle, len, data);
			//ESP_LOGI(SPP_TAG, "3handle is %d", handle);
			len = 0;
		} 
		else if (len && (sock != -1))
		{
		    int err = send(sock, data, len, 0);
			len = 0;
            if (err < 0) {
				ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
			}					
		}
		
    }
}
*/

static void uart1_event_task(void *pvParameters)
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
						
					if(handle > 0)
					{					
						if(!unpack_res)
						{
							res = esp_spp_write(handle, event.size, dtmp);
							if(res != ESP_OK) {
								ESP_LOGE(TAG, "Error sending SPP: errno %d", res);
								break;
							}
						}
						else if(getProtocol() == TP_MSV)
						{
							res = esp_spp_write(handle, event.size, dtmp);
							if(res != ESP_OK) {
								ESP_LOGE(TAG, "Error sending SPP: errno %d", res);
								break;
							}
						}
					} 
					else if (sock != -1)
					{
						if(!unpack_res)
						{
							res = send(sock, dtmp, event.size, 0);
							if (res < 0) {
								ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
								break;
							}
						}
						else if(getProtocol() == TP_MSV)
						{
							res = send(sock, dtmp, event.size, 0);
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


static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        //esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
		bt_set_device_name();
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
		esp_spp_disconnect(param->srv_open.handle);
		handle = -1;
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT, handle=%d, %d",param->srv_open.handle, handle);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
		
		
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
		// Write data to the UART
		decode_packet_and_send_to_gs((const char *) param->data_ind.data, param->data_ind.len);
        //uart_write_bytes(UART_NUM_1, (const char *) param->data_ind.data, param->data_ind.len);
        break;
		
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        //ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT, handle=%d", param->write.handle);
		handle = param->write.handle;
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        gettimeofday(&time_old, NULL);
		esp_spp_write(param->srv_open.handle, 14, (uint8_t*)"ATZ\rATE0\rATL0\r");
		//spp_wr_task_start_up(spp_read_handle, param->srv_open.fd);
        break;
    default:
        break;
    }
}
/*
static void esp_spp_stack_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    spp_task_work_dispatch(esp_spp_cb, event, param, sizeof(esp_spp_cb_param_t), NULL);
}
*/

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}


/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
void monitoring_task(void *pvParameter)
{
	UBaseType_t sizes[7]; 
	uint16_t cnt = 0;
	
	for(;;){
		if(cnt++%10==0)
		{
			sizes[0] = uxTaskGetStackHighWaterMark(xHandleTCP);
			sizes[1] = uxTaskGetStackHighWaterMark(xHandleMonitor);			
			sizes[2] = uxTaskGetStackHighWaterMark(xHandleOled);		
			sizes[3] = uxTaskGetStackHighWaterMark(xHandleUart1);
			sizes[4] = uxTaskGetStackHighWaterMark(xHandleUart2);
			sizes[5] = uxTaskGetStackHighWaterMark(xHandleSendToHost);
			sizes[6] = uxTaskGetStackHighWaterMark(xHandleTracker);
			
			
			ESP_LOGI(TAG, "freeheap: %d, %d, %d, %d, %d, %d, %d, %d",esp_get_free_heap_size(), sizes[0], sizes[1], sizes[2], sizes[3], sizes[4], sizes[5], sizes[6]);
		}
		vTaskDelay( pdMS_TO_TICKS(1000));
		upTimeS++;
		if(++upTimeSs > 59) {upTimeSs=0; upTimeM++;}
	}
}


void oled_task(void *pvParameter)
{
	char buf[20];
	char * p_buf;
	p_buf = buf;
	
	for(;;){
		
		u8g2_ClearBuffer(&u8g2);
		
		u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
		
		
		snprintf (buf, 12, "%02d:%02d", upTimeM, upTimeSs);
		u8g2_DrawStr(&u8g2, 0, 10, buf);

		snprintf (buf, 12, "El:%d", to_host_data.Track_elevation);
		u8g2_DrawStr(&u8g2, 50, 10, buf);

		snprintf (buf, 12, "Az:%d", to_host_data.Track_azimuth);
		u8g2_DrawStr(&u8g2, 90, 10, buf);

		if(firstU2byte && !getProtocol())
		{
			u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
			snprintf (buf, 19, "Detecting protocol");
			u8g2_DrawStr(&u8g2, 5, 30, buf);
		}
		else
		{
			if(tracker_m == TRACKING_V && getProtocol()!=TP_MSV)
			{
				tracker_m = TRACKING_T;
			}
			u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
			switch(tracker_m)
			{
				case TRACKING_V:
					snprintf (buf, 16, "Video->TRACKING");
					u8g2_DrawStr(&u8g2, 15, 30, buf);
				break;
				case TRACKING_T:
					snprintf (buf, 16, "Telem->TRACKING");
					u8g2_DrawStr(&u8g2, 15, 30, buf);				
				break;
				case MANUAL:
					snprintf (buf, 7, "MANUAL");
					u8g2_DrawStr(&u8g2, 42, 30, buf);
				break;
				case SETUP:
					snprintf (buf, 6, "SETUP");
					u8g2_DrawStr(&u8g2, 47, 30, buf);				
				break;
			}			
		}
		

		/*
		u8g2_DrawBox(&u8g2, 0, 26, tmp,6);
		u8g2_DrawFrame(&u8g2, 0,26,100,6);
		*/
		u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
		
		if(handle > 0)
		{
			u8g2_DrawStr(&u8g2, 0, 54, "BT:[V]");
		}
		else
		{
			u8g2_DrawStr(&u8g2, 0, 54, "BT:[X]");
		}
			
		switch(wifi_m)
		{
			case NO:
				u8g2_DrawStr(&u8g2, 0, 64, "No WiFi");
			break;
			case AP:
				snprintf (buf, 12, "%s", DEFAULT_AP_IP);
				u8g2_DrawStr(&u8g2, 0, 64, "AP:");
				u8g2_DrawStr(&u8g2, 20, 64, buf);
			break;
			case STA:
				p_buf = wifi_manager_get_sta_ip_string();
				u8g2_DrawStr(&u8g2, 0, 64, "STA:");
				u8g2_DrawStr(&u8g2, 20, 64, p_buf);
			break;
		}
		
		switch(getProtocol())
		{
			case TP_MFD:
				u8g2_DrawStr(&u8g2, 100, 64, "MFD");
			break;
			case TP_GPS_TELEMETRY:
				u8g2_DrawStr(&u8g2, 100, 64, "GPS");
			break;
			case TP_MAVLINK:
				u8g2_DrawStr(&u8g2, 100, 64, "MAV");
			break;
			case TP_RVOSD:
				u8g2_DrawStr(&u8g2, 100, 64, "RVOSD");
			break;
			case TP_FRSKY_D:
				u8g2_DrawStr(&u8g2, 100, 64, "FrD");
			break;
			case TP_FRSKY_X:
				u8g2_DrawStr(&u8g2, 100, 64, "FrX");
			break;
			case TP_LTM:
				u8g2_DrawStr(&u8g2, 100, 64, "LTM");
			break;
			case TP_PITLAB:
				u8g2_DrawStr(&u8g2, 100, 64, "PITLAB");
			break;
			case TP_MSP:
				u8g2_DrawStr(&u8g2, 100, 64, "MSP");
			break;
			case TP_MSV:
				u8g2_DrawStr(&u8g2, 100, 64, "MSV");
			break;
			default:
				u8g2_DrawStr(&u8g2, 100, 64, "NA");
			break;			
		}
		
		u8g2_SendBuffer(&u8g2);

		vTaskDelay( pdMS_TO_TICKS(400) );
	}
}


static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");
		
  //int flag = 1;
  //setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));		

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
				//ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);

				//uart_write_bytes(UART_NUM_1, (const char *) rx_buffer, len);
				decode_packet_and_send_to_gs((const char *) rx_buffer, len);
			}

        }

        if (sock != -1) {

            ESP_LOGE(TAG, "Closing socket and restarting...");

			close(listen_sock);
			shutdown(listen_sock,0);

            close(sock);
            shutdown(sock, 0);
			
			sock = -1;

			vTaskDelay(50);
        }
    }
    vTaskDelete(NULL);
}



/* brief this is an exemple of a callback that you can setup in your own app to get notified of wifi manager event */
void cb_connection_ok(void *pvParameter){
	ESP_LOGI(TAG, "I have a connection!");
	wifi_m = STA;
	if(xHandleTCP == NULL)
		xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
}

void cb_ap_ok(void *pvParameter){
	ESP_LOGI(TAG, "I'm in AP mode");
	wifi_m = AP;
	if(xHandleTCP == NULL)
		xTaskCreate(tcp_server_task, "tcp_server", 1024*2, NULL, 10, &xHandleTCP);
}

void cb_sta_disconnected(void *pvParameter){
	wifi_m = NONE;
}

//Send to Android Host task
static void send_to_host_task(void *pvParameters)
{
	while(1)
	{		
		if(getProtocol()!=TP_MSV)
		{
			uint8_t len = packPacket(0, txbuf, (uint8_t *)&to_host_data, sizeof(TO_HOST_DATA));

			if(handle > 0)
			{
				esp_spp_write(handle, len, txbuf);
				//ESP_LOGI(SPP_TAG, "written");
			} 
			else if (sock != -1)
			{
				int res = send(sock, txbuf, len, 0);
				if (res < 0) {
					ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
					//break;
				}		
			}
					
			sendHostHomeMessageToGS();
		}
		vTaskDelay(20);
	}
}


void app_main()
{
	esp_err_t ret;
	
    ret = nvs_flash_init();
	
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
        .baud_rate = 57600,
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
		ESP_LOGI(SPP_TAG, "baud is %d", baud);
		vTaskDelay(500);
	}
	ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_2));
	uart2_config.baud_rate = baud;
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart2_config));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BT_UART_RX_BUF_SZ*2, BT_UART_RX_BUF_SZ*2, 20, &uart2_queue, 0));
#endif	

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
	
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
	/*
	if ((ret = esp_spp_register_callback(esp_spp_stack_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
	*/
    //spp_task_task_start_up();

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    // Set default parameters for Secure Simple Pairing
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif


     // Set default parameters for Legacy Pairing
     // Use variable pin, input pin code when pairing
    
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
	ESP_LOGE(SPP_TAG, "%s spp init OK\n", __func__);
	
#if 0
	wifi_init_softap();
#else
	/* start the wifi manager */
	wifi_manager_start();

	/* register a callback as an example to how you can integrate your code with the wifi manager */
	wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_ok);
	wifi_manager_set_callback(ORDER_START_AP, &cb_ap_ok);
	wifi_manager_set_callback(EVENT_STA_DISCONNECTED, &cb_sta_disconnected);
#endif


	/* your code should go here. Here we simply create a task on core 2 that monitors free heap memory */
	xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 1024*2, NULL, 1, &xHandleMonitor, 1);
	
	init_oled_i2c();
	xTaskCreatePinnedToCore(&oled_task, "oled_task", 2048, NULL, 1, &xHandleOled, 1);
	
	initSPI();
	initSPIFS();
	runProgrammer();
	
	//Create a task to handler UART event from ISR
    xTaskCreate(uart1_event_task, "uart1_task", 2048, NULL, 12, &xHandleUart1);
	
	//Create a task to handler UART event from ISR
    xTaskCreate(uart2_event_task, "uart2_task", 2048, NULL, 12, &xHandleUart2);
	
	xTaskCreate(send_to_host_task, "send_to_host_task", 1024, NULL, 6, &xHandleSendToHost);
	xTaskCreate(tracker_task, "tracker_task", 1024, NULL, 5, &xHandleTracker);

}
