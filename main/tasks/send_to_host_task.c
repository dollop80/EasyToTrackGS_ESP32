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

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "../protocol_detection.h"
#include "../components/protocols/simple_bin.h"
#include "../tracker.h"
#include "define.h"
#include "send_to_host_task.h"

extern TO_HOST_DATA to_host_data;
extern AZ_ELEV_DATA az_elev_data;
#ifdef ESP32_ONLY
	extern CALIBR_DATA gCallibr;
	extern FROM_HOST_DATA from_host_data;
	extern bool g_servo_values_req;
	extern uint8_t EEP_mode360;
	extern uint8_t EEP_soundOn;
	extern uint8_t EEP_delay_change_ppm;
	extern uint16_t EEP_Off_azimuth;
#endif
extern int16_t gBThandle;
extern int16_t gWFsock;

static const char TAG[] = "ETT-TOHOST";

static uint8_t txbuf[128];

//Send to Android Host task
void send_to_host_task(void *pvParameters)
{
	while(1)
	{		
		if(getProtocol()!=TP_MSV)
		{
			uint8_t len = packPacket(0, txbuf, (uint8_t *)&to_host_data, sizeof(TO_HOST_DATA));

			if(gBThandle > 0)
			{
				esp_spp_write(gBThandle, len, txbuf);
				//ESP_LOGI(TAG, "written");
			} 
			else if (gWFsock != -1)
			{
				int res = send(gWFsock, txbuf, len, 0);
				if (res < 0) {
					ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
					//break;
				}		
			}
			
			
			#ifdef ESP32_ONLY
			uart_write_bytes(UART_NUM_0, (const char *) txbuf, len);
			
			if(g_servo_values_req){
				g_servo_values_req = false;
				
				from_host_data.OutPPM_Min[0] = gCallibr.OutPPM_Min[0] * 2.5; 
				from_host_data.OutPPM_Min[1] = gCallibr.OutPPM_Min[1] * 2.5; 
				from_host_data.OutPPM_Max[0] = gCallibr.OutPPM_Max[0] * 2.5;
				from_host_data.OutPPM_Max[1] = gCallibr.OutPPM_Max[1] * 2.5;
				from_host_data.mode_360 = EEP_mode360;
				from_host_data.AzimuthOffset = EEP_Off_azimuth;
				from_host_data.soundOn = EEP_soundOn; 
				from_host_data.delay_change_ppm = EEP_delay_change_ppm;
				
				len = packPacket(1, txbuf, (uint8_t *)&from_host_data, sizeof(FROM_HOST_DATA));

				if(gBThandle > 0)
				{
					esp_spp_write(gBThandle, len, txbuf);
					//ESP_LOGI(TAG, "written");
				} 
				else if (gWFsock != -1)
				{
					int res = send(gWFsock, txbuf, len, 0);
					if (res < 0) {
						ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
					}		
				}
				
				uart_write_bytes(UART_NUM_0, (const char *) txbuf, len);
			}

			#endif
			
			sendHostHomeMessageToGS();
		}
		vTaskDelay(20);
	}
}