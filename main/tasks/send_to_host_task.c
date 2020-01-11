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
					
			sendHostHomeMessageToGS();
		}
		vTaskDelay(20);
	}
}