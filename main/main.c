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
@author Constantine Safronov
@brief Entry point for the ESP32 application.
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

#include <sys/unistd.h>
#include <sys/stat.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "sdkconfig.h"
#include "define.h"

#include "components/wifi-manager/src/wifi_manager.h"
#ifdef DIGITAL_THRH_POT
	#include "components/tpl0401x/tpl0401x.h"
#endif

#include "BThelper.h"
#include "m8_programmer.h"
#include "tracker.h"
#include "protocol_detection.h"
#include "tasks/monitoring_task.h"
#include "tasks/tcp_server_task.h"
#include "tasks/oled_task.h"
#include "tasks/uart_task.h"
#include "tasks/send_to_host_task.h"


TELEM_DATA telem_data;
TO_HOST_DATA to_host_data;
FROM_HOST_DATA servo_settings_data;
AZ_ELEV_DATA az_elev_data;

TaskHandle_t xHandleTCP = NULL;
TaskHandle_t xHandleMonitor = NULL;
TaskHandle_t xHandleOled = NULL;
TaskHandle_t xHandleUart1 = NULL;
TaskHandle_t xHandleUart2 = NULL;
TaskHandle_t xHandleSendToHost = NULL;
TaskHandle_t xHandleTracker = NULL;

int16_t gBThandle;
int16_t gWFsock = -1;
uint32_t gUpTimeS = 0;
uint16_t gUpTimeM = 0;
uint8_t gUpTimeSs = 0;
bool gFirstU2byte = false;
uint16_t gTelAzimuth;
uint8_t gTelElevation;
uint8_t gVideoStandard;
uint8_t gVideoThreshold;
uint8_t gExtTelemType;
uint8_t gExtTelemBaud;

wifi_mode gWifi_m = NO;
tracker_mode gTracker_m = TRACKING_V;


void app_main()
{
	esp_err_t ret;
	
	ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
	{
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();	
    }
    ESP_ERROR_CHECK( ret );
	
	
	initBT();
    initWiFi();
	initOled();
	initSPI();
	initSPIFS();
	initADC();
	initVidStdPin();
	initProgModePin();
	if(getProgModePin())
	{
		oledShowProgramming(3);
		uint8_t p_res = runProgrammer();
		oledShowProgramming(p_res);
		while(true);
	}
	
	if(!tracker_fetch_video_config())
	{
		gVideoStandard = 0;
		gVideoThreshold = 64;
		tracker_save_video_config();
	}
	setVidStdPin(gVideoStandard);
	#ifdef DIGITAL_THRH_POT
		i2c_tpl0401_set(gVideoThreshold);
	#endif
	
	if(!tracker_fetch_ext_telemetry_config())
	{
		gExtTelemType = 0; //Auto
		gExtTelemBaud = 4; //57600
		tracker_save_ext_telemetry_config();
	}
	if(gExtTelemType != 0)
		setProtocol(1 << (gExtTelemType+1));
	
	initUart(gExtTelemBaud);
		
	xTaskCreatePinnedToCore(&oled_task, "oled_task", 2048, NULL, 1, &xHandleOled, 1);
	
	xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 1024*2, NULL, 1, &xHandleMonitor, 1);
	
	//Create a task to handler UART event from ISR
    xTaskCreate(uart1_event_task, "uart1_task", 2048, NULL, 12, &xHandleUart1);
	
	//Create a task to handler UART event from ISR
    xTaskCreate(uart2_event_task, "uart2_task", 2048, NULL, 12, &xHandleUart2);
	
	xTaskCreate(send_to_host_task, "send_to_host_task", 1024, NULL, 6, &xHandleSendToHost);
	
	xTaskCreate(tracker_task, "tracker_task", 1024, NULL, 5, &xHandleTracker);
}
