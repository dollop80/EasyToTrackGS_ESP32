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
#include "../define.h"
#include "../u8g2_esp32_hal.h"
#include "components/u8g2/csrc/u8g2.h"
#include "../protocol_detection.h"
#include "../tracker.h"
#include "oled_task.h"
#include "components/avr-isp/avr_isp_helper.h"

extern TaskHandle_t xHandleOled;
extern TO_HOST_DATA to_host_data;
extern uint32_t gUpTimeS;
extern uint16_t gUpTimeM;
extern uint8_t gUpTimeSs;
extern int16_t gBThandle;
extern int16_t gWFsock;
extern bool gFirstU2byte;
extern wifi_mode gWifi_m;
extern tracker_mode gTracker_m;
extern uint8_t gVideoStandard;
extern uint8_t gVideoThreshold;

extern uint8_t gTelemPhase;
uint8_t TelemPhase = 0;
uint8_t TelemPhasePrev;
int item = 9712;

extern uint16_t gTelAzimuth;
extern uint8_t gTelElevation;

u8g2_t u8g2; // a structure which will contain all the data for one display
uint8_t progRes = 0;


static const char TAG[] = "ETT-OLED";

void oled_task(void *pvParameter)
{
	uint8_t ii = 0;
	char buf[20];
	char t=' ';
	char * p_buf;
	p_buf = buf;
	
	for(;;){
		
		if(ii < 7)
		{
			ii++;
			u8g2_DrawBox(&u8g2, 14, 55, ii*14, 6);
			u8g2_SendBuffer(&u8g2);
		}
		else
		{
			u8g2_ClearBuffer(&u8g2);
			
			u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
			
			
			snprintf (buf, 12, "%02d:%02d", gUpTimeM, gUpTimeSs);
			u8g2_DrawStr(&u8g2, 0, 10, buf);

			snprintf (buf, 12, "El:%d", to_host_data.Track_elevation);
			u8g2_DrawStr(&u8g2, 50, 10, buf);

			snprintf (buf, 12, "Az:%d", to_host_data.Track_azimuth);
			u8g2_DrawStr(&u8g2, 90, 10, buf);
			
			//Draw Lat, Lon
			if(to_host_data.GPS_lat > 0)
				t='N';
			else
				t='S';
			snprintf (buf, 15, "%.6f%c", ((abs(to_host_data.GPS_lat))/100000.0f), t);
			u8g2_DrawStr(&u8g2, 0, 20, buf);
			if(to_host_data.GPS_lon > 0)
				t='W';
			else
				t='E';
			snprintf (buf, 15, "%.6f%c", ((abs(to_host_data.GPS_lon))/100000.0f), t);
			u8g2_DrawStr(&u8g2, 64, 20, buf);
			
			

			snprintf (buf, 12, "Verr:%d", to_host_data.AVErrors);
			u8g2_DrawStr(&u8g2, 40, 54, buf);

			snprintf (buf, 12, "RSSI:%d", to_host_data.AV_RSSI);
			u8g2_DrawStr(&u8g2, 88, 54, buf);

			// Draw working mode
			u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
			if(gFirstU2byte && !getProtocol())
			{
				snprintf (buf, 19, "Detecting protocol");
				u8g2_DrawStr(&u8g2, 5, 32, buf);
			}
			else
			{
				if(gTracker_m == TRACKING_V && getProtocol()!=TP_MSV)
				{
					gTracker_m = TRACKING_T;
				}
				switch(gTracker_m)
				{
					case TRACKING_V:
						snprintf (buf, 16, "Video->TRACKING");
						u8g2_DrawStr(&u8g2, 15, 32, buf);
					break;
					case TRACKING_T:
						snprintf (buf, 16, "Telem->TRACKING");
						u8g2_DrawStr(&u8g2, 15, 32, buf);				
					break;
					case MANUAL:
						snprintf (buf, 7, "MANUAL");
						u8g2_DrawStr(&u8g2, 42, 32, buf);
					break;
					case SETUP:
						snprintf (buf, 6, "SETUP");
						u8g2_DrawStr(&u8g2, 47, 32, buf);				
					break;
				}
				u8g2_SetFont(&u8g2, u8g2_font_unifont_t_75);
				
				//Draw telemetry status
				if(TelemPhasePrev != gTelemPhase) 
				{
					TelemPhasePrev = gTelemPhase;
					switch(++TelemPhase)
					{
						case 1:
							item = 9712;
						break;
						case 2:
							item = 9713;
						break;
						case 3:
							item = 9714;
						break;
						case 4:
							item = 9715;
							TelemPhase = 0;
						break;
					}
				}
				
				u8g2_DrawGlyph(&u8g2, 0, 44, item);					
			}
			
			//Draw voltage
			u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
			snprintf (buf, 16, "%.1fV", getVoltage()/(4700.0/(4700.0+47000.0))/1000.0);
			u8g2_DrawStr(&u8g2, 90, 44, buf);

			/*
			u8g2_DrawBox(&u8g2, 0, 26, tmp,6);
			u8g2_DrawFrame(&u8g2, 0,26,100,6);
			*/
			u8g2_SetFont(&u8g2, u8g2_font_5x7_mf);
			
			if(gBThandle > 0)
			{
				u8g2_DrawStr(&u8g2, 0, 54, "BT:[V]");
			}
			else
			{
				u8g2_DrawStr(&u8g2, 0, 54, "BT:[X]");
			}
				
			switch(gWifi_m)
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
			
			//Draw protocol type
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
		}
		u8g2_SendBuffer(&u8g2);

		vTaskDelay( pdMS_TO_TICKS(400) );
	}
}



void initOled(void) {

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);
    
	//Set the OLED type
	//u8g2_Setup_ssd1306_i2c_128x64_noname_f(
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
	//u8g2_DrawBox(&u8g2, 14, 55, 0, 6);
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
}

void oledShowProgramming(void *pvParameter)
{
	char buf[20];
	
	u8g2_ClearBuffer(&u8g2);
		
	//ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
	//ESP_LOGI(TAG, "u8g2_DrawStr");
	
	while(true)
	{
		u8g2_ClearBuffer(&u8g2);
		
		if(progRes == 3){
			u8g2_DrawStr(&u8g2, 1,11,"Programming");
			u8g2_DrawStr(&u8g2, 10,35,"ATMega8...");
			
			snprintf (buf, 12, "Line# %d", getLineNumber());
			u8g2_DrawStr(&u8g2, 20, 60, buf);
			
		} else if(progRes == 2){
			u8g2_DrawStr(&u8g2, 1,11,"Programming");
			u8g2_DrawStr(&u8g2, 10,35,"not needed");		
		} else if(progRes == 1) {
			u8g2_DrawStr(&u8g2, 1,11,"Programming");
			u8g2_DrawStr(&u8g2, 25,35,"DONE!");
		} else if(progRes == 0) {
			u8g2_DrawStr(&u8g2, 1,11,"Programming");
			u8g2_DrawStr(&u8g2, 15,35,"FAILED :(");
		}
				
		//ESP_LOGI(TAG, "u8g2_SendBuffer");
		u8g2_SendBuffer(&u8g2);
		vTaskDelay( pdMS_TO_TICKS(300) );
	}
}

void setProgrammingRes(uint8_t res)
{
	progRes = res;
}

