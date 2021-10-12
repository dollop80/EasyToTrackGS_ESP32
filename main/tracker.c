
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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "math.h"
#include "define.h"
#include "tracker.h"
#include "components/protocols/simple_bin.h"
#include "protocol_detection.h"
#ifdef DIGITAL_THRH_POT
	#include "components/tpl0401x/tpl0401x.h"
#endif

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


void CalcGpsVec(GPS_PNT *src, GPS_PNT *dst, GPS_VEC *v);
int16_t cos_b(int16_t angle);
int16_t AddAngleU(int16_t a, int16_t b);

extern TELEM_DATA telem_data;
extern TO_HOST_DATA to_host_data;
extern AZ_ELEV_DATA az_elev_data;

extern uint16_t gTelAzimuth;
extern uint8_t gTelElevation;
extern tracker_mode gTracker_m;
extern uint8_t gVideoStandard;
extern uint8_t gVideoThreshold;
extern uint8_t gExtTelemType;
extern uint8_t gExtTelemBaud;

extern int16_t gBThandle;
extern int16_t gWFsock;

static uint8_t gGPS_starting = 0;
static uint8_t gGPS_ready = 0;
static uint8_t gGPS_update = 0;
static uint8_t gGPS_tmp = 0;
static uint8_t gGPS_Timeout = 0;
static uint32_t gGPS_lon_gm = 0;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC1_CHANNEL_6;    //GPIO34 //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
uint32_t adc_reading = 0;
uint32_t voltage = 0;
bool forced_mnl_cntrl = false;

#if ESP32_ONLY == 1
	RSSI_DATA gRssiData;
	FROM_HOST_DATA from_host_data;
	bool g_servo_values_req = false;
	bool home_from_host = false;
	int16_t gTrack_elevation = 0;
	int16_t gTrack_azimuth = 0;
	bool settings_changed = false;
	bool g_gefault_az_elev_change = false;
	extern uint8_t EEP_soundOn;	
	extern uint16_t EEP_def_azimuth;
	extern uint8_t EEP_def_elevation;
	extern uint8_t EEP_delay_change_ppm;
	extern uint16_t EEP_Off_azimuth;
#endif

GPS_PNT gPntStart, gPntCurrent;
GPS_VEC gVecToStart;

static const char TAG[] = "ETT-TRCK";
const char tracker_nvs_namespace[] = "ettsettings";

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

void calc_los(void)
{
	gPntCurrent.Lon = to_host_data.GPS_lon;
	gPntCurrent.Lat = to_host_data.GPS_lat;
	gPntCurrent.Alt = telem_data.Alt;
	
    to_host_data.Altitude=gPntCurrent.Alt-gPntStart.Alt;

    if(gGPS_update==0)
      {
      if(gGPS_Timeout<80) gGPS_Timeout++;
      else gGPS_ready=0;
      return;
      }

	CalcGpsVec(&gPntCurrent, &gPntStart, &gVecToStart);
	gGPS_update=0;
}

void process_gps(void)
{
	uint32_t sl;
	telem_data.GPS_frame++;
	if(telem_data.GPS_frame>=3) { telem_data.GPS_frame=0; gGPS_Timeout=0;}

	if(!gGPS_starting && telem_data.GPS_mode==3)
	  {
	  if(++gGPS_tmp>=10)
		{ //2 sec
		gGPS_starting=1;
		gGPS_ready=1;
		gPntStart=gPntCurrent;
		to_host_data.Home_lon = gPntStart.Lon;
		to_host_data.Home_lat = gPntStart.Lat;
		sl=GRAD_METER_CONST; sl*=cos_b(gPntCurrent.Lat/100000);
		gGPS_lon_gm=(sl>>8);
		}
	  }

	if(gGPS_starting) { gGPS_ready=1; gGPS_update=1; }
	if(telem_data.GPS_mode < 3) { gGPS_ready=0; gGPS_tmp=0;}
}

//---------------------------------------------------------------
int16_t CalcTrackAzimut(void)
{ // asimut 0-360
	int16_t angle;

	angle=AddAngleU(gVecToStart.Course, 180); // rotate 180deg
	return angle;
}


//---------------------------------------------------------------
int16_t CalcTrackElevation(void)
{ // elevation 0-90grad
	int16_t angle, altitude;
	uint32_t ln;

	altitude=gPntCurrent.Alt-gPntStart.Alt;

	if(altitude<0) altitude=0;
	if(gVecToStart.Distance>altitude) 
	  { ln=altitude; ln*=45; angle=(int16_t)(ln/gVecToStart.Distance); }
	else
	  {
	  if(altitude==0) angle=90;
	  else { ln=gVecToStart.Distance; ln*=45; angle=90-((int16_t)(ln/altitude)); }
	  }
	return angle;  
}


//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
int16_t AddAngleU(int16_t a, int16_t b)
{
	int16_t c;

	c=a+b;
	while(c<0) c+=360;
	while(c>=360) c-=360;
	return c; // c=0..359
}

//---------------------------------------------------------------------------
const uint8_t cos_tab[]= {
0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFD, 0xFD, 0xFC, 0xFB,
0xFB, 0xFA, 0xF9, 0xF8, 0xF7, 0xF6, 0xF5, 0xF3, 0xF2, 0xF1,
0xEF, 0xEE, 0xEC, 0xEA, 0xE8, 0xE7, 0xE5, 0xE3, 0xE1, 0xDF,
0xDC, 0xDA, 0xD8, 0xD5, 0xD3, 0xD0, 0xCE, 0xCB, 0xC8, 0xC6,
0xC3, 0xC0, 0xBD, 0xBA, 0xB7, 0xB4, 0xB1, 0xAD, 0xAA, 0xA7,
0xA3, 0xA0, 0x9C, 0x99, 0x95, 0x92, 0x8E, 0x8A, 0x87, 0x83,
0x7F, 0x7B, 0x77, 0x73, 0x6F, 0x6B, 0x67, 0x63, 0x5F, 0x5B,
0x57, 0x53, 0x4E, 0x4A, 0x46, 0x41, 0x3D, 0x39, 0x35, 0x30,
0x2C, 0x27, 0x23, 0x1F, 0x1A, 0x16, 0x11, 0x0D, 0x08, 0x04, 0x00 };
//---------------------------------------------------------------------------
int16_t cos_b(int16_t angle)
{
	int16_t a, cs;

	angle=AddAngleU(angle, 0); //normalize 0..359
	a=angle;
	if(a>180) a-=180;
	if(a>90)  a=180-a;
	cs=cos_tab[a];
	if(angle>90 && angle<270) cs=-cs;
	return cs;
}

void CalcGpsVec(GPS_PNT *src, GPS_PNT *dst, GPS_VEC *v)
{
	uint32_t dx, dy;
	uint16_t course;

	if(dst->Lat>=src->Lat) dy=dst->Lat - src->Lat;
	else dy=src->Lat - dst->Lat;
	if(dst->Lon>=src->Lon) dx=dst->Lon - src->Lon;
	else dx=src->Lon - dst->Lon;
	dy*=GRAD_METER_CONST; dy>>=10;
	dx*=gGPS_lon_gm; dx>>=10;
	v->Distance=sqrt((dx*dx)+(dy*dy));
	//course
	if(v->Distance)
	{
	  if(dx>=dy) course=90-(dy*45)/dx;
	  else course=(dx*45)/dy;
	  if(dst->Lat < src->Lat)
		{
		if(dst->Lon >= src->Lon) course=180-course;
		else course=180+course;
		}
	  else if(dst->Lon < src->Lon) course=360-course;
	  v->Course=course;
	}
}

void sendHostHomeMessageToGS()
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
#if ESP32_ONLY == 1	
		uart_write_bytes(UART_NUM_0, (const char *) bufftosend, len);
#endif
}


void sendExtTelemetrySetMessageToHOST()
{
	    uint8_t encodedbuff[11];
        uint8_t bufftosend[11+6];
		uint8_t cnt = 2;
        uint8_t type = 0x0C; //EXT_TELEM_SETTINGS_ID
							
	    encodedbuff[0] = gExtTelemType;
        encodedbuff[1] = gExtTelemBaud;
		
		uint8_t len = packPacket(type, bufftosend, encodedbuff, cnt);		
			if(gBThandle > 0)
			{
				esp_spp_write(gBThandle, len, bufftosend);
				//ESP_LOGI(TAG, "written");
			} 
			else if (gWFsock != -1)
			{
				int res = send(gWFsock, bufftosend, len, 0);
				if (res < 0) {
					//ESP_LOGE(TAG, "Error sending SOCK: errno %d", errno);
					//break;
				}		
			}
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

void decode_packet_and_send_to_gs(const char * rx_buffer, int len)
{
	uint8_t mlen = 0;
	static _sb_proto deck_pack;
	for(int i = 0; i < len; i++)
	{
		mlen = parseChar(&deck_pack, rx_buffer[i]);
		if(mlen)
		{
			if(deck_pack.msg[0] == FROMHOST_MANUAL_AZIMUTH_MSG_ID && deck_pack.msg[4] == 0x01) //FROMHOST_MANUAL_AZIMUTH_MSG_ID and force manual mode 
			{
				forced_mnl_cntrl = true;
				gTracker_m = MANUAL;//manual_tracker_mode = true;
				gTelAzimuth = deck_pack.msg[2] + ((uint16_t)deck_pack.msg[1] << 8);
				gTelElevation = deck_pack.msg[3];
			}
			else if(deck_pack.msg[0] == FROMHOST_MANUAL_AZIMUTH_MSG_ID && deck_pack.msg[4] == 0x00)
			{
				forced_mnl_cntrl = false;
				//manual_tracker_mode = false;
				if(getProtocol() == TP_MSV)
					gTracker_m = TRACKING_V;
				else if(getProtocol())
					gTracker_m = TRACKING_T;
			}
			
			else if(deck_pack.msg[0] == FROMHOST_MSG_ID_SETUP)
			{
				if(deck_pack.msg[9] > 0)
				{
					 gTracker_m = SETUP;
				}
				else
				{
					if(getProtocol() == TP_MSV)
						gTracker_m = TRACKING_V;
					else if(getProtocol())
						gTracker_m = TRACKING_T;		 
				}
				
#if ESP32_ONLY == 1
				  from_host_data.OutPPM_Min[0] = (((uint16_t)deck_pack.msg[1]<<8) + deck_pack.msg[2])/2.5; 
				  from_host_data.OutPPM_Min[1] = (((uint16_t)deck_pack.msg[3]<<8) + deck_pack.msg[4])/2.5;
				  from_host_data.OutPPM_Max[0] = (((uint16_t)deck_pack.msg[5]<<8) + deck_pack.msg[6])/2.5;
				  from_host_data.OutPPM_Max[1] = (((uint16_t)deck_pack.msg[7]<<8) + deck_pack.msg[8])/2.5; 
				  from_host_data.mode_360 = deck_pack.msg[10];
				  from_host_data.mode = deck_pack.msg[9]; 
#endif
			}
#if ESP32_ONLY == 1			
			else if(deck_pack.msg[0] == SERVO_REQ_MSG_ID)
			{				
				g_servo_values_req = true;
			}
			
			else if(deck_pack.msg[0] == FROMHOST_AZIMUTH_MSG_ID)
			{			
				if (deck_pack.msg[3] == 1)
					settings_changed = true;
				else
					EEP_Off_azimuth = ((uint16_t)deck_pack.msg[1]<<8) + deck_pack.msg[2]; 
			}
			
			else if(deck_pack.msg[0] == FROMHOST_SOUND_MSG_ID)
			{
				EEP_soundOn = deck_pack.msg[1]; 
				settings_changed = true;
			}		               

			else if(deck_pack.msg[0] == FROMHOST_PARAMS_ID_SETUP)
			{
				settings_changed = true;   
				EEP_delay_change_ppm = deck_pack.msg[1];
				if(deck_pack.msg[2]==1) 
					g_gefault_az_elev_change = true;
			}

			else if(deck_pack.msg[0] == HOST2GS_POWEROFF_ID)
			{
				/*gTrack_azimuth*/gTelAzimuth = EEP_def_azimuth;
				/*gTrack_elevation*/gTelElevation = EEP_def_elevation; 
			}
 
			else if(deck_pack.msg[0] == HOST2GS_HOST_HOME_ID)
			{
				home_from_host = true;
				HOST_HOME_DATA * HostHomeData = (HOST_HOME_DATA *)deck_pack.msg;
				/*gTrack_azimuth*/gTelAzimuth=HostHomeData->Track_azimuth;
				/*gTrack_elevation*/gTelElevation=HostHomeData->Track_elevation; 
				to_host_data.Home_lat=HostHomeData->Home_lat;
				to_host_data.Home_lon=HostHomeData->Home_lon;
			}    

			else if(deck_pack.msg[0] == HOST2GS_RSSI_ID)
			{
				 RSSI_DATA * RssiData = (RSSI_DATA *)deck_pack.msg;
				 gRssiData.mode = RssiData->mode;
				 if (gRssiData.mode>0)
				 {
					gTracker_m = SETUP;//programming_mode = 1;
				 } else {
					//programming_mode = 0;
					if(getProtocol() == TP_MSV)
						gTracker_m = TRACKING_V;
					else if(getProtocol())
						gTracker_m = TRACKING_T;
				 }
				 if(gRssiData.mode==2)
				 {
					gRssiData.max = RssiData->max;
					gRssiData.min = RssiData->min;
				 }
			}  
#endif			
			 
			else if(deck_pack.msg[0] == FROMHOST_VIDEOSETTINGS_ID) //VIDEO_SETTINGS_ID
			{
				gVideoStandard = deck_pack.msg[2];
				gVideoThreshold = deck_pack.msg[3];
				if(deck_pack.msg[1] == 1) //Save settings
				{
					 tracker_save_video_config();
				}
				setVidStdPin(gVideoStandard);
				#ifdef DIGITAL_THRH_POT
					i2c_tpl0401_set(gVideoThreshold);
				#endif
			}
			
			else if(deck_pack.msg[0] == FROMHOST_EXTTELEMSETTINGS_ID) //EXT_TELEM_SETTINGS_ID
			{
				if(deck_pack.msg[1] == 1) //Save settings
				{
					if(deck_pack.msg[2] > 0)
						setProtocol(1 << (deck_pack.msg[2]+1));
					else
						enableProtocolDetection();
					
					//if(deck_pack.msg[3] > 0) //Add code for baud detection
					//else
						
					gExtTelemType = deck_pack.msg[2];
					gExtTelemBaud = deck_pack.msg[3];
					
					tracker_save_ext_telemetry_config();
				}
				else
				{
					sendExtTelemetrySetMessageToHOST();
				}
			}
			
			
			if(gTracker_m == MANUAL /*manual_tracker_mode*/)
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
	//uart_write_bytes(UART_NUM_0, (const char *)rx_buffer, len);
}

bool decode_packet_for_host(uint8_t * rx_buffer, int len)
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
					to_host_data.GPS_lat = ((deck_pack.msg[8]&0xff)<<24)+((deck_pack.msg[7]&0xff)<<16)+((deck_pack.msg[6]&0xff)<<8)+(deck_pack.msg[5]&0xff);
					to_host_data.GPS_lon = ((deck_pack.msg[12]&0xff)<<24)+((deck_pack.msg[11]&0xff)<<16)+((deck_pack.msg[10]&0xff)<<8)+(deck_pack.msg[9]&0xff);
					to_host_data.ID = deck_pack.msg[47];
				}
				else
				{
					//gTelAzimuth = ((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
					//gTelElevation = ((deck_pack.msg[4]&0xff)<<8)+(deck_pack.msg[3]&0xff);
				}
				
				to_host_data.GS_Version = deck_pack.msg[44];
				
				if(deck_pack.msg[41] < 64 && !getProtocol()) //Video telemetry error count
					setProtocol(TP_MSV);
				return true;
			}
			
			else if(deck_pack.msg[0] == 0x02)
			{
				az_elev_data.Track_azimuth =((deck_pack.msg[2]&0xff)<<8)+(deck_pack.msg[1]&0xff);
				az_elev_data.Track_elevation = (char)(deck_pack.msg[3]&0xff);
				//gTelAzimuth/*to_host_data.Track_azimuth*/ = az_elev_data.Track_azimuth;
				//gTelElevation/*to_host_data.Track_elevation*/ = az_elev_data.Track_elevation;
			}
			
		}
	}
	return false;
}

void tracker_task(void *pvParameters)
{
	while(1)
	{
		if(telem_data.GPS_mode > 2 && (gTracker_m != MANUAL /*!manual_tracker_mode*/))
		{
			calc_los();
			gTelAzimuth = CalcTrackAzimut();
			gTelElevation = CalcTrackElevation();
			to_host_data.InputMode = ESP32_ONLY_DEVICE_ID; //let the application to know that ESP32 works in stand alone mode
		}
		
		//read voltage
		if (unit == ADC_UNIT_1) {
			adc_reading = adc1_get_raw((adc1_channel_t)channel);
		} else {
			int raw;
			adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
			adc_reading = raw;
		}
		voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
			
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

esp_err_t tracker_save_last_coords(int32_t * lat, int32_t * lon){

	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "About to save last latlon to flash");

		esp_err = nvs_open(tracker_nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_i32(handle, "last_lat", *lat);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_i32(handle, "last_lon", *lon);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK) return esp_err;

		nvs_close(handle);

		ESP_LOGD(TAG, "eet_last_latlon written: lat:%d lon:%d",*lat,*lon);

	return ESP_OK;
}

bool tracker_fetch_last_coords(int32_t * lat, int32_t * lon){

	nvs_handle handle;
	esp_err_t esp_err;
	if(nvs_open(tracker_nvs_namespace, NVS_READONLY, &handle) == ESP_OK){

		esp_err = nvs_get_i32(handle, "last_lat", lat);
		if(esp_err != ESP_OK)
			return false;
		
		esp_err = nvs_get_i32(handle, "last_lon", lon);
		if(esp_err != ESP_OK)
			return false;

		nvs_close(handle);

		ESP_LOGI(TAG, "eet_last_coords fetched: lat:%d lon:%d",*lat,*lon);

		return true;
	}
	else
	{
		return false;
	}
}

esp_err_t tracker_save_video_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "About to save video config to flash");

		esp_err = nvs_open(tracker_nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_u8(handle, "vid_std", gVideoStandard);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_u8(handle, "vid_thr", gVideoThreshold);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK) return esp_err;

		nvs_close(handle);

		ESP_LOGD(TAG, "eet_vid_config written: vid_std:%d vid_thr:%d",gVideoStandard,gVideoThreshold);

	return ESP_OK;
}


bool tracker_fetch_video_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	if(nvs_open(tracker_nvs_namespace, NVS_READONLY, &handle) == ESP_OK){


		esp_err = nvs_get_u8(handle, "vid_std", &gVideoStandard);
		if(esp_err != ESP_OK)
			return false;
		
		esp_err = nvs_get_u8(handle, "vid_thr", &gVideoThreshold);
		if(esp_err != ESP_OK)
			return false;

		nvs_close(handle);

		ESP_LOGI(TAG, "eet_vid_config fetched: vid_std:%d vid_thr:%d",gVideoStandard,gVideoThreshold);

		return (gVideoStandard < 2) && (gVideoThreshold > 0) && (gVideoThreshold < 128);
	}
	else{
		return false;
	}
}

esp_err_t tracker_save_ext_telemetry_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "About to save external telemerty config to flash");

		esp_err = nvs_open(tracker_nvs_namespace, NVS_READWRITE, &handle);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_u8(handle, "ext_telem_type", gExtTelemType);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_set_u8(handle, "ext_telem_baud", gExtTelemBaud);
		if (esp_err != ESP_OK) return esp_err;

		esp_err = nvs_commit(handle);
		if (esp_err != ESP_OK) return esp_err;

		nvs_close(handle);

		ESP_LOGD(TAG, "eet_ext_telem_config written: type:%d baud:%d",gExtTelemType,gExtTelemBaud);

	return ESP_OK;
}


bool tracker_fetch_ext_telemetry_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	if(nvs_open(tracker_nvs_namespace, NVS_READONLY, &handle) == ESP_OK){


		esp_err = nvs_get_u8(handle, "ext_telem_type", &gExtTelemType);
		if(esp_err != ESP_OK)
			return false;
		
		esp_err = nvs_get_u8(handle, "ext_telem_baud", &gExtTelemBaud);
		if(esp_err != ESP_OK)
			return false;

		nvs_close(handle);

		ESP_LOGI(TAG, "eet_ext_telem_config fetched: vtype:%d baud:%d",gExtTelemType,gExtTelemBaud);

		return ((gExtTelemType < 12) && (gExtTelemBaud < 6));
	}
	else{
		return false;
	}
}

void initVidStdPin()
{
	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_VID_STD_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void setVidStdPin(uint8_t std)
{
	if(std == 0)
		gpio_set_level(PIN_NUM_VID_STD, 1);
	else
		gpio_set_level(PIN_NUM_VID_STD, 0);
}

void initProgModePin()
{
	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_PROG_MODE_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
bool getProgModePin(void)
{
	return !gpio_get_level(PIN_PROG_MODE);
}

void initADC()
{
	check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
   /* esp_adc_cal_value_t val_type =*/ esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

float getVoltage()
{
	return (float)voltage;
}