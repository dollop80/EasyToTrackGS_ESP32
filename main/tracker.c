
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

#include "math.h"
#include "define.h"
#include "components/protocols/simple_bin.h"
#include "protocol_detection.h"

void CalcGpsVec(GPS_PNT *src, GPS_PNT *dst, GPS_VEC *v);
int16_t cos_b(int16_t angle);
int16_t AddAngleU(int16_t a, int16_t b);

extern TELEM_DATA telem_data;
extern TO_HOST_DATA to_host_data;
extern AZ_ELEV_DATA az_elev_data;
extern uint16_t gTelAzimuth;
extern uint8_t gTelElevation;
extern tracker_mode gTracker_m;

static uint8_t gGPS_starting = 0;
static uint8_t gGPS_ready = 0;
static uint8_t gGPS_update = 0;
static uint8_t gGPS_tmp = 0;
static uint8_t gGPS_Timeout = 0;
static uint32_t gGPS_lon_gm = 0;

GPS_PNT gPntStart, gPntCurrent;
GPS_VEC gVecToStart;

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
			if(deck_pack.msg[0] == 0x04 && deck_pack.msg[4] == 0x01) //FROMHOST_MANUAL_AZIMUTH_MSG_ID and force manual mode 
			{
				gTracker_m = MANUAL;//manual_tracker_mode = true;
				to_host_data.Track_azimuth = deck_pack.msg[2] + ((uint16_t)deck_pack.msg[1] << 8);
				to_host_data.Track_elevation = deck_pack.msg[3];
			}
			else if(deck_pack.msg[0] == 0x04 && deck_pack.msg[4] == 0x00)
			{
				//manual_tracker_mode = false;
				if(getProtocol() == TP_MSV)
					gTracker_m = TRACKING_V;
				else if(getProtocol())
					gTracker_m = TRACKING_T;
			}
			
			else if(deck_pack.msg[0] == 0x00)
			{
				if(deck_pack.msg[9] == 1)
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
			
		}
	}
	return false;
}

void tracker_task(void *pvParameters)
{
	while(1)
	{
		if(telem_data.GPS_mode > 2 && (gTracker_m!=MANUAL /*!manual_tracker_mode*/))
		{
			calc_los();
			gTelAzimuth = CalcTrackAzimut();
			gTelElevation = CalcTrackElevation();
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}