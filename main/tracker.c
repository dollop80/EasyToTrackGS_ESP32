
#include "stdlib.h"
#include "math.h"
#include "define.h"

void CalcGpsVec(GPS_PNT *src, GPS_PNT *dst, GPS_VEC *v);
int16_t cos_b(int16_t angle);
int16_t AddAngleU(int16_t a, int16_t b);

extern TELEM_DATA telem_data;
extern TO_HOST_DATA to_host_data;

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