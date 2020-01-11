#include <stdlib.h>
#include <math.h>
#include "../../define.h"
#include "pitlab.h"

int32_t Restore_long(int idx);
int16_t Restore_short(int idx);
uint8_t Restore_byte(int idx);
void pitlab_encodeTargetData(uint8_t c);
void preProcessHexString(void);
uint8_t processPitlabFrame(void);
uint8_t hex2int(uint8_t *a, uint8_t len);

extern TO_HOST_DATA to_host_data;
extern TELEM_DATA telem_data;
/*
Telemetry packet on USART port is always 10 ASCII characters as follow:
'$' //sync
('A'+ N) //packet numer 'N': 'A'=0, 'B'=1 'C'=2 etc
32 bit data in HEX format, MSB first

Example:
$CA9FGDB15
$-start of packet
C- packet nr 2
A9 - byte3
FG - byte2
DB - byte1
15 - byte0
*/

int32_t gps_lat;
int32_t gps_lon;


//uint8_t type is single, unsigned byte (unsigned char)
uint8_t lsRxData[5]; //bufor na kolejne bajty odczytane z komunikatu (dan Hex zamienione na bajty)
uint8_t hexString[8];

enum PitlabDataState {
    IDLE,
    STATE_START1,
    STATE_START2,
    STATE_DATA
  };

static uint8_t dataState = IDLE;
uint8_t dataIdx=0;

int32_t Restore_long(int idx)
{
  return (int32_t)lsRxData[idx] + ((int32_t)lsRxData[idx+1] << 8) + ((int32_t)lsRxData[idx+2] << 16) + ((int32_t)lsRxData[idx+3] << 24);
}

int16_t Restore_short(int idx)
{
  return (int16_t)lsRxData[idx] + ((int16_t)lsRxData[idx+1]  << 8);
}

uint8_t Restore_byte(int idx)
{
  return lsRxData[idx];
}


uint8_t pitlab_parse(uint8_t c) {
	uint8_t res = 0;

	if (dataState == IDLE && c == '$') {
		dataIdx = 0;
		dataState = STATE_START1;
		return res;
	} else if (dataState == STATE_START1) {
		lsRxData[0] = c - 'A';
		dataState=STATE_START2;
		return res;
	} else if (dataState == STATE_START2) {
		hexString[dataIdx++] = c;
		if(dataIdx == 8)
			dataState = STATE_DATA;
	}
	if (dataState == STATE_DATA){
		preProcessHexString();
		res = processPitlabFrame();
		dataState = IDLE;
	}
	
	return res;
}

void preProcessHexString(void){
	uint8_t str_buffer[2];
	uint8_t sIdx = 0;
	for(uint8_t i = 1; i < 5; i++){
		for(uint8_t j = 0; j < 2; ++j){
			str_buffer[j] = hexString[sIdx++];
		}
		lsRxData[5-i] = hex2int(str_buffer,2);
	}
}

uint8_t  processPitlabFrame(void){
	uint8_t res = 0;
	switch(lsRxData[0])
	{
	case 0: // A
		telem_data.GPS_sats = (uint16_t)Restore_byte(4);
		res = 1;
		break;
	case 10: // 10 = K, pos 3 : Absolute altitude, 1 = B, pos 2 : Relative altitude
		telem_data.Alt = (int16_t)Restore_short(3);
		///gotAlt = true;
		res = 1;
		break;
	case 2: // C
		gps_lon = Restore_long(1);
		to_host_data.GPS_lon = (int32_t)(round(((double)gps_lon * 100.0)/60.0));
		res = 1;
		break;
	case 3: // D
		gps_lat = Restore_long(1);
		to_host_data.GPS_lat = (int32_t)(round(((double)gps_lat * 100.0)/60.0));
		///if(telemetry_sats >= 5) gotFix = true;
		res = 1;
		break;
	}
	return res;
}

uint8_t hex2int(uint8_t *a, uint8_t len)
{
   uint8_t i;
   int val = 0;

   for(i=0;i<len;i++) {
      if(a[i] <= 57)
       val += (a[i]-48)*(1<<(4*(len-1-i)));
      else
       val += (a[i]-55)*(1<<(4*(len-1-i)));
   }
   return val;
}
