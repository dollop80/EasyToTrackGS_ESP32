#include <string.h>
#include <math.h>
#include "tiny_ltm.h"
#include "../../define.h"

uint8_t serialBuffer[256];
extern TO_HOST_DATA to_host_data;
extern TELEM_DATA telem_data;


// Vars
struct __mw_ltm {
  uint8_t mode;
  uint8_t LTMreceiverIndex;
  uint8_t LTMcmd;
  uint8_t LTMrcvChecksum;
  uint8_t LTMreadIndex;
  uint8_t LTMframelength;
  uint16_t GPS_altitude_home;
  uint16_t batUsedCapacity;  
  float    GPS_scaleLonDown;  
}mw_ltm;


uint8_t ltmread_u8()  {
  return serialBuffer[mw_ltm.LTMreadIndex++];
}

uint16_t ltmread_u16() {
  uint16_t t = ltmread_u8();
  t |= (uint16_t)ltmread_u8() << 8;
  return t;
}

uint32_t ltmread_u32() {
  uint32_t t = ltmread_u16();
  t |= (uint32_t)ltmread_u16() << 16;
  return t;
}

int32_t ltmread_s32() {
  int32_t t = ltmread_u16();
  t |= (int32_t)ltmread_u16() << 16;
  return t;
}

uint8_t ltm_check() {
  uint8_t res = 0;
  uint32_t dummy;
  uint8_t ltm_satsfix;  
  uint8_t ltm_armfsmode;               
  mw_ltm.LTMreadIndex=0;
  
  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_GFRAME)
  {
    to_host_data.GPS_lat = ltmread_s32();
    to_host_data.GPS_lon = ltmread_s32();
    to_host_data.GPS_speed = (uint16_t)ltmread_u8() * 18/5;//36/10;                // LTM gives m/s, we expect km/h
    telem_data.Alt = (ltmread_s32())/100;   // LTM altitude in cm, we expect m.
    
	ltm_satsfix = ltmread_u8();
    telem_data.GPS_sats = (ltm_satsfix >> 2) & 0xFF;
    telem_data.GPS_mode = ((ltm_satsfix & 0b00000011) <= 1) ? 0 : 1; 
    
    res = 1;
  }

  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_AFRAME)
  {
    to_host_data.PitchAngle=(int16_t)ltmread_u16();
    to_host_data.RollAngle=(int16_t)ltmread_u16();
    to_host_data.GPS_course = (int16_t)ltmread_u16();
#ifdef HEADINGCORRECT
    if (to_host_data.GPS_course >= 180) to_host_data.GPS_course -= 360;
#endif
  }
  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_SFRAME)
  {
    to_host_data.BattVoltage = ltmread_u16()/100;
    to_host_data.BattCapacity = ltmread_u16();  
    to_host_data.CTRL_RSSI = ltmread_u8()/2.6; // 0-255 to 0-100
    dummy = ltmread_u8();
    ltm_armfsmode = ltmread_u8();
    
    dummy = (ltm_armfsmode >> 1) & 0b00000001; // uavData.isFailsafe
    mw_ltm.mode = (ltm_armfsmode >> 2) & 0b00111111; // uavData.flightMode
    mw_ltm.mode = (mw_ltm.mode>15) ? 15 : mw_ltm.mode;
    to_host_data.APmode = mw_ltm.mode; 
  }

  if (mw_ltm.LTMcmd == LIGHTTELEMETRY_OFRAME)
  {
    
  }
  return res;
}

uint8_t ltm_parse(uint8_t c) {
  uint8_t res = 0;
  static enum _serial_state {
    LTM_IDLE,
    LTM_HEADER_START1,
    LTM_HEADER_START2,
    LTM_HEADER_MSGTYPE,
    LTM_HEADER_DATA
  }
  c_state = LTM_IDLE;

  if (c_state == LTM_IDLE) {
    c_state = (c == '$') ? LTM_HEADER_START1 : LTM_IDLE;
  }
  else if (c_state == LTM_HEADER_START1) {
    c_state = (c == 'T') ? LTM_HEADER_START2 : LTM_IDLE;
  }
  else if (c_state == LTM_HEADER_START2) {
    switch (c) {
    case 'G': //G
      mw_ltm.LTMframelength = LIGHTTELEMETRY_GFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'A': //A
      mw_ltm.LTMframelength = LIGHTTELEMETRY_AFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'S': //S
      mw_ltm.LTMframelength = LIGHTTELEMETRY_SFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    case 'O': //O
      mw_ltm.LTMframelength = LIGHTTELEMETRY_OFRAMELENGTH;
      c_state = LTM_HEADER_MSGTYPE;
      break;
    default:
      c_state = LTM_IDLE;
    }
    mw_ltm.LTMcmd = c;
    mw_ltm.LTMreceiverIndex = 0;
  }
  else if (c_state == LTM_HEADER_MSGTYPE) {
    if (mw_ltm.LTMreceiverIndex == 0) {
      mw_ltm.LTMrcvChecksum = c;
    }
    else {
      mw_ltm.LTMrcvChecksum ^= c;
    }
    if (mw_ltm.LTMreceiverIndex == mw_ltm.LTMframelength - 4) { // received checksum byte
      if (mw_ltm.LTMrcvChecksum == 0) {
        res = ltm_check();
        c_state = LTM_IDLE;
      }
      else {                                                   // wrong checksum, drop packet
        c_state = LTM_IDLE;
      }
    }
    else serialBuffer[mw_ltm.LTMreceiverIndex++] = c;
  } 
  return res;
}




































