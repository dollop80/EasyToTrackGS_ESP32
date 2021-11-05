#include <string.h>
#include <math.h>
#include "tiny_mavlink.h"
#include "../../define.h"

uint8_t serialBuffer[256];
//extern tagSYSCONF gSysConf;
extern TO_HOST_DATA to_host_data;
extern TELEM_DATA telem_data;

// Vars
struct __mw_mav {
  uint8_t  message_cmd;
  uint8_t  message_length;
  uint8_t  mode;
  uint8_t  sequence;
  uint16_t serial_checksum;
  uint16_t tx_checksum;
}mw_mav;

union _px4_custom_mode{
struct{
  uint16_t reserved;
  uint8_t main_mode;
  uint8_t sub_mode;
};
uint32_t data;
} px4_custom_mode;


void mav_checksum(uint8_t val) {
  uint16_t tmp;
  tmp = (val ^ mw_mav.serial_checksum) & 0xFF;
  tmp ^= (tmp<<4)&0xFF;
  mw_mav.serial_checksum = (mw_mav.serial_checksum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);  
}


float serialbufferfloat(uint8_t offset){
  float f_temp = 0;
  uint8_t i;
  uint8_t * b = (uint8_t *) &f_temp;
  for(i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return f_temp;
}


int32_t serialbufferint(uint8_t offset){
  int32_t i_temp = 0; 
  uint8_t i;
  uint8_t * b = (uint8_t *) &i_temp;
  for(i=0;i<4;i++){
    b[i]=serialBuffer[offset+i];
  }
  return i_temp;
}

int16_t serialbufferint16(uint8_t offset){
  int16_t i_temp = 0; 
  uint8_t i;
  uint8_t * b = (uint8_t *) &i_temp;
  for(i=0;i<3;i++){
    b[i]=serialBuffer[offset+i];
  }
  return i_temp;
}


void mav_tx_checksum_func(int val) {
  long tmp;
  tmp = (val ^ mw_mav.tx_checksum) & 0xFF;
  tmp ^= (tmp<<4) & 0xFF;
  mw_mav.tx_checksum = ( mw_mav.tx_checksum>>8) ^ (tmp<<8) ^ (tmp<<3) ^ (tmp>>4);  
}


void mav_serialize8(uint8_t val) {
  mav_tx_checksum_func(val);
  ///write_USART(val);
}


void mav_serialize16(uint16_t val) {
  mav_serialize8((val   ) & 0xFF);
  mav_serialize8((val>>8) & 0xFF);
}


void mavlink_msg_request_data_stream_send(uint8_t MAVStreams, uint16_t MAVRates){
  //head:
  static int8_t tx_sequence=0;
  tx_sequence++;
  mw_mav.tx_checksum=0xFFFF; //init
  ///write_USART(0xFE);
  mav_serialize8(6);
  mav_serialize8(tx_sequence);
  mav_serialize8(99);
  mav_serialize8(99);
  mav_serialize8(MAVLINK_MSG_ID_REQUEST_DATA_STREAM);  
  //body:
  mav_serialize16(MAVRates); //MAVRates
  mav_serialize8(1);
  mav_serialize8(1);
  mav_serialize8(MAVStreams);
  mav_serialize8(1);
  //tail:
  mav_tx_checksum_func(MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC);
  ///write_USART((uint8_t)(mw_mav.tx_checksum&0xFF));
  ///write_USART((uint8_t)(mw_mav.tx_checksum>>8&0xFF));
}


void request_mavlink_rates(){
  #define  maxStreams 6
  int i;
  const uint8_t MAVStreams[maxStreams] = {
    MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_RC_CHANNELS,
    MAV_DATA_STREAM_POSITION,
    MAV_DATA_STREAM_EXTRA1, 
    MAV_DATA_STREAM_EXTRA2
  };
  const uint16_t MAVRates[maxStreams] = {
    0x02, 0x02, 0x05, 0x02, 0x05, 0x02                  
  };
  for (i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_send(MAVStreams[i], MAVRates[i]);
  }
}


uint8_t serialMAVCheck(){
  uint8_t res = 0;
  
  switch(mw_mav.message_cmd) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    
    if(0/*gSysConf.InputMode==MAVLINK_PX4*/)
    {
      px4_custom_mode.data = serialbufferint(0);
      if (px4_custom_mode.main_mode == 1)
        to_host_data.APmode = 0; // Manual
      else if (px4_custom_mode.main_mode == 2)
        to_host_data.APmode = 1; // Altitude control
      else if (px4_custom_mode.main_mode == 3)
        to_host_data.APmode = 2; // Position control
      else if (px4_custom_mode.main_mode == 4) {
        // mw_mav.mode = x; // Auto (ready)
        if (px4_custom_mode.sub_mode == 2)
          to_host_data.APmode = 3; // Takeoff
        else if (px4_custom_mode.sub_mode == 3)
          to_host_data.APmode = 4; // Loiter
        else if (px4_custom_mode.sub_mode == 4)
          to_host_data.APmode = 5; // Mission
        else if (px4_custom_mode.sub_mode == 5)
          to_host_data.APmode = 6; // Return to land
        else if (px4_custom_mode.sub_mode == 6)
          to_host_data.APmode = 7; // Landing
      }
      else if (px4_custom_mode.main_mode == 5)
        to_host_data.APmode = 8; // Acro mode (MC)
      else if (px4_custom_mode.main_mode == 6)
        to_host_data.APmode = 9; // Offboard control
      else if (px4_custom_mode.main_mode == 7)
        to_host_data.APmode = 10; // Stabilized mode (FW)
      else
        to_host_data.APmode = 11; // Unknown mode    
    }
    else
    {
        to_host_data.APmode = (uint8_t)serialbufferint(0);
    }
#if defined MAVLINKREQ
     static uint8_t mavreqdone=5;
     if (mavreqdone>0){ 
       request_mavlink_rates();
       mavreqdone--;
     }
#endif //MAVLINKREQ
    break;
  case MAVLINK_MSG_ID_VFR_HUD: 
    to_host_data.Airspeed = (int16_t)serialbufferfloat(0)*18/5;
    to_host_data.GPS_speed=(int16_t)serialbufferfloat(4)*18/5;//36/10;    
    telem_data.Alt = (int16_t)serialbufferfloat(8); 
    //gFc_Course = serialBuffer[16] | (int16_t)(serialBuffer[17] << 8);
    to_host_data.GPS_course = serialbufferint16(16);
    //gPntCurrent.Alt=(int16_t)serialbufferfloat(8);
    //gGPS_Course=serialBuffer[16]|serialBuffer[17]<<8; // deg (-->deg*10 if GPS heading)
    break;
  case MAVLINK_MSG_ID_ATTITUDE:
    to_host_data.RollAngle=(int16_t)(serialbufferfloat(4)*57.2958); // rad-->deg
    to_host_data.PitchAngle=-(int16_t)(serialbufferfloat(8)*57.2958); // rad-->deg
    break;
  case MAVLINK_MSG_ID_GPS_RAW_INT:
    telem_data.GPS_sats=serialBuffer[29];                                                                         
    telem_data.GPS_mode=serialBuffer[28];                                                                            
    //to_host_data.GPS_Course = (serialBuffer[26]|((uint16_t)(serialBuffer[27])<<8))/100;
    to_host_data.GPS_lat = serialbufferint(8)/100;
    to_host_data.GPS_lon = serialbufferint(12)/100;
    //t_data->PntCurrent.Alt=(int16_t)(serialbufferint(16)/1000); 
    telem_data.GPS_pdop = (uint8_t)((serialBuffer[20]|((uint16_t)(serialBuffer[21])<<8))/10);
    res = 1; 
    break;
  case MAVLINK_MSG_ID_SYS_STATUS:
    to_host_data.BattVoltage = (serialBuffer[14]|((uint16_t)(serialBuffer[15])<<8))/100;
    to_host_data.BattCurrent = (serialBuffer[16]|((int16_t)(serialBuffer[17])<<8))*10; 
    to_host_data.BattCapacity = serialBuffer[30];
    break;
  case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
    to_host_data.CTRL_RSSI = serialBuffer[21];
  }
  return res;
}


uint8_t mav_parse(uint8_t c)
{
  static uint8_t  mav_payload_index; 
  static uint16_t mav_checksum_rcv; 
  static uint8_t  mav_magic=0;  
  uint8_t  mav_len = 0, res = 0;

  static enum _serial_state {
    MAV_IDLE,
    MAV_HEADER_START,
    MAV_HEADER_LEN,
    MAV_HEADER_SEQ,
    MAV_HEADER_SYS,
    MAV_HEADER_COMP,
    MAV_HEADER_MSG,
    MAV_PAYLOAD,
    MAV_CHECKSUM,
  }
  mav_state = MAV_IDLE;

  if ((mav_state == MAV_IDLE)||(mav_state == MAV_PAYLOAD))
  {
  }
  else
  {
    mav_checksum(c);
  }

  if (mav_state == MAV_IDLE)
  {
    if (c==0xFE)
    {
      mw_mav.serial_checksum=0xFFFF;
      mav_payload_index=0;
      mav_state = MAV_HEADER_START;
    }
    else
    {
      mav_state = MAV_IDLE;
    } 
  }
  else if (mav_state == MAV_HEADER_START)
  {
    mw_mav.message_length = c;
    mav_state = MAV_HEADER_LEN;
  }
  else if (mav_state == MAV_HEADER_LEN)
  {
    mav_state = MAV_HEADER_SEQ;
  }
  else if (mav_state == MAV_HEADER_SEQ)
  {
    if (1 || c==MAV_SYS_ID){
      to_host_data.ID = c;
      mav_state = MAV_HEADER_SYS;        
    }
    else{
      mav_state = MAV_IDLE;  
    }
  }
  else if (mav_state == MAV_HEADER_SYS)
  {
    if (1 || c==MAV_COM_ID){
      mav_state = MAV_HEADER_COMP;        
    }
    else{
      mav_state = MAV_IDLE;  
    }
  }
  else if (mav_state == MAV_HEADER_COMP)
  {
    mw_mav.message_cmd = c;
     
    switch(c) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        mav_magic = MAVLINK_MSG_ID_HEARTBEAT_MAGIC;
        mav_len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
        break;
      case MAVLINK_MSG_ID_VFR_HUD:
        mav_magic = MAVLINK_MSG_ID_VFR_HUD_MAGIC;
        mav_len = MAVLINK_MSG_ID_VFR_HUD_LEN;
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        mav_magic = MAVLINK_MSG_ID_ATTITUDE_MAGIC;
        mav_len = MAVLINK_MSG_ID_ATTITUDE_LEN;
        break;
      case MAVLINK_MSG_ID_GPS_RAW_INT:
        mav_magic = MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC;
        mav_len = MAVLINK_MSG_ID_GPS_RAW_INT_LEN;
        break;
      case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
        mav_magic = MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC;
        mav_len = MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN;
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        mav_magic = MAVLINK_MSG_ID_SYS_STATUS_MAGIC;
        mav_len = MAVLINK_MSG_ID_SYS_STATUS_LEN;
        break;
      }
      if ((mw_mav.message_length) == mav_len){  // too much data so reset check
         mav_state = MAV_HEADER_MSG;
      }
      else{
        mav_state = MAV_IDLE;
      }
  }
  else if (mav_state == MAV_HEADER_MSG)
  {
    serialBuffer[mav_payload_index]=c;
    mav_payload_index++;
    if (mav_payload_index==mw_mav.message_length){  // end of data
      mav_state = MAV_PAYLOAD;
    }
  }
  else if (mav_state == MAV_PAYLOAD)
  {
    if (mav_payload_index==mw_mav.message_length){
      mav_checksum_rcv=c;
      mav_payload_index++;
    }
    else{
      mav_checksum_rcv+=((uint16_t)c<<8);
      mav_checksum(mav_magic);
      if(mav_checksum_rcv == mw_mav.serial_checksum) {
        res = serialMAVCheck();
      }
      mav_state = MAV_IDLE;
    }
  }
  return res;
}
