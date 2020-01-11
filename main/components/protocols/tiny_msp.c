#include <string.h>
#include <math.h>
#include "tiny_msp.h"
#include "../../define.h"

//#define BOXNAMES

#define TX_CHANNELS 8

uint8_t serialBuffer[256]; // this hold the imcoming string from serial O string
extern TO_HOST_DATA to_host_data;
extern TELEM_DATA telem_data;

static uint8_t receiverIndex;
static uint8_t dataSize;
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;
static uint8_t txChecksum;
static uint16_t MwSensorPresent;
static uint32_t MwSensorActive;
static uint8_t txChecksum;

///void send_USART(char *buff, uint8_t cnt);

static uint16_t MwRcData[1+16]={   // This hold receiver pulse signal
  1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500} ;

struct __flags {
  uint8_t ident;
  uint8_t box;
  uint8_t reset;
  uint8_t signaltype;
}
flags;

// Mode bits
struct __mode {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint8_t baro;
  uint8_t mag;
  uint16_t camstab;
  uint16_t gpshome;
  uint16_t gpshold;
  uint16_t passthru;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t gpsland;
}mode;


uint8_t read8()  {
  return serialBuffer[readIndex++];
}

uint16_t read16() {
  uint16_t t = read8();
  t |= (uint16_t)read8()<<8;
  return t;
}

int16_t read16s() {
  int16_t t = read8();
  t |= (int16_t)read8()<<8;
  return t;
}

uint32_t read32() {
  uint32_t t = read16();
  t |= (uint32_t)read16()<<16;
  return t;
}

int32_t read32s() {
  int32_t t = read16();
  t |= (int32_t)read16()<<16;
  return t;
}


#define skip8() {readIndex++;}
#define skip16() {readIndex+=2;}
#define skip32() {readIndex+=4;}
#define skipn(n) {readIndex+=n;}

//
// Legacy mspWrite*()
//


void mspWrite8(uint8_t t){
  ///write_USART(t);
  txChecksum ^= t;
}
/*
void mspWrite16(uint16_t t){
  mspWrite8((uint8_t)t);
  mspWrite8(t>>8);
}

void mspWrite32(uint32_t t){
  mspWrite8((uint8_t)t);
  mspWrite8(t>>8);
  mspWrite8(t>>16);
  mspWrite8(t>>24);
}
*/
void mspWriteChecksum(){
  ///write_USART(txChecksum);
}

void mspWriteRequest(uint8_t mspCommand, uint8_t txDataSize){
  //return;
  ///write_USART('$');
  ///write_USART('M');
  ///write_USART('<');
  txChecksum = 0;
  mspWrite8(txDataSize);
  mspWrite8(mspCommand);
  if(txDataSize == 0)
    mspWriteChecksum();
}



// Writes to GUI (OSD_xxx) is distinguished from writes to FC (MSP_xxx) by
// cfgWrite*() and mspWrite*().
//
// If I2C is not used, then all cfgWrite*() will be mspWrite*().
/*
# define cfgWriteRequest mspWriteRequest
# define cfgWrite8 mspWrite8
# define cfgWrite16 mspWrite16
# define cfgWrite32 mspWrite32
# define cfgWriteChecksum mspWriteChecksum
  */

// --------------------------------------------------------------------------------------
// Here are decoded received commands from MultiWii
uint8_t serialMSPCheck()
{
  uint8_t temp;
  uint8_t remaining, c, res=0;
  uint32_t Bit;
  readIndex = 0;
#ifdef BOXNAMES
  uint8_t  len=0;
#endif    

  if (cmdMSP==MSP_IDENT)
  {
    //MwVersion= read8();                             // MultiWii Firmware version   
  }

  if (cmdMSP==MSP_RAW_GPS)
  {
    temp=read8();
    telem_data.GPS_sats=read8();
    to_host_data.GPS_lat = read32s()/100;
    to_host_data.GPS_lon = read32s()/100;
    //t_data->PntCurrent.Alt = read16s();
	read16s();
    to_host_data.GPS_speed = read16()*4/111;
    //to_host_data.GPS_course = read16()/10;
    
	if (temp!=0 && telem_data.GPS_sats>=5) telem_data.GPS_mode = 3; //in MSP there is only GPS_FIX data, but we need fix mode
    else if (temp!=0 && telem_data.GPS_sats < 5) telem_data.GPS_mode = 1;
    else telem_data.GPS_mode = 0;
	
    res = 1;
  }

  if (cmdMSP==MSP_COMP_GPS)
  {
    //GPS_distanceToHome=read16();    
    //GPS_directionToHome=read16();
  }

  if (cmdMSP==MSP_ATTITUDE)
  {
    to_host_data.RollAngle = read16s()/10;
    to_host_data.PitchAngle = read16s()/10;  
    int16_t GPS_course = read16s(); //Range [-180;180]
    if (GPS_course < 0) GPS_course+=360; 
	to_host_data.GPS_course = GPS_course;
    /*
      MwHeading = read16();
    #ifdef HEADINGCORRECT
      if (MwHeading >= 180) MwHeading -= 360;
    #endif
    */
  }  
  
  if (cmdMSP==MSP_STATUS)
  {
    read16();
    read16();
    MwSensorPresent = read16();
    MwSensorActive = read32(); 
  }
     
  if (cmdMSP==MSP_RC)
  {
	/*
    for(temp=1;temp<=TX_CHANNELS;temp++)
      MwRcData[temp] = read16();
    t_data->Out_Roll = MwRcData[1]/5-300;  //-100..100
    t_data->Out_Pitch = MwRcData[2]/5-300; //-100..100
    t_data->Out_Thr = MwRcData[4]/5-300;   //-100..100
	*/
  }  
  
  if (cmdMSP==MSP_ANALOG)
  {
    to_host_data.BattVoltage=read8();
    to_host_data.BattCapacity=read16();
    to_host_data.CTRL_RSSI = (uint8_t)read16()/2;//2.6;
    to_host_data.BattCurrent = (uint16_t)read16()*10;
 }  
  
  if (cmdMSP==MSP_ALTITUDE)
  {
    telem_data.Alt = read32s()/100;
    //t_data->FcVario = read16s()/100;
  }

 
#ifdef BOXNAMES
  Bit = 1L;
  remaining = dataSize;
    
  if(cmdMSP==MSP_BOXNAMES) {
    flags.box=1;
    memset(&mode, 0, sizeof(mode));

    char boxname[20];

    while(remaining > 0) {
      char c = read8();
      if(c != ';') {
        boxname[len] = c;
        len++;
      }
      else {
          for (int i = 0; boxnames[i].name; i++) {
              if (strncmp_P(boxname, boxnames[i].name, len) == 0) {
                  switch (boxnames[i].size) {
                  case 8:
                      *(uint8_t*)boxnames[i].var |= Bit;
                      break;
                  case 16:
                      *(uint16_t*)boxnames[i].var |= Bit;
                      break;
                  case 32:
                      *(uint32_t*)boxnames[i].var |= Bit;
                      break;
                  }
                  break;
              }
          }

        len = 0;
        bit <<= 1L;
      }
      --remaining;
    }
  }
#else  
  if(cmdMSP==MSP_BOXIDS) {
    flags.box=1;
    Bit = 1;
    remaining = dataSize;
    
    memset(&mode, 0, sizeof(mode));

    while(remaining > 0) {
      c = read8();
      switch(c) {
      case 0:
        mode.armed |= Bit;
        break;
      case 1:
        mode.stable |= Bit;
        break;
      case 2:
        mode.horizon |= Bit;
        break;
      case 3:
        mode.baro |= Bit;
        break;
      case 5:
        mode.mag |= Bit;
        break;
      case 8:
        mode.camstab |= Bit;
       break;
      case 10:
        mode.gpshome |= Bit;
        break;
      case 11:
        mode.gpshold |= Bit; 
        break;
      case 12:
        mode.passthru  |= Bit;
        break;
      case 16:
        mode.llights |= Bit;
        break;
      case 19:
        mode.osd_switch |= Bit;
        break;
      case 21:
        mode.gpsland |= Bit;
        break;
      }
      Bit = Bit<<1L;
      --remaining;
    }
    
    //gApMode
    uint8_t ApModeExt = 0;
    to_host_data.APmode = 0;
    if ((MwSensorActive&mode.stable) || (MwSensorActive&mode.horizon))
    { 
        to_host_data.APmode = 1; 
        if (MwSensorActive&mode.horizon) bit_set(ApModeExt,BIT_HORIZ);
        else bit_set(ApModeExt,BIT_ANGLE);    
    }
    if(MwSensorActive&mode.gpshome)
    { to_host_data.APmode = 3;}
    if(MwSensorActive&mode.gpshold)
    { to_host_data.APmode = 4; bit_set(ApModeExt,BIT_HOLD);} 
    if(MwSensorActive&mode.passthru)
    { to_host_data.APmode = 5;} 
    
    if (MwSensorActive&mode.baro){
        to_host_data.APmode = 2;
        bit_set(ApModeExt,BIT_BARO);
        }
    if (MwSensorActive&mode.mag) bit_set(ApModeExt,BIT_MAG);  
  }
#endif

 //t_data->Armed = (MwSensorActive & mode.armed) != 0;
 return res; 
}
// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------

uint8_t msp_parse(uint8_t c)
{  
  uint8_t res = 0;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;  

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    }
    else if (c_state == HEADER_ARROW)
    {
      if (c > SERIALBUFFERSIZE)
      {  // now we are expecting the payload size
        c_state = IDLE;
      }
      else
      {
        dataSize = c;
        c_state = HEADER_SIZE;
        rcvChecksum = c;
      }
    }
    else if (c_state == HEADER_SIZE)
    {
      c_state = HEADER_CMD;
      cmdMSP = c;
      rcvChecksum ^= c;
      receiverIndex=0;
    }
    else if (c_state == HEADER_CMD)
    {
      rcvChecksum ^= c;
      if(receiverIndex == dataSize) // received checksum byte
      {
        if(rcvChecksum == 0) {
            res = serialMSPCheck();
        }
        c_state = IDLE;
      }
      else
        serialBuffer[receiverIndex++]=c;
    }
    return res;
  }


void setMspRequests(uint32_t modeMSPRequests, uint32_t queuedMSPRequests) {
    modeMSPRequests = 
      REQ_MSP_STATUS|
      REQ_MSP_RC|
      REQ_MSP_ALTITUDE|
      REQ_MSP_RAW_GPS| 
      //REQ_MSP_COMP_GPS|
      REQ_MSP_BOX|
      REQ_MSP_ANALOG|
      0; // Sigh...
  queuedMSPRequests &= modeMSPRequests;   // so we do not send requests that are not needed.
}

