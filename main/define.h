#ifndef define_h
#define define_h
//----------------------------------------
#include <stdint.h>
//----------------------------------
#define DG

#define ESP32_ONLY CONFIG_ETT_ESP32_ONLY

#define VERSION 20

#define PIN_NUM_VID_STD  15
#define GPIO_VID_STD_PIN_SEL (1ULL<<PIN_NUM_VID_STD)
#define PIN_PROG_MODE  13
#define GPIO_PROG_MODE_SEL (1ULL<<PIN_PROG_MODE)
#define DIGITAL_THRH_POT
//--------------------------------------
#define FROMHOST_MSG_ID_SETUP 0x00
#define SERVO_REQ_MSG_ID 0x01
#define FROMHOST_AZIMUTH_MSG_ID 0x02
#define FROMHOST_SOUND_MSG_ID 0x03
#define FROMHOST_MANUAL_AZIMUTH_MSG_ID 0x04
#define FROMHOST_PARAMS_ID_SETUP 0x05
#define HOST2GS_POWEROFF_ID 0x06
#define HOST2GS_HOST_HOME_ID 0x07
#define HOST2GS_RSSI_ID 0x08
#define GS2HOST_RSSI_ID 0x09
#define GS2HOST_ADDITIONAL_DATA_ID 0x0A
#define FROMHOST_VIDEOSETTINGS_ID 0x0B
#define FROMHOST_EXTTELEMSETTINGS_ID 0x0C
//--------------------------------------

#define ESP32_ONLY_DEVICE_ID 50

#define GRAD_METER_CONST 1139

#define SERIALBUFFERSIZE 150

#define TELEM_TIMEOUT	5000 //ms

#define BIT_HORIZ   0b00000001
#define BIT_BARO    0b00000010
#define BIT_MAG     0b00000100
#define BIT_HOLD    0b00001000
#define BIT_ANGLE   0b00010000
//
//-------------------------------------------------
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))


typedef enum {
	TP_MFD				= (1 << 2),
	TP_GPS_TELEMETRY	= (1 << 3),
	TP_MAVLINK			= (1 << 4),
	TP_RVOSD			= (1 << 5),
	TP_FRSKY_D			= (1 << 6),
	TP_FRSKY_X 			= (1 << 7),
	TP_LTM				= (1 << 8),
	TP_PITLAB			= (1 << 9),
	TP_MSP				= (1 << 10),
	TP_MSV				= (1 << 11),

} trackerProtocolFlags_t;


typedef enum {BT, WIFI} connection_type;
typedef enum {B9600, B19200, B38400, B57600, B115200} uart_baudrate;
typedef enum {NO, AP, STA} wifi_mode;
typedef enum {TRACKING_V, TRACKING_T, MANUAL, SETUP} tracker_mode;


//-------------------------------------------------

// VIDEO TELEMETRY FRAMES //
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x10
    int16_t Track_azimuth;    // 3,4 
    int16_t Track_elevation;  // 5,6 uint8_t enough!! 
    uint8_t Reserve[4];      // 7,8,9,10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_00;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x20
    int32_t GPS_lat;         // 3,4,5,6
    int32_t GPS_lon;         // 7,8,9,10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_01;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x30
    uint16_t  GPS_course;      // 3,4
    int16_t  Altitude;        // 5,6
    uint16_t  GPS_speed;       // 7,8 
    uint8_t APmode;          // 9
    uint8_t CTRL_RSSI;       // 10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_02;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x40
    int16_t RollAngle;        // 3,4 +-512;
    int16_t PitchAngle;       // 5,6 +-512;
    int16_t Airspeed;         //7, 8
    uint8_t ID;              // 9
    uint8_t Reserve;         //10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_03;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x50
    uint16_t BattVoltage;      // 3,4  V*100
    uint16_t BattCurrent;      // 5,6 mA 65,534A max
    uint16_t BattCapacity;     // 7,8 65534mAh max
    //uint8_t Reserve[2];    // 9,10 !!!!The difference from the original MSV
    uint8_t InputMode;       // gps nmea, gps ubx, mav plane, mav copter, mav rover, ltm inav...    
    uint8_t AP_Version;         // device fw version. /10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_04;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x60
    int32_t Home_lat;         // 3,4,5,6
    int32_t Home_lon;         // 7,8,9,10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_05;
//---------------------------------------------------------------
typedef struct
{
    //uint8_t start_byte;      // 1.   0xff
    uint8_t num_frame_line;  // 2.   0x70
    uint16_t  DistToHome;         // 3,4
    uint16_t  DistTraveled;         //5,6 
    uint8_t Reserved[4]; //7,8,9,10
    uint16_t  CRC;             // 11,12
} __attribute__ ((packed)) TT_FRAME_06;
//--------------------------------------------------------------

//---------------------------------------------------------------
// TO HOST MAIN DATA STRUCT
typedef struct
{
    int16_t  Track_azimuth;
    int16_t  Track_elevation;
    int32_t GPS_lat;
    int32_t GPS_lon;
    int32_t Home_lat;
    int32_t Home_lon;
    uint16_t  GPS_course;
    int16_t  Altitude;
    uint16_t  GPS_speed;
    uint8_t APmode;
    uint8_t CTRL_RSSI;
    int16_t RollAngle;
    int16_t PitchAngle;
    uint16_t BattVoltage;
    uint16_t BattCurrent;
    uint16_t BattCapacity;
    uint16_t  AV_RSSI;
    uint8_t AVErrors;
    int16_t Airspeed;    //added in the version 12
    uint8_t GS_Version; //in fully MSV-mode
    uint8_t InputMode;  //these two bytes are reserved 
    uint8_t AP_Version; //it is an additional byte   
    uint8_t ID; //another additional byte.
} __attribute__ ((packed)) TO_HOST_DATA;  // 46+1 bytes
//----------------------------------------------------------------------

// TO HOST ELEVATION AZIMUTH DATA STRUCT
typedef struct
{
    uint16_t  Track_azimuth;
    uint8_t  Track_elevation;
} __attribute__ ((packed)) AZ_ELEV_DATA;  // 3 bytes

// TO HOST RSSI DATA STRUCT
typedef struct
{
    uint8_t ID;
    uint8_t mode;
    uint16_t  curr;
    uint16_t  max;
    uint16_t  min;
}  __attribute__ ((packed)) RSSI_DATA;  // 6 bytes

// VIDEO SETUP STRUCT
typedef struct
{
    uint8_t ID;
    uint8_t save;
    uint8_t standard;
    uint8_t threshold;
}  __attribute__ ((packed)) VIDEO_DATA;  // 4 bytes

// TO GS EXTERNAL HOME DATA STRUCT
typedef struct
{
    uint8_t ID;
    uint16_t  Track_azimuth;
    uint8_t  Track_elevation;
    int32_t Home_lat;
    int32_t Home_lon;
} __attribute__ ((packed)) HOST_HOME_DATA;  // 12 bytes


// TO GS SETUP DATA STRUCT
typedef struct
{
    uint16_t OutPPM_Min[2];
    uint16_t OutPPM_Max[2];
    uint8_t mode_360;
    uint8_t mode; 
    uint16_t AzimuthOffset; 
    uint8_t soundOn; 
    uint8_t delay_change_ppm;
    uint16_t ang_min[2];
    uint16_t ang_max[2];
} __attribute__ ((packed)) FROM_HOST_DATA;  // 9 + 6 bytes

// GS 2 HOST ADDITIONAL DATA STRUCT
typedef struct
{
    uint16_t  DistToHome;
    uint16_t  DistTraveled;   
    uint8_t reserved[30]; //another additional byte.
} __attribute__ ((packed)) TO_HOST_ADD_DATA;  // 34 bytes

// TO HOST / FROM HOST SERVO CALIBRATION DATA STRUCT
typedef struct
{
    uint16_t OutPPM_Min[2];
    uint16_t OutPPM_Max[2];
} __attribute__ ((packed)) CALIBR_DATA;
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//-------------------------------------------------
enum { PCC_VER_ERR='0',
       PPC_GET_VER='1',
       PCC_GET_ACHANNEL='A',
       PCC_GET_CALIBR='B',
       PCC_GET_HP='C',
       PCC_GET_ALARMS='D',
       PCC_GET_CTRL='E',
       PCC_GET_IMU='F',
       PCC_GET_ROLL='G',
       PCC_GET_PITCH='H',
       PCC_GET_THR='I',
       PCC_GET_ADDPROP='J',
       PCC_GET_DISCRET='K',
       PCC_GET_AP='L',
       PCC_GET_ADD='M',
       PCC_GET_SCR='N',
       PCC_GET_POINT='O',
       PCC_GET_SYSCONF='P',    
       PCC_GET_ADDCONF='Q',
       PCC_GET_SCRITEMS='S',

       PCC_SET_REZERVE0='a',
       PCC_SET_CALIBR='b',
       PCC_SET_HP='c',
       PCC_SET_ALARMS='d',
       PCC_SET_CTRL='e',
       PCC_SET_IMU='f',
       PCC_SET_ROLL='g',
       PCC_SET_PITCH='h',
       PCC_SET_THR='i',
       PCC_SET_ADDPROP='j',
       PCC_SET_DISCRET='k',
       PCC_SET_AP='l',
       PCC_SET_ADD='m',
       PCC_SET_SCR='n',
       PCC_SET_POINT='o',
       PCC_SET_SYSCONF='p',
       PCC_SET_ADDCONF='q',
       PCC_SET_SCRITEMS='s'
       };
       
enum{
    MSV = 0, //default. It s for compatibility with MSV autopilot
    NMEA = 1,
    UBLOX = 2,
    MAVLINK_ARDUPLANE = 3,
    MAVLINK_ARDUCOPTER = 4,
    MAVLINK_ARDUROVER = 5,
    LTM = 6,
    MSP = 7,
    MAVLINK_PX4 = 8
}; //inputMode
enum{
    baud_19200 = 0,
    baud_38400 = 1,
    baud_57600 = 2,
    baud_115200 = 3
};

#define FLASH_VERSION 6

//-------------------------------------------------
typedef struct
{
uint16_t BattU;
uint16_t CurrentMin; // quant
uint16_t CurrentMax; // quant
uint16_t CurrentMV;  // mA
uint16_t RSSI_Min;
uint16_t RSSI_Max;
} __attribute__ ((packed)) tagCALIBR;
//-------------------------------------------------
typedef struct
{
uint8_t BoldGraphic:1;    // 0,1
uint8_t RSSI_graph:1;     // 0,1
uint8_t IMU_draw:1;       // 0,1
uint8_t MessageOut:1;     // 0,1
uint8_t GPS_pos:2;        // 0..2
uint8_t AltitudeSrc:2;    // 0-GPS only, 1 GPS+Baro, 2 Baro+GPS, 3 Baro only
uint8_t SpeedSrc:2;       // 0-GPS only, 1 GPS+Baro, 2 Baro+GPS, 3 Baro only
uint8_t TemperatureSrc:1; // 0-KTY81-120, 1-MS5611
uint8_t PressureOut:1;    // 0, 1
uint8_t CourceSrc:1;      // 0, 1 - 0-GPS, 1-FC
uint8_t CyrilicCompas:1;  // 0, 1 - 0-EN, 1-Cyr
uint8_t FlightTimerMode:2;// 0..2. 0 - GPS time, 1 - Power on time, 2 - Armed time
uint8_t Reserve;
} __attribute__ ((packed)) tagSCREEN;

//-------------------------------------------------
typedef struct
{
uint8_t PortSpeed; //19 - 19200, 38 - 38400, 57 - 57600, 115 - 115200
uint8_t InputMode; //0 - NMEA, 1 - UBX, 2 - MAVLINK, 3 - 
uint8_t BatteryRemainMode; //0 - Ah used, 1 - Persentage
uint8_t DisplayStatus; //0 - off, 1 - on
uint8_t VideoMode; //0 - PAL, 1 - NTSC
uint8_t StatusPosX;
uint8_t StatusPosY;
uint8_t ID;    //device ID 0..255
} __attribute__ ((packed)) tagSYSCONF;
typedef struct
{
uint8_t RssiType:2;    //0..3. 0 - Percentage, 1 - raw value
uint8_t RssiMin;       // 0..255
uint8_t RssiMax;       // 0..255
uint8_t TimeZone;
uint16_t  BattCapacity;  // ������� ���������
char  CallSign[12];
uint16_t Reserve3;
} __attribute__ ((packed)) tagADDCONF;
typedef struct
{
uint8_t versionfw;
uint8_t versioneeprom;
} __attribute__ ((packed)) tagVERSION;
//-------------------------------------------------
//-------------------------------------------------
//---------------------------------------------------------------------------
typedef struct
{
int32_t Lon;
int32_t Lat;
int16_t Alt;
} __attribute__ ((packed)) GPS_PNT;
//---------------------------------------------------------------------------
typedef struct
{
int16_t Course;
uint32_t Distance;
} __attribute__ ((packed)) GPS_VEC;
//---------------------------------------------------------------------------
typedef struct
{
int16_t Roll;
int16_t Pitch;
int16_t Yaw;
int8_t Overload;
uint8_t ErrorGyro;
uint8_t Debug;
uint8_t Flags;
uint8_t CRC; // for SPI only!
} __attribute__ ((packed)) IMU_DATA;
//---------------------------------------------------------------------------
enum { AP_MODE_MANUAL, AP_MODE_SETTRIMMER, AP_MODE_SLAVE, AP_MODE_FBW,
       AP_MODE_RTH, AP_MODE_FAILSAFE, AP_MODE_TAKEOFF, AP_MODE_LANDING, AP_MODE_FOP };
//-------------------------------------------------
typedef struct
{
	uint8_t 	ApModeExt;
	GPS_PNT 	PntHome;
	uint8_t 	GPS_mode;
	uint8_t 	GPS_sats;
	uint8_t 	GPS_pdop; 
	uint8_t		GPS_frame;
	int16_t 	Alt;	
	uint8_t 	Armed;
	uint8_t		Errors;
}  __attribute__ ((packed)) TELEM_DATA;
//-------------------------------------------------

#endif