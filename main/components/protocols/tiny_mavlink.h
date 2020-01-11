#ifndef TINY_MAVLINK_H_
#define TINY_MAVLINK_H_

#include <stdint.h>

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_MAGIC 50
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_VFR_HUD_MAGIC 20
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_MAGIC 39
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_MAGIC 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_MAGIC 244
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22    
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_MAGIC 124
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31  
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_MAGIC 148
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6

#define MAV_DATA_STREAM_RAW_SENSORS 1
#define MAV_DATA_STREAM_EXTENDED_STATUS 2
#define MAV_DATA_STREAM_RC_CHANNELS 3
#define MAV_DATA_STREAM_POSITION 6
#define MAV_DATA_STREAM_EXTRA1 10
#define MAV_DATA_STREAM_EXTRA2 11
#define  LAT  0
#define  LON  1

//#define MAVLINKREQ

#define MAV_SYS_ID 1                // System ID of MAV. 
#define MAV_COM_ID 1                // Component ID of MAV.

uint8_t mav_parse(uint8_t c);

#endif /* MAVLINK_TYPES_H_ */