#include "stdlib.h"

void calc_los(void);
void process_gps(void);
int16_t CalcTrackAzimut(void);
int16_t CalcTrackElevation(void);
void sendHostHomeMessageToGS(void);
bool decode_packet_for_host(uint8_t * rx_buffer, int len);
void decode_packet_and_send_to_gs(const char * rx_buffer, int len);
void tracker_task(void *pvParameters);
esp_err_t tracker_save_video_config(void);
bool tracker_fetch_video_config(void);
void initVidStdPin(void);
void setVidStdPin(uint8_t std);
void initProgModePin(void);
bool getProgModePin(void);