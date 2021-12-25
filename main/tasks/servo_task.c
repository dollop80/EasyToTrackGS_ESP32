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

#include "nvs_flash.h"
#include "nvs.h"

#include "../define.h"
#include "../tracker.h"
#include "servo_task.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "driver/uart.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

extern TaskHandle_t xHandleServo;
extern TO_HOST_DATA to_host_data;
extern AZ_ELEV_DATA az_elev_data;
extern FROM_HOST_DATA from_host_data;
extern tracker_mode gTracker_m;
extern RSSI_DATA gRssiData;
extern bool home_from_host;
extern int16_t gTrack_elevation;
extern int16_t gTrack_azimuth;
extern bool settings_changed;
extern bool g_gefault_az_elev_change;

static const char TAG[] = "ETT-SERVO";

//предельные лимиты для серв Выжимаем все.
//перед началом эксплуатации сконфигурить реальные лимиты.
#define servo_min_limit 400 //1000 //0.4ms //1500 // .6ms 
#define servo_max_limit 2600 //6500 //2.6ms //6000 // 2.4ms

CALIBR_DATA gCallibr;


extern uint16_t gTelAzimuth;
extern uint8_t gTelElevation;

uint16_t g_calibValueToSend[2];

uint16_t EEP_Off_azimuth;
uint16_t EEP_azimuth_min_limit;
uint16_t EEP_azimuth_max_limit;
uint16_t EEP_elevation_min_limit;
uint16_t EEP_elevation_max_limit;
uint8_t EEP_mode360;
uint8_t EEP_soundOn;
uint8_t EEP_delay_change_ppm;
uint16_t EEP_def_azimuth;
uint8_t EEP_def_elevation;
uint16_t EEP_RSSImax;
uint16_t EEP_RSSImin;

uint16_t EEP_ang_min[2];
uint16_t EEP_ang_max[2];

uint16_t gPPMOldVal[2] = {1000, 1000};
uint16_t gPPMOldMod[2] = {0};



const char servo_nvs_namespace[] = "ettsrvcfg";

esp_err_t servo_save_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	ESP_LOGI(TAG, "About to save servo config to flash");

	esp_err = nvs_open(servo_nvs_namespace, NVS_READWRITE, &handle);
	if (esp_err != ESP_OK) {ESP_LOGI(TAG, "NVS open ERROR"); return esp_err;}
		
	
	esp_err = nvs_set_u16(handle, "off_az", EEP_Off_azimuth);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 1"); return false;}
	
	esp_err = nvs_set_u16(handle, "az_min_lim", EEP_azimuth_min_limit);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 2"); return false;}
	
	esp_err = nvs_set_u16(handle, "az_max_lim", EEP_azimuth_max_limit);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 3"); return false;}
	
	esp_err = nvs_set_u16(handle, "el_min_lim", EEP_elevation_min_limit);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 4"); return false;}
	
	esp_err = nvs_set_u16(handle, "el_max_lim", EEP_elevation_max_limit);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 5"); return false;}
	
	esp_err = nvs_set_u8(handle, "mode360", EEP_mode360);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 6"); return false;}

	esp_err = nvs_set_u8(handle, "sound_on", EEP_soundOn);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 7"); return false;}		
	
	esp_err = nvs_set_u8(handle, "delay_ch_ppm", EEP_delay_change_ppm);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 8"); return false;}	

	esp_err = nvs_set_u16(handle, "def_az", EEP_def_azimuth);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 9"); return false;}

	esp_err = nvs_set_u8(handle, "def_el", EEP_def_elevation);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 10"); return false;}		

	esp_err = nvs_set_u16(handle, "rssi_max", EEP_RSSImax);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 11"); return false;}

	esp_err = nvs_set_u16(handle, "rssi_min", EEP_RSSImin);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 12"); return false;}			
	
	esp_err = nvs_set_u16(handle, "ang_el_min", EEP_ang_min[1]);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 13"); return false;}

	esp_err = nvs_set_u16(handle, "ang_el_max", EEP_ang_max[1]);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 14"); return false;}

	esp_err = nvs_set_u16(handle, "ang_az_min", EEP_ang_min[0]);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 15"); return false;}

	esp_err = nvs_set_u16(handle, "ang_az_max", EEP_ang_max[0]);
	if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Save servo config ERROR 16"); return false;}

	esp_err = nvs_commit(handle);
	if (esp_err != ESP_OK) {ESP_LOGI(TAG, "NVS commit ERROR"); return esp_err;}

	nvs_close(handle);

	ESP_LOGD(TAG, "servo_config written");

	return ESP_OK;
}

bool servo_fetch_config(){

	nvs_handle handle;
	esp_err_t esp_err;
	
	ESP_LOGI(TAG, "About to restore servo config from flash");
	
	if(nvs_open(servo_nvs_namespace, NVS_READONLY, &handle) == ESP_OK){


		esp_err = nvs_get_u16(handle, "off_az", &EEP_Off_azimuth);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 1"); return false;}
		
		esp_err = nvs_get_u16(handle, "az_min_lim", &EEP_azimuth_min_limit);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 2"); return false;}
		
		esp_err = nvs_get_u16(handle, "az_max_lim", &EEP_azimuth_max_limit);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 3"); return false;}
		
		esp_err = nvs_get_u16(handle, "el_min_lim", &EEP_elevation_min_limit);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 4"); return false;}
		
		esp_err = nvs_get_u16(handle, "el_max_lim", &EEP_elevation_max_limit);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 5"); return false;}
		
		esp_err = nvs_get_u8(handle, "mode360", &EEP_mode360);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 6"); return false;}
		
		esp_err = nvs_get_u8(handle, "sound_on", &EEP_soundOn);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 7"); return false;}
		
		esp_err = nvs_get_u8(handle, "delay_ch_ppm", &EEP_delay_change_ppm);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 8"); return false;}

		esp_err = nvs_get_u16(handle, "def_az", &EEP_def_azimuth);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 9"); return false;}

		esp_err = nvs_get_u8(handle, "def_el", &EEP_def_elevation);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 10"); return false;}

		esp_err = nvs_get_u16(handle, "rssi_max", &EEP_RSSImax);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 11"); return false;}

		esp_err = nvs_get_u16(handle, "rssi_min", &EEP_RSSImin);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 12"); return false;}

		esp_err = nvs_get_u16(handle, "ang_el_min", &EEP_ang_min[1]);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 13"); return false;}

		esp_err = nvs_get_u16(handle, "ang_el_max", &EEP_ang_max[1]);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 14"); return false;}

		esp_err = nvs_get_u16(handle, "ang_az_min", &EEP_ang_min[0]);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 15"); return false;}

		esp_err = nvs_get_u16(handle, "ang_az_max", &EEP_ang_max[0]);
		if(esp_err != ESP_OK) {ESP_LOGI(TAG, "Restore servo config ERROR 16"); return false;}

		nvs_close(handle);

		if(EEP_ang_min[0] == EEP_ang_max[0])
		{
			EEP_ang_min[0] = 0; 
			EEP_ang_max[0] = EEP_mode360 ? 360 : 180;
			return false;
		}
		if(EEP_ang_min[1] == EEP_ang_max[1])
		{
			EEP_ang_min[1] = 0;
			EEP_ang_max[1] = EEP_mode360 ? 90 : 180;
			return false;
		}	

		ESP_LOGI(TAG, "servo_config fetched");

		return ((EEP_Off_azimuth <= 360) && (EEP_azimuth_min_limit >= 400) && (EEP_azimuth_min_limit <= 2600) && 
											(EEP_azimuth_max_limit >= 400) && (EEP_azimuth_max_limit <= 2600) && 
											(EEP_elevation_min_limit >= 400) && (EEP_elevation_min_limit <= 2600) && 
											(EEP_elevation_max_limit >= 400) && (EEP_elevation_max_limit <= 2600) && 
											(EEP_mode360 < 2) && (EEP_soundOn < 2) && (EEP_delay_change_ppm <= 150) &&
											(EEP_def_azimuth <= 360) && (EEP_def_elevation <= 180) && 
											(EEP_RSSImax < 1024) && (EEP_RSSImin < 1024)
											);
	} else {
		nvs_close(handle);
		return false;
	}
}

//-----------------------------------------------------
int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//-----------------------------------------------------

static void set_ppm_out(uint8_t ch, uint16_t val)
{ 
	//val H 0-360град 
	//val V=90
	int32_t ul;

	int16_t new_val, old_val;
	int16_t m, y;
	
	if(EEP_delay_change_ppm == 0) EEP_delay_change_ppm = 1;

	//val=180-val; // Inverted!
	y=(gCallibr.OutPPM_Max[ch]-gCallibr.OutPPM_Min[ch]);
	int vals;
	ul=y; 
	//ul*=val;

 	if (ch == 0) { //Azimuth
		//val=360-val;   reverse
		if (EEP_mode360) {
			vals = map(val, EEP_ang_min[ch], EEP_ang_max[ch], 0, 360);
			ul *= vals;
			new_val = (int16_t)(ul / 360); //ul/360
		}
		else {
			vals = map(val, EEP_ang_min[ch], EEP_ang_max[ch], 0, 180);
			ul *= vals;
			new_val = (int16_t)(ul / 180); //ul/180
		}
	}
	else 
	{ //Elevation
		if (EEP_mode360) {
			vals = map(val, EEP_ang_min[ch], EEP_ang_max[ch], 0, 90);
			ul *= vals;
			new_val = (int16_t)(ul / 90); //ul / 90 Режим 360 градусов весь ход сервы элевации - 90 градусов
		}
		else {
			vals = map(val, EEP_ang_min[ch], EEP_ang_max[ch], 0, 180);
			ul *= vals;
			new_val = (int16_t)(ul / 180); //ul / 180 Режим 180 градусов весь ход сервы элевации - 180 градусов
		}
	}

	new_val += gCallibr.OutPPM_Min[ch];
	if (new_val > gCallibr.OutPPM_Max[ch]) new_val = gCallibr.OutPPM_Max[ch];
	if (new_val < gCallibr.OutPPM_Min[ch]) new_val = gCallibr.OutPPM_Min[ch];

	y=abs(y);
	old_val=gPPMOldVal[ch];
	if(old_val==new_val) return;
	m=gPPMOldMod[ch];
	if(old_val<new_val) m+=y; else m-=y;
	y=(m/EEP_delay_change_ppm);
	m-=(y*EEP_delay_change_ppm);
	gPPMOldMod[ch]=m;
	y+=old_val;
	if(old_val<new_val && y>new_val);
	else if(old_val>new_val && y<new_val);
	else new_val=y;
	gPPMOldVal[ch]=new_val;
	if(ch==1) { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, new_val); }
	else      { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, new_val); }

	char txbuf[30];
	int len = sprintf(txbuf, "ch=%d,Val=%d\r\n", ch, new_val);
	uart_write_bytes(UART_NUM_0, (const char *) txbuf, len);

}
//----------------------------------------------------------------------
static void CalcTrackPos(void)
{
	uint16_t angle;
	uint16_t Azimuth, Elevation;

	if(EEP_mode360 && gTelElevation > 90) gTelElevation = 90;  //correction when switched between 180/360 mode from application
	az_elev_data.Track_azimuth = gTelAzimuth;//gTrack_azimuth;
	az_elev_data.Track_elevation = gTelElevation;//gTrack_elevation;

	angle=gTelAzimuth;//gTrack_azimuth;

	angle+=EEP_Off_azimuth;
	//angle=0;
	if(angle>=360) {
		angle = angle % 360;
	}

	if((angle>180) && (!EEP_mode360))
	{
	//Режим 180 градусов - "запрокидывание головы"
	  Azimuth=(angle-180);
	  Elevation=(180-gTelElevation);
	  } 
	else
	  {
	  Azimuth=angle;
	  Elevation=gTelElevation;
	  }
	to_host_data.Track_azimuth=Azimuth;
	to_host_data.Track_elevation=Elevation;
	set_ppm_out(0, Azimuth);    
	set_ppm_out(1, Elevation);    
}


//-----------------------------------------------------
static void send_chanel_ppm(uint8_t ch, int16_t new_val){
	int16_t old_val;
	int16_t m, y;

	y=(servo_max_limit-servo_min_limit);
	old_val=gPPMOldVal[ch];
	if(old_val==new_val) return;
	m=gPPMOldMod[ch];
	if(old_val<new_val) m+=y; else m-=y;
	y=(m/EEP_delay_change_ppm);
	m-=(y*EEP_delay_change_ppm);
	gPPMOldMod[ch]=m;
	y+=old_val;
	if(old_val<new_val && y>new_val);
	else if(old_val>new_val && y<new_val);
	else new_val=y;
	gPPMOldVal[ch]=new_val;

	if(ch==1) { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, new_val); }
	else      { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, new_val); }
}


uint16_t check_servo_limit_mid(uint16_t servo_value){
// Установка начальных значений  лимитов при инициализации или сбое епрома.
    if( servo_value < servo_min_limit || servo_value > servo_max_limit){
       
        return 1500;//3750; // Базовая средняя точка  
    } 
    return servo_value;
}

uint16_t check_servo_limit(uint16_t servo_value){
// Контроль лимитов при настройке крайних точек.
    if( servo_value < servo_min_limit){
        //BUZ_1=1; //Легкий писк;
        return servo_min_limit;  
    } 
    
     if( servo_value > servo_max_limit){
        //BUZ_1=1; //Легкий писк;
        return servo_max_limit;  
    }  
    
    return servo_value;
}

void check_command_from_host(void)
{
static uint8_t cnt=0;
///BUZ=0;
switch(from_host_data.mode)
    {
        case 1: //change servo settings
            gCallibr.OutPPM_Min[0] = check_servo_limit(from_host_data.OutPPM_Min[0]); 
            g_calibValueToSend[0] = gCallibr.OutPPM_Min[0]; //send_chanel_ppm(0,gCallibr.OutPPM_Min[0]); //Антенна ВЛЕВО до упора 
        break;
        case 2: //change servo settings 
            gCallibr.OutPPM_Max[0] = check_servo_limit(from_host_data.OutPPM_Max[0]);  
            g_calibValueToSend[0] = gCallibr.OutPPM_Max[0];//send_chanel_ppm(0,gCallibr.OutPPM_Max[0]); //Антенна ВПРАВО до упора
            
        break;
        case 3: //change servo settings 
            gCallibr.OutPPM_Min[1] = check_servo_limit(from_host_data.OutPPM_Min[1]);  
            g_calibValueToSend[1] = gCallibr.OutPPM_Min[1];//send_chanel_ppm(1,gCallibr.OutPPM_Min[1]); //Антенна ВНИЗ до упора
        break;
        case 4: //change servo settings 
            gCallibr.OutPPM_Max[1] = check_servo_limit(from_host_data.OutPPM_Max[1]);  
            g_calibValueToSend[1] = gCallibr.OutPPM_Max[1];//send_chanel_ppm(1,gCallibr.OutPPM_Max[1]); //Антенна ВВЕРХ до упора 
        break;
        case 5:
            if(from_host_data.mode_360==0)
            {
                EEP_mode360=0;
            }
            else
            {
                EEP_mode360=1;
            }
   
        break;
        case 6: //save settings to EEPROM        

                EEP_azimuth_min_limit = gCallibr.OutPPM_Min[0];
                EEP_azimuth_max_limit = gCallibr.OutPPM_Max[0];
                EEP_elevation_min_limit = gCallibr.OutPPM_Min[1];
                EEP_elevation_max_limit = gCallibr.OutPPM_Max[1];

				EEP_ang_min[1] = from_host_data.ang_min[1];
				EEP_ang_max[1] = from_host_data.ang_max[1];
				EEP_ang_min[0] = from_host_data.ang_min[0];
				EEP_ang_max[0] = from_host_data.ang_max[0];

				if(EEP_ang_min[0] == EEP_ang_max[0])
				{
					EEP_ang_min[0] = 0; 
					EEP_ang_max[0] = from_host_data.mode_360 ? 360 : 180;
				}
				if(EEP_ang_min[1] == EEP_ang_max[1])
				{
					EEP_ang_min[1] = 0;
					EEP_ang_max[1] = from_host_data.mode_360 ? 90 : 180;
				}				

				char txbuf[50];
				int len = sprintf(txbuf, "Emin=%d,Emax=%d,Amin=%d,Amax=%d\r\n", EEP_ang_min[1], EEP_ang_max[1], EEP_ang_min[0], EEP_ang_max[0]);
				uart_write_bytes(UART_NUM_0, (const char *) txbuf, len);
				servo_save_config();

        break;
         
        default:
        break;
    }
	
    // switch(gRssiData.mode)
    // {
        // case 1: //rssi
            // if(cnt++>100){                  
                // gRssiData.curr = read_adc(0);
                // send_UART(9, (u_char *)&gRssiData, sizeof(RSSI_DATA), 1);
                // cnt=0;
            // }
        // break;
        // case 2: //save rssi   
            // EEP_RSSImax = gRssiData.max;
            // EEP_RSSImin = gRssiData.min; 
            // CalcRssiLevels(); 
			// /*
            // BUZ=1;
            // delay_ms(30);
            // BUZ=0;
            // delay_ms(20);
            // BUZ=1;
            // delay_ms(30);
            // BUZ=0;
			// */			
            // programming_mode = 0;            
        // break;
         
        // default:
        // break;    
    // }
    
    from_host_data.mode=0;
    // if(gRssiData.mode!=1)
        // gRssiData.mode = 0;   
}






 void initServo(void) {
	ESP_LOGI(TAG, "Initializing mcpwm servo control gpio...");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_AZ);    //Set PIN_AZ as PWM0A, to which servo is connected
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_EL);    //Set PIN_EL as PWM0A, to which servo is connected
	
	if(!servo_fetch_config())
	{ //it was an error, so reset to defaults!
		ESP_LOGI(TAG, "Something went wrong. Restoring defaults");
		EEP_Off_azimuth = 0;
		EEP_azimuth_min_limit = 1400;
		EEP_azimuth_max_limit = 1600;
		EEP_elevation_min_limit = 1400;
		EEP_elevation_max_limit = 1600;
		EEP_mode360 = 0;
		EEP_soundOn = 0;
		EEP_delay_change_ppm = 50;
		EEP_def_azimuth = 0;
		EEP_def_elevation = 0;
		EEP_RSSImax = 10;
		EEP_RSSImin = 0;		

		EEP_ang_min[1] = 0;
		EEP_ang_max[1] = 180;
		EEP_ang_min[0] = 0;
		EEP_ang_max[0] = 180;  

		servo_save_config();
	}
	
	gCallibr.OutPPM_Max[0] = EEP_azimuth_max_limit;
	gCallibr.OutPPM_Max[1] = EEP_elevation_max_limit;
	gCallibr.OutPPM_Min[0] = EEP_azimuth_min_limit;
	gCallibr.OutPPM_Min[1] = EEP_elevation_min_limit;
	
	g_calibValueToSend[0]=gCallibr.OutPPM_Min[0];
	g_calibValueToSend[1]=gCallibr.OutPPM_Min[1];
	
	to_host_data.Track_azimuth = gTelAzimuth;
	to_host_data.Track_elevation = gTelElevation;		

	to_host_data.ID = 0;
	to_host_data.InputMode = ESP32_ONLY_DEVICE_ID;

	to_host_data.GS_Version = VERSION + 11;
}


void servo_task(void *pvParameter)
{
    //1. mcpwm gpio initialization
    initServo();

    //2. initial mcpwm configuration
	ESP_LOGI(TAG, "Configuring Initial Parameters of mcpwm...");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	
    while (1) {
		if(g_gefault_az_elev_change)
		{
			g_gefault_az_elev_change=false;
			EEP_def_azimuth = gTelAzimuth;//gTrack_azimuth;
			EEP_def_elevation = gTelElevation;//gTrack_elevation;  
			settings_changed = true;				
		} 
		
		if(settings_changed)
		{
			settings_changed = false;
			servo_save_config();
		}
			
		if(gTracker_m == SETUP)
		{
			check_command_from_host();
					
            send_chanel_ppm(0,g_calibValueToSend[0]);  
            send_chanel_ppm(1,g_calibValueToSend[1]); 
			
		} else {
			CalcTrackPos();
		}
		vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
    }
}




