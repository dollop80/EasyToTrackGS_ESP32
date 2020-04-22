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

#include "../define.h"
#include "../tracker.h"
#include "servo_task.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

extern TaskHandle_t xHandleServo;
extern TO_HOST_DATA to_host_data;


//static const char TAG[] = "ETT-SERVO";

//предельные лимиты для серв Выжимаем все.
//перед началом эксплуатации сконфигурить реальные лимиты.
#define servo_min_limit 400 //1000 //0.4ms //1500 // .6ms 
#define servo_max_limit 2600 //6500 //2.6ms //6000 // 2.4ms

CALIBR_DATA gCallibr;

uint8_t mode_360_ram = 0;
int16_t gTrack_elevation = 10;
int16_t gTrack_azimuth = 30;

extern uint16_t gTelAzimuth;
extern uint8_t gTelElevation;

int16_t gOff_azimuth = 0;
int g_delay_change_ppm = 1;


//-----------------------------------------------------
u_int gPPMOldVal[2];
u_int gPPMOldMod[2];

static void set_ppm_out(uint8_t ch, uint16_t val)
{ 
//val H 0-360град 
//val V=90
int32_t ul;

int16_t new_val, old_val;
int16_t m, y;

//val=180-val; // Inverted!
y=(gCallibr.OutPPM_Max[ch]-gCallibr.OutPPM_Min[ch]);
ul=y; ul*=val;

if(ch==0){
//val=360-val;   reverse
    if(mode_360_ram){
        new_val=(int16_t)(ul/360);
    }
    else{ 
        new_val=(int16_t)(ul/180);
    }
    
    new_val+=gCallibr.OutPPM_Min[ch];
}
else{

    if(mode_360_ram){
         new_val=(int16_t)(ul/90); // Режим 360 градусов весь ход сервы элевации - 90 градусов
    }else{
         new_val=(int16_t)(ul/180); // Режим 180 градусов весь ход сервы элевации - 180 градусов
    }   
    new_val+=gCallibr.OutPPM_Min[ch];
}

y=abs(y);
old_val=gPPMOldVal[ch];
if(old_val==new_val) return;
m=gPPMOldMod[ch];
if(old_val<new_val) m+=y; else m-=y;
y=(m/g_delay_change_ppm);
m-=(y*g_delay_change_ppm);
gPPMOldMod[ch]=m;
y+=old_val;
if(old_val<new_val && y>new_val);
else if(old_val>new_val && y<new_val);
else new_val=y;
gPPMOldVal[ch]=new_val;

if(ch==1) { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, new_val); }
else      { mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, new_val); }

}
//----------------------------------------------------------------------
static void CalcTrackPos(void)
{
uint16_t angle;
uint16_t Azimuth, Elevation;

if(mode_360_ram && gTelElevation > 90) gTelElevation = 90;  //correction when switched between 180/360 mode from application
////gAzElevData.Track_azimuth = gTrack_azimuth;
////gAzElevData.Track_elevation = gTrack_elevation;

angle=gTelAzimuth;//gTrack_azimuth;

angle+=gOff_azimuth;
//angle=0;
if(angle>=360) {
    angle= angle % 360;
}

if((angle>180) && (!mode_360_ram))
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
////gHostData.Track_azimuth=Azimuth;
////gHostData.Track_elevation=Elevation;
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
y=(m/g_delay_change_ppm);
m-=(y*g_delay_change_ppm);
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






 void initServo(void) {
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_AZ);    //Set PIN_AZ as PWM0A, to which servo is connected
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_EL);    //Set PIN_EL as PWM0A, to which servo is connected
	
	gCallibr.OutPPM_Max[0] = 2000;
	gCallibr.OutPPM_Max[1] = 2000;
	gCallibr.OutPPM_Min[0] = 1000;
	gCallibr.OutPPM_Min[1] = 1000;
	
	
	to_host_data.Track_azimuth = gTelAzimuth;
	to_host_data.Track_elevation = gTelElevation;		

	to_host_data.ID = 19;

	to_host_data.GS_Version = 34;
	
}


static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

void servo_task(void *pvParameter)
{
	
	uint32_t angle, count;
    //1. mcpwm gpio initialization
    initServo();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	
    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
			/*
            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
			angle = servo_per_degree_init(SERVO_MAX_DEGREE - count);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, angle);
            */
			CalcTrackPos();
			vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}




