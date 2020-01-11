#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"

#include "monitoring_task.h"

extern TaskHandle_t xHandleTCP;
extern TaskHandle_t xHandleMonitor;
extern TaskHandle_t xHandleOled;
extern TaskHandle_t xHandleUart1;
extern TaskHandle_t xHandleUart2;
extern TaskHandle_t xHandleSendToHost;
extern TaskHandle_t xHandleTracker;

extern uint32_t gUpTimeS;
extern uint8_t gUpTimeSs; 
extern uint16_t gUpTimeM;

static const char TAG[] = "ETT-MON";
/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
void monitoring_task(void *pvParameter)
{
	UBaseType_t sizes[7]; 
	uint16_t cnt = 0;
	
	for(;;){
		if(cnt++%10==0)
		{
			sizes[0] = uxTaskGetStackHighWaterMark(xHandleTCP);
			sizes[1] = uxTaskGetStackHighWaterMark(xHandleMonitor);			
			sizes[2] = uxTaskGetStackHighWaterMark(xHandleOled);		
			sizes[3] = uxTaskGetStackHighWaterMark(xHandleUart1);
			sizes[4] = uxTaskGetStackHighWaterMark(xHandleUart2);
			sizes[5] = uxTaskGetStackHighWaterMark(xHandleSendToHost);
			sizes[6] = uxTaskGetStackHighWaterMark(xHandleTracker);
			
			
			ESP_LOGI(TAG, "freeheap: %d, %d, %d, %d, %d, %d, %d, %d",esp_get_free_heap_size(), sizes[0], sizes[1], sizes[2], sizes[3], sizes[4], sizes[5], sizes[6]);
		}
		vTaskDelay( pdMS_TO_TICKS(1000));
		gUpTimeS++;
		if(++gUpTimeSs > 59) {gUpTimeSs=0; gUpTimeM++;}
	}
}