#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_spiffs.h"
#include "esp_log.h"

#include "components/avr-isp/avr_isp_helper.h"
#include "m8_programmer.h"


const char * name = "/spiffs/GroundSt.hex";
int foundSig = -1;
uint8_t fuses [5];
esp_err_t ret;
spi_device_handle_t spi;
static const char TAG[] = "ETT-M8PROG";

void initSPI()
{
	ESP_LOGI(TAG, "Initializing SPI");
	
	spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=4
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*500,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the programmer to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
	
	gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_NUM_RST, 1);
}

void initSPIFS()
{
	ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = false
    };
	// Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
	
	size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

uint8_t runProgrammer()
{
	ESP_LOGI(TAG, "Starting Programmer");
	uint8_t ret = 0;
	target_poweron(spi);
	
	foundSig = getSignature (spi);
	getFuseBytes (spi, fuses);
	printf("\n\rFuses: ");
	  for(int i = 0; i < 5; i++)
		  printf(" 0x%02X ",fuses[i]);
	  // don't have signature? don't proceed
	  if (foundSig == -1)
		{
			stopProgramming (spi);
			return ret;
		}  // end of no signature	
	
	// verify that MCU flash is up to date
	if (readHexFile(spi, name, verifyFlash))
	{
      printf("\n\rStarting MCU flash update process\n\r");
	  bool ok = writeFlashContents (spi, name);
	  if(ok){
		ESP_LOGI(TAG, "DONE!!!!!");
		ret = 1;
	  } else {
		ESP_LOGI(TAG, "FAILED!!!!!"); 
		ret = 0;
	  }
	}
	else
	{
		ret = 2;
		printf("\n\rMCU flash is up to date\n\r");
	}
	stopProgramming (spi);

	gpio_set_level(PIN_NUM_RST, 1);  
	return ret;
}