idf_component_register(SRCS "main.c" 
					"u8g2_esp32_hal.c"
					"telem.c"
					"tracker.c"
			    		"protocol_detection.c"
					"m8_programmer.c"
					"BThelper.c"
					"tasks/monitoring_task.c"
					"tasks/tcp_server_task.c"
					"tasks/oled_task.c"
					"tasks/uart_task.c"
					"tasks/send_to_host_task.c"

                    INCLUDE_DIRS ".")


# Create a SPIFFS image from the contents of the 'spiffs_image' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash'. 
#spiffs_create_partition_image(storage ../atmega_hex FLASH_IN_PROJECT)

