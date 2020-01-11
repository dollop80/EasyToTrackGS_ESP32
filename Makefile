#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := wifi_manager

EXTRA_COMPONENT_DIRS := ../../
EXTRA_COMPONENT_DIRS += ./main/components/u8g2

include $(IDF_PATH)/make/project.mk
#include ./main/components/u8g2/component.mk

# Create a SPIFFS image from the contents of the 'spiffs_image' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'make flash'. 
SPIFFS_IMAGE_FLASH_IN_PROJECT := 1
$(eval $(call spiffs_create_partition_image,storage,atmega_hex))