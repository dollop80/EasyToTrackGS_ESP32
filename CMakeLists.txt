# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS ../../ ./main/components/u8g2 ./main/components/protocols ./main/components/wifi-manager ./main/components/avr-isp ./main/components/tpl0401x)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ett-esp32)
