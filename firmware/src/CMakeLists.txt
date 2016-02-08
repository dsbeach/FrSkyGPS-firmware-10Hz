#    FrSkyGPS 10Hz firmware
#    Copyright (C) 2016  David S. Beach
#    mailto:david.s.beach@gmail.com

cmake_minimum_required(VERSION 3.4)

include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0)

CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_VERBOSE_MAKEFILE ON)

#custom command to use objcopy to create .hex files out of ELF files
function(make_hex INPUT)
              add_custom_command(TARGET ${INPUT}.elf
                      COMMAND arm-none-eabi-objcopy -O ihex ${INPUT}.elf ${INPUT}.hex
                      COMMENT "objcopying to make firmware .hex file")
              set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${INPUT}_.hex)
endfunction(make_hex)
 

project (FrSkyGPS_10Hx_firmware)

set(HSE_VALUE 48000000)
add_definitions(-DSTM32F051)
add_definitions(-DUSE_STDPERIPH_DRIVER)

set(FIRMWARE_SRC ${FIRMWARE_SRC}
  FrSkySportSensor.cpp
  FrSkySportSensorGps.cpp
  GPSController.cpp
  main.cpp
  SPortController.cpp
  Timer.cpp
  TinyGPS++.cpp
  UBlox7.cpp
  ../system/src/cmsis/system_stm32f0xx.c
  ../system/src/cmsis/vectors_stm32f0xx.c
  ../system/src/cortexm/_initialize_hardware.c
  ../system/src/cortexm/_reset_hardware.c
  ../system/src/cortexm/exception_handlers.c
  ../system/src/diag/trace_impl.c
  ../system/src/diag/Trace.c
  ../system/src/newlib/_cxx.cpp
  ../system/src/newlib/_exit.c
  ../system/src/newlib/_sbrk.c
  ../system/src/newlib/_startup.c
  ../system/src/newlib/_syscalls.c
  ../system/src/newlib/assert.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_dma.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_gpio.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_misc.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_rcc.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_tim.c
  ../system/src/stm32f0-stdperiph/stm32f0xx_usart.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../system/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../system/include/cmsis)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../system/include/stm32f0-stdperiph)

set(CMAKE_LINKER_DIRECTORY ${CMAKE_LINKER_DIRECTORY} ${CMAKE_CURRENT_SOURCE_DIR}/../ldscripts/)


set(CMAKE_CXX_FLAGS "-mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra -g -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics")

set(CMAKE_C_FLAGS "-mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra -g -std=gnu11 -fabi-version=0 -fno-exceptions ")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -Wall -Wextra  -g -T mem.ld -T libs.ld -T sections.ld -nostartfiles -Xlinker --gc-sections -L${CMAKE_LINKER_DIRECTORY} --specs=nano.specs")

add_executable(FrSkyGPS_10Hz_firmware.elf ${FIRMWARE_SRC})
make_hex(FrSkyGPS_10Hz_firmware)