cmake_minimum_required(VERSION 3.5)

SET(XPCC_DIR ${CMAKE_SOURCE_DIR}/../xpcc/)
SET(PLATFORM "lpc11Uxx")
SET(PROJECT_NAME "SKYFalcon_Radio")
#SET(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/lpc11xx/linkscript.ld")
#SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")
#SET(LIBRARIES "${CMAKE_SOURCE_DIR}/../ardupilot/ArduCopter/libArduCopter.a") #additional libraries

SET(LINKER_FLASH_ORIGIN "0x00001000")
SET(LINKER_FLASH_LENGTH "28k")
SET(LINKER_RAM1_ORIGIN "0x100000C0")
SET(LINKER_RAM1_LENGTH "6k-0xC0")
SET(LINKER_RAM2_ORIGIN "0x10001800")
SET(LINKER_RAM2_LENGTH "2k")

set(SOURCES
    ./Axes.cpp
    ./xtea.cpp
    ./mavHandler.cpp
    ./rh_compat_11u.cpp
    ./blowfish.cpp
    ./remote_control.cpp
    ./main.cpp
    ./SwUart.cpp
    ./eeprom/eeprom.cpp
    ./lpc11xx/cmsis/system_LPC11xx.c
)

include(${XPCC_DIR}/xpccProject.cmake)

project(${PROJECT_NAME} CXX C)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-I")
set(CMAKE_INCLUDE_SYSTEM_FLAG_C "-I")

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND cp
                        ARGS "${PROJECT_NAME}.bin" "${PROJECT_NAME}.dfu"
                        COMMAND dfu-suffix
                        ARGS    -v ffff -p 12c9 -a "${PROJECT_NAME}.dfu"
                        COMMENT "Generating DFU Image"
                        VERBATIM)
                        
