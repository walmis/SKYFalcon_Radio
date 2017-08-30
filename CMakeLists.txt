cmake_minimum_required(VERSION 3.5)

SET(XPCC_DIR ${CMAKE_SOURCE_DIR}/../xpcc/)
SET(PLATFORM "lpc11Uxx")
SET(PROJECT_NAME "SKYFalcon_Radio")
#SET(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/lpc11xx/linkscript.ld")
#SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")
#SET(LIBRARIES "${CMAKE_SOURCE_DIR}/../ardupilot/ArduCopter/libArduCopter.a") #additional libraries
SET(XPCC_CMAKE_DEFINES "-DCHIBI_RTOS=1")

SET(LINKER_FLASH_ORIGIN "0x00001000")
SET(LINKER_FLASH_LENGTH "28k")
SET(LINKER_RAM1_ORIGIN "0x100000C0")
SET(LINKER_RAM1_LENGTH "8k-0xC0-512")
SET(LINKER_RAM2_ORIGIN "0x10001E00")
SET(LINKER_RAM2_LENGTH "512")
SET(LINKER_PROCESS_STACK_SIZE "256")
SET(LINKER_MAIN_STACK_SIZE "384")

SET(CHIBI_RTOS TRUE)

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

add_subdirectory(chibi)

add_custom_command(TARGET ${PROJECT_NAME}  POST_BUILD
                        COMMAND cp
                        ARGS "${PROJECT_NAME}.bin" "${PROJECT_NAME}.dfu"
                        COMMAND dfu-suffix
                        ARGS    -v ffff -p 12c9 -a "${PROJECT_NAME}.dfu"
                        COMMENT "Generating DFU Image"
                        VERBATIM)
