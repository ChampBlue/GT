cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(m10_project C CXX ASM)
pico_sdk_init()

#BOARD_CONFIG_H
add_library(board_cfg INTERFACE)
target_include_directories(board_cfg INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/board/include
)

#LED
add_library(led STATIC drivers/led/led.c)
target_include_directories(led PUBLIC ${CMAKE_CURRENT_LIST_DIR}/drivers/led)
target_link_libraries(led PUBLIC board_cfg hardware_gpio)

#MOTOR
add_library(motor STATIC drivers/motor/motor.c)
target_include_directories(motor PUBLIC ${CMAKE_CURRENT_LIST_DIR}/drivers/motor)
target_link_libraries(motor PUBLIC board_cfg hardware_gpio )

#BOARD
add_library(board STATIC board/src/board_init.c)
target_link_libraries(board PUBLIC pico_stdio hardware_clocks hardware_timer board_cfg led motor uart)

#main
add_executable(m10 main.c)
target_link_libraries(m10 board pico_stdlib m)

pico_enable_stdio_usb(m10 0)
pico_enable_stdio_uart(m10 1)
pico_add_extra_outputs(m10)