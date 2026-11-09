cmake_minimum_required(VERSION 3.13)
# Can be built standalone or through the repository root.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(ssd1306_motor_demo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()
set(SSD1306_BUILD_MOTOR_DEMO OFF CACHE BOOL "" FORCE)
set(SSD1306_BUILD_SCREENS_DEMO OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../.. ssd1306_lib)
endif()

set(OLED_SDA_PIN 4 CACHE STRING "I2C0 SDA GPIO")
set(OLED_SCL_PIN 1 CACHE STRING "I2C0 SCL GPIO")
set(OLED_ADDRESS 0x3C CACHE STRING "OLED 7-bit I2C address")
add_executable(motor_demo main.c demo_ui.c)
# Keep the same output path for Pico extension tasks and direct builds.
set_target_properties(motor_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_link_libraries(motor_demo PRIVATE ssd1306)
target_compile_definitions(motor_demo PRIVATE
    OLED_SDA_PIN=${OLED_SDA_PIN}
    OLED_SCL_PIN=${OLED_SCL_PIN}
    OLED_ADDRESS=${OLED_ADDRESS})
target_compile_options(motor_demo PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_usb(motor_demo 1)
pico_enable_stdio_uart(motor_demo 0)
pico_add_extra_outputs(motor_demo)
