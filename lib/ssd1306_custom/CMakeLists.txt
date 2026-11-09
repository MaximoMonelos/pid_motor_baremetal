cmake_minimum_required(VERSION 3.13)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(ssd1306_custom C CXX ASM)
pico_sdk_init()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(SSD1306_DEMO_DEFAULT ON)
else()
    set(SSD1306_DEMO_DEFAULT OFF)
endif()

if(NOT TARGET pico_stdlib OR NOT TARGET hardware_i2c)
    message(FATAL_ERROR "Initialize the Pico SDK with pico_sdk_init() before adding ssd1306_custom.")
endif()

# Reusable targets. Importing this folder does not initialize the parent's SDK,
# select its board/toolchain, change its language standard, or build demos.
add_library(ssd1306 STATIC src/ssd1306.c src/ssd1306_widgets.c)
target_include_directories(ssd1306 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(ssd1306 PUBLIC c_std_11)
target_link_libraries(ssd1306 PUBLIC pico_stdlib hardware_i2c)
add_library(ssd1306::core ALIAS ssd1306)

add_library(ssd1306_screens STATIC src/ssd1306_screens.c)
target_link_libraries(ssd1306_screens PUBLIC ssd1306)
add_library(ssd1306::screens ALIAS ssd1306_screens)

# Convenient single entry point for users who want the complete library.
add_library(ssd1306_custom INTERFACE)
target_link_libraries(ssd1306_custom INTERFACE ssd1306_screens)
add_library(ssd1306::all ALIAS ssd1306_custom)

# The demo is enabled when this repository is opened as a Pico project.
# Parent projects can continue linking only the libraries.
option(SSD1306_BUILD_MOTOR_DEMO "Build the simulated motor OLED demo" ${SSD1306_DEMO_DEFAULT})
if(SSD1306_BUILD_MOTOR_DEMO)
    add_subdirectory(examples/motor_demo)
endif()

option(SSD1306_BUILD_SCREENS_DEMO "Build the optional high-level screen demo" ${SSD1306_DEMO_DEFAULT})
if(SSD1306_BUILD_SCREENS_DEMO)
    add_subdirectory(examples/screens_demo)
endif()
