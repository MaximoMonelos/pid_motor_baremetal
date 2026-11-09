cmake_minimum_required(VERSION 3.13)
project(setpoint_serial_tests C)
set(CMAKE_C_STANDARD 11)
enable_testing()
add_executable(setpoint_serial_test setpoint_serial_test.c
    ../../setpoint_serial.c ../../setpoint_gen.c)
target_include_directories(setpoint_serial_test PRIVATE stubs ../..)
target_compile_options(setpoint_serial_test PRIVATE -Wall -Wextra -Werror)
add_test(NAME setpoint_serial_test COMMAND setpoint_serial_test)
