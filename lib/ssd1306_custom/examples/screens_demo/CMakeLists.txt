# Build via the repository root. The original motor_demo remains available.
add_executable(screens_demo main.c)
target_link_libraries(screens_demo PRIVATE ssd1306_screens)
target_compile_options(screens_demo PRIVATE -Wall -Wextra -Werror)
set_target_properties(screens_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
pico_enable_stdio_usb(screens_demo 1)
pico_enable_stdio_uart(screens_demo 0)
pico_add_extra_outputs(screens_demo)
