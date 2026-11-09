#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#define PICO_ERROR_TIMEOUT (-1)
int getchar_timeout_us(uint32_t timeout_us);
