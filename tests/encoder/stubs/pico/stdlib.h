#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define GPIO_IN 0
#define NUM_BANK0_GPIOS 30
void gpio_init(unsigned pin);
void gpio_set_dir(unsigned pin, bool output);
void gpio_pull_up(unsigned pin);
bool gpio_get(unsigned pin);
uint32_t time_us_32(void);
