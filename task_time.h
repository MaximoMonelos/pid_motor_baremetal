#ifndef TASK_TIME_H
#define TASK_TIME_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/time.h"

typedef struct {
    uint32_t period_ms;
    absolute_time_t last_time;
    absolute_time_t next_time;
    float dt_s;
} task_time_t;

bool task_time_init(task_time_t *task, uint32_t period_ms);
bool task_time_ready(task_time_t *task);

#endif