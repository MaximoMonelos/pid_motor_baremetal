#include "task_time.h"
#include <stddef.h>

bool task_time_init(task_time_t *task, uint32_t period_ms){

    if(task == NULL){
        return false;
    }

    task->period_ms = period_ms;
    task->dt_s = 0.0f;
    task->last_time = get_absolute_time();
    task->next_time = delayed_by_ms(task->last_time, period_ms);

    return period_ms > 0;
}

bool task_time_ready(task_time_t *task){

    if(task == NULL || task->period_ms == 0){
        return false;
    }

    absolute_time_t now = get_absolute_time();

    if(absolute_time_diff_us(now, task->next_time) > 0){
        return false;
    }

    task->dt_s = (float)absolute_time_diff_us(task->last_time, now) / 1000000.0f;

    task->last_time = now;

    uint64_t period_us = (uint64_t)task->period_ms * 1000u;

    uint64_t delay_us = (uint64_t)absolute_time_diff_us(task->next_time, now);

    uint64_t missed_periods = delay_us / period_us;
    uint64_t advance_us = (missed_periods + 1u) * period_us;
    task->next_time = delayed_by_us(task->next_time, advance_us);

    return true;
}