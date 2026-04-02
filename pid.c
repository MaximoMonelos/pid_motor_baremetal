#include <stdio.h>
#include "pico/stdlib.h"
#include "pid.h"
// #include "motor.h"
#include "dsp/filtering_functions.h"
#include "arm_math.h"
#include "driver_encoder_optico.h"

#define DEADBAND        10

/**
 * @brief Calcula la salida del PID basada en el error actual.
 * @param pid Puntero a la estructura de contexto.
 * @param error Diferencia entre Setpoint y Process Variable.
 * @return float Salida calculada y limitada (Clamped).
 */
float pid_calculate(pid_ctrl_t *pid, float error){

    float p = pid->kp * error;

    pid->integral += error;
    
    float i = pid->integral * pid->ki;


    float d = pid->kd * ((error - pid->prev_error) / pid->sampling_time);
    // printf("P: %.2f, D: %.2f, I: %.2f, sampling: %.2f\n", p, d, i, pid->sampling_time);
    pid->prev_error = error;

    float output = p + i + d;
    // printf("OUTPUT: %.2f\n", output);
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;
    return output;
}

float pid_set_rpm(float rpm_value, float set_point, pid_ctrl_t *pid){
    pid->current_error = set_point - rpm_value;

    if(pid->current_error < 2.0f && pid->current_error > -2.0f){
        pid->deadband_counter++;
        if (pid->deadband_counter > DEADBAND){
            return pid->last_output;
        }
    } else{
        pid->deadband_counter = 0;
    }
    
    pid->last_output = pid_calculate(pid, pid->current_error);
    return pid->last_output;
    
}

/**
 * @brief Reinicia los términos históricos del PID (Integral y error previo).
 */
void pid_reset(pid_ctrl_t *pid){
    if(pid != NULL){
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->deadband_counter = 0.0f;
    }
}