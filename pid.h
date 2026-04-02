#include <stdint.h>
#include "pico/stdlib.h"

/**
 * @brief Estructura de configuración y estado del PID.
 */
typedef struct {
    // Ganancias (Parameters)
    float kp;
    float ki;
    float kd;
    // Límites (Constraints)
    float out_min;
    float out_max;

    float current_error;
    float last_output;
    uint8_t deadband_counter;

    // Estado interno (Context)
    float integral;
    float prev_error;
    float sampling_time; // dt en segundos
} pid_ctrl_t;

float pid_calculate(pid_ctrl_t *pid, float error);

void pid_reset(pid_ctrl_t *pid);

float pid_set_rpm(float rpm_value, float set_point, pid_ctrl_t *pid);