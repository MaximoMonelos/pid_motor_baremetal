#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "logging.h"

#define SYS_CLK_FREQ    150000000
#define WRAP_ESTABLECIDO     4095

#define FREQ_MAX            36000 //Valor de frecuencia máx que se puede solicitar al ser para motor DC

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CHECK(func) do { \
    motor_error_t _res = (func); \
    if (_res <= 0) return _res; \
} while (0)


typedef enum{ 
    CLOCKWISE,
    COUNTERCLOCKWISE,
    STOP
} dir_t;

typedef enum {
    MOTOR_ERROR_PIN_INVALIDO = -7,
    MOTOR_ERROR_PIN_RESERVADO,
    MOTOR_ERROR_PUNTERO_NULO,
    MOTOR_ERROR_PWM_INVALIDO,
    MOTOR_ERROR_DIRECCION_INVALIDA,
    MOTOR_ERROR_DIV_INVALIDO,
    MOTOR_ERROR_SLICE_INVALIDO,
    MOTOR_ERROR = 0,
    MOTOR_SUCCESS = 1,
    MOTOR_WARNING_DUTY_CYCLE_SUPERADO,
    MOTOR_WARNING_FREQ_MAX_SUPERADA,
    MOTOR_WARNING_DIV_MAX_SUPERADO,
    
} motor_error_t;

typedef struct {
    uint16_t wrap_value;      
    float clk_div; 
    uint8_t slice;
    uint16_t pwm_value;
    uint16_t pwm_slice;   
} pwm_internal_config_t;

typedef struct {
    uint32_t frequency_hz;
    uint8_t pin_pwm;
    uint8_t pin_a;
    uint8_t pin_b;
} motor_config_t;

typedef struct{
    dir_t dir;
    float duty_cycle;

    motor_config_t motor_config;

    pwm_internal_config_t pwm_internal_config;
} motor_t;

motor_error_t config_gpio_pwm(uint8_t pin);
motor_error_t config_gpio_output(uint8_t pin);
motor_error_t motor_config(motor_t *m, const motor_config_t *motor_config);
motor_error_t motor_set_lvl(motor_t *m, float duty_cycle);
motor_error_t motor_set_dir(motor_t *m, dir_t dir);
motor_error_t motor_raise_error(const char* context, motor_error_t res);
const char* motor_strerror(motor_error_t res);
motor_error_t calculate_pwm_parameters(uint32_t *freq, pwm_internal_config_t *internal);
motor_error_t validate_duty_cycle(float *duty_cycle);
motor_error_t pwm_set_config(motor_t *m);
motor_error_t motor_pwm_set_enabled(uint8_t slice, bool estado);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H


