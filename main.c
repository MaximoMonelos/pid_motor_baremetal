#include <stdio.h>
#include "pico/stdlib.h"
#include "pid.h"
#include "motor.h"
#include "driver_encoder_optico.h"

#define PIN_PWM         16
#define PWM_FREQ        15000
#define CLK_DIV         1
#define PWM_WRAP        3750
//----Motor A---
#define PIN_A_1         22
#define PIN_A_2         20
#define PIN_ENCODER     10
#define TICKS_ENCODER   10.0f

#define SAMPLE_RATE_MS  100u
#define BUFFER_SIZE     21
#define NUM_TAPS        BUFFER_SIZE
#define BLOCK_SIZE      1
#define MS_TO_S         1000.0f
#define SETPOINT        500.0f
#define DEADBAND        10


static float32_t fir_state[BLOCK_SIZE + NUM_TAPS - 1];

const float coef[BUFFER_SIZE] = {
    0.007352112425088f, 0.009421423151889f, 0.015426603040574f, 0.024780055419825f, 0.036566341700744f, 0.049631796255090f, 
    0.062697470097659f, 0.074484345011617f, 0.083838557840372f, 0.089844373342472f, 0.091913843429340f, 0.089844373342472f,
    0.083838557840372f, 0.074484345011617f, 0.062697470097659f, 0.049631796255090f, 0.036566341700744f, 0.024780055419825f,
    0.015426603040574f, 0.009421423151889f, 0.007352112425088f
};

arm_fir_instance_f32 fir;

encoder_config_t enc_config = {
    .coef = coef,
    .num_taps = NUM_TAPS,
    .pin = PIN_ENCODER,
    .ticks = TICKS_ENCODER,
};

encoder_t enc = {};

motor_t motor_a = {
    .dir = CLOCKWISE,
    .duty_cycle = 50.0f,
};


pid_ctrl_t pid = {
    .out_max = 100.0f,
    .out_min = 0.0f,
    .kd = 0.0f,
    .ki= 0.004874599,
    .kp = 0.020009f,
    .prev_error = 0,
    .sampling_time = 0.1f,
};

motor_config_t motor_conf = {
    .pin_a = PIN_A_1,
    .pin_b = PIN_A_2,
    .pin_pwm = PIN_PWM,
    .frequency_hz = PWM_FREQ,
};

void isr_encoder(uint gpio, uint32_t events){
    if(gpio == PIN_ENCODER){
        enc.internal.counter_pulses++;
    }
}

void main()
{   
    float error;
    float pwm = 0;
    stdio_init_all();
    sleep_ms(5000);
    motor_config(&motor_a, &motor_conf);

    motor_set_lvl(&motor_a, 0);

    encoder_init(&enc, &enc_config, fir_state, (void *)isr_encoder);

    while (true){
        
        encoder_get_freq(&enc);
        encoder_get_rpm_filtered(&enc);
        // encoder_get_rpm_raw(&enc);
        // printf("%.2f\n", enc.rpm_filtered);
        pid_set_rpm(enc.rpm_filtered, SETPOINT, &pid);
        motor_set_lvl(&motor_a, pid.last_output);
        printf("%.2f, %.2f, %.2f, %.2f\n", enc.rpm_filtered, pid.current_error, SETPOINT, pid.last_output);

        sleep_ms(SAMPLE_RATE_MS);
    }
}