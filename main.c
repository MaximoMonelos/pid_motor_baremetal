#include <stdio.h>
#include "pico/stdlib.h"
#include "pid.h"
#include "motor.h"
#include "driver_encoder_optico.h"
#include "encoder_rot.h"
#include "setpoint_gen.h"
#include "setpoint_serial.h"
#include "task_time.h"

#define PIN_PWM         13
#define PWM_FREQ        15000
#define CLK_DIV         1
#define PWM_WRAP        3750
//----Motor A---
#define PIN_A_1         15
#define PIN_A_2         14
#define PIN_ENCODER     16
#define TICKS_ENCODER   25.0f

#define SAMPLE_RATE_MS  50u
// #define BUFFER_SIZE     21
#define BUFFER_SIZE     15
#define NUM_TAPS        BUFFER_SIZE
#define BLOCK_SIZE      1
#define MS_TO_S         1000.0f

#define SERIAL_RATE_MS  5u
#define LOG_RATE_MS     50u

#define DEADBAND        10

static float32_t fir_state[BLOCK_SIZE + NUM_TAPS - 1];
// ----------------------------------------------------------------------------------
// -------------------------------AGREGAR ANTIWINDUP---------------------------------
// ----------------------------------------------------------------------------------


// const float coef[BUFFER_SIZE] = {
//     0.007352112425088f, 0.009421423151889f, 0.015426603040574f, 0.024780055419825f, 0.036566341700744f, 0.049631796255090f, 
//     0.062697470097659f, 0.074484345011617f, 0.083838557840372f, 0.089844373342472f, 0.091913843429340f, 0.089844373342472f,
//     0.083838557840372f, 0.074484345011617f, 0.062697470097659f, 0.049631796255090f, 0.036566341700744f, 0.024780055419825f,
//     0.015426603040574f, 0.009421423151889f, 0.007352112425088f
// };

const float coef[BUFFER_SIZE] = {
    0.010334253955694f, 0.016287375789618f, 0.032962516319300f, 0.057140845242106f, 0.084059972474792f, 0.108371465577169f, 
    0.125222710691704f, 0.131241719899237f, 0.125222710691704f, 0.108371465577169f, 0.084059972474792f, 0.057140845242106f, 
    0.032962516319300f, 0.016287375789618f, 0.010334253955694f
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

setpoint_gen_config_t sp_config = {
    .mode = SETPOINT_MODE_RAMP,
    .target_value = 500.0f,
    .ramp_rate = 50.0f,      // 100 RPM por segundo
};

setpoint_gen_t sp_gen;

void isr_encoder(uint gpio, uint32_t events){
    if(gpio == PIN_ENCODER){
        enc.internal.counter_pulses++;
    }
}

int main(){
       
    stdio_init_all();
    sleep_ms(5000);
    motor_config(&motor_a, &motor_conf);
    motor_set_lvl(&motor_a, 0);

    encoder_init(&enc, &enc_config, fir_state, (void *)isr_encoder);
    setpoint_gen_init(&sp_gen, &sp_config);

    task_time_t control_time = {0};
    task_time_t serial_time = {0};
    task_time_t log_time = {0};

    if(!task_time_init(&control_time, SAMPLE_RATE_MS) || !task_time_init(&serial_time, SERIAL_RATE_MS) || !task_time_init(&log_time, LOG_RATE_MS)){
        return 1;
    }



    // motor_set_lvl(&motor_a, 50);

    while (true){

        // 1. Tarea de Control (50ms)
        if(task_time_ready(&control_time)){
            encoder_get_freq(&enc);
            encoder_get_rpm_filtered(&enc);

            setpoint_gen_update(&sp_gen, control_time.dt_s);

            pid_set_rpm(enc.rpm_filtered, sp_gen.current_value, &pid);
            motor_set_lvl(&motor_a, pid.last_output);
        }

        // 2. Tarea Serial (5ms)
        if (task_time_ready(&serial_time)) {
            setpoint_serial_update(&sp_gen);
        }

        // 3. Tarea de Log (50ms)
        if (task_time_ready(&log_time)) {
            printf("%.2f, %.2f,%.2f, %.2f\n", 
                   enc.rpm_filtered, 
                   sp_gen.current_value,
                   pid.current_error,
                   pid.last_output);
            // printf("%.2f, %.2f\n", enc.rpm_raw, enc.freq);            
        }
    }

    return 0;
}
