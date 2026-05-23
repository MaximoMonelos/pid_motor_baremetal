#include <stdio.h>
#include "pico/stdlib.h"
#include "pid.h"
#include "motor.h"
#include "driver_encoder_optico.h"
#include "encoder_rot.h"

// -----------------------------------------------------------------------
// Pines y configuracion de hardware
// -----------------------------------------------------------------------
#define PIN_PWM         17
#define PWM_FREQ        15000
#define PIN_A_1         22
#define PIN_A_2         20
#define PIN_ENCODER     11
#define TICKS_ENCODER   20.0f
#define PIN_DT          13
#define PIN_CLK         12
#define PIN_SW          14

// -----------------------------------------------------------------------
// Loop principal
// -----------------------------------------------------------------------
// Con ENCODER_METHOD_TIMER/PWM_IC: el loop puede correr mas rapido que
// la ventana; encoder_get_freq() retorna false si aun no hay nueva muestra.
// Con ENCODER_METHOD_PIO: hay nueva muestra casi en cada iteracion.
#define LOOP_DELAY_MS   20u     // resolución del loop (menor que ENCODER_WINDOW_MS)

// -----------------------------------------------------------------------
// PID
// -----------------------------------------------------------------------
#define SETPOINT        945.0f  // RPM objetivo (80% PWM segun osciloscopio)
#define PID_TS          0.1f    // sampling time del PID en segundos

// -----------------------------------------------------------------------
// FIR (se mantiene igual que antes)
// -----------------------------------------------------------------------
#define NUM_TAPS        21
#define BUFFER_SIZE     NUM_TAPS
static float32_t fir_state[BLOCK_SIZE_FIR + NUM_TAPS - 1];

const float coef[NUM_TAPS] = {
    0.007352112425088f, 0.009421423151889f, 0.015426603040574f,
    0.024780055419825f, 0.036566341700744f, 0.049631796255090f,
    0.062697470097659f, 0.074484345011617f, 0.083838557840372f,
    0.089844373342472f, 0.091913843429340f, 0.089844373342472f,
    0.083838557840372f, 0.074484345011617f, 0.062697470097659f,
    0.049631796255090f, 0.036566341700744f, 0.024780055419825f,
    0.015426603040574f, 0.009421423151889f, 0.007352112425088f
};

// -----------------------------------------------------------------------
// Objetos globales
// -----------------------------------------------------------------------
enc_rot_t enc_rot;

enc_rot_conf_t enc_rot_conf = {
    .pin_clk = PIN_CLK,
    .pin_dt  = PIN_DT,
    .pin_sw  = PIN_SW,
};

encoder_config_t enc_config = {
    .coef     = coef,
    .num_taps = NUM_TAPS,
    .pin      = PIN_ENCODER,
    .ticks    = TICKS_ENCODER,
};

encoder_t enc = {};

motor_t motor_a = {
    .dir        = CLOCKWISE,
    .duty_cycle = 50.0f,
};

pid_ctrl_t pid = {
    .out_max       = 100.0f,
    .out_min       = 0.0f,
    .kd            = 0.0f,
    .ki            = 0.004874599f,
    .kp            = 0.020009f,
    .prev_error    = 0.0f,
    .sampling_time = PID_TS,
};

motor_config_t motor_conf = {
    .pin_a        = PIN_A_1,
    .pin_b        = PIN_A_2,
    .pin_pwm      = PIN_PWM,
    .frequency_hz = PWM_FREQ,
};

// -----------------------------------------------------------------------
// ISR unificada
// -----------------------------------------------------------------------
void master_callback(uint gpio, uint32_t events){
    if(gpio == PIN_ENCODER){
        encoder_isr(&enc);                  // solo actua en ENCODER_METHOD_TIMER
    } else if(gpio == PIN_CLK || gpio == PIN_SW){
        encoder_rot_isr(gpio, events);
    }
}

// -----------------------------------------------------------------------
// main
// -----------------------------------------------------------------------
void main(void)
{
    stdio_init_all();
    sleep_ms(3000);

#if defined(ENCODER_METHOD_TIMER)
    printf("# Metodo: TIMER HARDWARE (ventana %u ms)\n", ENCODER_WINDOW_MS);
#elif defined(ENCODER_METHOD_PWM_IC)
    printf("# Metodo: PWM INPUT CAPTURE (ventana %u ms)\n", ENCODER_WINDOW_MS);
#elif defined(ENCODER_METHOD_PIO)
    printf("# Metodo: PIO FRECUENCIMETRO\n");
#endif
    printf("# formato: freq_Hz, rpm_raw, rpm_filtrado\n");

    motor_config(&motor_a, &motor_conf);
    motor_set_lvl(&motor_a, 80);

    // Para metodos PWM_IC y PIO pasar NULL como isr (no se usa)
#if defined(ENCODER_METHOD_TIMER)
    encoder_init(&enc, &enc_config, fir_state, (void *)master_callback);
#else
    encoder_init(&enc, &enc_config, fir_state, NULL);
#endif

    while(true){
        bool new_sample = encoder_get_freq(&enc);

        if(new_sample){
            encoder_get_rpm_filtered(&enc);

            // Imprimir CSV para graficar facilmente
            printf("%.2f, %.2f, %.2f\n",
                   enc.freq,
                   enc.rpm_raw,
                   enc.rpm_filtered);

            // --- PID (descomentar cuando quieras cerrar el lazo) ---
            // pid_set_rpm(enc.rpm_filtered, SETPOINT, &pid);
            // motor_set_lvl(&motor_a, pid.last_output);
        }

        sleep_ms(LOOP_DELAY_MS);
    }
}
