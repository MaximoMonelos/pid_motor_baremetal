#ifndef DRIVER_ENCODER_OPTICO_H
#define DRIVER_ENCODER_OPTICO_H

#include <stdint.h>
#include "arm_math.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

// ============================================================
// SELECTOR DE METODO — cambiar este define para testear
// ============================================================
//   ENCODER_METHOD_TIMER    -> Metodo 1: timer hardware exacto
//   ENCODER_METHOD_PWM_IC   -> Metodo 2: PWM input capture (sin ISR)
//   ENCODER_METHOD_PIO      -> Metodo 3: PIO frecuencimetro (maxima precision)
// ============================================================
#define ENCODER_METHOD_TIMER

// Ventana de integracion para metodos 1 y 2 (en ms)
// Tradeoff: mayor ventana = menor cuantizacion, mayor latencia
//   50ms  -> +-10Hz = +-30 RPM  (rapido, para PID)
//   100ms -> +-5Hz  = +-15 RPM  (balance)
//   200ms -> +-2.5Hz= +-7.5 RPM (suave, para display)
#define ENCODER_WINDOW_MS       100u

// FIR
#define BLOCK_SIZE_FIR          1

// PIO: instancia y state machine (solo para ENCODER_METHOD_PIO)
#define ENCODER_PIO             pio0
#define ENCODER_PIO_SM          0

// ------------------------------------------------------------
typedef struct {
    const float    *coef;
    uint32_t        num_taps;
    uint32_t        pin;
    float           ticks;
} encoder_config_t;

typedef struct {
    // --- comun ---
    volatile uint32_t       counter_pulses;
    float                  *fir_state;
    arm_fir_instance_f32    fir;

    // --- metodo 1: timer hardware ---
    volatile uint32_t       snap_pulses;        // captura atomica al vencer alarm
    volatile uint32_t       last_snap_pulses;
    volatile bool           timer_fired;

    // --- metodo 2: PWM input capture ---
    uint                    pwm_slice;
    uint32_t                pwm_last_count;
    uint32_t                pwm_last_time_us;

    // --- metodo 3: PIO ---
    PIO                     pio;
    uint                    pio_sm;
    uint                    pio_offset;

} encoder_internal_t;

typedef struct {
    encoder_config_t        config;
    encoder_internal_t      internal;
    float                   freq;           // Hz (igual que osciloscopio)
    float                   rpm_raw;
    float                   rpm_filtered;
} encoder_t;

// ------------------------------------------------------------
// API
// ------------------------------------------------------------
void encoder_init            (encoder_t *enc, encoder_config_t *conf_enc,
                               float *fir_state, void *isr);
bool encoder_get_freq        (encoder_t *enc);   // true = nueva muestra
void encoder_get_rpm_raw     (encoder_t *enc);
void encoder_get_rpm_filtered(encoder_t *enc);
void encoder_isr             (encoder_t *enc);   // llamar desde master_callback

#endif // DRIVER_ENCODER_OPTICO_H
