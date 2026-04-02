#ifndef DRIVER_ENCODER_OPTICO_H
#define DRIVER_ENCODER_OPTICO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"
#include "dsp/filtering_functions.h"
#include "arm_math.h"

typedef struct {
    uint8_t pin;
    float ticks;
    const float* coef;
    uint16_t num_taps;
} encoder_config_t;


typedef struct {
    volatile uint32_t counter_pulses;
    uint32_t last_time_us;
    uint32_t last_pulses;
    arm_fir_instance_f32 fir;
    float *fir_state; 
} encoder_internal_t;


typedef struct {
    encoder_config_t config;
    encoder_internal_t internal;
    float freq;
    float rpm_raw;
    float rpm_filtered;
} encoder_t;

void encoder_get_rpm_filtered(encoder_t *enc);
void encoder_get_rpm_raw(encoder_t *enc);
void encoder_get_freq(encoder_t *enc);
void encoder_init(encoder_t *enc, encoder_config_t *conf_enc, float *fir_state, void *isr);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_ENCODER_OPTICO_H
