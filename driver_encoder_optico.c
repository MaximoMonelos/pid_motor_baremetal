#include "driver_encoder_optico.h"
#include "pico/stdlib.h"

#define US_TO_MS        1000.0f
#define MS_TO_S         1000.0f
#define SEG_TO_MIN      60.0f
#define BLOCK_SIZE      1

void encoder_init(encoder_t *enc, encoder_config_t *conf_enc, float *fir_state, void *isr){

    enc->config = *conf_enc;

    if(enc->config.pin <= 25 && enc->config.pin >= 23) return;

    if(enc->config.pin > 29) return;

    gpio_init(enc->config.pin);
    gpio_set_dir(enc->config.pin, GPIO_IN);
    gpio_pull_up(enc->config.pin);

    enc->internal.counter_pulses = 0;
    enc->internal.last_time_us = time_us_32();
    enc->internal.counter_pulses = 0;
    enc->internal.last_pulses = 0;
    enc->internal.fir_state = fir_state;


    arm_fir_init_f32(&(enc->internal.fir),
                    enc->config.num_taps,
                    enc->config.coef,
                    enc->internal.fir_state,
                    BLOCK_SIZE);

    gpio_set_irq_enabled_with_callback(enc->config.pin, GPIO_IRQ_EDGE_RISE, true, isr);
}

void encoder_get_freq(encoder_t *enc){

    float actual_time_us = time_us_32();
    float actual_pulses = enc->internal.counter_pulses;

    float delta_tiempo_us = actual_time_us - enc->internal.last_time_us;
    float delta_pulses = actual_pulses - enc->internal.last_pulses;

    enc->internal.last_time_us = actual_time_us;
    enc->internal.last_pulses = actual_pulses;

    if(delta_tiempo_us > 0){
        enc->freq = ((float)delta_pulses * US_TO_MS * MS_TO_S) / delta_tiempo_us;
        return;
    }
    enc->freq = 0.0f;
}

void encoder_get_rpm_raw(encoder_t *enc){
    if(enc->freq > 0){   // 10 pulsos = 1 rev 1 min = 60s
        enc->rpm_raw = enc->freq / enc->config.ticks * SEG_TO_MIN;
        return;
    } 
    enc->rpm_raw = 0.0f;
    return;
}

void encoder_get_rpm_filtered(encoder_t *enc){
    if(enc->freq > 0){   // 10 pulsos = 1 rev 1 min = 60s
        enc->rpm_raw = enc->freq / enc->config.ticks * SEG_TO_MIN;
    }else{
        enc->rpm_raw = 0.0f;
    }
    arm_fir_f32(&(enc->internal.fir), &enc->rpm_raw, &enc->rpm_filtered, BLOCK_SIZE);
}