#ifndef ENCODER_ROT_H
#define ENCODER_ROT_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pico/stdlib.h"


typedef struct{
    uint8_t pin_dt;
    uint8_t pin_clk;
    uint8_t pin_sw;
} enc_rot_conf_t;

typedef enum{
    ENC_CLOCKWISE,
    ENC_CCLOCKWISE,
    ENC_BTN,
} enc_rot_event_t;

typedef struct {
    enc_rot_conf_t conf;
    enc_rot_event_t event;
    int counter;

} enc_rot_t;


void encoder_rot_config(enc_rot_t *enc, enc_rot_conf_t *enc_conf, void *isr);

void encoder_rot_isr(uint gpio, uint32_t events);

void gpio_output_conf(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_ROT