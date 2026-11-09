#include "encoder_rot.h"

#define BUTTON_DEBOUNCE_US 20000u

static uint8_t read_state(const enc_rot_t *enc){
    return (gpio_get(enc->conf.pin_clk) ? 2u : 0u) | (gpio_get(enc->conf.pin_dt) ? 1u : 0u);
}

static void input_pin(uint8_t pin){
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

bool encoder_rot_config(enc_rot_t *enc, const enc_rot_conf_t *conf){
    
    if (enc == NULL || conf == NULL || conf->pin_clk >= NUM_BANK0_GPIOS ||
        conf->pin_dt >= NUM_BANK0_GPIOS || conf->pin_sw >= NUM_BANK0_GPIOS ||
        conf->pin_clk == conf->pin_dt || conf->pin_clk == conf->pin_sw ||
        conf->pin_dt == conf->pin_sw) {
        return false;
    }

    *enc = (enc_rot_t){.conf = *conf};
    input_pin(conf->pin_clk);
    input_pin(conf->pin_dt);
    input_pin(conf->pin_sw);
    enc->previous_state = read_state(enc);
    enc->button_raw = !gpio_get(conf->pin_sw);
    enc->down = enc->button_raw;
    enc->button_changed_us = time_us_32();
    enc->configured = true;
    return true;
}

void encoder_rot_update(enc_rot_t *enc)
{
    if (enc == NULL || !enc->configured) {
        return;
    }
    enc->step = 0;
    enc->pressed = false;

    // Los rebotes que recorren una transicion de ida y vuelta se cancelan.
    static const int8_t direction[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
    uint8_t state = read_state(enc);
    if ((state ^ enc->previous_state) == 3u) {
        enc->transitions = 0; // Salto invalido: no inventar un paso.
    } else {
        enc->transitions += direction[enc->previous_state * 4u + state];
    }
    enc->previous_state = state;

    if (enc->transitions >= 4 || enc->transitions <= -4) {
        enc->step = enc->transitions > 0 ? 1 : -1;
        enc->transitions = 0;
        if (enc->step > 0 && enc->counter < INT32_MAX) enc->counter++;
        if (enc->step < 0 && enc->counter > INT32_MIN) enc->counter--;
    }

    uint32_t now = time_us_32();
    bool down = !gpio_get(enc->conf.pin_sw);
    if (down != enc->button_raw) {
        enc->button_raw = down;
        enc->button_changed_us = now;
    }
    if (down != enc->down && (uint32_t)(now - enc->button_changed_us) >= BUTTON_DEBOUNCE_US) {
        enc->down = down;
        enc->pressed = down;
    }
}
