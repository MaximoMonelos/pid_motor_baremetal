#include "pico/stdlib.h"
#include "encoder_rot.h"


enc_rot_event_t rot_event;

static enc_rot_t *instancia_encoder = NULL;
static absolute_time_t last_interrupt_time;

void gpio_output_conf(uint8_t pin){
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void encoder_rot_config(enc_rot_t *enc, enc_rot_conf_t *enc_conf, void *isr){

    enc->conf = *enc_conf;
    enc->counter = 0;

    instancia_encoder = enc;

    if( enc->conf.pin_clk  <= 25 && enc->conf.pin_clk  >= 23) return;

    if(enc->conf.pin_clk  > 29) return;

    gpio_output_conf(enc->conf.pin_clk);
    gpio_output_conf(enc->conf.pin_dt);
    gpio_output_conf(enc->conf.pin_sw);    

    gpio_set_irq_enabled_with_callback(enc->conf.pin_clk, GPIO_IRQ_EDGE_FALL, true, isr);
}

void encoder_rot_isr(uint gpio, uint32_t events){
    if (instancia_encoder == NULL) return;

    uint32_t current_time = time_us_32();
    
    // Si pasaron menos de 10ms (10000 us), es ruido. ¡Afuera!
    if (current_time - last_interrupt_time < 10000) {
        return;
    }
    last_interrupt_time = current_time;

    if (gpio == instancia_encoder->conf.pin_clk){
        if(gpio_get(instancia_encoder->conf.pin_dt)){
        instancia_encoder->event = ENC_CLOCKWISE;
        instancia_encoder->counter++;
    } else {
        instancia_encoder->event = ENC_CCLOCKWISE;
        instancia_encoder->counter--;
        }
    }
}