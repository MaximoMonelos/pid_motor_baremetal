#include "encoder_rot.h"
#include <assert.h>
#include <stdio.h>

static bool pins[30];
static uint32_t now;
static unsigned configured_pins;

void gpio_init(unsigned pin) { assert(pin < 30); configured_pins++; }
void gpio_set_dir(unsigned pin, bool output) { assert(pin < 30 && !output); }
void gpio_pull_up(unsigned pin) { assert(pin < 30); }
bool gpio_get(unsigned pin) { assert(pin < 30); return pins[pin]; }
uint32_t time_us_32(void) { return now; }

static void sample(enc_rot_t *enc, unsigned state)
{
    pins[9] = (state & 2u) != 0;
    pins[8] = (state & 1u) != 0;
    now += 1000;
    encoder_rot_update(enc);
}

int main(void)
{
    for (unsigned i = 0; i < 30; i++) pins[i] = true;
    enc_rot_t enc = {0};
    enc_rot_conf_t conf = {.pin_clk=9, .pin_dt=8, .pin_sw=7};
    assert(encoder_rot_config(&enc, &conf));
    assert(configured_pins == 3 && enc.counter == 0 && !enc.pressed);
    sample(&enc, 1); sample(&enc, 0); sample(&enc, 2); sample(&enc, 3);
    assert(enc.counter == 1 && enc.step == 1);
    sample(&enc, 3); assert(enc.step == 0 && enc.counter == 1);
    sample(&enc, 2); sample(&enc, 0); sample(&enc, 1); sample(&enc, 3);
    assert(enc.counter == 0 && enc.step == -1);
    // Contact bounce reverses a partial transition; only one complete step remains.
    sample(&enc, 1); sample(&enc, 3); sample(&enc, 1);
    sample(&enc, 0); sample(&enc, 2); sample(&enc, 3);
    assert(enc.counter == 1 && enc.step == 1);
    sample(&enc, 0); sample(&enc, 3); // Invalid two-bit jumps cannot count.
    assert(enc.counter == 1 && enc.step == 0);
    pins[7] = false; encoder_rot_update(&enc);
    now += 10000; pins[7] = true; encoder_rot_update(&enc);
    now += 10000; pins[7] = false; encoder_rot_update(&enc);
    now += 19000; encoder_rot_update(&enc); assert(!enc.pressed && !enc.down);
    now += 1000; encoder_rot_update(&enc); assert(enc.pressed && enc.down);
    now += 100000; encoder_rot_update(&enc); assert(!enc.pressed && enc.down);
    pins[7] = true; encoder_rot_update(&enc);
    now += 20000; encoder_rot_update(&enc); assert(!enc.down && !enc.pressed);
    // Unsigned time differences still debounce correctly across timer wrap.
    now = UINT32_MAX - 10000; pins[7] = false; encoder_rot_update(&enc);
    now += 20000; encoder_rot_update(&enc); assert(enc.pressed && enc.down);
    enc_rot_t other = {0};
    enc_rot_conf_t invalid = conf; invalid.pin_dt = invalid.pin_clk;
    assert(!encoder_rot_config(&other, &invalid));
    assert(!encoder_rot_config(NULL, &conf));
    assert(!encoder_rot_config(&other, NULL));
    encoder_rot_update(NULL); encoder_rot_update(&other);
    puts("PASS: polling, directions, contact bounce, invalid jumps, button and timer wrap");
    return 0;
}
