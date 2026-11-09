/* OLED 128x64 on I2C0: SDA=GP4, SCL=GP1, address=0x3C (CMake options).
 * Connect VCC to 3V3 and GND to GND. No motor pins are configured.
 * Automatic tour starts immediately; a serial USB terminal is optional.
 * Keys: w/s navigate, Enter opens, b returns, +/- adjusts reference,
 * p toggles simulated run/stop, m restarts the automatic tour.
 */
#include "demo_ui.h"
#include <stdio.h>

#ifndef OLED_SDA_PIN
#define OLED_SDA_PIN 4
#endif
#ifndef OLED_SCL_PIN
#define OLED_SCL_PIN 1
#endif
#ifndef OLED_ADDRESS
#define OLED_ADDRESS 0x3C
#endif

static void require_ok(ssd1306_status_t status, const char *operation) {
    if (status == SSD1306_OK) return;
    // Repeat so an error remains visible when USB connects after startup.
    while (true) {
        printf("OLED demo: %s failed (error %d). Check wiring/address.\n", operation, status);
        sleep_ms(1000);
    }
}

int main(void) {
    stdio_init_all();
    i2c_init(i2c0, 400000);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    sleep_ms(100);

    ssd1306_t display = {0};
    require_ok(ssd1306_init(&display, 128, 64, OLED_ADDRESS, i2c0, SSD1306), "init");
    // Full-frame white and black tests also reveal stuck pixels and bad rows.
    ssd1306_draw_square(&display, 0, 0, 128, 64);
    require_ok(ssd1306_show(&display), "white test");
    sleep_ms(600);
    ssd1306_clear(&display);
    require_ok(ssd1306_show(&display), "black test");
    sleep_ms(300);

    demo_ui_t ui;
    demo_ui_init(&ui);
    bool automatic = true;
    absolute_time_t start = get_absolute_time();
    absolute_time_t tour_start = start;
    printf("Motor OLED demo: w/s, Enter, b, +/-, p; m=auto.\n");
    while (true) {
        uint32_t elapsed_ms = (uint32_t)(absolute_time_diff_us(start, get_absolute_time()) / 1000);
        int key;
        while ((key = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            switch (key) {
            case 'w': case 's':
                automatic = false;
                ui.show_menu = true;
                ssd1306_menu_move(&ui.menu, key == 'w' ? -1 : 1);
                break;
            case '\r': case '\n': automatic = false; ui.show_menu = false; break;
            case 'b': automatic = false; ui.show_menu = true; break;
            case '+': case '-':
                automatic = false;
                ui.target_rpm += key == '+' ? 100 : -100;
                if (ui.target_rpm < 0) ui.target_rpm = 0;
                if (ui.target_rpm > 3000) ui.target_rpm = 3000;
                break;
            case 'p': automatic = false; ui.running = !ui.running; break;
            case 'm': automatic = true; tour_start = get_absolute_time(); break;
            default: break;
            }
        }
        if (automatic) {
            uint32_t tour_ms = (uint32_t)(absolute_time_diff_us(tour_start, get_absolute_time()) / 1000);
            ui.menu.selected = (tour_ms / 6000u) % DEMO_ITEM_COUNT;
            ui.show_menu = tour_ms % 6000u < 1500u;
        }
        require_ok(demo_ui_draw(&display, &ui, elapsed_ms), "draw");
        require_ok(ssd1306_show(&display), "refresh");
        sleep_ms(100); // This standalone demo has no control-loop timing demands.
    }
}
