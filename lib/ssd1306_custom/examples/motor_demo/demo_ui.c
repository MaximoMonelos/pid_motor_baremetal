#include "demo_ui.h"
#include <stdarg.h>
#include <stdio.h>

static const char *const items[DEMO_ITEM_COUNT] = {
    "Monitor motor", "Referencia RPM", "Ajustes PID",
    "Salida PWM", "Estado motor", "Prueba de pantalla"
};

#define DRAW(call) do { ssd1306_status_t s_ = (call); \
    if (s_ != SSD1306_OK) return s_; } while (0)

static ssd1306_status_t text(ssd1306_t *p, uint16_t x, uint16_t y,
    uint16_t width, uint16_t height, uint8_t scale, const char *format, ...)
    __attribute__((format(printf, 7, 8)));

static ssd1306_status_t text(ssd1306_t *p, uint16_t x, uint16_t y,
    uint16_t width, uint16_t height, uint8_t scale, const char *format, ...) {
    char buffer[64];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof(buffer)) return SSD1306_ERR_TRUNCATED;
    ssd1306_text_field_t field = {x, y, width, height, scale, SSD1306_ALIGN_LEFT, false};
    return ssd1306_field_text(p, &field, buffer);
}

static ssd1306_status_t bar(ssd1306_t *p, uint16_t x, uint16_t y,
    uint16_t width, uint16_t height, float min, float max, float value, bool vertical) {
    ssd1306_bar_t widget;
    ssd1306_bar_config_t config = {x, y, width, height, min, max, vertical};
    DRAW(ssd1306_bar_init(&widget, p, &config));
    return ssd1306_bar_update(&widget, value);
}

void demo_ui_init(demo_ui_t *ui) {
    *ui = (demo_ui_t){
        .menu = {.items = items, .count = DEMO_ITEM_COUNT},
        .show_menu = true, .running = true, .target_rpm = 1500
    };
}

ssd1306_status_t demo_ui_draw(ssd1306_t *p, demo_ui_t *ui, uint32_t ms) {
    if (!p || !p->buffer || !ui || p->width != 128 || p->height != 64 ||
        ui->menu.selected >= DEMO_ITEM_COUNT) return SSD1306_ERR_ARGUMENT;
    ssd1306_clear(p);
    ssd1306_text_field_t title = {0, 0, 128, 10, 1, SSD1306_ALIGN_CENTER, true};
    DRAW(ssd1306_field_text(p, &title, ui->show_menu ? "MOTOR / DEMO" : items[ui->menu.selected]));
    if (ui->show_menu) {
        ssd1306_text_field_t area = {0, 12, 128, 40, 1, SSD1306_ALIGN_LEFT, false};
        DRAW(ssd1306_menu_draw(p, &ui->menu, &area));
        return text(p, 0, 56, 128, 8, 1, "%u/%u  w/s + Enter",
            (unsigned)ui->menu.selected + 1, DEMO_ITEM_COUNT);
    }

    // A periodic triangle exercises both increasing and decreasing values,
    // including the transition from four-digit numbers to shorter readings.
    uint32_t phase = ms % 8000u;
    float ramp = (float)(phase <= 4000u ? phase : 8000u - phase) / 4000.0f;
    float rpm = ui->running ? 3000.0f * ramp : 0;
    float duty = ui->running ? 100.0f * ramp : 0;
    switch (ui->menu.selected) {
    case 0:
        DRAW(text(p, 0, 12, 128, 16, 2, "%4.0f RPM", rpm));
        DRAW(text(p, 0, 30, 128, 8, 1, "SP:%4.0f PWM:%3.0f%%", ui->target_rpm, duty));
        DRAW(bar(p, 0, 41, 128, 10, 0, 3000, rpm, false));
        DRAW(text(p, 0, 56, 128, 8, 1, "SIMULADO  b:menu"));
        break;
    case 1:
        DRAW(text(p, 0, 13, 128, 8, 1, "Referencia (0-3000)"));
        DRAW(text(p, 0, 25, 128, 16, 2, "%4.0f RPM", ui->target_rpm));
        DRAW(bar(p, 0, 44, 128, 9, 0, 3000, ui->target_rpm, false));
        DRAW(text(p, 0, 56, 128, 8, 1, "+/-:100 RPM b:menu"));
        break;
    case 2:
        DRAW(text(p, 0, 12, 128, 8, 1, "Kp: 0.0200"));
        DRAW(text(p, 0, 23, 128, 8, 1, "Ki: 0.0049"));
        DRAW(text(p, 0, 34, 128, 8, 1, "Kd: 0.0000"));
        DRAW(text(p, 0, 45, 128, 8, 1, "Muestreo: 50 ms"));
        DRAW(text(p, 0, 56, 128, 8, 1, "Valores demo b:menu"));
        break;
    case 3:
        DRAW(text(p, 0, 14, 104, 16, 2, "%3.0f %%", duty));
        DRAW(text(p, 0, 33, 104, 8, 1, "PWM simulado"));
        DRAW(bar(p, 0, 45, 100, 8, 0, 100, duty, false));
        DRAW(bar(p, 112, 12, 16, 42, 0, 100, duty, true));
        DRAW(text(p, 0, 56, 128, 8, 1, "p:Marcha/Paro b:menu"));
        break;
    case 4:
        DRAW(text(p, 0, 12, 128, 8, 1, "Motor: %s", ui->running ? "MARCHA" : "PARADO"));
        DRAW(text(p, 0, 23, 128, 8, 1, "Giro: horario"));
        DRAW(text(p, 0, 34, 128, 8, 1, "Error:%+6.0f RPM", ui->target_rpm - rpm));
        DRAW(text(p, 0, 45, 128, 8, 1, "Tiempo: %lu s", (unsigned long)(ms / 1000u)));
        DRAW(text(p, 0, 56, 128, 8, 1, "p:Marcha/Paro b:menu"));
        break;
    case 5:
        ssd1306_clear(p);
        // Border touches all four limits; moving checkerboard tests every pixel.
        for (uint32_t y = 0; y < 64; ++y)
            for (uint32_t x = 0; x < 128; ++x)
                if (x == 0 || x == 127 || y == 0 || y == 63 ||
                    (((x / 8) + (y / 8) + (ms / 500u)) & 1u))
                    ssd1306_draw_pixel(p, x, y);
        DRAW(text(p, 7, 24, 114, 16, 2, "128 x 64"));
        break;
    }
    return SSD1306_OK;
}
