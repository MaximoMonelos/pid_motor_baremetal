#ifndef MOTOR_DEMO_UI_H
#define MOTOR_DEMO_UI_H
#include "ssd1306_widgets.h"

#define DEMO_ITEM_COUNT 6u

typedef struct {
    ssd1306_menu_t menu;
    bool show_menu;
    bool running;
    float target_rpm;
} demo_ui_t;

void demo_ui_init(demo_ui_t *ui);
/* Draws a complete simulated 128x64 screen into RAM; never drives a motor. */
ssd1306_status_t demo_ui_draw(ssd1306_t *display, demo_ui_t *ui, uint32_t elapsed_ms);
#endif
