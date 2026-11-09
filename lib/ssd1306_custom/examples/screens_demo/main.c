/* Optional 128x64 screen layer demo. I2C0: SDA=GP4, SCL=GP1, address=0x3C.
 * USB: w/s = previous/next, Enter = open/confirm, b = back/cancel.
 * Automatic tour runs initially; any navigation key selects manual mode.
 * Press m to restart the tour. No motor pins are configured.
 */
#include "ssd1306_screens.h"
#include <stdio.h>

enum { HOME, MONITOR, REFERENCE, GAINS, KP, KI, PWM, INFO };
static const screen_menu_item_t home[] = {
    {"Monitor motor", MONITOR}, {"Referencia RPM", REFERENCE},
    {"Ganancias PID", GAINS}, {"Salida PWM", PWM}, {"Informacion", INFO}
};
static const screen_menu_item_t gains[] = {{"Editar Kp", KP}, {"Editar Ki", KI}};
static const screen_measurement_t measurements[] = {
    {"Velocidad", "RPM", 0}, {"Referencia", "RPM", 0}, {"Salida", "%", 1},
    {"Kp", NULL, 4}, {"Ki", NULL, 4}
};
static const screen_page_t pages[] = {
    [HOME] = {.kind=SCREEN_MENU, .title="Motor / Capa superior",
        .content.menu={home, 5}},
    [MONITOR] = {.kind=SCREEN_MEASUREMENTS, .title="Monitor motor",
        .content.measurements={measurements, 5}},
    [REFERENCE] = {.kind=SCREEN_NUMBER_EDITOR, .title="Referencia RPM",
        .content.editor={"RPM", 0, 0, 3000, 100}},
    [GAINS] = {.kind=SCREEN_MENU, .title="Ganancias PID", .content.menu={gains, 2}},
    [KP] = {.kind=SCREEN_NUMBER_EDITOR, .title="Editar Kp",
        .content.editor={NULL, 4, 0, 10, 0.001f}},
    [KI] = {.kind=SCREEN_NUMBER_EDITOR, .title="Editar Ki",
        .content.editor={NULL, 4, 0, 1, 0.0001f}},
    [PWM] = {.kind=SCREEN_INDICATOR, .title="Salida PWM",
        .content.indicator={"%", 1, 0, 100}},
    [INFO] = {.kind=SCREEN_MESSAGE, .title="Demo de biblioteca",
        .content.message={"Datos simulados\nSin motor conectado\nEnter: confirmar\nBack: cancelar"}}
};

static ssd1306_t oled;
static ssd1306_screens_t ui; // Static: avoid putting this storage on the stack.
static float reference = 1500, kp = 0.0200f, ki = 0.0049f;

static void check(ssd1306_status_t status) {
    if (status == SSD1306_OK) return;
    while (true) { printf("Screens demo error: %d\n", status); sleep_ms(1000); }
}

static void reset_ui(void) {
    check(ssd1306_screens_init(&ui, &oled, pages, sizeof(pages)/sizeof(pages[0]), HOME));
    check(ssd1306_screens_update(&ui, REFERENCE, &reference, 1));
    check(ssd1306_screens_update(&ui, KP, &kp, 1));
    check(ssd1306_screens_update(&ui, KI, &ki, 1));
}

static void input(screen_input_t event) {
    screen_result_t result;
    check(ssd1306_screens_input(&ui, event, &result));
    // Application state changes only here, on explicit confirmation.
    if (result.action == SCREEN_VALUE_CONFIRMED) {
        if (result.page == REFERENCE) reference = result.value;
        if (result.page == KP) kp = result.value;
        if (result.page == KI) ki = result.value;
        printf("Confirmed page %u: %.4f\n", (unsigned)result.page, (double)result.value);
    } else if (result.action == SCREEN_EDIT_CANCELLED) puts("Cancelled: application values unchanged");
}

// Exercise the same inputs an encoder would produce, including cancel/confirm.
static const screen_input_t tour[] = {
    SCREEN_ENTER, SCREEN_NEXT, SCREEN_BACK, // Monitor and second measurement page.
    SCREEN_NEXT, SCREEN_ENTER, SCREEN_NEXT, SCREEN_NEXT, SCREEN_BACK, // Cancel.
    SCREEN_ENTER, SCREEN_NEXT, SCREEN_ENTER, // Confirm reference +100 RPM.
    SCREEN_NEXT, SCREEN_ENTER, SCREEN_ENTER, SCREEN_NEXT, SCREEN_ENTER, // Kp.
    SCREEN_NEXT, SCREEN_ENTER, SCREEN_NEXT, SCREEN_BACK, // Cancel Ki.
    SCREEN_BACK, SCREEN_NEXT, SCREEN_ENTER, SCREEN_BACK, // PWM.
    SCREEN_NEXT, SCREEN_ENTER, SCREEN_BACK // Message.
};

int main(void) {
    stdio_init_all();
    i2c_init(i2c0, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C); gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(4); gpio_pull_up(1);
    sleep_ms(100);
    check(ssd1306_init(&oled, 128, 64, 0x3C, i2c0, SSD1306));
    reset_ui();
    bool automatic = true;
    bool previous_cr = false;
    size_t step = 0;
    absolute_time_t start = get_absolute_time(), next = delayed_by_ms(start, 1800);
    while (true) {
        int key;
        while ((key = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (key == '\n' && previous_cr) { previous_cr = false; continue; }
            previous_cr = key == '\r'; // A CRLF from a terminal means one Enter.
            if (key == 'm') {
                // Reset discards any unconfirmed draft; application values survive.
                reset_ui();
                step = 0; automatic = true; next = delayed_by_ms(get_absolute_time(), 1800);
            } else if (key == 'w' || key == 's' || key == '\r' || key == '\n' || key == 'b') {
                automatic = false;
                input(key == 'w' ? SCREEN_PREVIOUS : key == 's' ? SCREEN_NEXT :
                    key == 'b' ? SCREEN_BACK : SCREEN_ENTER);
            }
        }
        absolute_time_t now = get_absolute_time();
        if (automatic && absolute_time_diff_us(next, now) >= 0) {
            if (step < sizeof(tour)/sizeof(tour[0])) input(tour[step++]);
            // Tour ends at the home menu; restart only with m.
            next = delayed_by_ms(now, 1800);
        }
        uint32_t t = (uint32_t)(absolute_time_diff_us(start, now)/1000) % 8000u;
        float duty = (float)(t < 4000 ? t : 8000u-t)/40.0f;
        float values[] = {30*duty, reference, duty, kp, ki};
        check(ssd1306_screens_update(&ui, MONITOR, values, 5));
        check(ssd1306_screens_update(&ui, PWM, &duty, 1));
        // No GPIO logic, layout calculations or direct drawing in this loop.
        check(ssd1306_screens_show(&ui));
        sleep_ms(100);
    }
}
