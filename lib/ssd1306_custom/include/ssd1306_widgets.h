#ifndef SSD1306_WIDGETS_H
#define SSD1306_WIDGETS_H

#include "ssd1306.h"

/* Widgets only draw into RAM. Call show/show_pages explicitly afterwards.
 * Built-in font: printable ASCII, 5x8 pixels plus 1 pixel spacing.
 * Fields clip to their rectangle and erase its background on each draw.
 * One instance must be used by only one execution context at a time. */
typedef enum { SSD1306_ALIGN_LEFT, SSD1306_ALIGN_CENTER, SSD1306_ALIGN_RIGHT }
    ssd1306_align_t;

typedef struct {
    uint16_t x, y, width, height;
    uint8_t scale;
    ssd1306_align_t align;
    bool inverted;
} ssd1306_text_field_t;

ssd1306_status_t ssd1306_field_text(ssd1306_t *display,
    const ssd1306_text_field_t *field, const char *text);
/* Formatting is bounded to 127 characters. ERR_TRUNCATED also reports clipping.
 * Float formatting uses the application's C library support for snprintf. */
ssd1306_status_t ssd1306_field_printf(ssd1306_t *display,
    const ssd1306_text_field_t *field, const char *format, ...)
    __attribute__((format(printf, 3, 4)));

/* Keep labels/items alive while using the menu. Selection is application state;
 * the library never executes actions or reads buttons/encoders. No wraparound. */
typedef struct {
    const char *const *items;
    size_t count;
    size_t selected;
    size_t first_visible;
} ssd1306_menu_t;

void ssd1306_menu_move(ssd1306_menu_t *menu, int delta);
/* Area height determines visible rows (font height + 2 pixels padding).
 * Selection is highlighted; first_visible automatically follows selection.
 * ERR_TRUNCATED means a label was shortened, not that scrolling failed. */
ssd1306_status_t ssd1306_menu_draw(ssd1306_t *display,
    ssd1306_menu_t *menu, const ssd1306_text_field_t *area);

/* Progress bars have a one-pixel border and need at least 3x3 pixels.
 * Ranges and values support negatives and decimals; limits must be finite
 * with min < max. Values outside the range are clamped, NaN/Inf rejected.
 * Horizontal bars fill left to right; vertical bars fill bottom to top.
 * All drawing stays within the configured rectangle. No I2C is performed. */
typedef struct {
    uint16_t x, y, width, height;
    float min, max;
    bool vertical; /* false by default */
} ssd1306_bar_config_t;

typedef struct {
    ssd1306_t *display;
    ssd1306_bar_config_t config;
} ssd1306_bar_t;

/* Copies config and draws an empty bar. Display must remain initialized and
 * alive while using the bar. Invalid init leaves the bar and framebuffer intact. */
ssd1306_status_t ssd1306_bar_init(ssd1306_bar_t *bar, ssd1306_t *display,
    const ssd1306_bar_config_t *config);
/* Replaces the previous fill, including when value decreases. Pixel lengths
 * are rounded to the nearest pixel. Call show/show_pages to send the result. */
ssd1306_status_t ssd1306_bar_update(ssd1306_bar_t *bar, float value);

#endif
