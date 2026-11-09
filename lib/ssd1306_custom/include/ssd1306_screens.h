#ifndef SSD1306_SCREENS_H
#define SSD1306_SCREENS_H

#include "ssd1306_widgets.h"

/* Optional, allocation-free UI for a 128x64 display. Link ssd1306_screens.
 * Definitions and their strings/arrays are borrowed and must stay alive and
 * immutable. Runtime values are copied by update(); no application pointers
 * are retained. One execution context must own the display/framebuffer.
 * Printable ASCII only. Long labels are shortened to fit automatically. */
#define SSD1306_SCREENS_MAX_PAGES 12u
#define SSD1306_SCREENS_MAX_VALUES 8u
#define SSD1306_SCREENS_HISTORY 8u

typedef enum {
    SCREEN_MENU, SCREEN_MEASUREMENTS, SCREEN_INDICATOR,
    SCREEN_NUMBER_EDITOR, SCREEN_MESSAGE
} screen_kind_t;

typedef struct {
    const char *label;
    const char *unit; /* NULL means no unit */
    uint8_t decimals; /* 0..6; large numbers may use scientific notation */
} screen_measurement_t;

typedef struct {
    const char *label;
    size_t target; /* index in the page definition array */
} screen_menu_item_t;

typedef struct {
    screen_kind_t kind;
    const char *title;
    union {
        struct { const screen_menu_item_t *items; size_t count; } menu;
        struct { const screen_measurement_t *items; size_t count; } measurements;
        struct { const char *unit; uint8_t decimals; float min, max; } indicator;
        struct { const char *unit; uint8_t decimals; float min, max, step; } editor;
        struct { const char *text; } message;
    } content;
} screen_page_t;

typedef enum { SCREEN_NEXT, SCREEN_PREVIOUS, SCREEN_ENTER, SCREEN_BACK } screen_input_t;
typedef enum { SCREEN_NO_ACTION, SCREEN_VALUE_CONFIRMED, SCREEN_EDIT_CANCELLED } screen_action_t;
typedef struct {
    screen_action_t action;
    size_t page;
    float value;
} screen_result_t;

/* Public storage, private implementation fields: initialize with init(), do not
 * modify them directly. About 2 KB RAM, fixed size, no malloc by this layer. */
typedef struct {
    ssd1306_t *display;
    const screen_page_t *pages;
    size_t count, current, depth;
    size_t history[SSD1306_SCREENS_HISTORY];
    size_t selection[SSD1306_SCREENS_MAX_PAGES];
    size_t first[SSD1306_SCREENS_MAX_PAGES];
    float values[SSD1306_SCREENS_MAX_PAGES][SSD1306_SCREENS_MAX_VALUES];
    float draft;
    bool editing, dirty, drawn, pending;
    uint8_t image[1024];
} ssd1306_screens_t;

/* Root must be a non-editor page. All non-menu values initially default to 0
 * (editors start at min). Supply initial values with update() before showing. */
ssd1306_status_t ssd1306_screens_init(ssd1306_screens_t *ui, ssd1306_t *display,
    const screen_page_t *pages, size_t count, size_t root);

/* Counts: measurements = item count, indicator/editor = 1. All values must be
 * finite. An editor accepts values within its range and rejects updates while
 * it is being edited (ERR_STATE). Other pages may update during navigation.
 * Indicator values are displayed as supplied; only its bar is clamped. */
ssd1306_status_t ssd1306_screens_update(ssd1306_screens_t *ui,
    size_t page, const float *values, size_t count);

/* NEXT/PREVIOUS navigate or change a draft by step; no wraparound. ENTER opens
 * a menu destination or confirms an editor. BACK returns or cancels an editor.
 * Confirm/cancel return to the previous page. result may be NULL. Application
 * values are never written: handle SCREEN_VALUE_CONFIRMED in the application. */
ssd1306_status_t ssd1306_screens_input(ssd1306_screens_t *ui,
    screen_input_t input, screen_result_t *result);

/* draw() only changes RAM. changed reports pixel differences since last draw.
 * show() draws as needed and transmits only when pending; failed sends remain
 * pending for retry. Scheduling is the application's responsibility.
 * Do not draw with another module between calls without invalidate(). */
ssd1306_status_t ssd1306_screens_draw(ssd1306_screens_t *ui, bool *changed);
ssd1306_status_t ssd1306_screens_show(ssd1306_screens_t *ui);
void ssd1306_screens_invalidate(ssd1306_screens_t *ui);

#endif
