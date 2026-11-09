#include "ssd1306_screens.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool valid_ui(const ssd1306_screens_t *ui) {
    return ui && ui->display && ui->display->buffer && ui->pages &&
        ui->display->width == 128 && ui->display->height == 64 &&
        ui->display->bufsize >= 1024 && ui->count &&
        ui->count <= SSD1306_SCREENS_MAX_PAGES && ui->current < ui->count;
}

static bool valid_range(float min, float max) {
    return isfinite(min) && isfinite(max) && min < max;
}

ssd1306_status_t ssd1306_screens_init(ssd1306_screens_t *ui, ssd1306_t *p,
    const screen_page_t *pages, size_t count, size_t root) {
    if (!ui || !p || !p->buffer || p->width != 128 || p->height != 64 ||
        p->bufsize < 1024 || !pages || !count || count > SSD1306_SCREENS_MAX_PAGES ||
        root >= count || pages[root].kind == SCREEN_NUMBER_EDITOR)
        return SSD1306_ERR_ARGUMENT;
    for (size_t i = 0; i < count; ++i) {
        const screen_page_t *s = &pages[i];
        if (!s->title) return SSD1306_ERR_ARGUMENT;
        switch (s->kind) {
        case SCREEN_MENU:
            if (!s->content.menu.items || !s->content.menu.count) return SSD1306_ERR_ARGUMENT;
            for (size_t j = 0; j < s->content.menu.count; ++j)
                if (!s->content.menu.items[j].label || s->content.menu.items[j].target >= count)
                    return SSD1306_ERR_ARGUMENT;
            break;
        case SCREEN_MEASUREMENTS:
            if (!s->content.measurements.items || !s->content.measurements.count ||
                s->content.measurements.count > SSD1306_SCREENS_MAX_VALUES)
                return SSD1306_ERR_ARGUMENT;
            for (size_t j = 0; j < s->content.measurements.count; ++j)
                if (!s->content.measurements.items[j].label ||
                    s->content.measurements.items[j].decimals > 6) return SSD1306_ERR_ARGUMENT;
            break;
        case SCREEN_INDICATOR:
            if (!valid_range(s->content.indicator.min, s->content.indicator.max) ||
                s->content.indicator.decimals > 6) return SSD1306_ERR_ARGUMENT;
            break;
        case SCREEN_NUMBER_EDITOR:
            if (!valid_range(s->content.editor.min, s->content.editor.max) ||
                !isfinite(s->content.editor.step) || s->content.editor.step <= 0 ||
                s->content.editor.decimals > 6) return SSD1306_ERR_ARGUMENT;
            break;
        case SCREEN_MESSAGE:
            if (!s->content.message.text) return SSD1306_ERR_ARGUMENT;
            break;
        default: return SSD1306_ERR_ARGUMENT;
        }
    }
    memset(ui, 0, sizeof(*ui));
    ui->display = p; ui->pages = pages; ui->count = count; ui->current = root;
    ui->dirty = true; ui->pending = true;
    for (size_t i = 0; i < count; ++i)
        if (pages[i].kind == SCREEN_NUMBER_EDITOR) ui->values[i][0] = pages[i].content.editor.min;
    return SSD1306_OK;
}

ssd1306_status_t ssd1306_screens_update(ssd1306_screens_t *ui,
    size_t page, const float *values, size_t count) {
    if (!valid_ui(ui) || page >= ui->count || !values) return SSD1306_ERR_ARGUMENT;
    const screen_page_t *s = &ui->pages[page];
    size_t expected = s->kind == SCREEN_MEASUREMENTS ? s->content.measurements.count :
        (s->kind == SCREEN_INDICATOR || s->kind == SCREEN_NUMBER_EDITOR ? 1 : 0);
    if (!expected || count != expected) return SSD1306_ERR_ARGUMENT;
    if (s->kind == SCREEN_NUMBER_EDITOR && ui->editing && page == ui->current)
        return SSD1306_ERR_STATE;
    for (size_t i = 0; i < count; ++i)
        if (!isfinite(values[i])) return SSD1306_ERR_ARGUMENT;
    if (s->kind == SCREEN_NUMBER_EDITOR &&
        (values[0] < s->content.editor.min || values[0] > s->content.editor.max))
        return SSD1306_ERR_ARGUMENT;
    for (size_t i = 0; i < count; ++i) {
        if (ui->values[page][i] != values[i]) {
            ui->values[page][i] = values[i];
            if (page == ui->current) ui->dirty = true;
        }
    }
    return SSD1306_OK;
}

ssd1306_status_t ssd1306_screens_input(ssd1306_screens_t *ui,
    screen_input_t input, screen_result_t *result) {
    if (result) *result = (screen_result_t){0};
    if (!valid_ui(ui) || (unsigned)input > SCREEN_BACK) return SSD1306_ERR_ARGUMENT;
    const screen_page_t *s = &ui->pages[ui->current];
    if (ui->editing) {
        if (input == SCREEN_NEXT || input == SCREEN_PREVIOUS) {
            double next = (double)ui->draft +
                (input == SCREEN_NEXT ? (double)s->content.editor.step : -(double)s->content.editor.step);
            if (next < s->content.editor.min) next = s->content.editor.min;
            if (next > s->content.editor.max) next = s->content.editor.max;
            if (ui->draft != (float)next) { ui->draft = (float)next; ui->dirty = true; }
        } else {
            bool confirmed = input == SCREEN_ENTER;
            if (confirmed) ui->values[ui->current][0] = ui->draft;
            if (result) *result = (screen_result_t){
                confirmed ? SCREEN_VALUE_CONFIRMED : SCREEN_EDIT_CANCELLED,
                ui->current, ui->values[ui->current][0]};
            ui->editing = false;
            if (ui->depth) ui->current = ui->history[--ui->depth];
            ui->dirty = true;
        }
    } else if (input == SCREEN_BACK) {
        if (ui->depth) { ui->current = ui->history[--ui->depth]; ui->dirty = true; }
    } else if (input == SCREEN_ENTER && s->kind == SCREEN_MENU) {
        if (ui->depth == SSD1306_SCREENS_HISTORY) return SSD1306_ERR_STATE;
        size_t target = s->content.menu.items[ui->selection[ui->current]].target;
        ui->history[ui->depth++] = ui->current;
        ui->current = target;
        ui->editing = ui->pages[target].kind == SCREEN_NUMBER_EDITOR;
        if (ui->editing) ui->draft = ui->values[target][0];
        ui->dirty = true;
    } else if (input == SCREEN_NEXT || input == SCREEN_PREVIOUS) {
        size_t limit = s->kind == SCREEN_MENU ? s->content.menu.count :
            (s->kind == SCREEN_MEASUREMENTS ? (s->content.measurements.count + 3) / 4 : 1);
        size_t *index = &ui->selection[ui->current];
        if (input == SCREEN_NEXT && *index + 1 < limit) { ++*index; ui->dirty = true; }
        if (input == SCREEN_PREVIOUS && *index) { --*index; ui->dirty = true; }
    }
    return SSD1306_OK;
}

/* Fixed layout helpers. Shortened labels never leak outside their rectangle. */
static void text(ssd1306_t *p, unsigned x, unsigned y, unsigned w, unsigned h,
    unsigned scale, ssd1306_align_t align, bool inverted, const char *str) {
    char fit[22];
    size_t capacity = (w + scale) / (6u * scale);
    if (capacity > sizeof(fit) - 1) capacity = sizeof(fit) - 1;
    size_t n = strlen(str);
    if (n <= capacity) memcpy(fit, str, n + 1);
    else {
        memcpy(fit, str, capacity); fit[capacity] = 0;
        if (capacity >= 3) memcpy(fit + capacity - 3, "...", 3);
    }
    ssd1306_text_field_t f = {x, y, w, h, scale, align, inverted};
    (void)ssd1306_field_text(p, &f, fit);
}

static void number(char *out, size_t capacity, float value, unsigned decimals, const char *unit) {
    char n[56];
    if (!unit) unit = "";
    snprintf(n, sizeof(n), "%.*f", (int)decimals, (double)value);
    if (strlen(n) + strlen(unit) + (*unit ? 1u : 0u) >= capacity)
        snprintf(n, sizeof(n), "%.3g", (double)value);
    // Preserve the number if a long unit cannot fit; unit may be in page title.
    if (strlen(n) + strlen(unit) + (*unit ? 1u : 0u) < capacity)
        snprintf(out, capacity, "%s%s%s", n, *unit ? " " : "", unit);
    else snprintf(out, capacity, "%s", n);
}

static void render(ssd1306_screens_t *ui) {
    ssd1306_t *p = ui->display;
    const screen_page_t *s = &ui->pages[ui->current];
    char buffer[64];
    ssd1306_clear(p);
    text(p, 0, 0, 128, 10, 1, SSD1306_ALIGN_CENTER, true, s->title);
    switch (s->kind) {
    case SCREEN_MENU: {
        size_t selected = ui->selection[ui->current], *first = &ui->first[ui->current];
        if (selected < *first) *first = selected;
        if (selected - *first >= 4) *first = selected - 3;
        for (size_t row = 0; row < 4 && *first + row < s->content.menu.count; ++row)
            text(p, 0, 12 + row * 10, 128, 10, 1, SSD1306_ALIGN_LEFT,
                *first + row == selected, s->content.menu.items[*first + row].label);
        snprintf(buffer, sizeof(buffer), "%u/%u Enter:abrir", (unsigned)selected + 1, (unsigned)s->content.menu.count);
        break;
    }
    case SCREEN_MEASUREMENTS: {
        size_t batch = ui->selection[ui->current];
        for (size_t row = 0; row < 4 && batch * 4 + row < s->content.measurements.count; ++row) {
            size_t i = batch * 4 + row;
            const screen_measurement_t *item = &s->content.measurements.items[i];
            text(p, 0, 12 + row * 10, 64, 10, 1, SSD1306_ALIGN_LEFT, false, item->label);
            char value[11]; number(value, sizeof(value), ui->values[ui->current][i], item->decimals, item->unit);
            text(p, 64, 12 + row * 10, 64, 10, 1, SSD1306_ALIGN_RIGHT, false, value);
        }
        snprintf(buffer, sizeof(buffer), "%u/%u +/-:pag Back", (unsigned)batch + 1,
            (unsigned)((s->content.measurements.count + 3) / 4));
        break;
    }
    case SCREEN_INDICATOR: {
        char value[11]; number(value, sizeof(value), ui->values[ui->current][0], s->content.indicator.decimals, s->content.indicator.unit);
        text(p, 0, 15, 128, 16, 2, SSD1306_ALIGN_CENTER, false, value);
        ssd1306_bar_t bar;
        ssd1306_bar_config_t config = {0, 40, 128, 11, s->content.indicator.min, s->content.indicator.max, false};
        (void)ssd1306_bar_init(&bar, p, &config);
        (void)ssd1306_bar_update(&bar, ui->values[ui->current][0]);
        snprintf(buffer, sizeof(buffer), "Back:menu");
        break;
    }
    case SCREEN_NUMBER_EDITOR: {
        char value[11]; number(value, sizeof(value), ui->draft, s->content.editor.decimals, s->content.editor.unit);
        text(p, 0, 15, 128, 16, 2, SSD1306_ALIGN_CENTER, false, value);
        text(p, 0, 39, 128, 10, 1, SSD1306_ALIGN_CENTER, false, "+/-:cambiar");
        snprintf(buffer, sizeof(buffer), "Enter:OK Back:cancel");
        break;
    }
    case SCREEN_MESSAGE: {
        const char *cursor = s->content.message.text;
        for (unsigned row = 0; row < 4 && *cursor; ++row) {
            size_t n = 0;
            while (cursor[n] && cursor[n] != '\n' && n < 21) ++n;
            char line[22]; memcpy(line, cursor, n); line[n] = 0;
            text(p, 0, 12 + row * 10, 128, 10, 1, SSD1306_ALIGN_LEFT, false, line);
            cursor += n;
            if (*cursor == '\n') ++cursor;
        }
        snprintf(buffer, sizeof(buffer), "Back:menu");
        break;
    }
    }
    text(p, 0, 56, 128, 8, 1, SSD1306_ALIGN_LEFT, false, buffer);
}

ssd1306_status_t ssd1306_screens_draw(ssd1306_screens_t *ui, bool *changed) {
    if (changed) *changed = false;
    if (!valid_ui(ui)) return SSD1306_ERR_ARGUMENT;
    if (!ui->dirty) return SSD1306_OK;
    render(ui);
    bool different = !ui->drawn || memcmp(ui->image, ui->display->buffer, 1024) != 0;
    if (different) {
        memcpy(ui->image, ui->display->buffer, 1024);
        ui->pending = true;
    }
    ui->dirty = false; ui->drawn = true;
    if (changed) *changed = different;
    return SSD1306_OK;
}

ssd1306_status_t ssd1306_screens_show(ssd1306_screens_t *ui) {
    ssd1306_status_t status = ssd1306_screens_draw(ui, NULL);
    if (status != SSD1306_OK || !ui->pending) return status;
    status = ssd1306_show(ui->display);
    if (status == SSD1306_OK) ui->pending = false;
    return status;
}

void ssd1306_screens_invalidate(ssd1306_screens_t *ui) {
    if (valid_ui(ui)) { ui->dirty = true; ui->pending = true; }
}
