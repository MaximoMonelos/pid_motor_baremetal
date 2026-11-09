#include "ssd1306_widgets.h"
#include "font.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static bool valid_field(const ssd1306_t *p, const ssd1306_text_field_t *f) {
    return p && p->buffer && f && f->scale && f->width && f->height &&
        f->x < p->width && f->y < p->height &&
        f->width <= p->width - f->x && f->height <= p->height - f->y &&
        (unsigned)f->align <= SSD1306_ALIGN_RIGHT;
}

ssd1306_status_t ssd1306_field_text(ssd1306_t *p,
    const ssd1306_text_field_t *f, const char *text) {
    if (!text || !valid_field(p, f)) return SSD1306_ERR_ARGUMENT;
    size_t length = strlen(text);
    size_t cell = 6u * f->scale;
    // Last character needs no trailing spacing pixel.
    size_t capacity = (f->width + f->scale) / cell;
    size_t visible = length < capacity ? length : capacity;
    size_t text_width = visible ? visible * cell - f->scale : 0;
    uint32_t x = f->x;
    if (f->align == SSD1306_ALIGN_CENTER) x += (f->width - text_width) / 2;
    if (f->align == SSD1306_ALIGN_RIGHT) x += f->width - text_width;
    if (f->inverted) ssd1306_draw_square(p, f->x, f->y, f->width, f->height);
    else ssd1306_clear_square(p, f->x, f->y, f->width, f->height);
    uint32_t glyph_height = 8u * f->scale;
    uint32_t y = f->y + (f->height > glyph_height ? (f->height - glyph_height)/2 : 0);
    for (size_t i = 0; i < visible; ++i) {
        unsigned char c = (unsigned char)text[i];
        if (c < 32 || c > 126) c = '?';
        for (uint32_t col = 0; col < 5; ++col) {
            uint8_t bits = font_8x5[5 + (c - 32) * 5 + col];
            for (uint32_t row = 0; row < 8; ++row) {
                if (!(bits & (1u << row))) continue;
                for (uint32_t dy = 0; dy < f->scale; ++dy) {
                    uint32_t py = y + row * f->scale + dy;
                    if (py >= f->y + f->height) continue;
                    for (uint32_t dx = 0; dx < f->scale; ++dx) {
                        uint32_t px = x + i * cell + col * f->scale + dx;
                        if (f->inverted) ssd1306_clear_pixel(p, px, py);
                        else ssd1306_draw_pixel(p, px, py);
                    }
                }
            }
        }
    }
    return visible < length || (length && f->height < glyph_height)
        ? SSD1306_ERR_TRUNCATED : SSD1306_OK;
}

ssd1306_status_t ssd1306_field_printf(ssd1306_t *p,
    const ssd1306_text_field_t *f, const char *format, ...) {
    if (!format || !valid_field(p, f)) return SSD1306_ERR_ARGUMENT;
    char text[128];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (length < 0) return SSD1306_ERR_ARGUMENT;
    ssd1306_status_t status = ssd1306_field_text(p, f, text);
    if (status != SSD1306_OK) return status;
    return (size_t)length >= sizeof(text) ? SSD1306_ERR_TRUNCATED : SSD1306_OK;
}

void ssd1306_menu_move(ssd1306_menu_t *m, int delta) {
    if (!m || !m->count) return;
    if (m->selected >= m->count) m->selected = m->count - 1;
    // Avoid signed overflow even for INT_MIN.
    size_t steps = delta < 0 ? (size_t)(-(delta + 1)) + 1u : (size_t)delta;
    if (delta < 0) m->selected = steps > m->selected ? 0 : m->selected - steps;
    else {
        size_t remaining = m->count - 1 - m->selected;
        m->selected += steps > remaining ? remaining : steps;
    }
}

ssd1306_status_t ssd1306_menu_draw(ssd1306_t *p,
    ssd1306_menu_t *m, const ssd1306_text_field_t *area) {
    if (!m || !valid_field(p, area) || (m->count && !m->items))
        return SSD1306_ERR_ARGUMENT;
    uint32_t row_height = 8u * area->scale + 2u;
    size_t rows = area->height / row_height;
    if (!rows) return SSD1306_ERR_ARGUMENT;
    if (m->count) {
        if (m->selected >= m->count) m->selected = m->count - 1;
        size_t max_first = m->count > rows ? m->count - rows : 0;
        if (m->first_visible > max_first) m->first_visible = max_first;
        if (m->selected < m->first_visible) m->first_visible = m->selected;
        if (m->selected - m->first_visible >= rows)
            m->first_visible = m->selected - rows + 1;
    } else m->selected = m->first_visible = 0;
    ssd1306_clear_square(p, area->x, area->y, area->width, area->height);
    ssd1306_status_t result = SSD1306_OK;
    for (size_t row = 0; row < rows && row < m->count - m->first_visible; ++row) {
        size_t index = m->first_visible + row;
        ssd1306_text_field_t field = *area;
        field.y += row * row_height;
        field.height = row_height;
        field.inverted = index == m->selected;
        ssd1306_status_t status = ssd1306_field_text(p, &field,
            m->items[index] ? m->items[index] : "");
        if (status != SSD1306_OK) result = status;
    }
    return result;
}

static bool valid_bar(const ssd1306_t *p, const ssd1306_bar_config_t *c) {
    return p && p->buffer && c && c->width >= 3 && c->height >= 3 &&
        c->x < p->width && c->y < p->height &&
        c->width <= p->width - c->x && c->height <= p->height - c->y &&
        isfinite(c->min) && isfinite(c->max) && c->min < c->max;
}

ssd1306_status_t ssd1306_bar_init(ssd1306_bar_t *bar, ssd1306_t *p,
    const ssd1306_bar_config_t *config) {
    if (!bar || !valid_bar(p, config)) return SSD1306_ERR_ARGUMENT;
    bar->display = p;
    bar->config = *config;
    return ssd1306_bar_update(bar, config->min);
}

ssd1306_status_t ssd1306_bar_update(ssd1306_bar_t *bar, float value) {
    if (!bar || !valid_bar(bar->display, &bar->config) || !isfinite(value))
        return SSD1306_ERR_ARGUMENT;
    const ssd1306_bar_config_t *c = &bar->config;
    ssd1306_t *p = bar->display;
    uint32_t inner_width = c->width - 2u;
    uint32_t inner_height = c->height - 2u;
    uint32_t extent = c->vertical ? inner_height : inner_width;
    uint32_t filled;
    if (value <= c->min) filled = 0;
    else if (value >= c->max) filled = extent;
    else {
        // Double arithmetic prevents overflow for wide finite float ranges.
        double fraction = ((double)value - c->min) / ((double)c->max - c->min);
        filled = (uint32_t)(fraction * extent + 0.5);
    }
    ssd1306_draw_square(p, c->x, c->y, c->width, c->height);
    ssd1306_clear_square(p, c->x + 1u, c->y + 1u, inner_width, inner_height);
    if (c->vertical)
        ssd1306_draw_square(p, c->x + 1u, c->y + 1u + inner_height - filled,
            inner_width, filled);
    else
        ssd1306_draw_square(p, c->x + 1u, c->y + 1u, filled, inner_height);
    return SSD1306_OK;
}
