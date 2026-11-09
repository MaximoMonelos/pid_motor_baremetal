
/*

MIT License

Copyright (c) 2021 David Schramm

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ssd1306.h"
#include "font.h"
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


inline static void swap(int32_t *a, int32_t *b) {
    int32_t t=*a;
    *a=*b;
    *b=t;
}

static ssd1306_status_t write_bytes(ssd1306_t *p, const uint8_t *data, size_t len) {
    if (!p || !p->i2c_i || !p->buffer) return SSD1306_ERR_ARGUMENT;
    int result = i2c_write_timeout_us(p->i2c_i, p->address, data, len, false,
                                     p->timeout_us);
    if (result == PICO_ERROR_TIMEOUT) return SSD1306_ERR_TIMEOUT;
    return result == (int)len ? SSD1306_OK : SSD1306_ERR_I2C;
}

static ssd1306_status_t ssd1306_write(ssd1306_t *p, uint8_t val) {
    const uint8_t data[2] = {0x00, val};
    return write_bytes(p, data, sizeof(data));
}

ssd1306_status_t ssd1306_init(ssd1306_t *p, uint16_t width, uint16_t height, uint8_t address, i2c_inst_t *i2c_instance, ssd1306_display_type_t type) {
    if (!p || !i2c_instance || address < 0x08 || address > 0x77 ||
        (width != 64 && width != 128) || (height != 32 && height != 64) ||
        (type != SSD1306 && type != SH1106)) return SSD1306_ERR_ARGUMENT;
    // Instance must be zero-initialized; deinit before initializing it again.
    if (p->buffer) return SSD1306_ERR_STATE;
    p->width = width;
    p->height = height;
    p->pages = height / 8;
    p->address = address;
    p->type = type;
    p->i2c_i = i2c_instance;
    if (!p->timeout_us) p->timeout_us = 10000;
    p->bufsize = (size_t)p->pages * p->width;
    p->buffer = calloc(p->bufsize, 1);
    if (!p->buffer) {
        p->bufsize = 0;
        return SSD1306_ERR_MEMORY;
    }

    // from https://github.com/makerportal/rpi-pico-ssd1306
    uint8_t cmds[]= {
        SET_DISP,
        // timing and driving scheme
        SET_DISP_CLK_DIV,
        0x80,
        SET_MUX_RATIO,
        height - 1,
        SET_DISP_OFFSET,
        0x00,
        // resolution and layout
        SET_DISP_START_LINE,
        // charge pump
        SET_CHARGE_PUMP,
        p->external_vcc?0x10:0x14,
        SET_SEG_REMAP | 0x01,           // column addr 127 mapped to SEG0
        SET_COM_OUT_DIR | 0x08,         // scan from COM[N] to COM0
        SET_COM_PIN_CFG,
        width>2*height?0x02:0x12,
        // display
        SET_CONTRAST,
        0xff,
        SET_PRECHARGE,
        p->external_vcc?0x22:0xF1,
        SET_VCOM_DESEL,
        0x30,                           // or 0x40?
        SET_ENTIRE_ON,                  // output follows RAM contents
        SET_NORM_INV,                   // not inverted
        SET_DISP | 0x01,
        // address setting
        SET_MEM_ADDR,
        0x00,  // horizontal
    };

    // SH1106 does not use the SSD1306 horizontal-addressing command.
    size_t command_count = sizeof(cmds) - (type == SH1106 ? 2u : 0u);
    for (size_t i = 0; i < command_count; ++i) {
        ssd1306_status_t status = ssd1306_write(p, cmds[i]);
        if (status != SSD1306_OK) {
            ssd1306_deinit(p);
            return status;
        }
    }
    return SSD1306_OK;
}

void ssd1306_deinit(ssd1306_t *p) {
    if (!p) return;
    free(p->buffer);
    p->buffer = NULL;
    p->bufsize = 0;
}

ssd1306_status_t ssd1306_poweroff(ssd1306_t *p) {
    return ssd1306_write(p, SET_DISP);
}

ssd1306_status_t ssd1306_poweron(ssd1306_t *p) {
    return ssd1306_write(p, SET_DISP | 1);
}

ssd1306_status_t ssd1306_contrast(ssd1306_t *p, uint8_t val) {
    const uint8_t data[] = {0x00, SET_CONTRAST, val};
    return write_bytes(p, data, sizeof(data));
}

ssd1306_status_t ssd1306_invert(ssd1306_t *p, uint8_t inv) {
    return ssd1306_write(p, SET_NORM_INV | (inv & 1));
}

void ssd1306_clear(ssd1306_t *p) {
    if (p && p->buffer) memset(p->buffer, 0, p->bufsize);
}

void ssd1306_clear_pixel(ssd1306_t *p, uint32_t x, uint32_t y) {
    if(!p || !p->buffer || x>=p->width || y>=p->height) return;

    p->buffer[x+p->width*(y>>3)]&=~(0x1<<(y&0x07));
}

void ssd1306_draw_pixel(ssd1306_t *p, uint32_t x, uint32_t y) {
    if(!p || !p->buffer || x>=p->width || y>=p->height) return;

    p->buffer[x+p->width*(y>>3)]|=0x1<<(y&0x07); // y>>3==y/8 && y&0x7==y%8
}

void ssd1306_draw_line(ssd1306_t *p, int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    if(x1>x2) {
        swap(&x1, &x2);
        swap(&y1, &y2);
    }

    if(x1==x2) {
        if(y1>y2)
            swap(&y1, &y2);
        for(int32_t i=y1; i<=y2; ++i)
            ssd1306_draw_pixel(p, x1, i);
        return;
    }

    float m=(float) (y2-y1) / (float) (x2-x1);

    for(int32_t i=x1; i<=x2; ++i) {
        float y=m*(float) (i-x1)+(float) y1;
        ssd1306_draw_pixel(p, i, (uint32_t) y);
    }
}

void ssd1306_clear_square(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    for(uint32_t i=0; i<width; ++i)
        for(uint32_t j=0; j<height; ++j)
            ssd1306_clear_pixel(p, x+i, y+j);
}

void ssd1306_draw_square(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    for(uint32_t i=0; i<width; ++i)
        for(uint32_t j=0; j<height; ++j)
            ssd1306_draw_pixel(p, x+i, y+j);
}

/**
 * @brief Añadido
 * 
 */
void ssd1306_draw_filled_triangle(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t size, bool flip) {
    for (uint32_t j = 0; j < size; ++j) {
        for (uint32_t i = 0; i <= j; ++i) {
            uint32_t px = x + i;
            uint32_t py = flip ? (y + (size - 1 - j)) : (y + j);
            ssd1306_draw_pixel(p, px, py);
        }
    }
}

void ssd1306_draw_empty_square(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    ssd1306_draw_line(p, x, y, x+width, y);
    ssd1306_draw_line(p, x, y+height, x+width, y+height);
    ssd1306_draw_line(p, x, y, x, y+height);
    ssd1306_draw_line(p, x+width, y, x+width, y+height);
}

void ssd1306_draw_char_with_font(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale, const uint8_t *font, char c) {
    if(!p || !p->buffer || !font || !scale || x>=p->width || y>=p->height ||
       (unsigned char)c<font[3] || (unsigned char)c>font[4])
        return;

    uint32_t parts_per_line=(font[0]>>3)+((font[0]&7)>0);
    for(uint8_t w=0; w<font[1]; ++w) { // width
        uint32_t pp=(c-font[3])*font[1]*parts_per_line+w*parts_per_line+5;
        for(uint32_t lp=0; lp<parts_per_line; ++lp) {
            uint8_t line=font[pp + lp];

            for(int8_t j=0; j<8; ++j, line>>=1) {
                if(line & 1)
                    ssd1306_draw_square(p, x+w*scale, y+((lp<<3)+j)*scale, scale, scale);
            }

        }
    }
}

void ssd1306_draw_string_with_font(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale, const uint8_t *font, const char *s) {
    if (!p || !p->buffer || !font || !s || !scale || x>=p->width || y>=p->height) return;
    for(uint32_t x_n=x; *s && x_n<p->width; x_n+=(font[1]+font[2])*scale) {
        ssd1306_draw_char_with_font(p, x_n, y, scale, font, *(s++));
    }
}

void ssd1306_draw_char(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale, char c) {
    ssd1306_draw_char_with_font(p, x, y, scale, font_8x5, c);
}

void ssd1306_draw_string(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t scale, const char *s) {
    ssd1306_draw_string_with_font(p, x, y, scale, font_8x5, s);
}

static inline uint32_t ssd1306_bmp_get_val(const uint8_t *data, const size_t offset, uint8_t size) {
    switch(size) {
    case 1:
        return data[offset];
    case 2:
        return data[offset]|(data[offset+1]<<8);
    case 4:
        return data[offset]|(data[offset+1]<<8)|(data[offset+2]<<16)|(data[offset+3]<<24);
    default:
        __builtin_unreachable();
    }
    __builtin_unreachable();
}

void ssd1306_bmp_show_image_with_offset(ssd1306_t *p, const uint8_t *data, const long size, uint32_t x_offset, uint32_t y_offset) {
    if(size<54) // data smaller than header
        return;

    const uint32_t bfOffBits=ssd1306_bmp_get_val(data, 10, 4);
    const uint32_t biSize=ssd1306_bmp_get_val(data, 14, 4);
    const uint32_t biWidth=ssd1306_bmp_get_val(data, 18, 4);
    const int32_t biHeight=(int32_t) ssd1306_bmp_get_val(data, 22, 4);
    const uint16_t biBitCount=(uint16_t) ssd1306_bmp_get_val(data, 28, 2);
    const uint32_t biCompression=ssd1306_bmp_get_val(data, 30, 4);

    if(biBitCount!=1) // image not monochrome
        return;

    if(biCompression!=0) // image compressed
        return;

    const int table_start=14+biSize;
    uint8_t color_val=0;

    for(uint8_t i=0; i<2; ++i) {
        if(!((data[table_start+i*4]<<16)|(data[table_start+i*4+1]<<8)|data[table_start+i*4+2])) {
            color_val=i;
            break;
        }
    }

    uint32_t bytes_per_line=(biWidth/8)+(biWidth&7?1:0);
    if(bytes_per_line&3)
        bytes_per_line=(bytes_per_line^(bytes_per_line&3))+4;

    const uint8_t *img_data=data+bfOffBits;

    int32_t step=biHeight>0?-1:1;
    int32_t border=biHeight>0?-1:-biHeight;

    for(uint32_t y=biHeight>0?biHeight-1:0; y!=(uint32_t)border; y+=step) {
        for(uint32_t x=0; x<biWidth; ++x) {
            if(((img_data[x>>3]>>(7-(x&7)))&1)==color_val)
                ssd1306_draw_pixel(p, x_offset+x, y_offset+y);
        }
        img_data+=bytes_per_line;
    }
}

void ssd1306_bmp_show_image(ssd1306_t *p, const uint8_t *data, const long size) {
    ssd1306_bmp_show_image_with_offset(p, data, size, 0, 0);
}

ssd1306_status_t ssd1306_show_pages(ssd1306_t *p, uint8_t first, uint8_t count) {
    if (!p || !p->buffer || !count || first >= p->pages || count > p->pages-first)
        return SSD1306_ERR_ARGUMENT;
    uint8_t data[129];
    data[0] = 0x40;
    for (uint8_t page = first; page < first + count; ++page) {
        uint8_t offset = p->type == SH1106 ? 2 : (p->width == 64 ? 32 : 0);
        uint8_t commands[7] = {0x00, SET_COL_ADDR, offset,
            (uint8_t)(offset + p->width - 1), SET_PAGE_ADDR, page, page};
        size_t command_size = sizeof(commands);
        if (p->type == SH1106) {
            commands[1] = 0xB0 | page;
            commands[2] = offset & 0x0F;
            commands[3] = 0x10 | (offset >> 4);
            command_size = 4;
        }
        ssd1306_status_t status = write_bytes(p, commands, command_size);
        if (status != SSD1306_OK) return status;
        memcpy(data + 1, p->buffer + (size_t)page * p->width, p->width);
        status = write_bytes(p, data, p->width + 1u);
        if (status != SSD1306_OK) return status;
    }
    return SSD1306_OK;
}

ssd1306_status_t ssd1306_show(ssd1306_t *p) {
    if (!p) return SSD1306_ERR_ARGUMENT;
    return ssd1306_show_pages(p, 0, p->pages);
}

// Invierte los valores de los píxeles en un recuadro
void ssd1306_invert_square(ssd1306_t *p, uint32_t x, uint32_t y, uint32_t width, uint32_t height) {
    for(uint32_t i=0; i<width; ++i) {
        for(uint32_t j=0; j<height; ++j) {
            uint32_t px = x + i;
            uint32_t py = y + j;
            if(px < p->width && py < p->height) {
                uint32_t buf_idx = px + p->width * (py >> 3);
                uint8_t mask = 1 << (py & 0x07);
                p->buffer[buf_idx] ^= mask;
            }
        }
    }
}