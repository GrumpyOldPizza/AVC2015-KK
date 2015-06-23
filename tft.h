/*
 * Copyright (c) 2015 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#if !defined(_TFT_H)
#define _TFT_H

typedef struct _tft_font_t {
    uint8_t       font_width;
    uint8_t       font_height;
    uint16_t      font_offset;
    const uint8_t *font_data;
} tft_font_t;

#define TFT_COLOR_TABLE_ENTRY_BLACK         0
#define TFT_COLOR_TABLE_ENTRY_DARK_RED      1
#define TFT_COLOR_TABLE_ENTRY_DARK_GREEN    2
#define TFT_COLOR_TABLE_ENTRY_DARK_YELLOW   3
#define TFT_COLOR_TABLE_ENTRY_DARK_BLUE     4
#define TFT_COLOR_TABLE_ENTRY_DARK_MAGENTA  5
#define TFT_COLOR_TABLE_ENTRY_DARK_CYAN     6
#define TFT_COLOR_TABLE_ENTRY_LIGHT_GRAY    7
#define TFT_COLOR_TABLE_ENTRY_DARK_GRAY     8
#define TFT_COLOR_TABLE_ENTRY_LIGHT_RED     9
#define TFT_COLOR_TABLE_ENTRY_LIGHT_GREEN   10
#define TFT_COLOR_TABLE_ENTRY_LIGHT_YELLOW  11
#define TFT_COLOR_TABLE_ENTRY_LIGHT_BLUE    12
#define TFT_COLOR_TABLE_ENTRY_LIGHT_MAGENTA 13
#define TFT_COLOR_TABLE_ENTRY_LIGHT_CYAN    14
#define TFT_COLOR_TABLE_ENTRY_WHITE         15

extern const uint16_t tft_color_table[16];

extern const tft_font_t tft_font_6x8;
extern const tft_font_t tft_font_8x12;
extern const tft_font_t tft_font_12x16;

#define TFT_XLATE_COLOR(_r,_g,_b)  ((((_r) & 0xf8) << 8) | (((_g) & 0xfc) << 3) | (((_b) & 0xf8) >> 3))

static inline uint16_t tft_xlate_color(uint8_t r, uint8_t g, uint8_t b)
{
    return TFT_XLATE_COLOR(r, g, b);
}

extern void tft_initialize(void);
extern void tft_draw_point(uint32_t x, uint32_t y, uint16_t color);
extern void tft_draw_line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t color);
extern void tft_draw_rectangle(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t color);
extern void tft_draw_mono(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint8_t *data, uint16_t color0, uint16_t color1, int transparent);
extern void tft_draw_string(uint32_t x, uint32_t y, const char *string, const tft_font_t *font, uint16_t color0, uint16_t color1, int transparent);
extern void tft_fill_rectangle(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t color);
extern void tft_write_image(uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint8_t *data, const uint16_t *palette, int compressed);

#endif /* _TFT_H */

