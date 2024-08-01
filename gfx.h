#ifndef _GFX_H
#define _GFX_H

#include <stdint.h>
#include <string.h>

#define GFX_PRINTBUF_SIZE 64

struct gfx_inst
{
	void *disp_ptr;
	uint16_t width, height;

	int16_t cursor_x, cursor_y;
	uint16_t textsize_x, textsize_y;
	char print_buf[GFX_PRINTBUF_SIZE];

	uint16_t color_text, color_bg, clear_color;

	void (*pixel_draw_fun)(void *, int16_t, int16_t, uint16_t);
	void (*fill_rect_fun)(void *, int16_t, int16_t, int16_t, int16_t, uint16_t);
	void (*flush_fun)(void *);
};

typedef struct gfx_inst gfx_inst;

#define gfx_draw_pixel(gfx_ptr, x, y, col) (*((gfx_ptr)->pixel_draw_fun))((gfx_ptr)->disp_ptr, x, y, col)

#define gfx_flush(gfx_ptr) (*((gfx_ptr)->flush_fun))((gfx_ptr)->disp_ptr)

void gfx_clear(gfx_inst *gfx);
void gfx_fill(gfx_inst *gfx, uint16_t color);

void gfx_draw_line(gfx_inst *gfx, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void gfx_draw_fast_v_line(gfx_inst *gfx, int16_t x, int16_t y, int16_t h, uint16_t color);
void gfx_draw_fast_h_line(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, uint16_t color);

void gfx_draw_circle(gfx_inst *gfx, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void gfx_draw_rect(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void gfx_fill_rect(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

void gfx_draw_char(gfx_inst *gfx, int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
void gfx_write_char(gfx_inst *gfx, char c);

void gfx_print_string(gfx_inst *gfx, char s[]);
void gfx_printf(gfx_inst *gfx, const char *format, ...);
void gfx_print_float(gfx_inst *gfx, float v);

void gfx_set_cursor(gfx_inst *gfx, int16_t x, int16_t y);
void gfx_set_text_color(gfx_inst *gfx, uint16_t c, uint16_t bg);
void gfx_set_text_size(gfx_inst *gfx, uint16_t s);

void gfx_draw_bmp_1bpp(gfx_inst *gfx, int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);

#endif /* _GFX_H */
