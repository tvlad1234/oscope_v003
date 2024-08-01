#include "gfx.h"
#include "font.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) \
	{                       \
		int16_t t = a;      \
		a = b;              \
		b = t;              \
	}
#endif

void gfx_draw_line(gfx_inst *gfx, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{

	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep)
	{
		_swap_int16_t(x0, y0);
		_swap_int16_t(x1, y1);
	}

	if (x0 > x1)
	{
		_swap_int16_t(x0, x1);
		_swap_int16_t(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1)
	{
		ystep = 1;
	}
	else
	{
		ystep = -1;
	}

	for (; x0 <= x1; x0++)
	{
		if (steep)
		{
			gfx_draw_pixel(gfx, y0, x0, color);
		}
		else
		{
			gfx_draw_pixel(gfx, x0, y0, color);
		}
		err -= dy;
		if (err < 0)
		{
			y0 += ystep;
			err += dx;
		}
	}
}

void gfx_draw_fast_v_line(gfx_inst *gfx, int16_t x, int16_t y, int16_t h, uint16_t color)
{
	gfx_draw_line(gfx, x, y, x, y + h - 1, color);
}

void gfx_draw_fast_h_line(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, uint16_t color)
{
	gfx_draw_line(gfx, x, y, x + w - 1, y, color);
}

void gfx_draw_circle(gfx_inst *gfx, int16_t x0, int16_t y0, int16_t r, uint16_t color)
{

	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	gfx_draw_pixel(gfx, x0, y0 + r, color);
	gfx_draw_pixel(gfx, x0, y0 - r, color);
	gfx_draw_pixel(gfx, x0 + r, y0, color);
	gfx_draw_pixel(gfx, x0 - r, y0, color);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		gfx_draw_pixel(gfx, x0 + x, y0 + y, color);
		gfx_draw_pixel(gfx, x0 - x, y0 + y, color);
		gfx_draw_pixel(gfx, x0 + x, y0 - y, color);
		gfx_draw_pixel(gfx, x0 - x, y0 - y, color);
		gfx_draw_pixel(gfx, x0 + y, y0 + x, color);
		gfx_draw_pixel(gfx, x0 - y, y0 + x, color);
		gfx_draw_pixel(gfx, x0 + y, y0 - x, color);
		gfx_draw_pixel(gfx, x0 - y, y0 - x, color);
	}
}

void gfx_draw_rect(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	gfx_draw_fast_h_line(gfx, x, y, w, color);
	gfx_draw_fast_h_line(gfx, x, y + h - 1, w, color);
	gfx_draw_fast_v_line(gfx, x, y, h, color);
	gfx_draw_fast_v_line(gfx, x + w - 1, y, h, color);
}

void gfx_fill_rect_internal(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	for (int16_t i = x; i < x + w; i++)
		gfx_draw_fast_v_line(gfx, i, y, h, color);
}

void gfx_fill_rect(gfx_inst *gfx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	if (gfx->fill_rect_fun)
		(*(gfx->fill_rect_fun))(gfx->disp_ptr, x, y, w, h, color);
	else
		gfx_fill_rect_internal(gfx, x, y, w, h, color);
}

void gfx_fill(gfx_inst *gfx, uint16_t color)
{
	gfx_fill_rect(gfx, 0, 0, gfx->width, gfx->height, color);
}

void gfx_clear(gfx_inst *gfx)
{
	gfx_fill_rect(gfx, 0, 0, gfx->width, gfx->height, gfx->clear_color);
}

void gfx_draw_char(gfx_inst *gfx, int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y)
{
	if ((x >= gfx->width) ||		  // Clip right
		(y >= gfx->height) ||		  // Clip bottom
		((x + 6 * size_x - 1) < 0) || // Clip left
		((y + 8 * size_y - 1) < 0))	  // Clip top
		return;

	if (c >= 176)
		c++; // Handle 'classic' charset behavior

	for (int8_t i = 0; i < 5; i++)
	{ // Char bitmap = 5 columns
		uint8_t line = font[c * 5 + i];
		for (int8_t j = 0; j < 8; j++, line >>= 1)
		{
			if (line & 1)
			{
				if (size_x == 1 && size_y == 1)
					gfx_draw_pixel(gfx, x + i, y + j, color);
				else
					gfx_fill_rect(gfx, x + i * size_x, y + j * size_y, size_x, size_y,
								  color);
			}
			else if (bg != color)
			{
				if (size_x == 1 && size_y == 1)
					gfx_draw_pixel(gfx, x + i, y + j, bg);
				else
					gfx_fill_rect(gfx, x + i * size_x, y + j * size_y, size_x, size_y,
								  bg);
			}
		}
	}
	if (bg != color)
	{ // If opaque, draw vertical line for last column
		if (size_x == 1 && size_y == 1)
			gfx_draw_fast_v_line(gfx, x + 5, y, 8, bg);
		else
			gfx_fill_rect(gfx, x + 5 * size_x, y, size_x, 8 * size_y, bg);
	}
}

void gfx_set_cursor(gfx_inst *gfx, int16_t x, int16_t y)
{
	gfx->cursor_x = x;
	gfx->cursor_y = y;
}

void gfx_set_text_color(gfx_inst *gfx, uint16_t c, uint16_t bg)
{
	gfx->color_text = c;
	gfx->color_bg = bg;
}

void gfx_set_text_size(gfx_inst *gfx, uint16_t s)
{
	gfx->textsize_x = s;
	gfx->textsize_y = s;
}

void gfx_write_char(gfx_inst *gfx, char c)
{
	if (c == '\n')
	{										  // Newline?
		gfx->cursor_x = 0;					  // Reset x to zero,
		gfx->cursor_y += gfx->textsize_y * 8; // advance y one line
	}
	else if (c != '\r')
	{ // Ignore carriage returns
		if ((gfx->cursor_x + gfx->textsize_x * 6) > gfx->width)
		{										  // Off right?
			gfx->cursor_x = 0;					  // Reset x to zero,
			gfx->cursor_y += gfx->textsize_y * 8; // advance y one line
		}
		gfx_draw_char(gfx, gfx->cursor_x, gfx->cursor_y, c, gfx->color_text, gfx->color_bg, gfx->textsize_x,
					  gfx->textsize_y);
		gfx->cursor_x += gfx->textsize_x * 6; // Advance x one char
	}
}

void gfx_print_string(gfx_inst *gfx, char s[])
{
	uint8_t n = strlen(s);
	for (int i = 0; i < n; i++)
		gfx_write_char(gfx, s[i]);
}

void gfx_printf(gfx_inst *gfx, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	mini_vsnprintf(gfx->print_buf, GFX_PRINTBUF_SIZE, format, args);
	gfx_print_string(gfx, gfx->print_buf);
	va_end(args);
}

void gfx_print_float(gfx_inst *gfx, float v)
{
	int intV = (int)v;
	v *= 100.0f;
	int decV = (int)v % 100;
	if (decV < 0)
		decV -= 2 * decV;
	if (v < 0 && intV == 0)
		gfx_print_string(gfx, "-");
	gfx_printf(gfx, "%d.%d", intV, decV);
}

void gfx_draw_bmp_1bpp(gfx_inst *gfx, int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
{
	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;
	for (int16_t j = 0; j < h; j++, y++)
	{
		for (int16_t i = 0; i < w; i++)
		{
			if (i & 7)
				byte <<= 1;
			else
				byte = bitmap[j * byteWidth + i / 8];
			if (byte & 0x80)
				gfx_draw_pixel(gfx, x + i, y, color);
		}
	}
}
