#include "ssd1306.h"

#include <stddef.h>
#include <stdint.h>
#define I2C_GPIO GPIOA
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 1

#include "ch32v003_I2C_bitbang.h"
#define SSD1306_ADDR_CMD 0x00
#define SSD1306_ADDR_FB 0x40

#define SSD1306_MEMORYMODE 0x20			 ///< See datasheet
#define SSD1306_COLUMNADDR 0x21			 ///< See datasheet
#define SSD1306_PAGEADDR 0x22			 ///< See datasheet
#define SSD1306_SETCONTRAST 0x81		 ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D			 ///< See datasheet
#define SSD1306_SEGREMAP 0xA0			 ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5		 ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6		 ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7		 ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8		 ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE			 ///< See datasheet
#define SSD1306_DISPLAYON 0xAF			 ///< See datasheet
#define SSD1306_COMSCANINC 0xC0			 ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8			 ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3	 ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5	 ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9		 ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA			 ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB		 ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00  ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40  ///< See datasheet

#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26			  ///< Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27				  ///< Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29 ///< Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A  ///< Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL 0x2E					  ///< Stop scroll
#define SSD1306_ACTIVATE_SCROLL 0x2F					  ///< Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3			  ///< Set scroll range

#define ssd1306_swap(a, b) \
	(((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation

uint8_t _fb[128 * 64 / 8];

void ssd1306_command1(ssd1306_oled *oled, uint8_t c)
{
	i2c_write_mem(oled->i2c_addr, SSD1306_ADDR_CMD, &c, 1);
}

void ssd1306_command_list(ssd1306_oled *oled, const uint8_t *c, uint8_t n)
{
	i2c_write_mem(oled->i2c_addr, SSD1306_ADDR_CMD, c, n);
}

void ssd1306_framebuf_transmit(ssd1306_oled *oled)
{
	i2c_write_mem(oled->i2c_addr, SSD1306_ADDR_FB, oled->frameBuffer, 1024);
}

void oled_init(ssd1306_oled *oled, uint8_t addr, uint8_t vccstate, uint16_t w, uint16_t h)
{
	i2c_gpio_init();

	oled->width = w;
	oled->height = h;
	oled->inversion = 0;
	oled->rotation = 0;
	oled->i2c_addr = addr;
	oled->frameBuffer = _fb;
	//	oled_clear( oled );

	static const uint8_t init1[] = {SSD1306_DISPLAYOFF,			// 0xAE
									SSD1306_SETDISPLAYCLOCKDIV, // 0xD5
									0x80,						// the suggested ratio 0x80
									SSD1306_SETMULTIPLEX};		// 0xA8

	ssd1306_command_list(oled, init1, sizeof(init1));
	ssd1306_command1(oled, oled->height - 1);

	static const uint8_t init2[] = {SSD1306_SETDISPLAYOFFSET,	// 0xD3
									0x0,						// no offset
									SSD1306_SETSTARTLINE | 0x0, // line #0
									SSD1306_CHARGEPUMP};		// 0x8D
	ssd1306_command_list(oled, init2, sizeof(init2));

	ssd1306_command1(oled, (vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

	static const uint8_t init3[] = {SSD1306_MEMORYMODE, // 0x20
									0x00,				// 0x0 act like ks0108
									SSD1306_SEGREMAP | 0x1, SSD1306_COMSCANDEC};
	ssd1306_command_list(oled, init3, sizeof(init3));

	uint8_t comPins = 0x02;
	uint8_t contrast = 0x8F;

	if ((oled->width == 128) && (oled->height == 32))
	{
		comPins = 0x02;
		contrast = 0x8F;
	}
	else if ((oled->width == 128) && (oled->height == 64))
	{
		comPins = 0x12;
		contrast = (vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
	}
	else if ((oled->width == 96) && (oled->height == 16))
	{
		comPins = 0x2; // ada x12
		contrast = (vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF;
	}
	else
	{
		// Other screen varieties -- TBD
	}

	ssd1306_command1(oled, SSD1306_SETCOMPINS);
	ssd1306_command1(oled, comPins);
	ssd1306_command1(oled, SSD1306_SETCONTRAST);
	ssd1306_command1(oled, contrast);

	ssd1306_command1(oled, SSD1306_SETPRECHARGE); // 0xd9
	ssd1306_command1(oled, (vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
	static const uint8_t init5[] = {SSD1306_SETVCOMDETECT, // 0xDB
									0x40,
									SSD1306_DISPLAYALLON_RESUME,				   // 0xA4
									SSD1306_NORMALDISPLAY,						   // 0xA6
									SSD1306_DEACTIVATE_SCROLL, SSD1306_DISPLAYON}; // Main screen turn on
	ssd1306_command_list(oled, init5, sizeof(init5));
}

void oled_set_invert(ssd1306_oled *oled, uint8_t i)
{
	oled->inversion = i;
	ssd1306_command1(oled, i ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void oled_invert(ssd1306_oled *oled)
{
	oled->inversion = oled->inversion ? 0 : 1;
	ssd1306_command1(oled, oled->inversion ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void oled_clear(ssd1306_oled *oled)
{
	memset(oled->frameBuffer, 0, oled->width * ((oled->height + 7) / 8));
}

void oled_draw_bitmap(
	ssd1306_oled *oled, int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
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
				oled_draw_pixel(oled, x + i, y, color);
		}
	}
}

void oled_draw_pixel(ssd1306_oled *oled, int16_t x, int16_t y, uint16_t color)
{
	if ((x >= 0) && (x < oled->width) && (y >= 0) && (y < oled->height))
	{
		// Pixel is in-bounds. Rotate coordinates if needed.
		switch (oled->rotation)
		{
		case 1:
			ssd1306_swap(x, y);
			x = oled->width - x - 1;
			break;
		case 2:
			x = oled->width - x - 1;
			y = oled->height - y - 1;
			break;
		case 3:
			ssd1306_swap(x, y);
			y = oled->height - y - 1;
			break;
		}
		switch (color)
		{
		case SSD1306_WHITE:
			oled->frameBuffer[x + (y / 8) * oled->width] |= (1 << (y & 7));
			break;
		case SSD1306_BLACK:
			oled->frameBuffer[x + (y / 8) * oled->width] &= ~(1 << (y & 7));
			break;
		case SSD1306_INVERSE:
			oled->frameBuffer[x + (y / 8) * oled->width] ^= (1 << (y & 7));
			break;
		}
	}
}

void oled_flush(ssd1306_oled *oled)
{
	static const uint8_t dlist1[] = {SSD1306_PAGEADDR, 0,	 // Page start address
									 0xFF,					 // Page end (not really, but works here)
									 SSD1306_COLUMNADDR, 0}; // Column start address
	ssd1306_command_list(oled, dlist1, sizeof(dlist1));
	ssd1306_command1(oled, oled->width - 1); // Column end address
	ssd1306_framebuf_transmit(oled);
}
