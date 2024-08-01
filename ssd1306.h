#ifndef _SSD1306_H
#define _SSD1306_H

#include <stdint.h>

// Colors
#define BLACK SSD1306_BLACK		///< Draw 'off' pixels
#define WHITE SSD1306_WHITE		///< Draw 'on' pixels
#define INVERSE SSD1306_INVERSE ///< Invert pixels

/// fit into the SSD1306_ naming scheme
#define SSD1306_BLACK 0	  ///< Draw 'off' pixels
#define SSD1306_WHITE 1	  ///< Draw 'on' pixels
#define SSD1306_INVERSE 2 ///< Invert pixels

#define SSD1306_EXTERNALVCC 0x01  ///< External display voltage source
#define SSD1306_SWITCHCAPVCC 0x02 ///< Gen. display voltage from 3.3V

struct ssd1306_oled
{
	uint8_t i2c_addr;

	uint16_t width, height;
	uint8_t rotation, inversion;

	uint8_t *frameBuffer;
};
typedef struct ssd1306_oled ssd1306_oled;

// Init function
void oled_init(ssd1306_oled *oled, uint8_t addr, uint8_t vccstate, uint16_t w, uint16_t h);

// Command functions
void oled_set_invert(ssd1306_oled *oled, uint8_t i);
void oled_invert(ssd1306_oled *oled);

// Graphics Functions
void oled_flush(ssd1306_oled *oled);
void oled_clear(ssd1306_oled *oled);

void oled_draw_pixel(ssd1306_oled *oled, int16_t x, int16_t y, uint16_t color);
void oled_draw_bitmap(
	ssd1306_oled *oled, int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);

#endif /* _SSD1306_H */
