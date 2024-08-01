/*
 * CH32V003 oscilloscope
 * by tvlad1234
 */

#include "ch32v003fun.h"
#include <stdio.h>
#include <stdlib.h>

#include "gfx.h"
#include "ssd1306.h"

#define AN_IN_GPIO GPIOC
#define AN_IN_PIN 4
#define AN_IN_CH 2

#define BTN_L_GPIO GPIOC
#define BTN_L_PIN 2

#define BTN_C_GPIO GPIOC
#define BTN_C_PIN 6

#define BTN_R_GPIO GPIOD
#define BTN_R_PIN 4

#define RISING 1
#define FALLING 0

#define volts_from_adc(s) (((3.3f * s / 1023.0f) - 1.65f) * 2.0f)

// ADC capture buffers
#define BUFFER_LENGTH 60
volatile uint16_t buffer1[BUFFER_LENGTH] = {0};
volatile uint16_t buffer2[BUFFER_LENGTH] = {0};

volatile uint8_t dma_ready = 0; // DMA ready (conversion done) flag
volatile uint16_t *p;			// variable used for swapping pointers around

// available ADC clock dividers
const uint8_t availableAdcDivs[] = {2, 4, 6, 8, 12, 16, 24, 32, 64, 96, 128, 0};

// pointers to the two buffers
volatile uint16_t *writeBuffer = buffer1;
volatile uint16_t *readBuffer = buffer2;

// trigger settings and flag
volatile uint16_t trigLevel = 512;
volatile uint8_t trig = RISING;
volatile uint8_t trigged = 0;

volatile int wf_cnt = 0;

float sampPer;

enum
{
	UI_NONE,
	UI_TDIV,
	UI_TRIGLEV,
	UI_TRIGSLOPE,
	UI_END
};

uint8_t ui_selector = UI_NONE;

// OLED and GFX instances
ssd1306_oled myOled;
gfx_inst myGfx;

// link the oled to the gfx
void oled_link_gfx(ssd1306_oled *oled, gfx_inst *gfx)
{
	gfx->disp_ptr = oled;
	gfx->height = oled->height;
	gfx->width = oled->width;
	gfx->pixel_draw_fun = oled_draw_pixel;
	gfx->flush_fun = oled_flush;

	gfx_set_text_color(gfx, WHITE, BLACK);
	gfx_set_text_size(gfx, 1);
	gfx_set_cursor(gfx, 0, 0);
}

// arms the DMA to initiate waveform capture from ADC
void arm_dma()
{
	// Swap the buffer pointers around
	dma_ready = 0;
	p = readBuffer;
	readBuffer = writeBuffer;
	writeBuffer = p;

	// Start DMA clock
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	// Setup DMA Channel 1 (adc triggered) as reading, 16-bit, linear buffer
	DMA1_Channel1->CFGR =
		DMA_DIR_PeripheralSRC | DMA_MemoryInc_Enable | DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord;

	// No of samples to get before irq
	DMA1_Channel1->CNTR = BUFFER_LENGTH;

	// Source
	DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;

	// Destination
	DMA1_Channel1->MADDR = (uint32_t)writeBuffer;

	// Enable IRQ and DMA channel
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	DMA1_Channel1->CFGR |= DMA_IT_TC | DMA_CFGR1_EN;
}

// initializes ADC at startup
void init_adc()
{
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide by 2
	RCC->CFGR0 &= ~(0x1F << 11);

	// Enable GPIOC and set C4 as analog input
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
	AN_IN_GPIO->CFGLR &= ~(0xf << (4 * AN_IN_PIN)); // CNF = 00: Analog, MODE = 00: Input

	// Enable the ADC1 peripheral clock
	RCC->APB2PCENR |= RCC_APB2Periph_ADC1;

	// Reset the ADC to init all regs
	RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

	// Set sequencer to channel 2 only
	ADC1->RSQR3 = AN_IN_CH;

	// Possible times: 0->3,1->9,2->15,3->30,4->43,5->57,6->73,7->241 cycles
	ADC1->SAMPTR2 = 1 /*9 cycles*/ << (3 /*offset per channel*/ * 2 /*channel*/);

	// turn on ADC
	ADC1->CTLR2 |= ADC_ADON;

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while (ADC1->CTLR2 & ADC_RSTCAL)
		;

	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while (ADC1->CTLR2 & ADC_CAL)
		;

	// enable scanning, analog watchdog for single regular channel and interrupt
	ADC1->CTLR1 |= ADC_SCAN | ADC_AWDSGL | ADC_AWDEN | ADC_AWDIE;

	// set analog watchdog channel
	ADC1->CTLR1 |= AN_IN_CH;

	// Enable continuous conversion and DMA
	ADC1->CTLR2 |= ADC_CONT | ADC_DMA | ADC_EXTSEL;

	// clear analog watchdog flag
	ADC1->STATR = ~ADC_FLAG_AWD;

	// set analog watchdog triggering window below trigger level
	ADC1->WDLTR = trigLevel;
	ADC1->WDHTR = 1023;

	// enable ADC interrupt
	NVIC_EnableIRQ(ADC_IRQn);

	// start conversion
	ADC1->CTLR2 |= ADC_SWSTART;
}

volatile uint8_t trig_sm = 0;

// Interrupt handler for the ADC analog watchdog
void ADC1_IRQHandler(void) __attribute__((interrupt));
void ADC1_IRQHandler()
{
	if (ADC1->STATR & ADC_FLAG_AWD)
	{

		if (trig_sm == 0)
		{
			trig_sm = 1;

			// set watchdog triggering window above trigger level

			if (trig == RISING)
			{
				ADC1->WDLTR = 0;
				ADC1->WDHTR = trigLevel;
			}
			else
			{
				ADC1->WDLTR = trigLevel;
				ADC1->WDHTR = 1023;
			}
			ADC1->STATR = ~ADC_FLAG_AWD; // clear the watchdog flag
		}

		// and fire the DMA once we go above the threshold
		else
		{
			ADC1->CTLR1 &= ~ADC_AWDIE & ~ADC_AWDEN; // disable the watchdog and watchdog interrupt
			trig_sm = 0;

			// set watchdog triggering window below level
			if (trig == RISING)
			{
				ADC1->WDLTR = trigLevel;
				ADC1->WDHTR = 1023;
			}
			else
			{
				ADC1->WDLTR = 0;
				ADC1->WDHTR = trigLevel;
			}
			// start capture
			arm_dma();
		}
	}
}

// Interrupt handler for the DMA
void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel1_IRQHandler()
{
	if (DMA1->INTFR & DMA1_FLAG_TC1)
	{
		DMA1->INTFCR = DMA_CTCIF1; // clear Transfer Complete interrupt
		wf_cnt++;
		dma_ready = 1; // raise ready flag (conversion finished)
	}
}

// Draws a dotted horizontal line, used for drawing the graticule
void dotted_h_line(gfx_inst *gfx, int x, int y, int l)
{
	for (int i = 0; i <= l; i++)
	{
		if (i % 2)
			gfx_draw_pixel(gfx, x + i, y, WHITE);
		else
			gfx_draw_pixel(gfx, x + i, y, BLACK);
	}
}

// Draws a dotted vertical line, used for drawing the graticule
void dotted_v_line(gfx_inst *gfx, int x, int y, int l)
{
	for (int i = 0; i <= l; i++)
	{
		if (i % 2)
			gfx_draw_pixel(gfx, x, y + i, WHITE);
		else
			gfx_draw_pixel(gfx, x, y + i, BLACK);
	}
}

// Draws graticule, specified number of divisions and pixels/division
void draw_graticule(gfx_inst *gfx, uint16_t divx, uint16_t divy, uint16_t pix)
{
	uint16_t wit = divx * pix;
	uint16_t hei = divy * pix;
	for (int i = 0; i <= wit; i += pix)
		dotted_v_line(gfx, i, 0, hei);
	for (int i = 0; i <= hei; i += pix)
		dotted_h_line(gfx, 0, i, wit);
}

float measure_frequency(uint16_t *readBuffer, uint16_t trigLevel, float sampPer)
{
	uint8_t trigged = 0;
	int trigPoint = 0; // trigger point in the captured waveform
	int trigPoint2;	   // another trigger point, this will help us determine the period (so the frequency) of the signal

	// Look for trigger
	// The trigged variable will be 0 if we're not triggering, 1 if we only found 1 trigger point and 2 if we
	// have found at least two trigger points

	for (int i = 1; i < BUFFER_LENGTH && trigged != 2;
		 i++) // we're looking for trigger points in the first half of the buffer
		if ((trig == RISING && readBuffer[i] >= trigLevel && readBuffer[i - 1] < trigLevel) ||
			(trig == FALLING && readBuffer[i] <= trigLevel && readBuffer[i - 1] > trigLevel))
		{
			if (!trigged) // Looking for the first trigger point
			{
				trigPoint = i;
				trigged = 1;
			}
			else // Looking for the second one
			{
				trigPoint2 = i;
				trigged = 2;
			}
		}

	if (trigged == 2)
	{
		float sigPer = sampPer * (trigPoint2 - trigPoint); // we compute the period of the signal in uS
		float measuredFreq = 1000000.0f / sigPer;		   // and then we convert it into frequency, in Hz
		return measuredFreq;
	}
	return 0.0f;
}

void adc_set_div(uint8_t div)
{
	sampPer = 20.0f * (1 / 48.0f) * div;

	RCC->CFGR0 &= ~((uint32_t)(0b11111) << 11);
	switch (div)
	{
	case 2:
		break;
	case 4:
		RCC->CFGR0 |= RCC_ADCPRE_DIV4;
		break;
	case 8:
		RCC->CFGR0 |= RCC_ADCPRE_DIV8;
		break;
	case 6:
		RCC->CFGR0 |= RCC_ADCPRE_DIV6;
	case 12:
		RCC->CFGR0 |= 0xA000;
		break;
	case 16:
		RCC->CFGR0 |= 0xE000;
		break;
	case 24:
		RCC->CFGR0 |= 0xA800;
		break;
	case 32:
		RCC->CFGR0 |= 0xE800;
		break;
	case 64:
		RCC->CFGR0 |= 0xF000;
		break;
	case 96:
		RCC->CFGR0 |= 0xB800;
		break;
	case 128:
		RCC->CFGR0 |= 0xF800;
		break;
	default:
		break;
	}
}

int main()
{
	SystemInit();
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

	BTN_C_GPIO->CFGLR &= ~(0xf << (4 * BTN_C_PIN));
	BTN_C_GPIO->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * BTN_C_PIN);
	BTN_C_GPIO->BSHR = 1 << (BTN_C_PIN);

	BTN_L_GPIO->CFGLR &= ~(0xf << (4 * BTN_L_PIN));
	BTN_L_GPIO->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * BTN_L_PIN);
	BTN_L_GPIO->BSHR = 1 << (BTN_L_PIN);

	BTN_R_GPIO->CFGLR &= ~(0xf << (4 * BTN_R_PIN));
	BTN_R_GPIO->CFGLR |= (GPIO_Speed_In | GPIO_CNF_IN_PUPD) << (4 * BTN_R_PIN);
	BTN_R_GPIO->BSHR = 1 << (BTN_R_PIN);

	arm_dma();
	init_adc();
	oled_init(&myOled, 0x3C, SSD1306_SWITCHCAPVCC, 128, 64);
	oled_link_gfx(&myOled, &myGfx);

	uint16_t currentMs;
	uint16_t prevMs = SysTick->CNT / DELAY_MS_TIME;

	uint16_t currentDivMs;
	uint16_t prevDivMs = SysTick->CNT / DELAY_MS_TIME;

	uint8_t divSel = 0;
	adc_set_div(availableAdcDivs[divSel]);

	while (1)
	{
		if (dma_ready)
		{

			/*
			currentDivMs = SysTick->CNT / DELAY_MS_TIME;
			if ( currentDivMs < prevDivMs ) prevDivMs = currentDivMs;

			if ( currentDivMs - prevDivMs > 2000 )
			{
				// bla bla code

				prevDivMs = currentDivMs;
			}
			*/

			if (!(BTN_C_GPIO->INDR & (1 << BTN_C_PIN)))
			{
				ui_selector++;
				if (ui_selector == UI_END)
					ui_selector = UI_NONE;
			}

			if (!(BTN_L_GPIO->INDR & (1 << BTN_L_PIN)))
			{
				switch (ui_selector)
				{
				case UI_TDIV:
					if (divSel > 0)
					{
						divSel--;
						adc_set_div(availableAdcDivs[divSel]);
					}
					break;

				case UI_TRIGLEV:
					if (trigLevel > 10)
						trigLevel -= 10;
					trig_sm = 0;

					// set watchdog triggering window below level
					if (trig == RISING)
					{
						ADC1->WDLTR = trigLevel;
						ADC1->WDHTR = 1023;
					}
					else
					{
						ADC1->WDLTR = 0;
						ADC1->WDHTR = trigLevel;
					}
					break;

				case UI_TRIGSLOPE:
					trig = FALLING;
					break;

				default:
					break;
				}
			}

			if (!(BTN_R_GPIO->INDR & (1 << BTN_R_PIN)))
			{
				switch (ui_selector)
				{
				case UI_TDIV:

					divSel++;
					if (!availableAdcDivs[divSel])
						divSel--;
					adc_set_div(availableAdcDivs[divSel]);
					break;

				case UI_TRIGLEV:
					if (trigLevel < 1013)
						trigLevel += 10;
					trig_sm = 0;

					// set watchdog triggering window below level
					if (trig == RISING)
					{
						ADC1->WDLTR = trigLevel;
						ADC1->WDHTR = 1023;
					}
					else
					{
						ADC1->WDLTR = 0;
						ADC1->WDHTR = trigLevel;
					}
					break;

				case UI_TRIGSLOPE:
					trig = RISING;
					break;

				default:
					break;
				}
			}

			currentMs = SysTick->CNT / DELAY_MS_TIME;
			if (currentMs < prevMs)
				prevMs = currentMs;

			if (currentMs - prevMs > 150 && !(ADC1->STATR & ADC_FLAG_AWD))
			{
				trigged = 0;
				arm_dma();
				prevMs = currentMs;
			}
			else
			{
				trigged = 1;						  // mark triggered
				ADC1->STATR = ~ADC_FLAG_AWD;		  // clear analog watchdog flag
				ADC1->CTLR1 |= ADC_AWDEN | ADC_AWDIE; // enable watchdog again, for next capture
			}
		}

		float vmin = 0;
		float vmax = 0;
		float vAvg = 0;

		draw_graticule(&myGfx, 5, 5, 12);

		// Plot the waveform and retrieve voltage min, max and avg
		for (int i = 0; i < ((BUFFER_LENGTH)-1); i++)
		{
			int h1 = 59 - (readBuffer[i] / 17);
			int h2 = 59 - (readBuffer[i + 1] / 17);

			gfx_draw_line(&myGfx, i, h1, i + 1, h2, WHITE);

			float v = volts_from_adc(readBuffer[i]);

			if (v < vmin || vmin == 0)
				vmin = v;

			if (v > vmax)
				vmax = v;

			vAvg += v;
		}
		vAvg /= (float)((BUFFER_LENGTH)-1);

		float measuredFreq = measure_frequency(readBuffer, trigLevel, sampPer);

		gfx_set_cursor(&myGfx, 66, 0);
		gfx_print_float(&myGfx, vmin);
		gfx_print_string(&myGfx, "V min");

		gfx_set_cursor(&myGfx, 66, 8);
		gfx_print_float(&myGfx, vmax);
		gfx_print_string(&myGfx, "V max");

		gfx_set_cursor(&myGfx, 66, 16);
		gfx_printf(&myGfx, "%d kHz ", (int)measuredFreq / 1000);
		// gfx_print_float(&myGfx, measuredFreq / 1000.0f );
		// gfx_print_string( &myGfx, " kHz" );

		float tdiv = 16 * sampPer;

		if (ui_selector == UI_TDIV)
			gfx_set_text_color(&myGfx, BLACK, WHITE);
		else
			gfx_set_text_color(&myGfx, WHITE, BLACK);

		gfx_set_cursor(&myGfx, 66, 32);
		if (tdiv < 100)
		{
			gfx_print_float(&myGfx, 12 * sampPer);
			gfx_print_string(&myGfx, "us/d");
		}
		else
		{
			gfx_print_float(&myGfx, 16 * sampPer / 1000.0f);
			gfx_print_string(&myGfx, "ms/d");
		}

		if (ui_selector == UI_TRIGLEV)
		{
			gfx_set_text_color(&myGfx, BLACK, WHITE);
			gfx_draw_fast_h_line(&myGfx, 1, 59 - (trigLevel / 17), 59, WHITE);
		}
		else
			gfx_set_text_color(&myGfx, WHITE, BLACK);

		gfx_set_cursor(&myGfx, 66, 40);
		gfx_print_float(&myGfx, volts_from_adc(trigLevel));
		gfx_print_string(&myGfx, "Vtr");

		if (ui_selector == UI_TRIGSLOPE)
			gfx_set_text_color(&myGfx, BLACK, WHITE);
		else
			gfx_set_text_color(&myGfx, WHITE, BLACK);

		if (trig == RISING)
			gfx_print_string(&myGfx, " R");
		else
			gfx_print_string(&myGfx, " F");

		gfx_set_text_color(&myGfx, WHITE, BLACK);

		if (trigged)
		{
			gfx_set_cursor(&myGfx, 66, 48);
			gfx_print_string(&myGfx, "Trig'd");
		}

		/*
		gfx_set_cursor( &myGfx, 66, 48 );
		gfx_printf( &myGfx, "%d wfs", wf_cnt );
		*/

		gfx_flush(&myGfx);
		gfx_set_cursor(&myGfx, 0, 0);
		gfx_clear(&myGfx);
	}
}
