#ifndef CH32V003_I2C_BITBANG_H
#define CH32V003_I2C_BITBANG_H

#include "ch32v003fun.h"
#include <stdint.h>

/*######## library usage and configuration

in the .c files that use this library, you'll need to #define the used GPIO and pins _before_ the #include
"ch32v003_I2C_bitbang.h"

like so:
#define I2C_GPIO GPIOC
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3

*/

#define SET_SCL I2C_GPIO->BSHR = (1 << I2C_SCL_PIN)
#define RESET_SCL I2C_GPIO->BSHR = (1 << (16 + I2C_SCL_PIN))
#define READ_SCL ((I2C_GPIO->INDR >> I2C_SCL_PIN) & 0b1)

#define SET_SDA I2C_GPIO->BSHR = (1 << I2C_SDA_PIN)
#define RESET_SDA I2C_GPIO->BSHR = (1 << (16 + I2C_SDA_PIN))
#define READ_SDA ((I2C_GPIO->INDR >> I2C_SDA_PIN) & 0b1)

#define NOP asm("NOP")

#define I2C_DELAY                 \
	for (int i = 1; i <= 10; i++) \
	NOP

// initialize the GPIO pins
void i2c_gpio_init()
{
	I2C_GPIO->CFGLR &= (~(0xf << (4 * I2C_SCL_PIN))) & (~(0xf << (4 * I2C_SDA_PIN)));

	I2C_GPIO->CFGLR |= ((GPIO_Speed_50MHz | GPIO_CNF_OUT_OD) << (4 * I2C_SCL_PIN)) |
					   ((GPIO_Speed_50MHz | GPIO_CNF_OUT_OD) << (4 * I2C_SDA_PIN));
}

// transmit a start condition
void i2c_start()
{
	SET_SDA;
	I2C_DELAY;
	SET_SCL;
	I2C_DELAY;
	RESET_SDA;
	I2C_DELAY;
	RESET_SCL;
	I2C_DELAY;
}

// transmit a stop condition
void i2c_stop()
{
	RESET_SDA;
	I2C_DELAY;
	SET_SCL;
	I2C_DELAY;
	SET_SDA;
	I2C_DELAY;
}

// write a single byte
uint8_t i2c_tx_byte(uint8_t dat)
{

	for (uint8_t i = 8; i; i--)
	{
		if (dat & 0x80)
			SET_SDA;
		else
			RESET_SDA;

		dat <<= 1; // Move
		I2C_DELAY;
		SET_SCL;
		I2C_DELAY;
		RESET_SCL;
		I2C_DELAY;
	}
	SET_SDA;
	SET_SCL;
	I2C_DELAY;
	uint8_t ack = !READ_SDA; // Acknowledge bit
	RESET_SCL;
	I2C_DELAY;
	//	RESET_SDA;
	return ack;
}

// read a single byte, with or without ack
uint8_t i2c_rx_byte(uint8_t ack)
{
	uint8_t dat = 0;
	SET_SDA;
	for (uint8_t i = 0; i < 8; i++)
	{
		dat <<= 1;
		do
		{
			SET_SCL;
		} while (READ_SCL == 0); // clock stretching
		I2C_DELAY;
		if (READ_SDA)
			dat |= 1;
		I2C_DELAY;
		RESET_SCL;
	}
	if (ack)
		RESET_SDA;
	else
		SET_SDA;
	SET_SCL;
	I2C_DELAY;
	RESET_SCL;
	SET_SDA;
	I2C_DELAY;
	return (dat);
}

// write bytes to device register
uint8_t i2c_write_mem(uint8_t addr, uint8_t reg, uint8_t *buf, size_t count)
{
	i2c_start();
	i2c_tx_byte((addr << 1) | 0x00);
	i2c_tx_byte(reg);

	for (size_t i = 0; i < count; i++)
		i2c_tx_byte(buf[i]);
	I2C_DELAY;

	i2c_stop();
}

// read bytes from device register
uint8_t i2c_read_mem(uint8_t addr, uint8_t reg, uint8_t *buf, size_t count)
{
	i2c_start();
	i2c_tx_byte((addr << 1) | 0x00);
	i2c_tx_byte(reg);

	i2c_start();
	i2c_tx_byte((addr << 1) | 0x01);
	for (size_t i = 0; i < count; i++)
	{
		if (i == count - 1)
			buf[i] = i2c_rx_byte(0);
		else
			buf[i] = i2c_rx_byte(1);
	}
	I2C_DELAY;

	i2c_stop();
}

#endif