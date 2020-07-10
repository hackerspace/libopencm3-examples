/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "st7789.h"
#include "systick.h"
#include "config.h"

#define TFT_MAD_RGB		0x08
#define TFT_MAD_MY		0x80
#define TFT_MAD_MX		0x40
#define TFT_MAD_MV		0x20

#define ST7789_SLPOUT		0x11
#define ST7789_NORON		0x13
#define ST7789_MADCTL		0x36	/* Memory data access control */
#define ST7789_COLMOD		0x3a
#define ST7789_PORCTRL		0xb2	/* Porch control */
#define ST7789_GCTRL		0xb7	/* Gate control */
#define ST7789_VCOMS		0xbb	/* VCOMS setting */
#define ST7789_LCMCTRL		0xc0	/* LCM control */
#define ST7789_VDVVRHEN		0xc2	/* VDV and VRH command enable */
#define ST7789_VRHS		0xc3	/* VRH set */
#define ST7789_VDVSET		0xc4	/* VDV setting */
#define ST7789_FRCTR2		0xc6	/* FR Control 2 */
#define ST7789_PWCTRL1		0xd0	/* Power control 1 */
#define ST7789_PVGAMCTRL	0xe0	/* Positive voltage gamma control */
#define ST7789_NVGAMCTRL	0xe1	/* Negative voltage gamma control */
#define ST7789_INVON		0x21
#define ST7789_DISPON		0x29
#define ST7789_CASET		0x2a
#define ST7789_RASET		0x2b
#define ST7789_RAMWR		0x2c

static uint16_t colstart;
static uint16_t rowstart;
static uint16_t cols;
static uint16_t rows;
static uint8_t rotation;

static inline void data(void)
{
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_DC);
}

static void send(uint8_t dat)
{
	spi_write(ST7789_SPI, dat);
	spi_read(ST7789_SPI);

}

static void command(uint8_t dat)
{
	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_DC);
	send(dat);
}

static void send16(uint16_t dat)
{
	send(dat >> 8);
	send(dat);
}

void st7789_set_addrs(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
	command(ST7789_CASET);
	data();
	send16(x1 + colstart);
	send16(x2 + colstart);
	command(ST7789_RASET);
	data();
	send16(y1 + rowstart);
	send16(y2 + rowstart);
	command(ST7789_RAMWR);
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
}

static void st7789_set_rotation(uint8_t m)
{
	rotation = m % 4;
	command(ST7789_MADCTL);
	data();
	switch (rotation) {
	case 0:
		colstart = DISPLAY_COLSTART;
		rowstart = DISPLAY_ROWSTART;
		cols  = DISPLAY_ROWS;
		rows = DISPLAY_COLS;
		send(TFT_MAD_RGB);
		break;

	case 1:
		colstart = DISPLAY_ROWSTART;
		rowstart = DISPLAY_COLSTART + 1;
		cols  = DISPLAY_COLS;
		rows = DISPLAY_ROWS;
		send(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_RGB);
		break;
	case 2:
		colstart = DISPLAY_COLSTART;
		rowstart = DISPLAY_ROWSTART;
		cols  = DISPLAY_ROWS;
		rows = DISPLAY_COLS;
		send(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_RGB);
		break;
	case 3:
		colstart = DISPLAY_ROWSTART;
		rowstart = DISPLAY_COLSTART;
		cols  = DISPLAY_COLS;
		rows = DISPLAY_ROWS;
		send(TFT_MAD_MV | TFT_MAD_MY | TFT_MAD_RGB);
		break;
	}
}

void st7789_send_data(const uint8_t *bytes, int len)
{
	if (len % 2)
		while(1); /* Bad. */

	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
	data();
	while (len) {
		send(bytes[1]);
		send(bytes[0]);
		bytes += 2;
		len -= 2;
	}
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
}

static void display_splash(void)
{
	unsigned int i;
	static const uint8_t splash[] = {
		#include "splash.h"
	};


	st7789_set_addrs(0, 0, cols - 1, rows - 1);

	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
	data();
	for (i = 0; i < sizeof(splash)/sizeof(splash[0]); i++) {
		uint16_t c = 0;
		unsigned int n;

		c |= (splash[i] & 0xe0) << 8; // R3 ->
		c |= (splash[i] & 0xc0) << 5; //    -> R5

		c |= (splash[i] & 0x1c) << 6; // G3 ->
		c |= (splash[i] & 0x1c) << 3; //    -> G6

		c |= (splash[i] & 0x03) << 3; // B2 ->
		c |= (splash[i] & 0x03) << 1; //    ->
		c |= (splash[i] & 0x03) >> 1; //    -> B5

		switch (splash[i]) {
		case 0x00:
		case 0xff:
			n = splash[i++ + 1];
			break;
		default:
			n = 1;
		}
		while (n--)
			send16(c);
	}
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);
}

void st7789_init(void)
{
	/* Initialize the SPI bus */
	rcc_periph_clock_enable(ST7789_GPIO_CLOCK);
	rcc_periph_clock_enable(ST7789_SPI_CLOCK);


	gpio_set_mode(GPIOA,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      GPIO5 |
		      GPIO7);
	gpio_set_mode(ST7789_GPIO_BLOCK,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      ST7789_GPIO_CS |
		      ST7789_GPIO_RST |
		      ST7789_GPIO_DC |
		      ST7789_GPIO_BL);


	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);

	spi_reset(ST7789_SPI);
	spi_init_master(ST7789_SPI,
			// SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_BAUDRATE_FPCLK_DIV_8,
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_2,
			SPI_CR1_DFF_8BIT,
			SPI_CR1_MSBFIRST);

	/*
	 * On gd32f103 (the stm32f103 "compatible", not on gd32vf103)
	 * spi_enable_software_slave_management() causes the SPI_CR1_MSTR
	 * to fall off and SPI_SR_MODF to be asserted if we happen to call
	 * it with SPI_CR1_SSI off. Ugh.
	 */
	spi_set_nss_high(ST7789_SPI);
	spi_enable_software_slave_management(ST7789_SPI);

	spi_enable_crc(ST7789_SPI);
	SPI1_CRCPR = 7;

	spi_enable(ST7789_SPI);

	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);


	/* Reset */
	gpio_clear(ST7789_GPIO_BLOCK, ST7789_GPIO_RST);
	msleep(200);
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_RST);
	msleep(20);

	/* Wake up */
	command(ST7789_SLPOUT);
	msleep(120);

	command(ST7789_NORON);

	/* Display and color format setting */
	command(ST7789_MADCTL);		data(); send(TFT_MAD_RGB);

	/* XXX What command is this. */
	command(0xb6);			data(); send(0x0a); send(0x82);

	command(ST7789_COLMOD);		data(); send(0x55); msleep(10);

	/* Frame rate setting */
	command(ST7789_PORCTRL);	data(); send(0x0c); send(0x0c);
						send(0x00); send(0x33);
						send(0x33);

	/* Power setting */
	command(ST7789_GCTRL);		data(); send(0x35);
	command(ST7789_VCOMS);		data(); send(0x28);
	command(ST7789_LCMCTRL);	data(); send(0x0c);
	command(ST7789_VDVVRHEN);	data(); send(0x01); send(0xff);
	command(ST7789_VRHS);		data(); send(0x10);
	command(ST7789_VDVSET);		data(); send(0x20);
	command(ST7789_FRCTR2);		data(); send(0x0f);
	command(ST7789_PWCTRL1);	data(); send(0xa4); send(0xa1);

	/* Gamma setting */
	command(ST7789_PVGAMCTRL);	data(); send(0xd0); send(0x00);
						send(0x02); send(0x07);
						send(0x0a); send(0x28);
						send(0x32); send(0x44);
						send(0x42); send(0x06);
						send(0x0e); send(0x12);
						send(0x14); send(0x17);
	command(ST7789_NVGAMCTRL);	data(); send(0xd0); send(0x00);
						send(0x02); send(0x07);
						send(0x0a); send(0x28);
						send(0x31); send(0x54);
						send(0x47); send(0x0e);
						send(0x1c); send(0x17);
						send(0x1b); send(0x1e);

	command(ST7789_INVON);

	command(ST7789_CASET);		data(); send(0x00); send(0x00);
						send16(239);
	command(ST7789_RASET);		data(); send(0x00); send(0x00);
						send16(319);


	msleep(120);

	st7789_set_rotation(3);

	command(ST7789_DISPON);
	msleep(120);

	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_CS);

	display_splash();

	st7789_set_addrs(0, 0, cols - 1, rows - 1);

	/* Turn backlight on */
	gpio_set(ST7789_GPIO_BLOCK, ST7789_GPIO_BL);
}
