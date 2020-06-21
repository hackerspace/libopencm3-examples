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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "ssd1306.h"
#include "systick.h"
#include "config.h"

#include "splash.xbm"

#define SET_LOW_COLUMN			0x00
#define SET_HIGH_COLUMN			0x10
#define RIGHT_HORIZ_SCROLL		0x26
#define LEFT_HORIZ_SCROLL		0x27
#define VERT_AND_RIGHT_HORIZ_SCROLL	0x29
#define VERT_AND_LEFT_HORIZ_SCROLL	0x2a
#define DEACTIVATE_SCROLL		0x2e
#define ACTIVATE_SCROLL			0x2f
#define SET_VERT_SCROLL_AREA		0xa3
#define SET_COM_PINS			0xda
#define DISPLAY_OFF			0xae
#define SET_DISPLAY_CLOCK_DIV		0xd5
#define SET_MULTIPLEX			0xa8
#define SET_DISPLAY_OFFSET		0xd3
#define SET_START_LINE			0x40
#define CHARGE_PUMP			0x8d
#define SEG_REMAP			0xa0
#define COM_SCAN_INC			0xc0
#define COM_SCAN_DEC			0xc8
#define SET_CONTRAST			0x81
#define SET_PRECHARGE			0xd9
#define SET_VCOM_DETECT			0xdb
#define DISPLAY_ALL_ON_RESUME		0xa4
#define DISPLAY_ALL_ON			0xa5
#define NORMAL_DISPLAY			0xa6
#define INVERT_DISPLAY			0xa7
#define DISPLAY_ON			0xaf
#define SET_MEMORY_MODE			0x20
#define  MEMORY_MODE_HORIZ		0x00
#define  MEMORY_MODE_VERT		0x01
#define  MEMORY_MODE_PAGE		0x02
#define SET_COL_ADDRESS			0x21
#define SET_PAGE_ADDRESS		0x22

static inline void commands(void)
{
	spi_send_msb_first(SSD1306_SPI);
	gpio_clear(SSD1306_GPIO_BLOCK, SSD1306_GPIO_DC);
}

static inline void data(void)
{
	msleep(1); /* why? */
	gpio_set(SSD1306_GPIO_BLOCK, SSD1306_GPIO_DC);
	spi_send_lsb_first(SSD1306_SPI);
}

static inline void send(uint8_t byte)
{
	spi_send(SSD1306_SPI, byte);
}

void ssd1306_send_data(const uint8_t *bytes, int len)
{
	int i;

	for (i = 0; i < len; i++)
		send(bytes[i]);
}

void ssd1306_set_addrs(uint8_t col, uint8_t page)
{
	commands();
	send(SET_MEMORY_MODE); send(MEMORY_MODE_VERT);
	send(SET_COL_ADDRESS); send(col); send(127);
	send(SET_PAGE_ADDRESS); send(page); send(7);
	data();
}

void ssd1306_init(void)
{
	/* Initialize the SPI bus */
	rcc_periph_clock_enable(SSD1306_GPIO_CLOCK);
	rcc_periph_clock_enable(SSD1306_SPI_CLOCK);

	gpio_set_mode(SSD1306_GPIO_BLOCK,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      SSD1306_GPIO_SS |
		      SSD1306_GPIO_SCK |
		      SSD1306_GPIO_MOSI);
	gpio_set_mode(SSD1306_GPIO_BLOCK,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      SSD1306_GPIO_MISO);
	gpio_set_mode(SSD1306_GPIO_BLOCK,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      SSD1306_GPIO_RES |
		      SSD1306_GPIO_DC);

	spi_reset(SSD1306_SPI);
	spi_init_master(SSD1306_SPI,
			SPI_CR1_BAUDRATE_FPCLK_DIV_64,
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_2,
			SPI_CR1_DFF_8BIT,
			SPI_CR1_MSBFIRST);

	spi_enable_software_slave_management(SSD1306_SPI);
	spi_set_nss_high(SSD1306_SPI);

	spi_enable(SSD1306_SPI);

	/* Reset the OLED panel. */
	gpio_clear(SSD1306_GPIO_BLOCK, SSD1306_GPIO_RES);
	commands();
	gpio_set(SSD1306_GPIO_BLOCK, SSD1306_GPIO_RES);

	/* Configure the panel. */
	send(DISPLAY_OFF);
	send(SET_DISPLAY_CLOCK_DIV); send(0x80);

	if (SSD1306_COLS == 64) {
		send(SET_MULTIPLEX); send(0x3f);
		send(SET_COM_PINS); send(0x12);
	} else {
		send(SET_MULTIPLEX); send(0x1f);
		send(SET_COM_PINS); send(0x02);
	}

	send(SET_DISPLAY_OFFSET); send(0x00);
	send(SET_START_LINE | 0x00);
	send(CHARGE_PUMP); send(0x14);
	send(SEG_REMAP | 0x01);
	send(COM_SCAN_INC);
	send(SET_PRECHARGE); send(0xf1);
	send(SET_VCOM_DETECT); send(0x40);
	send(DISPLAY_ALL_ON_RESUME);
	send(NORMAL_DISPLAY);
	send(DISPLAY_ON);

	send(SET_MEMORY_MODE); send(MEMORY_MODE_VERT);
	send(SET_COL_ADDRESS); send(0); send(SSD1306_ROWS - 1);
	send(SET_PAGE_ADDRESS); send(0); send((SSD1306_COLS / 8) - 1);

	data();
	ssd1306_send_data(splash_bits, sizeof(splash_bits));
}
