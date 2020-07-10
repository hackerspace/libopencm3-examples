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

#ifndef __CONFIG_H
#define __CONFIG_H

//#define LED_GPIO_CLOCK		RCC_GPIOC
//#define LED_GPIO_BLOCK		GPIOC
//#define LED_GPIO		GPIO13

#define CONSOLE_UART		USART1
#define CONSOLE_UART_CLOCK	RCC_USART1
#define CONSOLE_UART_IRQ	NVIC_USART1_IRQ
#define CONSOLE_UART_GPIO_CLOCK	RCC_GPIOA
#define CONSOLE_UART_GPIO_BLOCK	GPIOA
#define CONSOLE_UART_GPIO_RX	GPIO_USART1_RX
#define CONSOLE_UART_GPIO_TX	GPIO_USART1_TX

#define DISPLAY_COLS		240
#define DISPLAY_ROWS		135
#define DISPLAY_COLSTART	52
#define DISPLAY_ROWSTART	40

#define ST7789_SPI		SPI1
#define ST7789_SPI_CLOCK	RCC_SPI1
#define ST7789_GPIO_BLOCK	GPIOB
#define ST7789_GPIO_CLOCK	RCC_GPIOB
#define ST7789_GPIO_CS		GPIO2
#define ST7789_GPIO_DC		GPIO0
#define ST7789_GPIO_RST		GPIO1
#define ST7789_GPIO_BL		GPIO10

#endif /* !__CONFIG_H */
