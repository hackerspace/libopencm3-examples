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

#define _COMPILING_NEWLIB

#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#ifdef GD32VF103
#include <libopencm3/gd32v/eclic.h>
#else
#include <libopencm3/cm3/nvic.h>
#endif

#include "console.h"
#include "config.h"

#ifdef GD32VF103

static inline void enable_irq(uint32_t irq)
{
        eclic_enable_irq(irq);
}

#else /* !GD32VF103 */

static inline void enable_irq(uint32_t irq)
{
	nvic_enable_irq(irq);
}

#endif

static void _putchar(char ch);

void __attribute__((interrupt)) usart1_isr(void)
{
	uint16_t ch;

	/* Blink the led on character input. */
	gpio_toggle(LED_GPIO_BLOCK, LED_GPIO);

	/* Echo the character. */
	ch = usart_recv(CONSOLE_UART);
	if (ch == '\r')
		ch = '\n';
	_putchar(ch);
}

void console_init(void)
{
	/* Configure and light on the LED. */
        rcc_periph_clock_enable(LED_GPIO_CLOCK);
        gpio_set_mode(LED_GPIO_BLOCK,
		      GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL,
		      LED_GPIO);
        gpio_set(LED_GPIO_BLOCK, LED_GPIO);

	/* Configure the serial port. */
	rcc_periph_clock_enable(CONSOLE_UART_GPIO_CLOCK);
	rcc_periph_clock_enable(CONSOLE_UART_CLOCK);
	gpio_set_mode(CONSOLE_UART_GPIO_BLOCK,
		      GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		      CONSOLE_UART_GPIO_TX);
	gpio_set_mode(CONSOLE_UART_GPIO_BLOCK,
		      GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT,
		      CONSOLE_UART_GPIO_RX);

	enable_irq(CONSOLE_UART_IRQ);

	usart_disable(CONSOLE_UART);
	usart_set_baudrate(CONSOLE_UART, 115200);
	usart_set_databits(CONSOLE_UART, 8);
	usart_set_stopbits(CONSOLE_UART, USART_STOPBITS_1);
	usart_set_parity(CONSOLE_UART, USART_PARITY_NONE);
	usart_set_flow_control(CONSOLE_UART, USART_FLOWCONTROL_NONE);
	usart_set_mode(CONSOLE_UART, USART_MODE_TX_RX);
	usart_enable_rx_interrupt(CONSOLE_UART);
	usart_enable(CONSOLE_UART);
}

/*
 * These are necessary to make newlib's printf() work with Nuclei
 * toolchain:
 */

static void _putchar(char ch)
{
	usart_send_blocking(CONSOLE_UART, ch);
	if (ch == '\n')
		_putchar('\r');
}

ssize_t _write(int fd, const void* ptr, size_t len)
{
	const char *str = (const char *)ptr;
	size_t i;

	if (fd != 1) {
		errno = EBADF;
		return -1;
	}

	for (i = 0; i < len; i++)
		_putchar(str[i]);

	return len;
}

void *_sbrk(ptrdiff_t incr)
{
	extern char end[];
	static char *curbrk = end;

	curbrk += incr;
	return curbrk - incr;
}

int _fstat(int fd, struct stat *st)
{
	if (fd != 1) {
		errno = EBADF;
		return -1;
	}

	memset(st, 0, sizeof(*st));
	return 0;
}
