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

#include <libopencm3/stm32/rcc.h>
#ifdef GD32VF103
#include <libopencm3/gd32v/eclic.h>
#include <libopencm3/gd32v/systick.h>
#else
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#endif

#include "systick.h"

#ifdef GD32VF103

void systick_init(void)
{
	systick_set_reload(rcc_ahb_frequency / SYSTICK_AHB_DIVIDER / 1000 - 1);
	systick_interrupt_enable();
}

#else /* !GD32VF103 */

void systick_init(void)
{
	/* 72MHz / 8 => 9000000 counts per second. */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

static void systick_set_value(uint64_t value)
{
	(void)value;
}

#endif

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

/* Called when systick fires */
void __attribute__((interrupt)) sys_tick_handler(void)
{
	system_millis++;
	systick_set_value(0);
}

/* sleep for delay milliseconds */
void msleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis);
}
