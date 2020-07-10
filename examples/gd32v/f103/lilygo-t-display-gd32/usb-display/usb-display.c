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

#include "console.h"
#include "systick.h"
#include "st7789.h"
#include "gud.h"

#ifdef GD32VF103

static inline void clock_init(void)
{
	rcc_clock_setup_pll(&rcc_hse8_configs[RCC_CLOCK_HSE8_96MHZ]);
}
#else

static inline void clock_init(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

#endif

int main(void)
{
	clock_init();
	console_init();
	systick_init();
	st7789_init();

	gud_init();
	while (1)
		gud_poll();
}
