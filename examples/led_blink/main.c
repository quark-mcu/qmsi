/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "qm_soc_regs.h"
#include "qm_gpio.h"
#include "clk.h"

#if (QUARK_SE)
/* Led on AtlasHills board TODO: remove codename*/
#define LED_BIT 25
#elif(QUARK_D2000)
/* LED on Intel(R) Quark(TM) Microcontroller D2000 Development Platform */
#define LED_BIT 24
#endif

#define DELAY 300000UL

#define MAX_LED_BLINKS (10)

static qm_gpio_port_config_t cfg;

int main(void)
{
	uint32_t counter = 0;
	QM_PUTS("Starting: Led blink\n");

	cfg.direction = BIT(LED_BIT);
	qm_gpio_set_config(QM_GPIO_0, &cfg);

	while (counter < MAX_LED_BLINKS) {
		qm_gpio_set_pin(QM_GPIO_0, LED_BIT);
		clk_sys_udelay(DELAY);
		qm_gpio_clear_pin(QM_GPIO_0, LED_BIT);
		clk_sys_udelay(DELAY);
		counter++;
	}
	QM_PUTS("Finished: Led blink\n");
	return 0;
}
