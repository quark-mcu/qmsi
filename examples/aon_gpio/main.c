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

#include "qm_gpio.h"
#include "qm_interrupt.h"
#include "qm_isr.h"

#include <inttypes.h>

/*
 * QMSI AON GPIO app example
 *
 * This example is QUARK_SE specific.
 *
 * On the Quark SE development platform, PIN_INTR is triggered by pressing push
 * button 0, which is marked as 'PB0' on the board.
 */
#define PIN_INTR 4

static void aon_gpio_example_callback(void *, uint32_t);

static volatile bool callback_invoked = false;

int main(void)
{
	qm_gpio_port_config_t cfg;

	QM_PUTS("Starting: AON GPIO\n");

	/* Request IRQ and write GPIO port config */
	cfg.direction = 0;		   /* All pins are input */
	cfg.int_en = BIT(PIN_INTR);	/* Interrupt enabled */
	cfg.int_type = BIT(PIN_INTR);      /* Edge sensitive interrupt */
	cfg.int_polarity = ~BIT(PIN_INTR); /* Falling edge */
	cfg.int_debounce = BIT(PIN_INTR);  /* Debounce enabled */
	cfg.int_bothedge = 0x0;		   /* Both edge disabled */
	cfg.callback = aon_gpio_example_callback;
	cfg.callback_data = NULL;

	qm_irq_request(QM_IRQ_AONGPIO_0, qm_aon_gpio_isr_0);

	qm_gpio_set_config(QM_AON_GPIO_0, &cfg);
	QM_PUTS("AON GPIO set up, press the Push Button 0 to trigger the "
		"interrupt\n");
	while (false == callback_invoked) {
	}
	QM_PUTS("Finished: AON GPIO\n");
	return 0;
}

void aon_gpio_example_callback(void *data, uint32_t status)
{
	QM_PRINTF("AON GPIO callback - status register = 0x%u\n", status);
	callback_invoked = true;
}
