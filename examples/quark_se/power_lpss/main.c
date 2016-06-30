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

#include "power_states.h"
#include "qm_common.h"
#include "qm_interrupt.h"
#include "qm_gpio.h"
#include "qm_pinmux.h"
#include "qm_isr.h"

/* POWER LPSS app example.
 *
 * This application must run in conjonction with its Sensor Subsystem
 * counterpart located in /examples/sensor/power_lpss/.
 *
 * This app requires a board to be set up with a jumper cable connecting the
 * pins listed below so the output pin can trigger an interrupt on the
 * input pin. PIN_OUT will be configured as an output pin and PIN_INTR will be
 * configured as an input pin with interrupts enabled.
 * The PIN_INTR will be triggered PIN_OUT from the power_lpss application
 * running on the Sensor Subsystem to initiate a state transition
 * on the host core.
 *
 * On Atlas Hills, PIN_OUT and PIN_INTR are located on header P4 PIN 42 and 40.
 *
 * Sensor collaboration is needed as LPSS can only be achieved
 * as a combination of the two cores states.
 *
 * States executed in this example are:
 * LPSS: Combination of C2/C2LP and SS2 (Sensor state)
 */

#define PIN_OUT (0)
#define PIN_INTR (3)
#define QM_SCSS_GP_CORE_STATE_C2 BIT(2)
#define QM_SCSS_GP_CORE_STATE_C2LP BIT(3)

static void gpio_example_callback(void *, uint32_t);

int main(void)
{
	qm_gpio_port_config_t cfg;

	QM_PUTS("Starting: Power LPSS example.");

	/* Set GPIO pin muxing */
	qm_pmux_select(PIN_OUT, QM_PMUX_FN_0);
	qm_pmux_select(PIN_INTR, QM_PMUX_FN_0);

	/*
	 * Demonstrating entering and exiting LPSS mode.
	 * The Sensor Subsystem will wake from LPSS into SS0.
	 * As the Host is in C2 at that moment, the sensor subsystem
	 * will trigger an interrupt into the Host via GPIO.
	 *
	 * Request IRQ and write GPIO port config
	 */
	cfg.direction = BIT(PIN_OUT);
	cfg.int_en = BIT(PIN_INTR);       /* Interrupt enabled */
	cfg.int_type = BIT(PIN_INTR);     /* Edge sensitive interrupt */
	cfg.int_polarity = BIT(PIN_INTR); /* Rising edge */
	cfg.int_debounce = BIT(PIN_INTR); /* Debounce enabled */
	cfg.int_bothedge = 0x0;		  /* Both edge disabled */
	cfg.callback = gpio_example_callback;
	cfg.callback_data = NULL;

	qm_irq_request(QM_IRQ_GPIO_0, qm_gpio_isr_0);

	qm_gpio_set_config(QM_GPIO_0, &cfg);

	QM_PUTS("Go to c2. Sensor in SS2. Enable LPSS");
	/* Enable LPSS */
	power_soc_lpss_enable();
	/* Tell sensor we are ready for LPSS mode */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_CORE_STATE_C2;
	/* Go to LPSS. Sensor will make me transition to C2 and wake me up */
	power_cpu_c2();
	/* Clear flag */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_CORE_STATE_C2;
	QM_PUTS("Wake up from LPSS.");

	QM_PUTS("Go to c2lp. Sensor in SS2. Enable LPSS");
	/* Enable LPSS */
	power_soc_lpss_enable();
	/* Tell sensor we are ready for LPSS mode */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_CORE_STATE_C2LP;
	/* Go to C2LP. Sensor will make me transition to LPSS and wake me up */
	power_cpu_c2lp();
	/* Clear flag */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_CORE_STATE_C2LP;
	QM_PUTS("Wake up from LPSS.");

	QM_PUTS("Finished: Power LPSS example.");

	return 0;
}

void gpio_example_callback(void *data, uint32_t status)
{
	QM_PUTS("GPIO interrupt triggered");
}
