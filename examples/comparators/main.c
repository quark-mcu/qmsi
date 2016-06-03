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
#include <stdio.h>

#include "qm_common.h"
#include "qm_comparator.h"
#include "qm_pinmux.h"
#include "qm_interrupt.h"
#include "qm_isr.h"

static void ac_example_callback(void *, uint32_t);

static volatile bool callback_invoked = false;

int main(void)
{
	/* Comparator example app
	 *
	 * This app sets up comparator 0 to fire an interrupt when the input
	 * voltage is greater than the reference voltage (0.95V)
	 *
	 * On the Intel(R) Quark(TM) Microcontroller D2000 Development Platform
	 * the pin used is marked "SSO 10"
	 */

	qm_ac_config_t ac_cfg;

	QM_PUTS("Starting: Analog Comparators\n");

#if (QM_SENSOR)
	QM_SCSS_INT->int_comparators_ss_mask &= ~BIT(0);
#endif

	/* Set up pin muxing and request IRQ*/
	qm_pmux_select(QM_PIN_ID_0, QM_PMUX_FN_1);
	qm_pmux_input_en(QM_PIN_ID_0, true);

	qm_irq_request(QM_IRQ_AC, qm_ac_isr);

	/* Write configs */
	ac_cfg.reference = BIT(0); /* Ref internal voltage */
	ac_cfg.polarity = 0x0;     /* Fire if greater than ref (high level) */
	ac_cfg.power = BIT(0);     /* Normal operation mode */
	ac_cfg.int_en = BIT(0);    /* AIN0 enable */
	ac_cfg.callback = ac_example_callback;

	qm_ac_set_config(&ac_cfg);

	while (false == callback_invoked) {
	}
	QM_PUTS("Finished: Analog Comparators\n");

	return 0;
}

static void ac_example_callback(void *data, uint32_t status)
{
#if (QM_SENSOR)
	/*
	 * The analog comparators use level triggered interrupts so we will get
	 * a constant stream of interrupts if we do not mask them.
	 */

	/*
	 * Comment the following line if you want to get more
	 * than one interrupt on the sensor subsystem.
	 */
	QM_SCSS_INT->int_comparators_ss_mask |= BIT(0);
#else
	/*
	 * Comment the following line if you want to get more
	 * than one interrupt on the x86 core.
	 */
	QM_SCSS_INT->int_comparators_host_mask |= BIT(0);
#endif

	QM_PUTS("Comparator interrupt fired");
	QM_ASSERT(0x1 == status);
	callback_invoked = true;
}
