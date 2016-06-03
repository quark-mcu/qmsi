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
#include "qm_pic_timer.h"
#include "qm_isr.h"

/* POWER CORE app example
 *
 * This example demonstrates core states.
 * The core is woken up by the PIC timer.
 *
 * States executed in this example are:
 * C1: Processor clock gated
 * C2: Processor clock gated, gateway to LPSS state
 * C2LP: Processor complex clock gated
 */
static void timer_expired(void *data);

int main(void)
{
	qm_pic_timer_config_t pic_conf;

	QM_PUTS("Starting: Power Core example");

	/* PIC timer to wake up from C1/C2/C2LP */
	pic_conf.mode = QM_PIC_TIMER_MODE_PERIODIC;
	pic_conf.int_en = true;
	pic_conf.callback = timer_expired;

	qm_int_vector_request(QM_INT_VECTOR_PIC_TIMER, qm_pic_timer_isr);

	qm_pic_timer_set_config(&pic_conf);

	/* Set period.  The following value generates one interrupt per second
	 * with a 32MHz sysclk.
	 */
	qm_pic_timer_set(0x2000000);

	QM_PUTS("Go to c1.");
	/* Halt the CPU, PIC TIMER INT will wake me up. */
	power_cpu_c1();
	QM_PUTS("Wake up from c1.");

	QM_PUTS("Go to c2.");
	/* Go to c2, PIC TIMER INT will wake me up. */
	power_cpu_c2();
	QM_PUTS("Wake up from c2.");

	QM_PUTS("Go to c2lp.");
	/* Go to c2lp, PIC TIMER INT will wake me up. */
	power_cpu_c2lp();
	QM_PUTS("Wake up from c2lp.");

	/* Remove PIC timer interrupts. */
	pic_conf.int_en = false;
	qm_pic_timer_set_config(&pic_conf);

	QM_PUTS("Finished: Power Core example");

	return 0;
}

void timer_expired(void *data)
{
	QM_PUTS("Timer interrupt triggered");
}
