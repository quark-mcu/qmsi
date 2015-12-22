/*
 * Copyright (c) 2015, Intel Corporation
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

#include "qm_wdt.h"
#include "qm_interrupt.h"

void wdt_example_callback(void);

volatile uint32_t wdt_fired;

/*
 * The watchdog timer can operate in two modes.
 * 1. If the timer expires reset the SoC immediately.
 * 2. If the timer expires, generate an interrupt, if the timer is not reloaded
 *    before the timer expires for a second time ,the WDT will reset the SoC.
 *
 * In this example, we operate in the second mode and reload the WDT every time
 * an interrupt is generated, this way the SoC does not reset.
 */

#define MAX_WDT_FIRINGS (10)

/* QMSI wdt app example */
int main(void)
{
	qm_wdt_config_t wr_cfg;
	wr_cfg.timeout = QM_WDT_2_POW_17_CYCLES;
	wr_cfg.mode = QM_WDT_MODE_INTERRUPT_RESET;
	wr_cfg.callback = wdt_example_callback;

	wdt_fired = 0;

	qm_wdt_set_config(QM_WDT_0, &wr_cfg);
	qm_irq_request(QM_IRQ_WDT_0, qm_wdt_isr_0);

	qm_wdt_start(QM_WDT_0);

	/* Wait for WDT to fire 10 times and then finish. */
	while (wdt_fired < MAX_WDT_FIRINGS) {
	}
	QM_PRINTF("Watchdog fired %d times\n", MAX_WDT_FIRINGS);
	return 0;
}

/* WDT Requires a callback, there is no interrupt enable / disable. */
void wdt_example_callback(void)
{
	wdt_fired++;
	qm_wdt_reload(QM_WDT_0);
}
