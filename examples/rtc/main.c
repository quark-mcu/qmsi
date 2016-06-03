/*
 *  Copyright (c) 2016, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. Neither the name of the Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "qm_rtc.h"
#include "qm_interrupt.h"
#include "qm_isr.h"

#define ALARM (QM_RTC_ALARM_MINUTE / 6)
#define MAX_RTC_FIRINGS (5)

void rtc_example_callback(void *);

static volatile uint32_t rtc_fired = 0;

/*  QMSI RTC app example */
int main(void)
{
	/*  Variables */
	qm_rtc_config_t cfg;

	QM_PRINTF("Starting: RTC\n");

	/*  Initialise RTC configuration */
	cfg.init_val = 0;
	cfg.alarm_en = true;
	cfg.alarm_val = ALARM;
	cfg.callback = rtc_example_callback;
	cfg.callback_data = NULL;

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	qm_rtc_set_config(QM_RTC_0, &cfg);

	/* Wait for RTC to fire 5 times and then finish. */
	while (rtc_fired < MAX_RTC_FIRINGS) {
	}

	QM_PRINTF("Finished: RTC\n");
	clk_periph_disable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);
	return 0;
}

void rtc_example_callback(void *data)
{
	QM_PUTS("Alarm!!\n");
	qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0].rtc_ccvr + ALARM));
	rtc_fired++;
}
