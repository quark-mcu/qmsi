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

#include <unistd.h>

#if (__IPP_ENABLED__)
#include <dsp.h>
#endif

#include "qm_interrupt.h"
#include "qm_rtc.h"

#include "bmc150/bmc150.h"

#define ALARM (QM_RTC_ALARM_SECOND >> 3)

#if (__IPP_ENABLED__)
#define SAMPLES_SIZE 15

static float32_t samples[SAMPLES_SIZE];

static void print_axis_stats(int16_t value)
{
	static uint16_t index = 0;
	static uint16_t count = 0;
	float32_t mean, var, rms;

	samples[index] = value;
	index = (index + 1) % SAMPLES_SIZE;

	count = count == SAMPLES_SIZE ? SAMPLES_SIZE : count + 1;

	ippsq_rms_f32(samples, count, &rms);
	ippsq_var_f32(samples, count, &var);
	ippsq_mean_f32(samples, count, &mean);

	QM_PRINTF("rms %d var %d mean %d\n", (int) rms, (int) var, (int) mean);
}
#endif

static void print_accel_callback(void)
{
	bmc150_accel_t accel = {0};
	qm_rc_t rc;

	rc = bmc150_read_accel(&accel);
	QM_PRINTF("rc %d x %d y %d z %d\n", rc, accel.x, accel.y, accel.z);

#if (__IPP_ENABLED__)
	print_axis_stats(accel.z);
#endif

	qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0].rtc_ccvr + ALARM));
}

int main(void)
{
	qm_rtc_config_t rtc;

	QM_PUTS("Accelerometer example app\n");

	rtc.init_val = 0;
	rtc.alarm_en = true;
	rtc.alarm_val = ALARM;
	rtc.callback = print_accel_callback;

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	bmc150_init(BMC150_J14_POS_0);

	bmc150_set_accel_mode(BMC150_MODE_2G);
	bmc150_set_bandwidth(BMC150_BANDWIDTH_64MS);

	qm_rtc_set_config(QM_RTC_0, &rtc);

	return 0;
}
