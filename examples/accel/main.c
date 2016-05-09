
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

#include <unistd.h>

#if (__IPP_ENABLED__)
#include <dsp.h>
#endif

#include "qm_interrupt.h"
#include "qm_rtc.h"
#include "qm_uart.h"
#include "qm_isr.h"

#include "bmx1xx/bmx1xx.h"

#define ALARM (QM_RTC_ALARM_SECOND >> 3)

#define SAMPLING_DURATION 500
static uint16_t cb_count = 0;

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

	QM_PRINTF("rms %d var %d mean %d\n", (int)rms, (int)var, (int)mean);
}
#endif

static void print_accel_callback(void *data)
{
	bmx1xx_accel_t accel = {0};
	int rc;

	rc = bmx1xx_read_accel(&accel);
	if (rc == 0) {
		QM_PRINTF("x %d y %d z %d\n", accel.x, accel.y, accel.z);
	} else {
		QM_PRINTF("Error reading data from sensor.\n");
	}

#if (__IPP_ENABLED__)
	print_axis_stats(accel.z);
#endif

	if (cb_count < SAMPLING_DURATION) {
		qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0].rtc_ccvr + ALARM));
		cb_count++;
	} else {
		QM_PUTS("Finished: Accelerometer example app\n");
	}
}

int main(void)
{
	qm_rtc_config_t rtc;
	bmx1xx_setup_config_t cfg;

	QM_PUTS("Starting: Accelerometer example app\n");

	rtc.init_val = 0;
	rtc.alarm_en = true;
	rtc.alarm_val = ALARM;
	rtc.callback = print_accel_callback;
	rtc.callback_data = NULL;

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

#if (QUARK_D2000)
	cfg.pos = BMC150_J14_POS_0;
#endif

	bmx1xx_init(cfg);

	bmx1xx_accel_set_mode(BMX1XX_MODE_2G);

#if (BMC150_SENSOR)
	bmx1xx_set_bandwidth(BMC150_BANDWIDTH_64MS);
#elif(BMI160_SENSOR)
	bmx1xx_set_bandwidth(BMI160_BANDWIDTH_10MS);
#endif
	qm_rtc_set_config(QM_RTC_0, &rtc);

	return 0;
}
