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
#include <math.h>

#include "qm_interrupt.h"
#include "qm_rtc.h"

#include "bmc150/bmc150.h"

#define ALARM (QM_RTC_ALARM_SECOND >> 3)

#define M_PI 3.14159265358979323846

static const char *degrees_to_direction(unsigned int deg)
{
	if (deg >= 360) {
		deg %= 360;
	}

	if (deg >= 338 || deg < 23) {
		return "N";
	} else if (deg < 68) {
		return "NE";
	} else if (deg < 113) {
		return "E";
	} else if (deg < 158) {
		return "SE";
	} else if (deg < 203) {
		return "S";
	} else if (deg < 248) {
		return "SW";
	} else if (deg < 293) {
		return "W";
	} else {
		return "NW";
	}
}

static void print_magneto_callback(void)
{
	bmc150_mag_t mag = {0};
	double heading;
	int deg;

	bmc150_read_mag(&mag);

	heading = atan2(mag.y, mag.x);

	if (heading < 0) {
		heading += 2 * M_PI;
	}

	deg = (int)(heading * 180 / M_PI);

	QM_PRINTF("mag x %d y %d z %d deg %d direction %s\n", mag.x, mag.y,
		  mag.z, deg, degrees_to_direction(deg));

	qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0].rtc_ccvr + ALARM));
}

int main(void)
{
	qm_rtc_config_t rtc;
	qm_rc_t rc;

	QM_PUTS("Magnetometer example app\n");

	rtc.init_val = 0;
	rtc.alarm_en = true;
	rtc.alarm_val = ALARM;
	rtc.callback = print_magneto_callback;

	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	rc = bmc150_init(BMC150_J14_POS_0);
	if (rc != QM_RC_OK) {
		return rc;
	}

	rc = bmc150_mag_set_power(BMC150_MAG_POWER_ACTIVE);
	if (rc != QM_RC_OK) {
		return rc;
	}

	rc = bmc150_mag_set_preset(BMC150_MAG_PRESET_HIGH_ACCURACY);
	if (rc != QM_RC_OK) {
		return rc;
	}

	qm_rtc_set_config(QM_RTC_0, &rtc);

	return rc;
}
