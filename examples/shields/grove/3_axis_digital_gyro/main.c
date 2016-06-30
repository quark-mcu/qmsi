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

#include "qm_interrupt.h"
#include "qm_isr.h"
#include "qm_rtc.h"
#include "itg3200_gyro.h"

/*
 * This sample application measures the angular velocity using
 * Grove digital gyro sensor v1.3 ITG-3200 on QUARK SE and
 * QUARK D2000.
 *
 * The device is connected to I2C port on the grove shield. The
 * pin IDs 22 and 23 are multiplexed to use on QUARK SE and pinIDs
 * 6 and 7 are multiplexed to use on QUARK D2000.
 *
 * More information about the sensor could be found on the below link
 * http://www.seeedstudio.com/wiki/Grove_-_3-Axis_Digital_Gyro
 *
 * The application configures the I2C and RTC. The RTC callback reads
 * the sensor data registers on a time-quantum and calculates the
 * angular velocity for 3 axis.
 */

/* Time quantum set to trigger RTC callback. */
#define ALARM (QM_RTC_ALARM_SECOND >> 3)
/* Maximum number of samples to be read. */
#define MAX_SAMPLING (500)

/* Sampling count grows till MAX_SAMPLING to limit interrupt triggers. */
static volatile uint16_t sampling_count;

/* Set I2C configuration based on SoC. */
static int i2c_cfg_init(void);
/* Set RTC configuration. */
static void rtc_config(void);
/* RTC callback. */
static void rtc_callback(void *data);

/*
 * Entry point function of the application.
 *
 * This routine initializes and configures I2C, sensor
 * registers and RTC.
 */
int main(void)
{
	int status;

	QM_PUTS("Starting: Digital gyro sensor reading");

	/* Initialize and register as I2C device. */
	status = i2c_cfg_init();
	if (status) {
		QM_PRINTF("Error: I2C configuration failed! with 0x%x\n",
			  status);
	}

	/* Initialize device specific configuration. */
	status = itg3200_gyro_cfg_init();
	if (status) {
		QM_PRINTF("Error: sensor configuration failed! with 0x%x\n",
			  status);
	}

	if (!status) {
		/* Initialize and set RTC configuration. */
		rtc_config();

		/* Wait for RTC to fire MAX_SAMPLING times and then finish. */
		while (sampling_count < MAX_SAMPLING) {
		};
	}

	QM_PUTS("Finished: Digital gyro sensor reading");

	clk_periph_disable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	return 0;
}

/*
 * Set I2C configuration.
 *
 * This routine enables clock lines for I2C, sets up pin
 * multiplexing for I2C and sets the configuration for I2C.
 */
static int i2c_cfg_init(void)
{

	qm_i2c_config_t config = {
	    .address_mode = QM_I2C_7_BIT,
	    .mode = QM_I2C_MASTER,
	    .speed = QM_I2C_SPEED_FAST,
	};

	/* Enable I2C. */
	clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_MX_REGISTER);

	/* Setup pin mux. */
	qm_pmux_select(QM_PIN_ID_XX, QM_PMUX_FN_X);
	qm_pmux_select(QM_PIN_ID_YY, QM_PMUX_FN_X);

	/* Set I2C configuration. */
	return qm_i2c_set_config(QM_I2C_X, &config);
}

/*
 * Set RTC configuration.
 *
 * This routine registers rtc0 interrupt handler, enables clock,
 * initializes and sets the configuration for RTC.
 */
static void rtc_config(void)
{
	qm_rtc_config_t rtc_cfg;

	/* Register RTC ISR for periodical interrupt. */
	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	/* Initialize RTC module configuration. */
	rtc_cfg.alarm_en = true;
	rtc_cfg.init_val = 0;
	rtc_cfg.alarm_val = ALARM;
	rtc_cfg.callback = rtc_callback;
	rtc_cfg.callback_data = NULL;

	/* Set RTC configuration. */
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);
}

/*
 * RTC callback.
 *
 * This routine is invoked by the RTC subsystem on time-quantum. The
 * routine prints the angular velocity for x, y and z axis.
 * It sets the RTC alarm for next trigger.
 */
static void rtc_callback(void *data)
{
	int rc;
	float ax, ay, az;
	int16_t axis[ITG3200_MAX_AXIS] = {0};

	/* Calculate angular velocity for x, y, z axis. */
	rc = itg3200_gyro_get_angular_vel(&ax, &ay, &az, axis);
	if (rc) {
		sampling_count = MAX_SAMPLING;
		return;
	}

	/* Print the angular velocity as float. */
	itg3200_gyro_print_float(ax, ay, az, axis);

	/* Set a new RTC alarm. */
	++sampling_count;
	qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0].rtc_ccvr + ALARM));
}
