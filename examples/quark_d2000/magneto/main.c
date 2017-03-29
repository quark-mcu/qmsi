/*
 * Copyright (c) 2017, Intel Corporation
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

/*
 * Magnetometer
 *
 * This application will read the magneto data from the onboard sensor on the
 * Intel(R) Quark(TM) D2000 development platform and print it to the console
 * every 125 milliseconds. The app will complete once it has read 500 samples.
 */

#include <math.h>
#include "clk.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_uart.h"
#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "bmx1xx/bmx1xx.h"
#ifdef ENABLE_CALIBRATION
#include "calibration.h"
#endif

#define NUM_SAMPLES (5000)
#define M_PI 3.14159265358979323846

/*
 * mag_bias or hard-iron distortions: are created by objects that produce a
 * magnetic field (e.g. a speaker or a piece of magnetized iron).
 *
 * mag_scale or soft-iron distortions: deflections or alterations in the
 * existing magnetic field, commonly caused by metals such as nickel and
 * iron.
 */

double mag_bias[3] = {0, 0, 0}; /* x, y, z biases */
#ifdef ENABLE_CALIBRATION
double mag_scale[3] = {0, 0, 0}; /* x, y, z scales */
#endif				 /* ENABLE_CALIBRATION */

/* Convert degrees into compass direction. */
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

int main(void)
{
	bmx1xx_setup_config_t cfg;
	bmx1xx_mag_t mag = {0};
	double heading;
	int deg;
	uint32_t count = 0;
	/* Adjusted magnetometer readings */
	double mag_x, mag_y, mag_z;

	QM_PUTS("Starting: Magnetometer");

	/* Initialise the magneto. */
	cfg.pos = BMC150_J14_POS_0;

	if (0 != bmx1xx_init(cfg)) {
		QM_PUTS("Error: Unable to initialise BMC150.");
		return 1;
	}

	/* Activate the magneto. */
	if (0 != bmx1xx_mag_set_power(BMX1XX_MAG_POWER_ACTIVE)) {
		QM_PUTS("Error: Unable to set BMC150 power state.");
		return 1;
	}

	/* Set the magneto to high accuracy. */
	if (0 != bmx1xx_mag_set_preset(BMX1XX_MAG_PRESET_HIGH_ACCURACY)) {
		QM_PUTS("Error: Unable to set BBMC150 to high accuracy.");
		return 1;
	}
#ifdef ENABLE_CALIBRATION
	uint8_t c = 0;
	/* Mux out RX pin */
	if (STDOUT_UART == QM_UART_0) {
		qm_pmux_select(QM_PIN_ID_13, QM_PIN_13_FN_UART0_RXD);
		qm_pmux_input_en(QM_PIN_ID_13, true);
	} else {
		qm_pmux_select(QM_PIN_ID_21, QM_PIN_21_FN_UART1_RXD);
		qm_pmux_input_en(QM_PIN_ID_21, true);
	}
	QM_PRINTF("Do you want to calibrate the magnetometer? [y/n]\n");
	/* Awaiting user's input */
	while (0 != qm_uart_read(STDOUT_UART, &c, 0x00)) {
		QM_PRINTF("Error reading from STDOUT_UART!\n");
	};
	if (('y' == c) || ('Y' == c)) {
		if (0 != mag_calibration(mag_bias, mag_scale)) {
			QM_PRINTF("Calibration failed!\n");
			return 1;
		}
	} else {
		QM_PRINTF("Magnetometer calibration skipped!\n");
		mag_scale[0] = 1;
		mag_scale[1] = 1;
		mag_scale[2] = 1;
	}
#endif /* ENABLE_CALIBRATION */

	for (count = 0; count < NUM_SAMPLES; count++) {
		clk_sys_udelay(125000);

		/* Read the value from the magneto. */
		bmx1xx_read_mag(&mag);

		/* Adjust magnetometer readings */
		/* Correct hard-iron errors */
		mag_x = mag.x - mag_bias[0];
		mag_y = mag.y - mag_bias[1];
		mag_z = mag.z - mag_bias[2];
#ifdef ENABLE_CALIBRATION
		/* Correct soft-iron errors */
		mag_x *= mag_scale[0];
		mag_y *= mag_scale[1];
		mag_z *= mag_scale[2];
#endif /* ENABLE_CALIBRATION */

		/* Calculate the heading. */
		heading = atan2(mag_y, mag_x);
		if (heading < 0) {
			heading += (2 * M_PI);
		}

		/* Convert the heading into degrees. */
		deg = (int)((heading * 180) / M_PI);

		QM_PRINTF("x:\t%d\ty:\t%d\tz:\t%d\tdeg:\t%d\tdirection:\t%s\n",
			  (int)mag_x, (int)mag_y, (int)mag_z, deg,
			  degrees_to_direction(deg));
	}

	QM_PUTS("Finished: Magnetometer");

	return 0;
}
