/*
 * Copyright (c) 2016 Samuel Cowen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/
 */

#include "calibration.h"
#include "bmx1xx/bmx1xx.h"
#include "clk.h"

#define ONE_SEC 1000000
#define MSEC 1000

int32_t mag_calibration(double *mag_bias, double *mag_scale)
{
	/* If Data rate = 20 Hz => 20 samples/seconds =>
	 *  600  samples = 30 * 20 = 30 seconds
	 *  1200 samples = 60 * 20 = 60 seconds */
	uint32_t sample_count = 30 * 20;
	uint32_t i;
	bmx1xx_mag_t mag = {0};
	bmx1xx_mag_t mag_min = {0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF};
	bmx1xx_mag_t mag_max = {0x80000000, 0x80000000, 0x80000000};
	double max[3], min[3];
	double avg[3], avg_rad;
	int32_t rc = -1;

	if ((NULL == mag_bias) || (NULL == mag_scale)) {
		rc = -EFAULT;
		goto end;
	}

	QM_PRINTF("\nStarting the calibration of the magnetometer...\n\n");

	QM_PRINTF("Move the device slowly while calibrating.\n");
	QM_PRINTF("Move it in 8 shape and rotate it as well.\n");
	QM_PRINTF("It should take approximately 30 seconds.\n\n");
	clk_sys_udelay(ONE_SEC);
	/* Read sample_count samples and determine the minimum and maximum
	 * values read on every axis.	 */
	for (i = 0; i < sample_count; ++i) {
		rc = bmx1xx_read_mag(&mag);
		if (rc) {
			QM_PUTS("Error reading the sensor data!");
			goto end;
		}
		if (mag.x > mag_max.x)
			mag_max.x = mag.x;
		if (mag.x < mag_min.x)
			mag_min.x = mag.x;
		if (mag.y > mag_max.y)
			mag_max.y = mag.y;
		if (mag.y < mag_min.y)
			mag_min.y = mag.y;
		if (mag.z > mag_max.z)
			mag_max.z = mag.z;
		if (mag.z < mag_min.z)
			mag_min.z = mag.z;

		clk_sys_udelay(50 * MSEC);
	}
	/* Get and store hard-iron correction estimates for later use */
	mag_bias[0] = (mag_max.x + mag_min.x) / 2.0;
	mag_bias[1] = (mag_max.y + mag_min.y) / 2.0;
	mag_bias[2] = (mag_max.z + mag_min.z) / 2.0;

	/* Get soft-iron correction estimate */

	/* First remove hard-iron errors from Min and Max */
	max[0] = mag_max.x - mag_bias[0];
	max[1] = mag_max.y - mag_bias[1];
	max[2] = mag_max.z - mag_bias[2];
	min[0] = mag_min.x - mag_bias[0];
	min[1] = mag_min.y - mag_bias[1];
	min[2] = mag_min.z - mag_bias[2];
	/* Calculate the average distance from the centre */
	for (i = 0; i < 3; i++) {
		if (min[i] < 0) {
			min[i] *= -1;
		}
		avg[i] = (max[i] + min[i]) / 2;
	}
	/* Average out the x, y, z components */
	avg_rad = (avg[0] + avg[1] + avg[2]) / 3.0;
	/* Calculate the scale factor by dividing average radius by average
	 * value for that axis. */
	mag_scale[0] = avg_rad / avg[0];
	mag_scale[1] = avg_rad / avg[1];
	mag_scale[2] = avg_rad / avg[2];

	QM_PRINTF("Magnetometer calibration successfully done!\n");
end:
	return rc;
}
