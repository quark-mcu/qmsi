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

#ifndef __QM_CALIBRATION_H__
#define __QM_CALIBRATION_H__

#include <stdint.h>

/**
 * Compute hard and soft iron calibration constants
 *
 * Because the hard-iron distortion is caused by the magnetic field generated
 * by other materials which exhibit a constant additive field to the earth's
 * magnetic field, it seems important to use the sensor in the same position it
 * was calibrated and as far as possible from electronic equipment as laptops,
 * monitors, phones, etc.
 *
 * @param[out] mag_bias Hard-iron correction estimated
 * @param[out] mag_scale	Soft-iron corrections.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 *
 */
int32_t mag_calibration(double *mag_bias, double *mag_scale);

#endif /* __QM_CALIBRATION_H__ */
