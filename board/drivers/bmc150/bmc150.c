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

#include "qm_i2c.h"
#include "qm_pinmux.h"
#include "qm_scss.h"
#include "bmc150.h"

#define BMC150_REG_ACCEL_CHIPID 0x00
#define BMC150_REG_ACCEL_X_LSB 0x02
#define BMC150_REG_ACCEL_X_MSB 0x03
#define BMC150_REG_ACCEL_Y_LSB 0x04
#define BMC150_REG_ACCEL_Y_MSB 0x05
#define BMC150_REG_ACCEL_Z_LSB 0x06
#define BMC150_REG_ACCEL_Z_MSB 0x07
#define BMC150_REG_ACCEL_PMU_RANGE 0x0F
#define BMC150_REG_ACCEL_PMU_BW 0x10

#define BMC150_REG_MAG_CHIPID 0x40
#define BMC150_REG_MAG_X_LSB 0x42
#define BMC150_REG_MAG_X_MSB 0x43
#define BMC150_REG_MAG_Y_LSB 0x44
#define BMC150_REG_MAG_Y_MSB 0x45
#define BMC150_REG_MAG_Z_LSB 0x46
#define BMC150_REG_MAG_Z_MSB 0x47
#define BMC150_REG_MAG_RHALL_LSB 0x48
#define BMC150_REG_MAG_RHALL_MSB 0x49
#define BMC150_REG_MAG_POWER_MODES 0x4B
#define BMC150_REG_MAG_OPERATION_MODES 0x4C
#define BMC150_REG_MAG_REPETION_XY 0x51
#define BMC150_REG_MAG_REPETION_Z 0x52

/* Undocumented registers necessary for temperature compensation. */
#define BMC150_REG_MAG_DIG_X1 0x5d
#define BMC150_REG_MAG_DIG_Y1 0x5e
#define BMC150_REG_MAG_DIG_Z4_LSB 0x62
#define BMC150_REG_MAG_DIG_Z4_MSB 0x63
#define BMC150_REG_MAG_DIG_X2 0x64
#define BMC150_REG_MAG_DIG_Y2 0x65
#define BMC150_REG_MAG_DIG_Z2_LSB 0x68
#define BMC150_REG_MAG_DIG_Z2_MSB 0x69
#define BMC150_REG_MAG_DIG_Z1_LSB 0x6a
#define BMC150_REG_MAG_DIG_Z1_MSB 0x6b
#define BMC150_REG_MAG_DIG_XYZ1_LSB 0x6c
#define BMC150_REG_MAG_DIG_XYZ1_MSB 0x6d
#define BMC150_REG_MAG_DIG_Z3_LSB 0x6e
#define BMC150_REG_MAG_DIG_Z3_MSB 0x6f
#define BMC150_REG_MAG_DIG_XY2 0x70
#define BMC150_REG_MAG_DIG_XY1 0x71

/* Overflow */
#define BMC150_FLIP_OVERFLOW_ADCVAL (-4096)
#define BMC150_HALL_OVERFLOW_ADCVAL (-16384)
#define BMC150_OVERFLOW_OUTPUT (0x8000)

#define EXTRACT_ACCEL(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 4)

#define EXTRACT_MAG_XY(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 3)
#define EXTRACT_MAG_Z(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 1)
#define EXTRACT_RHALL(msb, lsb) ((int16_t)((lsb) | (msb) << 8) >> 2)
#define EXTRACT_UINT16(msb, lsb) ((uint16_t)((lsb) | (msb) << 8))

static const qm_i2c_config_t config = {
    .address_mode = QM_I2C_7_BIT,
    .mode = QM_I2C_MASTER,
    .speed = QM_I2C_SPEED_STD,
};

static struct compensation {
	int8_t x1, y1, x2, y2, xy1, xy2;
	uint16_t z1, z2, z3, z4, xyz1;
	bool init;
} digital_comp;

static uint16_t accel_addr = 0x10;
static uint16_t magneto_addr = 0x12;

/*
 * From the BMC150 datasheet, page 125:
 * "The register address is automatically incremented and, therefore, more
 * than one byte can be sequentially read out. Once a new data read
 * transmission starts, the start address will be set to the register address
 * specified in the latest I2C write command."
 */
static qm_rc_t read_register(uint16_t addr, uint8_t reg, uint8_t *const data,
			     uint32_t len)
{
	qm_rc_t rc;

	rc = qm_i2c_master_write(QM_I2C_0, addr, &reg, sizeof(reg), false);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = qm_i2c_master_read(QM_I2C_0, addr, data, len, true);
	if (QM_RC_OK != rc) {
		return rc;
	}

	return QM_RC_OK;
}

static qm_rc_t write_register(uint16_t addr, uint8_t reg, uint8_t data)
{
	qm_rc_t rc;

	rc = qm_i2c_master_write(QM_I2C_0, addr, &reg, sizeof(reg), false);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = qm_i2c_master_write(QM_I2C_0, addr, &data, sizeof(data), true);
	if (QM_RC_OK != rc) {
		return rc;
	}

	return QM_RC_OK;
}

qm_rc_t bmc150_read_accel(bmc150_accel_t *const accel)
{
	qm_rc_t rc;
	uint8_t raw_accel[6];

	/* Reading the 6 registers at once. */
	rc = read_register(accel_addr, BMC150_REG_ACCEL_X_LSB, raw_accel,
			   sizeof(raw_accel));
	if (QM_RC_OK != rc) {
		return rc;
	}

	accel->x = EXTRACT_ACCEL(raw_accel[1], raw_accel[0]);
	accel->y = EXTRACT_ACCEL(raw_accel[3], raw_accel[2]);
	accel->z = EXTRACT_ACCEL(raw_accel[5], raw_accel[4]);

	return QM_RC_OK;
}

qm_rc_t bmc150_set_accel_mode(bmc150_accel_mode_t mode)
{
	return write_register(accel_addr, BMC150_REG_ACCEL_PMU_RANGE, mode);
}

qm_rc_t bmc150_set_bandwidth(bmc150_bandwidth_t bw)
{
	return write_register(accel_addr, BMC150_REG_ACCEL_PMU_BW, bw);
}

static qm_rc_t read_dig_comp(struct compensation *const comp)
{
	uint8_t raw_comp[21];
	qm_rc_t rc;

	rc = read_register(magneto_addr, BMC150_REG_MAG_DIG_X1, raw_comp,
			   sizeof(raw_comp));
	if (QM_RC_OK != rc) {
		return rc;
	}

	comp->x1 = raw_comp[0];
	comp->y1 = raw_comp[1];
	comp->z4 = EXTRACT_UINT16(raw_comp[6], raw_comp[5]);
	comp->x2 = raw_comp[7];
	comp->y2 = raw_comp[8];
	comp->z2 = EXTRACT_UINT16(raw_comp[12], raw_comp[11]);
	comp->z1 = EXTRACT_UINT16(raw_comp[14], raw_comp[13]);
	comp->xyz1 = EXTRACT_UINT16(raw_comp[16], raw_comp[15]);
	comp->z3 = EXTRACT_UINT16(raw_comp[18], raw_comp[17]);
	comp->xy2 = raw_comp[19];
	comp->xy1 = raw_comp[20];

	return QM_RC_OK;
}

typedef enum {
	AXIS_X,
	AXIS_Y,
} axis_t;

static int compensate_xy(struct compensation *const comp, int16_t rhall,
			 int16_t raw, axis_t axis)
{
	int inter = 0;
	int value;

	if (rhall != 0) {
		inter = ((int)comp->xyz1 << 14) / rhall - (1 << 14);
	}

	if (BMC150_FLIP_OVERFLOW_ADCVAL == raw) {
		return BMC150_OVERFLOW_OUTPUT;
	}

	value = comp->xy2 * ((inter * inter) >> 7);
	value += inter * ((int)comp->xy1 << 7);
	value >>= 9;
	value += 1 << (8 + 12);
	value *= (int)(AXIS_X == axis ? comp->x2 : comp->y2) + 160;
	value >>= 12;
	value *= raw;
	value >>= 13;
	value += (int)(AXIS_X == axis ? comp->x1 : comp->y1) << 3;

	return value;
}

static int compensate_z(struct compensation *const comp, int rhall, int16_t raw)
{
	int dividend, divisor;
	int value;

	if (BMC150_HALL_OVERFLOW_ADCVAL == raw) {
		return BMC150_OVERFLOW_OUTPUT;
	}

	dividend = (raw - (int)comp->z4) << 15;
	dividend -= (comp->z3 * (rhall - (int)comp->xyz1)) >> 2;
	/* add 1 << 15 to round to next integer. */
	divisor = (int)comp->z1 * (rhall << 1) + (1 << 15);
	divisor >>= 16;
	divisor += (int)comp->z2;
	value = dividend / divisor;
	if (value > (1 << 15) || value < -(1 << 15)) {
		value = BMC150_OVERFLOW_OUTPUT;
	}

	return value;
}

qm_rc_t bmc150_read_mag(bmc150_mag_t *const mag)
{
	qm_rc_t rc = QM_RC_OK;
	uint8_t raw_mag[8];
	int16_t x, y, z, rhall;

	if (!digital_comp.init) {
		rc = read_dig_comp(&digital_comp);
		if (QM_RC_OK == rc) {
			digital_comp.init = true;
		}
	}

	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = read_register(magneto_addr, BMC150_REG_MAG_X_LSB, raw_mag,
			   sizeof(raw_mag));
	if (QM_RC_OK != rc) {
		return rc;
	}

	x = EXTRACT_MAG_XY(raw_mag[1], raw_mag[0]);
	y = EXTRACT_MAG_XY(raw_mag[3], raw_mag[2]);
	z = EXTRACT_MAG_Z(raw_mag[5], raw_mag[4]);
	rhall = EXTRACT_RHALL(raw_mag[7], raw_mag[6]);

	mag->x = compensate_xy(&digital_comp, rhall, x, AXIS_X);
	mag->y = compensate_xy(&digital_comp, rhall, y, AXIS_Y);
	mag->z = compensate_z(&digital_comp, rhall, z);

	return QM_RC_OK;
}

qm_rc_t bmc150_mag_set_power(bmc150_mag_power_t power)
{
	return write_register(magneto_addr, BMC150_REG_MAG_POWER_MODES, power);
}

/*
 * These values used for the presets are from Table 6 (page 24) of the
 * BMC150 datasheet.
 */
static const uint8_t operation_modes[] = {
    0,      /* 10 Hz */
    0,      /* 10 Hz */
    0,      /* 10 Hz */
    5 << 3, /* 20 Hz */
};

/* The formula here is repetions = 1 + 2*(mode) */
static const uint8_t repetion_modes_xy[] = {
    1, 3, 7, 23,
};

/* The formula here is repetions = 1 + (mode) */
static const uint8_t repetion_modes_z[] = {
    2, 14, 26, 82,
};

qm_rc_t bmc150_mag_set_preset(bmc150_mag_preset_t preset)
{
	qm_rc_t rc;

	rc = write_register(magneto_addr, BMC150_REG_MAG_OPERATION_MODES,
			    operation_modes[preset]);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = write_register(magneto_addr, BMC150_REG_MAG_REPETION_XY,
			    repetion_modes_xy[preset]);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = write_register(magneto_addr, BMC150_REG_MAG_REPETION_Z,
			    repetion_modes_z[preset]);
	if (QM_RC_OK != rc) {
		return rc;
	}

	return QM_RC_OK;
}

qm_rc_t bmc150_init(bmc150_j14_pos_t pos)
{
	qm_rc_t rc;

	rc = clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_I2C_M0_REGISTER);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = qm_pmux_select(QM_PIN_ID_6, QM_PMUX_FN_2);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = qm_pmux_select(QM_PIN_ID_7, QM_PMUX_FN_2);
	if (QM_RC_OK != rc) {
		return rc;
	}

	rc = qm_i2c_set_config(QM_I2C_0, &config);
	if (QM_RC_OK != rc) {
		return rc;
	}

	if (pos == BMC150_J14_POS_1) {
		accel_addr = 0x11;
		magneto_addr = 0x13;
	}

	return QM_RC_OK;
}
