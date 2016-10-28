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

#ifndef __QM_SS_ISR_H__
#define __QM_SS_ISR_H__

#include "qm_common.h"

/**
 * Sensor Subsystem Interrupt Service Routines.
 *
 * @defgroup groupSSISR SS ISR
 * @{
 */

/**
 * ISR for ADC interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_ADC_0_INT, qm_ss_adc_0_isr);
 * @endcode if IRQ based conversions are used.
 */
QM_ISR_DECLARE(qm_ss_adc_0_isr);

/**
 * ISR for ADC error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_ADC_0_ERROR_INT,
 * qm_ss_adc_0_error_isr);
 * @endcode if IRQ based conversions are used.
 */
QM_ISR_DECLARE(qm_ss_adc_0_error_isr);

/**
 * ISR for SS ADC 0 calibration interrupt.
 *
 * This function needs to be registered with
 * @code qm_irq_request(QM_SS_IRQ_ADC_0_CAL_INT, qm_ss_adc_0_cal_isr);
 * @endcode if IRQ based calibration is used.
 */
QM_ISR_DECLARE(qm_ss_adc_0_cal_isr);

/**
 * ISR for SS ADC 0 mode change interrupt.
 *
 * This function needs to be registered with
 * @code qm_irq_request(QM_SS_IRQ_ADC_0_PWR_INT, qm_ss_adc_0_pwr_isr);
 * @endcode if IRQ based mode change is used.
 */
QM_ISR_DECLARE(qm_ss_adc_0_pwr_isr);

/**
 * ISR for GPIO 0 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_GPIO_0_INT, qm_ss_gpio_0_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_gpio_0_isr);

/**
 * ISR for GPIO 1 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_GPIO_1_INT, qm_ss_gpio_1_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_gpio_1_isr);

/**
 * ISR for I2C 0 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_0_ERROR_INT, qm_ss_i2c_0_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_0_RX_AVAIL_INT, qm_ss_i2c_0_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_0_TX_REQ_INT, qm_ss_i2c_0_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_0_STOP_DET_INT, qm_ss_i2c_0_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_i2c_0_isr);

/**
 * ISR for I2C 1 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_1_ERROR_INT, qm_ss_i2c_1_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_1_RX_AVAIL_INT, qm_ss_i2c_1_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_1_TX_REQ_INT, qm_ss_i2c_1_isr);
 * @code qm_ss_irq_request(QM_SS_IRQ_I2C_1_STOP_DET_INT, qm_ss_i2c_1_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_i2c_1_isr);

/**
 * ISR for SPI 0 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_0_ERROR_INT,
 * qm_ss_spi_0_error_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_0_error_isr);

/**
 * ISR for SPI 1 error interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_1_ERROR_INT,
 * qm_ss_spi_1_error_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_1_error_isr);

/**
 * ISR for SPI 0 TX data requested interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_0_TX_REQ_INT,
 * qm_ss_spi_0_tx_req_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_0_tx_req_isr);

/**
 * ISR for SPI 1 TX data requested interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_1_TX_REQ_INT,
 * qm_ss_spi_1_tx_req_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_1_tx_req_isr);

/**
 * ISR for SPI 0 RX data available interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_0_RX_AVAIL_INT,
 * qm_ss_spi_0_rx_avail_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_0_rx_avail_isr);

/**
 * ISR for SPI 1 data available interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_irq_request(QM_SS_IRQ_SPI_1_RX_AVAIL_INT,
 * qm_ss_spi_1_rx_avail_isr);
 * @endcode if IRQ based transfers are used.
 */
QM_ISR_DECLARE(qm_ss_spi_1_rx_avail_isr);

/**
 * ISR for SS Timer 0 interrupt.
 *
 * This function needs to be registered with
 * @code qm_ss_int_vector_request(QM_ARC_TIMER_0_INT, qm_ss_timer_0_isr);
 * @endcode
 */
QM_ISR_DECLARE(qm_ss_timer_0_isr);

/**
 * @}
 */

#endif /* __QM_SS_ISR_H__ */
