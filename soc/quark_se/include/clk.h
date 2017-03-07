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

#ifndef __CLK_H__
#define __CLK_H__

#include "qm_common.h"
#include "qm_soc_regs.h"
#if (QM_SENSOR)
#include "qm_sensor_regs.h"
#endif

/**
 * Clock Management.
 *
 * @defgroup groupCLK Clock Management
 * @{
 */

/*
 * When using an external crystal, this value must be set to the number of
 * system ticks per micro second. The expected value is 32 ticks for a 32MHz
 * crystal.
 */
#define SYS_TICKS_PER_US_XTAL (32)
/* System ticks per microseconds for a 32MHz oscillator. */
#define SYS_TICKS_PER_US_32MHZ (32)
/* System ticks per microseconds for a 16MHz oscillator. */
#define SYS_TICKS_PER_US_16MHZ (16)
/* System ticks per microseconds for a 8MHz oscillator. */
#define SYS_TICKS_PER_US_8MHZ (8)
/* System ticks per microseconds for a 4MHz oscillator. */
#define SYS_TICKS_PER_US_4MHZ (4)

/**
 * System clock divider type.
 */
typedef enum {
	CLK_SYS_DIV_1, /**< Clock Divider = 1. */
	CLK_SYS_DIV_2, /**< Clock Divider = 2. */
	CLK_SYS_DIV_4, /**< Clock Divider = 4. */
	CLK_SYS_DIV_8, /**< Clock Divider = 8. */
	CLK_SYS_DIV_NUM
} clk_sys_div_t;

/**
 * System clock mode type.
 */
typedef enum {
	CLK_SYS_HYB_OSC_32MHZ = 0, /**< 32MHz Hybrid Oscillator Clock. */
	CLK_SYS_HYB_OSC_16MHZ = 1, /**< 16MHz Hybrid Oscillator Clock. */
	CLK_SYS_HYB_OSC_8MHZ = 2,  /**< 8MHz Hybrid Oscillator Clock. */
	CLK_SYS_HYB_OSC_4MHZ = 3,  /**< 4MHz Hybrid Oscillator Clock. */
	CLK_SYS_RTC_OSC = 4,       /**< Real Time Clock. */
	CLK_SYS_CRYSTAL_OSC = 5    /**< Crystal Oscillator Clock. */
} clk_sys_mode_t;

/**
 * Peripheral clock divider type.
 */
typedef enum {
	CLK_PERIPH_DIV_1, /**< Peripheral Clock Divider = 1. */
	CLK_PERIPH_DIV_2, /**< Peripheral Clock Divider = 2. */
	CLK_PERIPH_DIV_4, /**< Peripheral Clock Divider = 4. */
	CLK_PERIPH_DIV_8  /**< Peripheral Clock Divider = 8. */
} clk_periph_div_t;

/**
 * GPIO clock debounce divider type.
 */
typedef enum {
	CLK_GPIO_DB_DIV_1,  /**< GPIO Clock Debounce Divider = 1. */
	CLK_GPIO_DB_DIV_2,  /**< GPIO Clock Debounce Divider = 2. */
	CLK_GPIO_DB_DIV_4,  /**< GPIO Clock Debounce Divider = 4. */
	CLK_GPIO_DB_DIV_8,  /**< GPIO Clock Debounce Divider = 8. */
	CLK_GPIO_DB_DIV_16, /**< GPIO Clock Debounce Divider = 16. */
	CLK_GPIO_DB_DIV_32, /**< GPIO Clock Debounce Divider = 32. */
	CLK_GPIO_DB_DIV_64, /**< GPIO Clock Debounce Divider = 64. */
	CLK_GPIO_DB_DIV_128 /**< GPIO Clock Debounce Divider = 128. */
} clk_gpio_db_div_t;

/**
 * External crystal clock divider type.
 */
typedef enum {
	CLK_EXT_DIV_1, /**< External Crystal Clock Divider = 1. */
	CLK_EXT_DIV_2, /**< External Crystal Clock Divider = 2. */
	CLK_EXT_DIV_4, /**< External Crystal Clock Divider = 4. */
	CLK_EXT_DIV_8  /**< External Crystal Clock Divider = 8. */
} clk_ext_div_t;

/**
 * RTC clock divider type.
 */
typedef enum {
	CLK_RTC_DIV_1,     /**< Real Time Clock Divider = 1. */
	CLK_RTC_DIV_2,     /**< Real Time Clock Divider = 2. */
	CLK_RTC_DIV_4,     /**< Real Time Clock Divider = 4. */
	CLK_RTC_DIV_8,     /**< Real Time Clock Divider = 8. */
	CLK_RTC_DIV_16,    /**< Real Time Clock Divider = 16. */
	CLK_RTC_DIV_32,    /**< Real Time Clock Divider = 32. */
	CLK_RTC_DIV_64,    /**< Real Time Clock Divider = 64. */
	CLK_RTC_DIV_128,   /**< Real Time Clock Divider = 128. */
	CLK_RTC_DIV_256,   /**< Real Time Clock Divider = 256. */
	CLK_RTC_DIV_512,   /**< Real Time Clock Divider = 512. */
	CLK_RTC_DIV_1024,  /**< Real Time Clock Divider = 1024. */
	CLK_RTC_DIV_2048,  /**< Real Time Clock Divider = 2048. */
	CLK_RTC_DIV_4096,  /**< Real Time Clock Divider = 4096. */
	CLK_RTC_DIV_8192,  /**< Real Time Clock Divider = 8192. */
	CLK_RTC_DIV_16384, /**< Real Time Clock Divider = 16384. */
	CLK_RTC_DIV_32768  /**< Real Time Clock Divider = 32768. */
} clk_rtc_div_t;

/**
 * Set clock mode and divisor.
 *
 * Change the operating mode and clock divisor of the system
 * clock source. Changing this clock speed affects all
 * peripherals.
 * This applies the correct trim code if available.
 *
 * If trim code is not available, it is not computed
 * and previous trim code is not modified.
 *
 * @param[in] mode System clock source operating mode.
 * @param[in] div System clock divisor.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_sys_set_mode(const clk_sys_mode_t mode, const clk_sys_div_t div);

/**
 * Read the silicon oscillator trim code for the current frequency.
 *
 * @param[out] value Pointer to store the trim code.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_trim_read(uint32_t *const value);

/**
 * Apply silicon oscillator trim code.
 *
 * @param[in] value Trim code to apply.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_trim_apply(const uint32_t value);

/**
 * Change divider value of peripheral clock.
 *
 * Change Peripheral clock divider value.
 * The maximum divisor is /8.
 * Refer to the list of supported peripherals for your SoC.
 *
 * @param[in] div Divider value for the peripheral clock.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_periph_set_div(const clk_periph_div_t div);

/**
 * Change divider value of GPIO debounce clock.
 *
 * Change GPIO debounce clock divider value.
 * The maximum divisor is /128.
 *
 * @param[in] div Divider value for the GPIO debounce clock.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_gpio_db_set_div(const clk_gpio_db_div_t div);

/**
 * Change divider value of external clock.
 *
 * Change External clock divider value.
 * The maximum divisor is /8.
 *
 * @param[in] div Divider value for the external clock.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_ext_set_div(const clk_ext_div_t div);

/**
 * Change divider value of RTC.
 *
 * Change RTC divider value.
 * The maximum divisor is /32768.
 *
 * @param[in] div Divider value for the RTC.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_rtc_set_div(const clk_rtc_div_t div);

/**
 * Enable clocks for peripherals / registers.
 *
 * @param[in] clocks Which peripheral and register clocks to enable.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_periph_enable(const clk_periph_t clocks);

/**
 * Disable clocks for peripherals / registers.
 *
 * @param[in] clocks Which peripheral and register clocks to disable.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_periph_disable(const clk_periph_t clocks);

/**
 * Get number of system ticks per micro second.
 *
 * @return uint32_t Number of system ticks per micro second.
 */
uint32_t clk_sys_get_ticks_per_us(void);

/**
 * Idle loop the processor for at least the value given in microseconds.
 *
 * This function will wait until at least the given number of microseconds has
 * elapsed since calling this function.
 *
 * Note:
 * It is dependent on the system clock speed.
 * The delay parameter does not include, calling the function, returning
 * from it, calculation setup and while loops.
 *
 * @param[in] microseconds Minimum number of micro seconds to delay for.
 */
void clk_sys_udelay(uint32_t microseconds);

/**
 * Enable the USB Clock mode.
 *
 * For Quark SE, this function will set the system clock to CLK_SYS_CRYSTAL_OSC
 * mode with CLK_SYS_DIV_1 divisor. It then will enable the USB Clock and PLL,
 * blocking execution until the USB PLL lock is ready.
 *
 * Note:
 * Application must retain the original system clock mode / divisor in case it
 * will need to restore it after disabling the usb clock.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_sys_usb_enable(void);

/**
 * Disable the USB Clock mode.
 *
 * This function will disable the USB Clock and PLL.
 * The system clock will remain as CLK_SYS_CRYSTAL_OSC mode and CLK_SYS_DIV_1
 * divider.
 *
 * Note:
 * If the application must restore the original system clock mode / divisor
 * it must explictly call clk_sys_set_mode() restoring the previous config.
 * This can only be done after the USB clock mode is disabled.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_sys_usb_disable(void);

/**
 * Enable the DMA clock.
 *
 * Enable the DMA clock by setting the corresponding bit in the AHB Control
 * register. By default the DMA clock is disabled.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_dma_enable(void);

/**
 * Disable the DMA clock.
 *
 * Disable the DMA clock by clearing the corresponding bit in the AHB Control
 * register.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int clk_dma_disable(void);

/**
 * Get I2C clock frequency in MHz.
 *
 * @return [uint32_t] I2C freq_in_mhz.
 */
uint32_t get_i2c_clk_freq_in_mhz(void);

/**
 * @}
 */

#endif /* __CLK_H__ */
