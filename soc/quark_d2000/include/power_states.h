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

#ifndef __POWER_STATES_H__
#define __POWER_STATES_H__

#include "qm_common.h"
#include "qm_soc_regs.h"

/**
 * Power mode control for Quark D2000 Microcontrollers.
 *
 * @defgroup groupD2000Power Quark D2000 Power states
 * @{
 */

/**
* Wake source for deep sleep mode type.
*/
typedef enum {
	POWER_WAKE_FROM_GPIO_COMP, /**< Use GPIO / Comparator as wake source. */
	POWER_WAKE_FROM_RTC,       /**< Use RTC as wake source. */
} power_wake_event_t;

/**
 * Put CPU in halt state.
 *
 * Halts the CPU until next interrupt or reset.
 *
 * This function can be called with interrupts disabled.
 * Interrupts will be enabled before triggering the transition.
 */
void power_cpu_halt(void);

/**
 * Put SoC to sleep.
 *
 * Enter into sleep mode. The hybrid oscillator is disabled, most peripherals
 * are disabled and the voltage regulator is set into retention mode.
 * The following peripherals are disabled in this mode:
 *  - I2C
 *  - SPI
 *  - GPIO debouncing
 *  - Watchdog timer
 *  - PWM / Timers
 *  - UART
 *
 * The SoC operates from the 32 kHz clock source and the following peripherals
 * may bring the SoC back into an active state:
 *
 *  - GPIO interrupts
 *  - AON Timers
 *  - RTC
 *  - Low power comparators
 */
void power_soc_sleep();

/**
 * Put SoC to deep sleep.
 *
 * Enter into deep sleep mode. All clocks are gated. The Wake source for this
 * function depends on the input parameter, POWER_WAKE_FROM_GPIO_COMP will
 * enable waking from GPIO or comparator pins and POWER_WAKE_FROM_RTC will
 * enable waking from the RTC.
 *
 * @param[in] wake_event Select wake source for deep sleep mode.
 */
void power_soc_deep_sleep(const power_wake_event_t wake_event);

/**
 * Restore system state after sleep or deep sleep.
 *
 * On wakeup, the system is restored to the previous state before
 * power_soc_sleep() or power_soc_deep_sleep() was called.
 */
void power_soc_restore(void);

/**
 * Retention alternator regulator for Quark D2000.
 *
 * @defgroup groupRAR Quark(TM) D2000 Retention Alternator Regulator (RAR).
 * @{
 */

/**
 * RAR modes type.
 */
typedef enum {
	RAR_NORMAL,   /**< Normal mode = 50 mA. */
	RAR_RETENTION /**< Retention mode = 300 uA. */
} rar_state_t;

/**
 * Change operating mode of RAR.
 *
 * Normal mode is able to source up to 50 mA.
 * Retention mode is able to source up to 300 uA.
 * Care must be taken when entering into retention mode
 * to ensure the overall system draw is less than 300 uA.
 *
 * @param[in] mode Operating mode of the RAR.
 *
 * @return Standard errno return type for QMSI.
 * @retval 0 on success.
 * @retval Negative @ref errno for possible error codes.
 */
int rar_set_mode(const rar_state_t mode);

/**
 * @}
 */
#endif /* __POWER_STATES_H__ */
