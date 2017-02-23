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
 * Sensor Subsystem (SS) Low Power Sensing Standby (LPSS) State
 *
 * This application must run in conjunction with its Host counterpart
 * located in ./examples/quark_se/power_lpss/.
 * Refer to the host application for the board setup.
 *
 * States executed in this example are:
 * LPSS: Combination of C2/C2LP (Host state) and SS2
 */

#include "clk.h"
#include "qm_gpio.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_rtc.h"
#include "qm_soc_regs.h"
#include "qm_ss_interrupt.h"
#include "qm_ss_isr.h"
#include "qm_ss_timer.h"
#include "qm_uart.h"
#include "ss_power_states.h"
#include "soc_watch.h"

#define PIN_OUT (0)

#define QM_SCSS_GP_SENSOR_READY BIT(2)

#define RTC_SYNC_CLK_COUNT (5)

#define GPIO_TOGGLE_DELAY 500

static uint32_t switch_rtc_to_level(void)
{
	uint32_t prev_trigger;

	/* The sensor cannot be woken up with an edge triggered
	 * interrupt from the RTC and the AON Counter.
	 * Switch to Level triggered interrupts and restore
	 * the setting when waking up.
	 */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	prev_trigger = __builtin_arc_lr(QM_SS_AUX_IRQ_TRIGGER);
	__builtin_arc_sr(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	return prev_trigger;
}

static void restore_rtc_trigger(uint32_t trigger)
{
	/* Restore the RTC interrupt trigger when waking up. */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(trigger, QM_SS_AUX_IRQ_TRIGGER);
}

int main(void)
{
	qm_rtc_config_t rtc_cfg;
	uint32_t aonc_start;
	uint32_t rtc_trigger;

	/*  Initialise RTC configuration. */
	rtc_cfg.init_val = 0; /* Set initial value to 0. */
	rtc_cfg.alarm_en = 1; /* Enable alarm. */
	rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1); /* 1s alarm. */
	rtc_cfg.callback = NULL;
	rtc_cfg.callback_data = NULL;
	rtc_cfg.prescaler = CLK_RTC_DIV_1;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	/*
	 * The RTC clock resides in a different clock domain
	 * to the system clock.
	 * It takes 3-4 RTC ticks for a system clock write to propagate
	 * to the RTC domain.
	 * If an entry to sleep is initiated without waiting for the
	 * transaction to complete the SOC will not wake from sleep.
	 */
	aonc_start = QM_AONC[QM_AONC_0]->aonc_cnt;
	while (QM_AONC[QM_AONC_0]->aonc_cnt - aonc_start < RTC_SYNC_CLK_COUNT) {
	}

	QM_IR_UNMASK_INT(QM_IRQ_RTC_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_RTC_0_INT, qm_rtc_0_isr);

	/*
	 * Enable LPSS by the Sensor Subsystem.
	 * This will clock gate sensor peripherals.
	 */
	qm_ss_power_soc_lpss_enable();

	/*
	 * Signal to the x86 core that the Sensor Subsystem
	 * is ready to enter LPSS mode.
	 */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_SENSOR_READY;

	rtc_trigger = switch_rtc_to_level();

	/* Go to LPSS, RTC will wake the Sensor Subsystem up. */
	qm_ss_power_cpu_ss2();

	/* Log the interrupt event in soc_watch. */
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_INTERRUPT, QM_IRQ_RTC_0_INT_VECTOR);

	restore_rtc_trigger(rtc_trigger);

	/* Clear the SENSOR_READY flag in General Purpose Sticky Register 2. */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_SENSOR_READY;

	/*
	 * Disable LPSS.
	 * This will restore clock gating of sensor peripherals.
	 */
	qm_ss_power_soc_lpss_disable();

	/* Core still in C2 mode. */
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);
	clk_sys_udelay(GPIO_TOGGLE_DELAY);
	qm_gpio_set_pin(QM_GPIO_0, PIN_OUT);
	clk_sys_udelay(GPIO_TOGGLE_DELAY);
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);

	/* Set another alarm 1 second from now. */
	qm_rtc_set_alarm(QM_RTC_0, QM_RTC[QM_RTC_0]->rtc_ccvr +
				       (QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1)));
	/*
	 * The RTC clock resides in a different clock domain
	 * to the system clock.
	 * It takes 3-4 RTC ticks for a system clock write to propagate
	 * to the RTC domain.
	 * If an entry to sleep is initiated without waiting for the
	 * transaction to complete the SOC will not wake from sleep.
	 */
	aonc_start = QM_AONC[QM_AONC_0]->aonc_cnt;
	while (QM_AONC[QM_AONC_0]->aonc_cnt - aonc_start < RTC_SYNC_CLK_COUNT) {
	}

	/*
	 * Enable LPSS by the Sensor Subsystem.
	 * This will clock gate sensor peripherals.
	 */
	qm_ss_power_soc_lpss_enable();

	/*
	 * Signal to the x86 core that the Sensor Subsystem
	 * is ready to enter LPSS mode.
	 */
	QM_SCSS_GP->gps2 |= QM_SCSS_GP_SENSOR_READY;

	rtc_trigger = switch_rtc_to_level();

	/* Go to LPSS, RTC will wake the Sensor Subsystem up. */
	qm_ss_power_cpu_ss2();

	/* Log the interrupt event in soc_watch. */
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_INTERRUPT, QM_IRQ_RTC_0_INT_VECTOR);
	restore_rtc_trigger(rtc_trigger);

	/* Clear the SENSOR_READY flag in General Purpose Sticky Register 2. */
	QM_SCSS_GP->gps2 &= ~QM_SCSS_GP_SENSOR_READY;

	/*
	 * Disable LPSS.
	 * This will restore clock gating of sensor peripherals.
	 */
	qm_ss_power_soc_lpss_disable();

	/* Core still in C2LP mode. */
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);
	clk_sys_udelay(GPIO_TOGGLE_DELAY);
	qm_gpio_set_pin(QM_GPIO_0, PIN_OUT);
	clk_sys_udelay(GPIO_TOGGLE_DELAY);
	qm_gpio_clear_pin(QM_GPIO_0, PIN_OUT);

	/* Trigger soc_watch flush. */
	SOC_WATCH_TRIGGER_FLUSH();

	return 0;
}
