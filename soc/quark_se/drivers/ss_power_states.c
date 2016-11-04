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

#include "power_states.h"
#include "ss_power_states.h"
#include "qm_isr.h"
#include "qm_sensor_regs.h"

/* Sensor Subsystem sleep operand definition.
 * Only a subset applies as internal sensor RTC
 * is not available.
 *
 *  OP | Core | Timers | RTC
 * 000 |    0 |      1 |   1 <-- used for SS1
 * 001 |    0 |      0 |   1
 * 010 |    0 |      1 |   0
 * 011 |    0 |      0 |   0 <-- used for SS2
 * 100 |    0 |      0 |   0
 * 101 |    0 |      0 |   0
 * 110 |    0 |      0 |   0
 * 111 |    0 |      0 |   0
 *
 * sleep opcode argument:
 *  - [7:5] : Sleep Operand
 *  - [4]   : Interrupt enable
 *  - [3:0] : Interrupt threshold value
 */
#define QM_SS_SLEEP_MODE_CORE_OFF (0x0)
#define QM_SS_SLEEP_MODE_CORE_OFF_TIMER_OFF (0x20)
#define QM_SS_SLEEP_MODE_CORE_TIMERS_RTC_OFF (0x60)

void ss_power_soc_lpss_enable()
{
	uint32_t creg_mst0_ctrl = 0;

	creg_mst0_ctrl = __builtin_arc_lr(QM_SS_CREG_BASE);

	/*
	 * Clock gate the sensor peripherals at CREG level.
	 * This clock gating is independent of the peripheral-specific clock
	 * gating provided in ss_clk.h .
	 */
	creg_mst0_ctrl |= (QM_SS_IO_CREG_MST0_CTRL_ADC_CLK_GATE |
			   QM_SS_IO_CREG_MST0_CTRL_I2C1_CLK_GATE |
			   QM_SS_IO_CREG_MST0_CTRL_I2C0_CLK_GATE |
			   QM_SS_IO_CREG_MST0_CTRL_SPI1_CLK_GATE |
			   QM_SS_IO_CREG_MST0_CTRL_SPI0_CLK_GATE);

	__builtin_arc_sr(creg_mst0_ctrl, QM_SS_CREG_BASE);

	QM_SCSS_CCU->ccu_lp_clk_ctl |= QM_SCSS_CCU_SS_LPS_EN;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_LP_CLK_CTL);
}

void ss_power_soc_lpss_disable()
{
	uint32_t creg_mst0_ctrl = 0;

	creg_mst0_ctrl = __builtin_arc_lr(QM_SS_CREG_BASE);

	/*
	 * Restore clock gate of the sensor peripherals at CREG level.
	 * CREG is not used anywhere else so we can safely restore
	 * the configuration to its POR default.
	 */
	creg_mst0_ctrl &= ~(QM_SS_IO_CREG_MST0_CTRL_ADC_CLK_GATE |
			    QM_SS_IO_CREG_MST0_CTRL_I2C1_CLK_GATE |
			    QM_SS_IO_CREG_MST0_CTRL_I2C0_CLK_GATE |
			    QM_SS_IO_CREG_MST0_CTRL_SPI1_CLK_GATE |
			    QM_SS_IO_CREG_MST0_CTRL_SPI0_CLK_GATE);

	__builtin_arc_sr(creg_mst0_ctrl, QM_SS_CREG_BASE);

	QM_SCSS_CCU->ccu_lp_clk_ctl &= ~QM_SCSS_CCU_SS_LPS_EN;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_LP_CLK_CTL);
}

/* Enter SS1 :
 * SLEEP + sleep operand
 * __builtin_arc_sleep is not used here as it does not propagate sleep operand.
 */
void ss_power_cpu_ss1(const ss_power_cpu_ss1_mode_t mode)
{
	/* The sensor cannot be woken up with an edge triggered
	 * interrupt from the RTC and the AON Counter.
	 * Switch to Level triggered interrupts and restore
	 * the setting when waking up.
	 */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	__builtin_arc_sr(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	/* Enter SS1 */
	switch (mode) {
	case SS_POWER_CPU_SS1_TIMER_OFF:
		__asm__ __volatile__(
		    "sleep %0"
		    :
		    : "i"(QM_SS_SLEEP_MODE_CORE_OFF_TIMER_OFF));
		break;
	case SS_POWER_CPU_SS1_TIMER_ON:
	default:
		__asm__ __volatile__("sleep %0"
				     :
				     : "i"(QM_SS_SLEEP_MODE_CORE_OFF));
		break;
	}

	/* Restore the RTC and AONC to edge interrupt after when waking up. */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_EDGE_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	__builtin_arc_sr(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_EDGE_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);
}

/* Enter SS2 :
 * SLEEP + sleep operand
 * __builtin_arc_sleep is not used here as it does not propagate sleep operand.
 */
void ss_power_cpu_ss2(void)
{
	/* The sensor cannot be woken up with an edge triggered
	 * interrupt from the RTC and the AON Counter.
	 * Switch to Level triggered interrupts and restore
	 * the setting when waking up.
	 */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	__builtin_arc_sr(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_LEVEL_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	/* Enter SS2 */
	__asm__ __volatile__("sleep %0"
			     :
			     : "i"(QM_SS_SLEEP_MODE_CORE_TIMERS_RTC_OFF));

	/* Restore the RTC and AONC to edge interrupt after when waking up. */
	__builtin_arc_sr(QM_IRQ_RTC_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_EDGE_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);

	__builtin_arc_sr(QM_IRQ_AONPT_0_INT_VECTOR, QM_SS_AUX_IRQ_SELECT);
	__builtin_arc_sr(QM_SS_IRQ_EDGE_SENSITIVE, QM_SS_AUX_IRQ_TRIGGER);
}

#if (ENABLE_RESTORE_CONTEXT)
extern uint32_t arc_restore_addr;
uint32_t cpu_context[33];
void ss_power_soc_sleep_restore(void)
{
	/*
	 * Save sensor restore trap address.
	 * The first parameter in this macro represents the label defined in
	 * the qm_ss_restore_context() macro, which is actually the restore
	 * trap address.
	 */
	qm_ss_set_resume_vector(sleep_restore_trap, arc_restore_addr);

	/* Save ARC execution context. */
	qm_ss_save_context(cpu_context);

	/* Set restore flags. */
	power_soc_set_ss_restore_flag();

	/* Enter sleep. */
	power_soc_sleep();

	/*
	 * Restore sensor execution context.
	 * The sensor startup code will jump to this location after waking up
	 * from sleep. The restore trap address is the label defined in the
	 * macro and the label is exposed here through the first parameter.
	 */
	qm_ss_restore_context(sleep_restore_trap, cpu_context);
}
void ss_power_soc_deep_sleep_restore(void)
{
	/*
	 * Save sensor restore trap address.
	 * The first parameter in this macro represents the label defined in
	 * the qm_ss_restore_context() macro, which is actually the restore
	 * trap address.
	 */
	qm_ss_set_resume_vector(deep_sleep_restore_trap, arc_restore_addr);

	/* Save ARC execution context. */
	qm_ss_save_context(cpu_context);

	/* Set restore flags. */
	power_soc_set_ss_restore_flag();

	/* Enter sleep. */
	power_soc_deep_sleep();

	/*
	 * Restore sensor execution context.
	 * The sensor startup code will jump to this location after waking up
	 * from sleep. The restore trap address is the label defined in the
	 * macro and the label is exposed here through the first parameter.
	 */
	qm_ss_restore_context(deep_sleep_restore_trap, cpu_context);
}

void ss_power_sleep_wait(void)
{
	/*
	 * Save sensor restore trap address.
	 * The first parameter in this macro represents the label defined in
	 * the qm_ss_restore_context() macro, which is actually the restore
	 * trap address.
	 */
	qm_ss_set_resume_vector(sleep_restore_trap, arc_restore_addr);

	/* Save ARC execution context. */
	qm_ss_save_context(cpu_context);

	/* Set restore flags. */
	power_soc_set_ss_restore_flag();

	/* Enter SS1 and stay in it until sleep and wake-up. */
	while (1) {
		ss_power_cpu_ss1(SS_POWER_CPU_SS1_TIMER_ON);
	}

	/*
	 * Restore sensor execution context.
	 * The sensor startup code will jump to this location after waking up
	 * from sleep. The restore trap address is the label defined in the
	 * macro and the label is exposed here through the first parameter.
	 */
	qm_ss_restore_context(sleep_restore_trap, cpu_context);
}

void power_soc_set_ss_restore_flag(void)
{
	QM_SCSS_GP->gps0 |= BIT(QM_GPS0_BIT_SENSOR_WAKEUP);
}

#endif /* ENABLE_RESTORE_CONTEXT */
