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
#include "clk.h"
#include "qm_comparator.h"
#include "qm_isr.h"
#include "qm_adc.h"
#include "qm_flash.h"
#include "soc_watch.h"

typedef struct {
	uint32_t ac_power_save;
	uint32_t clk_gate_save;
	uint32_t sys_clk_ctl_save;
	uint32_t osc0_cfg_save;
	uint32_t osc1_cfg_save;
	uint32_t adc_mode_save;
	uint32_t aon_vr_save;
	uint32_t flash_tmg_save;
	uint32_t ext_clock_save;
	uint32_t lp_clk_save;
	uint32_t pmux_slew_save;
} power_context_t;

static power_context_t power_context;

void power_cpu_halt(void)
{
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_HALT, 0);
	/*
	 * STI sets the IF flag. After the IF flag is set,
	 * the core begins responding to external,
	 * maskable interrupts after the next instruction is executed.
	 * When this function is called with interrupts disabled,
	 * this guarantees that an interrupt is caught only
	 * after the processor has transitioned into HLT.
	 */
	__asm__ __volatile__("sti\n\t"
			     "hlt\n\t");
}

static void clear_all_pending_interrupts(void)
{
	/* Clear comparator interrupts. */
	QM_SCSS_CMP->cmp_stat_clr = -1;

	/* Clear RTC interrupts. */
	QM_RTC->rtc_eoi;

	/* Clear timers interrupt flag. */
	QM_PWM->timerseoi;

	/* Clear GPIO interrupts. */
	QM_GPIO[QM_GPIO_0]->gpio_porta_eoi = -1;
}

void power_soc_sleep(void)
{
	/* Save register values. */
	power_context.ac_power_save = QM_SCSS_CMP->cmp_pwr;
	power_context.clk_gate_save = QM_SCSS_CCU->ccu_periph_clk_gate_ctl;
	power_context.sys_clk_ctl_save = QM_SCSS_CCU->ccu_sys_clk_ctl;
	power_context.osc0_cfg_save = QM_SCSS_CCU->osc0_cfg1;
	power_context.adc_mode_save = QM_ADC->adc_op_mode;
	power_context.flash_tmg_save = QM_FLASH[QM_FLASH_0]->tmg_ctrl;
	power_context.lp_clk_save = QM_SCSS_CCU->ccu_lp_clk_ctl;

	QM_SCSS_GP->gps0 |= QM_GPS0_POWER_STATE_SLEEP;

	/* Clear any pending interrupts. */
	clear_all_pending_interrupts();

	qm_adc_set_mode(QM_ADC_0, QM_ADC_MODE_PWR_DOWN);

	/* Turn off high power comparators. */
	QM_SCSS_CMP->cmp_pwr &= QM_AC_HP_COMPARATORS_MASK;

	/*
	 * Program WAKE_MASK.WAKE_MASK[31:0],
	 * CCU_LP_CLK_CTL.WAKE_PROBE_MODE_MASK registers identical to Interrupt
	 * Mask registers.
	 */
	QM_SCSS_CCU->ccu_lp_clk_ctl &= ~QM_WAKE_PROBE_MODE_MASK;
	/* Enable all wake sources as interrupts. */
	QM_SCSS_CCU->wake_mask = 0;

	/*
	 * Ensure that powering down of oscillators is delayed by hardware until
	 * core  executes HALT instruction.
	 */
	/* HYB_OSC_PD_LATCH_EN = 0, RTC_OSC_PD_LATCH_EN=0 */
	QM_SCSS_CCU->ccu_lp_clk_ctl &=
	    ~(QM_HYB_OSC_PD_LATCH_EN | QM_RTC_OSC_PD_LATCH_EN);

	/* Ensure that at exit, hardware will switch system clock to Hybrid
	 * oscillator clock so as to minimize exit latency by running at higher
	 * frequency than RTC clock.
	 */
	/* CCU_LP_CLK_CTL.CCU_EXIT_TO_HYBOSC */
	QM_SCSS_CCU->ccu_lp_clk_ctl |=
	    QM_CCU_EXIT_TO_HYBOSC | QM_CCU_MEM_HALT_EN | QM_CCU_CPU_HALT_EN;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_LP_CLK_CTL);

	/*
	 * Only the following peripherals can be used as a wakeup source:
	 *  - GPIO Interrupts
	 *  - AON timers
	 *  - RTC
	 *  - low power comparators
	 */
	clk_periph_disable(
	    CLK_PERIPH_I2C_M0 | CLK_PERIPH_SPI_S | CLK_PERIPH_SPI_M0 |
	    CLK_PERIPH_GPIO_DB | CLK_PERIPH_WDT_REGISTER |
	    CLK_PERIPH_PWM_REGISTER | CLK_PERIPH_GPIO_REGISTER |
	    CLK_PERIPH_SPI_M0_REGISTER | CLK_PERIPH_SPI_S_REGISTER |
	    CLK_PERIPH_UARTA_REGISTER | CLK_PERIPH_UARTB_REGISTER |
	    CLK_PERIPH_I2C_M0_REGISTER);

	/* Set system clock source to hyb osc, 4 MHz, scaled to 512 kHz. */
	clk_sys_set_mode(CLK_SYS_HYB_OSC_4MHZ, CLK_SYS_DIV_8);

	/* Set the RAR to retention mode. */
	rar_set_mode(RAR_RETENTION);

	/*
	 * If wake source is any of AON Timer, RTC, GPIO interrupt, program
	 * CCU_SYS_CLK_CTL.CCU_SYS_CLK_SEL to RTC Oscillator.
	 */
	/* Enter SoC sleep mode. */
	power_cpu_halt();
}

void power_soc_sleep_restore(void)
{
	/* From here on, restore the SoC to an active state. */
	/* Set the RAR to normal mode. */
	rar_set_mode(RAR_NORMAL);

	/*
	 * Since we are running below 4MHz, 0 wait states are configured.
	 * If the previous frequency was > 4MHz, 0 wait states will
	 * violate the flash timings.
	 * In the worst case scenario, when switching back to 32MHz,
	 * 2 wait states will be restored.
	 * This setting will be too conservative until the frequency has been
	 * restored.
	 */
	QM_FLASH[QM_FLASH_0]->tmg_ctrl = power_context.flash_tmg_save;

	/* Restore all previous values. */
	QM_SCSS_CCU->ccu_sys_clk_ctl = power_context.sys_clk_ctl_save;
	/* Re-apply clock divider values. DIV_EN must go 0 -> 1. */
	QM_SCSS_CCU->ccu_sys_clk_ctl &=
	    ~(QM_CCU_SYS_CLK_DIV_EN | QM_CCU_RTC_CLK_DIV_EN);
	QM_SCSS_CCU->ccu_sys_clk_ctl |=
	    QM_CCU_SYS_CLK_DIV_EN | QM_CCU_RTC_CLK_DIV_EN;

	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_SYS_CLK_CTL);

	/* Wait for the XTAL or SI oscillator to stabilise. */
	while (!(QM_SCSS_CCU->osc0_stat1 &
		 (QM_OSC0_LOCK_SI | QM_OSC0_LOCK_XTAL))) {
	};

	/* Restore original clocking, ADC, analog comparator states. */
	QM_SCSS_CCU->osc0_cfg1 = power_context.osc0_cfg_save;
	QM_SCSS_CCU->ccu_periph_clk_gate_ctl = power_context.clk_gate_save;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_OSC0_CFG1);
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER,
			    SOCW_REG_CCU_PERIPH_CLK_GATE_CTL);
	QM_SCSS_CMP->cmp_pwr = power_context.ac_power_save;
	QM_ADC->adc_op_mode = power_context.adc_mode_save;
	QM_SCSS_CCU->ccu_lp_clk_ctl = power_context.lp_clk_save;

	QM_SCSS_GP->gps0 &= ~QM_GPS0_POWER_STATE_SLEEP;
}

void power_soc_deep_sleep(const power_wake_event_t wake_event)
{
	/* Save register values. */
	power_context.ac_power_save = QM_SCSS_CMP->cmp_pwr;
	power_context.clk_gate_save = QM_SCSS_CCU->ccu_periph_clk_gate_ctl;
	power_context.sys_clk_ctl_save = QM_SCSS_CCU->ccu_sys_clk_ctl;
	power_context.osc0_cfg_save = QM_SCSS_CCU->osc0_cfg1;
	power_context.osc1_cfg_save = QM_SCSS_CCU->osc1_cfg0;
	power_context.adc_mode_save = QM_ADC->adc_op_mode;
	power_context.aon_vr_save = QM_SCSS_PMU->aon_vr;
	power_context.flash_tmg_save = QM_FLASH[QM_FLASH_0]->tmg_ctrl;
	power_context.pmux_slew_save = QM_SCSS_PMUX->pmux_slew[0];
	power_context.ext_clock_save = QM_SCSS_CCU->ccu_ext_clock_ctl;
	power_context.lp_clk_save = QM_SCSS_CCU->ccu_lp_clk_ctl;

	QM_SCSS_GP->gps0 |= QM_GPS0_POWER_STATE_DEEP_SLEEP;

	/* Clear any pending interrupts. */
	clear_all_pending_interrupts();

	/*
	 * Clear the wake mask bits. Default behaviour is to wake from GPIO /
	 * comparator.
	 */
	switch (wake_event) {
	case POWER_WAKE_FROM_RTC:
		QM_SCSS_CCU->wake_mask =
		    SET_ALL_BITS & ~QM_CCU_WAKE_MASK_RTC_BIT;
		break;
	case POWER_WAKE_FROM_GPIO_COMP:
	default:
		QM_SCSS_CCU->wake_mask = SET_ALL_BITS &
					 ~(QM_CCU_WAKE_MASK_COMPARATOR_BIT |
					   QM_CCU_WAKE_MASK_GPIO_BIT);
		break;
	}

	qm_adc_set_mode(QM_ADC_0, QM_ADC_MODE_DEEP_PWR_DOWN);

	/* Turn off high power comparators. */
	QM_SCSS_CMP->cmp_pwr &= QM_AC_HP_COMPARATORS_MASK;

	/* Disable all peripheral clocks. */
	clk_periph_disable(CLK_PERIPH_REGISTER);

	/* Disable external clocks. */
	QM_SCSS_CCU->ccu_ext_clock_ctl = 0;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_EXT_CLK_CTL);

	/* Set slew rate of all pins to 12mA. */
	QM_SCSS_PMUX->pmux_slew[0] = 0;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_PMUX_SLEW);

	if (wake_event != POWER_WAKE_FROM_RTC) {
		/* Disable RTC. */
		QM_SCSS_CCU->osc1_cfg0 &= ~QM_OSC1_PD;

		/* Set system clock source to
		 * Silicon Oscillator 4 MHz, scaled down to 32 kHz. */
		clk_sys_set_mode(CLK_SYS_HYB_OSC_4MHZ, CLK_SYS_DIV_128);
	}

	/* Power down the oscillator after the halt instruction is executed. */
	QM_SCSS_CCU->ccu_lp_clk_ctl &= ~QM_HYB_OSC_PD_LATCH_EN;
	/*
	 * Enable memory halt and CPU halt. When exiting sleep mode, use hybrid
	 * oscillator.
	 */
	QM_SCSS_CCU->ccu_lp_clk_ctl |=
	    QM_CCU_EXIT_TO_HYBOSC | QM_CCU_MEM_HALT_EN | QM_CCU_CPU_HALT_EN;
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_LP_CLK_CTL);

	/* Power down hybrid oscillator. */
	QM_SCSS_CCU->osc0_cfg1 |= QM_OSC0_PD;

	/* Disable gpio debounce clocking. */
	QM_SCSS_CCU->ccu_gpio_db_clk_ctl &= ~QM_CCU_GPIO_DB_CLK_EN;
	/* Set retention voltage to 1.35V. */
	/* SCSS.OSC0_CFG0.OSC0_HYB_SET_REG1.OSC0_CFG0[0]  = 1; */
	QM_SCSS_CCU->osc0_cfg0 |= QM_SI_OSC_1V2_MODE;

	/* Enable low voltage mode for flash controller. */
	/* FlashCtrl.CTRL.LVE_MODE = 1; */
	QM_FLASH[QM_FLASH_0]->ctrl |= QM_FLASH_LVE_MODE;

	/* Select 1.35V for voltage regulator. */
	/* SCSS.AON_VR.VSEL = 0xB; */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE |
	     (power_context.aon_vr_save & QM_AON_VR_VSEL_MASK) |
	     QM_AON_VR_VSEL_1V35);
	/* SCSS.AON_VR.ROK_BUF_VREG_MASK = 1; */
	QM_SCSS_PMU->aon_vr = (QM_AON_VR_PASS_CODE | QM_SCSS_PMU->aon_vr |
			       QM_AON_VR_ROK_BUF_VREG_MASK);

	/* SCSS.AON_VR.VSEL_STROBE = 1;  */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE | QM_SCSS_PMU->aon_vr | QM_AON_VR_VSTRB);

	/* Wait >= 1 usec, at 256 kHz this is 1 cycle. */
	__asm__ __volatile__("nop");

	/* SCSS.AON_VR.VSEL_STROBE = 0; */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE | (QM_SCSS_PMU->aon_vr & ~QM_AON_VR_VSTRB));

	/* Wait >= 2 usec, at 256 kHz this is 1 cycle. */
	__asm__ __volatile__("nop");

	/* Set the RAR to retention mode. */
	rar_set_mode(RAR_RETENTION);

	if (wake_event == POWER_WAKE_FROM_RTC) {
		/* Start running on the rtc clock */
		clk_sys_set_mode(CLK_SYS_RTC_OSC, CLK_SYS_DIV_1);
	}

	/* Disable all peripheral clocks. */
	clk_periph_disable(CLK_PERIPH_REGISTER | CLK_PERIPH_CLK);

	/* Enter SoC deep sleep mode. */
	power_cpu_halt();
}

void power_soc_deep_sleep_restore(void)
{
	/* We are now exiting from deep sleep mode. */
	/* Set the RAR to normal mode. */
	rar_set_mode(RAR_NORMAL);

	/*
	 * Since we are running below 4MHz, 0 wait states are configured.
	 * If the previous frequency was > 4MHz, 0 wait states will
	 * violate the flash timings.
	 * In the worst case scenario, when switching back to 32MHz,
	 * 2 wait states will be restored.
	 * This setting will be too conservative until the frequency has been
	 * restored.
	 */
	QM_FLASH[QM_FLASH_0]->tmg_ctrl = power_context.flash_tmg_save;

	/* Restore operating voltage to 1.8V. */
	/* SCSS.AON_VR.VSEL = 0x10; */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE | (QM_SCSS_PMU->aon_vr & QM_AON_VR_VSEL_MASK) |
	     QM_AON_VR_VSEL_1V8 | QM_AON_VR_ROK_BUF_VREG_MASK);

	/* SCSS.AON_VR.VSEL_STROBE = 1;  */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE | QM_SCSS_PMU->aon_vr | QM_AON_VR_VSTRB);

	/* Wait >= 1 usec, at 256 kHz this is 1 cycle. */
	__asm__ __volatile__("nop");

	/* SCSS.AON_VR.VSEL_STROBE = 0; */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE | (QM_SCSS_PMU->aon_vr & ~QM_AON_VR_VSTRB));

	/* Wait >= 2 usec, at 256 kHz this is 1 cycle. */
	__asm__ __volatile__("nop");

	/* SCSS.AON_VR.ROK_BUF_VREG_MASK = 0;  */
	QM_SCSS_PMU->aon_vr =
	    (QM_AON_VR_PASS_CODE |
	     (QM_SCSS_PMU->aon_vr & ~QM_AON_VR_ROK_BUF_VREG_MASK));

	/* Wait >= 1 usec, at 256 kHz this is 1 cycle. */
	__asm__ __volatile__("nop");

	/* Wait for voltage regulator to attain 1.8V regulation. */
	while (!(QM_SCSS_PMU->aon_vr & QM_AON_VR_ROK_BUF_VREG_STATUS)) {
	}

	/* SCSS.OSC0_CFG0.OSC0_HYB_SET_REG1.OSC0_CFG0[0]  = 0; */
	QM_SCSS_CCU->osc0_cfg0 &= ~QM_SI_OSC_1V2_MODE;

	/* FlashCtrl.CTRL.LVE_MODE = 0; */
	QM_FLASH[QM_FLASH_0]->ctrl &= ~QM_FLASH_LVE_MODE;

	/* Restore all previous values. */
	QM_SCSS_CCU->ccu_sys_clk_ctl = power_context.sys_clk_ctl_save;
	/* Re-apply clock divider values. DIV_EN must go 0 -> 1. */
	QM_SCSS_CCU->ccu_sys_clk_ctl &=
	    ~(QM_CCU_SYS_CLK_DIV_EN | QM_CCU_RTC_CLK_DIV_EN);
	QM_SCSS_CCU->ccu_sys_clk_ctl |=
	    QM_CCU_SYS_CLK_DIV_EN | QM_CCU_RTC_CLK_DIV_EN;

	/* Wait for the XTAL or SI oscillator to stabilise. */
	while (!(QM_SCSS_CCU->osc0_stat1 &
		 (QM_OSC0_LOCK_SI | QM_OSC0_LOCK_XTAL))) {
	};
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_SYS_CLK_CTL);

	/* Re-enable clocks. */
	clk_periph_enable(CLK_PERIPH_REGISTER);

	/* Re-enable gpio debounce clocking. */
	QM_SCSS_CCU->ccu_gpio_db_clk_ctl |= QM_CCU_GPIO_DB_CLK_EN;

	/* Restore original clocking, ADC, analog comparator states. */
	QM_SCSS_CCU->osc0_cfg1 = power_context.osc0_cfg_save;
	QM_SCSS_CCU->ccu_periph_clk_gate_ctl = power_context.clk_gate_save;
	QM_SCSS_CCU->osc1_cfg0 = power_context.osc1_cfg_save;

	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_OSC0_CFG1);
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER,
			    SOCW_REG_CCU_PERIPH_CLK_GATE_CTL);
	QM_SCSS_CMP->cmp_pwr = power_context.ac_power_save;
	QM_ADC->adc_op_mode = power_context.adc_mode_save;

	QM_SCSS_PMUX->pmux_slew[0] = power_context.pmux_slew_save;
	QM_SCSS_CCU->ccu_ext_clock_ctl = power_context.ext_clock_save;
	QM_SCSS_CCU->ccu_lp_clk_ctl = power_context.lp_clk_save;

	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_PMUX_SLEW);
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_LP_CLK_CTL);
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_REGISTER, SOCW_REG_CCU_EXT_CLK_CTL);

	QM_SCSS_CCU->wake_mask = SET_ALL_BITS;
	QM_SCSS_GP->gps0 &= ~QM_GPS0_POWER_STATE_DEEP_SLEEP;
}

void power_soc_restore(void)
{
	/*
	 * If the SoC is waking from sleep or deep sleep mode then the full
	 * system state must be restored.
	 */
	if (QM_SCSS_GP->gps0 & QM_GPS0_POWER_STATE_SLEEP) {
		power_soc_sleep_restore();
	} else if (QM_SCSS_GP->gps0 & QM_GPS0_POWER_STATE_DEEP_SLEEP) {
		power_soc_deep_sleep_restore();
	}
}

int rar_set_mode(const rar_state_t mode)
{
	QM_CHECK(mode <= RAR_RETENTION, -EINVAL);
	volatile uint32_t i = 32;
	volatile uint32_t reg;

	switch (mode) {
	case RAR_RETENTION:
		QM_SCSS_PMU->aon_vr |=
		    (QM_AON_VR_PASS_CODE | QM_AON_VR_ROK_BUF_VREG_MASK);
		QM_SCSS_PMU->aon_vr |=
		    (QM_AON_VR_PASS_CODE | QM_AON_VR_VREG_SEL);
		break;

	case RAR_NORMAL:
		reg = QM_SCSS_PMU->aon_vr & ~QM_AON_VR_VREG_SEL;
		QM_SCSS_PMU->aon_vr = QM_AON_VR_PASS_CODE | reg;
		/* Wait for >= 2usec, at most 64 clock cycles. */
		while (i--) {
			__asm__ __volatile__("nop");
		}
		reg = QM_SCSS_PMU->aon_vr & ~QM_AON_VR_ROK_BUF_VREG_MASK;
		QM_SCSS_PMU->aon_vr = QM_AON_VR_PASS_CODE | reg;
		break;
	}
	return 0;
}
