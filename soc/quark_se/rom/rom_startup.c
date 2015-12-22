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

#include <string.h>
#include "apic.h"
#include "idt.h"
#include "qm_interrupt.h"
#include "qm_pinmux.h"
#include "qm_scss.h"

/* Address of application's entry point (Flash1) */
#define ENTRY_POINT_ADDRESS ((void *)0x40030000)

static void spurious_isr(void)
{
}

/*
 * System power settings
 */
static __inline__ void power_setup(void)
{
	/* Pin MUX slew rate settings */
	QM_PMUX_SLEW0 = QM_PMUX_SLEW_4MA_DRIVER;
	QM_PMUX_SLEW1 = QM_PMUX_SLEW_4MA_DRIVER;
	QM_PMUX_SLEW2 = QM_PMUX_SLEW_4MA_DRIVER;
	QM_PMUX_SLEW3 = QM_PMUX_SLEW_4MA_DRIVER;
}

/*
 * System clock settings
 */
static __inline__ void clock_setup(void)
{
	/* NOTE: Switch to silicon oscillator when trim data is available */
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);

	/* Turn off the clock gate */
	QM_CCU_MLAYER_AHB_CTL |= QM_CCU_USB_CLK_EN;

	/* Set up the PLL */
	QM_USB_PLL_CFG0 = QM_USB_PLL_CFG0_DEFAULT | QM_USB_PLL_PDLD;

	/* Wait for the PLL lock */
	while (0 == (QM_USB_PLL_CFG0 & QM_USB_PLL_LOCK)) {
	}
}

/*
 * Initialize IRQ on AON peripheral so as to catch a spurious interrupt after a
 * warm reset.
 */
static __inline__ void aon_handle_spurious_irq(void)
{
	/* The PIC IRR register may be asserted by the application before a warm
	 * reset. IRR cannot be cleared by software, so let's just catch this
	 * single spurious interrupt. */
	qm_irq_request(QM_IRQ_AONPT_0, spurious_isr);
	qm_irq_request(QM_IRQ_AONGPIO_0, spurious_isr);
}

/*
 * SCSS interrupt routing initalization.
 */
static __inline__ void irq_setup(void)
{
	uint8_t i;

	/* Apply POR settings to SCSS int routing as SCSS regs are sticky */
	for (i = 0; i < QM_SCSS_INT_MASK_NUMREG; i++) {
		*((volatile uint32_t *)QM_SCSS_INT_BASE + i) =
		    QM_SCSS_INT_MASK_DEFAULT;
	}
}

/*
 * System update pin MUX settings
 */
static __inline__ void sys_update_pin_mux_setup(void)
{
#if (SYSTEM_UPDATE_VIA_SPI)
	/* SPI Slave*/
	qm_pmux_select(QM_PIN_ID_0, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_1, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_2, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_3, QM_PMUX_FN_2);

#elif(SYSTEM_UPDATE_VIA_UART0)
	/* UART 0*/
	qm_pmux_select(QM_PIN_ID_18, QM_PMUX_FN_0);
	qm_pmux_select(QM_PIN_ID_19, QM_PMUX_FN_0);

#elif(SYSTEM_UPDATE_VIA_UART1)
	/* UART 1*/
	qm_pmux_select(QM_PIN_ID_16, QM_PMUX_FN_2);
	qm_pmux_select(QM_PIN_ID_17, QM_PMUX_FN_2);
#endif
}

static __inline__ void sys_update_isr_setup(void)
{
}

static __inline__ void sys_update_setup(void)
{
	sys_update_pin_mux_setup();
	sys_update_isr_setup();

	/* Check system update mode sticky bit */
	/* if (SYS_UPDATE_MODE) {
		* Perform system update
	}
	*/
}

static __inline__ void boot_services_setup()
{
}

/*
 * C runtime initialization.
 * This will be called from rom_startup.s
 */
void rom_startup(void)
{
	void (*entry_point)(void) = ENTRY_POINT_ADDRESS;

	extern uint32_t __bss_start[];
	extern uint32_t __data_vma[];
	extern uint32_t __data_lma[];
	extern uint32_t __data_size[];
	extern uint32_t __bss_end[];

	/* Zero out bss */
	memset(__bss_start, 0x00, (uint32_t)__bss_end - (uint32_t)__bss_start);

	/* Copy initialised variables */
	memcpy(__data_vma, __data_lma, (size_t)__data_size);

	power_setup();
	clock_setup();

	/* Interrupt initialisation */
	irq_setup();
	idt_init();
	aon_handle_spurious_irq();
	apic_init();
	__asm__ __volatile__("sti");

#if (SYSTEM_UPDATE_ENABLE)
	sys_update_setup();
#endif

	boot_services_setup();

	/* Jump to application code */
	entry_point();
}
