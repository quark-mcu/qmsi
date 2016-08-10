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

#include <string.h>
#include "mvic.h"
#include "idt.h"
#include "qm_interrupt.h"
#include "qm_pinmux.h"
#include "boot.h"
#include "boot_clk.h"
#include "flash_layout.h"
#include "power_states.h"

#if (SYSTEM_UPDATE_ENABLE)
#include "dm.h"
#include "bl_data.h"
#include "qm_gpio.h"

/* Flash configuration defines, valid when we are running at 32MHz */
#define FLASH_US_COUNT (0x20)
#define FLASH_WAIT_STATES (0x01)

/** Flash configuration for writing to bl-data and QFU images. */
static const qm_flash_config_t cfg_wr = {
    .us_count = SYS_TICKS_PER_US_32MHZ / BIT(CLK_SYS_DIV_1),
    .wait_states = FLASH_WAIT_STATES,
    .write_disable = QM_FLASH_WRITE_ENABLE};
#endif

#if (DEBUG)
static QM_ISR_DECLARE(double_fault_isr)
{
	power_cpu_halt();
}
#endif

/*
 * Version number defines for the ROM
 */

/*
 * ROM Major version number
 */
#define QM_VER_ROM_MAJOR (1)

/*
 * ROM Minor version number
 */
#define QM_VER_ROM_MINOR (1)

/*
 * ROM Patch version number
 */
#define QM_VER_ROM_PATCH (0)

/* ROM version number combined */
static const unsigned int rom_version __attribute__((used))
__attribute__((section(".rom_version"))) =
    (QM_VER_ROM_MAJOR * 10000) + (QM_VER_ROM_MINOR * 100) + QM_VER_ROM_PATCH;

/* Lakemont application's entry point */
#define LMT_APP_ADDR (0x00180000)

/* Factory settings for Crystal Oscillator */
/* 7.45 pF load cap for Crystal */
#define OSC0_CFG1_OSC0_FADJ_XTAL_DEFAULT (0x4)
/* Crystal count value set to 5375 */
#define OSC0_CFG0_OSC0_XTAL_COUNT_VALUE_DEFAULT (0x2)

/*
 * System power settings
 */
static __inline__ void power_setup(void)
{
	/* Pin MUX slew rate settings */
	/* By default, all pins are 12 mA, should be fine for now. */
}

/*
 * System clock settings
 */
static __inline__ void clock_setup(void)
{
#if (HAS_HYB_XTAL)
	/* Apply factory settings for Crystal Oscillator stabilization
	 * These settings adjust the trimming value and the counter value
	 * for the Crystal Oscillator */
	QM_SCSS_CCU->osc0_cfg1 &= ~OSC0_CFG1_OSC0_FADJ_XTAL_MASK;
	QM_SCSS_CCU->osc0_cfg1 |=
	    (OSC0_CFG1_OSC0_FADJ_XTAL_DEFAULT << OSC0_CFG1_OSC0_FADJ_XTAL_OFFS);
	QM_SCSS_CCU->osc0_cfg0 &= ~OSC0_CFG0_OSC0_XTAL_COUNT_VALUE_MASK;
	QM_SCSS_CCU->osc0_cfg0 |= (OSC0_CFG0_OSC0_XTAL_COUNT_VALUE_DEFAULT
				   << OSC0_CFG0_OSC0_XTAL_COUNT_VALUE_OFFS);
#endif /* HAS_HYB_XTAL */

	/*
	 * Switch to each silicon oscillator to set up trim data
	 * This sets up the trim codes for the first boot.
	 * This consists of computing the trim code if not available
	 * in non volatile memory and write this results in flash.
	 *
	 * This step is only performed if the shadow region
	 * is not populated.
	 * We rely on the 32MHz trim code to be shadowed to
	 * consider the region populated.
	 *
	 * This can be modified if this policy does not match your
	 * specific requirements.
	 */
	if ((QM_FLASH_DATA_TRIM_CODE->osc_trim_32mhz &
	     QM_FLASH_TRIM_PRESENT_MASK) != QM_FLASH_TRIM_PRESENT) {
		boot_clk_trim_code_setup();
	}

	/* Switch to 32MHz silicon oscillator */
	boot_clk_hyb_set_mode(CLK_SYS_HYB_OSC_32MHZ, CLK_SYS_DIV_1);
	clk_trim_apply(QM_FLASH_DATA_TRIM_CODE->osc_trim_32mhz);
}

/*
 * SCSS interrupt routing initialization.
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

static __inline__ void boot_services_setup()
{
}

/*
 * C runtime initialization.
 * This will be called from rom_startup.s
 */
void rom_startup(void)
{
	void (*app_entry)(void) = (void *)LMT_APP_ADDR;

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

	boot_sense_jtag_probe();

	/* Interrupt initialisation */
	irq_setup();
	idt_init();
	boot_aon_handle_spurious_irq();
#if (DEBUG)
	qm_int_vector_request(QM_INT_VECTOR_DOUBLE_FAULT, double_fault_isr);
#endif
	mvic_init();
	__asm__ __volatile__("sti");
#if (SYSTEM_UPDATE_ENABLE)
	qm_flash_set_config(QM_FLASH_0, &cfg_wr);
	bl_data_sanitize();

	/* Get DM pin status */
	qm_gpio_state_t state;

	clk_periph_enable(CLK_PERIPH_REGISTER | CLK_PERIPH_CLK |
			  CLK_PERIPH_GPIO_REGISTER);
	qm_pmux_select(DM_CONFIG_GPIO_PIN, QM_PMUX_FN_0);
	qm_pmux_pullup_en(DM_CONFIG_GPIO_PIN, true);
	qm_pmux_input_en(DM_CONFIG_GPIO_PIN, true);
	/* No need to configure the GPIO, default configuration is okay. */
	qm_gpio_read_pin(DM_CONFIG_GPIO_PORT, DM_CONFIG_GPIO_PIN, &state);
	qm_pmux_pullup_en(DM_CONFIG_GPIO_PIN, false);
	/* Enter DM mode if DM sticky bit is set or DM_GPIO_PIN is low */
	if (DM_STICKY_BIT_CHK() || (state == QM_GPIO_LOW)) {
		DM_STICKY_BIT_CLR();
		/* run the device management code; dm_main() never returns */
		dm_main();
	}
#endif

	boot_services_setup();

	/* Execute application, provided that it has been programmed. */
	if (0xffffffff != *(uint32_t *)LMT_APP_ADDR) {
		app_entry();
	}

	power_cpu_halt();
}
