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
 * Power Profiler
 *
 * This app demonstrates how SoCWatch profiles enter different power modes on
 * the Intel(R) Quark(TM) development platforms.
 */

#include "clk.h"
#include "power_states.h"
#include "qm_adc.h"
#include "qm_common.h"
#include "qm_comparator.h"
#include "qm_gpio.h"
#include "qm_interrupt.h"
#include "qm_interrupt_router.h"
#include "qm_isr.h"
#include "qm_pinmux.h"
#include "qm_pin_functions.h"
#include "qm_rtc.h"
#include "soc_watch.h"

/*
 * Header files and macro defines specific to LMT and ARC Sensor.
 */
#if (!QM_SENSOR)
#include <x86intrin.h>
/* 64bit Timestamp counter. */
#define get_ticks() _rdtsc()
#else
#include "qm_sensor_regs.h"
#include "ss_power_states.h"
/* Timestamp counter for sensor subsystem is 32bit. */
#define get_ticks() __builtin_arc_lr(QM_SS_TSC_BASE + QM_SS_TIMER_COUNT)
#endif

/* Configure the test features by defining these with non-zero values. */
#if (!QM_SENSOR)
/* Configure the test for x86. */
#if (QUARK_D2000)
#define SLOW_MODE_TEST (1)
#define HALT_TEST (1)
#define CORE_SLEEP_TEST (0)
#define SOC_SLEEP_TEST (1)
#define SOC_DEEP_SLEEP_TEST (0)
#elif(QUARK_SE)
#define SLOW_MODE_TEST (1)
#define HALT_TEST (1)
#define CORE_SLEEP_TEST (1)
#define SOC_SLEEP_TEST (0)
#define SOC_DEEP_SLEEP_TEST (0)
#endif // QUARK_D2000
#else
/* Configure the test for sensor. */
#define SLOW_MODE_TEST (0)
#define HALT_TEST (1)
#define CORE_SLEEP_TEST (1)
#define SOC_SLEEP_TEST (0)
#define SOC_DEEP_SLEEP_TEST (0)
#endif /* !QM_SENSOR */

/*
 * This is used for the deep sleep test. On the Intel(R) Quark(TM) D2000
 * Development Platform this pin is marked as "A5".
 */
#define BOARD_LED_PIN (24)

#if (!QM_SENSOR)
#if (QUARK_D2000)
#define ENTER_HALT() qm_power_cpu_halt()
#define ENTER_CORE_SLEEP_1()
#define ENTER_CORE_SLEEP_2()
#define ENTER_SOC_SLEEP() qm_power_soc_sleep()
#define ENTER_SOC_DEEP_SLEEP() qm_power_soc_deep_sleep(POWER_WAKE_FROM_RTC)
#define PIN_LED_ID (QM_PIN_ID_24)
#define PIN_MUX_FN (QM_PIN_24_FN_GPIO_24)
#elif(QUARK_SE)
#define ENTER_HALT() qm_power_cpu_c1()
#define ENTER_CORE_SLEEP_1() qm_power_cpu_c2()
#define ENTER_CORE_SLEEP_2() qm_power_cpu_c2lp()
#define ENTER_SOC_SLEEP() qm_power_soc_sleep()
#define ENTER_SOC_DEEP_SLEEP() qm_power_soc_deep_sleep()
#define PIN_LED_ID (QM_PIN_ID_58)
#define PIN_MUX_FN (QM_PIN_58_FN_GPIO_24)
#endif /* QUARK_D2000 */
#else
#define ENTER_HALT() qm_ss_power_cpu_ss1(QM_SS_POWER_CPU_SS1_TIMER_ON)
#define ENTER_CORE_SLEEP_1() qm_ss_power_cpu_ss2()
#define ENTER_CORE_SLEEP_2()
#define ENTER_SOC_SLEEP()
#define ENTER_SOC_DEEP_SLEEP()
#define PIN_LED_ID (QM_PIN_ID_58)
#define PIN_MUX_FN (QM_PIN_58_FN_GPIO_24)
#endif /* !QM_SENSOR */

/* QMSI GPIO Pin configuration struct.*/
qm_gpio_port_config_t gpio_cfg;

int rtc_tick = 0;

#if (QM_SENSOR)
static uint32_t rtc_trigger;

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
#endif

/* Invert an LED. */
static void led_flip(unsigned int pin)
{
	QM_GPIO[QM_GPIO_0]->gpio_swporta_dr ^= 1 << pin;
}

static void rtc_example_callback()
{
	/* Log the interrupt event. */
	SOC_WATCH_LOG_EVENT(SOCW_EVENT_INTERRUPT, QM_IRQ_RTC_0_INT_VECTOR);

	/* Invert On-board LED. */
	led_flip(BOARD_LED_PIN);
	++rtc_tick;

	/* Reschedule next tick. */
	qm_rtc_set_alarm(QM_RTC_0, (QM_RTC[QM_RTC_0]->rtc_ccvr +
				    (QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1) / 2)));
}

static void gpio_init()
{
	gpio_cfg.direction = 0; /* Configure all pins as inputs. */
	gpio_cfg.int_en = 0;    /* Interrupt disabled. */
	gpio_cfg.int_type = 0;  /* Turn off GPIO interrupts. */
	gpio_cfg.int_polarity = 0;
	gpio_cfg.int_debounce = 0;   /* Debounce disabled. */
	gpio_cfg.int_bothedge = 0x0; /* Both edge disabled. */
	gpio_cfg.callback = NULL;
	gpio_cfg.callback_data = NULL;

	qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);
}

/*
 * Configure a GPIO pin (or onboard LED pin) as an output and drive an
 * initial value.
 */
static void gpio_set_out(unsigned int pin, unsigned int initial_value)
{
	gpio_cfg.direction = BIT(pin); /* Configure pin for output. */
	qm_gpio_set_config(QM_GPIO_0, &gpio_cfg);

	if (initial_value) {
		qm_gpio_set_pin(QM_GPIO_0, pin);
	} else {
		qm_gpio_clear_pin(QM_GPIO_0, pin);
	}
}

#if (!QM_SENSOR)
/*
 * Returns the start and end RTC times for this busy loop.
 * Ideally, by examining the TSC and RTC times, we should be able to
 * identify their correlation.
 */
static uint64_t rtc_tsc_correlate(unsigned int *rtc_start,
				  unsigned int *rtc_end)
{
#if (!QM_SENSOR)
	uint64_t start_tsc;
	uint64_t end_tsc;
#else
	uint32_t start_tsc;
	uint32_t end_tsc;
#endif

retry:
	*rtc_start = QM_RTC[QM_RTC_0]->rtc_ccvr;
	start_tsc = get_ticks();
	clk_sys_udelay(400);
	*rtc_end = QM_RTC[QM_RTC_0]->rtc_ccvr;

	/* To prevent int_32 bit overflow while computing the rtc difference. */
	if ((*rtc_end < *rtc_start) &&
	    (!((*rtc_start & 0xF0000000) == 0xF0000000))) {
		goto retry;
	}
	end_tsc = get_ticks();
	return end_tsc - start_tsc;
}

static void test_clock_rates(void)
{
	uint64_t diff;
	unsigned int rtc_start, rtc_end = 0;

	/* Set clock to 32 MHz. */
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);
	diff = rtc_tsc_correlate(&rtc_start, &rtc_end);
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);
	/* Output is limited to 32 bits here. */
	QM_PRINTF("Fast Clk loop: %d TSC ticks; RTC diff=%d  : %d - %d\n",
		  (unsigned int)(diff & 0xffffffff), rtc_end - rtc_start,
		  rtc_end, rtc_start);

	/* Set clock to 4 MHz. */
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_8);
	diff = rtc_tsc_correlate(&rtc_start, &rtc_end);
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);
	/* Output is limited to 32 bits here. */
	QM_PRINTF("Slow Clk loop: %d TSC ticks; RTC diff=%d : %d - %d\n",
		  (unsigned int)(diff & 0xffffffff), rtc_end - rtc_start,
		  rtc_end, rtc_start);
}
#endif

static void slow_mode_test(void)
{
#if SLOW_MODE_TEST
	/* Drop into low-power compute mode. */
	QM_PUTS("\nSlow");
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_8);
	ENTER_HALT();
	clk_sys_set_mode(CLK_SYS_CRYSTAL_OSC, CLK_SYS_DIV_1);
#endif
}

static void halt_test(void)
{
#if HALT_TEST
	/* Halt the CPU, RTC alarm will wake me up. */
	QM_PUTS("Halt");
#if (QM_SENSOR)
	/* switch rtc interrupt to level triggered.*/
	rtc_trigger = switch_rtc_to_level();
#endif
	ENTER_HALT();
#if (QM_SENSOR)
	/* restore rtc interrupt back to edge triggered.*/
	restore_rtc_trigger(rtc_trigger);
#endif
#endif
}

static void core_sleep_test(void)
{
#if CORE_SLEEP_TEST
	/* Halt the CPU, RTC alarm will wake me up. */
	QM_PUTS("CPU C2");
#if (QM_SENSOR)
	/* switch rtc interrupt to level triggered.*/
	rtc_trigger = switch_rtc_to_level();
#endif
	ENTER_CORE_SLEEP_1();
#if (QM_SENSOR)
	/* restore rtc interrupt back to edge triggered.*/
	restore_rtc_trigger(rtc_trigger);
#endif
	QM_PUTS("CPU C2LP");
	ENTER_CORE_SLEEP_2();
#endif
}

static void soc_sleep_test(void)
{
#if SOC_SLEEP_TEST
	/* Go to sleep, (power down some IO). RTC will wake me up. */
	QM_PUTS("SOC Sleep");
	ENTER_SOC_SLEEP();
#endif
}

static void soc_deep_sleep_test(void)
{
#if SOC_DEEP_SLEEP_TEST
	/* Go to sleep, (power down some IO). RTC will wake me up. */
	QM_PUTS("SOC Deep Sleep");
	/* TODO : save/restore context soc_deep_sleep test. */
	ENTER_SOC_DEEP_SLEEP();
#endif
}

static void pin_mux_setup(void)
{
	qm_pmux_select(PIN_LED_ID, PIN_MUX_FN); /* Pin Muxing. */
}

int main(void)
{
	qm_rtc_config_t rtc_cfg;
	unsigned int count = 0;
	/* loop_max : Maximum number of iterations.
	 * A value of 50 will run the application for roughly 30s.
	 */
	const unsigned int loop_max = 50;
	gpio_init();
	pin_mux_setup();
	gpio_set_out(BOARD_LED_PIN, 0); /* Configure the onboard LED pin. */

	/* Clear Screen. */
	QM_PUTS("Starting: Power Profiler");
	QM_PUTS("Low power mode example.");
	QM_PRINTF("Increment = %d\n", QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1));

	clk_periph_enable(CLK_PERIPH_RTC_REGISTER | CLK_PERIPH_CLK);

	/* Initialise RTC configuration: Run, but don't interrupt. */
	rtc_cfg.init_val = 0;
	rtc_cfg.alarm_en = 0;
	rtc_cfg.alarm_val = QM_RTC_ALARM_SECOND(CLK_RTC_DIV_1);
	rtc_cfg.callback = rtc_example_callback;
	rtc_cfg.prescaler = CLK_RTC_DIV_1;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	QM_IR_UNMASK_INT(QM_IRQ_RTC_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_RTC_0_INT, qm_rtc_0_isr);

#if (!QM_SENSOR)
	test_clock_rates();
#endif

	/* Enable the RTC Interrupt. */
	rtc_cfg.alarm_en = 1;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	count = 0;
	while (++count < loop_max) {
		QM_PRINTF("\nC:%d R:%d => ", count, rtc_tick);

		slow_mode_test();
		halt_test();
		core_sleep_test();
		/* TODO : Enable soc_sleep test for c1000. */
		soc_sleep_test();
		/* TODO : Enable soc_deep_sleep test for d2000 and c1000. */
		soc_deep_sleep_test();
	}
	SOC_WATCH_TRIGGER_FLUSH();
	QM_PUTS("Finished: Power Profiler");
	return 0;
}
