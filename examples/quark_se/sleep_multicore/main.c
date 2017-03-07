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
 * Sleep multicore
 *
 * This example app is exclusive to the Intel(R) Quark(TM) SE SoC. This app
 * shows how to implement a multicore sleep synchronization procedure.
 *
 * This example must be compiled for x86 and sensor subsystem targets, and both
 * binary objects must be loaded on the target.
 * x86 default STDOUT is on UART1 and sensor subsystem default STDOUT is on
 * UART0.
 *
 * -- Case 1: Core 2 agrees on going to sleep
 * Core 1                        Core 2
 *  |                               |
 *  Request sleep ----------------> |
 *  |                               |
 *  | <---------------------------- Send ACK
 *  |                               Go to power sleep wait
 *  Wait for Core 2 to be           (sleep, do nothing)
 *  ready to go to SoC sleep
 *  |
 *  Set RTC
 *  Put SoC in sleep
 *  --- (SoC sleep, do nothing) ----
 *
 *
 *  -- Case 2: Core 2 disagrees on going to sleep
 * Core 1                        Core 2
 *  |                               |
 *  Request sleep ----------------> |
 *  |                               |
 *  | <---------------------------- Send NAK
 *  |                              (continue doing something)
 *  Go to core halt.
 *  (Do nothing until next
 *  interrupt)
 *
 * -- Case 3: Both cores request to go to sleep at same time
 * x86                             SS
 *  |                               |
 *  |<-----> Request sleep <------> |
 *  |                               |
 *  |                               Go to power sleep wait
 *  Wait for SS to be               (sleep, do nothing)
 *  ready to go to SoC sleep
 *  |
 *  Set RTC
 *  Put SoC in sleep
 *  --- (SoC sleep, do nothing) ----
 *
 * -- Case: After wake up
 * x86                             SS
 *  |                               -
 *  Restore context                 -
 *  |                               -
 *  Activate SS ------------------->|
 *  |                               Restore context
 *  (continue)                      (continue)
 *
 */

#include "clk.h"
#include "power_states.h"
#include "qm_interrupt_router.h"
#include "qm_interrupt.h"
#include "qm_isr.h"
#include "qm_mailbox.h"
#include "qm_rtc.h"
#include "qm_uart.h"
#include "ss_init.h"
#if QM_SENSOR
#include "ss_power_states.h"
#endif

#define RTC_PRESCALER (CLK_RTC_DIV_1)
#define RTC_ALARM_SECONDS(n) (QM_RTC_ALARM_SECOND(RTC_PRESCALER) * n)
#define RTC_SYNC_CLK_COUNT (5)
#define DELAY_1_SECOND (1000000)

#if !QM_SENSOR
/* Application specifics. */
#define MYNAME "x86"
#define TX_MBOX 0
#define RX_MBOX 1

/* Core/drivers specifics. */
#define OTHER_CORE_SLEEP_FLAG BIT(QM_GPS0_BIT_SENSOR_WAKEUP)
#define MBOX_DST QM_MBOX_TO_LMT
#define POWER_SOC_DEEP_SLEEP() qm_power_soc_deep_sleep_restore()
#define POWER_SLEEP_WAIT() qm_power_sleep_wait()
#define CORE_HALT() qm_power_cpu_c2()

#else /* !QM_SENSOR */
/* Application specifics. */
#define MYNAME "SS"
#define TX_MBOX 1
#define RX_MBOX 0

/* Core/drivers specifics. */
#define OTHER_CORE_SLEEP_FLAG BIT(QM_GPS0_BIT_X86_WAKEUP)
#define MBOX_DST QM_MBOX_TO_SS
#define POWER_SOC_DEEP_SLEEP() qm_ss_power_soc_deep_sleep_restore()
#define POWER_SLEEP_WAIT() qm_ss_power_sleep_wait()
#define CORE_HALT() qm_ss_power_cpu_ss1(QM_SS_POWER_CPU_SS1_TIMER_ON)
#endif /* !QM_SENSOR */

typedef enum {
	SLEEP_NONE = 0,
	SLEEP_REQ = BIT(0),
	SLEEP_ACK = BIT(1),
	SLEEP_NAK = BIT(2)
} sleep_msg_t;

typedef enum { CORE_SLEEP, SOC_SLEEP } device_sleep_t;

volatile bool is_ready = true;
volatile bool mbox_cb_fired = false;
qm_mbox_msg_t tx_data, rx_data;
qm_mbox_ch_t mbox_pair[2] = {QM_MBOX_CH_0, QM_MBOX_CH_1};
qm_mbox_config_t mbox_rx_config;
qm_mbox_ch_t mbox_tx = {0}, mbox_rx = {0};
qm_uart_context_t uart_ctx;
qm_rtc_context_t rtc_ctx;
qm_irq_context_t irq_ctx;

void mailbox_cb(void *data)
{
	mbox_cb_fired = true;
}

/*
 * Each core sets a flag to let know it has been in sleep state and can be
 * restored (set in sleep functions). It is used here to know if the opposite
 * core is ready to go to SoC sleep.
 */
bool other_core_sleep_flag(void)
{
	return (QM_SCSS_GP->gps0 & OTHER_CORE_SLEEP_FLAG) ? true : false;
}

void save_my_context(device_sleep_t dev)
{
	/*
	 * Each core saves its own UART.
	 * STDOUT_UART is defined by the Makefile and should be different for
	 * both cores.
	 */
	qm_uart_save_context(STDOUT_UART, &uart_ctx);

	/*
	 * RTC is only used for the master which triggers the sleep.
	 * Only restore the RTC for this core.
	 */
	if (dev == SOC_SLEEP) {
		qm_rtc_save_context(QM_RTC_0, &rtc_ctx);
	}

	/*
	 * Each core saves its own interrupt context. The function call is the
	 * same for both cores.
	 */
	qm_irq_save_context(&irq_ctx);
}

void restore_my_context(device_sleep_t dev)
{
	/*
	 * Each core restores its own interrupt context. The function call is
	 * the same for both cores.
	 */
	qm_irq_restore_context(&irq_ctx);

	/*
	 * RTC is only used for the master which triggers the sleep.
	 * Only restore the RTC for this core.
	 */
	if (dev == SOC_SLEEP) {
		qm_rtc_restore_context(QM_RTC_0, &rtc_ctx);
	}

	/*
	 * Each core restores its own UART.
	 * STDOUT_UART is defined by the Makefile and should be different for
	 * both cores.
	 */
	qm_uart_restore_context(STDOUT_UART, &uart_ctx);
}

void start_rtc()
{
	uint32_t aonc_start;
	qm_rtc_config_t cfg;

	/* Make sure RTC is enabled during sleep. */
	QM_SCSS_PMU->slp_cfg &= ~QM_SCSS_SLP_CFG_RTC_DIS;

	cfg.init_val = 0;
	cfg.alarm_en = true;
	cfg.alarm_val = RTC_ALARM_SECONDS(2);
	cfg.prescaler = RTC_PRESCALER;
	cfg.callback = NULL;
	cfg.callback_data = NULL;

	QM_IR_UNMASK_INT(QM_IRQ_RTC_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_RTC_0_INT, qm_rtc_0_isr);
	qm_rtc_set_config(QM_RTC_0, &cfg);

	/*
	 * The RTC clock resides in a different clock domain
	 * to the system clock.
	 * It takes 3-4 RTC ticks for a system clock write to propagate
	 * to the RTC domain.
	 * If an entry to sleep is initiated without waiting for the
	 * transaction to complete the SOC will not wake from sleep.
	 */
	aonc_start = QM_AONC[0]->aonc_cnt;
	while (QM_AONC[0]->aonc_cnt - aonc_start < RTC_SYNC_CLK_COUNT) {
	}
}

void clear_flags()
{
	/* Dummy read. */
	qm_mbox_ch_read(mbox_rx, &rx_data);
	rx_data.ctrl = SLEEP_NONE;
	mbox_cb_fired = false;
}

int send_msg(sleep_msg_t msg)
{
	tx_data.ctrl = msg;
	if (0 != qm_mbox_ch_write(mbox_tx, &tx_data)) {
		QM_PRINTF("Error: mbox %d write\n", mbox_tx);
		return 1;
	}

	return 0;
}

void soc_sleep(device_sleep_t dev)
{
	save_my_context(dev);

	if (dev == SOC_SLEEP) {
		start_rtc();
		QM_PUTS("SoC going to sleep");
		POWER_SOC_DEEP_SLEEP();
	} else {
		QM_PUTS("Core going to sleep");
		POWER_SLEEP_WAIT();
	}

	restore_my_context(dev);
	clear_flags();

#if !QM_SENSOR
	if (other_core_sleep_flag()) {
		sensor_activation();
	}
#endif
}

void request_soc_sleep(void)
{
	unsigned int key;
	qm_mbox_ch_status_t stat;

	QM_PUTS("Request SoC sleep");

	key = qm_irq_lock();

	/* Send sleep request. */
	send_msg(SLEEP_REQ);

	/* Wait for the answer from other core. */
	do {
		if (qm_mbox_ch_get_status(mbox_rx, &stat) != 0) {
			QM_PUTS("Unable to get mailbox status");
		}
	} while (stat == QM_MBOX_CH_IDLE);

	qm_mbox_ch_read(mbox_rx, &rx_data);

	switch (rx_data.ctrl) {
	case SLEEP_NAK:
		QM_PUTS("MBox NAK");
		qm_irq_unlock(key);

		CORE_HALT();
		break;

	case SLEEP_REQ:
		QM_PUTS("MBox REQ");
/*
 * There is a race condition where both cores want to go to
 * sleep. The policy is to make the x86 perform the SoC sleep
 * transition. Sensor will go to core sleep.
 */
#if QM_SENSOR
		soc_sleep(CORE_SLEEP);
		qm_irq_unlock(key);
#else
		/* Wait for other core to be ready. */
		while (!other_core_sleep_flag()) {
		}
		soc_sleep(SOC_SLEEP);
		qm_irq_unlock(key);
#endif
		break;

	case SLEEP_ACK:
		QM_PUTS("MBox ACK");

		/* Wait for other core to be actually ready for sleep mode. */
		while (!other_core_sleep_flag()) {
		}

		soc_sleep(SOC_SLEEP);
		qm_irq_unlock(key);
		break;

	default:
		QM_PUTS("Wrong message received");
		break;
	}
}

void soc_sleep_requested(bool ready_to_sleep)
{
	unsigned int key;

	QM_PUTS("SoC sleep requested");

	if (!ready_to_sleep) {
		QM_PUTS("Sleep NAK'd");
		send_msg(SLEEP_NAK);
	} else {
		key = qm_irq_lock();
		send_msg(SLEEP_ACK);
		soc_sleep(CORE_SLEEP);
		qm_irq_unlock(key);
	}

	return;
}

void init_mailbox(void)
{
	mbox_rx = mbox_pair[RX_MBOX];
	mbox_tx = mbox_pair[TX_MBOX];

	mbox_rx_config.dest = MBOX_DST;
	mbox_rx_config.mode = QM_MBOX_INTERRUPT_MODE;
	mbox_rx_config.callback = mailbox_cb;
	mbox_rx_config.callback_data = NULL;

	/* Register the interrupt handler. */
	QM_IR_UNMASK_INT(QM_IRQ_MAILBOX_0_INT);
	QM_IRQ_REQUEST(QM_IRQ_MAILBOX_0_INT, qm_mailbox_0_isr);

	/* Configure RX channel. */
	qm_mbox_ch_set_config(mbox_rx, &mbox_rx_config);
}

void state_machine(int cur_state, bool *sleep_ready, bool *do_soc_sleep)
{
	static int prev_state = -1;

/* Values for do_soc_sleep and sleep_ready for each states. */
#if !QM_SENSOR
	bool states[5][2] = {{true, false},  /* x86 requests sleep, SS NAK */
			     {false, false}, /* SS requests sleep, x86 NAK */
			     {true, false},  /* x86 requests sleep, SS ACK */
			     {false, true},  /* SS requests sleep, x86 ACK */
			     {true, true}};  /* Both request sleep */
#else
	bool states[5][2] = {{false, false}, /* x86 requests sleep, SS NAK */
			     {true, false},  /* SS requests sleep, x86 NAK */
			     {false, true},  /* x86 requests sleep, SS ACK */
			     {true, false},  /* SS requests sleep, x86 ACK */
			     {true, true}};  /* Both request sleep */
#endif

	if (prev_state == cur_state) {
		return;
	}

	QM_PRINTF("\nState number: %d\n", cur_state);
	prev_state = cur_state;

	if (cur_state < 5) {
		*do_soc_sleep = states[cur_state][0];
		*sleep_ready = states[cur_state][1];
	}
}

int main(void)
{
	bool do_soc_sleep, sleep_ready;
	int current_state = 0;
	QM_PRINTF("Starting: Sleep multicore %s\n", MYNAME);

#if !QM_SENSOR
	sensor_activation();
#endif
	init_mailbox();

	while (current_state < 5) {
		clk_sys_udelay(DELAY_1_SECOND);
		state_machine(current_state, &sleep_ready, &do_soc_sleep);

		/*
		 * In that order, we can force the race condition when both
		 * request sleep.
		 */
		if (do_soc_sleep) {
			request_soc_sleep();
			current_state++;
		} else if (mbox_cb_fired) {
			mbox_cb_fired = false;

			qm_mbox_ch_read(mbox_rx, &rx_data);
			if (rx_data.ctrl == SLEEP_REQ) {
				soc_sleep_requested(sleep_ready);
				current_state++;
			}
		}
	}

	QM_PRINTF("Finished: Sleep multicore %s\n", MYNAME);
	return 0;
}
