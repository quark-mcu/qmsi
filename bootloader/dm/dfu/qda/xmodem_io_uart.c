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

#include <errno.h>

#include "qm_isr.h"
#include "qm_pinmux.h"
#include "qm_interrupt.h"
#include "qm_rtc.h"
#include "qm_uart.h"
#include "clk.h"

#include "xmodem_io.h"

/* NOTE: move this define to a bootloader configuration file */
#define XMODEM_UART (0)

#define XMODEM_UART_TIMEOUT_S (3)

#define XMODEM_UART_DIVISOR BOOTROM_UART_115200

/* NOTE: to do: move this macros to SoC-specific header files */
#if (QUARK_SE)
#if (XMODEM_UART == 0)
#define XMODEM_UART_PIN_TX_ID (QM_PIN_ID_18)
#define XMODEM_UART_PIN_TX_FN (QM_PMUX_FN_0)
#define XMODEM_UART_PIN_RX_ID (QM_PIN_ID_19)
#define XMODEM_UART_PIN_RX_FN (QM_PMUX_FN_0)
#define XMODEM_UART_CLK (CLK_PERIPH_UARTA_REGISTER)
/*
 * Using a function-looking macro since qm_irq_request() is actually a macro
 * stringifying its first argument
 */
#define xmodem_uart_enable_irq() qm_irq_request(QM_IRQ_UART_0, qm_uart_0_isr)
#elif(XMODEM_UART == 1)
#define XMODEM_UART_PIN_TX_ID (QM_PIN_ID_16)
#define XMODEM_UART_PIN_TX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_PIN_RX_ID (QM_PIN_ID_17)
#define XMODEM_UART_PIN_RX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_IRQ (QM_IRQ_UART_1)
#define XMODEM_UART_CLK (CLK_PERIPH_UARTB_REGISTER)
#define xmodem_uart_enable_irq() qm_irq_request(QM_IRQ_UART_1, qm_uart_1_isr)
#else
#error "Invalid UART ID for XMODEM"
#endif /* XMODEM_UART */
#elif(QUARK_D2000)
#if (XMODEM_UART == 0)
#define XMODEM_UART_PIN_TX_ID (QM_PIN_ID_12)
#define XMODEM_UART_PIN_TX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_PIN_RX_ID (QM_PIN_ID_13)
#define XMODEM_UART_PIN_RX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_CLK (CLK_PERIPH_UARTA_REGISTER)
#define xmodem_uart_enable_irq() qm_irq_request(QM_IRQ_UART_0, qm_uart_0_isr)
#elif(XMODEM_UART == 1)
#define XMODEM_UART_PIN_TX_ID (QM_PIN_ID_20)
#define XMODEM_UART_PIN_TX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_PIN_RX_ID (QM_PIN_ID_21)
#define XMODEM_UART_PIN_RX_FN (QM_PMUX_FN_2)
#define XMODEM_UART_IRQ (QM_IRQ_UART_1)
#define XMODEM_UART_CLK (CLK_PERIPH_UARTB_REGISTER)
#define xmodem_uart_enable_irq() qm_irq_request(QM_IRQ_UART_1, qm_uart_1_isr)
#else
#error "Invalid UART ID for XMODEM"
#endif /* XMODEM_UART */
#else
#error "SOC not supported"
#endif

/*-------------------------------------------------------------------------*/
/*                          FORWARD DECLARATIONS                           */
/*-------------------------------------------------------------------------*/
static void uart_callback(void *data, int error, qm_uart_status_t status,
			  uint32_t len);
static void rtc_callback(void *data);

/*-------------------------------------------------------------------------*/
/*                            GLOBAL VARIABLES                             */
/*-------------------------------------------------------------------------*/

/** The variable where UART isr stores the read byte (see uart_transfer var). */
static uint8_t in_byte;

/** The XMODEM UART configuration. */
static const qm_uart_config_t uart_config = {
    .baud_divisor = XMODEM_UART_DIVISOR,
    .line_control = QM_UART_LC_8N1,
    .hw_fc = false,
};

/** The XMODEM UART transfer configuration. */
static const qm_uart_transfer_t uart_transfer = {
    .data = &in_byte,
    .data_len = 1,
    .callback = uart_callback,
    .callback_data = NULL,
};

/** The XMODEM RTC configuration. */
static qm_rtc_config_t rtc_cfg = {
    .init_val = 0,
    .alarm_en = false,
    .alarm_val = (QM_RTC_ALARM_SECOND * XMODEM_UART_TIMEOUT_S),
    .callback = rtc_callback,
    .callback_data = NULL,
};

/** The XMODEM RX state enum. */
static volatile enum rx_state {
	STATE_UART_ERROR = -EIO,
	STATE_TIMEOUT = -ETIME,
	STATE_UART_RX_DONE = 1,
	STATE_WAITING = 2,
} xmodem_io_rx_state;

/*-------------------------------------------------------------------------*/
/*                             CALLBACKS                                   */
/*-------------------------------------------------------------------------*/

static void uart_callback(void *data, int error, qm_uart_status_t status,
			  uint32_t len)
{
	if (error < 0) {
		xmodem_io_rx_state = STATE_UART_ERROR;
	} else {
		xmodem_io_rx_state = STATE_UART_RX_DONE;
	}
}

static void rtc_callback(void *data)
{
	xmodem_io_rx_state = STATE_TIMEOUT;
}

/*-------------------------------------------------------------------------*/
/*                         XMODEM I/O FUNCTIONS                            */
/*-------------------------------------------------------------------------*/
/* Send one byte */
int xmodem_io_putc(const uint8_t *ch)
{
	return qm_uart_write(XMODEM_UART, *ch);
}

/* Receive one byte */
int xmodem_io_getc(uint8_t *ch)
{
	int status;
	enum rx_state retv;

	/* Set up timeout timer */
	rtc_cfg.alarm_en = true;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	/* Reseting the state and read byte */
	xmodem_io_rx_state = STATE_WAITING;
	status = qm_uart_irq_read(XMODEM_UART, &uart_transfer);
	if (status != 0) {
		return STATE_UART_ERROR;
	}
	while (retv = xmodem_io_rx_state, retv == STATE_WAITING) {
		; /* Busy wait until one of the callbacks is called */
	}
	switch (retv) {
	case STATE_TIMEOUT:
		qm_uart_irq_read_terminate(XMODEM_UART);
		break;
	case STATE_UART_RX_DONE:
		/* Got byte */
		*ch = in_byte;
		break;
	case STATE_UART_ERROR:
	case STATE_WAITING:
		/* Impossible case, included anyway for completeness */
		break;
	}
	/* Disable timer alarm */
	rtc_cfg.alarm_en = false;
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);

	return retv;
}

/*-------------------------------------------------------------------------*/
/*                           GLOBAL FUNCTIONS                              */
/*-------------------------------------------------------------------------*/
void xmodem_io_uart_init()
{
	/* Pinmux for UART_x */
	qm_pmux_select(XMODEM_UART_PIN_TX_ID, XMODEM_UART_PIN_TX_FN);
	qm_pmux_select(XMODEM_UART_PIN_RX_ID, XMODEM_UART_PIN_RX_FN);
	qm_pmux_input_en(XMODEM_UART_PIN_RX_ID, true);

	/* Enable UART and RTC clocks */
	clk_periph_enable(XMODEM_UART_CLK | CLK_PERIPH_RTC_REGISTER |
			  CLK_PERIPH_CLK);

	/* Setup UART */
	qm_uart_set_config(XMODEM_UART, &uart_config);
	xmodem_uart_enable_irq();

	/* Set up timeout timer (RTC) */
	qm_irq_request(QM_IRQ_RTC_0, qm_rtc_isr_0);
	qm_rtc_set_config(QM_RTC_0, &rtc_cfg);
}
