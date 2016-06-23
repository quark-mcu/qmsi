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

#include "qm_soc_regs.h"
#include "qm_flash.h"

#include "dfu/qda/xmodem.h"
#include "dfu/qda/qda.h"
#include "qdm/qdm.h"
#include "qfu/qfu.h"

#define XMODEM_TIMEOUT_S (3)

#define BUFFER_SIZE (2048)
#define XMODEM_BLOCK_SIZE (128)

/*
 * NOTE: in future, we will probably move this struct to a configuration file
 * for the device management module
 */
/* clang-format off */
static const dfu_cfg_t my_dfu_cfg = {
	0xBEEF, /* vid */
	0x00DA, /* pid */
	0xFFDA, /* pid_dfu */
	0x0100, /* dev_bcd */
	0xFFFF, /* detach timeout */
	2048,   /* max_block_size */
	0x0101, /* DFU version */
	/*
	 * DFU bmAttributes (bitfield): 0x07 (bitWillDetach = 0,
	 * bitManifestationTollerant = 1, bitCanUpload = 1, bitCanDnload = 1)
	 */
	0x07,
	2,	       /* num_alt_settings */
	{&qdm_dfu_rh, &qfu_dfu_rh}, /* The array of DFU request handlers */
};
/* clang-format on */

/* The RX buffer */
static uint8_t dm_rx_buf[BUFFER_SIZE];

/*--------------------------------------------------------------------------*/
/*                       XMODEM-REQUIRED FUNCTIONS                          */
/*--------------------------------------------------------------------------*/
int xmodem_getc(uint8_t *ch)
{
	/* NOTE: stub */
	return -1;
}

void xmodem_putc(uint8_t *ch)
{
	/* NOTE: stub */
}

/*--------------------------------------------------------------------------*/
/*                           STATIC FUNCTIONS                               */
/*--------------------------------------------------------------------------*/
/**
 * Initialize the hardware required for the DM communication link.
 *
 * Initialize the UART and the RTC timer needed by XMODEM
 */
static void dm_com_hw_init()
{
	/* NOTE: stub */
}

/*--------------------------------------------------------------------------*/
/*                           GLOBAL FUNCTIONS                               */
/*--------------------------------------------------------------------------*/

void dm_hook_setup(void)
{
	/* NOTE: stub */
}

void __attribute__((__noreturn__)) dm_main(void)
{
	int rx_len;

	/* QDA stays inactive until qda_process_pkt() is called */
	/* NOTE: to do: update the following line when the new XMODEM is in */
	qda_init(NULL, &my_dfu_cfg);
	/* NOTE: to do: init flash */
	/* NOTE: init the HW required by XMODEM (i.e., UART and RTC timer) */
	dm_com_hw_init();
	while (1) {
		/*
		 * Receive a new packet using XMODEM.
		 * xmodem_receive() is blocking: the function returns
		 * when the XMODEM transfer is completed; data is put
		 * into rx_buf.
		 */
		/*
		 * NOTE: to do: change the following line with a call to
		 * xmodem_receive()
		 */
		rx_len = 0;
		qda_process_pkt(dm_rx_buf, rx_len);
	}
	/* we never reach this point, so no need to call xmodem_close() */
}
