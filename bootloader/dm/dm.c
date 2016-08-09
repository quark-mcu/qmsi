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
#include "qm_gpio.h"
#include "qm_flash.h"
#include "qm_init.h"
#include "qm_interrupt.h"
#include "qm_isr.h"
#include "qm_pinmux.h"

#include "bl_data.h"
#include "dm.h"
#include "dm_config.h"
#include "dfu/qda/qda.h"
#include "qdm/qdm.h"
#include "qfu/qfu.h"

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
	/*
	 * Number of alternate settings: number of partitions + QDM alternate
	 * setting (i.e., alt setting 0).
	 */
	(1 + BL_FLASH_PARTITIONS_NUM),
};
/* clang-format on */

/*--------------------------------------------------------------------------*/
/*                           GLOBAL FUNCTIONS                               */
/*--------------------------------------------------------------------------*/

void dm_main(void)
{
	qm_gpio_state_t state;

	/*
	 * qda_init() implicitly init the HW required by XMODEM (i.e., UART and
	 * PIC timer) and the DFU state machine
	 */
	qda_init(&my_dfu_cfg);
	do {
		/*
		 * The following function returns only when no data is received
		 * for 10 seconds.
		 */
		qda_receive_loop();
		qm_gpio_read_pin(DM_CONFIG_GPIO_PORT, DM_CONFIG_GPIO_PIN,
				 &state);
	} while (state == QM_GPIO_LOW);
	/*
	 * Cold reboot in order to restore the default system configuration
	 * (thus getting rid of all the changes done by the DM mode, like the
	 * UART configuration).
	 */
	qm_soc_reset(QM_COLD_RESET);
}
