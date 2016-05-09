/**
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
#include <errno.h>

#include "qm_common.h"
#include "xmodem.h"
#include "xmodem_io.h"

/* Custom value, not transfered via XMODEM */
#define TMO (0x00)
#define ERR (0xFF)
#define ECC (0xFE)

/* XMODEM package IDs */
#define SOH (0x01)
#define EOT (0x04)
#define ACK (0x06)
#define NAK (0x15)
#define CAN (0x18)

#define FRAME_SIZE (128)
#define RETRANSMIT_CNT (8)

/* Activate debug messages by defining DEBUG_MSG */
/* #define DEBUG_MSG */

#ifdef DEBUG_MSG
#define printd(...) QM_PRINTF(__VA_ARGS__)
#else
#define printd(...)
#endif

/* Conditional return macro */
#define RETURN_IF_RET(ret)                                                     \
	{                                                                      \
		__typeof__(ret) __ret = (ret);                                 \
		if (__ret != 0)                                                \
			return __ret;                                          \
	}

uint8_t frame_buffer[FRAME_SIZE + 4];
uint8_t package_number;

/* NOTE: Missing CRC implementation. */
static uint16_t crc_xmodem(void)
{
	return 0x0000;
}

/**
 * Write XMODEM package to serial interface.
 *
 * @param[in] cmd XMODEM command
 *
 * @return Resulting status code.
 *
 * @retval 0 Sucessfull
 **/
static int xmodem_write(uint8_t cmd)
{
	/* make function pointer local to save .text */
	printd("W: 0x%x\n", cmd);
	xmodem_io_putc(&cmd);
	if (cmd != SOH) {
		return 0;
	}

	/* Continue if SOH with PKG numbers, data and crc */
	uint16_t crc = crc_xmodem();
	frame_buffer[FRAME_SIZE + 2] = (uint8_t)(crc >> 8);
	frame_buffer[FRAME_SIZE + 3] = (uint8_t)crc;
	frame_buffer[0] = package_number;
	frame_buffer[1] = 0xFF - package_number;
	/* Write data */
	uint32_t n = 0;
	for (n = 0; n < FRAME_SIZE + 4; n++) {
		xmodem_io_putc(frame_buffer + n);
	}
	return 0;
}

/**
 * Read XMODEM package from serial interface.
 *
 * @return Received command, timeout or error.
 *
 * @retval TMO Timeout
 * @retval ERR Read Error
 * @retval ECC XMODEM CRC Error
 * @retval <0xF0 Received command
 **/
static uint8_t xmodem_read(void)
{
	printd("R: ");
	uint8_t cmd;

	/* Read first byte (CMD) */
	if (xmodem_io_getc(&cmd) == -ETIME) {
		printd("----\n");
		return TMO;
	}
	printd("0x%x\n", cmd);

	/* NOTE: Not checking return code for -1. */

	switch (cmd) {
	case SOH:
		break;
	case 'C':
	case EOT:
	case ACK:
	case NAK:
		/* NOTE: CAN (Cancel) is not implemented. */
		printd("0x%x\n", cmd);
		return cmd;
	default:
		printd("0x%x (Unknown)\n", cmd);
		return ERR;
	}

	/* Read package number, data and CRC. */
	uint8_t n = 0;
	for (n = 0; n < FRAME_SIZE + 4; n++) {
		if (xmodem_io_getc(frame_buffer + n) == -ETIME) {
			printd("----\n");
			return TMO;
		}
	}

	/* Check package numbers. */
	uint8_t pkg = ~frame_buffer[1];
	if (frame_buffer[0] != pkg) {
		printd("ERROR: PKG NR (%x != ~%x)\n", frame_buffer[0],
		       frame_buffer[1]);
		return ERR;
	}
	/* Check CRC */
	uint16_t crc_r = *(uint16_t *)(frame_buffer + FRAME_SIZE + 2);
	crc_r = ((crc_r >> 8) + (crc_r << 8));
	uint16_t crc_c = crc_xmodem();

	if (crc_c != crc_r) {
		printd("ERROR: CRC (0x%x != 0x%x)\n", crc_r, crc_c);
		return ECC;
	}
	printd("SOH\n");
	return SOH;
}

/**
 * Try to write XMODEM message for x-times.
 *
 * This function calles xmodem_write and checks if an ACK was received. If no
 * ACK was received, the write function is called again. This will be done until
 * 'RETRANSMIT_CNT' is exieded.
 *
 * @param[in] cmd XMODEM command
 *
 * @return Exit status.
 * @retval 0 Successful
 * @retval -1 Error, retransmit count exceeded.
 **/
static int xmodem_retransmit(const uint8_t cmd)
{
	uint8_t retransmit = RETRANSMIT_CNT;
	while (retransmit--) {
		xmodem_write(cmd);
		if (xmodem_read() == ACK)
			return 0;
	}
	return -1;
}

int xmodem_transmit_package(uint8_t *package, size_t len)
{
	package_number = 1;
	uint8_t retransmit = RETRANSMIT_CNT;
	while (retransmit--) {
		if (xmodem_read() == 'C') {
			goto start_transmit;
		}
	}
	return -1;

start_transmit:
	/* Send frames as long data */
	while (len) {
		memcpy(frame_buffer + 2,
		       package + ((package_number - 1) * FRAME_SIZE),
		       (len >= FRAME_SIZE) ? FRAME_SIZE : len);
		len -= (len >= FRAME_SIZE) ? FRAME_SIZE : len;
		RETURN_IF_RET(xmodem_retransmit(SOH));
		package_number++;
	}
	RETURN_IF_RET(xmodem_retransmit(EOT));
	return (package_number - 1) * FRAME_SIZE;
}

int xmodem_receive_package(uint8_t *package, size_t len)
{
	package_number = 1;
	uint32_t wait = 1;
	uint8_t retransmit = RETRANSMIT_CNT;
	while (1) {
		if (wait) {
			xmodem_write('C');
		}
		uint8_t cmd = xmodem_read();
		printd("CMD: %d\n", cmd);
		switch (cmd) {
		case SOH:
			wait = 0;
			retransmit = RETRANSMIT_CNT;
			if (package_number * FRAME_SIZE <= len) {
				memcpy(package +
					   ((package_number - 1) * FRAME_SIZE),
				       frame_buffer + 2, FRAME_SIZE);
				package_number++;
				xmodem_write(ACK);
			} else {
				/* No space left in buffer */
				xmodem_write(CAN);
				return -1;
			}
			break;
		case EOT:
			xmodem_write(ACK);
			return package_number * FRAME_SIZE;
		case TMO:
			/* Never time out */
			break;
		default:
			xmodem_write(NAK);
			if (!retransmit--) {
				return -1;
			}
		}
	}
	return 0;
}
