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

#ifndef __USB_COMMON_H__
#define __USB_COMMON_H__

/**
 * USB protocol constants for the USB stack.
 *
 * @defgroup groupUSB USB
 * @{
 */

/* Decriptor size in bytes */
#define USB_DEVICE_DESC_SIZE (18)
#define USB_CONFIGURATION_DESC_SIZE (9)
#define USB_INTERFACE_DESC_SIZE (9)
#define USB_ENDPOINT_DESC_SIZE (7)
#define USB_STRING_DESC_SIZE (4)
#define USB_HID_DESC_SIZE (9)
#define USB_DFU_DESC_SIZE (9)

/* Descriptor type */
#define USB_DEVICE_DESC (0x01)
#define USB_CONFIGURATION_DESC (0x02)
#define USB_STRING_DESC (0x03)
#define USB_INTERFACE_DESC (0x04)
#define USB_ENDPOINT_DESC (0x05)
#define USB_HID_DESC (0x21)
#define USB_HID_REPORT_DESC (0x22)
#define USB_DFU_FUNCTIONAL_DESC (0x21)

/* Useful define */
#define USB_1_1 (0x0110)

/* TODO: make this configurable at build time. */
#define VENDOR_ID (0x8086)

#define BCDDEVICE_RELNUM (0x0100)
#define MAX_PACKET_SIZE_EP0 (0x0040)

/* 100mA max power, per 2mA units */
/* USB 1.1 spec indicates 100mA(max) per unit load, up to 5 loads */
#define USB_MAX_LOW_POWER (0x32)
#define USB_MAX_HIGH_POWER (0xFA)

/* bmAttributes:
 * D7:Reserved, always 1,
 * D6:Self-Powered -> 1,
 * D5:Remote Wakeup -> 0,
 * D4...0:Reserved -> 0
 */
#define USB_CONFIGURATION_ATTRIBUTES (0xC0)

/* Classes */
#define USB_COMMUNICATION_DEVICE_CLASS (0x02)
#define USB_COMMUNICATION_DEVICE_CLASS_DATA (0x0A)
#define USB_HID_CLASS (0x03)
#define USB_MASS_STORAGE_CLASS (0x08)
#define USB_CUSTOM_CLASS (0xFF)
#define USB_DFU_CLASS (0xFE)

/* Sub-classes */
#define USB_ACM_SUBCLASS (0x02)
#define USB_BOOT_INTERFACE_SUBCLASS (0x01)
#define USB_SCSI_TRANSPARENT_SUBCLASS (0x06)
#define USB_DFU_INTERFACE_SUBCLASS (0x01)

/* Protocols */
#define USB_V25TER_PROTOCOL (0x01)
#define USB_MOUSE_PROTOCOL (0x02)
#define USB_BULK_ONLY_PROTOCOL (0x50)
#define USB_DFU_RUNTIME_PROTOCOL (0x01)
#define USB_DFU_MODE_PROTOCOL (0x02)

#endif /* __USB_COMMON_H__ */
