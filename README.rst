================================================
Intel® Quark™ Microcontroller Software Interface
================================================

License
-------
Copyright © 2015, Intel Corporation.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the Intel Corporation nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Overview
--------
Intel® Quark™ Microcontroller Software Interface (QMSI) is a Hardware
Abstraction Layer (HAL) for Intel® Quark™ Microcontroller products.
It currently support the following SoCs:
* Intel® Quark™ D2000 Microcontroller


Support
-------
Information and support regarding Intel® Quark™ MCUs can be found in the
following website:

http://www.intel.com/quark/mcu


Hardware Compatibility
-----------------------------------
This release has been validated with the following hardware:
* Intel® Quark™ Microcontroller D2000 Series.
* Intel® Quark™ Microcontroller D2000 Development Platform.


External Dependencies
---------------------
* The i586-intel-elfiamcu toolchain is required to build the source code.
* OpenOCD is required to flash applications and ROM files onto the SoC.
* GDB is optional, it is used as a supplement to OpenOCD for debugging.
* Intel® System Studio for Microcontrollers is optional.


Organization
------------
::

	.
	├── board           : Board level drivers
	├── doc             : Doxygen documentation
	├── drivers         : Intel® SoC drivers
	│   └── include     : QMSI driver headers
	├── examples        : Examples using the QMSI API
	├── include         : Top level headers
	├── SoC             : Intel® SoCs support
	│   ├── quark_d2000 : Intel® Quark™ Microcontroller D2000 support
	│   └── quark_se    : Intel® Quark™ SE SoC early support
	└─ sys              : Application entry and Newlib syscalls


Building
--------
The build system is based on the make tool.

**Build modes**

Debug and release builds are supported setting the ``BUILD`` variable.

To build in debug mode:

``make BUILD=debug``

To build in release mode:

``make BUILD=release``


**Targets**

The top level Makefile contains two targets: ``rom`` and ``libqmsi``. The output
directory is ``build``. 

The ROM must be flashed on the OTP ROM flash region. To build the ``rom``
target, run:

``make rom``

Libqmsi is a library archive of all the QMSI drivers for the SoC.

To build the ``libqmsi`` target, run the following command from the top level
directory:

``make libqmsi``

To build any of the provided example apps run make inside the corresponding
directory or use the –C make option from the top level directory.

E.g. to build the ``hello_world`` example app (by default it will be built in
debug mode):

``make –C examples/hello_world``


Known Issues and Workarounds
----------------------------

=========== ====================================================================
Issue       SPI half-duplex transfers with the SoC as a receiver fails when the
            transfer length is longer than 8 bytes.
----------- --------------------------------------------------------------------
Implication SPI half-duplex RX mode is not working with transfer longer than 8
            bytes.
----------- --------------------------------------------------------------------
Workaround  Use full-duplex instead.
=========== ====================================================================

=========== ====================================================================
Issue       After power on reset or cold reset de-assertion, if AONPT_RST is set
            to 1 before the first RTC clock edge after reset de-assertion, then
            the AON Periodic Timer will not get started as expected.
----------- --------------------------------------------------------------------
Implication AON Periodic Timer might not start if configured early after reset.
----------- --------------------------------------------------------------------
Workaround  Check that the AON Counter value is greater than 0 before
            configuring the AON Periodic Timer. This ensures that the first RTC
            clock cycle has ticked.
=========== ====================================================================


Change Log
----------
**Supported features**

* Always-On (AON) Counters.
* Always-On (AON) Periodic Timer.
* Analog Comparators.
* Analog-to-Digital Converter (ADC).
* Clock Control.
* Flash library.
* Flash Protection Regions (FPR).
* General Purpose Input Output (GPIO).
* Inter-Integrated Circuit (I2C) master.
* Interrupt Controller Timer.
* Interrupt Controller.
* Memory Protection Regions (MPR).
* Pin Muxing.
* Power states.
* Pulse Width Modulation (PWM)/Timers.
* Real-Time Clock (RTC).
* Retention Alternating Regulator (RAR).
* Serial Peripheral Interface (SPI) master.
* System on Chip (SoC) Identification.
* Universal Asynchronous Receiver/Transmitter (UART).
* Watchdog Timer (WDT).

**Unsupported Features**

* Direct Memory Access (DMA).
* Serial Peripheral Interface (SPI) slave.
* Inter-Integrated Circuit (I2C) slave.
* Continuous mode ADC conversions.
