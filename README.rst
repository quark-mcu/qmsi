Intel® Quark™ Microcontroller Software Interface
################################################

Overview
********

Intel® Quark™ Microcontroller Software Interface (QMSI) is a Hardware
Abstraction Layer (HAL) for Intel® Quark™ Microcontroller products.
It currently support the following SoCs:

* Intel® Quark™ D2000 Microcontroller
* Intel® Quark™ SE Microcontroller

.. contents::

Support
*******

Information and support regarding Intel® Quark™ MCUs can be found in the
following website:

http://www.intel.com/quark/mcu

Hardware Compatibility
**********************

This release has been validated with the following hardware:

* Intel® Quark™ Microcontroller D2000 Series.
* Intel® Quark™ Microcontroller D2000 Development Platform.

External Dependencies
*********************

* The i586-intel-elfiamcu toolchain is required to build the source code.
* OpenOCD is required to flash applications and ROM files onto the SoC.
* GDB is optional, it is used as a supplement to OpenOCD for debugging.
* Intel® System Studio for Microcontrollers is optional.

License
*******

Please refer to `COPYING <COPYING>`_.

Organization
************
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
********

The build system is based on the make tool.

Targets
========

Debug and release builds are supported setting the ``BUILD`` variable.

To build in debug mode:

``make BUILD=debug``

To build in release mode:

``make BUILD=release``

Build modes
===========

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
****************************

=========== ====================================================================
Issue       MPR example app in release mode has spurious interrupt in
            Quark™ D2000
----------- --------------------------------------------------------------------
Implication D2000 board generates an extra interrupt when running the MPR example
            app in release mode.
----------- --------------------------------------------------------------------
Workaround  Run MPR example app in debug mode.
=========== ====================================================================

Change Log
**********

Supported features
==================

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

Unsupported Features
====================

* Direct Memory Access (DMA).
* Serial Peripheral Interface (SPI) slave.
* Inter-Integrated Circuit (I2C) slave.
* Continuous mode ADC conversions.
