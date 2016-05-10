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

* Intel® Quark™ Microcontroller SE Series.
* Intel® Quark™ Microcontroller D2000 Series.
* Intel® Quark™ Microcontroller D2000 Development Platform.

External Dependencies
*********************

* The IAMCU toolchain (i586-intel-elfiamcu) is required to build the source code.
* OpenOCD is required to flash applications and ROM files onto the SoC.
* GDB is optional, it is used as a supplement to OpenOCD for debugging.
* `Intel® System Studio for Microcontrollers <https://software.intel.com/en-us/intel-system-studio-microcontrollers>`_ is optional.

* The toolchain is provided from both within the ISSM package or `standalone tarballs <https://github.com/01org/qmsi/releases/tag/v1.0.1>`_.

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
Make sure you have downloaded the toolchain as described in `External Dependencies`_.

Environment
===========
You must first set the IAMCU_TOOLCHAIN_DIR environment variable.
Assuming the toolchain was unpacked into *$HOME/issm_2016.0.019/* and
that you would find *i586-intel-elfiamcu-gcc* at *$HOME/issm_2016.0.019/tools/compiler/bin*, the variable can be set with:

``export IAMCU_TOOLCHAIN_DIR=$HOME/issm_2016.0.019/tools/compiler/bin``

SoC Targets
===========

Both Quark D2000 and Quark SE are supported. You can select them by setting the ``SOC``
variable.

To build for D2000:

``make SOC=quark_d2000``

To build for Quark SE:

``make SOC=quark_se``

SoC Core
========

On Quark SE SoC, there are two separate cores: x86 (Intel Lakemont) and sensor (ARC).
You can select them by setting the ``TARGET`` variable.

To build for the Lakemont core:

``make SOC=quark_se TARGET=x86``

To build for the ARC core:

``make SOC=quark_se TARGET=sensor``

Build modes
===========

Debug and release builds are supported setting the ``BUILD`` variable.

To build in debug mode:

``make BUILD=debug``

To build in release mode:

``make BUILD=release``

Targets
=======

The top level Makefile contains two make targets: ``rom`` and ``libqmsi``. The output
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

Advanced build options
======================

Some operating systems may use their own interrupt system instead of the one
provided by QMSI. In order to properly integrate with those OSs, the ISRs
defined in QMSI drivers should be compiled as regular functions (e.g. no
interrupt-related prologue and epilogue, no end-of-interrupt handling). To
achieve that, you should set 'ISR=handled' when building libqmsi.

For instance, the following command builds libqmsi for Quark D2000 with no
interrupt handling support.

``make libqmsi SOC=quark_d2000 ISR=handled``

Flashing
========

For flashing the board OpenOCD must be used. You can optionally use gdb
as a frontend for OpenOCD as described below.

You must first flash a bootstrap rom before flashing an application.
Assuming the toolchain was unpacked into *$HOME/issm_2016.0.019/*, this can be
done with:

``$ cd $HOME/issm_2016.0.019/tools/debugger/openocd``

``$ ./bin/openocd -f scripts/board/quark_d2000_onboard.cfg``

``$ gdb``

``(gdb) target remote :3333``

``(gdb) monitor clk32M 5000``

``(gdb) monitor load_image $PATH_TO_QMSI/build/debug/quark_d2000/rom/quark_d2000_rom.bin 0x0``

``(gdb) monitor load_image $PATH_TO_QMSI/examples/hello_world/debug/quark_d2000/bin/hello_world.bin 0x00180000``

Serial Output
=============

You can check UART console output with picocom or screen:

``$ picocom -b 115200 --imap lfcrlf /dev/ttyUSB0``

or

``$ screen /dev/ttyUSB0 115200``


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
* Always-On GPIO.
* Analog Comparators.
* Analog-to-Digital Converter (ADC).
* Clock Control.
* Direct Memory Access (DMA).
* DMA support for peripherals:

    + UART master for Lakemont
    + SPI master for Lakemont
    + I2C master for Lakemont
* Flash library.
* Flash Protection Regions (FPR).
* General Purpose Input Output (GPIO).
* Inter-Integrated Circuit (I2C) master.
* Interrupt Controller Timer.
* Interrupt Controllers:

    + Quark SE Lakemont (APIC)
    + Quark SE ARC
    + Quark D2000 (MVIC)
* Quark SE Mailbox.
* Quark SE Sensor Subsystem (ARC):

    + Timer
    + GPIO
    + SPI
    + I2C
    + ADC
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

* Serial Peripheral Interface (SPI) slave.
* Inter-Integrated Circuit (I2C) slave.
* I2S
