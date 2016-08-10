Intel® Quark™ Microcontroller Software Interface
################################################

Overview
********

Intel® Quark™ Microcontroller Software Interface (QMSI) is a Hardware
Abstraction Layer (HAL) for Intel® Quark™ Microcontroller products.
It currently support the following SoCs:

* Intel® Quark™ D2000 Microcontroller (D2000)
* Intel® Quark™ SE Microcontroller C1000 (SE C1000)

.. contents::

Support
*******

Information and support regarding Intel® Quark™ MCUs can be found in the
following website:

http://www.intel.com/quark/mcu

Hardware Compatibility
**********************

This release has been validated with the following hardware:

* Intel® Quark™ SE Microcontroller C1000.
* Intel® Quark™ SE Microcontroller C1000 Development Platform.
* Intel® Quark™ Microcontroller D2000.
* Intel® Quark™ Microcontroller D2000 Development Platform.

External Dependencies
*********************

* The ISSM toolchain is required to build the source code. It provides both the
  IAMCU and the ARCMCU toolchains (i586-intel-elfiamcu and arc-elf32, respectively).
  Currently supported version is "2016-05-12".
* OpenOCD is required to flash applications and ROM files onto the SoC.
* GDB is optional, it is used as a supplement to OpenOCD for debugging.
* `Intel® System Studio for Microcontrollers <https://software.intel.com/en-us/intel-system-studio-microcontrollers>`_ is optional.

* The toolchain is provided from both within the ISSM package or `standalone tarballs <https://software.intel.com/en-us/articles/issm-toolchain-only-download>`_.


More info about dependencies can be found in `dependencies <doc/dependencies.rst>`__ file.

License
*******

Please refer to `COPYING <COPYING>`_.

Organization
************
::

	.
	├── board           : Board level drivers
	├── bootloader      : QMSI Bootloader
	│   ├── boot        : Common Bootstrap code
	│   └── dm          : Device Management
	├── doc             : Project documentation and Guidelines
	│   └── api         : Doxygen documentation
	├── drivers         : Intel® SoC drivers
	│   └── include     : QMSI driver headers
	│   └── sensor      : SE C1000 Sensor Subsystem drivers
	├── examples        : Examples using the QMSI API
	├── include         : Top level headers
	├── soc             : Intel® MCUs SoCs support
	│   ├── quark_d2000 : Intel® Quark™ Microcontroller D2000 support
	│   └── quark_se    : Intel® Quark™ SE Microcontroller C1000 support
	└─ sys              : Application entry and Newlib syscalls


Building
********

The build system is based on the make tool.
Make sure you have downloaded the toolchain as described in `External Dependencies`_.

Environment
===========
You must first set the IAMCU_TOOLCHAIN_DIR environment variable.
Assuming the toolchain was unpacked into *$HOME/issm_2016/* and
that you would find *i586-intel-elfiamcu-gcc* at *$HOME/issm_2016/tools/compiler/gcc-ia/5.2.1/bin*, the variable can be set with:

``export IAMCU_TOOLCHAIN_DIR=$HOME/issm_2016/tools/compiler/gcc-ia/5.2.1/bin``

For SE C1000, if developing for the Sensor Subsystem (ARC), you must also set ARCMCU_TOOLCHAIN_DIR.
Assuming the ARC toolchain was unpacked into *$HOME/issm_2016/* and
that you would find *arc-elf32-gcc* at *$HOME/issm_2016/tools/compiler/gcc-arc/4.8.5/bin*, the variable can be set with:

``export ARCMCU_TOOLCHAIN_DIR=$HOME/issm_2016/tools/compiler/gcc-arc/4.8.5/bin``

SoC Targets
===========

Both D2000 and SE C1000 are supported. You can select them by setting the ``SOC``
variable.

To build for D2000:

``make SOC=quark_d2000``

To build for SE C1000:

``make SOC=quark_se``

SoC Core
========

On SE C1000, there are two separate cores: x86 (Intel® Lakemont) and sensor (ARC).
You can select them by setting the ``TARGET`` variable.

To build for the Lakemont core:

``make SOC=quark_se TARGET=x86``

To build for the ARC:

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

The top level Makefile contains two make targets: ``rom`` and ``libqmsi``.
The output directory is ``build``.

The ROM must be flashed on the OTP ROM flash region. To build the ``rom``
target, run:

``make rom``

When building the ROM, there are two possible build time flags available:
ENABLE_DM and START_ARC. ENABLE_DM is used to enable device management inside of
the rom, and START_ARC is used to start the ARC in the ROM.

By default, device management mode is not enabled.
To build the rom target with device management enabled, run:

``make rom ENABLE_DM=1``

On SE C1000, there is an option to not start the ARC as part of the ROM flow.
The default behavior for SE C1000 is to start the ARC as part of the ROM flow.
To build the rom target without starting the ARC, run:

``make rom START_ARC=0``

Libqmsi is a library archive of all the QMSI drivers for the SoC.

To build the ``libqmsi`` target, run the following command from the top level
directory:

``make libqmsi``

To build any of the provided example apps run make inside the corresponding
directory or use the –C make option from the top level directory.

E.g. to build the ``hello_world`` example app (by default it will be built in
release mode):

``make –C examples/hello_world``

Advanced build options
======================

Some operating systems may use their own interrupt system instead of the one
provided by QMSI. In order to properly integrate with those OSs, the ISRs
defined in QMSI drivers should be compiled as regular functions (e.g. no
interrupt-related prologue and epilogue, no end-of-interrupt handling). To
achieve that, you should set 'ISR=handled' when building libqmsi.

For instance, the following command builds libqmsi for D2000 with no
interrupt handling support.

``make libqmsi SOC=quark_d2000 ISR=handled``

Flashing
========

For flashing the board OpenOCD must be used. You can optionally use gdb
as a frontend for OpenOCD as described below.

You must first flash a bootstrap rom before flashing an application.
Assuming the toolchain was unpacked into *$HOME/issm_2016/*, this can be
done with:

``$ cd $HOME/issm_2016/tools/debugger/openocd``

For D2000 start OpenOCD with the following command:

``$ ./bin/openocd -f scripts/board/quark_d2000_onboard.cfg``

For SE C1000 start OpenOCD with the following command:

``$ ./bin/openocd -f scripts/board/quark_se_onboard.cfg``

Create a new terminal session at this point and set environment variables accordingly.
Then launch a GDB session using:

``$ gdb``

To connect to the repote port, enter the following GDB command:

``(gdb) target remote :333X``

For D2000 and SE C1000 (Lakemont), the remote port value is 3333.
For SE C1000 (ARC), the remote port value is 3334.

``(gdb) monitor clk32M 5000``

For D2000, the following commands are used to flash a ROM and application to the device:

``(gdb) monitor load_image $PATH_TO_QMSI/build/release/quark_d2000/rom/quark_d2000_rom.bin 0x0``

``(gdb) monitor load_image $PATH_TO_QMSI/examples/hello_world/release/quark_d2000/x86/bin/hello_world.bin 0x00180000``

For SE C1000, the following commands are used to flash a ROM and application to the device:

``(gdb) monitor load_image $PATH_TO_QMSI/build/release/quark_se/rom/quark_se_rom.bin 0xFFFFE000``

Applications for the Lakemont core are flashed using the following command:

``(gdb) monitor load_image $PATH_TO_QMSI/examples/hello_world/release/quark_se/sensor/bin/hello_world.bin 0x40000000``

Applications for the ARC are flashed using the following command:

``(gdb) monitor load_image $PATH_TO_QMSI/examples/hello_world/release/quark_se/x86/bin/hello_world.bin 0x40030000``

Serial Output
=============

You can check UART console output with picocom or screen:

``$ picocom -b 115200 --imap lfcrlf /dev/ttyUSBXXX``

or

``$ screen /dev/ttyUSBXXX 115200``

Where /dev/ttyUSBXXX is the path to the attached UART device.
e.g. /dev/ttyUSB0


Known Issues and Workarounds
****************************

Affected version: QMSI 1.1.0.

=========== ====================================================================
Issue       DMA errors are not generated for peripherals with invalid settings
----------- --------------------------------------------------------------------
Implication If an invalid address is provided for a peripheral in a DMA
            transfer, an error callback is not triggered.
----------- --------------------------------------------------------------------
Workaround  Use correct addresses for peripherals in DMA transfers.
=========== ====================================================================

=========== ====================================================================
Issue       SPI 16 MHz transfer failing on SE C1000 development platform
----------- --------------------------------------------------------------------
Implication On SE C1000, comparison of RX and TX is not correct when using the
            16 MHz speed.
----------- --------------------------------------------------------------------
Workaround  Use a transfer speed slower than 16 MHz.
=========== ====================================================================

=========== ====================================================================
Issue       I2C high speed mode fails on SE C1000 Development Platform
----------- --------------------------------------------------------------------
Implication On the SE C1000 development platform, Fab A/B, 330Ω resistor causes
            I2C transfers to fail in high-speed scenarios.
----------- --------------------------------------------------------------------
Workaround  Use the SE C1000 development platform Fab C, which has a 33Ω
            resistor.
=========== ====================================================================

=========== ====================================================================
Issue       UART - DMA transfers do not immediately report errors.
----------- --------------------------------------------------------------------
Implication Break interrupts or FIFO overruns may not be caught in a DMA UART
            transfer.
----------- --------------------------------------------------------------------
Workaround  If interrupts are required, use IRQ-based transfers instead.
=========== ====================================================================

=========== ====================================================================
Issue       If an application wakes up from power_soc_sleep() using the RTC on
            D2000, and completes, the system becomes bricked.
----------- --------------------------------------------------------------------
Implication The system is not fully restored from the soc_sleep function when
            using RTC as wake up source.
----------- --------------------------------------------------------------------
Workaround  The function power_soc_sleep() needs to be updated with the
            following:
	    Place the following line at the start of the function:
	    uint32_t lp_clk_save = QM_SCSS_CCU->ccu_lp_clk_ctl;
	    Place the following line at the end of the function(last line).
	    QM_SCSS_CCU->ccu_lp_clk_ctl = lp_clk_save;
=========== ====================================================================

=========== ====================================================================
Issue       D2000 hangs if the UART prints during soc_deep_sleep before the
            system has fully restored to the active state.
----------- --------------------------------------------------------------------
Implication If the user callback attempts to send data over the UART during a
            soc_deep_sleep callback when the system is still transitioning to
	    the active state, the system will hang on wake.
----------- --------------------------------------------------------------------
Workaround  Avoid printing over the UART during user callbacks until after the
            SoC has fully resumed operations in the active state.
=========== ====================================================================

=========== ====================================================================
Issue       Grove shield electricity sensor does not compile for x86 on SE
            C1000.
----------- --------------------------------------------------------------------
Implication Building the example application for x86 on the SE C1000 will result
            in a compilation error
----------- --------------------------------------------------------------------
Workaround  Compile the example for the SE C1000 ARC.
=========== ====================================================================

=========== ====================================================================
Issue       Power_soc sample application comment: "On the SE C1000 development
            platform this pin is found on header J13 PIN 20".
----------- --------------------------------------------------------------------
Implication Incorrect header number in comment
----------- --------------------------------------------------------------------
Workaround  Should be J14 not J13
=========== ====================================================================

=========== ====================================================================
Issue       GPIO sample app comments say: "On the SE C1000 development board,
            PIN_OUT and PIN_INTR are located on header P4 PIN 42 and 40"
----------- --------------------------------------------------------------------
Implication Incorrect header number in comment
----------- --------------------------------------------------------------------
Workaround  Should be J15 not P4
=========== ====================================================================

=========== ====================================================================
Issue       sensor/gpio sample app comments say: "On the SE C1000 development
            platform, PIN_OUT (J15 header, PIN 36) and PIN_INTR (J15 header,
	    PIN 42)."
----------- --------------------------------------------------------------------
Implication Incorrect pin number in comment
----------- --------------------------------------------------------------------
Workaround  Should be pin 40 not 42
=========== ====================================================================

=========== ====================================================================
Issue       sensor/interrupt sample App	comments say: "On the SE C1000
            development platform, PIN_OUT and PIN_INTR are located on header
	    J15, PIN 36 and 42 respectively"
----------- --------------------------------------------------------------------
Implication Incorrect pin number in comment
----------- --------------------------------------------------------------------
Workaround  Should be pin 40 not 42
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
* Firmware Update over UART (without authentication)
* General Purpose Input Output (GPIO).
* Inter-Integrated Circuit (I2C) master.
* Interrupt Controller Timer.
* Interrupt Controllers:

    + SE C1000 Lakemont (APIC)
    + SE C1000 ARC
    + D2000 (MVIC)
* SE C1000 Mailbox.
* SE C1000 Sensor Subsystem (ARC):

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
* Update utilities.
* Watchdog Timer (WDT).

Unsupported Features
====================

* Serial Peripheral Interface (SPI) slave.
* Inter-Integrated Circuit (I2C) slave.
* I2S
