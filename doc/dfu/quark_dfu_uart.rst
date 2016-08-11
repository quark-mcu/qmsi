Quark DFU over UART Guide
#########################

.. contents::

Hardware Setup
**************

The hardware setup consists of two main steps:

 * Connecting the board to a Linux host using UART.
 * Having the possibility to connect the DM GPIO pin to GND or to press the DM
   Button.

The DM GPIO pin is the pin used to put the device into DM mode. The device has
to be reset while the GPIO is connected to GND. On Quark Developer Boards a
button connects the DM GPIO to GND if pressed.

Software Setup
**************

For environment setup, please refer to `README.rst <../../README.rst>`__.

Enabling DM functionality in ROM
================================

In order to use the Device Management (DM) functionality over UART on the target
platform, you must install a ROM with such functionality enabled. To do that,
perform the following steps:

 * Compile the ROM code using the DM_ENABLE flag:
   ::

       make rom SOC=[SOC Name] ENABLE_DM=1

 * Flash the new ROM image to the target. (A guide can be found in the
   main `README.rst <../../README.rst>`__)

Compiling dfu-util-qda host tool
================================

dfu-util-qda is the tool needed to flash QFU images to the target or manage the
device.

 * Clone the qm-dfu-util repository
   ::

        git clone https://github.com/quark-mcu/qm-dfu-util.git

 * Ensure `dh-autoreconf` is installed.
 * Configure, build and install the tool
   ::

       cd qm-dfu-util
       ./autogen.sh
       ./configure
       sudo make install

DM functionality usage
**********************

Creating and flashing a QFU image
=================================

The following example assumes that you want to use the Bootloader DM
functionality to flash the LED-blink (blinky) example app to a Quark SE
development board.

Simple way (using 'make flash'):
 * Reset the device while connecting the DM GPIO to GND.
 * Compile, upload and run the example app:
   ::

       cd examples/blinky
       make flash SERIAL_PORT=[path_to_serial_interface] SOC=[SOC Name] TARGET=[Target Name]

Step by step process (without make flash):
 * Go to the example directory:
   ::

       cd examples/blinky

 * Build the project:
   ::

       make SOC=[SOC Name] TARGET=[Target Name]

 * Create the dfu file:
   ::

       python ../../tools/sysupdate/qm_make_dfu.py -v ./release/quark_se/x86/bin/blinky.bin -p 1

   - Note: The binary-path may differ if built for Quark D2000.
   - ARC binaries need to be downloaded to partition 2 (`-p 2`).


 * You should get the following output if you add -v as a parameter:
   ::

      qm_make_dfu.py: QFU-Header and DFU-Suffix content:
            Partition:   1
            Vendor ID:   0
            Product ID:  0
            Version:     0
            Block Size:  2048
            Blocks:      2
            DFU CRC:     0x...
      qm_make_dfu.py: blinky.dfu written

   blinky.dfu is your generated QFU image.
 * Reset the device while connecting the DM GPIO to GND.
 * Download the image:
   ::

       dfu-util-qda -D ./release/quark_se/x86/bin/blinky.bin.dfu -p [path_to_serial_interface] -R -a 1

   - Note: The binary-path may differ if built for Quark D2000.
   - Note: ARC binaries need to be downloaded to alternate setting 2 (`-a 2`).

   The -R parameter will reset the device after the download is finished.
   However, the DM mode is entered again if the DM GPIO is still connected to
   GND.

Application Erase / System Information Retrieval
================================================

System information can be retrieved by a Python script located in the
tools/sysupdate directory. This script uses the dfu-util-qda binary
to communicate with the device.

 * Make sure qfu-util-qda is installed.
 * Go to the tools/sysupdate directory.
 * Run the python script `qm_manage.py --help` to display possible commands.

Erase Applications:
 * Enter device DFU mode by resetting the device while the DM GPIO is connected
   to GND.
 * run
   ::

       qm_manage.py erase -p [path_to_serial_interface]

System Information:
 * Enter device DFU mode by resetting the device while the DM GPIO is connected
   to GND.
 * run
   ::

       qm_manage.py info -p [path_to_serial_interface]

.. note:: By specifying `--format` the output format can be set. (text or json)
