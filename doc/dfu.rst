Bootloader: Device Management (a.k.a. Firmware Manager) Guide
#############################################################

This guide explains how to use the Device Management (DM) functionality
provided by the Quark bootloader.

The QDM (Quark Device Management) design and protocol is described in
the `README.rst <../bootloader/dm/README.rst>`_ located in `bootloader/dm/`.

Currently, DM functionality consists in the following services provided over
UART:

 * Application programming (a.k.a. Firmware Update)
 * Application erase
 * System information retrieval (bootloader version, application version, etc.)

Transport Specific Guidelines
*****************************

 * `Quark DFU UART Guide <dfu/quark_dfu_uart.rst>`__

Platform Specific Information
*****************************

+------------------+------------+---------------+-------------+
| Platform         | UART Port  | DM GPIO       | DM Button   |
+==================+============+===============+=============+
| Quark D2000 DP   | UART 0     | GPIO2 (J4.6)  | SW2         |
+------------------+------------+---------------+-------------+
| Quark SE DP      | UART 1     | AON4 (J14.43) | SW1         |
+------------------+------------+---------------+-------------+
