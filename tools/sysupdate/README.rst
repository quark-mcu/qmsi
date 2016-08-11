sysupdate
#########

Scripts
*******

Scripts are provided for creating DFU (Device Firmware Update) images
`qm_make_dfu.py` and managing a device `qm_manage.py`. These scripts
can be used as they are or as a template to create customized scripts.

qm_make_dfu
===========

To upgrade a device with new firmware, the compiled binary needs to be converted
to a QM (Quark Microcontroller) compatible DFU file before downloading to the
device. The script `qm_make_dfu.py` provides this functionality. The script
takes as input a binary file and a target partition. You can also optionally
provide a configuration file specifying additional metadata (e.g., Vendor Id and
Product Id). The resulting file is named after the input file with a `.dfu`
extension or specified by argument.

qm_manage
=========

qm_manage can be used to retrieve device information and erase application data.
The argument `erase` or `info` and the serial port `-p` need to be provided.
This script is using `dfu-util-qda` binary to communicate with the device.

Command Line Interface
----------------------

The command line output for `qm_manage.py -h` ::

    usage: qm_manage.py [-h] [--version] {info,erase,list}

    Intel© Quark™ Microcontroller management tool.

    positional arguments:
      {info,erase}       run specific command

    optional arguments:
      -h, --help         show this help message and exit
      --version          show program's version number and exit
      -q, --quiet        suppress non-error messages
      -v, --verbose      increase verbosity
      -p SERIAL_PORT     specify the serial port to use

    possible commands:
      erase         erase all applications
      info          retrieve device information

Please use the `-h` option to display a help for a specific command. ::

    ./qm_manage.py <cmd> -h

A device can be connected by passing the `-p` option defining the path of the
serial port (`/dev/ttyUSBx` or `/dev/sttyxx`). This option can not be used with
the `list` command as serial ports can not be listed.

qmdmlib
*******

The `qmdmlib` library supports the host side features of Quark Microcontroller
sysupdate. It is used by `qm_make_dfu.py` and `qm_manage.py`.

Installation
************

setup.py can be used to install the python qmdmlib to your local python
packages. If you do not have root access add the *--user* option to install the
library in your user specific installation path.

Installation is not needed if the scripts are not moved to another path as
qmdmlib is located in the same folder.

.. code::

    python setup.py install

Compability
***********

All sysupdate related Python code is supported for Python 2.5+ and Linux
operating systems.
