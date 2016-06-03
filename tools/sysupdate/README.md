# sysupdate #

## Scripts ##

Scripts are provided for creating DFU (Device Firmware Update) images
`qm_make_dfu.py` and writing images to a device `qm_update.py`. These scripts
can be used as they are or as a template to create costumed scripts.

### qm_make_dfu.py ###

To update a device with a new binary, the binary needs to be converted to a QM
(Quark Microcontroller) compatible dfu file. The script `qm_make_dfu.py` will
provide this functionality. Provide a binary file, a configuration file in
C-header format `-c`, and specify the target partition `-p`. The resulting file
is named after the input file with a `.dfu` extension or specified with `-o`.
If, once created, the file can be downloaded to a device using a standard
`dfu-util`, the `dfu-util-qda` or the `qm-update.py` script.

## qmdmlib ##

The `qmdmlib` library supports the host side features of Quark Microcontroller
sysupdate. It is used by `qm_make_dfu.py` and `qm_update.py`.

### Installation: ###

```
python setup.py install
```

### Usage: ###

Please read the module documentation located in the doc folder.

