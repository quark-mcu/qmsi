In order to start debugging, follow those steps:
- Install virtualenv (if not already installed)
- Move to jflash directory and run the following commands:
  virtualenv --no-setuptools --no-pip --no-wheel -p <toolchain_path>/tools/python/bin/python2.7 .
  source bin/activate
  export PYTHONHOME=<toochain_path>/tools/python
- For Quark SE x86 run this command:
  python jflash.py -d quarkse_dev <file_path>
  For Quark SE ARC run this command:
  python jflash.py -d -s quarkse_dev <file_path>
  For Quark D2000 run this command:
  python jflash.py -d d2000_dev <file_path>
