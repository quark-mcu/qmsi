#!/usr/bin/python -tt
# -*- coding: utf-8 -*-
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the Intel Corporation nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

""" qm-update: Script for Quark Microcontroller magagement.

Note: Need more verbose description.

::

   usage: qm_manage.py CMD [options]
"""

from __future__ import print_function, division, absolute_import
import os
import sys
import argparse
import subprocess
import tempfile

import qmdmlib

__version__ = "1.1"

desc = u"""Intel\u00A9 Quark\u2122 Microcontroller management tool."""


def _add_parser_defaults(parser):
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-q", "--quiet", action="store_true",
        help="suppress non-error messages")
    group.add_argument(
        "-v", "--verbose", action="count",
        help="increase verbosity")

    parser.add_argument(
        "-p", metavar="SERIAL_PORT", type=str, dest="port", required=True,
        help="specify the serial port to use")


def info():
    """ Perform 'info' tasks. """

    desc_add = "Retrieve device information."
    parser = argparse.ArgumentParser(description=desc+" "+desc_add)
    _add_parser_defaults(parser)
    parser.add_argument("--format", help="presentation format [default: text]",
                        choices=['text', 'json'])
    args = parser.parse_args(sys.argv[2:])

    print("Requesting system information...\t", end="")
    sys.stdout.flush()

    # Prepare sys info request.
    request = qmdmlib.QDMRequest(qmdmlib.QDMRequest.REQ_SYS_INFO).content
    image = qmdmlib.DFUImage()
    data = image.add_suffix(request)

    # Write temp output file. This file will be passed on to dfu-utils-qda.
    file_name = None
    try:
        file_handler = tempfile.NamedTemporaryFile("wb", delete=False)
        file_name = file_handler.name
        file_handler.write(data)
        file_handler.close()
    except IOError as error:
        parser.error(error)

    # Download request to device. Using alternate setting 0 (QDM).
    try:
        cmd = ["dfu-util-qda", "-a", "0", "-p", args.port, "-D", file_name]
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        print("[FAIL]")
        exit(1)
    finally:
        os.remove(file_name)
    print("[DONE]")

    print("Reading system information...\t\t", end="")
    sys.stdout.flush()
    try:
        # Create and delete a temporary file. This is done to check the file
        # permissions of the output file we give dfu-utils-qda to store the
        # result of our requested response.
        file_handler = tempfile.NamedTemporaryFile("wb", delete=True)
        file_name = file_handler.name
        file_handler.close()
    except IOError as error:
        parser.error(error)

    # Download request to device. Using alternate setting 0 (QDM).
    try:
        cmd = ["dfu-util-qda", "-a", "0", "-p", args.port, "-U", file_name]
        subprocess.check_output(cmd)
    except subprocess.CalledProcessError:
        print("[FAIL]")
        exit(1)
    print("[DONE]")

    in_file = open(file_name, "rb")
    response = qmdmlib.QDMResponse(in_file.read())
    in_file.close()

    if not response.cmd == qmdmlib.QDMResponse.RESP_SYS_INFO:
        print("Error: Invalid response.")
        exit(1)

    # Parse and Present Sys-Info data.
    info = qmdmlib.QDMSysInfo(response.content)

    if args.format == "json":
        print(info.info_json())
    else:
        print(info.info_string())


def erase():
    """ Perform 'erase' tasks. """

    desc_add = "Erase all applications on the device."
    parser = argparse.ArgumentParser(description=desc+" "+desc_add)
    _add_parser_defaults(parser)
    args = parser.parse_args(sys.argv[2:])
    print("Erasing all application data...\t\t", end="")
    sys.stdout.flush()
    request = qmdmlib.QDMRequest(qmdmlib.QDMRequest.REQ_APP_ERASE).content
    image = qmdmlib.DFUImage()
    data = image.add_suffix(request)

    # Write output file.
    file_name = None
    try:
        file_handler = tempfile.NamedTemporaryFile("wb", delete=False)
        file_name = file_handler.name
        file_handler.write(data)
        file_handler.close()
    except IOError as error:
        parser.error(error)

    try:
        cmd = ["dfu-util-qda", "-a", "0", "-p", args.port, "-D", file_name]
        subprocess.check_output(cmd, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        print("[FAIL]")
        exit(1)
    finally:
        os.remove(file_name)
    print("[DONE]")

if __name__ == "__main__":
    version = "qm_manage {version}".format(version=__version__)

    if "--version" in sys.argv:
        print (version)
        exit(0)

    choices_desc = "possible commands:\n" + \
                   "  erase         erase all applications\n" + \
                   "  info          retrieve device information"
    parser = argparse.ArgumentParser(
        description=desc,
        epilog=choices_desc,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    # Note: Version is not parsed by argparse if --version is not added.
    parser.add_argument('--version', action='version', version=version)
    parser.add_argument("cmd", help="run specific command",
                        choices=['info', 'erase'])
    args = parser.parse_args(sys.argv[1:2])

    if args.cmd == "info":
        info()
        exit(0)

    if args.cmd == "erase":
        erase()
        exit(0)
    exit(1)
