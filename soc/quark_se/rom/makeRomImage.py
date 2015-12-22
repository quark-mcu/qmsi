#!/usr/bin/env python
# @file This file merges the reset vector and the ROM code into an 8 KB image

#
# Copyright (c) 2015, Intel Corporation
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
#

import os
import sys
import struct

# Command line parameters
resetVectorFilename = sys.argv[1]
romCodeFilename = sys.argv[2]
outputFilename = sys.argv[3]

imageFile = open(outputFilename, 'wb+')
resetVectorFile = open(resetVectorFilename, 'rb').read()
romCodeFile = open(romCodeFilename, 'rb').read()
imageLen = 0x2000  # ROM size for Quark SE SOC is 8 KB (0x2000)
dataAreaLen = 0x400  # We reserve 1 KB for the OTP word and data storage
offset = 0

# Pad data area with 0xFF
for offset in range(dataAreaLen):
    imageFile.write(b'\xFF')
    offset += 1

# Write ROM code
imageFile.write(romCodeFile)
romCodeLen = len(romCodeFile)
offset += romCodeLen

# Write 0xFF padding (empty area)
resetVectorLen = len(resetVectorFile)
padLen = imageLen - offset - resetVectorLen
for romLenCount in range(padLen):
    imageFile.write(b'\xFF')

# Write reset vector
imageFile.write(resetVectorFile)

# Calculate relative jump offset value
romCodeStart = dataAreaLen  # ROM code starts immediately after data area
# Reset vector location + 5 bytes => 0F 09 E9 XX XX
origin = (imageLen - resetVectorLen) + 5
relJump = romCodeStart - origin
relJump16 = relJump & 0xFFFF  # 16-bit mask

# Patch reset vector jump address to ROM code
seekVal = resetVectorLen - 3  # To skip 0F 09 E9
imageFile.seek(-seekVal, os.SEEK_END)
imageFile.write(struct.pack('B', relJump16 & 0xFF))
imageFile.write(struct.pack('B', (relJump16 >> 8) & 0xFF))
imageFile.close()

print('. . . . . . . . . . . . . . . . . . . . . . . . . . . .')
print('Image size = ' + hex(imageLen))
print('Data area size = ' + hex(dataAreaLen))
print('ROM code size = ' + hex(romCodeLen))
print('Padding size = ' + hex(padLen))
print('Reset vector size = ' + hex(resetVectorLen))
print('ROM code start offset = ' + hex(romCodeStart))
print('Relative jump distance = ' + hex(relJump))
print('Relative jump 16-bit = ' + hex(relJump16))
print('. . . . . . . . . . . . . . . . . . . . . . . . . . . .')
