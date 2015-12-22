#------------------------------------------------------------------------------
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
# Module Name:
#
#  rom_startup.S
#
# Abstract:
#
# This code handles the transition from real-mode to protected mode.
#
#------------------------------------------------------------------------------

.extern rom_startup

.extern __stack_start

#
# Contrary to the name, this file contains 16 bit code as well.
#
.text
#----------------------------------------------------------------------------
#
# Procedure:    _rom_start
#
# Input:        None
#
# Output:       None
#
# Destroys:     Assume all registers
#
# Description:
#
#   Transition to non-paged flat-model protected mode from a
#   hard-coded GDT that provides exactly two descriptors.
#   This is a bare bones transition to protected mode only
#   used for a while in PEI and possibly DXE.
#
#   After enabling protected mode, a far jump is executed to
#   transfer to PEI using the newly loaded GDT.
#
# Return:       None
#
#----------------------------------------------------------------------------
#
# These bytes allow the post processing script to find the address of the
# '_rom_start' procedure. The address is then put as a relative jump
# after the reset vector entry point.
#
.global _rom_start
_rom_start:

  #
  # Store the the BIST value in EBP (Built-in self-test)
  #
  .byte   0xbe,0x00,0xf0   #movw    $0xF000, %si
  .byte   0x8e,0xde        #movw    %si, %ds
  .byte   0xbe,0xf0,0xff   #movw    $0xFFF0, %si

  .byte   0x66,0x8b,0xe8   #movl    %eax, %ebp

  #
  # Load the GDT table in GdtDesc
  #
  .byte   0x66,0xbe        #movl    $GdtDesc, %esi
  .long   GdtDesc

  .byte   0x66,0x2e,0x0f,0x01,0x14   #lgdt    %cs:(%si)

  #
  # Transition to 16 bit protected mode
  #
  .byte   0x0f,0x20,0xc0       #movl    %cr0, %eax                  # Get control register 0
  .byte   0x66,0x83,0xc8,0x03  #orl     $0x0000003, %eax           # Set PE bit (bit #0) & MP bit (bit #1)
  .byte   0x0f,0x22,0xc0       #movl    %eax, %cr0                  # Activate protected mode

  #
  # Now we're in 16 bit protected mode
  # Set up the selectors for 32 bit protected mode entry
  #
  .byte   0xb8                 #movw    SYS_DATA_SEL, %ax
  .word   SYS_DATA_SEL

  .byte   0x8e,0xd8            #movw    %ax, %ds
  .byte   0x8e,0xc0            #movw    %ax, %es
  .byte   0x8e,0xe0            #movw    %ax, %fs
  .byte   0x8e,0xe8            #movw    %ax, %gs
  .byte   0x8e,0xd0            #movw    %ax, %ss

  #
  # Transition to Flat 32 bit protected mode
  # The jump to a far pointer causes the transition to 32 bit mode
  #
  .byte   0x66,0xbe            #movl   ProtectedModeEntryLinearAddress, %esi
  .long   ProtectedModeEntryLinearAddress
  .byte   0x66,0x2e,0xff,0x2c  #jmp    %cs:(%esi)

#
# Protected mode portion initializes stack, and calls C entry point
#

#----------------------------------------------------------------------------
#
# Procedure:    ProtectedModeEntryPoint
#
# Input:        Executing in 32 Bit Protected (flat) mode
#                               cs: 0-4GB
#                               ds: 0-4GB
#                               es: 0-4GB
#                               fs: 0-4GB
#                               gs: 0-4GB
#                               ss: 0-4GB
#
# Output:       This function never returns
#
# Destroys:
#               ecx
#               edi
#               esi
#               esp
#
# Description:
#               Perform any essential early platform initilaisation
#               Setup a stack
#
#----------------------------------------------------------------------------
ProtectedModeEntryPoint:

  #
  # Set up stack pointer see linker script for more info
  movl    $__stack_start, %esp     # ESP = top of stack (stack grows downwards).


  #
  # Begin the switch to the OS
  #
  call rom_startup

  #
  # Forever loop at end of last routine so should not return here.
  #
  jmp     .

#----------------------------------------------------------------------------
#
# Procedure:    BootGdtTable
#
# Input:        N/A
#
# Output:       N/A
#
# Destroys:     N/A
#
# Description:
#              Below are a set of constants used in setting up the GDT before
#              the switch to 32 bit mode 
#
#----------------------------------------------------------------------------
Function:
    .long   BootGdtTable

#
# ROM-based Global-Descriptor Table for the Tiano PEI Phase
#
.align 16
#
# GDT[0]: 000h: Null entry, never used.
#
.equ   NULL_SEL, . - GDT_BASE         # Selector [0]
GDT_BASE:
BootGdtTable:
        .long   0
        .long   0
#
# Linear code segment descriptor
#
.equ     LINEAR_CODE_SEL, . - GDT_BASE         # Selector [08h]
        .word   0xFFFF                      # limit 0FFFFh
        .word   0                           # base 0
        .byte   0
        .byte   0x9B                        # present, ring 0, data, expand-up, not-writable
        .byte   0xCF                        # page-granular, 32-bit
        .byte   0
#
# System data segment descriptor
#
.equ    SYS_DATA_SEL, . - GDT_BASE         # Selector [010h]
        .word   0xFFFF                      # limit 0FFFFh
        .word   0                           # base 0
        .byte   0
        .byte   0x93                        # present, ring 0, data, expand-up, not-writable
        .byte   0xCF                        # page-granular, 32-bit
        .byte   0

.equ            GDT_SIZE, . - BootGdtTable  # Size, in bytes

#
# GDT Descriptor

GdtDesc:                                     # GDT descriptor
       .word    GDT_SIZE - 1
       .long    BootGdtTable

ProtectedModeEntryLinearAddress:
ProtectedModeEntryLinearOffset:
       .long    ProtectedModeEntryPoint
       .word    LINEAR_CODE_SEL
