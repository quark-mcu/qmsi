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

### Tools
PREFIX ?= i586-intel-elfiamcu
ifeq ($(IAMCU_TOOLCHAIN_DIR),)
$(info Toolchain path is not defined. Please run:)
$(info export IAMCU_TOOLCHAIN_DIR=<TOOLCHAIN_PATH>)
$(error IAMCU_TOOLCHAIN_DIR is not defined)
endif

### OS specific
ifeq ($(OS),Windows_NT)
# Windows variants
export PATH := $(IAMCU_TOOLCHAIN_DIR);$(PATH)
OSNAME := $(shell wmic os get name)
ifneq (,$(findstring Microsoft Windows Server, $(OSNAME)))
# Windows Server
mkdir = @mkdir -p $(1) || exit 0
copy = @cp $(1) $(2) || exit 0
else
# Any other version of Windows
mkdir = @md $(subst /,\,$(1)) > nul 2>&1 || exit 0
copy = @copy $(subst /,\,$(1)) $(subst /,\,$(2)) > nul 2>&1 || exit 0
endif
else
# Unix variants
export PATH := $(IAMCU_TOOLCHAIN_DIR):$(PATH)
mkdir = @mkdir -p $(1)
copy = @cp $(1) $(2)
endif

SIZE = $(PREFIX)-size
OBJCOPY = $(PREFIX)-objcopy
AR = $(PREFIX)-ar
CC = $(PREFIX)-gcc
LD = $(CC)

### Environment checks
ifeq ($(BASE_DIR),)
$(error BASE_DIR is not defined)
endif

### Variables
BUILD ?= debug
ifeq ($(BUILD), debug)
CFLAGS += -O0 -g -DDEBUG
else ifeq ($(BUILD), release)
CFLAGS += -Os -ffunction-sections -fdata-sections
LDFLAGS += -Xlinker --gc-sections
else
$(error Supported BUILD values are 'release' and 'debug')
endif
$(info BUILD = $(BUILD))

BIN = bin
OBJ = obj

BUILD_DIR = $(BASE_DIR)/build
LIBQMSI_DIR = $(BUILD_DIR)/$(BUILD)/$(SOC)/libqmsi
LIBQMSI_LIB_DIR = $(LIBQMSI_DIR)/lib
LIBQMSI_INCLUDE_DIR = $(LIBQMSI_DIR)/include

### Flags
CFLAGS += -std=c90 -Wall -Wextra -Werror -Wno-unused-parameter
CFLAGS += -fmessage-length=0
CFLAGS += -I$(BASE_DIR)/include
CFLAGS += -fno-asynchronous-unwind-tables
LDFLAGS += -nostdlib
LDLIBS += -lc -lnosys -lsoftfp -lgcc

.PHONY: all clean

all: $(APP)

### Clean up
clean:
	$(RM) -r $(OBJ_DIRS) $(BUILD_DIR)
