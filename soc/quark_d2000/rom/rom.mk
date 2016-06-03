#
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
#

### Variables
ROM_DIR = $(SOC_DIR)/rom
ROM_BUILD_DIR = $(BUILD_DIR)/$(BUILD)/$(SOC)/rom
OBJ_DIRS += $(ROM_DIR)/$(BUILD)
OBJ_DIRS += $(ROM_BUILD_DIR)
GENERATED_DIRS += $(ROM_DIR)/$(BUILD)
STARTUP = $(ROM_DIR)/$(BUILD)/$(OBJ)/rom_startup.bin
STARTUP_OBJS = $(ROM_DIR)/$(BUILD)/$(OBJ)/rom_startup.s.o \
              $(ROM_DIR)/$(BUILD)/$(OBJ)/rom_startup.o
BOOT_OBJS = $(BOOT_DIR)/clk/$(BUILD)/$(SOC)/$(OBJ)/boot_clk.o

ROM = $(ROM_BUILD_DIR)/quark_d2000_rom.bin
ROM_LINKER_FILE ?= $(ROM_DIR)/rom.ld

CFLAGS += -I$(BASE_DIR)/drivers
CFLAGS += -I$(BASE_DIR)/bootloader/clk

### For ROM we always manage our own ISRs.
CFLAGS += -UISR_HANDLED

ifeq ($(ENABLE_DM),1)
### Enable DM mode
include $(BASE_DIR)/bootloader/dm/dm.mk
STARTUP_OBJS += $(DM_OBJS)
# NOTE: '-Wno-unused-parameter' added to make DM module compile; to be removed
# once the root cause is fixed
CFLAGS += -Wno-unused-parameter -DSYSTEM_UPDATE_ENABLE=1
endif

.PHONY: rom

rom: $(ROM)

### Make 8kB ROM image
$(ROM): $(STARTUP)
	$(call mkdir, $(ROM_BUILD_DIR))
	python $(ROM_DIR)/makeRomImage.py $(STARTUP) $(ROM)

### Link STARTUP.elf and get raw binary
$(STARTUP): $(STARTUP_OBJS) $(BOOT_OBJS) libqmsi
	$(LD) $(LDFLAGS) -Xlinker -T$(ROM_LINKER_FILE) \
		-Xlinker -A$(OUTPUT_ARCH) \
		-Xlinker --oformat$(OUTPUT_FORMAT) \
		-Xlinker -Map=$(STARTUP).map \
		-o $(STARTUP).elf $(STARTUP_OBJS) $(BOOT_OBJS) $(LDLIBS)
	$(SIZE) $(STARTUP).elf
	$(OBJCOPY) -O binary $(STARTUP).elf $@

### Build C files
$(ROM_DIR)/$(BUILD)/$(OBJ)/%.o: $(ROM_DIR)/%.c $(BOOT_OBJS) libqmsi
	$(CC) $(CFLAGS) -c -o $@ $<

### Build assembly files
$(ROM_DIR)/$(BUILD)/$(OBJ)/%.s.o: $(ROM_DIR)/%.s
	$(call mkdir, $(ROM_DIR)/$(BUILD)/$(OBJ))
	$(CC) $(CFLAGS) -xassembler-with-cpp -c -o $@ $<
