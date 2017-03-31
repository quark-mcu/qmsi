#
# Copyright (c) 2017, Intel Corporation
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

### Make all specific examples

SUBDIRS := $(filter %/, $(wildcard ./*/))

### Exclude any apps that are only meant for x86
ifeq ($(TARGET), sensor)
SUBDIRS := $(subst ./start_arc/,,${SUBDIRS})
SUBDIRS := $(subst ./configure_lpss/,,${SUBDIRS})
SUBDIRS := $(subst ./power_core/,,${SUBDIRS})
SUBDIRS := $(subst ./power_lpss/,,${SUBDIRS})
SUBDIRS := $(subst ./usb_cdc_acm/,,${SUBDIRS})
SUBDIRS := $(subst ./usb_dfu/,,${SUBDIRS})
endif

ifeq ($(ENABLE_RESTORE_CONTEXT), 0)
SUBDIRS := $(subst ./sleep_multicore/,,${SUBDIRS})
endif

ifneq ($(SOC),quark_se)
$(error Supported SOC value is 'quark_se')
endif

all clean realclean:: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) SOC=$(SOC) TARGET=$(TARGET) BUILD=$(BUILD) -C $@ $(MAKECMDGOALS)

.PHONY: $(SUBDIRS)
