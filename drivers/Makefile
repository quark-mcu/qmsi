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

ifeq ($(SOC), quark_se)
else ifeq ($(SOC), quark_d2000)
else
$(error Supported SOC values are 'quark_se' and 'quark_d2000')
endif

### Variables
BASE_DIR = ..

HEADERS = $(wildcard $(BASE_DIR)/include/*.h)
HEADERS += $(wildcard $(BASE_DIR)/soc/$(SOC)/include/*.h)
HEADERS += $(wildcard $(BASE_DIR)/drivers/include/*.h)

EXPORTED_HEADERS += $(addprefix $(LIBQMSI_INCLUDE_DIR)/, $(notdir $(HEADERS)))

### Make includes
include $(BASE_DIR)/base.mk
include $(BASE_DIR)/drivers/drivers.mk
include $(BASE_DIR)/soc/$(SOC)/$(SOC).mk

$(LIBQMSI_LIB_DIR)/libqmsi.a: $(OBJECTS)
	$(call mkdir, $(LIBQMSI_LIB_DIR))
	$(AR) rcs $(LIBQMSI_LIB_DIR)/libqmsi.a $?

$(LIBQMSI_INCLUDE_DIR)/%.h: $(HEADERS)
	$(call mkdir, $(LIBQMSI_INCLUDE_DIR))
	$(call copy, $(filter %/$(notdir $@), $(HEADERS)), $@)

.PHONY: all

all: $(LIBQMSI_LIB_DIR)/libqmsi.a $(EXPORTED_HEADERS)
