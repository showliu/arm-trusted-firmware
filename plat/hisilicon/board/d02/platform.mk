#
# Copyright (c) 2013-2016, ARM Limited and Contributors. All rights reserved.
# Copyright (c) 2016, Huawei Technologies/Hisilicon. All rights reserved.
# Copyright (c) 2016, Linaro Limited. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Before building, set cross compiler, e.g.,:
# export CROSS_COMPILE=aarch64-linux-gnu-
#
# To build:
# make -j PLAT=d02 bl1 fip
#
# To clean:
# make PLAT=d02 clean
#
# Useful debug flags:
# make PLAT=d02 DEBUG=1 LOG_LEVEL=50 bl1 fip


PLAT_INCLUDES		:=	-Iinclude/plat/arm/common/		\
				-Iinclude/plat/arm/common/aarch64/	\
				-Iplat/hisilicon/board/d02/include

PLAT_BL_COMMON_SOURCES	:=	drivers/console/console.S		\
				drivers/hisilicon/uart/hi16xx_console.S	\
				drivers/io/io_fip.c			\
				drivers/io/io_memmap.c			\
				drivers/io/io_storage.c			\
				lib/xlat_tables/xlat_tables_common.c	\
				lib/xlat_tables/aarch64/xlat_tables.c	\
				plat/common/aarch64/plat_common.c	\
				plat/hisilicon/board/d02/d02_common.c	\
				plat/hisilicon/board/d02/d02_io_storage.c

BL1_SOURCES		+=	drivers/arm/cci/cci.c			\
				lib/cpus/aarch64/cortex_a57.S		\
				plat/common/aarch64/platform_up_stack.S \
				plat/hisilicon/board/d02/bl1_plat_helpers.S \
				plat/hisilicon/board/d02/bl1_plat_setup.c \
				plat/hisilicon/board/d02/plat_helpers.S

BL2_SOURCES		+=	drivers/arm/tzc/tzc400.c		\
				plat/common/aarch64/platform_up_stack.S	\
				plat/hisilicon/board/d02/bl2_plat_setup.c

BL31_SOURCES		+=	drivers/arm/gic/gic_v2.c		\
				lib/cpus/aarch64/cortex_a57.S		\
				plat/common/aarch64/platform_mp_stack.S	\
				plat/hisilicon/board/d02/bl31_plat_setup.c \
				plat/hisilicon/board/d02/d02_gic.c	\
				plat/hisilicon/board/d02/d02_sysconf.c	\
				plat/hisilicon/board/d02/plat_gic.c	\
				plat/hisilicon/board/d02/plat_helpers.S	\
				plat/hisilicon/board/d02/plat_pm.c	\
				plat/hisilicon/board/d02/plat_topology.c

# Enable workarounds for selected Cortex-A57 erratas
ERRATA_A57_806969		:= 1
ERRATA_A57_813420		:= 1

# TODO: Update plat_pm.c and disable the PSCI platform compatibility layer
ENABLE_PLAT_COMPAT		:= 1

ARM_DISABLE_TRUSTED_WDOG	:= 1

# Enable D02-specific modifications in common code -- to be reviewed
$(eval $(call add_define,D02_HACKS))
ifeq ($(DEBUG),1)
# VERBOSE(...) logs the file name and line number
$(eval $(call add_define,LOG_FILE_LINE))
endif

# There is no BL33 binary. Instead, ARM-TF returns to UEFI.
NEED_BL33 := no
PRELOADED_BL33_BASE=0
