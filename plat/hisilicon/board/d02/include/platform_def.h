/*
 * Copyright (c) 2014-2016, ARM Limited and Contributors. All rights reserved.
 * Copyright (c) 2016, Huawei Technologies/Hisilicon. All rights reserved.
 * Copyright (c) 2016, Linaro Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLATFORM_DEF_H__
#define __PLATFORM_DEF_H__

#include <arch.h>
#include <common_def.h>
#include "../d02_def.h"

#define PLATFORM_STACK_SIZE	0x800

#define CACHE_WRITEBACK_SHIFT	6
#define CACHE_WRITEBACK_GRANULE	(1 << CACHE_WRITEBACK_SHIFT)

#define PLATFORM_CLUSTER_COUNT	4
#define PLATFORM_CORE_COUNT	16
#define PLATFORM_NUM_AFFS	(PLATFORM_CORE_COUNT + \
				PLATFORM_CLUSTER_COUNT)

#if 0
/* TODO: use this when plat_pm.c does not need compat anymore */

#define PLAT_MAX_PWR_LVL	1
#define PLAT_MAX_OFF_STATE	2
#define PLAT_MAX_RET_STATE	1

#else

#define PLATFORM_MAX_AFFLVL	MPIDR_AFFLVL1

#endif

#define MHU_SECURE_BASE		0x7FC00000
#define MHU_SECURE_SIZE		0x00001000
#define TZRAM_BASE		(MHU_SECURE_BASE + MHU_SECURE_SIZE)
#define TZRAM_SIZE		0x0003F000

#define FLASH_NS_BASE		0xA4800000
#define UEFI_SIZE		0x00200000
#define TZROM_BASE		(FLASH_NS_BASE + UEFI_SIZE)
#define TZROM_SIZE		0x00020000
#define FLASH_BASE		(FLASH_NS_BASE + UEFI_SIZE + TZROM_SIZE)
#define FLASH_SIZE		0x00800000

#define BL1_RO_BASE		TZROM_BASE
#define BL1_RO_LIMIT		(TZROM_BASE + TZROM_SIZE)

#define BL1_RW_BASE		(TZRAM_BASE + TZRAM_SIZE - 0xA000)
#define BL1_RW_LIMIT		(TZRAM_BASE + TZRAM_SIZE)

#define BL2_BASE		(BL31_BASE - 0x1D000)
#define BL2_LIMIT		BL31_BASE

#define BL31_BASE		(TZRAM_BASE + TZRAM_SIZE - 0x21000)
#define BL31_LIMIT		(TZRAM_BASE + TZRAM_SIZE)

#define BL32_BASE		0x50400000
#define BL32_LIMIT		0x51800000


/* Define to non-zero values to enable OP-TEE pager */
#define BL32_SRAM_BASE		0x0
#define BL32_SRAM_LIMIT		0x0

#define FIP_IMAGE_ID	0
#define BL2_IMAGE_ID	1
#define BL31_IMAGE_ID	2
#define BL32_IMAGE_ID	3
#define BL33_IMAGE_ID	4

#if TRUSTED_BOARD_BOOT
#error Not supported yet
#endif

#define MAX_XLAT_TABLES		8
#define MAX_MMAP_REGIONS	16
#define ADDR_SPACE_SIZE		(1ull << 43)

#define MAX_IO_DEVICES		3
#define MAX_IO_HANDLES		4

#endif /* __PLATFORM_DEF_H__ */
