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

#ifndef __D02_DEF_H__
#define __D02_DEF_H__

#define BASE_GICC_BASE	0xFE000000
#define BASE_GICD_BASE	0x8D000000

#define SRAM_BASE		0xE1000000
/*
 * UEFI is not invoked as BL33. Instead, ARM-TF is called by UEFI via SMC.
 * In order to be able to return to UEFI, we save the return address in SRAM.
 */
#define ELR_PEI_SAVE_ADDR		(SRAM_BASE + 0x0001F800)
/*
 * Hack!
 * BL1 saves TPIDR_EL0 on entry. It contains address of the PEI service table.
 * BL31 needs to restore it when returning to UEFI (EL2).
 * This address is in DRAM just below OP-TEE (BL32_BASE).
 */
#define TPIDR_EL0_SAVE_ADDR		0x4FFFFFF8

/* Cold/warm boot detection */
#define PRIMARY_CORE_CPUON_FLAG_ADDR	(SRAM_BASE + 0x0001F000)
#define MAGIC_NUMBER			0x5A5A5A5A

#define PRIMARY_CPU			0x0000

#define TRUSTED_MAILBOXES_BASE	0x7FC00000
#define TRUSTED_MAILBOXES_SIZE	0x00001000
#define TRUSTED_MAILBOX_SHIFT	4

#define PERI_SUB_CTRL_ADDR	0x80000000
#define ALG_BASE		0xD0000000
#define CPLD_BASE		0x98000000

/*
 * Reset registers
 */

#define SYS_CTRL_BASE		(PERI_SUB_CTRL_ADDR + 0x00010000)

#define SC_CLUS0_CRG_RESET_REQ	0x3068
#define SC_CLUS0_CRG_RESET_DREQ	0x306c
#define SC_CLUS0_CRG_RESET_ST	0xc018

#define SC_CLUS1_CRG_RESET_REQ	0x3078
#define SC_CLUS1_CRG_RESET_DREQ	0x307c
#define SC_CLUS1_CRG_RESET_ST	0xc020

#define SC_CLUS2_CRG_RESET_REQ	0x3088
#define SC_CLUS2_CRG_RESET_DREQ	0x308c
#define SC_CLUS2_CRG_RESET_ST	0xc028

#define SC_CLUS3_CRG_RESET_REQ	0x3098
#define SC_CLUS3_CRG_RESET_DREQ	0x309c
#define SC_CLUS3_CRG_RESET_ST	0xc030


#define RESET_CONTROL	0
#define DRESET_CONTROL	1
#define ALLRESET_BIT	0x1FE
#define ONERESET_BIT	0x22
#define L2_SRST_DREQ	0x1

/* TODO Revisit */
#define CLRBIT0		(0 << 0)
#define CLRBIT1		(0 << 1)
#define CLRBIT2		(0 << 2)
#define CLRBIT3		(0 << 3)

#define SETBIT0		(1 << 0)
#define SETBIT1		(1 << 1)
#define SETBIT2		(1 << 2)
#define SETBIT3		(1 << 3)

#define IDIE_ID		0
#define CDIEA_ID	1
#define CDIEC_ID	2
#define CDIEB_ID	3
#define DIE_ID_MAX	4


/*
 * Interrupt handling
 */

#define GICD_BASE	0x8D000000
#define GICC_BASE	0xFE000000

#define IRQ_SEC_PHY_TIMER	29
#define IRQ_SEC_SGI_0		8
#define IRQ_SEC_SGI_1		9
#define IRQ_SEC_SGI_2		10
#define IRQ_SEC_SGI_3		11
#define IRQ_SEC_SGI_4		12
#define IRQ_SEC_SGI_5		13
#define IRQ_SEC_SGI_6		14
#define IRQ_SEC_SGI_7		15

#define GICD_DIEIDR	0xC
#define GICD_CPUEN_GRP0	0x20A0
#define GICD_CPUEN_GRP1	0x20A4

#define FIQ_EL3	(1 << 2)

/*
 * Console UART
 */
#define D02_UART0_BASE		(PERI_SUB_CTRL_ADDR + 0x00300000)
#define D02_UART0_CLK_IN_HZ	200000000
#define D02_UART0_BAUDRATE	115200

#define SC_DJTAG_MSTR_START_EN		0x6804
#define SC_DJTAG_DEBUG_MODULE_SEL	0x680c
#define SC_DJTAG_MSTR_WR		0x6810
#define SC_DJTAG_MSTR_ADDR		0x6818
#define SC_DJTAG_MSTR_DATA		0x681C
#define SC_DJTAG_RD_DATA0		0xE800
#define CPLD_TC_CPU_BIST_REG		0xC126


#define SYS_APB_IF_BASE	0x10000
#define ADDRESS_MAP_INVALID	((uint64_t)(-1))

#define SYS_COUNTER_FREQ	50000000

#define DEVICE0_BASE	0x80000000
#define DEVICE0_SIZE	0x24800000

#define DEVICE1_BASE	0xA5000000
#define DEVICE1_SIZE	0x5B000000

#define DRAM_BASE	0x00000000
#define DRAM_SIZE	0x50000000 /* Contains TPIDR_EL0_SAVE_ADDR */

#define DRAM_NS_BASE	DRAM_BASE
#define DRAM_NS_SIZE	DRAM_SIZE

#endif /* __D02_DEF_H__ */
