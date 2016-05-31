/*
 * Copyright (c) 2016, ARM Limited and Contributors. All rights reserved.
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

#include <arch_helpers.h>
#include <assert.h>
#include <arm_gic.h>
#include <bl_common.h>
#include <debug.h>
#include <gic_v2.h>
#include <interrupt_mgmt.h>
#include <platform.h>
#include <stdint.h>
#include "d02_def.h"
#include "d02_private.h"


/* Value used to initialize Non-Secure IRQ priorities four at a time */
#define GICD_IPRIORITYR_DEF_VAL \
	(GIC_HIGHEST_NS_PRIORITY | \
	(GIC_HIGHEST_NS_PRIORITY << 8) | \
	(GIC_HIGHEST_NS_PRIORITY << 16) | \
	(GIC_HIGHEST_NS_PRIORITY << 24))

interrupt_type_handler_t handlers[128] = { NULL };

/*******************************************************************************
 * Enable secure interrupts and use FIQs to route them. Disable legacy bypass
 * and set the priority mask register to allow all interrupts to trickle in.
 ******************************************************************************/
void d02_gic_cpuif_setup(unsigned int gicc_base)
{
	unsigned int val;

	gicc_write_pmr(gicc_base, GIC_PRI_MASK);

	val = ENABLE_GRP0 | FIQ_EN;
	val |= FIQ_BYP_DIS_GRP0 | IRQ_BYP_DIS_GRP0;
	val |= FIQ_BYP_DIS_GRP1 | IRQ_BYP_DIS_GRP1;
	gicc_write_ctlr(gicc_base, val);
}

/*******************************************************************************
 * Place the cpu interface in a state where it can never make a cpu exit wfi as
 * as result of an asserted interrupt. This is critical for powering down a cpu
 ******************************************************************************/
void d02_gic_cpuif_deactivate(unsigned int gicc_base)
{
	unsigned int val;

	/* Disable secure, non-secure interrupts and disable their bypass */
	val = gicc_read_ctlr(gicc_base);
	val &= ~(ENABLE_GRP0 | ENABLE_GRP1);
	val |= FIQ_BYP_DIS_GRP1 | FIQ_BYP_DIS_GRP0;
	val |= IRQ_BYP_DIS_GRP0 | IRQ_BYP_DIS_GRP1;
	gicc_write_ctlr(gicc_base, val);
}

#if 0
static void gic_set_secure(unsigned int gicd_base, unsigned int id)
{
	/* Set interrupt as Group 0 */
	gicd_clr_igroupr(gicd_base, id);

	/* Set priority to max */
	gicd_set_ipriorityr(gicd_base, id, GIC_HIGHEST_SEC_PRIORITY);
}
#endif

#if 0
/*******************************************************************************
 * Per cpu gic distributor setup which will be done by all cpus after a cold
 * boot/hotplug. This marks out the secure interrupts & enables them.
 ******************************************************************************/
static void d02_gic_pcpu_distif_setup(unsigned int gicd_base)
{
	int i;

	static const char sec_irq[] = {
		IRQ_SEC_PHY_TIMER,
		IRQ_SEC_SGI_0,
		IRQ_SEC_SGI_1,
		IRQ_SEC_SGI_2,
		IRQ_SEC_SGI_3,
		IRQ_SEC_SGI_4,
		IRQ_SEC_SGI_5,
		IRQ_SEC_SGI_6,
		IRQ_SEC_SGI_7,
	};

	/* Mark all 32 PPI interrupts as Groupe 1 (non-secure) */
	mmio_write_32(gicd_base + GICD_IGROUPR, 0xFFFFFFFF);

	/* Setup PPI priorities doing four at a time */
	for (i = 0; i < 32; i += 4)
		mmio_write_32(gicd_base + GICD_IPRIORITYR + i,
			      GICD_IPRIORITYR_DEF_VAL);

	for (i = 0; i < ARRAY_SIZE(sec_irq); i++) {
		gic_set_secure(gicd_base, sec_irq[i]);
		gicd_set_isenabler(gicd_base, sec_irq[i]);
	}
}
#endif

/* Enable interrrupts for all CPUs on current die */
static void d02_ic_enable_all_cpus(unsigned int gicd_base)
{
	uint32_t die_id;

	die_id = mmio_read_32(gicd_base + GICD_DIEIDR);

	mmio_write_32(gicd_base + GICD_CPUEN_GRP0 + (die_id * 8), 0xFFFFFFFF);
	mmio_write_32(gicd_base + GICD_CPUEN_GRP1 + (die_id * 8), 0xFFFFFFFF);
}

void d02_ic_enable_one_cpu(unsigned int gicd_base, unsigned int core_id)
{
	uint32_t die_id;
	uint32_t val;
	uintptr_t addr;

	if (core_id >= PLATFORM_CORE_COUNT)
		return;

	die_id = mmio_read_32(gicd_base + GICD_DIEIDR);
	addr = gicd_base + (die_id * 8);

	val = mmio_read_32(addr + GICD_CPUEN_GRP0);
	val |= (1 << core_id);
	mmio_write_32(addr + GICD_CPUEN_GRP0, val);

	val = mmio_read_32(addr + GICD_CPUEN_GRP1);
	val |= (1 << core_id);
	val |= (1 << (core_id + PLATFORM_CORE_COUNT));
	mmio_write_32(addr + GICD_CPUEN_GRP1, val);
}

void d02_ic_disable_one_cpu(unsigned int gicd_base, unsigned int core_id)
{
	uint32_t die_id;
	uint32_t val;
	uintptr_t addr;

	if (core_id >= PLATFORM_CORE_COUNT)
		return;

	die_id = mmio_read_32(gicd_base + GICD_DIEIDR);
	addr = gicd_base + (die_id * 8);

	val = mmio_read_32(addr + GICD_CPUEN_GRP0);
	val &= ~(uint32_t)(1 << core_id);
	mmio_write_32(addr + GICD_CPUEN_GRP0, val);

	val = mmio_read_32(addr + GICD_CPUEN_GRP1);
	val &= ~(uint32_t)(1 << core_id);
	val &= ~(uint32_t)(1 << (core_id + PLATFORM_CORE_COUNT));
	mmio_write_32(addr + GICD_CPUEN_GRP1, val);
}

/*******************************************************************************
 * Global gic distributor setup which will be done by the primary cpu after a
 * cold boot. It marks out the non secure SPIs, PPIs & SGIs and enables them.
 * It then enables the secure GIC distributor interface.
 ******************************************************************************/
static void d02_gic_distif_setup(unsigned int gicd_base)
{
	unsigned int i, ctlr;
	unsigned int it_lines;

	it_lines = gicd_read_typer(gicd_base) & IT_LINES_NO_MASK;
	/*
	 * TODO: REVISIT
	 * "On P660 it_lines will be 0x1f, however we can only access 128
	 *  SPIs on one die."
	 */
	it_lines = (it_lines + 1) / 8 - 1;

	/* Disable the distributor */
	ctlr = gicd_read_ctlr(gicd_base);
	ctlr &= ~(ENABLE_GRP0 | ENABLE_GRP1);
	gicd_write_ctlr(gicd_base, ctlr);

	/* Mark all lines of SPIs as Group 1 (non-secure) */
	for (i = 0; i < it_lines; i++)
		mmio_write_32(gicd_base + GICD_IGROUPR + 4 + (i * 4),
			      0xFFFFFFFF);

	/* Setup SPI priorities douing four at a time */
	for (i = 0; i < 32; i += 4)
		mmio_write_32(gicd_base + GICD_IPRIORITYR + 32 + i,
			      GICD_IPRIORITYR_DEF_VAL);

	/* We don't need SGI and PPI right now on P660 */

	/* Enable interrrupts for all CPUs on current die */
	d02_ic_enable_all_cpus(gicd_base);

	/* Enable Group 0 (secure) interrupts */
	gicd_write_ctlr(gicd_base, ctlr | ENABLE_GRP0);
}

uint64_t fiq_handler(uint32_t id, uint32_t flags, void *handle, void *cookie)
{
	uint32_t it;
	uint32_t die_id;
	interrupt_type_handler_t handler;

	die_id = mmio_read_32(GICD_BASE + GICD_DIEIDR);
	it = mmio_read_32(GICC_BASE + GICC_IAR);
	it -= die_id * 128;

	if (it >= 128) {
		/* Special interrupt does not need to be acknowledged */
		goto exit;
	}

	/* TODO: useless because there is no code that registers handlers */
	handler = handlers[it];
	if (handler)
		handler(id, flags, handle, cookie);
	else
		ERROR("Interrupt handler is NULL\n");

exit:
	/*
	 * Note: GICC_IAR is set to 0x3ff after being read, so it can not be
	 * read again here. Instead, re-use the saved value.
	 */
	mmio_write_32(GICC_BASE + GICC_EOIR, it + die_id * 128);
	dsb();

	return 0;
}

void d02_gic_setup(void)
{
	uint32_t scr_el3;
	uint32_t gicc_ctlr_s;

	d02_gic_cpuif_setup(GICC_BASE);
	d02_gic_distif_setup(GICD_BASE);

	/*
	 * TODO: check this, reference code simply does:
	 * intr_type_descs[INTR_TYPE_EL3].handler = fiq_handler;
	 */
	register_interrupt_type_handler(INTR_TYPE_EL3, fiq_handler, 0);

	/* Physical FIQ while executing at any EL are taken at EL3 */
	scr_el3 = read_scr_el3();
	scr_el3 |= FIQ_EL3;
	write_scr_el3(scr_el3);

	/* */
	gicc_ctlr_s = mmio_read_32(GICC_BASE + GICC_CTLR);
	gicc_ctlr_s |= FIQ_EN;
	gicc_ctlr_s &= ~FIQ_BYP_DIS_GRP0; /* Disable bypass */
	mmio_write_32(GICC_BASE + GICC_CTLR, gicc_ctlr_s);
}

struct gic_register_para {
	uint32_t source;
	interrupt_type_handler_t handler;
	void (*init_fun)(void);
	uint32_t core_id_bit;
};

struct gic_register_para gic_register_para[] = {
	{ INTR_ID_UNAVAILABLE, NULL, NULL, 0 },
};

void d02_register_interrupt_svc()
{
	struct gic_register_para *rp;

	for (rp = gic_register_para; rp->source != INTR_ID_UNAVAILABLE; rp++)
		assert(!"Not implemented");
}
