/*
 * Copyright (c) 2015-2016, ARM Limited and Contributors. All rights reserved.
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
#include <debug.h>
#include <errno.h>
#include <mmio.h>
#include <platform.h>
#include <psci.h>
#include <stdint.h>
#include "d02_def.h"
#include "d02_private.h"

/* Program the mailbox for a CPU before it is released from reset */
static void d02_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	uint64_t mbox;

	linear_id = plat_core_pos_by_mpidr(mpidr);
	mbox = TRUSTED_MAILBOXES_BASE + (linear_id << TRUSTED_MAILBOX_SHIFT);
	*(uint64_t *)mbox = address;
	flush_dcache_range(mbox, sizeof(mbox));
}

/*******************************************************************************
* Private function which is used to determine if any platform actions
* should be performed for the specified affinity instance given its
* state. Nothing needs to be done if the 'state' is not off or if this is not
* the highest affinity level which will enter the 'state'.
*******************************************************************************/
static int32_t d02_do_plat_actions(uint32_t afflvl, uint32_t state)
{
	uint32_t max_phys_off_afflvl;

	assert(afflvl <= MPIDR_AFFLVL1);

	if (state != PSCI_STATE_OFF)
		return -EAGAIN;

	/*
	 * Find the highest affinity level which will be suspended and postpone
	 * all the platform specific actions until that level is hit.
	 */
	max_phys_off_afflvl = psci_get_max_phys_off_afflvl();
	assert(max_phys_off_afflvl != PSCI_INVALID_DATA);
	if (afflvl != max_phys_off_afflvl)
		return -EAGAIN;

	return 0;
}

int32_t d02_validate_power_state(unsigned int power_state)
{
	/* Only CPU cores (i.e., affinity level 0) can enter standby */
	if (psci_get_pstate_type(power_state) == PSTATE_TYPE_STANDBY)
		if (psci_get_pstate_afflvl(power_state) != MPIDR_AFFLVL0)
			return PSCI_E_INVALID_PARAMS;

	/* State ID is expected to be zero */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}

int d02_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/*
	 * On D02 with UEFI, NS is entered by returning to UEFI so there is
	 * nothing to validate here
	 */
	return PSCI_E_SUCCESS;
}

static inline uint32_t rst_reg_for_cluster(int num)
{
	/* 0x3068, 0x3078, 0x3088, 0x3098 */
	return SYS_CTRL_BASE + 0x3008 + ((6 + num) << 4);
}

static inline uint32_t drst_reg_for_cluster(int num)
{
	/* 0x306c, 0x307c, 0x308c, 0x309c */
	return rst_reg_for_cluster(num) + 4;
}

static void set_power_state(uint64_t mpidr, enum power_state cpu_state,
			    enum system_state cluster_state)
{
	uint32_t reg;
	unsigned int pos = platform_get_core_pos(mpidr);
	uint32_t cluster_num = pos / 4;
	uint32_t domain_cluster = 0;

	if (cpu_state != power_on)
		return;

	assert(cluster_num <= 4);

	/* */

	reg = rst_reg_for_cluster(cluster_num);
	mmio_write_32(reg, ONERESET_BIT << (pos % 4));

	d02_get_nm_config(mpidr, RESET_CONTROL, &domain_cluster);
	d02_djtag_write(0x10, 0x300 | domain_cluster, 0xB, CDIEC_ID);

	/* */

	reg = drst_reg_for_cluster(cluster_num);
	mmio_write_32(reg, (ONERESET_BIT << (pos % 4)) | L2_SRST_DREQ);
	dsb();
	isb();

	d02_get_nm_config(mpidr, DRESET_CONTROL, &domain_cluster);
	d02_djtag_write(0x10, 0x300 | domain_cluster, 0xB, CDIEC_ID);
	dsb();
	isb();
}

int32_t d02_affinst_on(uint64_t mpidr, uint64_t sec_entrypoint,
		       uint32_t afflvl, uint32_t state)
{
	/*
	 * TODO: check this (copied from Juno?): assumes the SCP takes care of
	 * powering down higher affinity levels
	 */
	if (afflvl != MPIDR_AFFLVL0)
		return PSCI_E_SUCCESS;

	if (platform_get_core_pos(mpidr))
		*(volatile uint32_t *)(PRIMARY_CORE_CPUON_FLAG_ADDR) = MAGIC_NUMBER;

	d02_program_mailbox(mpidr, sec_entrypoint);

	set_power_state(mpidr, power_on, power_on);

	return PSCI_E_SUCCESS;
}

void d02_affinst_on_finish(uint32_t afflvl, uint32_t state)
{
	uint64_t mpidr = read_mpidr_el1();
	unsigned int pos = platform_get_core_pos(mpidr);

	if (d02_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	if (afflvl != MPIDR_AFFLVL0) {
		/* TODO */
		/* cci_enable_cluster_coherency(mpidr); */
	}

	/* Enable the GIC CPU interface */
	d02_gic_cpuif_setup(GICC_BASE);

	/* */
	d02_ic_enable_one_cpu(GICD_BASE, pos);

	/* Warm boot of primary core through PSCI cpuon: clean flag */
	if (!pos)
		mmio_write_32(PRIMARY_CORE_CPUON_FLAG_ADDR, 0);

	/* Clear the mailbox for this CPU */
	d02_program_mailbox(mpidr, 0);
}

DEFINE_SYSREG_RW_FUNCS(S3_1_C15_C2_1)

static void d02_power_down_common(uint32_t afflvl, uint64_t mpidr)
{
	enum power_state cluster_state = power_on;
	uint32_t sctlr_el3;
	uint64_t cpuectlr_el1;

	/* Flush caches */
	d02_flush_l1l2l3_cache();

	/* Disable caching */
	sctlr_el3 = read_sctlr_el3();
	write_sctlr_el3(sctlr_el3 & ~SCTLR_C_BIT);

	isb();
	dsb();

	/* Adjust CPU Extended Control Register, EL1 */
	/* Disable ceche prefetch */
	cpuectlr_el1 = read_S3_1_C15_C2_1();
	cpuectlr_el1 |= (1UL << 38);
	cpuectlr_el1 &= ~(3UL << 35);
	cpuectlr_el1 &= ~(3UL << 32);
	write_S3_1_C15_C2_1(cpuectlr_el1);

#define SMPEN (1 << 6)
	/* Disable hardware data coherency with other cores in the cluster */
	cpuectlr_el1 = read_S3_1_C15_C2_1();
	cpuectlr_el1 &= ~SMPEN;
	write_S3_1_C15_C2_1(cpuectlr_el1);

	/* Disbale interrupts to this core */
	d02_gic_cpuif_deactivate(GICC_BASE);

	/* "Prevent out die interrupt" */
	d02_ic_disable_one_cpu(GICD_BASE, mpidr);

	/* TODO */
	/* disable_smp(); */

	isb();
	dsb();

	if (afflvl > MPIDR_AFFLVL0) {
		/* TODO */
		/* cci_disable_cluster_coherency(read_mpidr_el1()); */
		cluster_state = power_off;
	}

	/*
	 * Ask SCP to power down the appropriate cpmponents depending upon
	 * their state.
	 */
	set_power_state(read_mpidr_el1(), power_off, cluster_state);
}

static void d02_affinst_off(uint32_t afflvl, uint32_t state)
{
	if (d02_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	if (afflvl != MPIDR_AFFLVL0)
	{
		/* TODO */
	}

	d02_power_down_common(afflvl, read_mpidr_el1());
}

static void d02_affinst_suspend(uint64_t sec_entrypoint, uint32_t afflvl,
				uint32_t state)
{
	uint64_t mpidr = read_mpidr_el1();
	unsigned int pos = platform_get_core_pos(mpidr);

	if (d02_do_plat_actions(afflvl, state) == -EAGAIN)
		return;

	if (!pos)
		mmio_write_32(PRIMARY_CORE_CPUON_FLAG_ADDR, MAGIC_NUMBER);

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up
	 */
	d02_program_mailbox(mpidr, sec_entrypoint);

	d02_power_down_common(afflvl, mpidr);
}

static void d02_affinst_suspend_finish(uint32_t afflvl, uint32_t state)
{
	d02_affinst_on_finish(afflvl, state);
}

static void __dead2 d02_system_off(void)
{
	if (d02_set_sys_power_state(system_shutdown) < 0)
		panic();

	wfi();
	ERROR("System shutdown: operation not handled\n");
	panic();
}


static void __dead2 d02_system_reset(void)
{
	if (d02_set_sys_power_state(system_reboot) < 0)
		panic();

	wfi();
	ERROR("System reset: operation not handled\n");
	panic();
}

void d02_affinst_standby(unsigned int power_state)
{
	unsigned int scr;

	/* Enable PhysicalIRQ bit for NS world to wake the CPU */
	scr = read_scr_el3();
	write_scr_el3(scr | SCR_IRQ_BIT);

	isb();
	dsb();
	wfi();

	/*
	 * Restore SCR to the original value, synchronisation of scr_el3 is
	 * done by eret while el3_exit to save some execution cycles.
	 */
	write_scr_el3(scr);
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const struct plat_pm_ops d02_pm_ops = {
	.affinst_standby = d02_affinst_standby,
	.affinst_on = d02_affinst_on,
	.affinst_off = d02_affinst_off,
	.affinst_suspend = d02_affinst_suspend,
	.affinst_on_finish = d02_affinst_on_finish,
	.affinst_suspend_finish = d02_affinst_suspend_finish,
	.system_off = d02_system_off,
	.system_reset = d02_system_reset,
	.validate_power_state = d02_validate_power_state,
	.validate_ns_entrypoint = d02_validate_ns_entrypoint,
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &d02_pm_ops;
	return 0;
}
