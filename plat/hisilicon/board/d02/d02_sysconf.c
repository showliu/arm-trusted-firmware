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
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include "d02_def.h"
#include "d02_private.h"

static inline uint32_t cluster0_mn_bit(void)
{
	uint32_t val = mmio_read_32(SYS_CTRL_BASE + SC_CLUS0_CRG_RESET_ST);

	return (val == ALLRESET_BIT) ? CLRBIT0 : SETBIT0;
}

static inline uint32_t cluster1_mn_bit(void)
{
	uint32_t val = mmio_read_32(SYS_CTRL_BASE + SC_CLUS1_CRG_RESET_ST);

	return (val == ALLRESET_BIT) ? CLRBIT1 : SETBIT1;
}

static inline uint32_t cluster2_mn_bit(void)
{
	uint32_t val = mmio_read_32(SYS_CTRL_BASE + SC_CLUS2_CRG_RESET_ST);

	return (val == ALLRESET_BIT) ? CLRBIT2 : SETBIT2;
}

static inline uint32_t cluster3_mn_bit(void)
{
	uint32_t val = mmio_read_32(SYS_CTRL_BASE + SC_CLUS3_CRG_RESET_ST);

	return (val == ALLRESET_BIT) ? CLRBIT3 : SETBIT3;
}

/*
 * Helper function to get the domain_cluster value to be passed as the 'value'
 * parameter to d02_djtag_write()
 */
void d02_get_nm_config(uint64_t mpidr, uint32_t op, uint32_t *domain_cluster)
{
	uint32_t pos = platform_get_core_pos(mpidr) / 4;
	uint32_t rst_state_reg;
	uint32_t rst_state;
	int is_all_reset;
	int is_current_reset;

	if (op == RESET_CONTROL) {

		*domain_cluster = 0;

		switch (pos) {
		case 0:
			rst_state_reg = SYS_CTRL_BASE + SC_CLUS0_CRG_RESET_ST;
			*domain_cluster |= cluster1_mn_bit();
			*domain_cluster |= cluster2_mn_bit();
			*domain_cluster |= cluster3_mn_bit();
			break;
		case 1:
			rst_state_reg = SYS_CTRL_BASE + SC_CLUS0_CRG_RESET_ST;
			*domain_cluster |= cluster0_mn_bit();
			*domain_cluster |= cluster2_mn_bit();
			*domain_cluster |= cluster3_mn_bit();
			break;
		case 2:
			rst_state_reg = SYS_CTRL_BASE + SC_CLUS0_CRG_RESET_ST;
			*domain_cluster |= cluster0_mn_bit();
			*domain_cluster |= cluster1_mn_bit();
			*domain_cluster |= cluster3_mn_bit();
			break;
		case 3:
			rst_state_reg = SYS_CTRL_BASE + SC_CLUS0_CRG_RESET_ST;
			*domain_cluster |= cluster0_mn_bit();
			*domain_cluster |= cluster1_mn_bit();
			*domain_cluster |= cluster2_mn_bit();
			break;
		default:
			ERROR("%s:%d: %s: Unexpected value %d\n", __FILE__,
			      __LINE__, __func__, pos);
			panic();
		}

		rst_state = mmio_read_32(rst_state_reg);
		is_all_reset = ((rst_state & ALLRESET_BIT) == ALLRESET_BIT);
		is_current_reset = ((rst_state & ALLRESET_BIT) ==
				    (ALLRESET_BIT &
					(~(uint32_t)(ONERESET_BIT <<
						     (pos % 4)))));
		if (!is_all_reset && !is_current_reset)
			*domain_cluster |= (1 << pos);

	} else if (op == DRESET_CONTROL) {

		*domain_cluster = cluster0_mn_bit();
		*domain_cluster |= cluster1_mn_bit();
		*domain_cluster |= cluster2_mn_bit();
		*domain_cluster |= cluster3_mn_bit();
		*domain_cluster |= (1 << pos);
	}
}

#define PACKAGE_TYPE_NUM	3

union u_node_id {
	struct {
		uint32_t die_id:2;
		uint32_t socket_id:1;
		uint32_t reserved:29;
	} bits;
	uint32_t data;
};

static uint32_t die_present_mask[PACKAGE_TYPE_NUM] = {
	(1 << CDIEC_ID) | (1 << IDIE_ID),
	(1 << CDIEC_ID) | (1 << IDIE_ID) | (1 << CDIEA_ID),
	(1 << CDIEC_ID) | (1 << IDIE_ID) | (1 << CDIEA_ID) | (1 << CDIEB_ID),
};

static uint64_t peri_sub_base[][DIE_ID_MAX] = {
	{ ADDRESS_MAP_INVALID, ADDRESS_MAP_INVALID, PERI_SUB_CTRL_ADDR,
	  ADDRESS_MAP_INVALID },
	{ ADDRESS_MAP_INVALID, ADDRESS_MAP_INVALID, ADDRESS_MAP_INVALID,
	  ADDRESS_MAP_INVALID },
};

static uint64_t alg_sub_base[][DIE_ID_MAX] = {
	{ ALG_BASE, ADDRESS_MAP_INVALID, PERI_SUB_CTRL_ADDR,
	  ADDRESS_MAP_INVALID },
	{ ADDRESS_MAP_INVALID, ADDRESS_MAP_INVALID, ADDRESS_MAP_INVALID,
	  ADDRESS_MAP_INVALID },
};

static uint32_t get_package_type()
{
	return 0;
}

static uint8_t die_flag(uint64_t die, uint32_t die_mask[])
{
	uint32_t package_type = 0;

	if (die >= DIE_ID_MAX)
		return 0;

	package_type = get_package_type();

	if (package_type >= PACKAGE_TYPE_NUM)
		return 0;

	return !!(die_mask[package_type] & (1 << die));
}

static uint64_t oem_get_peri_sub_base(uint32_t node_id)
{
	union u_node_id node;

	node.data = node_id;

	return peri_sub_base[node.bits.socket_id][node.bits.die_id];
}

static uint64_t oem_get_alg_sub_base(uint32_t node_id)
{
	union u_node_id node;

	node.data = node_id;

	return alg_sub_base[node.bits.socket_id][node.bits.die_id];
}

uint8_t oem_is_socket_present(uint64_t socket)
{
	/* We only support 1P currently */
	return !socket;
}

static uint8_t is_totem_die(uint32_t node_id)
{
	union u_node_id node;

	node.data = node_id;

	return die_flag(node.bits.die_id, die_present_mask);
}

static uint8_t is_die_present(uint32_t node_id)
{
	union u_node_id node;

	node.data = node_id;

	if (!oem_is_socket_present(node.bits.socket_id))
		return 0;

	return die_flag(node.bits.die_id, die_present_mask);
}

static uint64_t get_djtag_reg_base(uint32_t node_id)
{
	if (is_totem_die(node_id))
		return oem_get_peri_sub_base(node_id) + SYS_APB_IF_BASE;
	else
		return oem_get_alg_sub_base(node_id);
}


uint32_t d02_djtag_read(uint32_t offset, uint32_t chain_id, uint32_t node_id)
{
	uint64_t base;
	uint32_t flag;

	if (!is_die_present(node_id)) {
		ERROR("%s:%d: %s: invalid node ID\n", __FILE__, __LINE__,
		      __func__);
		return DJTAG_READ_INVALID_VALUE;
	}

	base = get_djtag_reg_base(node_id);

	mmio_write_32(base + SC_DJTAG_DEBUG_MODULE_SEL, chain_id);
	mmio_write_32(base + SC_DJTAG_MSTR_WR, 0);
	mmio_write_32(base + SC_DJTAG_MSTR_ADDR, offset);

	mmio_write_32(base + SC_DJTAG_MSTR_START_EN, 1);
	do {
		flag = mmio_read_32(base + SC_DJTAG_MSTR_START_EN);
	} while (flag);

	return mmio_read_32(base + SC_DJTAG_RD_DATA0);
}

void d02_djtag_write(uint32_t offset, uint32_t value, uint32_t chain_id,
		     uint32_t node_id)
{
	uint64_t base;
	uint32_t flag;

	if (!is_die_present(node_id)) {
		ERROR("%s:%d: %s: invalid node ID\n", __FILE__, __LINE__,
		      __func__);
		return;
	}

	base = get_djtag_reg_base(node_id);

	mmio_write_32(base + SC_DJTAG_DEBUG_MODULE_SEL, chain_id);
	mmio_write_32(base + SC_DJTAG_MSTR_WR, 1);
	mmio_write_32(base + SC_DJTAG_MSTR_ADDR, offset);
	mmio_write_32(base + SC_DJTAG_MSTR_DATA, value);

	mmio_write_32(base + SC_DJTAG_MSTR_START_EN, 1);
	do {
		flag = mmio_read_32(base + SC_DJTAG_MSTR_START_EN);
	} while (flag);
}


int d02_set_sys_power_state(enum power_state state)
{
	switch (state) {
	case system_reboot:

		d02_flush_l1l2l3_cache();

		NOTICE("Rebooting");
		mmio_write_8(CPLD_BASE + 0x17, 0x55);

		/*
		 * Wait for reboot.
		 * TODO: why is this needed? (called does wfi)
		 */
		__asm__("b .");

		break;

	case system_shutdown:

		d02_flush_l1l2l3_cache();

		NOTICE("Shutting down");
		mmio_write_8(CPLD_BASE + 0x05, 0x55);

		break;

	default:
		ERROR("%s:%d: %s: unsupported system state (%d)\n", __FILE__,
		      __LINE__, __func__, state);
		return -1;
	}

	return 0;
}
