/*
 * Copyright (c) 2013-2016, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <gic_v2.h>
#include <interrupt_mgmt.h>
#include <platform.h>
#include "d02_def.h"

/*******************************************************************************
 * This functions reads the GIC cpu interface Interrupt Acknowledge register
 * to start handling the pending interrupt. It returns the contents of the IAR.
 ******************************************************************************/
uint32_t plat_gic_acknowledge_interrupt(void)
{
	return gicc_read_IAR(GICC_BASE);
}

/*******************************************************************************
 * This function returns the type of the highest priority pending interrupt at
 * the GIC cpu interface. INTR_TYPE_INVAL is returned when there is no
 * interrupt pending.
 ******************************************************************************/
uint32_t plat_ic_get_pending_interrupt_type(void)
{
	uint32_t id;

	id = gicc_read_hppir(GICC_BASE);

	/* Assume that all secure interrupts are EL3 interrupts? */
	if (id < 1022)
		return INTR_TYPE_EL3;

	if (id == GIC_SPURIOUS_INTERRUPT)
		return INTR_TYPE_INVAL;

	return INTR_TYPE_NS;
}

uint32_t plat_interrupt_type_to_line(uint32_t type, uint32_t security_state)
{
	assert(type == INTR_TYPE_S_EL1 ||
	       type == INTR_TYPE_EL3 ||
	       type == INTR_TYPE_NS);

	assert(sec_state_is_valid(security_state));

	return gicv2_interrupt_type_to_line(GICC_BASE, type);
}
