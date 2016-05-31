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

#include <arch.h>
#include <asm_macros.S>
#include "d02_def.h"

	.globl	platform_is_primary_cpu
	.globl	platform_get_entrypoint
	.globl	plat_secondary_cold_boot_setup
	.globl	plat_report_exception

func platform_is_primary_cpu
	and	x0, x0, #(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK)
	cmp	x0, #PRIMARY_CPU
	cset	x0, eq
	ret
endfunc platform_is_primary_cpu

func platform_get_entrypoint
	mov	x9, x30 // lr

	mrs	x0, mpidr_el1
	bl	platform_get_core_pos

	cbnz	x0, warmboot_run	/* Secondary core: read mailbox directly */
					/* TODO: why? */

	/* Primary core */
	ldr	x1, =PRIMARY_CORE_CPUON_FLAG_ADDR
	ldr	w2, =MAGIC_NUMBER
	ldr	w3, [x1]
	cmp	w2, w3
#if 0
/*
 * FIXME: primary core hangs after reset if allowed to take the warm boot path
 */
	beq	warmboot_run		/* Primary warm boot: read mailbox */
#endif

	/* Primary core cold boot: clear the mailbox for this core */
	ldr	x1, =TRUSTED_MAILBOXES_BASE
	ldr	x2, =0x0
	str	x2, [x1]
	str	x2, [x1, #8]

warmboot_run:
	ldr	x1, =TRUSTED_MAILBOXES_BASE
	lsl	x0, x0, #TRUSTED_MAILBOX_SHIFT
	ldr	x0, [x1, x0]

	ret x9
endfunc platform_get_entrypoint


func plat_secondary_cold_boot_setup
	/* TODO: is this really called? */
1:
	b	1b
endfunc plat_secondary_cold_boot_setup

.section .rodata.exception_msg
exception_msg:
	.asciz	"** ARM-TF EXCEPTION **\n\n"

func plat_report_exception
	adr	x4, exception_msg
	bl	asm_print_str
	mrs	x1, CurrentEl
endfunc plat_report_exception
