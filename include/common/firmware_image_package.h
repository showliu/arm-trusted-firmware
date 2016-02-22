/*
 * Copyright (c) 2014, ARM Limited and Contributors. All rights reserved.
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

#ifndef __FIRMWARE_IMAGE_PACKAGE_H__
#define __FIRMWARE_IMAGE_PACKAGE_H__

#include <stdint.h>
#include <uuid.h>

/* This is used as a signature to validate the blob header */
#define TOC_HEADER_NAME	0xAA640001


/* ToC Entry UUIDs */
#define UUID_TRUSTED_BOOT_FIRMWARE_BL2 \
	{0x0becf95f, 0x224d, 0x4d3e, 0xa5, 0x44, {0xc3, 0x9d, 0x81, 0xc7, 0x3f, 0x0a} }
#define UUID_SCP_FIRMWARE_BL30 \
	{0x3dfd6697, 0xbe89, 0x49e8, 0xae, 0x5d, {0x78, 0xa1, 0x40, 0x60, 0x82, 0x13} }
#define UUID_EL3_RUNTIME_FIRMWARE_BL31 \
	{0x6d08d447, 0xfe4c, 0x4698, 0x9b, 0x95, {0x29, 0x50, 0xcb, 0xbd, 0x5a, 0x00} }
#define UUID_SECURE_PAYLOAD_BL32 \
	{0x89e1d005, 0xdc53, 0x4713, 0x8d, 0x2b, {0x50, 0x0a, 0x4b, 0x7a, 0x3e, 0x38} }
#define UUID_NON_TRUSTED_FIRMWARE_BL33 \
	{0xa7eed0d6, 0xeafc, 0x4bd5, 0x97, 0x82, {0x99, 0x34, 0xf2, 0x34, 0xb6, 0xe4} }
#define UUID_DEVICE_TREE_BLOB \
	{0x20cfb5aa, 0x6645, 0x4b5f, 0x96, 0x67, {0x75, 0xe2, 0x72, 0x63, 0xc0, 0x1d} }
/* Key certificates */
#define UUID_ROT_KEY_CERT \
	{0x721d2d86, 0x60f8, 0x11e4, 0x92, 0x0b, {0x8b, 0xe7, 0x62, 0x16, 0x0f, 0x24} }
#define UUID_TRUSTED_KEY_CERT \
	{0x90e87e82, 0x60f8, 0x11e4, 0xa1, 0xb4, {0x77, 0x7a, 0x21, 0xb4, 0xf9, 0x4c} }
#define UUID_NON_TRUSTED_WORLD_KEY_CERT \
	{0x3d87671c, 0x635f, 0x11e4, 0x97, 0x8d, {0x27, 0xc0, 0xc7, 0x14, 0x8a, 0xbd} }
#define UUID_SCP_FIRMWARE_BL30_KEY_CERT \
	{0xa1214202, 0x60f8, 0x11e4, 0x8d, 0x9b, {0xf3, 0x3c, 0x0e, 0x15, 0xa0, 0x14} }
#define UUID_EL3_RUNTIME_FIRMWARE_BL31_KEY_CERT \
	{0xccbeb88a, 0x60f9, 0x11e4, 0x9a, 0xd0, {0xeb, 0x48, 0x22, 0xd8, 0xdc, 0xf8} }
#define UUID_SECURE_PAYLOAD_BL32_KEY_CERT \
	{0x03d67794, 0x60fb, 0x11e4, 0x85, 0xdd, {0xb7, 0x10, 0x5b, 0x8c, 0xee, 0x04} }
#define UUID_NON_TRUSTED_FIRMWARE_BL33_KEY_CERT \
	{0x2a83d58a, 0x60fb, 0x11e4, 0x8a, 0xaf, {0xdf, 0x30, 0xbb, 0xc4, 0x98, 0x59} }
/* Content certificates */
#define UUID_TRUSTED_BOOT_FIRMWARE_BL2_CERT \
	{0xea69e2d6, 0x635d, 0x11e4, 0x8d, 0x8c, {0x9f, 0xba, 0xbe, 0x99, 0x56, 0xa5} }
#define UUID_SCP_FIRMWARE_BL30_CERT \
	{0x046fbe44, 0x635e, 0x11e4, 0xb2, 0x8b, {0x73, 0xd8, 0xea, 0xae, 0x96, 0x56} }
#define UUID_EL3_RUNTIME_FIRMWARE_BL31_CERT \
	{0x200cb2e2, 0x635e, 0x11e4, 0x9c, 0xe8, {0xab, 0xcc, 0xf9, 0x2b, 0xb6, 0x66} }
#define UUID_SECURE_PAYLOAD_BL32_CERT \
	{0x11449fa4, 0x635e, 0x11e4, 0x87, 0x28, {0x3f, 0x05, 0x72, 0x2a, 0xf3, 0x3d} }
#define UUID_NON_TRUSTED_FIRMWARE_BL33_CERT \
	{0xf3c1c48e, 0x635d, 0x11e4, 0xa7, 0xa9, {0x87, 0xee, 0x40, 0xb2, 0x3f, 0xa7} }

typedef struct fip_toc_header {
	uint32_t	name;
	uint32_t	serial_number;
	uint64_t	flags;
} fip_toc_header_t;

typedef struct fip_toc_entry {
	uuid_t		uuid;
	uint64_t	offset_address;
	uint64_t	size;
	uint64_t	flags;
} fip_toc_entry_t;

#endif /* __FIRMWARE_IMAGE_PACKAGE_H__ */
