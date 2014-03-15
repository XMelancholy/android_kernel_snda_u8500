/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef TEE_MEM_H
#define TEE_MEM_H

#include <linux/types.h>

#define TEE_MEM_DESC_XN              (1 << 0)
#define TEE_MEM_DESC_AP_MASK         (3 << 4)
#define TEE_MEM_DESC_AP0             (1 << 4)
#define TEE_MEM_DESC_AP1             (2 << 4)
#define TEE_MEM_DESC_TEX(x)          ((x) << 6)
#define TEE_MEM_DESC_APX             (1 << 9)
#define TEE_MEM_DESC_SHARED          (1 << 10)
#define TEE_MEM_DESC_NG              (1 << 11)

struct tee_mem;

/**
 * tee_mem_alloc - Allocate TEE memory
 * @size: Size of memory to allocate
 *
 * Returns a TEE memory handle
 */
struct tee_mem *tee_mem_alloc(size_t size);

/**
 * tee_mem_register - Register memory based on GID (Global ID)
 * @gid: HWMEM GID
 *
 * Returns a TEE memory handle
 */
struct tee_mem *tee_mem_register(s32 gid);

/**
 * tee_mem_free - Free TEE memory
 * @mem: TEE memory handle
 */
void tee_mem_free(struct tee_mem *mem);

/**
 * tee_mem_get_vaddr - Get virtual address of TEE memory buffer and map it to
 * kernel memory
 * @mem: TEE memory handle
 *
 * Returns the virtual address of TEE memory buffer
 */
void *tee_mem_get_vaddr(struct tee_mem *mem);

/**
 * tee_mem_get_paddr - Get physical address of TEE memory buffer
 * @mem: TEE memory handle
 *
 * Returns the physical address of TEE memory buffer
 */
phys_addr_t tee_mem_get_paddr(struct tee_mem *mem);

/**
 * tee_mem_get_gid - Get GID of TEE memory buffer
 * @mem: TEE memory handle
 *
 * Returns the GID of the TEE memory
 */
s32 tee_mem_get_gid(struct tee_mem *mem);

/**
 * tee_mem_get_gid - Get memory descriptor of TEE memory buffer
 * @mem: TEE memory handle
 *
 * Returns the memory descriptor of the memory
 */
u32 tee_mem_get_desc(struct tee_mem *mem);

/**
 * tee_mem_get_base - Get the virtual user space start address of TEE memory
 * buffer
 * @va: Any virtual user space address within the buffer
 *
 * Returns the user space start address of the buffer
 */
void *tee_mem_get_base(void *va);

/**
 * tee_mem_uva2gid - Get the GID based on any virtual user space start address
 * of TEE memory
 * @va: Any virtual user space address within the buffer
 *
 * Returns the GID of the buffer
 */
s32 tee_mem_uva2gid(void *va);

#endif
