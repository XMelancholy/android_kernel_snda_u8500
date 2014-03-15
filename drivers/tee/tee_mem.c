/*
 * Copyright (C) ST-Ericsson SA 2012
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */
#include <linux/hwmem.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/sched.h>

#include "tee_mem.h"

#define TEE_MEM_PFX "TEE_MEM: "

#define HWMEM_FLAGS (							\
		HWMEM_ALLOC_HINT_WRITE_COMBINE |			\
		HWMEM_ALLOC_HINT_CACHED |				\
		HWMEM_ALLOC_HINT_CACHE_WB |				\
		HWMEM_ALLOC_HINT_CACHE_AOW |				\
		HWMEM_ALLOC_HINT_INNER_AND_OUTER_CACHE)
#define HWMEM_ACCESS (							\
		HWMEM_ACCESS_READ | HWMEM_ACCESS_WRITE |		\
		HWMEM_ACCESS_IMPORT)
#define HWMEM_TYPE (HWMEM_MEM_CONTIGUOUS_SYS)

struct tee_mem {
	struct hwmem_alloc *alloc;
	s32 gid;
	void *vaddr;
	phys_addr_t paddr;
};

struct tee_mem *tee_mem_alloc(size_t size)
{
	size_t mem_chunks_length = 1;
	struct hwmem_mem_chunk mem_chunks;
	struct tee_mem *mem = kmalloc(sizeof(struct tee_mem), GFP_KERNEL);

	if (!mem) {
		pr_err(TEE_MEM_PFX "[%s] error, out of memory\n", __func__);
		goto exit;
	}
	memset(mem, 0, sizeof(struct tee_mem));

	mem->alloc = hwmem_alloc(size, HWMEM_FLAGS, HWMEM_ACCESS, HWMEM_TYPE);
	if (IS_ERR(mem->alloc)) {
		pr_err(TEE_MEM_PFX "[%s] hwmem_alloc failed\n", __func__);
		goto exit;
	}

	if (hwmem_pin(mem->alloc, &mem_chunks, &mem_chunks_length)) {
		pr_err(TEE_MEM_PFX "[%s] hwmem_pin failed\n", __func__);
		goto exit;
	}
	mem->paddr = mem_chunks.paddr;
	if (!mem->paddr) {
		pr_err(TEE_MEM_PFX "[%s] failed to get paddr\n", __func__);
		goto exit;
	}

exit:
	if (mem->paddr <= 0) {
		hwmem_unpin(mem->alloc);
		hwmem_release(mem->alloc);
		kfree(mem);
		mem = NULL;
	}

	return mem;
}

s32 tee_mem_uva2gid(void *va)
{
	struct hwmem_alloc *alloc = hwmem_resolve_by_vm_addr(va);
	s32 ret;

	if (IS_ERR(alloc))
		return -EFAULT;

	ret = hwmem_get_name(alloc);
	hwmem_release(alloc);

	return ret;
}

void *tee_mem_get_base(void *va)
{
	void *ret = NULL;
	struct vm_area_struct *vma;
	struct mm_struct *mm = current->mm;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, (u32)va);
	if (vma == NULL)
		goto exit;

	ret = (void *)vma->vm_start;
exit:
	up_read(&mm->mmap_sem);

	return ret;
}

void tee_mem_free(struct tee_mem *mem)
{
	if (!mem)
		return;

	if (mem->vaddr)
		hwmem_kunmap(mem->alloc);

	hwmem_unpin(mem->alloc);
	hwmem_release(mem->alloc);
	kfree(mem);
}

struct tee_mem *tee_mem_register(s32 gid)
{
	int ret = 0;
	size_t mem_chunks_length = 1;
	struct hwmem_mem_chunk mem_chunks;
	struct tee_mem *mem = kmalloc(sizeof(struct tee_mem), GFP_KERNEL);

	if (!mem) {
		pr_err(TEE_MEM_PFX "[%s] error, out of memory\n", __func__);
		goto exit;
	}
	memset(mem, 0, sizeof(struct tee_mem));

	mem->alloc = hwmem_resolve_by_name(gid);
	if (IS_ERR(mem->alloc)) {
		pr_err(TEE_MEM_PFX "[%s] hwmem_resolve failed, gid [%i]\n",
		       __func__, gid);
		goto exit;
	}

	ret = hwmem_pin(mem->alloc, &mem_chunks, &mem_chunks_length);
	if (ret) {
		pr_err(TEE_MEM_PFX "[%s] hwmem_pin failed\n", __func__);
		goto exit;
	}
	mem->paddr = mem_chunks.paddr;
	if (!mem->paddr) {
		pr_err(TEE_MEM_PFX "[%s] failed to get paddr\n", __func__);
		goto exit;
	}

exit:

	if (mem && !mem->paddr) {
		hwmem_unpin(mem->alloc);
		hwmem_release(mem->alloc);
		kfree(mem);
		mem = NULL;
	}

	return mem;
}

void *tee_mem_get_vaddr(struct tee_mem *mem)
{
	if (!mem)
		return NULL;

	if (!mem->vaddr) {
		mem->vaddr = hwmem_kmap(mem->alloc);
		if (!mem->vaddr) {
			pr_err(TEE_MEM_PFX "[%s] failed to get vaddr\n",
			       __func__);
			return NULL;
		}
	}

	return mem->vaddr;
}

phys_addr_t tee_mem_get_paddr(struct tee_mem *mem)
{
	if (!mem)
		return 0;

	return mem->paddr;
}

s32 tee_mem_get_gid(struct tee_mem *mem)
{
	if (!mem)
		return 0;

	if (mem->gid == 0) {
		mem->gid = hwmem_get_name(mem->alloc);
		if (mem->gid <= 0) {
			pr_err(TEE_MEM_PFX "[%s] failed to get gid\n",
			       __func__);
			mem->gid = 0;
		}
	}

	return mem->gid;
}

u32 tee_mem_get_desc(struct tee_mem *mem)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep;
	void *va;

	if (!mem)
		return 0;

	va = tee_mem_get_vaddr(mem);

	if (va == NULL)
		return 0;

	pgd = pgd_offset_k((u32)va);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pud = pud_offset(pgd, (u32)va);
	pmd = pmd_offset(pud, (u32)va);

	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	ptep = pte_offset_kernel(pmd, (u32)va);

	if (!pte_present(*ptep))
		return 0;

	return (u32)*ptep & ~PAGE_MASK;
}
