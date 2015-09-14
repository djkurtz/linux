/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt)	"arm-short-desc io-pgtable: "fmt

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/errno.h>
#include "io-pgtable.h"

typedef u32 arm_short_iopte;

struct arm_short_io_pgtable {
	struct io_pgtable	iop;
	struct kmem_cache	*pgtable_cached;
	size_t			pgd_size;
	void			*pgd;
};

#define io_pgtable_to_data(x)			\
	container_of((x), struct arm_short_io_pgtable, iop)

#define io_pgtable_ops_to_data(x)		\
	io_pgtable_to_data(io_pgtable_ops_to_pgtable(x))

#define io_pgtable_cfg_to_pgtable(x)		\
	container_of((x), struct io_pgtable, cfg)

#define io_pgtable_cfg_to_data(x)		\
	io_pgtable_to_data(io_pgtable_cfg_to_pgtable(x))

#define ARM_SHORT_PGDIR_SHIFT			20
#define ARM_SHORT_PAGE_SHIFT			12
#define ARM_SHORT_PTRS_PER_PTE			\
	(1 << (ARM_SHORT_PGDIR_SHIFT - ARM_SHORT_PAGE_SHIFT))
#define ARM_SHORT_BYTES_PER_PTE			\
	(ARM_SHORT_PTRS_PER_PTE * sizeof(arm_short_iopte))

/* level 1 pagetable */
#define ARM_SHORT_PGD_TYPE_PGTABLE		BIT(0)
#define ARM_SHORT_PGD_TYPE_SECTION		BIT(1)
#define ARM_SHORT_PGD_B				BIT(2)
#define ARM_SHORT_PGD_C				BIT(3)
#define ARM_SHORT_PGD_PGTABLE_NS		BIT(3)
#define ARM_SHORT_PGD_SECTION_XN		BIT(4)
#define ARM_SHORT_PGD_IMPLE			BIT(9)
#define ARM_SHORT_PGD_RD_WR			(3 << 10)
#define ARM_SHORT_PGD_RDONLY			BIT(15)
#define ARM_SHORT_PGD_S				BIT(16)
#define ARM_SHORT_PGD_nG			BIT(17)
#define ARM_SHORT_PGD_SUPERSECTION		BIT(18)
#define ARM_SHORT_PGD_SECTION_NS		BIT(19)

#define ARM_SHORT_PGD_TYPE_SUPERSECTION		\
	(ARM_SHORT_PGD_TYPE_SECTION | ARM_SHORT_PGD_SUPERSECTION)
#define ARM_SHORT_PGD_SECTION_TYPE_MSK		\
	(ARM_SHORT_PGD_TYPE_SECTION | ARM_SHORT_PGD_SUPERSECTION)
#define ARM_SHORT_PGD_PGTABLE_TYPE_MSK		\
	(ARM_SHORT_PGD_TYPE_SECTION | ARM_SHORT_PGD_TYPE_PGTABLE)
#define ARM_SHORT_PGD_TYPE_IS_PGTABLE(pgd)	\
	(((pgd) & ARM_SHORT_PGD_PGTABLE_TYPE_MSK) == ARM_SHORT_PGD_TYPE_PGTABLE)
#define ARM_SHORT_PGD_TYPE_IS_SECTION(pgd)	\
	(((pgd) & ARM_SHORT_PGD_SECTION_TYPE_MSK) == ARM_SHORT_PGD_TYPE_SECTION)
#define ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(pgd)	\
	(((pgd) & ARM_SHORT_PGD_SECTION_TYPE_MSK) == \
	ARM_SHORT_PGD_TYPE_SUPERSECTION)
#define ARM_SHORT_PGD_PGTABLE_MSK		0xfffffc00
#define ARM_SHORT_PGD_SECTION_MSK		(~(SZ_1M - 1))
#define ARM_SHORT_PGD_SUPERSECTION_MSK		(~(SZ_16M - 1))

/* level 2 pagetable */
#define ARM_SHORT_PTE_TYPE_LARGE		BIT(0)
#define ARM_SHORT_PTE_SMALL_XN			BIT(0)
#define ARM_SHORT_PTE_TYPE_SMALL		BIT(1)
#define ARM_SHORT_PTE_B				BIT(2)
#define ARM_SHORT_PTE_C				BIT(3)
#define ARM_SHORT_PTE_RD_WR			(3 << 4)
#define ARM_SHORT_PTE_RDONLY			BIT(9)
#define ARM_SHORT_PTE_S				BIT(10)
#define ARM_SHORT_PTE_nG			BIT(11)
#define ARM_SHORT_PTE_LARGE_XN			BIT(15)
#define ARM_SHORT_PTE_LARGE_MSK			(~(SZ_64K - 1))
#define ARM_SHORT_PTE_SMALL_MSK			(~(SZ_4K - 1))
#define ARM_SHORT_PTE_TYPE_MSK			\
	(ARM_SHORT_PTE_TYPE_LARGE | ARM_SHORT_PTE_TYPE_SMALL)
#define ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(pte)	\
	(((pte) & ARM_SHORT_PTE_TYPE_SMALL) == ARM_SHORT_PTE_TYPE_SMALL)
#define ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(pte)	\
	(((pte) & ARM_SHORT_PTE_TYPE_MSK) == ARM_SHORT_PTE_TYPE_LARGE)

#define ARM_SHORT_PGD_IDX(a)			((a) >> ARM_SHORT_PGDIR_SHIFT)
#define ARM_SHORT_PTE_IDX(a)			\
	(((a) >> ARM_SHORT_PAGE_SHIFT) & (ARM_SHORT_PTRS_PER_PTE - 1))

#define ARM_SHORT_GET_PGTABLE_VA(pgd)		\
	(phys_to_virt((unsigned long)pgd & ARM_SHORT_PGD_PGTABLE_MSK))

#define ARM_SHORT_PTE_LARGE_GET_PROT(pte)	\
	(((pte) & (~ARM_SHORT_PTE_LARGE_MSK)) & ~ARM_SHORT_PTE_TYPE_MSK)

#define ARM_SHORT_PGD_GET_PROT(pgd)		\
	(((pgd) & (~ARM_SHORT_PGD_SECTION_MSK)) & ~ARM_SHORT_PGD_SUPERSECTION)

static bool selftest_running;

static arm_short_iopte *
arm_short_get_pte_in_pgd(arm_short_iopte pgd, unsigned int iova)
{
	arm_short_iopte *pte;

	pte = ARM_SHORT_GET_PGTABLE_VA(pgd);
	pte += ARM_SHORT_PTE_IDX(iova);
	return pte;
}

static dma_addr_t
__arm_short_dma_addr(struct device *dev, void *va)
{
	return phys_to_dma(dev, virt_to_phys(va));
}

static int
__arm_short_set_pte(arm_short_iopte *ptep, arm_short_iopte pte,
		    unsigned int ptenr, struct io_pgtable_cfg *cfg)
{
	struct device *dev = cfg->iommu_dev;
	int i;

	for (i = 0; i < ptenr; i++) {
		if (ptep[i] && pte) {
			/* Someone else may have allocated for this pte */
			WARN_ON(!selftest_running);
			goto err_exist_pte;
		}
		ptep[i] = pte;
	}

	if (selftest_running)
		return 0;

	dma_sync_single_for_device(dev, __arm_short_dma_addr(dev, ptep),
				   sizeof(*ptep) * ptenr, DMA_TO_DEVICE);
	return 0;

err_exist_pte:
	while (i--)
		ptep[i] = 0;
	return -EEXIST;
}

static void *
__arm_short_alloc_pgtable(size_t size, gfp_t gfp, bool pgd,
			  struct io_pgtable_cfg *cfg)
{
	struct arm_short_io_pgtable *data = io_pgtable_cfg_to_data(cfg);
	struct device *dev = cfg->iommu_dev;
	dma_addr_t dma;
	void *va;

	if (pgd) /* lvl1 pagetable */
		va = alloc_pages_exact(size, gfp);
	else     /* lvl2 pagetable */
		va = kmem_cache_zalloc(data->pgtable_cached, gfp);

	if (!va)
		return NULL;

	if (selftest_running)
		return va;

	dma = dma_map_single(dev, va, size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma))
		goto out_free;

	if (dma != __arm_short_dma_addr(dev, va))
		goto out_unmap;

	if (!pgd) {
		kmemleak_ignore(va);
		dma_sync_single_for_device(dev, __arm_short_dma_addr(dev, va),
					   size, DMA_TO_DEVICE);
	}

	return va;

out_unmap:
	dev_err_ratelimited(dev, "Cannot accommodate DMA translation for IOMMU page tables\n");
	dma_unmap_single(dev, dma, size, DMA_TO_DEVICE);
out_free:
	if (pgd)
		free_pages_exact(va, size);
	else
		kmem_cache_free(data->pgtable_cached, va);
	return NULL;
}

static void
__arm_short_free_pgtable(void *va, size_t size, bool pgd,
			 struct io_pgtable_cfg *cfg)
{
	struct arm_short_io_pgtable *data = io_pgtable_cfg_to_data(cfg);
	struct device *dev = cfg->iommu_dev;

	if (!selftest_running)
		dma_unmap_single(dev, __arm_short_dma_addr(dev, va),
				 size, DMA_TO_DEVICE);

	if (pgd)
		free_pages_exact(va, size);
	else
		kmem_cache_free(data->pgtable_cached, va);
}

static arm_short_iopte
__arm_short_pte_prot(struct arm_short_io_pgtable *data, int prot, bool large)
{
	arm_short_iopte pteprot;
	int quirk = data->iop.cfg.quirks;

	pteprot = ARM_SHORT_PTE_S | ARM_SHORT_PTE_nG;
	pteprot |= large ? ARM_SHORT_PTE_TYPE_LARGE :
				ARM_SHORT_PTE_TYPE_SMALL;
	if (prot & IOMMU_CACHE)
		pteprot |=  ARM_SHORT_PTE_B | ARM_SHORT_PTE_C;
	if (!(quirk & IO_PGTABLE_QUIRK_SHORT_NO_XN) && (prot & IOMMU_NOEXEC)) {
			pteprot |= large ? ARM_SHORT_PTE_LARGE_XN :
				ARM_SHORT_PTE_SMALL_XN;
	}
	if (!(quirk & IO_PGTABLE_QUIRK_SHORT_NO_PERMS)) {
		pteprot |= ARM_SHORT_PTE_RD_WR;
		if (!(prot & IOMMU_WRITE) && (prot & IOMMU_READ))
			pteprot |= ARM_SHORT_PTE_RDONLY;
	}
	return pteprot;
}

static arm_short_iopte
__arm_short_pgd_prot(struct arm_short_io_pgtable *data, int prot, bool super)
{
	arm_short_iopte pgdprot;
	int quirk = data->iop.cfg.quirks;

	pgdprot = ARM_SHORT_PGD_S | ARM_SHORT_PGD_nG;
	pgdprot |= super ? ARM_SHORT_PGD_TYPE_SUPERSECTION :
				ARM_SHORT_PGD_TYPE_SECTION;
	if (prot & IOMMU_CACHE)
		pgdprot |= ARM_SHORT_PGD_C | ARM_SHORT_PGD_B;
	if (quirk & IO_PGTABLE_QUIRK_ARM_NS)
		pgdprot |= ARM_SHORT_PGD_SECTION_NS;

	if (!(quirk & IO_PGTABLE_QUIRK_SHORT_NO_XN) && (prot & IOMMU_NOEXEC))
			pgdprot |= ARM_SHORT_PGD_SECTION_XN;

	if (!(quirk & IO_PGTABLE_QUIRK_SHORT_NO_PERMS)) {
		pgdprot |= ARM_SHORT_PGD_RD_WR;
		if (!(prot & IOMMU_WRITE) && (prot & IOMMU_READ))
			pgdprot |= ARM_SHORT_PGD_RDONLY;
	}
	return pgdprot;
}

static arm_short_iopte
__arm_short_pte_prot_split(struct arm_short_io_pgtable *data,
			   arm_short_iopte pgdprot,
			   arm_short_iopte pteprot_large,
			   bool large)
{
	arm_short_iopte pteprot = 0;

	pteprot = ARM_SHORT_PTE_S | ARM_SHORT_PTE_nG | ARM_SHORT_PTE_RD_WR;
	pteprot |= large ? ARM_SHORT_PTE_TYPE_LARGE :
				ARM_SHORT_PTE_TYPE_SMALL;

	/* large page to small page pte prot. Only large page may split */
	if (!pgdprot && !large) {
		pteprot |= pteprot_large & ~ARM_SHORT_PTE_SMALL_MSK;
		if (pteprot_large & ARM_SHORT_PTE_LARGE_XN)
			pteprot |= ARM_SHORT_PTE_SMALL_XN;
	}

	/* section to pte prot */
	if (pgdprot & ARM_SHORT_PGD_C)
		pteprot |= ARM_SHORT_PTE_C;
	if (pgdprot & ARM_SHORT_PGD_B)
		pteprot |= ARM_SHORT_PTE_B;
	if (pgdprot & ARM_SHORT_PGD_nG)
		pteprot |= ARM_SHORT_PTE_nG;
	if (pgdprot & ARM_SHORT_PGD_SECTION_XN)
		pteprot |= large ? ARM_SHORT_PTE_LARGE_XN :
				ARM_SHORT_PTE_SMALL_XN;
	if (pgdprot & ARM_SHORT_PGD_RD_WR)
		pteprot |= ARM_SHORT_PTE_RD_WR;
	if (pgdprot & ARM_SHORT_PGD_RDONLY)
		pteprot |= ARM_SHORT_PTE_RDONLY;

	return pteprot;
}

static arm_short_iopte
__arm_short_pgtable_prot(struct arm_short_io_pgtable *data)
{
	arm_short_iopte pgdprot = 0;

	pgdprot = ARM_SHORT_PGD_TYPE_PGTABLE;
	if (data->iop.cfg.quirks & IO_PGTABLE_QUIRK_ARM_NS)
		pgdprot |= ARM_SHORT_PGD_PGTABLE_NS;
	return pgdprot;
}

static int
_arm_short_map(struct arm_short_io_pgtable *data,
	       unsigned int iova, phys_addr_t paddr,
	       arm_short_iopte pgdprot, arm_short_iopte pteprot,
	       bool large)
{
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	arm_short_iopte *pgd = data->pgd, *pte;
	void *pte_new = NULL;
	int ret;

	pgd += ARM_SHORT_PGD_IDX(iova);

	if (!pteprot) { /* section or supersection */
		pte = pgd;
		pteprot = pgdprot;
	} else {        /* page or largepage */
		if (!(*pgd)) {
			pte_new = __arm_short_alloc_pgtable(
					ARM_SHORT_BYTES_PER_PTE,
					GFP_ATOMIC, false, cfg);
			if (unlikely(!pte_new))
				return -ENOMEM;

			pgdprot |= virt_to_phys(pte_new);
			__arm_short_set_pte(pgd, pgdprot, 1, cfg);
		}
		pte = arm_short_get_pte_in_pgd(*pgd, iova);
	}

	pteprot |= (arm_short_iopte)paddr;
	ret = __arm_short_set_pte(pte, pteprot, large ? 16 : 1, cfg);
	if (ret && pte_new)
		__arm_short_free_pgtable(pte_new, ARM_SHORT_BYTES_PER_PTE,
					 false, cfg);
	return ret;
}

static int arm_short_map(struct io_pgtable_ops *ops, unsigned long iova,
			 phys_addr_t paddr, size_t size, int prot)
{
	struct arm_short_io_pgtable *data = io_pgtable_ops_to_data(ops);
	arm_short_iopte pgdprot = 0, pteprot = 0;
	bool large;

	/* If no access, then nothing to do */
	if (!(prot & (IOMMU_READ | IOMMU_WRITE)))
		return 0;

	if (WARN_ON((iova | paddr) & (size - 1)))
		return -EINVAL;

	switch (size) {
	case SZ_4K:
	case SZ_64K:
		large = (size == SZ_64K) ? true : false;
		pteprot = __arm_short_pte_prot(data, prot, large);
		pgdprot = __arm_short_pgtable_prot(data);
		break;

	case SZ_1M:
	case SZ_16M:
		large = (size == SZ_16M) ? true : false;
		pgdprot = __arm_short_pgd_prot(data, prot, large);
		break;
	default:
		return -EINVAL;
	}

	return _arm_short_map(data, iova, paddr, pgdprot, pteprot, large);
}

static phys_addr_t arm_short_iova_to_phys(struct io_pgtable_ops *ops,
					  unsigned long iova)
{
	struct arm_short_io_pgtable *data = io_pgtable_ops_to_data(ops);
	arm_short_iopte *pte, *pgd = data->pgd;
	phys_addr_t pa = 0;

	pgd += ARM_SHORT_PGD_IDX(iova);

	if (ARM_SHORT_PGD_TYPE_IS_PGTABLE(*pgd)) {
		pte = arm_short_get_pte_in_pgd(*pgd, iova);

		if (ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(*pte)) {
			pa = (*pte) & ARM_SHORT_PTE_LARGE_MSK;
			pa |= iova & ~ARM_SHORT_PTE_LARGE_MSK;
		} else if (ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(*pte)) {
			pa = (*pte) & ARM_SHORT_PTE_SMALL_MSK;
			pa |= iova & ~ARM_SHORT_PTE_SMALL_MSK;
		}
	} else if (ARM_SHORT_PGD_TYPE_IS_SECTION(*pgd)) {
		pa = (*pgd) & ARM_SHORT_PGD_SECTION_MSK;
		pa |= iova & ~ARM_SHORT_PGD_SECTION_MSK;
	} else if (ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(*pgd)) {
		pa = (*pgd) & ARM_SHORT_PGD_SUPERSECTION_MSK;
		pa |= iova & ~ARM_SHORT_PGD_SUPERSECTION_MSK;
	}

	return pa;
}

static bool _arm_short_whether_free_pgtable(arm_short_iopte *pgd)
{
	arm_short_iopte *pte;
	int i;

	pte = ARM_SHORT_GET_PGTABLE_VA(*pgd);
	for (i = 0; i < ARM_SHORT_PTRS_PER_PTE; i++) {
		if (pte[i] != 0)
			return false;
	}

	return true;
}

static int
arm_short_split_blk_unmap(struct io_pgtable_ops *ops, unsigned int iova,
			  phys_addr_t paddr, size_t size,
			  arm_short_iopte pgdprotup, arm_short_iopte pteprotup,
			  size_t blk_size)
{
	struct arm_short_io_pgtable *data = io_pgtable_ops_to_data(ops);
	const struct iommu_gather_ops *tlb = data->iop.cfg.tlb;
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	unsigned long *pgbitmap = &cfg->pgsize_bitmap;
	unsigned int blk_base, blk_start, blk_end, i;
	arm_short_iopte pgdprot, pteprot;
	phys_addr_t blk_paddr;
	size_t mapsize = 0, nextmapsize;
	int ret;

	/* find the nearest mapsize */
	for (i = find_first_bit(pgbitmap, BITS_PER_LONG);
	     i < BITS_PER_LONG && ((1 << i) < blk_size) &&
	     IS_ALIGNED(size, 1 << i);
	     i = find_next_bit(pgbitmap, BITS_PER_LONG, i + 1))
		mapsize = 1 << i;

	if (WARN_ON(!mapsize))
		return 0; /* Bytes unmapped */
	nextmapsize = 1 << i;

	blk_base = iova & ~(blk_size - 1);
	blk_start = blk_base;
	blk_end = blk_start + blk_size;
	blk_paddr = paddr;

	for (; blk_start < blk_end;
	     blk_start += mapsize, blk_paddr += mapsize) {
		/* Unmap! */
		if (blk_start == iova)
			continue;

		/* Try to upper map */
		if (blk_base != blk_start &&
		    IS_ALIGNED(blk_start | blk_paddr, nextmapsize) &&
		    mapsize != nextmapsize) {
			mapsize = nextmapsize;
			i = find_next_bit(pgbitmap, BITS_PER_LONG, i + 1);
			if (i < BITS_PER_LONG)
				nextmapsize = 1 << i;
		}

		if (mapsize == SZ_1M) {
			pgdprot = pgdprotup;
			pgdprot |= __arm_short_pgd_prot(data, 0, false);
			pteprot = 0;
		} else { /* small or large page */
			pgdprot = (blk_size == SZ_64K) ? 0 : pgdprotup;
			pteprot = __arm_short_pte_prot_split(
					data, pgdprot, pteprotup,
					mapsize == SZ_64K);
			pgdprot = __arm_short_pgtable_prot(data);
		}

		ret = _arm_short_map(data, blk_start, blk_paddr, pgdprot,
				     pteprot, mapsize == SZ_64K);
		if (ret < 0) {
			/* Free the table we allocated */
			arm_short_iopte *pgd = data->pgd, *pte;

			pgd += ARM_SHORT_PGD_IDX(blk_base);
			if (*pgd) {
				pte = ARM_SHORT_GET_PGTABLE_VA(*pgd);
				__arm_short_set_pte(pgd, 0, 1, cfg);
				tlb->tlb_add_flush(blk_base, blk_size, true,
						   data->iop.cookie);
				tlb->tlb_sync(data->iop.cookie);
				__arm_short_free_pgtable(
					pte, ARM_SHORT_BYTES_PER_PTE,
					false, cfg);
			}
			return 0;/* Bytes unmapped */
		}
	}

	tlb->tlb_add_flush(blk_base, blk_size, true, data->iop.cookie);
	tlb->tlb_sync(data->iop.cookie);
	return size;
}

static int arm_short_unmap(struct io_pgtable_ops *ops,
			   unsigned long iova,
			   size_t size)
{
	struct arm_short_io_pgtable *data = io_pgtable_ops_to_data(ops);
	struct io_pgtable_cfg *cfg = &data->iop.cfg;
	arm_short_iopte *pgd, *pte = NULL;
	arm_short_iopte curpgd, curpte = 0;
	phys_addr_t paddr;
	unsigned int iova_base, blk_size = 0;
	void *cookie = data->iop.cookie;
	bool pgtablefree = false;

	pgd = (arm_short_iopte *)data->pgd + ARM_SHORT_PGD_IDX(iova);

	/* Get block size */
	if (ARM_SHORT_PGD_TYPE_IS_PGTABLE(*pgd)) {
		pte = arm_short_get_pte_in_pgd(*pgd, iova);

		if (ARM_SHORT_PTE_TYPE_IS_SMALLPAGE(*pte))
			blk_size = SZ_4K;
		else if (ARM_SHORT_PTE_TYPE_IS_LARGEPAGE(*pte))
			blk_size = SZ_64K;
		else
			WARN_ON(1);
	} else if (ARM_SHORT_PGD_TYPE_IS_SECTION(*pgd)) {
		blk_size = SZ_1M;
	} else if (ARM_SHORT_PGD_TYPE_IS_SUPERSECTION(*pgd)) {
		blk_size = SZ_16M;
	} else {
		WARN_ON(1);
	}

	iova_base = iova & ~(blk_size - 1);
	pgd = (arm_short_iopte *)data->pgd + ARM_SHORT_PGD_IDX(iova_base);
	paddr = arm_short_iova_to_phys(ops, iova_base);
	curpgd = *pgd;

	if (blk_size == SZ_4K || blk_size == SZ_64K) {
		pte = arm_short_get_pte_in_pgd(*pgd, iova_base);
		curpte = *pte;
		__arm_short_set_pte(pte, 0, blk_size / SZ_4K, cfg);

		pgtablefree = _arm_short_whether_free_pgtable(pgd);
		if (pgtablefree)
			__arm_short_set_pte(pgd, 0, 1, cfg);
	} else if (blk_size == SZ_1M || blk_size == SZ_16M) {
		__arm_short_set_pte(pgd, 0, blk_size / SZ_1M, cfg);
	}

	cfg->tlb->tlb_add_flush(iova_base, blk_size, true, cookie);
	cfg->tlb->tlb_sync(cookie);

	if (pgtablefree)/* Free pgtable after tlb-flush */
		__arm_short_free_pgtable(ARM_SHORT_GET_PGTABLE_VA(curpgd),
					 ARM_SHORT_BYTES_PER_PTE, false, cfg);

	if (blk_size > size) { /* Split the block */
		return arm_short_split_blk_unmap(
				ops, iova, paddr, size,
				ARM_SHORT_PGD_GET_PROT(curpgd),
				ARM_SHORT_PTE_LARGE_GET_PROT(curpte),
				blk_size);
	} else if (blk_size < size) {
		/* Unmap the block while remap partial again after split */
		return blk_size +
			arm_short_unmap(ops, iova + blk_size, size - blk_size);
	}

	return size;
}

static struct io_pgtable *
arm_short_alloc_pgtable(struct io_pgtable_cfg *cfg, void *cookie)
{
	struct arm_short_io_pgtable *data;

	if (cfg->ias > 32 || cfg->oas > 32)
		return NULL;

	cfg->pgsize_bitmap &=
		(cfg->quirks & IO_PGTABLE_QUIRK_SHORT_SUPERSECTION) ?
		(SZ_4K | SZ_64K | SZ_1M | SZ_16M) : (SZ_4K | SZ_64K | SZ_1M);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	data->pgd_size = SZ_16K;
	data->pgd = __arm_short_alloc_pgtable(
					data->pgd_size,
					GFP_KERNEL | __GFP_ZERO | __GFP_DMA,
					true, cfg);
	if (!data->pgd)
		goto out_free_data;
	wmb();/* Ensure the empty pgd is visible before any actual TTBR write */

	data->pgtable_cached = kmem_cache_create(
					"io-pgtable-arm-short",
					 ARM_SHORT_BYTES_PER_PTE,
					 ARM_SHORT_BYTES_PER_PTE,
					 0, NULL);
	if (!data->pgtable_cached)
		goto out_free_pgd;

	/* TTBRs */
	cfg->arm_short_cfg.ttbr[0] = virt_to_phys(data->pgd);
	cfg->arm_short_cfg.ttbr[1] = 0;
	cfg->arm_short_cfg.tcr = 0;
	cfg->arm_short_cfg.nmrr = 0;
	cfg->arm_short_cfg.prrr = 0;

	data->iop.ops = (struct io_pgtable_ops) {
		.map		= arm_short_map,
		.unmap		= arm_short_unmap,
		.iova_to_phys	= arm_short_iova_to_phys,
	};

	return &data->iop;

out_free_pgd:
	__arm_short_free_pgtable(data->pgd, data->pgd_size, true, cfg);
out_free_data:
	kfree(data);
	return NULL;
}

static void arm_short_free_pgtable(struct io_pgtable *iop)
{
	struct arm_short_io_pgtable *data = io_pgtable_to_data(iop);

	kmem_cache_destroy(data->pgtable_cached);
	__arm_short_free_pgtable(data->pgd, data->pgd_size,
				 true, &data->iop.cfg);
	kfree(data);
}

struct io_pgtable_init_fns io_pgtable_arm_short_init_fns = {
	.alloc	= arm_short_alloc_pgtable,
	.free	= arm_short_free_pgtable,
};

#ifdef CONFIG_IOMMU_IO_PGTABLE_SHORT_SELFTEST

static struct io_pgtable_cfg *cfg_cookie;

static void dummy_tlb_flush_all(void *cookie)
{
	WARN_ON(cookie != cfg_cookie);
}

static void dummy_tlb_add_flush(unsigned long iova, size_t size, bool leaf,
				void *cookie)
{
	WARN_ON(cookie != cfg_cookie);
	WARN_ON(!(size & cfg_cookie->pgsize_bitmap));
}

static void dummy_tlb_sync(void *cookie)
{
	WARN_ON(cookie != cfg_cookie);
}

static struct iommu_gather_ops dummy_tlb_ops = {
	.tlb_flush_all	= dummy_tlb_flush_all,
	.tlb_add_flush	= dummy_tlb_add_flush,
	.tlb_sync	= dummy_tlb_sync,
};

#define __FAIL(ops)	({				\
		WARN(1, "selftest: test failed\n");	\
		selftest_running = false;		\
		-EFAULT;				\
})

static int __init arm_short_do_selftests(void)
{
	struct io_pgtable_ops *ops;
	struct io_pgtable_cfg cfg = {
		.tlb = &dummy_tlb_ops,
		.oas = 32,
		.ias = 32,
		.quirks = IO_PGTABLE_QUIRK_ARM_NS |
			IO_PGTABLE_QUIRK_SHORT_SUPERSECTION,
		.pgsize_bitmap = SZ_4K | SZ_64K | SZ_1M | SZ_16M,
	};
	unsigned int iova, size, iova_start;
	unsigned int i, loopnr = 0;

	selftest_running = true;

	cfg_cookie = &cfg;

	ops = alloc_io_pgtable_ops(ARM_SHORT_DESC, &cfg, &cfg);
	if (!ops) {
		pr_err("Failed to alloc short desc io pgtable\n");
		return -EINVAL;
	}

	/*
	 * Initial sanity checks.
	 * Empty page tables shouldn't provide any translations.
	 */
	if (ops->iova_to_phys(ops, 42))
		return __FAIL(ops);

	if (ops->iova_to_phys(ops, SZ_1G + 42))
		return __FAIL(ops);

	if (ops->iova_to_phys(ops, SZ_2G + 42))
		return __FAIL(ops);

	/*
	 * Distinct mappings of different granule sizes.
	 */
	iova = 0;
	i = find_first_bit(&cfg.pgsize_bitmap, BITS_PER_LONG);
	while (i != BITS_PER_LONG) {
		size = 1UL << i;
		if (ops->map(ops, iova, iova, size, IOMMU_READ |
						    IOMMU_WRITE |
						    IOMMU_NOEXEC |
						    IOMMU_CACHE))
			return __FAIL(ops);

		/* Overlapping mappings */
		if (!ops->map(ops, iova, iova + size, size,
			      IOMMU_READ | IOMMU_NOEXEC))
			return __FAIL(ops);

		if (ops->iova_to_phys(ops, iova + 42) != (iova + 42))
			return __FAIL(ops);

		iova += SZ_16M;
		i++;
		i = find_next_bit(&cfg.pgsize_bitmap, BITS_PER_LONG, i);
		loopnr++;
	}

	/* Partial unmap */
	i = 1;
	size = 1UL << __ffs(cfg.pgsize_bitmap);
	while (i < loopnr) {
		iova_start = i * SZ_16M;
		if (ops->unmap(ops, iova_start + size, size) != size)
			return __FAIL(ops);

		/* Remap of partial unmap */
		if (ops->map(ops, iova_start + size, size, size, IOMMU_READ))
			return __FAIL(ops);

		if (ops->iova_to_phys(ops, iova_start + size + 42)
		    != (size + 42))
			return __FAIL(ops);
		i++;
	}

	/* Full unmap */
	iova = 0;
	i = find_first_bit(&cfg.pgsize_bitmap, BITS_PER_LONG);
	while (i != BITS_PER_LONG) {
		size = 1UL << i;

		if (ops->unmap(ops, iova, size) != size)
			return __FAIL(ops);

		if (ops->iova_to_phys(ops, iova + 42))
			return __FAIL(ops);

		/* Remap full block */
		if (ops->map(ops, iova, iova, size, IOMMU_WRITE))
			return __FAIL(ops);

		if (ops->iova_to_phys(ops, iova + 42) != (iova + 42))
			return __FAIL(ops);

		iova += SZ_16M;
		i++;
		i = find_next_bit(&cfg.pgsize_bitmap, BITS_PER_LONG, i);
	}

	free_io_pgtable_ops(ops);

	selftest_running = false;
	return 0;
}

subsys_initcall(arm_short_do_selftests);
#endif
