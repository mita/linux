// SPDX-License-Identifier: GPL-2.0
/*
 * DAMON Code for Virtual Address Spaces
 *
 * Author: SeongJae Park <sj@kernel.org>
 */

#define pr_fmt(fmt) "damon-va: " fmt

#include <linux/highmem.h>
#include <linux/hugetlb.h>
#include <linux/mman.h>
#include <linux/mmu_notifier.h>
#include <linux/page_idle.h>
#include <linux/pagewalk.h>
#include <linux/sched/mm.h>

#include <linux/circ_buf.h>
#include <linux/perf_event.h>
#include <linux/xarray.h>

#include "../internal.h"
#include "ops-common.h"

#ifdef CONFIG_DAMON_VADDR_KUNIT_TEST
#undef DAMON_MIN_REGION
#define DAMON_MIN_REGION 1
#endif

/*
 * 't->pid' should be the pointer to the relevant 'struct pid' having reference
 * count.  Caller must put the returned task, unless it is NULL.
 */
static inline struct task_struct *damon_get_task_struct(struct damon_target *t)
{
	return get_pid_task(t->pid, PIDTYPE_PID);
}

/*
 * Get the mm_struct of the given target
 *
 * Caller _must_ put the mm_struct after use, unless it is NULL.
 *
 * Returns the mm_struct of the target on success, NULL on failure
 */
static struct mm_struct *damon_get_mm(struct damon_target *t)
{
	struct task_struct *task;
	struct mm_struct *mm;

	task = damon_get_task_struct(t);
	if (!task)
		return NULL;

	mm = get_task_mm(task);
	put_task_struct(task);
	return mm;
}

/*
 * Functions for the initial monitoring target regions construction
 */

/*
 * Size-evenly split a region into 'nr_pieces' small regions
 *
 * Returns 0 on success, or negative error code otherwise.
 */
static int damon_va_evenly_split_region(struct damon_target *t,
		struct damon_region *r, unsigned int nr_pieces)
{
	unsigned long sz_piece;
	unsigned long min_region_sz = max(DAMON_MIN_REGION, t->min_region_sz);

	if (!r || !nr_pieces)
		return -EINVAL;

	if (nr_pieces == 1)
		return 0;

	sz_piece = ALIGN_DOWN(damon_sz_region(r) / nr_pieces, min_region_sz);

	if (!sz_piece)
		return -EINVAL;

	return damon_evenly_split_region(t, r, -1, sz_piece);
}

static unsigned long sz_range(struct damon_addr_range *r)
{
	return r->end - r->start;
}

/*
 * Find three regions separated by two biggest unmapped regions
 *
 * vma		the head vma of the target address space
 * regions	an array of three address ranges that results will be saved
 *
 * This function receives an address space and finds three regions in it which
 * separated by the two biggest unmapped regions in the space.  Please refer to
 * below comments of '__damon_va_init_regions()' function to know why this is
 * necessary.
 *
 * Returns 0 if success, or negative error code otherwise.
 */
static int __damon_va_three_regions(struct damon_target *t, struct mm_struct *mm,
				       struct damon_addr_range regions[3])
{
	struct damon_addr_range first_gap = {0}, second_gap = {0};
	VMA_ITERATOR(vmi, mm, 0);
	struct vm_area_struct *vma, *prev = NULL;
	unsigned long min_region_sz = max(DAMON_MIN_REGION, t->min_region_sz);
	unsigned long start;

	/*
	 * Find the two biggest gaps so that first_gap > second_gap > others.
	 * If this is too slow, it can be optimised to examine the maple
	 * tree gaps.
	 */
	rcu_read_lock();
	for_each_vma(vmi, vma) {
		unsigned long gap;

		if (!prev) {
			start = vma->vm_start;
			goto next;
		}
		gap = vma->vm_start - prev->vm_end;

		if (gap > sz_range(&first_gap)) {
			second_gap = first_gap;
			first_gap.start = prev->vm_end;
			first_gap.end = vma->vm_start;
		} else if (gap > sz_range(&second_gap)) {
			second_gap.start = prev->vm_end;
			second_gap.end = vma->vm_start;
		}
next:
		prev = vma;
	}
	rcu_read_unlock();

	if (!sz_range(&second_gap) || !sz_range(&first_gap)) {
		pr_warn_once("The size of the first and second gaps are %lu and %lu\n",
				sz_range(&first_gap), sz_range(&second_gap));
		return -EINVAL;
	}

	/* Sort the two biggest gaps by address */
	if (first_gap.start > second_gap.start)
		swap(first_gap, second_gap);

	/* Store the result */
	regions[0].start = ALIGN_DOWN(start, min_region_sz);
	regions[0].end = ALIGN(first_gap.start, min_region_sz);
	regions[1].start = ALIGN_DOWN(first_gap.end, min_region_sz);
	regions[1].end = ALIGN(second_gap.start, min_region_sz);
	regions[2].start = ALIGN_DOWN(second_gap.end, min_region_sz);
	regions[2].end = ALIGN(prev->vm_end, min_region_sz);

	for (int i = 0; i < 3; i++) {
		if (!sz_range(&regions[i])) {
			pr_warn_once("The size of the %dth range is %lu\n",
					i, sz_range(&regions[i]));
			return -EINVAL;
		}
		if (i > 0 && regions[i - 1].end >= regions[i].start) {
			pr_warn_once("%dth and %dth regions overlap\n", i - 1, i);
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * Get the three regions in the given target (task)
 *
 * Returns 0 on success, negative error code otherwise.
 */
static int damon_va_three_regions(struct damon_target *t,
				struct damon_addr_range regions[3])
{
	struct mm_struct *mm;
	int rc;

	mm = damon_get_mm(t);
	if (!mm)
		return -EINVAL;

	mmap_read_lock(mm);
	rc = __damon_va_three_regions(t, mm, regions);
	mmap_read_unlock(mm);

	mmput(mm);
	return rc;
}

/*
 * Initialize the monitoring target regions for the given target (task)
 *
 * t	the given target
 *
 * Because only a number of small portions of the entire address space
 * is actually mapped to the memory and accessed, monitoring the unmapped
 * regions is wasteful.  That said, because we can deal with small noises,
 * tracking every mapping is not strictly required but could even incur a high
 * overhead if the mapping frequently changes or the number of mappings is
 * high.  The adaptive regions adjustment mechanism will further help to deal
 * with the noise by simply identifying the unmapped areas as a region that
 * has no access.  Moreover, applying the real mappings that would have many
 * unmapped areas inside will make the adaptive mechanism quite complex.  That
 * said, too huge unmapped areas inside the monitoring target should be removed
 * to not take the time for the adaptive mechanism.
 *
 * For the reason, we convert the complex mappings to three distinct regions
 * that cover every mapped area of the address space.  Also the two gaps
 * between the three regions are the two biggest unmapped areas in the given
 * address space.  In detail, this function first identifies the start and the
 * end of the mappings and the two biggest unmapped areas of the address space.
 * Then, it constructs the three regions as below:
 *
 *     [mappings[0]->start, big_two_unmapped_areas[0]->start)
 *     [big_two_unmapped_areas[0]->end, big_two_unmapped_areas[1]->start)
 *     [big_two_unmapped_areas[1]->end, mappings[nr_mappings - 1]->end)
 *
 * As usual memory map of processes is as below, the gap between the heap and
 * the uppermost mmap()-ed region, and the gap between the lowermost mmap()-ed
 * region and the stack will be two biggest unmapped regions.  Because these
 * gaps are exceptionally huge areas in usual address space, excluding these
 * two biggest unmapped regions will be sufficient to make a trade-off.
 *
 *   <heap>
 *   <BIG UNMAPPED REGION 1>
 *   <uppermost mmap()-ed region>
 *   (other mmap()-ed regions and small unmapped regions)
 *   <lowermost mmap()-ed region>
 *   <BIG UNMAPPED REGION 2>
 *   <stack>
 */
static void __damon_va_init_regions(struct damon_ctx *ctx,
				     struct damon_target *t)
{
	struct damon_target *ti;
	struct damon_region *r;
	struct damon_addr_range regions[3];
	unsigned long min_region_sz = max(DAMON_MIN_REGION, t->min_region_sz);
	unsigned long sz = 0, nr_pieces;
	int i, tidx = 0;

	if (damon_va_three_regions(t, regions)) {
		damon_for_each_target(ti, ctx) {
			if (ti == t)
				break;
			tidx++;
		}
		pr_debug("Failed to get three regions of %dth target\n", tidx);
		return;
	}

	for (i = 0; i < 3; i++)
		sz += regions[i].end - regions[i].start;
	if (ctx->attrs.min_nr_regions)
		sz /= ctx->attrs.min_nr_regions;
	if (t->max_region_sz)
		sz = clamp(sz, min_region_sz, t->max_region_sz);
	else
		sz = max(sz, min_region_sz);

	/* Set the initial three regions of the target */
	for (i = 0; i < 3; i++) {
		r = damon_new_region(regions[i].start, regions[i].end);
		if (!r) {
			pr_err("%d'th init region creation failed\n", i);
			return;
		}
		damon_add_region(r, t);

		nr_pieces = (regions[i].end - regions[i].start) / sz;
		damon_va_evenly_split_region(t, r, nr_pieces);
	}
}

/* Initialize '->regions_list' of every target (task) */
static void damon_va_init_regions(struct damon_ctx *ctx)
{
	struct damon_target *t;

	damon_for_each_target(t, ctx) {
		/* the user may set the target regions as they want */
		if (!damon_nr_regions(t))
			__damon_va_init_regions(ctx, t);
	}
}

/*
 * Update regions for current memory mappings
 */
static void damon_va_update(struct damon_ctx *ctx)
{
	struct damon_addr_range three_regions[3];
	struct damon_target *t;

	damon_for_each_target(t, ctx) {
		if (damon_va_three_regions(t, three_regions))
			continue;
		damon_set_regions(t, three_regions, 3, DAMON_MIN_REGION, true);
	}
}

static int damon_mkold_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	pte_t *pte;
	spinlock_t *ptl;

	ptl = pmd_trans_huge_lock(pmd, walk->vma);
	if (ptl) {
		pmd_t pmde = pmdp_get(pmd);

		if (pmd_present(pmde))
			damon_pmdp_mkold(pmd, walk->vma, addr);
		spin_unlock(ptl);
		return 0;
	}

	pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte)
		return 0;
	if (!pte_present(ptep_get(pte)))
		goto out;
	damon_ptep_mkold(pte, walk->vma, addr);
out:
	pte_unmap_unlock(pte, ptl);
	return 0;
}

#ifdef CONFIG_HUGETLB_PAGE
static void damon_hugetlb_mkold(pte_t *pte, struct mm_struct *mm,
				struct vm_area_struct *vma, unsigned long addr)
{
	bool referenced = false;
	pte_t entry = huge_ptep_get(mm, addr, pte);
	struct folio *folio = pfn_folio(pte_pfn(entry));
	unsigned long psize = huge_page_size(hstate_vma(vma));

	folio_get(folio);

	if (pte_young(entry)) {
		referenced = true;
		entry = pte_mkold(entry);
		set_huge_pte_at(mm, addr, pte, entry, psize);
	}

	if (mmu_notifier_clear_young(mm, addr,
				     addr + huge_page_size(hstate_vma(vma))))
		referenced = true;

	if (referenced)
		folio_set_young(folio);

	folio_set_idle(folio);
	folio_put(folio);
}

static int damon_mkold_hugetlb_entry(pte_t *pte, unsigned long hmask,
				     unsigned long addr, unsigned long end,
				     struct mm_walk *walk)
{
	struct hstate *h = hstate_vma(walk->vma);
	spinlock_t *ptl;
	pte_t entry;

	ptl = huge_pte_lock(h, walk->mm, pte);
	entry = huge_ptep_get(walk->mm, addr, pte);
	if (!pte_present(entry))
		goto out;

	damon_hugetlb_mkold(pte, walk->mm, walk->vma, addr);

out:
	spin_unlock(ptl);
	return 0;
}
#else
#define damon_mkold_hugetlb_entry NULL
#endif /* CONFIG_HUGETLB_PAGE */

static const struct mm_walk_ops damon_mkold_ops = {
	.pmd_entry = damon_mkold_pmd_entry,
	.hugetlb_entry = damon_mkold_hugetlb_entry,
	.walk_lock = PGWALK_RDLOCK,
};

static void damon_va_mkold(struct mm_struct *mm, unsigned long addr)
{
	mmap_read_lock(mm);
	walk_page_range(mm, addr, addr + 1, &damon_mkold_ops, NULL);
	mmap_read_unlock(mm);
}

/*
 * Functions for the access checking of the regions
 */

static void __damon_va_basic_prepare_access_check(struct mm_struct *mm,
					struct damon_region *r)
{
	r->sampling_addr = damon_rand(r->ar.start, r->ar.end);

	damon_va_mkold(mm, r->sampling_addr);
}

static void damon_va_basic_prepare_access_checks(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct mm_struct *mm;
	struct damon_region *r;

	damon_for_each_target(t, ctx) {
		mm = damon_get_mm(t);
		if (!mm)
			continue;
		damon_for_each_region(r, t)
			__damon_va_basic_prepare_access_check(mm, r);
		mmput(mm);
	}
}

struct damon_young_walk_private {
	/* size of the folio for the access checked virtual memory address */
	unsigned long *folio_sz;
	bool young;
};

static int damon_young_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	pte_t *pte;
	pte_t ptent;
	spinlock_t *ptl;
	struct folio *folio;
	struct damon_young_walk_private *priv = walk->private;

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	ptl = pmd_trans_huge_lock(pmd, walk->vma);
	if (ptl) {
		pmd_t pmde = pmdp_get(pmd);

		if (!pmd_present(pmde))
			goto huge_out;
		folio = vm_normal_folio_pmd(walk->vma, addr, pmde);
		if (!folio)
			goto huge_out;
		if (pmd_young(pmde) || !folio_test_idle(folio) ||
					mmu_notifier_test_young(walk->mm,
						addr))
			priv->young = true;
		*priv->folio_sz = HPAGE_PMD_SIZE;
huge_out:
		spin_unlock(ptl);
		return 0;
	}
#endif	/* CONFIG_TRANSPARENT_HUGEPAGE */

	pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte)
		return 0;
	ptent = ptep_get(pte);
	if (!pte_present(ptent))
		goto out;
	folio = vm_normal_folio(walk->vma, addr, ptent);
	if (!folio)
		goto out;
	if (pte_young(ptent) || !folio_test_idle(folio) ||
			mmu_notifier_test_young(walk->mm, addr))
		priv->young = true;
	*priv->folio_sz = folio_size(folio);
out:
	pte_unmap_unlock(pte, ptl);
	return 0;
}

#ifdef CONFIG_HUGETLB_PAGE
static int damon_young_hugetlb_entry(pte_t *pte, unsigned long hmask,
				     unsigned long addr, unsigned long end,
				     struct mm_walk *walk)
{
	struct damon_young_walk_private *priv = walk->private;
	struct hstate *h = hstate_vma(walk->vma);
	struct folio *folio;
	spinlock_t *ptl;
	pte_t entry;

	ptl = huge_pte_lock(h, walk->mm, pte);
	entry = huge_ptep_get(walk->mm, addr, pte);
	if (!pte_present(entry))
		goto out;

	folio = pfn_folio(pte_pfn(entry));
	folio_get(folio);

	if (pte_young(entry) || !folio_test_idle(folio) ||
	    mmu_notifier_test_young(walk->mm, addr))
		priv->young = true;
	*priv->folio_sz = huge_page_size(h);

	folio_put(folio);

out:
	spin_unlock(ptl);
	return 0;
}
#else
#define damon_young_hugetlb_entry NULL
#endif /* CONFIG_HUGETLB_PAGE */

static const struct mm_walk_ops damon_young_ops = {
	.pmd_entry = damon_young_pmd_entry,
	.hugetlb_entry = damon_young_hugetlb_entry,
	.walk_lock = PGWALK_RDLOCK,
};

static bool damon_va_young(struct mm_struct *mm, unsigned long addr,
		unsigned long *folio_sz)
{
	struct damon_young_walk_private arg = {
		.folio_sz = folio_sz,
		.young = false,
	};

	mmap_read_lock(mm);
	walk_page_range(mm, addr, addr + 1, &damon_young_ops, &arg);
	mmap_read_unlock(mm);
	return arg.young;
}

/*
 * Check whether the region was accessed after the last preparation
 *
 * mm	'mm_struct' for the given virtual address space
 * r	the region to be checked
 */
static void __damon_va_basic_check_access(struct mm_struct *mm,
				struct damon_region *r, bool same_target,
				struct damon_attrs *attrs)
{
	static unsigned long last_addr;
	static unsigned long last_folio_sz = PAGE_SIZE;
	static bool last_accessed;

	if (!mm) {
		damon_update_region_access_rate(r, false, attrs);
		return;
	}

	/* If the region is in the last checked page, reuse the result */
	if (same_target && (ALIGN_DOWN(last_addr, last_folio_sz) ==
				ALIGN_DOWN(r->sampling_addr, last_folio_sz))) {
		damon_update_region_access_rate(r, last_accessed, attrs);
		return;
	}

	last_accessed = damon_va_young(mm, r->sampling_addr, &last_folio_sz);
	damon_update_region_access_rate(r, last_accessed, attrs);

	last_addr = r->sampling_addr;
}

static unsigned int damon_va_basic_check_accesses(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct mm_struct *mm;
	struct damon_region *r;
	unsigned int max_nr_accesses = 0;
	bool same_target;

	damon_for_each_target(t, ctx) {
		mm = damon_get_mm(t);
		same_target = false;
		damon_for_each_region(r, t) {
			__damon_va_basic_check_access(mm, r, same_target,
					&ctx->attrs);
			max_nr_accesses = max(r->nr_accesses, max_nr_accesses);
			same_target = true;
		}
		if (mm)
			mmput(mm);
	}

	return max_nr_accesses;
}

static bool damos_va_filter_young_match(struct damos_filter *filter,
		struct folio *folio, struct vm_area_struct *vma,
		unsigned long addr, pte_t *ptep, pmd_t *pmdp)
{
	bool young = false;

	if (ptep)
		young = pte_young(ptep_get(ptep));
	else if (pmdp)
		young = pmd_young(pmdp_get(pmdp));

	young = young || !folio_test_idle(folio) ||
		mmu_notifier_test_young(vma->vm_mm, addr);

	if (young && ptep)
		damon_ptep_mkold(ptep, vma, addr);
	else if (young && pmdp)
		damon_pmdp_mkold(pmdp, vma, addr);

	return young == filter->matching;
}

static bool damos_va_filter_out(struct damos *scheme, struct folio *folio,
		struct vm_area_struct *vma, unsigned long addr,
		pte_t *ptep, pmd_t *pmdp)
{
	struct damos_filter *filter;
	bool matched;

	if (scheme->core_filters_allowed)
		return false;

	damos_for_each_ops_filter(filter, scheme) {
		/*
		 * damos_folio_filter_match checks the young filter by doing an
		 * rmap on the folio to find its page table. However, being the
		 * vaddr scheme, we have direct access to the page tables, so
		 * use that instead.
		 */
		if (filter->type == DAMOS_FILTER_TYPE_YOUNG)
			matched = damos_va_filter_young_match(filter, folio,
				vma, addr, ptep, pmdp);
		else
			matched = damos_folio_filter_match(filter, folio);

		if (matched)
			return !filter->allow;
	}
	return scheme->ops_filters_default_reject;
}

struct damos_va_migrate_private {
	struct list_head *migration_lists;
	struct damos *scheme;
};

/*
 * Place the given folio in the migration_list corresponding to where the folio
 * should be migrated.
 *
 * The algorithm used here is similar to weighted_interleave_nid()
 */
static void damos_va_migrate_dests_add(struct folio *folio,
		struct vm_area_struct *vma, unsigned long addr,
		struct damos_migrate_dests *dests,
		struct list_head *migration_lists)
{
	pgoff_t ilx;
	int order;
	unsigned int target;
	unsigned int weight_total = 0;
	int i;

	/*
	 * If dests is empty, there is only one migration list corresponding
	 * to s->target_nid.
	 */
	if (!dests->nr_dests) {
		i = 0;
		goto isolate;
	}

	order = folio_order(folio);
	ilx = vma->vm_pgoff >> order;
	ilx += (addr - vma->vm_start) >> (PAGE_SHIFT + order);

	for (i = 0; i < dests->nr_dests; i++)
		weight_total += dests->weight_arr[i];

	/* If the total weights are somehow 0, don't migrate at all */
	if (!weight_total)
		return;

	target = ilx % weight_total;
	for (i = 0; i < dests->nr_dests; i++) {
		if (target < dests->weight_arr[i])
			break;
		target -= dests->weight_arr[i];
	}

	/* If the folio is already in the right node, don't do anything */
	if (folio_nid(folio) == dests->node_id_arr[i])
		return;

isolate:
	if (!folio_isolate_lru(folio))
		return;

	list_add(&folio->lru, &migration_lists[i]);
}

static int damos_va_migrate_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	struct damos_va_migrate_private *priv = walk->private;
	struct list_head *migration_lists = priv->migration_lists;
	struct damos *s = priv->scheme;
	struct damos_migrate_dests *dests = &s->migrate_dests;
	struct folio *folio;
	spinlock_t *ptl;
	pte_t *start_pte, *pte, ptent;
	int nr;

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	ptl = pmd_trans_huge_lock(pmd, walk->vma);
	if (ptl) {
		pmd_t pmde = pmdp_get(pmd);

		if (!pmd_present(pmde))
			goto huge_out;
		folio = vm_normal_folio_pmd(walk->vma, addr, pmde);
		if (!folio)
			goto huge_out;
		if (damos_va_filter_out(s, folio, walk->vma, addr, NULL, pmd))
			goto huge_out;
		damos_va_migrate_dests_add(folio, walk->vma, addr, dests,
				migration_lists);
huge_out:
		spin_unlock(ptl);
		return 0;
	}
#endif	/* CONFIG_TRANSPARENT_HUGEPAGE */

	start_pte = pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte)
		return 0;

	for (; addr < next; pte += nr, addr += nr * PAGE_SIZE) {
		nr = 1;
		ptent = ptep_get(pte);

		if (pte_none(ptent) || !pte_present(ptent))
			continue;
		folio = vm_normal_folio(walk->vma, addr, ptent);
		if (!folio)
			continue;
		if (damos_va_filter_out(s, folio, walk->vma, addr, pte, NULL))
			continue;
		damos_va_migrate_dests_add(folio, walk->vma, addr, dests,
				migration_lists);
		nr = folio_nr_pages(folio);
	}
	pte_unmap_unlock(start_pte, ptl);
	return 0;
}

/*
 * Functions for the target validity check and cleanup
 */

static bool damon_va_target_valid(struct damon_target *t)
{
	struct task_struct *task;

	task = damon_get_task_struct(t);
	if (task) {
		put_task_struct(task);
		return true;
	}

	return false;
}

static void damon_va_cleanup_target(struct damon_target *t)
{
	put_pid(t->pid);
}

#ifndef CONFIG_ADVISE_SYSCALLS
static unsigned long damos_madvise(struct damon_target *target,
		struct damon_region *r, int behavior)
{
	return 0;
}
#else
static unsigned long damos_madvise(struct damon_target *target,
		struct damon_region *r, int behavior)
{
	struct mm_struct *mm;
	unsigned long start = PAGE_ALIGN(r->ar.start);
	unsigned long len = PAGE_ALIGN(damon_sz_region(r));
	unsigned long applied;

	mm = damon_get_mm(target);
	if (!mm)
		return 0;

	applied = do_madvise(mm, start, len, behavior) ? 0 : len;
	mmput(mm);

	return applied;
}
#endif	/* CONFIG_ADVISE_SYSCALLS */

static unsigned long damos_va_migrate(struct damon_target *target,
		struct damon_region *r, struct damos *s,
		unsigned long *sz_filter_passed)
{
	LIST_HEAD(folio_list);
	struct damos_va_migrate_private priv;
	struct mm_struct *mm;
	int nr_dests;
	int nid;
	bool use_target_nid;
	unsigned long applied = 0;
	struct damos_migrate_dests *dests = &s->migrate_dests;
	struct mm_walk_ops walk_ops = {
		.pmd_entry = damos_va_migrate_pmd_entry,
		.pte_entry = NULL,
		.walk_lock = PGWALK_RDLOCK,
	};

	use_target_nid = dests->nr_dests == 0;
	nr_dests = use_target_nid ? 1 : dests->nr_dests;
	priv.scheme = s;
	priv.migration_lists = kmalloc_array(nr_dests,
		sizeof(*priv.migration_lists), GFP_KERNEL);
	if (!priv.migration_lists)
		return 0;

	for (int i = 0; i < nr_dests; i++)
		INIT_LIST_HEAD(&priv.migration_lists[i]);


	mm = damon_get_mm(target);
	if (!mm)
		goto free_lists;

	mmap_read_lock(mm);
	walk_page_range(mm, r->ar.start, r->ar.end, &walk_ops, &priv);
	mmap_read_unlock(mm);
	mmput(mm);

	for (int i = 0; i < nr_dests; i++) {
		nid = use_target_nid ? s->target_nid : dests->node_id_arr[i];
		applied += damon_migrate_pages(&priv.migration_lists[i], nid);
		cond_resched();
	}

free_lists:
	kfree(priv.migration_lists);
	return applied * PAGE_SIZE;
}

struct damos_va_stat_private {
	struct damos *scheme;
	unsigned long *sz_filter_passed;
};

static inline bool damos_va_invalid_folio(struct folio *folio,
		struct damos *s)
{
	return !folio || folio == s->last_applied;
}

static int damos_va_stat_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	struct damos_va_stat_private *priv = walk->private;
	struct damos *s = priv->scheme;
	unsigned long *sz_filter_passed = priv->sz_filter_passed;
	struct vm_area_struct *vma = walk->vma;
	struct folio *folio;
	spinlock_t *ptl;
	pte_t *start_pte, *pte, ptent;
	int nr;

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	ptl = pmd_trans_huge_lock(pmd, vma);
	if (ptl) {
		pmd_t pmde = pmdp_get(pmd);

		if (!pmd_present(pmde))
			goto huge_unlock;

		folio = vm_normal_folio_pmd(vma, addr, pmde);

		if (damos_va_invalid_folio(folio, s))
			goto huge_unlock;

		if (!damos_va_filter_out(s, folio, vma, addr, NULL, pmd))
			*sz_filter_passed += folio_size(folio);
		s->last_applied = folio;

huge_unlock:
		spin_unlock(ptl);
		return 0;
	}
#endif
	start_pte = pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	if (!start_pte)
		return 0;

	for (; addr < next; pte += nr, addr += nr * PAGE_SIZE) {
		nr = 1;
		ptent = ptep_get(pte);

		if (pte_none(ptent) || !pte_present(ptent))
			continue;

		folio = vm_normal_folio(vma, addr, ptent);

		if (damos_va_invalid_folio(folio, s))
			continue;

		if (!damos_va_filter_out(s, folio, vma, addr, pte, NULL))
			*sz_filter_passed += folio_size(folio);
		nr = folio_nr_pages(folio);
		s->last_applied = folio;
	}
	pte_unmap_unlock(start_pte, ptl);
	return 0;
}

static unsigned long damos_va_stat(struct damon_target *target,
		struct damon_region *r, struct damos *s,
		unsigned long *sz_filter_passed)
{
	struct damos_va_stat_private priv;
	struct mm_struct *mm;
	struct mm_walk_ops walk_ops = {
		.pmd_entry = damos_va_stat_pmd_entry,
		.walk_lock = PGWALK_RDLOCK,
	};

	priv.scheme = s;
	priv.sz_filter_passed = sz_filter_passed;

	if (!damos_ops_has_filter(s))
		return 0;

	mm = damon_get_mm(target);
	if (!mm)
		return 0;

	mmap_read_lock(mm);
	walk_page_range(mm, r->ar.start, r->ar.end, &walk_ops, &priv);
	mmap_read_unlock(mm);
	mmput(mm);
	return 0;
}

static unsigned long damon_va_apply_scheme(struct damon_ctx *ctx,
		struct damon_target *t, struct damon_region *r,
		struct damos *scheme, unsigned long *sz_filter_passed)
{
	int madv_action;

	switch (scheme->action) {
	case DAMOS_WILLNEED:
		madv_action = MADV_WILLNEED;
		break;
	case DAMOS_COLD:
		madv_action = MADV_COLD;
		break;
	case DAMOS_PAGEOUT:
		madv_action = MADV_PAGEOUT;
		break;
	case DAMOS_HUGEPAGE:
		madv_action = MADV_HUGEPAGE;
		break;
	case DAMOS_NOHUGEPAGE:
		madv_action = MADV_NOHUGEPAGE;
		break;
	case DAMOS_MIGRATE_HOT:
	case DAMOS_MIGRATE_COLD:
		return damos_va_migrate(t, r, scheme, sz_filter_passed);
	case DAMOS_STAT:
		return damos_va_stat(t, r, scheme, sz_filter_passed);
	default:
		/*
		 * DAMOS actions that are not yet supported by 'vaddr'.
		 */
		return 0;
	}

	return damos_madvise(t, r, madv_action);
}

static int damon_va_scheme_score(struct damon_ctx *context,
		struct damon_target *t, struct damon_region *r,
		struct damos *scheme)
{

	switch (scheme->action) {
	case DAMOS_PAGEOUT:
		return damon_cold_score(context, r, scheme);
	case DAMOS_MIGRATE_HOT:
		return damon_hot_score(context, r, scheme);
	case DAMOS_MIGRATE_COLD:
		return damon_cold_score(context, r, scheme);
	default:
		break;
	}

	return DAMOS_MAX_SCORE;
}

#ifdef CONFIG_PERF_EVENTS

struct damon_perf_record {
	u32 pid;
	u32 tid;
	u64 addr;
	u64 phys_addr;
};

struct damon_perf_buffer {
	struct damon_perf_record *records;
	unsigned long head;
	unsigned long tail;
	unsigned long size;
};

struct damon_vaddr_histogram_per_target {
	struct xarray accesses;
};

static void damon_vaddr_histogram_init(struct damon_vaddr_histogram *histogram)
{
	xa_init(&histogram->targets);
}

static void damon_vaddr_histogram_add(struct damon_vaddr_histogram *histogram, u32 pid,
						u64 vaddr)
{
	struct damon_vaddr_histogram_per_target *target;
	unsigned long nr_accesses;

	while (!(target = xa_load(&histogram->targets, pid))) {
		target = kmalloc(sizeof(*target), GFP_KERNEL);
		if (!target)
			return;

		xa_init(&target->accesses);

		if (xa_err(xa_store(&histogram->targets, pid, target, GFP_KERNEL))) {
			pr_warn_once("Failed to store target histogram\n");
			kfree(target);
			return;
		}
	}

	nr_accesses = xa_to_value(xa_load(&target->accesses, vaddr));
	xa_store(&target->accesses, vaddr, xa_mk_value(nr_accesses + 1), GFP_KERNEL);
}

static unsigned long damon_vaddr_histogram_count(struct damon_vaddr_histogram *histogram,
							u32 pid, u64 vaddr)
{
	struct damon_vaddr_histogram_per_target *target;
	unsigned long nr_accesses = 0;

	target = xa_load(&histogram->targets, pid);
	if (target)
		nr_accesses = xa_to_value(xa_load(&target->accesses, vaddr));

	return nr_accesses;
}

static void damon_vaddr_histogram_destroy(struct damon_vaddr_histogram *histogram)
{
	unsigned long index;
	struct damon_vaddr_histogram_per_target *target;

	xa_for_each(&histogram->targets, index, target)
		xa_destroy(&target->accesses);

	xa_destroy(&histogram->targets);
}

static void damon_perf_overflow(struct perf_event *perf_event, struct perf_sample_data *data,
				struct pt_regs *regs)
{
	struct damon_perf_event *event = perf_event->overflow_handler_context;
	struct damon_perf *perf = event->priv;
	struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, smp_processor_id());
	unsigned long head = buffer->head;
	unsigned long tail = READ_ONCE(buffer->tail);

	if (CIRC_SPACE(head, tail, buffer->size) >= 1) {
		struct damon_perf_record *record = &buffer->records[head];

		record->pid = task_tgid_nr(current);
		record->tid = task_pid_nr(current);
		record->addr = data->addr;
		record->phys_addr = data->phys_addr;

		smp_store_release(&buffer->head, (head + 1) & (buffer->size - 1));
	}
}

#define DAMON_PERF_MAX_RECORDS (1UL << 20)
#define DAMON_PERF_INIT_RECORDS (1UL << 15)

static int damon_perf_init(struct damon_ctx *ctx, struct damon_perf_event *event)
{
	struct damon_perf *perf;
	struct perf_event_attr attr = {
		.type = PERF_TYPE_RAW,
		.size = sizeof(attr),
		.type = event->attr.type,
		.config = event->attr.config,
		.config1 = event->attr.config1,
		.config2 = event->attr.config2,
		.sample_freq = event->attr.sample_freq,
		.freq = 1,
		.sample_type = PERF_SAMPLE_TIME | PERF_SAMPLE_ADDR |
			PERF_SAMPLE_PERIOD | PERF_SAMPLE_DATA_SRC |
			(event->attr.sample_phys_addr ? PERF_SAMPLE_PHYS_ADDR : 0) |
			PERF_SAMPLE_WEIGHT_STRUCT,
		.precise_ip = 3,
		.pinned = 1,
		.disabled = 1,
	};
	int cpu;
	int err = -ENOMEM;
	bool found = false;

	perf = kzalloc(sizeof(*perf), GFP_KERNEL);
	if (!perf)
		return -ENOMEM;

	perf->event = alloc_percpu(typeof(*perf->event));
	if (!perf->event)
		goto free_percpu;

	perf->buffer = alloc_percpu(typeof(*perf->buffer));
	if (!perf->buffer)
		goto free_percpu;

	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event;
		struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, cpu);

		perf_event = perf_event_create_kernel_counter(&attr, cpu, NULL,
				damon_perf_overflow, event);
		if (IS_ERR(perf_event)) {
			err = PTR_ERR(perf_event);
			if (err == -ENODEV)
				continue;
			pr_err("perf event create on CPU %d failed with %d\n", cpu, err);
			goto free_for_each_cpu;
		}
		found = true;
		*per_cpu_ptr(perf->event, cpu) = perf_event;

		buffer->size = DAMON_PERF_INIT_RECORDS;
		buffer->records = kvcalloc_node(buffer->size, sizeof(buffer->records[0]),
						GFP_KERNEL, cpu_to_node(cpu));
		if (!buffer->records)
			goto free_for_each_cpu;
	}
	event->priv = perf;

	return found ? 0 : -ENODEV;

free_for_each_cpu:
	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event = per_cpu(*perf->event, cpu);
		struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, cpu);

		if (perf_event)
			perf_event_release_kernel(perf_event);
		kvfree(buffer->records);
	}
free_percpu:
	free_percpu(perf->event);
	free_percpu(perf->buffer);
	kfree(perf);

	return err;
}

static void damon_perf_cleanup(struct damon_ctx *ctx, struct damon_perf_event *event)
{
	struct damon_perf *perf = event->priv;
	int cpu;

	if (!perf)
		return;

	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event = per_cpu(*perf->event, cpu);
		struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, cpu);

		if (!perf_event)
			continue;
		perf_event_disable(perf_event);
		perf_event_release_kernel(perf_event);
		kvfree(buffer->records);
	}
	free_percpu(perf->event);
	free_percpu(perf->buffer);
	kfree(perf);
	event->priv = NULL;
}

void damon_perf_prepare_access_checks(struct damon_ctx *ctx,
		struct damon_perf_event *event)
{
	struct damon_perf *perf = event->priv;
	int cpu;

	if (!perf)
		return;

	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event = per_cpu(*perf->event, cpu);

		if (perf_event)
			perf_event_enable(perf_event);
	}
}

static void damon_va_perf_check_accesses_by_vaddr(struct damon_ctx *ctx,
		struct damon_perf_event *event)
{
	struct damon_perf *perf = event->priv;
	struct damon_target *t;
	int cpu;
	unsigned int tidx = 0;

	if (!perf)
		return;

	damon_vaddr_histogram_init(&perf->vaddr_histogram);

	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event = per_cpu(*perf->event, cpu);
		struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, cpu);
		unsigned long head, tail, count, i;

		if (!perf_event)
			continue;

		perf_event_disable(perf_event);

		head = smp_load_acquire(&buffer->head);
		tail = buffer->tail;
		count = CIRC_CNT(head, tail, buffer->size);

		for (i = 0; i < count; i++) {
			struct damon_perf_record *record =
				&buffer->records[(tail + i) & (buffer->size - 1)];

			damon_vaddr_histogram_add(&perf->vaddr_histogram, record->pid,
							record->addr & PAGE_MASK);
		}
		smp_store_release(&buffer->tail, (tail + count) & (buffer->size - 1));

		if ((count == buffer->size - 1) && (buffer->size < DAMON_PERF_MAX_RECORDS)) {
			void *new_records = kvcalloc_node(buffer->size * 2,
					sizeof(buffer->records[0]), GFP_KERNEL,
					cpu_to_node(cpu));

			if (new_records) {
				kvfree(buffer->records);
				buffer->records = new_records;
				buffer->head = 0;
				buffer->tail = 0;
				buffer->size *= 2;
			}
		}
	}

	damon_for_each_target(t, ctx) {
		struct damon_region *r;
		u32 pid = pid_nr(t->pid);
		unsigned int nr_accessed = 0;

		damon_for_each_region(r, t) {
			unsigned long addr;

			if (r->accessed)
				continue;

			for (addr = r->ar.start; addr < r->ar.end; addr += PAGE_SIZE) {
				if (damon_vaddr_histogram_count(&perf->vaddr_histogram, pid,
									addr & PAGE_MASK)) {
					r->accessed = true;
					nr_accessed++;
					break;
				}
			}
		}
		tidx++;
	}

	damon_vaddr_histogram_destroy(&perf->vaddr_histogram);
}

struct damon_paddr_walk {
	struct damon_paddr_histogram *histogram;
	bool accessed;
};

static void damon_paddr_histogram_add(struct damon_paddr_histogram *histogram, u64 paddr);
static const struct mm_walk_ops damon_paddr_ops;

void damon_perf_populate_paddr_histogram(struct damon_ctx *ctx, struct damon_perf_event *event)
{
	struct damon_perf *perf = event->priv;
	int cpu;

	if (!perf)
		return;

	for_each_possible_cpu(cpu) {
		struct perf_event *perf_event = per_cpu(*perf->event, cpu);
		struct damon_perf_buffer *buffer = per_cpu_ptr(perf->buffer, cpu);
		unsigned long head, tail, count, i;

		if (!perf_event)
			continue;

		perf_event_disable(perf_event);

		head = smp_load_acquire(&buffer->head);
		tail = buffer->tail;
		count = CIRC_CNT(head, tail, buffer->size);

		for (i = 0; i < count; i++) {
			struct damon_perf_record *record =
				&buffer->records[(tail + i) & (buffer->size - 1)];

			damon_paddr_histogram_add(&perf->paddr_histogram,
							record->phys_addr & PAGE_MASK);
		}
		smp_store_release(&buffer->tail, (tail + count) & (buffer->size - 1));

		if ((count == buffer->size - 1) && (buffer->size < DAMON_PERF_MAX_RECORDS)) {
			void *new_records = kvcalloc_node(buffer->size * 2,
					sizeof(buffer->records[0]), GFP_KERNEL,
					cpu_to_node(cpu));

			if (new_records) {
				kvfree(buffer->records);
				buffer->records = new_records;
				buffer->head = 0;
				buffer->tail = 0;
				buffer->size *= 2;
			}
		}
	}
}

static void damon_va_perf_check_accesses_by_paddr(struct damon_ctx *ctx,
		struct damon_perf_event *event)
{
	struct damon_perf *perf = event->priv;
	struct damon_target *t;
	unsigned int tidx = 0;

	if (!perf)
		return;

	damon_paddr_histogram_init(&perf->paddr_histogram);

	damon_perf_populate_paddr_histogram(ctx, event);

	damon_for_each_target(t, ctx) {
		struct damon_region *r;
		struct mm_struct *mm = damon_get_mm(t);
		unsigned int nr_accessed = 0;

		if (!mm)
			continue;

		mmap_read_lock(mm);
		damon_for_each_region(r, t) {
			struct damon_paddr_walk walk_private = {
				.histogram = &perf->paddr_histogram,
				.accessed = false,
			};

			if (r->accessed)
				continue;

			walk_page_range(mm, r->ar.start, r->ar.end, &damon_paddr_ops,
					&walk_private);
			if (walk_private.accessed) {
				r->accessed = true;
				nr_accessed++;
			}
		}
		mmap_read_unlock(mm);
		mmput(mm);
		tidx++;
	}

	damon_paddr_histogram_destroy(&perf->paddr_histogram);
}

static void damon_va_perf_check_accesses(struct damon_ctx *ctx, struct damon_perf_event *event)
{
	if (event->attr.sample_phys_addr)
		return damon_va_perf_check_accesses_by_paddr(ctx, event);
	else
		return damon_va_perf_check_accesses_by_vaddr(ctx, event);
}

void damon_paddr_histogram_init(struct damon_paddr_histogram *histogram)
{
	xa_init(&histogram->accesses);
}

static void damon_paddr_histogram_add(struct damon_paddr_histogram *histogram,
						u64 paddr)
{
	unsigned long nr_accesses;

	nr_accesses = xa_to_value(xa_load(&histogram->accesses, paddr));
	xa_store(&histogram->accesses, paddr, xa_mk_value(nr_accesses + 1), GFP_KERNEL);
}

unsigned long damon_paddr_histogram_count(struct damon_paddr_histogram *histogram, u64 paddr)
{
	return xa_to_value(xa_load(&histogram->accesses, paddr));
}

void damon_paddr_histogram_destroy(struct damon_paddr_histogram *histogram)
{
	xa_destroy(&histogram->accesses);
}

static int damon_paddr_pmd_entry(pmd_t *pmd, unsigned long addr,
		unsigned long next, struct mm_walk *walk)
{
	pte_t *pte;
	spinlock_t *ptl;
	struct damon_paddr_walk *paddr_walk = walk->private;

	ptl = pmd_trans_huge_lock(pmd, walk->vma);
	if (ptl) {
		pmd_t pmde = pmdp_get(pmd);

		if (pmd_present(pmde)) {
			for (; addr < next && !paddr_walk->accessed; addr += PAGE_SIZE) {
				u64 frame = pmd_pfn(pmde) +
					((addr & ~HPAGE_PMD_MASK) >> PAGE_SHIFT);

				if (damon_paddr_histogram_count(paddr_walk->histogram,
						PFN_PHYS(frame))) {
					paddr_walk->accessed = true;
					break;
				}
			}
		}
		spin_unlock(ptl);
		return paddr_walk->accessed ? 1 : 0;
	}

	pte = pte_offset_map_lock(walk->mm, pmd, addr, &ptl);
	if (!pte) {
		walk->action = ACTION_AGAIN;
		return 0;
	}

	for (; addr < next && !paddr_walk->accessed; pte++, addr += PAGE_SIZE) {
		pte_t ptent = ptep_get(pte);

		if (pte_present(ptent)) {
			if (damon_paddr_histogram_count(paddr_walk->histogram,
					PFN_PHYS(pte_pfn(ptent)))) {
				paddr_walk->accessed = true;
			}
		}
	}

	pte_unmap_unlock(pte - 1, ptl);

	return paddr_walk->accessed ? 1 : 0;
}

#ifdef CONFIG_HUGETLB_PAGE
static int damon_paddr_hugetlb_entry(pte_t *pte, unsigned long hmask,
		unsigned long addr, unsigned long end,
		struct mm_walk *walk)
{
	struct damon_paddr_walk *paddr_walk = walk->private;
	pte_t entry = huge_ptep_get(walk->mm, addr, pte);

	if (pte_present(entry)) {
		for (; addr < end; addr += PAGE_SIZE) {
			u64 frame = pte_pfn(entry) + ((addr & ~hmask) >> PAGE_SHIFT);

			if (damon_paddr_histogram_count(paddr_walk->histogram,
						PFN_PHYS(frame))) {
				paddr_walk->accessed = true;
				break;
			}
		}
	}

	return paddr_walk->accessed ? 1 : 0;
}
#else
#define damon_perf_hugetlb_entry NULL
#endif /* CONFIG_HUGETLB_PAGE */

static const struct mm_walk_ops damon_paddr_ops = {
	.pmd_entry = damon_paddr_pmd_entry,
	.hugetlb_entry = damon_paddr_hugetlb_entry,
	.walk_lock = PGWALK_RDLOCK,
};

#else

static inline int damon_perf_init(struct damon_ctx *ctx, struct damon_perf_event *event)
{
	return 0;
}

static inline void damon_perf_cleanup(struct damon_ctx *ctx, struct damon_perf_event *event)
{
}

static void damon_va_perf_check_accesses(struct damon_ctx *ctx, struct damon_perf_event *event)
{
}

#endif /* CONFIG_PERF_EVENTS */

void damon_ops_init(struct damon_ctx *ctx)
{
	struct damon_perf_event *event, *next;
	int err = 0;

	list_for_each_entry_safe(event, next, &ctx->perf_events, list) {
		err = damon_perf_init(ctx, event);
		if (err) {
			list_del(&event->list);
			kfree(event);
		}
	}
}

static void damon_va_init(struct damon_ctx *ctx)
{
	damon_ops_init(ctx);

	if (ctx->ops.id == DAMON_OPS_VADDR)
		damon_va_init_regions(ctx);
}

static void damon_va_prepare_access_checks(struct damon_ctx *ctx)
{
	struct damon_perf_event *event;

	if (list_empty(&ctx->perf_events))
		return damon_va_basic_prepare_access_checks(ctx);

	list_for_each_entry(event, &ctx->perf_events, list) {
		damon_perf_prepare_access_checks(ctx, event);
	}
}

static unsigned int damon_va_check_accesses(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct damon_perf_event *event;
	unsigned int max_nr_accesses = 0;

	if (list_empty(&ctx->perf_events))
		return damon_va_basic_check_accesses(ctx);

	damon_for_each_target(t, ctx) {
		struct damon_region *r;

		damon_for_each_region(r, t)
			r->accessed = false;
	}

	list_for_each_entry(event, &ctx->perf_events, list) {
		damon_va_perf_check_accesses(ctx, event);
	}

	damon_for_each_target(t, ctx) {
		struct damon_region *r;

		damon_for_each_region(r, t) {
			damon_update_region_access_rate(r, r->accessed, &ctx->attrs);
			max_nr_accesses = max(r->nr_accesses, max_nr_accesses);
		}
	}

	return max_nr_accesses;
}

void damon_ops_cleanup(struct damon_ctx *ctx)
{
	struct damon_perf_event *event;

	list_for_each_entry(event, &ctx->perf_events, list) {
		damon_perf_cleanup(ctx, event);
	}
}

static int __init damon_va_initcall(void)
{
	struct damon_operations ops = {
		.id = DAMON_OPS_VADDR,
		.init = damon_va_init,
		.update = damon_va_update,
		.prepare_access_checks = damon_va_prepare_access_checks,
		.check_accesses = damon_va_check_accesses,
		.target_valid = damon_va_target_valid,
		.cleanup_target = damon_va_cleanup_target,
		.cleanup = damon_ops_cleanup,
		.apply_scheme = damon_va_apply_scheme,
		.get_scheme_score = damon_va_scheme_score,
	};
	/* ops for fixed virtual address ranges */
	struct damon_operations ops_fvaddr = ops;
	int err;

	/* Don't set the monitoring target regions for the entire mapping */
	ops_fvaddr.id = DAMON_OPS_FVADDR;
	ops_fvaddr.update = NULL;

	err = damon_register_ops(&ops);
	if (err)
		return err;
	return damon_register_ops(&ops_fvaddr);
};

subsys_initcall(damon_va_initcall);

#include "tests/vaddr-kunit.h"
