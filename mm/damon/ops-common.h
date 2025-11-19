/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common Code for Data Access Monitoring
 *
 * Author: SeongJae Park <sj@kernel.org>
 */

#include <linux/damon.h>

struct folio *damon_get_folio(unsigned long pfn);

void damon_ptep_mkold(pte_t *pte, struct vm_area_struct *vma, unsigned long addr);
void damon_pmdp_mkold(pmd_t *pmd, struct vm_area_struct *vma, unsigned long addr);
void damon_folio_mkold(struct folio *folio);
bool damon_folio_young(struct folio *folio);

int damon_cold_score(struct damon_ctx *c, struct damon_region *r,
			struct damos *s);
int damon_hot_score(struct damon_ctx *c, struct damon_region *r,
			struct damos *s);

bool damos_folio_filter_match(struct damos_filter *filter, struct folio *folio);
unsigned long damon_migrate_pages(struct list_head *folio_list, int target_nid);

bool damos_ops_has_filter(struct damos *s);

#ifdef CONFIG_PERF_EVENTS

void damon_perf_prepare_access_checks(struct damon_ctx *ctx, struct damon_perf_event *event);

struct damon_vaddr_histogram {
	struct xarray targets;
};

struct damon_paddr_histogram {
	struct xarray accesses;
};

struct damon_perf {
	struct perf_event * __percpu *event;
	struct damon_perf_buffer __percpu *buffer;
	union {
		struct damon_vaddr_histogram vaddr_histogram;
		struct damon_paddr_histogram paddr_histogram;
	};
};

void damon_paddr_histogram_init(struct damon_paddr_histogram *histogram);
unsigned long damon_paddr_histogram_count(struct damon_paddr_histogram *histogram, u64 paddr);
void damon_paddr_histogram_destroy(struct damon_paddr_histogram *histogram);

void damon_perf_populate_paddr_histogram(struct damon_ctx *ctx, struct damon_perf_event *event);

#else /* CONFIG_PERF_EVENTS */

static inline void damon_perf_prepare_access_checks(struct damon_ctx *ctx,
		struct damon_perf_event *event)
{
}

#endif /* CONFIG_PERF_EVENTS */

void damon_ops_init(struct damon_ctx *ctx);
void damon_ops_cleanup(struct damon_ctx *ctx);
