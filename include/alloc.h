#ifndef ALLOC_H
#define ALLOC_H

#include <libliftoff.h>
#include <time.h>

struct liftoff_alloc_strategy {
	/**
	 * alloc - algorithm for allocating layers to DRM planes
	 *
	 * Given the output and initialized result,
	*/
	int (*alloc)(struct liftoff_output *output,
		     struct alloc_result *result);

	int (*reuse)(struct liftoff_output *output,
		     drmModeAtomicReq *req, uint32_t flags);
};

extern const struct liftoff_alloc_strategy alloc_overlay_strategy;

#endif
