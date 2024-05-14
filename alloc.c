#define _POSIX_C_SOURCE 200112L
#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "alloc.h"
#include "log.h"
#include "pixel_format.h"
#include "private.h"

/* Plane allocation algorithm
 *
 * Goal: KMS exposes a set of hardware planes, user submitted a set of layers.
 * We want to map as many layers as possible to planes.
 *
 * However, all layers can't be mapped to any plane. There are constraints,
 * sometimes depending on driver-specific limitations or the configuration of
 * other planes.
 *
 * The only way to discover driver-specific limitations is via an atomic test
 * commit: we submit a plane configuration, and KMS replies whether it's
 * supported or not. Thus we need to incrementally build a valid configuration.
 *
 * Let's take an example with 2 planes and 3 layers. Plane 1 is only compatible
 * with layer 2 and plane 2 is only compatible with layer 3. Our algorithm will
 * discover the solution by building the mapping one plane at a time. It first
 * starts with plane 1: an atomic commit assigning layer 1 to plane 1 is
 * submitted. It fails, because this isn't supported by the driver. Then layer
 * 2 is assigned to plane 1 and the atomic test succeeds. We can go on and
 * repeat the operation with plane 2. After exploring the whole tree, we end up
 * with a valid allocation.
 *
 *
 *                    layer 1                 layer 1
 *                  +---------> failure     +---------> failure
 *                  |                       |
 *                  |                       |
 *                  |                       |
 *     +---------+  |          +---------+  |
 *     |         |  | layer 2  |         |  | layer 3   final allocation:
 *     | plane 1 +------------>+ plane 2 +--+---------> plane 1 → layer 2
 *     |         |  |          |         |              plane 2 → layer 3
 *     +---------+  |          +---------+
 *                  |
 *                  |
 *                  | layer 3
 *                  +---------> failure
 *
 *
 * Note how layer 2 isn't considered for plane 2: it's already mapped to plane
 * 1. Also note that branches are pruned as soon as an atomic test fails.
 *
 * In practice, the primary plane is treated separately. This is where layers
 * that can't be mapped to any plane (e.g. layer 1 in our example) will be
 * composited. The primary plane is the first that will be allocated, because
 * some drivers require it to be enabled in order to light up any other plane.
 * Then all other planes will be allocated, from the topmost one to the
 * bottommost one.
 *
 * The "zpos" property (which defines ordering between layers/planes) is handled
 * as a special case. If it's set on layers, it adds additional constraints on
 * their relative ordering. If two layers intersect, their relative zpos needs
 * to be preserved during plane allocation.
 *
 * Implementation-wise, the output_choose_layers function is called at each node
 * of the tree. It iterates over layers, check constraints, performs an atomic
 * test commit and calls itself recursively on the next plane.
 */

/**
 * The Underlay allocation algorithm
 * =================================
 *
 * The algorithm can be seen as an extension of the existing overlay-only
 * algorithm. Some allocation constraints were loosened to allow for underlay
 * planes, while others were added to ensure valid allocations. The DFS was also
 * reordered to prefer underlay allocations first.
 *
 * The goal of the algorithm isn't to be the most optimal. Rather, the goal is
 * to have broad support for all DRM drivers that support MPO. It may not arrive
 * at the most optimal allocation for a specific driver at the deadline, but it
 * will find *an* underlay allocation if the layers allow.
 *
 * First, to clarify some terminology:
 *
 * - The *composition plane* refers to the plane that has the *composition
 *   layer* allocated to it. In underlay, this does not have to be the PRIMARY
 *   plane. In addition, the composition layer must support alpha to allow hole-
 *   punching by the compositor. As always, there is only one composition plane.
 *
 * - A layer is *underlay-capable* iff it is visible, not force-composited, and
 *   fully opaque.
 *
 * - *Underlay planes* refer to planes *underneath* (of lower zpos than) the
 *   composition plane, with an underlay-capable layer allocated to it.
 *
 * - *Overlay planes* refer to planes *over* (of higher zpos than) the
 *   composition plane, with a visible and non-force-composited layer allocated
 *   to it.
 *
 * - Layers that are allocated to these planes are therefore referred to as
 *   composition, underlay, and overlay layers respectively.
 *
 *
 * Let's go through an example of a possible underlay allocation. Say there are
 * the following layers and DRM planes:
 *
 * Layers (in order of highest to lowest priority)
 * - Lo: ui/osd (zpos=3) (has-alpha, force-composition, overlaps Lv)
 * - Lv: video (zpos=2) (opaque)
 * - Lc: cursor (zpos=4) (has-alpha)
 * - Lp: composition layer (zpos=1) (has-alpha)
 *
 * Planes
 * - Pp: DRM_PRIMARY plane (zpos=1)
 * - Pc: DRM_CURSOR Plane (zpos=3)
 * - Po: DRM_OVERLAY Plane (zpos=2)
 *
 * The DFS tree traversed by the algorithm looks like so:
 *
 *                    Lp                 Lp
 *                 +------>Reroute*   +------>Fail(1)
 *                 |                  |
 *      +------+   |  Lv   +------+   |  Lo                 Lp
 *      |  Pp  |---------->|  Pc  |---------->Fail(2)    +------>Score=3
 *      +------+   |       +------+   |                  |
 *                 |                  |  Lc   +------+   |  Lo
 *                 |                  +------>|  Po  |---------->Fail(2)
 *                 |                          +------+
 *                 |
 *                 |                     Lo
 *                 |                  +------>Fail(2)
 *                 |                  |
 *                 | *Lp   +------+   |  Lv                 Lo
 *                 +------>|  Pc  |---------->Fail(3)    +------>Fail(2)
 *                         +------+   |                  |
 *                                    |  Lc   +------+   |  Lv
 *                                    +------>|  Po  |---------->Fail(3) Score=1
 *                                            +------+
 *
 * Final allocation:
 * - `Lv->Pp`: Video as an underlay
 * - `Lc->Pc`: Cursor as an overlay
 * - 'Lp->Po`: Composition layer assigned to DRM_OVERLAY plane
 *
 *
 * Unlike the overlay allocation algorithm, the sequence that layer->plane
 * mappings are tried matters. We start with the DRM_PRIMARY plane `Pp` by
 * trying to allocate the composition layer `Lp` to it. It succeeds, but we *do
 * not* continue this branch. Rather, we reroute it to be traversed at the end.
 * This is because underlay allocations are prioritized. If we continued with
 * `Lp->Pp`, it would prioritize overlay. Yet we cannot try it last, since this
 * is the fallback if we hit the deadline before another allocation is found.
 * (This rerouting is really an optimisation. The DFS will eventually search
 * through underlay allocations if there was no deadline.)
 *
 * Next, we continue by trying to enable `Pp` as an underlay plane. We find the
 * highest-priority, underlay-compatible layer `Lv` and try to allocate it to
 * `Pp`. It succeeds!
 *
 * Why allocate `Lv` first when `Lo` has higher priority? To guarantee an
 * underlay allocation before the deadline, we want to *only* try
 * underlay-compatible layers on `Pp`, if they exist.  If it happens that none
 * of the layers are underlay-compatible, the algorithm falls back to the
 * overlay strategy.
 *
 * Why allocate `Pp` first when it's the lowest z-pos plane? Some drivers
 * require the PRIMARY plane to be enabled first. Therefore, `Pp` needs special
 * treatment. After `Pp` has been allocated, we take a slightly different
 * approach.
 *
 * Next in the list is `Pc`, the top-most plane. Because the composition layer
 * `Lp` has not been allocated (we've tried `Lp->Pp`, but haven't allocated it),
 * we now try `Lp->Pc`. Say this fails(1), because `Lp` does not match the DRM
 * driver's cursor format requirements. Continuing down the the layer priority
 * list, we try `Lo->Pc`. This fails(2) since `Lo` is force composited. That
 * leaves `Lc->Pc`. We try it, and success!
 *
 * Why allocate `Lp` first? For hole-punching, composition is necessary. In
 * order to prioritize underlay, `Lp` needs to be allocated up front. Since the
 * planes are iterated in descending z-order (with the exception of
 * DRM_PRIMARY), all future planes will be allocated as underlay planes the
 * moment that `Lp` is allocated.
 *
 * Continuing on to `Po`, we find that `Lp` still has not been allocated. Trying
 * `Lp->Po` results in success: `Po` works as the composition plane. We then
 * realize we've allocated all the planes. The score is collected (3 in this
 * case), and the allocation saved. We also try `Lo->Po`, since `Lo` has not
 * been allocated. It results in a fail for the same reason as (2). Note that in
 * practice, this branch will be pruned, since it cannot obtain a score >3. It
 * is kept here for explanation's sake.
 *
 * This DFS branch is now exhausted, and we go back up to `Pp`. Remember that
 * `Lp->Pp` was rerouted to be traversed at the end? Well, we're at the end now!
 * Note that there's no need to try `Lp->Pp` again, since we've already done
 * that at the start. This is important, since in the event that we deadline
 * with score == 0, we can rest assured that there's a fallback option for
 * full-composition.
 *
 * With `Lp->Pp` as the root, an overlay solution is now being considered.
 * Instead of tracing out this branch, here are some key points:
 *
 * - `Lv->Pc` and `Lv->Po` both fails(3) because `Lo` overlaps it, and is force
 *   composited. An overlay allocation is not possible for `Lv`.
 *
 * - Allocating `Lp` in the overlay strategy does not +1 the score, whereas it
 *   does for the underlay strategy. This gives preference for the underlay
 *   solution, even if it offloads one less layer. It can be thought of as a +1
 *   bonus for providing a "more stable" allocation.
 *
 * - We do not try to allocate layers that have alpha to `Pp` if there exists an
 *   underlay-capable layer.
 *
 * - In practice, this DFS branch will be pruned, since the allocation score
 *   will never be >3.
 *
 *
 * In summary, the key differences in the underlay algorithm are:
 *
 * 1. The composition layer can be assigned to any DRM plane, not just
 *    DRM_PRIMARY.
 *
 * 2. Layers can be positioned under the composition plane, as long as they are
 *    underlay compatible.
 *
 * 3. Layers occluded by a force-composition layer *can* be offloaded if it's
 *    allocated plane is an underlay plane.
 *
 * 4. Z-order of underlay layers need not respect the z-order of the composition
 *    layer, nor overlay layers. However:
 *      1. Z-order of underlay planes need to be below the composition plane
 *      2. Z-order of underlay layers need to respect the z-order of other
 *         underlay layers.
 *
 * 5. If there exists an underlay-compatible layer, then only the composition
 *    layer and underlay-compatible layers will be tried with DRM_PRIMARY.
 *    Otherwise, the overlay algorithm is applied instead.
 *
 * 6. The composition layer to DRM_PRIMARY allocation is tried first, but the
 *    DFS branch rooted on it is rerouted to be traversed at the end. This,
 *    along with point 4, serves to prioritize allocations where DRM_PRIMARY is
 *    an underlay plane.
 */

/* Global data for the allocation algorithm */
struct alloc_result {
	drmModeAtomicReq *req;
	uint32_t flags;
	size_t planes_len;

	struct liftoff_layer **best;
	int best_score;

	struct timespec started_at;

	/* per-output */
	bool has_composition_layer;
	size_t non_composition_layers_len;

	/* Bitmask storing whether a plane is an underlay plane */
	uint32_t underlay_plane_mask;
};

/* Transient data, arguments for each step */
struct alloc_step {
	struct liftoff_list *plane_link; /* liftoff_plane.link */
	size_t plane_idx;

	struct liftoff_layer **alloc; /* only items up to plane_idx are valid */
	int score; /* number of allocated layers */
	int last_layer_zpos;
	int primary_layer_zpos, primary_plane_zpos;

	bool composited; /* per-output */

	uint32_t underlay_plane_mask;
	bool underlay_has_opaque_layer;

	char log_prefix[64];
};

static const int64_t NSEC_PER_SEC = 1000 * 1000 * 1000;

static int64_t
timespec_to_nsec(struct timespec ts)
{
	return (int64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

static const int64_t ALLOC_TIMEOUT_NSEC = 1000 * 1000; // 1ms

static bool
check_deadline(struct timespec start)
{
	struct timespec now;
	int64_t deadline;

	if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
		liftoff_log_errno(LIFTOFF_ERROR, "clock_gettime");
		return false;
	}

	deadline = timespec_to_nsec(start) + ALLOC_TIMEOUT_NSEC;
	return timespec_to_nsec(now) < deadline;
}

static bool
step_is_underlay_alloc(struct liftoff_layer *layer, struct alloc_step *step)
{
	struct liftoff_plane *plane;

	if (layer == NULL) {
		return false;
	}

	if (!step->underlay_has_opaque_layer) {
		return false;
	}

	plane = liftoff_container_of(step->plane_link, plane, link);

	if (step->composited) {
		/* If the composition layer is alloc'd to the PRIMARY plane,
		 * then no underlay alloc is possible.
		 *
		 * Otherwise, this step is an underlay alloc since the
		 * composition layer has been previously alloc'd to a
		 * non-primary plane, and we're alloc'ing planes in descending
		 * z-order.
		 */
		return (step->alloc[0] == NULL ||
			step->alloc[0] != layer->output->composition_layer);
	} else {
		/*
		 * If the composition layer has not been alloc'd, then no
		 * underlay alloc is possible.
		 *
		 * EXCEPT if a non-composition layer is alloc'd to the PRIMARY
		 * plane -- this is an underlay alloc.
		*/
		return (plane->type == DRM_PLANE_TYPE_PRIMARY &&
			layer != NULL &&
			layer != layer->output->composition_layer);
	}
}

static void
plane_step_init_next(struct alloc_step *step, struct alloc_step *prev,
		     struct liftoff_layer *layer)
{
	struct liftoff_layer_property *zpos_prop;
	struct liftoff_plane *plane;
	bool is_underlay;
	size_t len;

	plane = liftoff_container_of(prev->plane_link, plane, link);

	step->plane_link = prev->plane_link->next;
	step->plane_idx = prev->plane_idx + 1;
	step->alloc = prev->alloc;
	step->alloc[prev->plane_idx] = layer;
	step->underlay_has_opaque_layer = prev->underlay_has_opaque_layer;

	step->underlay_plane_mask = prev->underlay_plane_mask;
	/* Record whether this alloc is underlay */
	if (layer != NULL &&
	    layer->output->device->alloc_strategy == &alloc_underlay_strategy) {
		is_underlay = step_is_underlay_alloc(layer, prev);
		step->underlay_plane_mask |=
			((uint32_t)is_underlay << prev->plane_idx);
	}

	if (layer != NULL && layer == layer->output->composition_layer) {
		assert(!prev->composited);
		step->composited = true;
	} else {
		step->composited = prev->composited;
	}

	if (layer != NULL && layer != layer->output->composition_layer) {
		step->score = prev->score + 1;
	} else {
		step->score = prev->score;
	}

	zpos_prop = NULL;
	if (layer != NULL) {
		zpos_prop = layer_get_property(layer, "zpos");
	}
	if (zpos_prop != NULL && plane->type != DRM_PLANE_TYPE_PRIMARY) {
		step->last_layer_zpos = zpos_prop->value;
	} else {
		step->last_layer_zpos = prev->last_layer_zpos;
	}
	if (zpos_prop != NULL && plane->type == DRM_PLANE_TYPE_PRIMARY) {
		step->primary_layer_zpos = zpos_prop->value;
		step->primary_plane_zpos = plane->zpos;
	} else {
		step->primary_layer_zpos = prev->primary_layer_zpos;
		step->primary_plane_zpos = prev->primary_plane_zpos;
	}

	if (layer != NULL) {
		len = strlen(prev->log_prefix) + 2;
		if (len > sizeof(step->log_prefix) - 1) {
			len = sizeof(step->log_prefix) - 1;
		}
		memset(step->log_prefix, ' ', len);
		step->log_prefix[len] = '\0';
	} else {
		memcpy(step->log_prefix, prev->log_prefix,
		       sizeof(step->log_prefix));
	}
}

static bool
is_layer_allocated(struct alloc_step *step, struct liftoff_layer *layer)
{
	size_t i;

	/* TODO: speed this up with an array of bools indicating whether a layer
	 * has been allocated */
	for (i = 0; i < step->plane_idx; i++) {
		if (step->alloc[i] == layer) {
			return true;
		}
	}
	return false;
}

static bool
has_composited_layer_over(struct liftoff_output *output,
			  struct alloc_step *step, struct liftoff_layer *layer)
{
	struct liftoff_layer *other_layer;
	struct liftoff_layer_property *zpos_prop, *other_zpos_prop;

	zpos_prop = layer_get_property(layer, "zpos");
	if (zpos_prop == NULL) {
		return false;
	}

	liftoff_list_for_each(other_layer, &output->layers, link) {
		if (is_layer_allocated(step, other_layer)) {
			continue;
		}

		other_zpos_prop = layer_get_property(other_layer, "zpos");
		if (other_zpos_prop == NULL) {
			continue;
		}

		if (layer_intersects(layer, other_layer) &&
		    other_zpos_prop->value > zpos_prop->value) {
			return true;
		}
	}

	return false;
}

static bool
has_allocated_layer_over(struct liftoff_output *output, struct alloc_step *step,
			 struct liftoff_layer *layer)
{
	ssize_t i;
	struct liftoff_plane *other_plane;
	struct liftoff_layer *other_layer;
	struct liftoff_layer_property *zpos_prop, *other_zpos_prop;

	zpos_prop = layer_get_property(layer, "zpos");
	if (zpos_prop == NULL) {
		return false;
	}

	i = -1;
	liftoff_list_for_each(other_plane, &output->device->planes, link) {
		i++;
		if (i >= (ssize_t)step->plane_idx) {
			break;
		}
		if (other_plane->type == DRM_PLANE_TYPE_PRIMARY) {
			continue;
		}

		other_layer = step->alloc[i];
		if (other_layer == NULL) {
			continue;
		}

		other_zpos_prop = layer_get_property(other_layer, "zpos");
		if (other_zpos_prop == NULL) {
			continue;
		}

		/* Since plane zpos is descending, this means the other layer is
		 * supposed to be under but is mapped to a plane over the
		 * current one. */
		if (zpos_prop->value > other_zpos_prop->value &&
		    layer_intersects(layer, other_layer)) {
			return true;
		}
	}

	return false;
}

static bool
has_allocated_plane_under(struct liftoff_output *output,
			  struct alloc_step *step, struct liftoff_layer *layer)
{
	struct liftoff_plane *plane, *other_plane;
	ssize_t i;

	plane = liftoff_container_of(step->plane_link, plane, link);

	i = -1;
	liftoff_list_for_each(other_plane, &output->device->planes, link) {
		i++;
		if (i >= (ssize_t)step->plane_idx) {
			break;
		}
		if (other_plane->type == DRM_PLANE_TYPE_PRIMARY) {
			continue;
		}
		if (step->alloc[i] == NULL) {
			continue;
		}

		if (plane->zpos >= other_plane->zpos &&
		    layer_intersects(layer, step->alloc[i])) {
			return true;
		}
	}

	return false;
}

static bool
check_layer_plane_compatible(struct alloc_step *step,
			     struct liftoff_layer *layer,
			     struct liftoff_plane *plane)
{
	struct liftoff_output *output;
	struct liftoff_layer_property *zpos_prop;

	output = layer->output;

	/* Skip this layer if already allocated */
	if (is_layer_allocated(step, layer)) {
		return false;
	}

	zpos_prop = layer_get_property(layer, "zpos");
	if (zpos_prop != NULL) {
		if ((int)zpos_prop->value > step->last_layer_zpos &&
		    has_allocated_layer_over(output, step, layer)) {
			/* This layer needs to be on top of the last
			 * allocated one */
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": "
				    "layer zpos invalid",
				    step->log_prefix, (void *)layer, plane->id);
			return false;
		}
		if ((int)zpos_prop->value < step->last_layer_zpos &&
		    has_allocated_plane_under(output, step, layer)) {
			/* This layer needs to be under the last
			 * allocated one, but this plane isn't under the
			 * last one (in practice, since planes are
			 * sorted by zpos it means it has the same zpos,
			 * ie. undefined ordering). */
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": "
				    "plane zpos invalid",
				    step->log_prefix, (void *)layer, plane->id);
			return false;
		}
		if (plane->type != DRM_PLANE_TYPE_PRIMARY &&
		    (int)zpos_prop->value < step->primary_layer_zpos &&
		    plane->zpos > step->primary_plane_zpos) {
			/* Primary planes are handled up front, because some
			 * drivers fail all atomic commits when it's missing.
			 * However that messes up with our zpos checks. In
			 * particular, we need to make sure we don't put a layer
			 * configured to be over the primary plane under it.
			 * TODO: revisit this once we add underlay support. */
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": "
				    "layer zpos under primary",
				    step->log_prefix, (void *)layer, plane->id);
			return false;
		}
	}

	if (plane->type != DRM_PLANE_TYPE_PRIMARY &&
	    has_composited_layer_over(output, step, layer)) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%s Layer %p -> plane %"PRIu32": "
			    "has composited layer on top",
			    step->log_prefix, (void *)layer, plane->id);
		return false;
	}

	if (plane->type != DRM_PLANE_TYPE_PRIMARY &&
	    layer == layer->output->composition_layer) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%s Layer %p -> plane %"PRIu32": "
			    "cannot put composition layer on "
			    "non-primary plane",
			    step->log_prefix, (void *)layer, plane->id);
		return false;
	}

	return true;
}

static bool
check_alloc_valid(struct liftoff_output *output, struct alloc_result *result,
		  struct alloc_step *step)
{
	/* If composition isn't used, we need to have allocated all
	 * layers. */
	/* TODO: find a way to fail earlier, e.g. when the number of
	 * layers exceeds the number of planes. */
	if (result->has_composition_layer && !step->composited &&
	    step->score != (int)result->non_composition_layers_len) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%sCannot skip composition: some layers "
			    "are missing a plane", step->log_prefix);
		return false;
	}
	/* On the other hand, if we manage to allocate all layers, we
	 * don't want to use composition. We don't want to use the
	 * composition layer at all. */
	if (step->composited &&
	    step->score == (int)result->non_composition_layers_len) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%sRefusing to use composition: all layers "
			    "have been put in a plane", step->log_prefix);
		return false;
	}

	/* TODO: check allocation isn't empty */

	return true;
}

static int
output_choose_layers(struct liftoff_output *output, struct alloc_result *result,
		     struct alloc_step *step)
{
	struct liftoff_device *device;
	struct liftoff_plane *plane;
	struct liftoff_layer *layer;
	int cursor, ret;
	size_t remaining_planes;
	struct alloc_step next_step = {0};

	device = output->device;

	if (step->plane_link == &device->planes) { /* Allocation finished */
		if (step->score > result->best_score &&
		    check_alloc_valid(output, result, step)) {
			/* We found a better allocation */
			liftoff_log(LIFTOFF_DEBUG,
				    "%sFound a better allocation with score=%d",
				    step->log_prefix, step->score);
			result->best_score = step->score;
			memcpy(result->best, step->alloc,
			       result->planes_len * sizeof(struct liftoff_layer *));
		}
		return 0;
	}

	plane = liftoff_container_of(step->plane_link, plane, link);

	remaining_planes = result->planes_len - step->plane_idx;
	if (result->best_score >= step->score + (int)remaining_planes) {
		/* Even if we find a layer for all remaining planes, we won't
		 * find a better allocation. Give up. */
		/* TODO: change remaining_planes to only count those whose
		 * possible CRTC match and which aren't allocated */
		return 0;
	}

	cursor = drmModeAtomicGetCursor(result->req);

	if (plane->layer != NULL) {
		goto skip;
	}
	if ((plane->possible_crtcs & (1 << output->crtc_index)) == 0) {
		goto skip;
	}

	liftoff_log(LIFTOFF_DEBUG,
		    "%sPerforming allocation for plane %"PRIu32" (%zu/%zu)",
		    step->log_prefix, plane->id, step->plane_idx + 1, result->planes_len);

	liftoff_list_for_each(layer, &output->layers, link) {
		if (layer->plane != NULL) {
			continue;
		}
		if (!layer_is_visible(layer)) {
			continue;
		}
		if (!check_layer_plane_compatible(step, layer, plane)) {
			continue;
		}

		if (!check_deadline(result->started_at)) {
			liftoff_log(LIFTOFF_DEBUG, "%s Deadline exceeded",
				    step->log_prefix);
			break;
		}

		/* Try to use this layer for the current plane */
		ret = plane_apply(plane, layer, result->req);
		if (ret == -EINVAL) {
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": "
				    "incompatible properties",
				    step->log_prefix, (void *)layer, plane->id);
			continue;
		} else if (ret != 0) {
			return ret;
		}

		layer_add_candidate_plane(layer, plane);

		/* If composition is forced, wait until after the
		 * layer_add_candidate_plane() call to reject the plane: we want
		 * to return a meaningful list of candidate planes so that the
		 * API user has the opportunity to re-allocate its buffers with
		 * scanout-capable ones. Same deal for the FB check. */
		if (layer->force_composition || !plane_check_layer_fb(plane, layer)) {
			drmModeAtomicSetCursor(result->req, cursor);
			continue;
		}

		ret = device_test_commit(device, result->req, result->flags);
		if (ret == 0) {
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": success",
				    step->log_prefix, (void *)layer, plane->id);
			/* Continue with the next plane */
			plane_step_init_next(&next_step, step, layer);
			ret = output_choose_layers(output, result, &next_step);
			if (ret != 0) {
				return ret;
			}
		} else if (ret != -EINVAL && ret != -ERANGE && ret != -ENOSPC) {
			return ret;
		} else {
			liftoff_log(LIFTOFF_DEBUG,
				    "%s Layer %p -> plane %"PRIu32": "
				    "test-only commit failed (%s)",
				    step->log_prefix, (void *)layer, plane->id,
				    strerror(-ret));
		}

		drmModeAtomicSetCursor(result->req, cursor);
	}

skip:
	/* Try not to use the current plane */
	plane_step_init_next(&next_step, step, NULL);
	ret = output_choose_layers(output, result, &next_step);
	if (ret != 0) {
		return ret;
	}
	drmModeAtomicSetCursor(result->req, cursor);

	return 0;
}

static int
apply_current(struct liftoff_device *device, drmModeAtomicReq *req)
{
	struct liftoff_plane *plane;
	int cursor, ret;

	cursor = drmModeAtomicGetCursor(req);

	liftoff_list_for_each(plane, &device->planes, link) {
		ret = plane_apply(plane, plane->layer, req);
		if (ret != 0) {
			drmModeAtomicSetCursor(req, cursor);
			return ret;
		}
	}

	return 0;
}

static bool
fb_info_needs_realloc(const drmModeFB2 *a, const drmModeFB2 *b)
{
	if (a->width != b->width || a->height != b->height ||
	    a->pixel_format != b->pixel_format || a->modifier != b->modifier) {
		return true;
	}

	/* TODO: consider checking pitch and offset? */

	return false;
}

static bool
layer_intersection_changed(struct liftoff_layer *this,
			   struct liftoff_output *output)
{
	struct liftoff_layer *other;

	struct liftoff_rect this_cur, this_prev, other_cur, other_prev;

	layer_get_rect(this, &this_cur, false);
	layer_get_rect(this, &this_prev, true);
	liftoff_list_for_each(other, &output->layers, link) {
		if (this == other) {
			continue;
		}

		layer_get_rect(other, &other_cur, false);
		layer_get_rect(other, &other_prev, true);

		if (rect_intersects(&this_cur, &other_cur) !=
		    rect_intersects(&this_prev, &other_prev)) {
			return true;
		}
	}

	return false;
}

static bool
layer_needs_realloc(struct liftoff_layer *layer, struct liftoff_output *output)
{
	struct liftoff_layer_property *prop;
	bool check_crtc_intersect = false;
	size_t i;

	if (layer->changed) {
		liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
			    "layer property added or force composition changed");
		return true;
	}

	for (i = 0; i < layer->props_len; i++) {
		prop = &layer->props[i];

		/* If FB_ID changes from non-zero to zero, we don't need to
		 * display this layer anymore, so we may be able to re-use its
		 * plane for another layer. If FB_ID changes from zero to
		 * non-zero, we might be able to find a plane for this layer.
		 * If FB_ID changes from non-zero to non-zero and the FB
		 * attributes didn't change, we can try to re-use the previous
		 * allocation. */
		if (strcmp(prop->name, "FB_ID") == 0) {
			if (prop->value == 0 && prop->prev_value == 0) {
				continue;
			}

			if (prop->value == 0 || prop->prev_value == 0) {
				liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
					    "layer enabled or disabled");
				return true;
			}

			if (fb_info_needs_realloc(&layer->fb_info,
						  &layer->prev_fb_info)) {
				liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
					    "FB info changed");
				return true;
			}

			continue;
		}

		/* For all properties except FB_ID, we can skip realloc if the
		 * value didn't change. */
		if (prop->value == prop->prev_value) {
			continue;
		}

		/* If the layer was or becomes completely transparent or
		 * completely opaque, we might be able to find a better
		 * allocation. Otherwise, we can keep the current one. */
		if (strcmp(prop->name, "alpha") == 0) {
			if (prop->value == 0 || prop->prev_value == 0 ||
			    prop->value == 0xFFFF || prop->prev_value == 0xFFFF) {
				liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
					    "alpha changed");
				return true;
			}
			continue;
		}

		/* We should never need a re-alloc when IN_FENCE_FD or
		 * FB_DAMAGE_CLIPS changes. */
		if (strcmp(prop->name, "IN_FENCE_FD") == 0 ||
		    strcmp(prop->name, "FB_DAMAGE_CLIPS") == 0) {
			continue;
		}

		// If CRTC_* changed, check for intersection later
		if (strcmp(prop->name, "CRTC_X") == 0 ||
		    strcmp(prop->name, "CRTC_Y") == 0 ||
		    strcmp(prop->name, "CRTC_W") == 0 ||
		    strcmp(prop->name, "CRTC_H") == 0) {
			check_crtc_intersect = true;
			continue;
		}

		liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
			    "property \"%s\" changed", prop->name);
		return true;
	}

	if (check_crtc_intersect &&
	    layer_intersection_changed(layer, output)) {
		liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
			    "intersection with other layer(s) changed");
		return true;
	}

	return false;
}

static bool
layer_is_higher_priority(struct liftoff_layer *this, struct liftoff_layer *other)
{
	struct liftoff_layer_property *this_zpos, *other_zpos;
	bool this_visible, other_visible, intersects;

	// The composition layer should be highest priority.
	if (this->output->composition_layer == this) {
		return true;
	} else if (this->output->composition_layer == other) {
		return false;
	}

	// Invisible layers are given lowest priority. Pass-thru if both have
	// same visibility
	this_visible = layer_is_visible(this);
	other_visible = layer_is_visible(other);
	if (this_visible != other_visible) {
		return this_visible;
	}

	// A layer's overall priority is determined by a combination of it's
	// current_priority, it's zpos, and whether it intersects with others.
	//
	// Consider two layers. If they do not intersect, the layer with higher
	// priority is given overall priority. However if both layers have
	// identical priority, then the layer with higher zpos is given overall
	// priority.
	//
	// If the layers intersect, their zpos determines the overall priority.
	// If their zpos are identical, then simply fallback to looking at
	// current_priority. Otherwise, the layer with higher zpos is given
	// overall priority, since the top layer needs to be offloaded in order
	// to offload the bottom layer.

	this_zpos = layer_get_property(this, "zpos");
	other_zpos = layer_get_property(other, "zpos");
	intersects = layer_intersects(this, other);

	if (this_zpos != NULL && other_zpos != NULL) {
		if (intersects) {
			return this_zpos->value == other_zpos->value ?
			       this->current_priority > other->current_priority :
			       this_zpos->value > other_zpos->value;
		} else {
			return this->current_priority == other->current_priority ?
			       this_zpos->value > other_zpos->value :
			       this->current_priority > other->current_priority;
		}
	} else if (this_zpos == NULL && other_zpos == NULL) {
		return this->current_priority > other->current_priority;
	} else {
		// Either this or other zpos is null
		return this_zpos != NULL;
	}
}

static bool
update_layers_order(struct liftoff_output *output)
{
	struct liftoff_list *search, *max, *cur, *head;
	struct liftoff_layer *this_layer, *other_layer;
	bool order_changed = false;

	head = &output->layers;
	cur = head;

	// Run a insertion sort to order layers by priority.
	while (cur->next != head) {
		cur = cur->next;

		max = cur;
		search = cur;
		while (search->next != head) {
			search = search->next;
			this_layer = liftoff_container_of(search, this_layer, link);
			other_layer = liftoff_container_of(max, other_layer, link);
			if (layer_is_higher_priority(this_layer, other_layer)) {
				max = search;
			}
		}

		if (cur != max) {
			liftoff_list_swap(cur, max);
			// max is now where iterator cur was, relocate to continue
			cur = max;
			order_changed = true;
		}
	}

	return order_changed;
}

static int
reuse_previous_alloc(struct liftoff_output *output, drmModeAtomicReq *req,
		     uint32_t flags)
{
	struct liftoff_device *device;
	struct liftoff_layer *layer;
	int cursor, ret;
	bool layer_order_changed;

	device = output->device;

	layer_order_changed = update_layers_order(output);

	if (output->layers_changed) {
		liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
			    "a layer has been added or removed");
		return -EINVAL;
	}

	liftoff_list_for_each(layer, &output->layers, link) {
		if (layer_needs_realloc(layer, output)) {
			return -EINVAL;
		}
	}

	if (layer_order_changed) {
		liftoff_log(LIFTOFF_DEBUG, "Cannot re-use previous allocation: "
			    "layer priority order changed.");
		return -EINVAL;
	}

	cursor = drmModeAtomicGetCursor(req);

	ret = apply_current(device, req);
	if (ret != 0) {
		return ret;
	}

	ret = device_test_commit(device, req, flags);
	if (ret != 0) {
		drmModeAtomicSetCursor(req, cursor);
	}
	return ret;
}

static void
mark_layers_clean(struct liftoff_output *output)
{
	struct liftoff_layer *layer;

	output->layers_changed = false;

	liftoff_list_for_each(layer, &output->layers, link) {
		layer_mark_clean(layer);
	}
}

static void
update_layers_priority(struct liftoff_device *device)
{
	struct liftoff_output *output;
	struct liftoff_layer *layer;
	bool period_elapsed;

	device->page_flip_counter++;
	period_elapsed = device->page_flip_counter >= LIFTOFF_PRIORITY_PERIOD;
	if (period_elapsed) {
		device->page_flip_counter = 0;
	}

	liftoff_list_for_each(output, &device->outputs, link) {
		liftoff_list_for_each(layer, &output->layers, link) {
			layer_update_priority(layer, period_elapsed);
		}
	}
}

static void
update_layers_fb_info(struct liftoff_output *output)
{
	struct liftoff_layer *layer;

	/* We don't know what the library user did in-between
	 * liftoff_output_apply() calls. They might've removed the FB and
	 * re-created a completely different one which happens to have the same
	 * FB ID. */
	liftoff_list_for_each(layer, &output->layers, link) {
		memset(&layer->fb_info, 0, sizeof(layer->fb_info));
		layer_cache_fb_info(layer);
		/* TODO: propagate error? */
	}
}

static void
log_reuse(struct liftoff_output *output)
{
	if (output->alloc_reused_counter == 0) {
		liftoff_log(LIFTOFF_DEBUG,
			    "Reusing previous plane allocation on output %p",
			    (void *)output);
	}
	output->alloc_reused_counter++;
}

static void
log_no_reuse(struct liftoff_output *output)
{
	liftoff_log(LIFTOFF_DEBUG, "Computing plane allocation on output %p",
		    (void *)output);

	if (output->alloc_reused_counter != 0) {
		liftoff_log(LIFTOFF_DEBUG,
			    "Stopped reusing previous plane allocation on "
			    "output %p (had reused it %d times)",
			    (void *)output, output->alloc_reused_counter);
		output->alloc_reused_counter = 0;
	}
}

static size_t
non_composition_layers_length(struct liftoff_output *output)
{
	struct liftoff_layer *layer;
	size_t n;

	n = 0;
	liftoff_list_for_each(layer, &output->layers, link) {
		if (layer_is_visible(layer) &&
		    output->composition_layer != layer) {
			n++;
		}
	}

	return n;
}

static bool check_layer_plane_zorder(struct alloc_step *step,
				     struct liftoff_layer *layer,
				     bool check_underlay)
{
	struct liftoff_layer_property *zpos_prop, *other_zpos_prop;
	struct liftoff_plane *plane, *other_plane;
	struct liftoff_layer *other_layer;
	ssize_t i;

	zpos_prop = layer_get_property(layer, "zpos");
	if (zpos_prop == NULL) {
		return true;
	}

	plane = liftoff_container_of(step->plane_link, plane, link);

	i = -1;
	liftoff_list_for_each(other_plane, &layer->output->device->planes, link) {
		i++;
		if (i >= (ssize_t)step->plane_idx) {
			break;
		}

		other_layer = step->alloc[i];
		if (other_layer == NULL) {
			continue;
		}

		/* Only compare planes with same kind of alloc (underlay or
		 * overlay). The composition layer - although not an underlay -
		 * need to be considered when checking z-ordering amongst
		 * underlay planes.
		 */
		if (check_underlay &&
		    other_layer != layer->output->composition_layer &&
		    (step->underlay_plane_mask & (1 << i)) == 0) {
			continue;
		}
		if (!check_underlay && (step->underlay_plane_mask & (1 << i)) > 0) {
			continue;
		}

		/* z-order doesn't matter if layers don't intersect */
		if (!layer_intersects(layer, other_layer)) {
			continue;
		}

		other_zpos_prop = layer_get_property(other_layer, "zpos");
		if (other_zpos_prop == NULL) {
			continue;
		}

		if (check_underlay &&
		    other_layer == layer->output->composition_layer &&
		    other_plane->zpos > plane->zpos) {
			/* Only check plane ordering for the composition plane.
			 * It is valid for underlay layers of higher zpos to be
			 * allocated to planes of lower zpos */
			continue;
		} else if (other_plane->zpos > plane->zpos &&
			   other_zpos_prop->value > zpos_prop->value) {
			/* For all non-composition planes, ensure their z-order
			 * aligns with their allocated layers' z-order */
			continue;
		} else if (other_plane->zpos < plane->zpos &&
			   other_zpos_prop->value < zpos_prop->value) {
			continue;
		}
		/* Else, either:
		 * 1. misaligned plane/layer z-order, or
		 * 2. equivalent plane zpos leading to undefined z-order
		 */

		liftoff_log(LIFTOFF_DEBUG,
			"%sLayer %p -> plane %"PRIu32": "
			"%s layer zpos invalid. zpos for "
			"plane/layer this: %d/%lu other: %d/%lu",
			step->log_prefix, (void *)layer, plane->id,
			check_underlay ? "Underlay" : "Overlay",
			plane->zpos, zpos_prop->value,
			other_plane->zpos, other_zpos_prop->value);
		return false;
	}
	return true;
}

static bool layer_is_opaque(struct liftoff_layer *layer)
{
	return !pixel_format_has_alpha(layer->fb_info.pixel_format);

	/* TODO: Consider alpha property? */
}

static bool underlay_check_layer_plane_compatible(struct alloc_step *step,
						  struct liftoff_layer *layer,
						  struct liftoff_plane *plane)
{
	/* Skip this layer if already allocated */
	if (layer == NULL || is_layer_allocated(step, layer)) {
		return false;
	}

	if (step_is_underlay_alloc(layer, step)) {
		/* Underlay layer must be opaque */
		if (!layer_is_opaque(layer)) {
			liftoff_log(LIFTOFF_DEBUG,
				"%sLayer %p -> plane %"PRIu32": "
				"Underlay layer is not opaque",
				step->log_prefix, (void *)layer, plane->id);
			return false;
		}

		if (!check_layer_plane_zorder(step, layer, true)) {
			return false;
		}

		/* No need to check for intersecting force-composited layers,
		 * since punch-through composition takes care of that for
		 * underlay. */
		return true;

	} else {
		/* Overlays cannot have a force-composited layer occluding it */
		if (plane->type != DRM_PLANE_TYPE_PRIMARY &&
		    layer != layer->output->composition_layer &&
		    has_composited_layer_over(layer->output, step, layer)) {
			liftoff_log(LIFTOFF_DEBUG,
				"%sLayer %p -> plane %"PRIu32": "
				"has composited layer on top",
				step->log_prefix, (void *)layer, plane->id);
			return false;
		}

		if(!check_layer_plane_zorder(step, layer, false)) {
			return false;
		}
		return true;
	}
}

static bool underlay_has_opaque_layer(struct liftoff_output *output)
{
	struct liftoff_layer *layer;

	liftoff_list_for_each(layer, &output->layers, link) {
		if (!layer->force_composition && layer_is_opaque(layer) &&
		    layer_is_visible(layer)) {
			return true;
		}
	}

	return false;
}

static bool underlay_check_alloc_valid(struct liftoff_output *output,
				       struct alloc_result *result,
				       struct alloc_step *step)
{
	/* If composition isn't used, we need to have allocated all
	 * layers. */
	/* TODO: find a way to fail earlier, e.g. when the number of
	 * layers exceeds the number of planes. */
	if (result->has_composition_layer && !step->composited &&
	    step->score != (int)result->non_composition_layers_len) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%sCannot skip composition: some layers "
			    "are missing a plane", step->log_prefix);
		return false;
	}
	/* On the other hand, if we manage to allocate all layers, we
	 * don't want to use composition. We don't want to use the
	 * composition layer at all. */
	if (step->composited && step->underlay_plane_mask == 0 &&
	    step->score == (int)result->non_composition_layers_len) {
		liftoff_log(LIFTOFF_DEBUG,
			    "%sRefusing to use composition: all layers "
			    "have been put in a plane", step->log_prefix);
		return false;
	}

	/* TODO: check allocation isn't empty */

	return true;
}

/**
 * Check whether the current allocation in `step` is the best allocation
 * thus far. If so, save it into `result`.
*/
static void underlay_update_best_alloc(struct liftoff_output *output,
				       struct alloc_result *result,
				       struct alloc_step *step)
{
	int step_score = step->score;

	if (step->composited && step->underlay_plane_mask != 0) {
		/* Give preference to underlay alloc even if an overlay alloc
		 * can skip the composition layer */
		step_score += 1;
	}

	if (step_score > result->best_score &&
	    underlay_check_alloc_valid(output, result, step)) {
		/* We found a better allocation */
		liftoff_log(LIFTOFF_DEBUG,
			    "%sFound a better %sallocation with score=%d%s",
			    step->log_prefix,
			    step->underlay_plane_mask == 0 ? "" : "underlay ",
			    step->score,
			    step->underlay_plane_mask == 0 ? "" : "+1");
		result->best_score = step_score;
		result->underlay_plane_mask = step->underlay_plane_mask;
		memcpy(result->best, step->alloc,
		       result->planes_len * sizeof(struct liftoff_layer *));
	}
}

/**
 * Check whether the DFS rooted at the current step can be pruned
*/
static bool underlay_can_prune_branch(struct alloc_result *result,
				      struct alloc_step *step)
{
	int best_score, remaining_planes;

	best_score = result->best_score;
	/* The composition plane is included in the score for underlay allocs.
	 * Take this into account when calculating remaining planes */
	if (result->underlay_plane_mask != 0) {
		best_score -= 1;
	}

	remaining_planes = result->planes_len - step->plane_idx;
	/* Even if we find a layer for all remaining planes, we won't find a
	 * better allocation. Give up. */
	/* TODO: change remaining_planes to only count those whose possible CRTC
	 * match and which aren't allocated */
	if (best_score >= step->score + (int)remaining_planes) {
		return true;
	}
	return false;
}

static int underlay_output_choose_layers(struct liftoff_output *output,
					 struct alloc_result *result,
					 struct alloc_step *step)
{

	struct liftoff_device *device;
	struct liftoff_plane *plane;
	struct liftoff_layer *layer;
	int cursor, ret;
	struct alloc_step next_step = {0};
	bool reroute_primary_composition_alloc = false;

	device = output->device;

	/* Base case - all planes have been tried*/
	if (step->plane_link == &device->planes) {
		underlay_update_best_alloc(output, result, step);
		return 0;
	}

	plane = liftoff_container_of(step->plane_link, plane, link);

	if (underlay_can_prune_branch(result, step)) {
		return 0;
	}

	cursor = drmModeAtomicGetCursor(result->req);

	if (plane->layer != NULL) {
		goto skip;
	}
	if ((plane->possible_crtcs & (1 << output->crtc_index)) == 0) {
		goto skip;
	}

	liftoff_log(LIFTOFF_DEBUG,
		    "%sPerforming allocation for plane %"PRIu32" type %d (%zu/%zu)",
		    step->log_prefix, plane->id, plane->type,
		    step->plane_idx + 1, result->planes_len);

	liftoff_list_for_each(layer, &output->layers, link) {
		if (layer->plane != NULL) {
			continue;
		}

		if (!layer_is_visible(layer)) {
			continue;
		}

		/* If composition layer has been allocated, only consider
		 * underlay layers. Otherwise, consider allocating the
		 * composition layer (which must be first in output->layers).
		 * And if that also fails, consider overlay layers */
		if (!underlay_check_layer_plane_compatible(
		    step, layer, plane)) {
			continue;
		}

		if (is_layer_allocated(step, layer)) {
			continue;
		}

		if (!check_deadline(result->started_at)) {
			liftoff_log(LIFTOFF_DEBUG, "%sDeadline exceeded",
				    step->log_prefix);
			break;
		}

		/* Try to use this layer for the current plane */
		ret = plane_apply(plane, layer, result->req);
		if (ret == -EINVAL) {
			liftoff_log(LIFTOFF_DEBUG,
				    "%sLayer %p -> plane %"PRIu32": "
				    "incompatible properties",
				    step->log_prefix, (void *)layer, plane->id);
			continue;
		} else if (ret != 0) {
			return ret;
		}

		layer_add_candidate_plane(layer, plane);

		/* If composition is forced, wait until after the
		 * layer_add_candidate_plane() call to reject the plane: we want
		 * to return a meaningful list of candidate planes so that the
		 * API user has the opportunity to re-allocate its buffers with
		 * scanout-capable ones. Same deal for the FB check. */
		if (layer->force_composition ||
		    !plane_check_layer_fb(plane, layer)) {
			drmModeAtomicSetCursor(result->req, cursor);
			continue;
		}

		ret = device_test_commit(device, result->req, result->flags);
		/*
		 * This reroutes the DFS tree by moving the branch rooted at the
		 * composition_layer -> PRIMARY_plane allocation from being
		 * traversed first, to being traversed last.
		 *
		 * It serves as an optimization to try underlay_layer->PRIMARY
		 * allocations first, while treating composition_layer->PRIMARY
		 * as a fallback.
		 */
		if (ret == 0 && plane->type == DRM_PLANE_TYPE_PRIMARY &&
		    layer == output->composition_layer) {
			liftoff_log(LIFTOFF_DEBUG,
				    "%sComposition Layer %p -> Primary plane "
				    "%"PRIu32": success. Rerouting to end of "
				    "DFS.",
				    step->log_prefix, (void *)layer, plane->id);
			reroute_primary_composition_alloc = true;
			continue;
		}
		if (ret == 0) {
			liftoff_log(LIFTOFF_DEBUG,
				    "%sLayer %p -> plane %"PRIu32": success",
				    step->log_prefix, (void *)layer, plane->id);
			/* Continue with the next plane */
			plane_step_init_next(&next_step, step, layer);
			ret = underlay_output_choose_layers(output, result,
							    &next_step);
			if (ret != 0) {
				return ret;
			}
		} else if (ret != -EINVAL && ret != -ERANGE && ret != -ENOSPC) {
			return ret;
		} else {
			liftoff_log(LIFTOFF_DEBUG,
				    "%sLayer %p -> plane %"PRIu32": "
				    "test-only commit failed (%s)",
				    step->log_prefix, (void *)layer, plane->id,
				    strerror(-ret));
		}

		drmModeAtomicSetCursor(result->req, cursor);
	}

skip:
	if (reroute_primary_composition_alloc) {
		liftoff_log(LIFTOFF_DEBUG,
			"%sPrimary plane %"PRIu32": Retry composition layer",
			step->log_prefix, plane->id);
		/* This should never fail since we've applied before */
		ret = plane_apply(plane, output->composition_layer, result->req);
		assert(ret == 0);
		plane_step_init_next(&next_step, step, output->composition_layer);
	} else {
		/* Try not using the current plane */
		liftoff_log(LIFTOFF_DEBUG,
			"%sPlane %"PRIu32": Skipping.",
			step->log_prefix, plane->id);
		plane_step_init_next(&next_step, step, NULL);
	}
	ret = underlay_output_choose_layers(output, result, &next_step);
	if (ret != 0) {
		return ret;
	}
	drmModeAtomicSetCursor(result->req, cursor);
	return 0;
}

static int run_alloc(struct liftoff_output *output,
		       struct alloc_result *result,
		       bool underlay_support) {
	struct liftoff_device *device;
	struct alloc_step step = {0};
	int ret;

	device = output->device;

	step.alloc = malloc(result->planes_len * sizeof(*step.alloc));
	if (step.alloc == NULL || result->best == NULL) {
		liftoff_log_errno(LIFTOFF_ERROR, "malloc");
		return -ENOMEM;
	}

	if (clock_gettime(CLOCK_MONOTONIC, &result->started_at) != 0) {
		liftoff_log_errno(LIFTOFF_ERROR, "clock_gettime");
		return -errno;
	}

	/* For each plane, try to find a layer. Don't do it the other
	 * way around (ie. for each layer, try to find a plane) because
	 * some drivers want user-space to enable the primary plane
	 * before any other plane. */

	result->best_score = -1;
	result->underlay_plane_mask = 0;
	memset(result->best, 0, result->planes_len * sizeof(*result->best));
	result->has_composition_layer = output->composition_layer != NULL;
	result->non_composition_layers_len =
		non_composition_layers_length(output);
	step.plane_link = device->planes.next;
	step.plane_idx = 0;
	step.score = 0;
	step.last_layer_zpos = INT_MAX;
	step.primary_layer_zpos = INT_MIN;
	step.primary_plane_zpos = INT_MAX;
	step.composited = false;
	step.underlay_plane_mask = 0;
	if (underlay_support) {
		step.underlay_has_opaque_layer = underlay_has_opaque_layer(output);
		ret = underlay_output_choose_layers(output, result, &step);
	} else {
		ret = output_choose_layers(output, result, &step);
	}
	free(step.alloc);
	return ret;
}

static int overlay_alloc(struct liftoff_output *output,
			 struct alloc_result *result) {
	return run_alloc(output, result, false);
}

static int underlay_alloc(struct liftoff_output *output,
			  struct alloc_result *result)
{
	return run_alloc(output, result, true);
}

const struct liftoff_alloc_strategy alloc_overlay_strategy = {
	.alloc = overlay_alloc,
	.reuse = reuse_previous_alloc,
};

const struct liftoff_alloc_strategy alloc_underlay_strategy = {
	.alloc = underlay_alloc,
	.reuse = reuse_previous_alloc,
};

int
liftoff_output_apply(struct liftoff_output *output, drmModeAtomicReq *req,
		     uint32_t flags)
{
	struct liftoff_device *device;
	struct liftoff_plane *plane;
	struct liftoff_layer *layer;
	struct alloc_result result = {0};
	size_t i, candidate_planes;
	int ret;

	device = output->device;

	update_layers_priority(device);
	update_layers_fb_info(output);

	ret = device->alloc_strategy->reuse(output, req, flags);
	if (ret == 0) {
		log_reuse(output);
		mark_layers_clean(output);
		return 0;
	}
	log_no_reuse(output);

	/* Reset layers' candidate planes */
	liftoff_list_for_each(layer, &output->layers, link) {
		layer_reset_candidate_planes(layer);
	}

	device->test_commit_counter = 0;
	output_log_layers(output);

	/* Unset all existing plane and layer mappings. */
	liftoff_list_for_each(plane, &device->planes, link) {
		if (plane->layer != NULL && plane->layer->output == output) {
			plane->layer->plane = NULL;
			plane->layer = NULL;
		}
	}

	/* Disable all planes we might use. Do it before building mappings to
	 * make sure not to hit bandwidth limits because too many planes are
	 * enabled. */
	candidate_planes = 0;
	liftoff_list_for_each(plane, &device->planes, link) {
		if (plane->layer == NULL) {
			candidate_planes++;
			liftoff_log(LIFTOFF_DEBUG,
				    "Disabling plane %"PRIu32, plane->id);
			ret = plane_apply(plane, NULL, req);
			assert(ret != -EINVAL);
			if (ret != 0) {
				return ret;
			}
		}
	}

	result.req = req;
	result.flags = flags;
	result.planes_len = liftoff_list_length(&device->planes);

	result.best = malloc(result.planes_len * sizeof(*result.best));
	if (result.best == NULL) {
		liftoff_log_errno(LIFTOFF_ERROR, "malloc");
		return -ENOMEM;
	}

	ret = device->alloc_strategy->alloc(output, &result);
	if (ret != 0) {
		return ret;
	}

	liftoff_log(LIFTOFF_DEBUG,
		    "Found plane allocation for output %p (score: %d, candidate planes: %zu, tests: %d):",
		    (void *)output, result.best_score, candidate_planes,
		    device->test_commit_counter);

	/* Apply the best allocation */
	i = 0;
	liftoff_list_for_each(plane, &device->planes, link) {
		layer = result.best[i];
		i++;
		if (layer == NULL) {
			continue;
		}

		liftoff_log(LIFTOFF_DEBUG, "  Layer %p -> plane %"PRIu32,
			    (void *)layer, plane->id);

		assert(plane->layer == NULL);
		assert(layer->plane == NULL);
		plane->layer = layer;
		layer->plane = plane;

		if ((result.underlay_plane_mask & (1 << (i-1))) > 0) {
			layer->is_underlay = true;
		}
	}
	if (i == 0) {
		liftoff_log(LIFTOFF_DEBUG, "  (No layer has a plane)");
	}

	ret = apply_current(device, req);
	if (ret != 0) {
		return ret;
	}

	free(result.best);

	mark_layers_clean(output);

	return 0;
}
