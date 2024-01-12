#include <assert.h>
#include <drm_fourcc.h>
#include <errno.h>
#include <unistd.h>
#include <libliftoff.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "libdrm_mock.h"

static struct liftoff_layer *
add_layer(struct liftoff_output *output, int x, int y, int width, int height,
	  uint32_t drm_format)
{
	uint32_t fb_id;
	drmModeFB2 fb_info;
	struct liftoff_layer *layer;

	layer = liftoff_layer_create(output);
	fb_id = liftoff_mock_drm_create_fb(layer);
	fb_info = (drmModeFB2) {
		.fb_id = fb_id,
		.width = (uint32_t)width,
		.height = (uint32_t)height,
		.flags = 0,
		.pixel_format = drm_format,
	};
	liftoff_mock_drm_set_fb_info(&fb_info);
	liftoff_layer_set_property(layer, "FB_ID", fb_id);
	liftoff_layer_set_property(layer, "CRTC_X", (uint64_t)x);
	liftoff_layer_set_property(layer, "CRTC_Y", (uint64_t)y);
	liftoff_layer_set_property(layer, "CRTC_W", (uint64_t)width);
	liftoff_layer_set_property(layer, "CRTC_H", (uint64_t)height);
	liftoff_layer_set_property(layer, "SRC_X", 0);
	liftoff_layer_set_property(layer, "SRC_Y", 0);
	liftoff_layer_set_property(layer, "SRC_W", (uint64_t)width << 16);
	liftoff_layer_set_property(layer, "SRC_H", (uint64_t)height << 16);

	return layer;
}

struct test_plane {
	uint64_t type;
};

struct test_prop {
	const char *name;
	uint64_t value;
};

/* This structure describes a layer in a test case. The first block of fields
 * describe the layer properties: geometry, vertical ordering, etc. The `compat`
 * field describes which hardware planes the layer is compatible with. The
 * `result` field describes the expected hardware plane returned by libliftoff.
 */
struct test_layer {
	int x, y, width, height;
	bool composition;
	bool force_composited;
	bool is_underlay;
	bool no_alpha;
	struct test_prop props[64];

	struct test_plane *compat[64];
	struct test_plane *result;
};

struct test_case {
	const char *name;
	bool needs_composition;
	struct test_layer layers[64];
};

/* This array describes the hardware we're going to perform the tests with. Our
 * hardware has one primary plane at the bottom position, two overlay planes
 * at the middle position (with undefined ordering between themselves), and one
 * cursor plane at the top.
 */
static struct test_plane test_setup[] = {
	{ .type = DRM_PLANE_TYPE_PRIMARY }, /* zpos = 0 */
	{ .type = DRM_PLANE_TYPE_CURSOR }, /* zpos = 2 */
	{ .type = DRM_PLANE_TYPE_OVERLAY }, /* zpos = 1 */
	{ .type = DRM_PLANE_TYPE_OVERLAY }, /* zpos = 1 */
};

static const size_t test_setup_len = sizeof(test_setup) / sizeof(test_setup[0]);

#define PRIMARY_PLANE &test_setup[0]
#define CURSOR_PLANE &test_setup[1]
#define OVERLAY_PLANE &test_setup[2]
#define OVERLAY_PLANE_B &test_setup[3]

/* non-primary planes */
#define NON_CURSOR_PLANES { &test_setup[0], &test_setup[2], &test_setup[3] }
#define FIRST_2_SECONDARY_PLANES { &test_setup[1], &test_setup[2] }
#define FIRST_3_SECONDARY_PLANES { &test_setup[1], &test_setup[2], \
				   &test_setup[3] }
#define ALL_PLANES { &test_setup[0], &test_setup[1], &test_setup[2], \
		     &test_setup[3]}

static const struct test_case tests[] = {
	{
		.name = "empty",
		.needs_composition = false,
	},
	{
		.name = "simple-1x-fail",
		.needs_composition = true,
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				.compat = { NULL },
				.result = NULL,
			},
		},
	},
	{
		.name = "simple-1x",
		.needs_composition = false,
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				.compat = { PRIMARY_PLANE },
				.result = PRIMARY_PLANE,
			},
		},
	},
	{
		.name = "simple-1x-no-composition",
		.needs_composition = true,
		/**
		 * Given
		*/
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				.compat = { PRIMARY_PLANE },
				.result = PRIMARY_PLANE,
			},
						{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.compat = { PRIMARY_PLANE },
				.result = NULL,
			},
		},
	},
	{
		.name = "simple-3x",
		.needs_composition = false,
		/*
		 * Underlay allocation will place opaque layer underneath
		 * the composition layer.
		*/
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
			{
				.width = 100,
				.height = 100,
				.compat = { CURSOR_PLANE },
				.result = CURSOR_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.compat = NON_CURSOR_PLANES,
				.result = PRIMARY_PLANE,
			},
		},
	},
	{
		.name = "simple-3x-all-alpha",
		.needs_composition = false,
		/*
		 * No layers can be underlay'd since they all have alpha
		 * channel. Fallback to overlay-only.
		*/
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				// .composition = true,
				.compat = NON_CURSOR_PLANES,
				.result = PRIMARY_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.compat = { CURSOR_PLANE },
				.result = CURSOR_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
		},
	},
	{
		.name = "zpos-2x-fail",
		.needs_composition = true,
		.layers = {
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 1 }},
				.compat = { CURSOR_PLANE },
				.result = NULL,
			},
			{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.props = {{ "zpos", 2 }},
				.compat = { PRIMARY_PLANE },
				.result = PRIMARY_PLANE,
			},
		},
	},
	{
		.name = "zpos-3x",
		.needs_composition = false,
		/*
		 * The highest z-pos layer that is compatible with CURSOR will
		 * be assigned.
		*/
		.layers = {
			{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.props = {{ "zpos", 1 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 2 }},
				.compat = NON_CURSOR_PLANES,
				.result = PRIMARY_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 3 }},
				.compat = FIRST_3_SECONDARY_PLANES,
				.result = CURSOR_PLANE,
			},
		},
	},
	{
		.name = "zpos-4x-partial-undefined-overlay",
		.needs_composition = true,
		/**
		 * Since the two OVERLAY planes have undefined z-ordering, they
		 * cannot be enabled at the same time. Test that we cannot
		 * enable both OVERLAYs using the composition layer, and the
		 * overlay layer immediately above it
		*/
		.layers = {
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 4 }},
				.compat = ALL_PLANES,
				.result = CURSOR_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 3 }},
				.compat = ALL_PLANES,
				.result = NULL,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 2 }},
				.compat = ALL_PLANES,
				.result = PRIMARY_PLANE,
			},
			{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.props = {{ "zpos", 1 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
		},
	},
	{
		.name = "zpos-4x-partial-undefined-underlay",
		.needs_composition = true,
		/**
		 * Same as above, except test using the composition layer, and
		 * the underlay layer immediately below it.
		*/
		.layers = {
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 4 }},
				.compat = ALL_PLANES,
				.result = CURSOR_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 3 }},
				.compat = ALL_PLANES,
				.result = PRIMARY_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = false,
				.no_alpha = true,
				.props = {{ "zpos", 2 }},
				.compat = ALL_PLANES,
				.result = NULL,
			},
			{
				.width = 1920,
				.height = 1080,
				.composition = true,
				.props = {{ "zpos", 1 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
		},
	},
	{
		.name = "zpos-4x-undefined-underlay",
		.needs_composition = false,
		/**
		 * Same as above, except test using the composition layer, and
		 * the underlay layer immediately below it.
		*/
		.layers = {
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 4 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
			{
				.x = 100,
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 3 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE,
			},
			{
				.width = 100,
				.height = 100,
				.is_underlay = true,
				.no_alpha = true,
				.props = {{ "zpos", 2 }},
				.compat = NON_CURSOR_PLANES,
				.result = PRIMARY_PLANE,
			},
			{
				.width = 1920,
				.height = 1080,
				.props = {{ "zpos", 1 }},
				.composition = true,
				.compat = ALL_PLANES,
				.result = CURSOR_PLANE,
			},
		},
	},
	{
		.name = "zpos-4x-undefined-overlay",
		.needs_composition = false,
		/**
		 * Same as above, except test using the composition layer, and
		 * the underlay layer immediately below it.
		*/
		.layers = {
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 4 }},
				.compat = ALL_PLANES,
				.result = CURSOR_PLANE,
			},
			{
				.x = 100,
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 3 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE_B,
			},
			{
				.width = 100,
				.height = 100,
				.props = {{ "zpos", 2 }},
				.compat = NON_CURSOR_PLANES,
				.result = OVERLAY_PLANE,
			},
			{
				.width = 1920,
				.height = 1080,
				.props = {{ "zpos", 1 }},
				.compat = ALL_PLANES,
				.result = PRIMARY_PLANE,
			},
		},
	},
};

static void
run_test(const struct test_case *test)
{
	size_t i, j, len;
	ssize_t plane_index_got, plane_index_want;
	struct liftoff_mock_plane *mock_planes[64];
	struct liftoff_mock_plane *mock_plane;
	const struct test_layer *test_layer;
	int drm_fd;
	struct liftoff_device *device;
	struct liftoff_output *output;
	struct liftoff_layer *layers[64];
	struct liftoff_plane *plane;
	struct liftoff_init_opts opts = { .punchthru_supported = true };
	drmModeAtomicReq *req;
	bool ok;
	int ret;
	uint32_t plane_id;

	liftoff_mock_require_primary_plane = true;

	for (i = 0; i < test_setup_len; i++) {
		mock_planes[i] = liftoff_mock_drm_create_plane(test_setup[i].type);
	}

	drm_fd = liftoff_mock_drm_open();
	device = liftoff_device_create(drm_fd, &opts);
	assert(device != NULL);

	liftoff_device_register_all_planes(device);

	output = liftoff_output_create(device, liftoff_mock_drm_crtc_id);
	for (i = 0; test->layers[i].width > 0; i++) {
		test_layer = &test->layers[i];

		layers[i] = add_layer(output, test_layer->x, test_layer->y,
				      test_layer->width, test_layer->height,
				      (test_layer->no_alpha ?
				       DRM_FORMAT_XRGB8888 :
				       DRM_FORMAT_ARGB8888));

		len = sizeof(test_layer->props) / sizeof(test_layer->props[0]);
		for (j = 0; j < len && test_layer->props[j].name != NULL; j++) {
			liftoff_layer_set_property(layers[i],
						   test_layer->props[j].name,
						   test_layer->props[j].value);
		}

		if (test_layer->composition) {
			liftoff_output_set_composition_layer(output, layers[i]);
		}
		if (test_layer->force_composited) {
			liftoff_layer_set_fb_composited(layers[i]);
		}

		len = sizeof(test_layer->compat) / sizeof(test_layer->compat[0]);
		for (j = 0; j < len && test_layer->compat[j] != NULL; j++) {
			mock_plane = mock_planes[test_layer->compat[j] -
						 test_setup];
			liftoff_mock_plane_add_compatible_layer(mock_plane,
								layers[i]);
		}
	}

	req = drmModeAtomicAlloc();
	ret = liftoff_output_apply(output, req, 0);
	assert(ret == 0);
	ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
	assert(ret == 0);
	drmModeAtomicFree(req);

	ok = true;
	for (i = 0; test->layers[i].width > 0; i++) {
		plane = liftoff_layer_get_plane(layers[i]);
		mock_plane = NULL;
		if (plane != NULL) {
			plane_id = liftoff_plane_get_id(plane);
			mock_plane = liftoff_mock_drm_get_plane(plane_id);
		}
		plane_index_got = -1;
		for (j = 0; j < test_setup_len; j++) {
			if (mock_planes[j] == mock_plane) {
				plane_index_got = (ssize_t)j;
				break;
			}
		}
		assert(mock_plane == NULL || plane_index_got >= 0);

		fprintf(stderr, "layer %zu got assigned to plane %d\n",
			i, (int)plane_index_got);

		plane_index_want = -1;
		if (test->layers[i].result != NULL) {
			plane_index_want = test->layers[i].result - test_setup;
		}

		if (plane_index_got != plane_index_want) {
			fprintf(stderr, "  ERROR: want plane %d\n",
				(int)plane_index_want);
			ok = false;
		}

		if (test->layers[i].is_underlay !=
		    liftoff_layer_is_underlay(layers[i])) {
			fprintf(stderr, "  ERROR: Layer %zu is underlay=%d, "
				"expected =%d\n",
				i, liftoff_layer_is_underlay(layers[i]),
				test->layers[i].is_underlay);
			ok = false;
		}
	}
	assert(ok);

	assert(test->needs_composition ==
	       liftoff_output_needs_composition(output));

	liftoff_output_destroy(output);
	liftoff_device_destroy(device);
	close(drm_fd);
}

/**
 * The most basic case for underlay is the composition layer -> PRIMARY plane
 * allocation.
 *
 * TODO: Allow underlay to fallback to overlay strategy if no composition layer
 * is provided.
*/
static void
test_basic(void)
{
	struct liftoff_mock_plane *mock_plane;
	int drm_fd;
	struct liftoff_device *device;
	struct liftoff_output *output;
	struct liftoff_layer *layer;
	struct liftoff_init_opts opts = { .punchthru_supported = false };
	drmModeAtomicReq *req;
	int ret;

	mock_plane = liftoff_mock_drm_create_plane(DRM_PLANE_TYPE_PRIMARY);

	drm_fd = liftoff_mock_drm_open();
	device = liftoff_device_create(drm_fd, &opts);
	assert(device != NULL);

	liftoff_device_register_all_planes(device);

	output = liftoff_output_create(device, liftoff_mock_drm_crtc_id);
	layer = add_layer(output, 0, 0, 1920, 1080, DRM_FORMAT_ARGB8888);

	liftoff_mock_plane_add_compatible_layer(mock_plane, layer);

	req = drmModeAtomicAlloc();
	ret = liftoff_output_apply(output, req, 0);
	assert(ret == 0);
	ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
	assert(ret == 0);
	assert(liftoff_mock_plane_get_layer(mock_plane) == layer);
	drmModeAtomicFree(req);

	liftoff_device_destroy(device);
	close(drm_fd);
}

/* Checks that the library doesn't allocate a plane for a layer without FB_ID
 * set. */
static void
test_no_props_fail(void)
{
	struct liftoff_mock_plane *mock_plane;
	int drm_fd;
	struct liftoff_device *device;
	struct liftoff_output *output;
	struct liftoff_layer *layer;
	struct liftoff_init_opts opts = { .punchthru_supported = true };
	drmModeAtomicReq *req;
	int ret;

	mock_plane = liftoff_mock_drm_create_plane(DRM_PLANE_TYPE_PRIMARY);

	drm_fd = liftoff_mock_drm_open();
	device = liftoff_device_create(drm_fd, &opts);
	assert(device != NULL);

	liftoff_device_register_all_planes(device);

	output = liftoff_output_create(device, liftoff_mock_drm_crtc_id);
	layer = liftoff_layer_create(output);

	liftoff_output_set_composition_layer(output, layer);
	liftoff_mock_plane_add_compatible_layer(mock_plane, layer);

	req = drmModeAtomicAlloc();
	ret = liftoff_output_apply(output, req, 0);
	assert(ret == 0);
	ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
	assert(ret == 0);
	assert(liftoff_mock_plane_get_layer(mock_plane) == NULL);
	drmModeAtomicFree(req);

	liftoff_device_destroy(device);
	close(drm_fd);
}

/* Checks that the library doesn't fallback to composition when a layer doesn't
 * have a FB. */
static void
test_composition_no_props(void)
{
	struct liftoff_mock_plane *mock_plane;
	int drm_fd;
	struct liftoff_device *device;
	struct liftoff_output *output;
	struct liftoff_layer *composition_layer, *layer_with_fb,
			     *layer_without_fb;
	struct liftoff_init_opts opts = { .punchthru_supported = true };
	drmModeAtomicReq *req;
	int ret;

	mock_plane = liftoff_mock_drm_create_plane(DRM_PLANE_TYPE_PRIMARY);

	drm_fd = liftoff_mock_drm_open();
	device = liftoff_device_create(drm_fd, &opts);
	assert(device != NULL);

	liftoff_device_register_all_planes(device);

	output = liftoff_output_create(device, liftoff_mock_drm_crtc_id);
	composition_layer = add_layer(output, 0, 0, 1920, 1080,
				      DRM_FORMAT_ARGB8888);
	layer_with_fb = add_layer(output, 0, 0, 1920, 1080,
				  DRM_FORMAT_ARGB8888);
	layer_without_fb = liftoff_layer_create(output);
	(void)layer_with_fb;

	liftoff_output_set_composition_layer(output, composition_layer);

	liftoff_mock_plane_add_compatible_layer(mock_plane, composition_layer);
	liftoff_mock_plane_add_compatible_layer(mock_plane, layer_without_fb);
	liftoff_mock_plane_add_compatible_layer(mock_plane, layer_with_fb);

	req = drmModeAtomicAlloc();
	ret = liftoff_output_apply(output, req, 0);
	assert(ret == 0);
	ret = drmModeAtomicCommit(drm_fd, req, 0, NULL);
	assert(ret == 0);
	assert(liftoff_mock_plane_get_layer(mock_plane) == layer_with_fb);
	drmModeAtomicFree(req);

	liftoff_device_destroy(device);
	close(drm_fd);
}

int
main(int argc, char *argv[])
{
	const char *test_name;
	size_t i;

	liftoff_log_set_priority(LIFTOFF_DEBUG);

	if (argc != 2) {
		fprintf(stderr, "usage: %s <test-name>\n", argv[0]);
		return 1;
	}
	test_name = argv[1];

	if (strcmp(test_name, "basic") == 0) {
		test_basic();
		return 0;
	} else if (strcmp(test_name, "no-props-fail") == 0) {
		test_no_props_fail();
		return 0;
	} else if (strcmp(test_name, "composition-no-props") == 0) {
		test_composition_no_props();
		return 0;
	}

	for (i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
		if (strcmp(tests[i].name, test_name) == 0) {
			run_test(&tests[i]);
			return 0;
		}
	}

	fprintf(stderr, "no such test: %s\n", test_name);
	return 1;
}
