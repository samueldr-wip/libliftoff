#define _POSIX_C_SOURCE 200809L
#include <fcntl.h>
#include <libliftoff.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <sys/mman.h>
#include <drm_fourcc.h>

static drmModeConnector *pick_connector(int drm_fd, drmModeRes *drm_res)
{
	int i;
	drmModeConnector *connector;

	for (i = 0; i < drm_res->count_connectors; i++) {
		connector = drmModeGetConnector(drm_fd, drm_res->connectors[i]);
		if (connector->connection == DRM_MODE_CONNECTED) {
			return connector;
		}
		drmModeFreeConnector(connector);
	}

	return NULL;
}

static drmModeCrtc *pick_crtc(int drm_fd, drmModeRes *drm_res,
			      drmModeConnector *connector)
{
	drmModeEncoder *enc;
	uint32_t crtc_id;

	/* TODO: don't blindly use current CRTC */
	enc = drmModeGetEncoder(drm_fd, connector->encoder_id);
	crtc_id = enc->crtc_id;
	drmModeFreeEncoder(enc);

	return drmModeGetCrtc(drm_fd, crtc_id);
}

static void disable_all_crtcs_except(int drm_fd, drmModeRes *drm_res,
				     uint32_t crtc_id)
{
	int i;

	for (i = 0; i < drm_res->count_crtcs; i++) {
		if (drm_res->crtcs[i] == crtc_id) {
			continue;
		}
		drmModeSetCrtc(drm_fd, drm_res->crtcs[i],
			0, 0, 0, NULL, 0, NULL);
	}
}

static uint32_t create_argb_fb(int drm_fd, uint32_t width, uint32_t height,
			       uint32_t color, bool with_alpha)
{
	int ret;
	uint32_t fb_id;
	uint32_t *data;
	size_t i;

	struct drm_mode_create_dumb create = {
		.width = width,
		.height = height,
		.bpp = 32,
		.flags = 0,
	};
	ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create);
	if (ret < 0) {
		return 0;
	}

	uint32_t fmt = with_alpha ? DRM_FORMAT_ARGB8888 : DRM_FORMAT_XRGB8888;
	uint32_t handles[4] = { create.handle };
	uint32_t strides[4] = { create.pitch };
	uint32_t offsets[4] = { 0 };
	ret = drmModeAddFB2(drm_fd, width, height, fmt, handles, strides,
			    offsets, &fb_id, 0);
	if (ret < 0) {
		return 0;
	}

	struct drm_mode_map_dumb map = { .handle = create.handle };
	ret = drmIoctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map);
	if (ret < 0) {
		return 0;
	}

	data = mmap(0, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd,
		    map.offset);
	if (data == MAP_FAILED) {
		return 0;
	}

	for (i = 0; i < create.size / sizeof(uint32_t); i++) {
		data[i] = color;
	}

	munmap(data, create.size);
	return fb_id;
}

/* ARGB 8:8:8:8 */
static const uint32_t colors[] = {
	0xFFFF0000, /* red */
	0xFF00FF00, /* green */
	0xFF0000FF, /* blue */
	0xFFFFFF00, /* yellow */
};

static struct liftoff_layer *add_layer(int drm_fd, struct liftoff_output *output,
				       int x, int y, int width, int height,
				       bool with_alpha)
{
	static size_t color_idx = 0;
	uint32_t fb_id;
	struct liftoff_layer *layer;

	fb_id = create_argb_fb(drm_fd, width, height, colors[color_idx],
			       with_alpha);
	if (fb_id == 0) {
		fprintf(stderr, "failed to create framebuffer\n");
		return NULL;
	}
	printf("Created FB %d with size %dx%d\n", fb_id, width, height);
	color_idx = (color_idx + 1) % (sizeof(colors) / sizeof(colors[0]));

	layer = liftoff_layer_create(output);
	liftoff_layer_set_property(layer, "FB_ID", fb_id);
	liftoff_layer_set_property(layer, "CRTC_X", x);
	liftoff_layer_set_property(layer, "CRTC_Y", y);
	liftoff_layer_set_property(layer, "CRTC_W", width);
	liftoff_layer_set_property(layer, "CRTC_H", height);
	liftoff_layer_set_property(layer, "SRC_X", 0);
	liftoff_layer_set_property(layer, "SRC_Y", 0);
	liftoff_layer_set_property(layer, "SRC_W", width << 16);
	liftoff_layer_set_property(layer, "SRC_H", height << 16);

	return layer;
}

int main(int argc, char *argv[])
{
	int drm_fd;
	struct liftoff_display *display;
	drmModeRes *drm_res;
	drmModeCrtc *crtc;
	drmModeConnector *connector;
	struct liftoff_output *output;
	struct liftoff_layer *layers[4];
	drmModeAtomicReq *req;
	int ret;
	size_t i;

	drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
	if (drm_fd < 0) {
		perror("open");
		return 1;
	}

	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) < 0) {
		perror("drmSetClientCap(UNIVERSAL_PLANES)");
		return 1;
	}
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) < 0) {
		perror("drmSetClientCap(ATOMIC)");
		return 1;
	}

	display = liftoff_display_create(drm_fd);
	if (display == NULL) {
		perror("liftoff_display_create");
		return 1;
	}

	drm_res = drmModeGetResources(drm_fd);
	connector = pick_connector(drm_fd, drm_res);
	crtc = pick_crtc(drm_fd, drm_res, connector);
	disable_all_crtcs_except(drm_fd, drm_res, crtc->crtc_id);
	output = liftoff_output_create(display, crtc->crtc_id);
	drmModeFreeResources(drm_res);

	if (connector == NULL) {
		fprintf(stderr, "no connector found\n");
		return 1;
	}
	if (crtc == NULL || !crtc->mode_valid) {
		fprintf(stderr, "no CRTC found\n");
		return 1;
	}

	printf("Using connector %d, CRTC %d\n", connector->connector_id,
	       crtc->crtc_id);

	layers[0] = add_layer(drm_fd, output, 0, 0, crtc->mode.hdisplay,
			      crtc->mode.vdisplay, false);
	layers[1] = add_layer(drm_fd, output, 50, 50, 256, 256, true);
	layers[2] = add_layer(drm_fd, output, 300, 300, 128, 128, false);
	layers[3] = add_layer(drm_fd, output, 400, 400, 128, 128, true);

	liftoff_layer_set_property(layers[0], "zpos", 0);
	liftoff_layer_set_property(layers[1], "zpos", 1);
	liftoff_layer_set_property(layers[2], "zpos", 2);
	liftoff_layer_set_property(layers[3], "zpos", 3);

	req = drmModeAtomicAlloc();
	if (!liftoff_display_apply(display, req)) {
		perror("liftoff_display_commit");
		return 1;
	}

	ret = drmModeAtomicCommit(drm_fd, req, DRM_MODE_ATOMIC_NONBLOCK, NULL);
	if (ret < 0) {
		perror("drmModeAtomicCommit");
		return false;
	}

	for (i = 0; i < sizeof(layers) / sizeof(layers[0]); i++) {
		printf("Layer %zu got assigned to plane %u\n", i,
		       liftoff_layer_get_plane_id(layers[i]));
	}

	sleep(1);

	drmModeAtomicFree(req);
	for (i = 0; i < sizeof(layers) / sizeof(layers[0]); i++) {
		liftoff_layer_destroy(layers[i]);
	}
	liftoff_output_destroy(output);
	drmModeFreeCrtc(crtc);
	drmModeFreeConnector(connector);
	liftoff_display_destroy(display);
	return 0;
}