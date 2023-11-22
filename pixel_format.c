#include <drm_fourcc.h>
#include <stddef.h>
#include "pixel_format.h"

static const struct liftoff_pixel_format_info pixel_format_info[] = {
	{
		.drm_format = DRM_FORMAT_XRGB8888,
		.has_alpha = false,
	},
	{
		.drm_format = DRM_FORMAT_NV12,
		.has_alpha = false,
	},
	{
		.drm_format = DRM_FORMAT_P010,
		.has_alpha = false,
	}
};

static const size_t pixel_format_info_size =
	sizeof(pixel_format_info) / sizeof(pixel_format_info[0]);

bool pixel_format_has_alpha(uint32_t fmt)
{
	size_t i;
	for (i = 0; i < pixel_format_info_size; i++) {
		if (pixel_format_info[i].drm_format == fmt) {
			return pixel_format_info[i].has_alpha;
		}
	}
	// Default is true: transparency imposes more restrictions for liftoff
	return true;
}
