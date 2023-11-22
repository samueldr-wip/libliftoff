#ifndef PIXEL_FORMAT_H
#define PIXEL_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

struct liftoff_pixel_format_info {
	uint32_t drm_format;
	bool has_alpha;
};

bool pixel_format_has_alpha(uint32_t fmt);

#endif
