// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Misc helpers for device drivers.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "math/m_api.h"
#include "util/u_device.h"
#include "util/u_misc.h"


/*
 *
 * Matricies.
 *
 */

const struct xrt_matrix_2x2 u_device_rotation_right = {{
    .vecs =
        {
            {0, 1},
            {-1, 0},
        },
}};


const struct xrt_matrix_2x2 u_device_rotation_left = {{
    .vecs =
        {
            {0, -1},
            {1, 0},
        },
}};

const struct xrt_matrix_2x2 u_device_rotation_ident = {{
    .vecs =
        {
            {1, 0},
            {0, 1},
        },
}};

const struct xrt_matrix_2x2 u_device_rotation_180 = {{
    .vecs =
        {
            {-1, 0},
            {0, -1},
        },
}};


/*
 *
 * Print helpers.
 *
 */

#define PRINT(...) fprintf(stderr, __VA_ARGS__)

#define PRINT_STR(name, val) PRINT("\t%s = %s\n", name, val)

#define PRINT_INT(name, val) PRINT("\t%s = %u\n", name, val)

#define PRINT_MM(name, val)                                                    \
	PRINT("\t%s = %f (%i.%02imm)\n", name, val, (int32_t)(val * 1000.f),   \
	      abs((int32_t)(val * 100000.f)) % 100)

#define PRINT_ANGLE(name, val)                                                 \
	PRINT("\t%s = %f (%i°)\n", name, val, (int32_t)(val * (180 / M_PI)))

#define PRINT_MAT2X2(name, rot)                                                \
	PRINT("\t%s = {%f, %f} {%f, %f}\n", name, rot.v[0], rot.v[1],          \
	      rot.v[2], rot.v[3])

/*!
 * Dump the device config to stderr.
 */
void
u_device_dump_config(struct xrt_device *xdev,
                     const char *prefix,
                     const char *prod)
{
	// clang-format off
	fprintf(stderr, "%s - device_setup\n", prefix);
	PRINT_STR(   "prod", prod);
	if (xdev->hmd != NULL) {
		PRINT_INT(   "screens[0].w_pixels ", xdev->hmd->screens[0].w_pixels);
		PRINT_INT(   "screens[0].h_pixels ", xdev->hmd->screens[0].h_pixels);
//		PRINT_MM(    "info.display.w_meters", info.display.w_meters);
//		PRINT_MM(    "info.display.h_meters", info.display.h_meters);
		PRINT_INT(   "views[0].viewport.x_pixels   ", xdev->hmd->views[0].viewport.x_pixels);
		PRINT_INT(   "views[0].viewport.y_pixels   ", xdev->hmd->views[0].viewport.y_pixels);
		PRINT_INT(   "views[0].viewport.w_pixels   ", xdev->hmd->views[0].viewport.w_pixels);
		PRINT_INT(   "views[0].viewport.h_pixels   ", xdev->hmd->views[0].viewport.h_pixels);
		PRINT_INT(   "views[0].display.w_pixels    ", xdev->hmd->views[0].display.w_pixels);
		PRINT_INT(   "views[0].display.h_pixels    ", xdev->hmd->views[0].display.h_pixels);
		PRINT_MM(    "views[0].display.w_meters    ", xdev->hmd->views[0].display.w_meters);
		PRINT_MM(    "views[0].display.h_meters    ", xdev->hmd->views[0].display.h_meters);
		PRINT_MM(    "views[0].lens_center.x_meters", xdev->hmd->views[0].lens_center.x_meters);
		PRINT_MM(    "views[0].lens_center.y_meters", xdev->hmd->views[0].lens_center.y_meters);
		PRINT_MAT2X2("views[0].rot            ", xdev->hmd->views[0].rot);
		PRINT_ANGLE( "views[0].fov.angle_left ", xdev->hmd->views[0].fov.angle_left);
		PRINT_ANGLE( "views[0].fov.angle_right", xdev->hmd->views[0].fov.angle_right);
		PRINT_ANGLE( "views[0].fov.angle_up   ", xdev->hmd->views[0].fov.angle_up);
		PRINT_ANGLE( "views[0].fov.angle_down ", xdev->hmd->views[0].fov.angle_down);
//		PRINT_ANGLE( "info.views[0].fov       ", info.views[0].fov);
		PRINT_INT(   "views[1].viewport.x_pixels   ", xdev->hmd->views[1].viewport.x_pixels);
		PRINT_INT(   "views[1].viewport.y_pixels   ", xdev->hmd->views[1].viewport.y_pixels);
		PRINT_INT(   "views[1].viewport.w_pixels   ", xdev->hmd->views[1].viewport.w_pixels);
		PRINT_INT(   "views[1].viewport.h_pixels   ", xdev->hmd->views[1].viewport.h_pixels);
		PRINT_INT(   "views[1].display.w_pixels    ", xdev->hmd->views[1].display.w_pixels);
		PRINT_INT(   "views[1].display.h_pixels    ", xdev->hmd->views[1].display.h_pixels);
		PRINT_MM(    "views[1].display.w_meters    ", xdev->hmd->views[1].display.w_meters);
		PRINT_MM(    "views[1].display.h_meters    ", xdev->hmd->views[1].display.h_meters);
		PRINT_MM(    "views[1].lens_center.x_meters", xdev->hmd->views[1].lens_center.x_meters);
		PRINT_MM(    "views[1].lens_center.y_meters", xdev->hmd->views[1].lens_center.y_meters);
		PRINT_MAT2X2("views[1].rot            ", xdev->hmd->views[1].rot);
		PRINT_ANGLE( "views[1].fov.angle_left ", xdev->hmd->views[1].fov.angle_left);
		PRINT_ANGLE( "views[1].fov.angle_right", xdev->hmd->views[1].fov.angle_right);
		PRINT_ANGLE( "views[1].fov.angle_up   ", xdev->hmd->views[1].fov.angle_up);
		PRINT_ANGLE( "views[1].fov.angle_down ", xdev->hmd->views[1].fov.angle_down);
//		PRINT_ANGLE( "info.views[1].fov       ", info.views[0].fov);
	}
	// clang-format on
}


/*
 *
 * Helper setup functions.
 *
 */

bool
u_device_setup_split_side_by_side(struct xrt_device *xdev,
                                  const struct u_device_simple_info *info)
{
	uint32_t w_pixels = info->display.w_pixels / 2;
	uint32_t h_pixels = info->display.h_pixels;
	float w_meters = info->display.w_meters / 2;
	float h_meters = info->display.h_meters;

	float lens_center_x_meters[2] = {
	    w_meters - info->lens_horizontal_separation_meters / 2.0f,
	    info->lens_horizontal_separation_meters / 2.0f,
	};

	float lens_center_y_meters[2] = {
	    info->lens_vertical_position_meters,
	    info->lens_vertical_position_meters,
	};

	// Common
	xdev->hmd->blend_mode = XRT_BLEND_MODE_OPAQUE;
	if (xdev->hmd->distortion.models == NULL) {
		xdev->hmd->distortion.models = XRT_DISTORTION_MODEL_NONE;
		xdev->hmd->distortion.preferred = XRT_DISTORTION_MODEL_NONE;
	}
	xdev->hmd->screens[0].w_pixels = info->display.w_pixels;
	xdev->hmd->screens[0].h_pixels = info->display.h_pixels;

	// Left
	xdev->hmd->views[0].display.w_meters = w_meters;
	xdev->hmd->views[0].display.h_meters = h_meters;
	xdev->hmd->views[0].lens_center.x_meters = lens_center_x_meters[0];
	xdev->hmd->views[0].lens_center.y_meters = lens_center_y_meters[0];
	xdev->hmd->views[0].display.w_pixels = w_pixels;
	xdev->hmd->views[0].display.h_pixels = h_pixels;
	xdev->hmd->views[0].viewport.x_pixels = 0;
	xdev->hmd->views[0].viewport.y_pixels = 0;
	xdev->hmd->views[0].viewport.w_pixels = w_pixels;
	xdev->hmd->views[0].viewport.h_pixels = h_pixels;
	xdev->hmd->views[0].rot = u_device_rotation_ident;

	// Right
	xdev->hmd->views[1].display.w_meters = w_meters;
	xdev->hmd->views[1].display.h_meters = h_meters;
	xdev->hmd->views[1].lens_center.x_meters = lens_center_x_meters[1];
	xdev->hmd->views[1].lens_center.y_meters = lens_center_y_meters[1];
	xdev->hmd->views[1].display.w_pixels = w_pixels;
	xdev->hmd->views[1].display.h_pixels = h_pixels;
	xdev->hmd->views[1].viewport.x_pixels = w_pixels;
	xdev->hmd->views[1].viewport.y_pixels = 0;
	xdev->hmd->views[1].viewport.w_pixels = w_pixels;
	xdev->hmd->views[1].viewport.h_pixels = h_pixels;
	xdev->hmd->views[1].rot = u_device_rotation_ident;

	{
		/* right eye */
		if (!math_compute_fovs(w_meters, lens_center_x_meters[1],
		                       info->views[1].fov, h_meters,
		                       lens_center_y_meters[1], 0,
		                       &xdev->hmd->views[1].fov)) {
			return false;
		}
	}
	{
		/* left eye - just mirroring right eye now */
		xdev->hmd->views[0].fov.angle_up =
		    xdev->hmd->views[1].fov.angle_up;
		xdev->hmd->views[0].fov.angle_down =
		    xdev->hmd->views[1].fov.angle_down;

		xdev->hmd->views[0].fov.angle_left =
		    -xdev->hmd->views[1].fov.angle_right;
		xdev->hmd->views[0].fov.angle_right =
		    -xdev->hmd->views[1].fov.angle_left;
	}

	return true;
}

void *
u_device_allocate(enum u_device_alloc_flags flags,
                  size_t size,
                  size_t num_inputs,
                  size_t num_outputs)
{
	bool alloc_hmd = (flags & U_DEVICE_ALLOC_HMD) != 0;
	bool alloc_tracking = (flags & U_DEVICE_ALLOC_TRACKING_NONE) != 0;

	size_t total_size = size;

	// Inputs
	size_t offset_inputs = total_size;
	total_size += num_inputs * sizeof(struct xrt_input);

	// Outputs
	size_t offset_outputs = total_size;
	total_size += num_outputs * sizeof(struct xrt_output);

	// HMD
	size_t offset_hmd = total_size;
	total_size += alloc_hmd ? sizeof(struct xrt_hmd_parts) : 0;

	// Tracking
	size_t offset_tracking = total_size;
	total_size += alloc_tracking ? sizeof(struct xrt_tracking_origin) : 0;

	// Do the allocation
	char *ptr = U_TYPED_ARRAY_CALLOC(char, total_size);
	struct xrt_device *xdev = (struct xrt_device *)ptr;

	if (num_inputs > 0) {
		xdev->num_inputs = num_inputs;
		xdev->inputs = (struct xrt_input *)(ptr + offset_inputs);
	}

	if (num_outputs > 0) {
		xdev->num_outputs = num_outputs;
		xdev->outputs = (struct xrt_output *)(ptr + offset_outputs);
	}

	if (alloc_hmd) {
		xdev->hmd = (struct xrt_hmd_parts *)(ptr + offset_hmd);
	}

	if (alloc_tracking) {
		xdev->tracking_origin =
		    (struct xrt_tracking_origin *)(ptr + offset_tracking);
		xdev->tracking_origin->type = XRT_TRACKING_TYPE_NONE;
		xdev->tracking_origin->offset.orientation.w = 1.0f;
		snprintf(xdev->tracking_origin->name, XRT_TRACKING_NAME_LEN,
		         "%s", "No tracking");
	}

	return xdev;
}
