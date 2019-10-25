// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Handling of calibration data from PS Move and PSVR devices
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */

#include "u_pscalib.h"

#include <string.h>

struct ps_axis_min_max
{
	struct xrt_vec3 min_val;
	struct xrt_vec3 max_val;
};

struct ps_calib_data
{
	struct ps_axis_min_max vals[3];
};

static struct ps_calib_data
load_calib(float vals[18])
{
	struct ps_calib_data ret = {0};
	// max y
	memcpy(&(ret.vals[1].max_val), &(vals[0]), sizeof(struct xrt_vec3));
	// min x
	memcpy(&(ret.vals[0].min_val), &(vals[3]), sizeof(struct xrt_vec3));
	// min y
	memcpy(&(ret.vals[1].min_val), &(vals[6]), sizeof(struct xrt_vec3));
	// max x
	memcpy(&(ret.vals[0].max_val), &(vals[9]), sizeof(struct xrt_vec3));
	// max z
	memcpy(&(ret.vals[2].max_val), &(vals[12]), sizeof(struct xrt_vec3));
	// min z
	memcpy(&(ret.vals[2].min_val), &(vals[15]), sizeof(struct xrt_vec3));
	return ret;
}

#define PER_AXIS(_) _(0, x) _(1, y) _(2, z)

#define CREATE_AXIS_CALIB(IDX, AXIS)                                           \
	out_offset_scale->scale.AXIS =                                         \
	    2.f / (data.vals[IDX].max_val.AXIS - data.vals[IDX].min_val.AXIS); \
	out_offset_scale->offset.AXIS =                                        \
	    (data.vals[IDX].max_val.AXIS + data.vals[IDX].min_val.AXIS) / 2.f;

void
u_load_ps_calib(float vals[18], struct xrt_offset_scale3 *out_offset_scale)
{
	struct ps_calib_data data = load_calib(vals);
	PER_AXIS(CREATE_AXIS_CALIB);
}

void
u_apply_offset_scale(const struct xrt_offset_scale3 *offset_scale,
                     const struct xrt_vec3 *input,
                     struct xrt_vec3 *out_vec)
{
#define MULT_ADD(_, AXIS)                                                      \
	out_vec->AXIS = (input->AXIS - offset_scale->offset.AXIS) *            \
	                offset_scale->scale.AXIS;

	PER_AXIS(MULT_ADD);
}
