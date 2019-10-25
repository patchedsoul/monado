// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Handling of calibration data from PS Move and PSVR devices
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */
#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

//! To apply this, use (x - offset) * scale componentwise
struct xrt_offset_scale3
{
	struct xrt_vec3 offset;
	struct xrt_vec3 scale;
};

/*!
 * @brief Takes the 18 floats of the PS Move/VR accel calibration data, and
 * outputs an array of offsets and scale.
 *
 * @ingroup aux_util
 */
void
u_load_ps_calib(float vals[18], struct xrt_offset_scale3 *out_offset_scale);


/*!
 * @brief Apply calibration to the input vector.
 *
 * @ingroup aux_util
 */
void
u_apply_offset_scale(const struct xrt_offset_scale3 *offset_scale,
                     const struct xrt_vec3 *input,
                     struct xrt_vec3 *out_vec);

#ifdef __cplusplus
}
#endif
