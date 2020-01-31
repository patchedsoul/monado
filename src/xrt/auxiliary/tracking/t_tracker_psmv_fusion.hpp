// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#ifndef __cplusplus
#error "This header is C++-only."
#endif

#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"

#include "util/u_time.h"

#include <memory>


namespace xrt_fusion {
class PSMVFusionInterface
{
public:
	static std::unique_ptr<PSMVFusionInterface>
	create();
	virtual ~PSMVFusionInterface() = default;

	/*!
	 * @brief If you've lost sight of the position tracking and won't even
	 * enter another function in this class.
	 */
	virtual void
	clear_position_tracked_flag() = 0;

	virtual void
	process_imu_data(
	    timepoint_ns timestamp_ns,
	    const struct xrt_tracking_sample *sample,
	    const struct xrt_vec3 *orientation_variance_optional) = 0;

	/*!
	 * @brief Incorporate vision-based tracking info.
	 *
	 * @param timestamp_ns Time the image was acquired.
	 * @param position The measured 3D position of the light-up sphere.
	 * @param variance_optional Measurement variance, if you want to
	 * override the default.
	 * @param lever_arm_optional Distance from the "origin" to the tracked
	 * location, if you wish to override the default.
	 * @param residual_limit The limit for residual (distance between
	 * expected and measured) that will trigger a tracker resete for
	 * inconsistency.
	 *
	 */
	virtual void
	process_3d_vision_data(timepoint_ns timestamp_ns,
	                       const struct xrt_vec3 *position,
	                       const struct xrt_vec3 *variance_optional,
	                       const struct xrt_vec3 *lever_arm_optional,
	                       float residual_limit) = 0;

	virtual void
	get_prediction(timepoint_ns when_ns,
	               struct xrt_space_relation *out_relation) = 0;
};
} // namespace xrt_fusion
