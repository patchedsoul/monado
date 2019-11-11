// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS VR tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_defines.h"
#include "xrt/xrt_tracking.h"

#include "util/u_time.h"

#include <memory>

namespace xrt_fusion {
enum class PSVRMarker
{
	LEFT,
	TOP_LEFT,
	BOTTOM_LEFT,
	CENTER,
	TOP_RIGHT,
	BOTTOM_RIGHT,
	RIGHT,
	REAR_LEFT,
	REAR_RIGHT,
};
class PSVRFusionInterface
{
public:
	static std::unique_ptr<PSVRFusionInterface>
	create();
	virtual ~PSVRFusionInterface() = default;

	/*!
	 * @brief If you've lost sight of the position tracking and won't even
	 * enter another function in this class.
	 */
	virtual void
	clear_position_tracked_flag() = 0;

	virtual void
	process_imu_data(
	    time_duration_ns delta_ns,
	    const struct xrt_tracking_sample *sample,
	    const struct xrt_vec3 *orientation_variance_optional) = 0;
	virtual void
	process_3d_vision_data(time_duration_ns delta_ns,
	                       const struct xrt_vec3 *position,
	                       const struct xrt_vec3 *variance_optional,
	                       const struct xrt_vec3 *lever_arm_optional,
	                       float residual_limit) = 0;

	virtual void
	get_prediction(timepoint_ns when_ns,
	               struct xrt_space_relation *out_relation) = 0;
};
} // namespace xrt_fusion
