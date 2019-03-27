// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common defines and enums for XRT.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Which blend mode does the device support, used as both a bitfield and value.
 *
 * @ingroup xrt_iface
 */
enum xrt_blend_mode
{
	// clang-format off
	XRT_BLEND_MODE_OPAQUE      = 1 << 0,
	XRT_BLEND_MODE_ADDITIVE    = 1 << 1,
	XRT_BLEND_MODE_ALPHA_BLEND = 1 << 2,
	// clang-format on
};

/*!
 * Which distortion model does the device expose,
 * used both as a bitfield and value.
 */
enum xrt_distortion_model
{
	// clang-format off
	XRT_DISTORTION_MODEL_NONE      = 1 << 0,
	XRT_DISTORTION_MODEL_PANOTOOLS = 1 << 1,
	XRT_DISTORTION_MODEL_VIVE      = 1 << 2,
    XRT_DISTORTION_MODEL_MESHUV    = 1 << 4,
	// clang-format on
};

/*!
 * A quaternion with single floats.
 *
 * @ingroup xrt_iface math
 */
struct xrt_quat
{
	float x;
	float y;
	float z;
	float w;
};

/*!
 * A 3 element vector with single floats.
 *
 * @ingroup xrt_iface math
 */
struct xrt_vec3
{
	float x;
	float y;
	float z;
};

/*!
 * A 2 element vector with single floats.
 *
 * @ingroup xrt_iface math
 */
struct xrt_vec2
{
	float x;
	float y;
};

/*!
 * A pose composed of a position and orientation.
 *
 * @see xrt_qaut
 * @see xrt_vec3
 * @ingroup xrt_iface math
 */
struct xrt_pose
{
	struct xrt_quat orientation;
	struct xrt_vec3 position;
};

/*!
 * Describes a projection matrix fov.
 *
 * @ingroup xrt_iface math
 */
struct xrt_fov
{
	float angle_left;
	float angle_right;
	float angle_up;
	float angle_down;
};

/*!
 * A tightly packed 2x2 matrix of floats.
 *
 * @ingroup xrt_iface math
 */
struct xrt_matrix_2x2
{
	union {
		float v[4];
		struct xrt_vec2 vecs[2];
	};
};

/*!
 * A tightly packed 4x4 matrix of floats.
 *
 * @ingroup xrt_iface math
 */
struct xrt_matrix_4x4
{
	float v[16];
};

/*!
 * A range of API versions supported.
 *
 * @ingroup xrt_iface math
 */
struct xrt_api_requirements
{
	uint32_t min_major;
	uint32_t min_minor;
	uint32_t min_patch;

	uint32_t max_major;
	uint32_t max_minor;
	uint32_t max_patch;
};

/*!
 * Flags of which components of a @ref xrt_space_relation is valid.
 *
 * @see xrt_space_relation
 * @ingroup xrt_iface math
 */
enum xrt_space_relation_flags
{
	XRT_SPACE_RELATION_ORIENTATION_VALID_BIT = 0x00000001,
	XRT_SPACE_RELATION_POSITION_VALID_BIT = 0x00000002,
	XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT = 0x00000004,
	XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT = 0x00000008,
	XRT_SPACE_RELATION_LINEAR_ACCELERATION_VALID_BIT = 0x00000010,
	XRT_SPACE_RELATION_ANGULAR_ACCELERATION_VALID_BIT = 0x00000020,
	XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT = 0x00000040,
	XRT_SPACE_RELATION_POSITION_TRACKED_BIT = 0x00000080,
	XRT_SPACE_RELATION_BITMASK_ALL =
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_LINEAR_ACCELERATION_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_ACCELERATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT,
	XRT_SPACE_RELATION_BITMASK_NONE = 0
};

/*!
 * A relation with two spaces, includes velocity and acceleration.
 *
 * @see xrt_quat
 * @see xrt_vec3
 * @see xrt_pose
 * @see xrt_space_relation_flags
 * @ingroup xrt_iface math
 */
struct xrt_space_relation
{
	enum xrt_space_relation_flags relation_flags;
	struct xrt_pose pose;
	struct xrt_vec3 linear_velocity;
	struct xrt_vec3 angular_velocity;
	struct xrt_vec3 linear_acceleration;
	struct xrt_vec3 angular_acceleration;
};

#ifdef __cplusplus
}
#endif
