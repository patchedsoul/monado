// Copyright 2019, Collabora, Ltd.
// Copyright 2014, Kevin M. Godby
// Copyright 2014-2018, Sensics, Inc.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for an OSVR Hacker Dev Kit device.
 *
 * Based in part on the corresponding VRPN driver,
 * available under BSL-1.0.
 *
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Kevin M. Godby <kevin@godby.org>
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <type_traits>

#include <hidapi.h>

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "util/u_misc.h"
#include "util/u_device.h"

#include "hdk_device.h"


/**
 * A fixed-point to float conversion function.
 *
 * Values are signed, two's-complement, if the supplied integer is.
 *
 * The conversion is effectively from the fixed-point arithmetic type known
 * "unambiguously" as Q INTEGER_BITS.FRACTIONAL_BITS - the number of integer
 * bits is not inferred, though it is checked to ensure it adds up.
 *
 * @tparam INTEGER_BITS The number of bits devoted to the integer part.
 * @tparam FRACTIONAL_BITS The number of bits devoted to the fractional
 * part.
 * @tparam IntegerType The input integer type, typically deduced (do not need to
 * specify explicitly)
 * @param v An input "integer" that is actually a fixed-point value.
 *
 * INTEGER_BITS and FRACTIONAL_BITS must sum to 8 * sizeof(v), the bit width of
 * the input integer, for unsigned values, or to one less than that (for the
 * sign bit) for signed values.
 *
 * Based in part on the VRPN header vrpn_FixedPoint.h,
 * available under BSL-1.0.
 */
template <size_t INTEGER_BITS, size_t FRACTIONAL_BITS, typename IntegerType>
static inline float
fromFixedPoint(IntegerType v)
{
	constexpr size_t SIGN_BIT = std::is_signed<IntegerType>::value ? 1 : 0;
	static_assert(INTEGER_BITS + FRACTIONAL_BITS + SIGN_BIT ==
	                  8 * sizeof(IntegerType),
	              "INTEGER_BITS and FRACTIONAL_BITS, plus 1 for a sign bit "
	              "if applicable, must sum to the input "
	              "integer width, but do not.");
	return static_cast<float>(v) / (1 << FRACTIONAL_BITS);
}

static inline uint16_t
hdk_get_le_uint16(uint8_t *&bufPtr)
{
	assert(bufPtr != nullptr);
	uint16_t ret = static_cast<uint16_t>(*bufPtr) |
	               (static_cast<uint16_t>(*(bufPtr + 1)) << 8);
	bufPtr += 2;
	return ret;
}

static inline int16_t
hdk_get_le_int16(uint8_t *&bufPtr)
{
	return static_cast<int16_t>(hdk_get_le_uint16(bufPtr));
}

static void
hdk_device_destroy(struct xrt_device *xdev)
{
	struct hdk_device *hd = hdk_device(xdev);

	if (hd->dev != NULL) {
		hid_close(hd->dev);
		hd->dev = NULL;
	}

	free(hd);
}

static void
hdk_device_get_tracked_pose(struct xrt_device *xdev,
                            struct xrt_space_relation *out_relation)
{
	struct hdk_device *hd = hdk_device(xdev);

	uint8_t buffer[32];
	auto bytesRead = hid_read(hd->dev, &(buffer[0]), sizeof(buffer));
	if (bytesRead != 32 && bytesRead != 16) {
		HDK_DEBUG(hd, "Only got %d bytes", bytesRead);
		out_relation->relation_flags = XRT_SPACE_RELATION_BITMASK_NONE;
		return;
	}
	uint8_t *buf = &(buffer[0]);

#if 0
	uint8_t version = uint8_t(0x0f) & *buf;
	uint8_t hdmi_status = (uint8_t(0xf0) & *buf) >> 4;
#endif
	buf++;

	// HDMI status only valid in reports version 3.
	// Expecting either version 1 (100Hz) or 3 (400Hz):
	// https://github.com/OSVR/OSVR-HDK-MCU-Firmware/blob/master/Source%20code/Embedded/src/DeviceDrivers/BNO070_using_hostif.c#L511

	// Next byte is sequence number, ignore
	buf++;

	struct xrt_quat quat;
	quat.x = fromFixedPoint<1, 14>(hdk_get_le_int16(buf)) * -1;
	quat.y = fromFixedPoint<1, 14>(hdk_get_le_int16(buf)) * -1;
	quat.z = fromFixedPoint<1, 14>(hdk_get_le_int16(buf));
	quat.w = fromFixedPoint<1, 14>(hdk_get_le_int16(buf));

	out_relation->pose.orientation = quat;

	/// @todo might not be accurate on some version 1 reports??

	// This is in the "world" coordinate system.
	struct xrt_vec3 ang_vel;
	ang_vel.x = fromFixedPoint<6, 9>(hdk_get_le_int16(buf));
	ang_vel.y = fromFixedPoint<6, 9>(hdk_get_le_int16(buf));
	ang_vel.z = fromFixedPoint<6, 9>(hdk_get_le_int16(buf));

	out_relation->angular_velocity = ang_vel;

	out_relation->relation_flags = xrt_space_relation_flags(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	HDK_SPEW(hd, "GET_TRACKED_POSE (%f, %f, %f, %f) ANG_VEL (%f, %f, %f)",
	         quat.x, quat.y, quat.z, quat.w, ang_vel.x, ang_vel.y,
	         ang_vel.z);
}

static void
hdk_device_get_view_pose(struct xrt_device *xdev,
                         struct xrt_vec3 *eye_relation,
                         uint32_t view_index,
                         struct xrt_pose *out_pose)
{
	struct xrt_pose pose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	bool adjust = view_index == 0;

	pose.position.x = eye_relation->x / 2.0f;
	pose.position.y = eye_relation->y / 2.0f;
	pose.position.z = eye_relation->z / 2.0f;

	// Adjust for left/right while also making sure there aren't any -0.f.
	if (pose.position.x > 0.0f && adjust) {
		pose.position.x = -pose.position.x;
	}
	if (pose.position.y > 0.0f && adjust) {
		pose.position.y = -pose.position.y;
	}
	if (pose.position.z > 0.0f && adjust) {
		pose.position.z = -pose.position.z;
	}

	*out_pose = pose;
}

#define HDK_DEBUG_INT(hd, name, val) HDK_DEBUG(hd, "\t%s = %u", name, val)

#define HDK_DEBUG_MM(hd, name, val)                                            \
	HDK_DEBUG(hd, "\t%s = %i.%02imm", name, (int32_t)(val * 1000.f),       \
	          abs((int32_t)(val * 100000.f)) % 100)

#define HDK_DEBUG_ANGLE(hd, name, val)                                         \
	HDK_DEBUG(hd, "\t%s = %f (%i)", name, val,                             \
	          (int32_t)(val * (180 / M_PI)))

#define HDK_DEBUG_MAT2X2(hd, name, rot)                                        \
	HDK_DEBUG(hd, "\t%s = {%f, %f} {%f, %f}", name, rot.v[0], rot.v[1],    \
	          rot.v[2], rot.v[3])

struct hdk_device *
hdk_device_create(hid_device *dev,
                  enum HDK_VARIANT variant,
                  bool print_spew,
                  bool print_debug)
{
	struct hdk_device *hd =
	    (struct hdk_device *)calloc(1, sizeof(struct hdk_device));
	hd->base.blend_mode = XRT_BLEND_MODE_OPAQUE;
	hd->base.destroy = hdk_device_destroy;
	hd->base.get_tracked_pose = hdk_device_get_tracked_pose;
	hd->base.get_view_pose = hdk_device_get_view_pose;
	hd->dev = dev;
	hd->print_spew = print_spew;
	hd->print_debug = print_debug;

	if (variant != HDK_VARIANT_2) {
		HDK_ERROR(hd,
		          "Only recognize HDK2 for now, and this isn't it!");
		hdk_device_destroy(&hd->base);
		return NULL;
	}

	// Treat as symmetric right now.
	const double FOV = 92.0 / 180 * M_PI;
	{
		/* right eye */
		math_compute_fovs(1.0, 0.5, FOV, 1, 0.5, FOV,
		                  &hd->base.views[1].fov);
	}
	{
		/* left eye - just mirroring right eye now */
		hd->base.views[0].fov.angle_up = hd->base.views[1].fov.angle_up;
		hd->base.views[0].fov.angle_down =
		    hd->base.views[1].fov.angle_down;

		hd->base.views[0].fov.angle_left =
		    -hd->base.views[1].fov.angle_right;
		hd->base.views[0].fov.angle_right =
		    -hd->base.views[1].fov.angle_left;
	}

	// HDK2 is upside down :facepalm:

	// clang-format off
	// Main display.
	hd->base.screens[0].w_pixels = 2160;
	hd->base.screens[0].h_pixels = 1200;

	// Left
	hd->base.views[0].display.w_pixels = 1080;
	hd->base.views[0].display.h_pixels = 1200;
	hd->base.views[0].viewport.x_pixels = 1080; // right half of display
	hd->base.views[0].viewport.y_pixels = 60;
	hd->base.views[0].viewport.w_pixels = 1080;
	hd->base.views[0].viewport.h_pixels = 1080;
	hd->base.views[0].rot = u_device_rotation_180;

	// Right
	hd->base.views[1].display.w_pixels = 1080;
	hd->base.views[1].display.h_pixels = 1200;
	hd->base.views[1].viewport.x_pixels = 0;
	hd->base.views[1].viewport.y_pixels = 60;
	hd->base.views[1].viewport.w_pixels = 1080;
	hd->base.views[1].viewport.h_pixels = 1080;
	hd->base.views[1].rot = u_device_rotation_180;

	// Distortion
	hd->base.distortion.models = XRT_DISTORTION_MODEL_NONE;
	hd->base.distortion.preferred = XRT_DISTORTION_MODEL_NONE;
	// clang-format on


	if (hd->print_debug) {
		u_device_dump_config(&hd->base, __func__,
		                     "OSVR HDK-family Device");
	}

	return hd;
}