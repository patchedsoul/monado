// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Adaptor to a OpenHMD device.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "openhmd.h"

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include "oh_device.h"

// Define this if you have the appropriately hacked-up OpenHMD version.
#undef OHMD_HAVE_ANG_VEL

static void
oh_device_destroy(struct xrt_device *xdev)
{
	struct oh_device *ohd = oh_device(xdev);

	if (ohd->dev != NULL) {
		ohmd_close_device(ohd->dev);
		ohd->dev = NULL;
	}

	free(ohd);
}

static void
oh_device_get_tracked_pose(struct xrt_device *xdev,
                           struct xrt_space_relation *out_relation)
{
	struct oh_device *ohd = oh_device(xdev);
	struct xrt_quat quat = {0.f, 0.f, 0.f, 1.f};
	ohmd_ctx_update(ohd->ctx);
	ohmd_device_getf(ohd->dev, OHMD_ROTATION_QUAT, &quat.x);
	out_relation->pose.orientation = quat;
	//! @todo assuming that orientation is actually currently tracked.
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

#ifdef OHMD_HAVE_ANG_VEL
	if (!ohd->skip_ang_vel) {
		struct xrt_vec3 ang_vel;
		if (0 == ohmd_device_getf(ohd->dev, OHMD_ANGULAR_VELOCITY,
		                          &ang_vel.x)) {
			out_relation->angular_velocity = ang_vel;
			out_relation->relation_flags =
			    (enum xrt_space_relation_flags)(
			        out_relation->relation_flags |
			        XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
			OH_SPEW(
			    ohd,
			    "GET_TRACKED_POSE (%f, %f, %f, %f) (%f, %f, %f)",
			    quat.x, quat.y, quat.z, quat.w, ang_vel.x,
			    ang_vel.y, ang_vel.z);
			return;
		} else {
			// we now know this device doesn't return angular
			// velocity.
			ohd->skip_ang_vel = true;
		}
	}
#endif
	OH_SPEW(ohd, "GET_TRACKED_POSE (%f, %f, %f, %f)", quat.x, quat.y,
	        quat.z, quat.w);
}

static void
oh_device_get_view_pose(struct xrt_device *xdev,
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

struct display_info
{
	float w_meters;
	float h_meters;
	int w_pixels;
	int h_pixels;
};

struct device_info
{
	struct display_info display;

	float lens_horizontal_separation;
	float lens_vertical_position;

	float pano_distortion_k[4];
	float pano_aberration_k[4];
	float pano_warp_scale;

	struct
	{
		float fov;

		struct display_info display;

		float lens_center_x_meters;
		float lens_center_y_meters;
	} views[2];
};

static struct device_info
get_info(struct oh_device *ohd)
{
	struct device_info info = {0};

	// clang-format off
	ohmd_device_getf(ohd->dev, OHMD_SCREEN_HORIZONTAL_SIZE, &info.display.w_meters);
	ohmd_device_getf(ohd->dev, OHMD_SCREEN_VERTICAL_SIZE, &info.display.h_meters);
	ohmd_device_getf(ohd->dev, OHMD_LENS_HORIZONTAL_SEPARATION, &info.lens_horizontal_separation);
	ohmd_device_getf(ohd->dev, OHMD_LENS_VERTICAL_POSITION, &info.lens_vertical_position);
	ohmd_device_getf(ohd->dev, OHMD_LEFT_EYE_FOV, &info.views[0].fov);
	ohmd_device_getf(ohd->dev, OHMD_RIGHT_EYE_FOV, &info.views[1].fov);
	ohmd_device_geti(ohd->dev, OHMD_SCREEN_HORIZONTAL_RESOLUTION, &info.display.w_pixels);
	ohmd_device_geti(ohd->dev, OHMD_SCREEN_VERTICAL_RESOLUTION, &info.display.h_pixels);
	ohmd_device_getf(ohd->dev, OHMD_UNIVERSAL_DISTORTION_K, &info.pano_distortion_k[0]);
	ohmd_device_getf(ohd->dev, OHMD_UNIVERSAL_ABERRATION_K, &info.pano_aberration_k[0]);

	/*
	 * Assumptions made here:
	 *
	 * - There is a single, continuous, flat display serving both eyes, with
	 *   no dead space/gap between eyes.
	 * - This single panel is (effectively) perpendicular to the forward
	 *   (-Z) direction, with edges aligned with the X and Y axes.
	 * - Lens position is symmetrical about the center ("bridge of  nose").
	 * - Pixels are square and uniform across the entirety of the panel.
	 *
	 * If any of these are not true, then either the rendering will
	 * be inaccurate, or the properties will have to be "fudged" to
	 * make the math work.
	 */

	info.views[0].display.w_meters = info.display.w_meters / 2.0;
	info.views[0].display.h_meters = info.display.h_meters;
	info.views[1].display.w_meters = info.display.w_meters / 2.0;
	info.views[1].display.h_meters = info.display.h_meters;

	info.views[0].display.w_pixels = info.display.w_pixels / 2;
	info.views[0].display.h_pixels = info.display.h_pixels;
	info.views[1].display.w_pixels = info.display.w_pixels / 2;
	info.views[1].display.h_pixels = info.display.h_pixels;

	/*
	 * Assuming the lenses are centered vertically on the
	 * display. It's not universal, but 0.5 COP on Y is more
	 * common than on X, and it looked like many of the
	 * driver lens_vpos values were copy/pasted or marked
	 * with FIXME. Safer to fix it to 0.5 than risk an
	 * extreme geometry mismatch.
	 */

	const double cop_y = 0.5;
	const double h_1 = cop_y * info.display.h_meters;

	//! @todo This are probably all wrong!
	info.views[0].lens_center_x_meters = info.views[0].display.w_meters - info.lens_horizontal_separation / 2.0;
	info.views[0].lens_center_y_meters = h_1;

	info.views[1].lens_center_x_meters = info.lens_horizontal_separation / 2.0;
	info.views[1].lens_center_y_meters = h_1;

	//! @todo This is most definitely wrong!
	//!       3Glasses likes the opposite better.
	info.pano_warp_scale =
		(info.views[0].lens_center_x_meters > info.views[0].lens_center_x_meters) ?
			info.views[0].lens_center_x_meters :
			info.views[0].lens_center_x_meters;
	// clang-format on

	return info;
}

struct oh_device *
oh_device_create(ohmd_context *ctx,
                 ohmd_device *dev,
                 const char *prod,
                 bool print_spew,
                 bool print_debug)
{
	struct oh_device *ohd =
	    (struct oh_device *)calloc(1, sizeof(struct oh_device));
	ohd->base.destroy = oh_device_destroy;
	ohd->base.get_tracked_pose = oh_device_get_tracked_pose;
	ohd->base.get_view_pose = oh_device_get_view_pose;
	ohd->ctx = ctx;
	ohd->dev = dev;
	ohd->print_spew = print_spew;
	ohd->print_debug = print_debug;

	const struct device_info info = get_info(ohd);

	{
		/* right eye */
		if (!math_compute_fovs(info.views[1].display.w_meters,
		                       info.views[1].lens_center_x_meters,
		                       info.views[1].fov,
		                       info.views[1].display.h_meters,
		                       info.views[1].lens_center_y_meters, 0,
		                       &ohd->base.views[1].fov)) {
			OH_ERROR(
			    ohd,
			    "Failed to compute the partial fields of view.");
			free(ohd);
			return NULL;
		}
	}
	{
		/* left eye - just mirroring right eye now */
		ohd->base.views[0].fov.angle_up =
		    ohd->base.views[1].fov.angle_up;
		ohd->base.views[0].fov.angle_down =
		    ohd->base.views[1].fov.angle_down;

		ohd->base.views[0].fov.angle_left =
		    -ohd->base.views[1].fov.angle_right;
		ohd->base.views[0].fov.angle_right =
		    -ohd->base.views[1].fov.angle_left;
	}

	// clang-format off
	// Main display.
	ohd->base.distortion.models = XRT_DISTORTION_MODEL_PANOTOOLS;
	ohd->base.distortion.preferred = XRT_DISTORTION_MODEL_PANOTOOLS;
	ohd->base.screens[0].w_pixels = info.display.w_pixels;
	ohd->base.screens[0].h_pixels = info.display.h_pixels;
	ohd->base.distortion.pano.distortion_k[0] = info.pano_distortion_k[0];
	ohd->base.distortion.pano.distortion_k[1] = info.pano_distortion_k[1];
	ohd->base.distortion.pano.distortion_k[2] = info.pano_distortion_k[2];
	ohd->base.distortion.pano.distortion_k[3] = info.pano_distortion_k[3];
	ohd->base.distortion.pano.aberration_k[0] = info.pano_aberration_k[0];
	ohd->base.distortion.pano.aberration_k[1] = info.pano_aberration_k[1];
	ohd->base.distortion.pano.aberration_k[2] = info.pano_aberration_k[2];
	ohd->base.distortion.pano.warp_scale = info.pano_warp_scale;

	// Left
	ohd->base.views[0].display.w_meters = info.views[0].display.w_meters;
	ohd->base.views[0].display.h_meters = info.views[0].display.h_meters;
	ohd->base.views[0].lens_center.x_meters = info.views[0].lens_center_x_meters;
	ohd->base.views[0].lens_center.y_meters = info.views[0].lens_center_y_meters;
	ohd->base.views[0].display.w_pixels = info.views[0].display.w_pixels;
	ohd->base.views[0].display.h_pixels = info.views[0].display.h_pixels;
	ohd->base.views[0].viewport.x_pixels = 0;
	ohd->base.views[0].viewport.y_pixels = 0;
	ohd->base.views[0].viewport.w_pixels = info.views[0].display.w_pixels;
	ohd->base.views[0].viewport.h_pixels = info.views[0].display.h_pixels;
	ohd->base.views[0].rot = u_device_rotation_ident;

	// Right
	ohd->base.views[1].display.w_meters = info.views[1].display.w_meters;
	ohd->base.views[1].display.h_meters = info.views[1].display.h_meters;
	ohd->base.views[1].lens_center.x_meters = info.views[1].lens_center_x_meters;
	ohd->base.views[1].lens_center.y_meters = info.views[1].lens_center_y_meters;
	ohd->base.views[1].display.w_pixels = info.views[1].display.w_pixels;
	ohd->base.views[1].display.h_pixels = info.views[1].display.h_pixels;
	ohd->base.views[1].viewport.x_pixels = info.views[0].display.w_pixels;
	ohd->base.views[1].viewport.y_pixels = 0;
	ohd->base.views[1].viewport.w_pixels = info.views[1].display.w_pixels;
	ohd->base.views[1].viewport.h_pixels = info.views[1].display.h_pixels;
	ohd->base.views[1].rot = u_device_rotation_ident;
	// clang-format on

	// Find any needed quirks.
	bool quirk_rotate_right = false;
	bool quirk_rotate_inwards = false;
	bool quirk_video_see_through = false;
	bool quirk_video_distortion_none = false;
	bool quirk_video_distortion_vive = false;
	bool quirk_left_center_pano_scale = false;

	// Needs to be rotated.
	if (strcmp(prod, "3Glasses-D3V2") == 0) {
		quirk_rotate_right = true;
		quirk_left_center_pano_scale = true;
	}

	if (strcmp(prod, "HTC Vive") == 0) {
		quirk_video_distortion_vive = true;
		quirk_video_see_through = true;
	}

	if (strcmp(prod, "LGR100") == 0) {
		quirk_rotate_inwards = true;
	}

	if (strcmp(prod, "External Device") == 0) {
		quirk_video_distortion_none = true;
	}

	// Which blend modes does the device support.
	ohd->base.blend_mode |= XRT_BLEND_MODE_OPAQUE;
	if (quirk_video_see_through) {
		ohd->base.blend_mode |= XRT_BLEND_MODE_ALPHA_BLEND;
	}

	if (quirk_video_distortion_vive) {
		ohd->base.distortion.models |= XRT_DISTORTION_MODEL_VIVE;
		ohd->base.distortion.preferred = XRT_DISTORTION_MODEL_VIVE;

		// clang-format off
		// These need to be aquired from the vive config
		ohd->base.distortion.vive.aspect_x_over_y = 0.8999999761581421f;
		ohd->base.distortion.vive.grow_for_undistort = 0.6000000238418579f;
		ohd->base.distortion.vive.undistort_r2_cutoff[0] = 1.11622154712677f;
		ohd->base.distortion.vive.undistort_r2_cutoff[1] = 1.101870775222778f;
		ohd->base.distortion.vive.center[0][0] = 0.08946027017045266f;
		ohd->base.distortion.vive.center[0][1] = -0.009002181016260827f;
		ohd->base.distortion.vive.center[1][0] = -0.08933516629552526f;
		ohd->base.distortion.vive.center[1][1] = -0.006014565287238661f;

		// left
		// green
		ohd->base.distortion.vive.coefficients[0][0][0] = -0.188236068524731f;
		ohd->base.distortion.vive.coefficients[0][0][1] = -0.221086205321053f;
		ohd->base.distortion.vive.coefficients[0][0][2] = -0.2537849057915209f;

		// blue
		ohd->base.distortion.vive.coefficients[0][1][0] = -0.07316590815739493f;
		ohd->base.distortion.vive.coefficients[0][1][1] = -0.02332400789561968f;
		ohd->base.distortion.vive.coefficients[0][1][2] = 0.02469959434698275f;

		// red
		ohd->base.distortion.vive.coefficients[0][2][0] = -0.02223805567703767f;
		ohd->base.distortion.vive.coefficients[0][2][1] = -0.04931309279533211f;
		ohd->base.distortion.vive.coefficients[0][2][2] = -0.07862881939243466f;

		// right
		// green
		ohd->base.distortion.vive.coefficients[1][0][0] = -0.1906209981894497f;
		ohd->base.distortion.vive.coefficients[1][0][1] = -0.2248896677207884f;
		ohd->base.distortion.vive.coefficients[1][0][2] = -0.2721364516782803f;

		// blue
		ohd->base.distortion.vive.coefficients[1][1][0] = -0.07346071902951497f;
		ohd->base.distortion.vive.coefficients[1][1][1] = -0.02189527566250131f;
		ohd->base.distortion.vive.coefficients[1][1][2] = 0.0581378652359256f;

		// red
		ohd->base.distortion.vive.coefficients[1][2][0] = -0.01755850332081247f;
		ohd->base.distortion.vive.coefficients[1][2][1] = -0.04517245633373419f;
		ohd->base.distortion.vive.coefficients[1][2][2] = -0.0928909347763f;
		// clang-format on
	}

	if (quirk_video_distortion_none) {
		ohd->base.distortion.models = XRT_DISTORTION_MODEL_NONE;
		ohd->base.distortion.preferred = XRT_DISTORTION_MODEL_NONE;
	}

	if (quirk_left_center_pano_scale) {
		ohd->base.distortion.pano.warp_scale =
		    info.views[0].lens_center_x_meters;
	}

	if (quirk_rotate_right) {
		int w = info.display.w_pixels;
		int h = info.display.h_pixels;

		ohd->base.screens[0].w_pixels = h;
		ohd->base.screens[0].h_pixels = w;

		ohd->base.views[0].viewport.x_pixels = 0;
		ohd->base.views[0].viewport.y_pixels = 0;
		ohd->base.views[0].viewport.w_pixels = h;
		ohd->base.views[0].viewport.h_pixels = w / 2;
		ohd->base.views[0].rot = u_device_rotation_right;

		ohd->base.views[1].viewport.x_pixels = 0;
		ohd->base.views[1].viewport.y_pixels = w / 2;
		ohd->base.views[1].viewport.w_pixels = h;
		ohd->base.views[1].viewport.h_pixels = w / 2;
		ohd->base.views[1].rot = u_device_rotation_right;
	}

	if (quirk_rotate_inwards) {
		ohd->base.views[0].rot = u_device_rotation_right;
		ohd->base.views[1].rot = u_device_rotation_left;
	}

	if (ohd->print_debug) {
		u_device_dump_config(&ohd->base, __func__, prod);
	}

	return ohd;
}