// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: Apache-2.0
/*!
 * @file
 * @brief  Adaptor to a Libsurvive.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_misc.h"

#include "survive_device.h"

static void
survive_device_destroy(struct xrt_device *xdev)
{
	struct survive_device *survive = survive_device(xdev);

	if (survive->ctx != NULL) {
		survive_simple_close(survive->ctx);
	}

	free(survive);
}

static void
survive_device_get_tracked_pose(struct xrt_device *xdev,
                                struct xrt_space_relation *out_relation)
{
	struct survive_device *survive = survive_device(xdev);
	out_relation->relation_flags = 0;

	bool new_data = false;

	static struct xrt_quat last_rot = {.x = 0, .y = 0, .z = 0, .w = 1};
	static struct xrt_vec3 last_pos = {.x = 0, .y = 0, .z = 0};

	for (const SurviveSimpleObject *it =
	         survive_simple_get_next_updated(survive->ctx);
	     it != 0; it = survive_simple_get_next_updated(survive->ctx)) {
		const char *codename = survive_simple_object_name(it);

		if (strcmp(codename, "HMD") != 0)
			continue;

		new_data = true;
		SurvivePose pose;

		uint32_t timecode =
		    survive_simple_object_get_latest_pose(it, &pose);
		(void)timecode;

		struct xrt_quat out_rot = {.x = pose.Rot[1],
		                           .y = pose.Rot[2],
		                           .z = pose.Rot[3],
		                           .w = pose.Rot[0]};

		/* libsurvive looks down when it should be looking forward, so
		 * rotate the quat.
		 * because the HMD quat is the opposite of the in world
		 * rotation, we rotate down. */
		struct xrt_quat down_rot;
		down_rot.x = sqrtf(2) / 2.;
		down_rot.y = 0;
		down_rot.z = 0;
		down_rot.w = -sqrtf(2) / 2.;

		math_quat_rotate(&down_rot, &out_rot, &out_rot);

		out_relation->pose.orientation = out_rot;

		/* because the quat is rotated, y and z axes are switched. */
		out_relation->pose.position.x = pose.Pos[0];
		out_relation->pose.position.y = pose.Pos[2];
		out_relation->pose.position.z = -pose.Pos[1];

		out_relation->relation_flags =
		    (enum xrt_space_relation_flags)(
		        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) |
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT;

		SURVIVE_SPEW(
		    survive,
		    "GET_POSITION (%f %f %f) GET_ORIENTATION (%f, %f, %f, %f)",
		    out_relation->pose.position.x,
		    out_relation->pose.position.y,
		    out_relation->pose.position.z, out_rot.x, out_rot.y,
		    out_rot.z, out_rot.w);
		last_rot = out_relation->pose.orientation;
		last_pos = out_relation->pose.position;
	}

	/// @todo: Handle device supplying data too slowly better
	if (!new_data) {
		out_relation->pose.orientation = last_rot;
		out_relation->pose.position = last_pos;
		out_relation->relation_flags =
		    (enum xrt_space_relation_flags)(
		        XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		        XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) |
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT;
	}
}

static void
survive_device_get_view_pose(struct xrt_device *xdev,
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
get_info()
{
	struct device_info info = {0};

	//! @todo: hardcoded values for Vive 1 from openhmd vive driver
	info.display.w_meters = 0.122822f;
	info.display.h_meters = 0.068234f;
	info.lens_horizontal_separation = 0.056;
	info.lens_vertical_position = 0.032;
	info.views[0].fov =
	    2 * atan2f(0.122822f / 2. - 0.056 / 2., 0.023226876441867737);
	info.views[1].fov = info.views[0].fov;
	info.display.w_pixels = 2160;
	info.display.h_pixels = 1200;
	info.pano_distortion_k[0] = 1.318397;
	info.pano_distortion_k[1] = -1.490242;
	info.pano_distortion_k[2] = 0.663824;
	info.pano_distortion_k[3] = 0.508021;
	info.pano_aberration_k[0] = 1.00010147892f;
	info.pano_aberration_k[1] = 1.000f;
	info.pano_aberration_k[2] = 1.00019614479f;

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
	info.views[0].lens_center_x_meters =
	    info.views[0].display.w_meters -
	    info.lens_horizontal_separation / 2.0;
	info.views[0].lens_center_y_meters = h_1;

	info.views[1].lens_center_x_meters =
	    info.lens_horizontal_separation / 2.0;
	info.views[1].lens_center_y_meters = h_1;

	//! @todo This is most definitely wrong!
	//!       3Glasses likes the oposite better.
	info.pano_warp_scale = (info.views[0].lens_center_x_meters >
	                        info.views[0].lens_center_x_meters)
	                           ? info.views[0].lens_center_x_meters
	                           : info.views[0].lens_center_x_meters;
	return info;
}

struct survive_device *
survive_device_create(bool print_spew, bool print_debug)
{
	char *survive_args[] = {
	    "Monado-libsurvive",
	    // Improves the situation when one basestation goes out of view
	    //"--time-window", "1500000"
	};
	SurviveSimpleContext *actx = survive_simple_init(
	    sizeof(survive_args) / sizeof(survive_args[0]), survive_args);

	//! @todo: when no vive is connected, this prober will behave badly.
	// * No calibration present: It segfaults
	// * Calibration present. It calls exit()
	// it should really return NULL instead.

	if (!actx)
		return NULL;

	survive_simple_start_thread(actx);

	struct survive_device *survive =
	    (struct survive_device *)calloc(1, sizeof(struct survive_device));
	survive->base.destroy = survive_device_destroy;
	survive->base.get_tracked_pose = survive_device_get_tracked_pose;
	survive->base.get_view_pose = survive_device_get_view_pose;
	survive->ctx = actx;
	survive->print_spew = print_spew;
	survive->print_debug = print_debug;

	const struct device_info info = get_info();

	{
		/* right eye */
		if (!math_compute_fovs(info.views[1].display.w_meters,
		                       info.views[1].lens_center_x_meters,
		                       info.views[1].fov,
		                       info.views[1].display.h_meters,
		                       info.views[1].lens_center_y_meters, 0,
		                       &survive->base.views[1].fov)) {
			SURVIVE_ERROR(
			    survive,
			    "Failed to compute the partial fields of view.");
			free(survive);
			return NULL;
		}
	}
	{
		/* left eye - just mirroring right eye now */
		survive->base.views[0].fov.angle_up =
		    survive->base.views[1].fov.angle_up;
		survive->base.views[0].fov.angle_down =
		    survive->base.views[1].fov.angle_down;

		survive->base.views[0].fov.angle_left =
		    -survive->base.views[1].fov.angle_right;
		survive->base.views[0].fov.angle_right =
		    -survive->base.views[1].fov.angle_left;
	}

	// clang-format off
	// Main display.
	survive->base.distortion.models = XRT_DISTORTION_MODEL_PANOTOOLS;
	survive->base.distortion.preferred = XRT_DISTORTION_MODEL_PANOTOOLS;
	survive->base.screens[0].w_pixels = info.display.w_pixels;
	survive->base.screens[0].h_pixels = info.display.h_pixels;
	survive->base.distortion.pano.distortion_k[0] = info.pano_distortion_k[0];
	survive->base.distortion.pano.distortion_k[1] = info.pano_distortion_k[1];
	survive->base.distortion.pano.distortion_k[2] = info.pano_distortion_k[2];
	survive->base.distortion.pano.distortion_k[3] = info.pano_distortion_k[3];
	survive->base.distortion.pano.aberration_k[0] = info.pano_aberration_k[0];
	survive->base.distortion.pano.aberration_k[1] = info.pano_aberration_k[1];
	survive->base.distortion.pano.aberration_k[2] = info.pano_aberration_k[2];

	// Left
	survive->base.views[0].display.w_meters = info.views[0].display.w_meters;
	survive->base.views[0].display.h_meters = info.views[0].display.h_meters;
	survive->base.views[0].lens_center.x_meters = info.views[0].lens_center_x_meters;
	survive->base.views[0].lens_center.y_meters = info.views[0].lens_center_y_meters;
	survive->base.views[0].display.w_pixels = info.views[0].display.w_pixels;
	survive->base.views[0].display.h_pixels = info.views[0].display.h_pixels;
	survive->base.views[0].viewport.x_pixels = 0;
	survive->base.views[0].viewport.y_pixels = 0;
	survive->base.views[0].viewport.w_pixels = info.views[0].display.w_pixels;
	survive->base.views[0].viewport.h_pixels = info.views[0].display.h_pixels;
	survive->base.views[0].rot = u_device_rotation_ident;

	// Right
	survive->base.views[1].display.w_meters = info.views[1].display.w_meters;
	survive->base.views[1].display.h_meters = info.views[1].display.h_meters;
	survive->base.views[1].lens_center.x_meters = info.views[1].lens_center_x_meters;
	survive->base.views[1].lens_center.y_meters = info.views[1].lens_center_y_meters;
	survive->base.views[1].display.w_pixels = info.views[1].display.w_pixels;
	survive->base.views[1].display.h_pixels = info.views[1].display.h_pixels;
	survive->base.views[1].viewport.x_pixels = info.views[0].display.w_pixels;
	survive->base.views[1].viewport.y_pixels = 0;
	survive->base.views[1].viewport.w_pixels = info.views[1].display.w_pixels;
	survive->base.views[1].viewport.h_pixels = info.views[1].display.h_pixels;
	survive->base.views[1].rot = u_device_rotation_ident;
	// clang-format on

	// Find any needed quirks.
	bool quirk_video_see_through = false;
	bool quirk_video_distortion_vive = false;

	quirk_video_distortion_vive = true;
	quirk_video_see_through = false;

	// Which blend modes does the device support.
	survive->base.blend_mode |= XRT_BLEND_MODE_OPAQUE;
	if (quirk_video_see_through) {
		survive->base.blend_mode |= XRT_BLEND_MODE_ALPHA_BLEND;
	}

	if (quirk_video_distortion_vive) {
		survive->base.distortion.models |= XRT_DISTORTION_MODEL_VIVE;
		survive->base.distortion.preferred = XRT_DISTORTION_MODEL_VIVE;

		// clang-format off
		// These need to be aquired from the vive config
		survive->base.distortion.vive.aspect_x_over_y = 0.8999999761581421f;
		survive->base.distortion.vive.grow_for_undistort = 0.6000000238418579f;
		survive->base.distortion.vive.undistort_r2_cutoff[0] = 1.11622154712677f;
		survive->base.distortion.vive.undistort_r2_cutoff[1] = 1.101870775222778f;
		survive->base.distortion.vive.center[0][0] = 0.08946027017045266f;
		survive->base.distortion.vive.center[0][1] = -0.009002181016260827f;
		survive->base.distortion.vive.center[1][0] = -0.08933516629552526f;
		survive->base.distortion.vive.center[1][1] = -0.006014565287238661f;

		// left
		// green
		survive->base.distortion.vive.coefficients[0][0][0] = -0.188236068524731f;
		survive->base.distortion.vive.coefficients[0][0][1] = -0.221086205321053f;
		survive->base.distortion.vive.coefficients[0][0][2] = -0.2537849057915209f;

		// blue
		survive->base.distortion.vive.coefficients[0][1][0] = -0.07316590815739493f;
		survive->base.distortion.vive.coefficients[0][1][1] = -0.02332400789561968f;
		survive->base.distortion.vive.coefficients[0][1][2] = 0.02469959434698275f;

		// red
		survive->base.distortion.vive.coefficients[0][2][0] = -0.02223805567703767f;
		survive->base.distortion.vive.coefficients[0][2][1] = -0.04931309279533211f;
		survive->base.distortion.vive.coefficients[0][2][2] = -0.07862881939243466f;

		// right
		// green
		survive->base.distortion.vive.coefficients[1][0][0] = -0.1906209981894497f;
		survive->base.distortion.vive.coefficients[1][0][1] = -0.2248896677207884f;
		survive->base.distortion.vive.coefficients[1][0][2] = -0.2721364516782803f;

		// blue
		survive->base.distortion.vive.coefficients[1][1][0] = -0.07346071902951497f;
		survive->base.distortion.vive.coefficients[1][1][1] = -0.02189527566250131f;
		survive->base.distortion.vive.coefficients[1][1][2] = 0.0581378652359256f;

		// red
		survive->base.distortion.vive.coefficients[1][2][0] = -0.01755850332081247f;
		survive->base.distortion.vive.coefficients[1][2][1] = -0.04517245633373419f;
		survive->base.distortion.vive.coefficients[1][2][2] = -0.0928909347763f;
		// clang-format on
	}

	if (survive->print_debug) {
		u_device_dump_config(&survive->base, __func__, "libsurvive");
	}
	return survive;
}
