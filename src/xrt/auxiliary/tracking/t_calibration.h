// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C interface to Calibration code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Calibration data.
 *
 */

//! Maximum size of rectilinear distortion coefficient array
#define XRT_DISTORTION_MAX_DIM (5)

/*!
 * @brief Essential calibration data for a single camera, or single lens/sensor
 * of a stereo camera.
 */
struct t_camera_calibration
{
	//! Source image size
	struct xrt_size image_size_pixels;

	//! Camera intrinsics matrix
	double intrinsics[3][3];

	//! Rectilinear distortion coefficients: k1, k2, p1, p2[, k3[, k4, k5,
	//! k6[, s1, s2, s3, s4]]
	double distortion[XRT_DISTORTION_MAX_DIM];

	//! Fisheye camera distortion coefficients
	double distortion_fisheye[4];

	//! Is the camera fisheye?
	bool use_fisheye;
};

/*!
 * Stereo camera calibration data to be given to trackers.
 */
struct t_stereo_camera_calibration
{
	//! Calibration of individual views/sensor
	struct t_camera_calibration view[2];

	//! Source image size.
	struct xrt_size image_size_pixels;

	//! Translation from first to second in the stereo pair.
	double camera_translation[3];
	//! Rotation matrix from first to second in the stereo pair.
	double camera_rotation[3][3];

	//! Essential matrix.
	double camera_essential[3][3];
	//! Fundamental matrix.
	double camera_fundamental[3][3];
};

/*!
 * Free stereo calibration data.
 */
void
t_stereo_camera_calibration_free(struct t_stereo_camera_calibration **data_ptr);

/*!
 * Load stereo calibration data from a given file.
 */
bool
t_stereo_camera_calibration_load_v1(
    FILE *calib_file, struct t_stereo_camera_calibration **out_data);

/*!
 * Load a stereo calibration struct from a hardcoded place.
 */
bool
t_stereo_camera_calibration_load_v1_hack(
    struct t_stereo_camera_calibration **out_data);

/*!
 * Save raw calibration data to file, hack until prober has storage for such
 * things.
 */
bool
t_file_save_raw_data_hack(struct t_stereo_camera_calibration *data);


/*
 *
 * Calibration sink.
 *
 */

/*!
 * Board pattern type.
 */
enum t_board_pattern
{
	T_BOARD_CHECKERS,
	T_BOARD_CIRCLES,
	T_BOARD_ASYMMETRIC_CIRCLES,
};

struct t_calibration_status
{
	//! Is calibration finished?
	bool finished;
	//! Was the target found this frame?
	bool found;
	//! Number of frames collected
	int num_collected;
	//! Number of moving frames before another capture
	int cooldown;
	//! Number of non-moving frames before capture.
	int waits_remaining;
	//! Returned data.
	struct t_stereo_camera_calibration data;
};

struct t_calibration_params
{
	//! Should we use fisheye version of the calibration functions.
	bool use_fisheye;
	//! Is the camera a stereo sbs camera, mostly for image loading.
	bool stereo_sbs;
	//! What type of pattern are we using for calibration.
	enum t_board_pattern pattern;

	struct
	{
		int cols;
		int rows;
		float size_meters;

		bool subpixel_enable;
		int subpixel_size;
	} checkers;

	struct
	{
		int cols;
		int rows;
		float distance_meters;
	} circles;

	struct
	{
		int cols;
		int rows;
		float diagonal_distance_meters;
	} asymmetric_circles;

	struct
	{
		bool enabled;
		int num_images;
	} load;

	int num_cooldown_frames;
	int num_wait_for;
	int num_collect_total;
	int num_collect_restart;

	/*!
	 * Should we mirror the RGB image?
	 *
	 * Before text is written out, has no effect on actual image capture.
	 */
	bool mirror_rgb_image;

	bool save_images;
};

/*!
 * Sets the calibration parameters to the their default values.
 */
XRT_MAYBE_UNUSED static inline void
t_calibration_params_default(struct t_calibration_params *p)
{
	// Camera config.
	p->use_fisheye = false;
	p->stereo_sbs = true;

	// Which board should we calibrate against.
	p->pattern = T_BOARD_CHECKERS;

	// Checker board.
	p->checkers.cols = 9;
	p->checkers.rows = 7;
	p->checkers.size_meters = 0.025f;
	p->checkers.subpixel_enable = true;
	p->checkers.subpixel_size = 5;

	// Symmetrical circles.
	p->circles.cols = 9;
	p->circles.rows = 7;
	p->circles.distance_meters = 0.025f;

	// Asymmetrical circles.
	p->asymmetric_circles.cols = 5;
	p->asymmetric_circles.rows = 17;
	p->asymmetric_circles.diagonal_distance_meters = 0.02f;

	// Loading of images.
	p->load.enabled = false;
	p->load.num_images = 20;

	// Frame collection info.
	p->num_cooldown_frames = 20;
	p->num_wait_for = 5;
	p->num_collect_total = 20;
	p->num_collect_restart = 1;

	// Misc.
	p->mirror_rgb_image = false;
	p->save_images = true;
}

/*!
 * @brief Create the camera calibration frame sink.
 *
 * @param xfctx Context for frame transport.
 * @param params Parameters to use during calibration. Values copied, pointer
 * not retained.
 * @param status Optional pointer to structure for status information. Pointer
 * retained, and pointed-to struct modified.
 * @param gui Frame sink
 * @param out_sink Output: created frame sink.
 */
int
t_calibration_create(struct xrt_frame_context *xfctx,
                     const struct t_calibration_params *params,
                     struct t_calibration_status *status,
                     struct xrt_frame_sink *gui,
                     struct xrt_frame_sink **out_sink);


#ifdef __cplusplus
}
#endif
