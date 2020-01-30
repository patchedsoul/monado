// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_tracking.h"

#include "tracking/t_tracking.h"
#include "tracking/t_calibration_opencv.hpp"
#include "tracking/t_tracker_psmv_fusion.hpp"
#include "tracking/t_helper_debug_sink.hpp"
#include "tracking/t_conefitting.hpp"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_format.h"

#include "math/m_api.h"

#include "os/os_threading.h"

#include <stdio.h>
#include <assert.h>
#include <pthread.h>
#include <numeric>

static const xrt_vec3 sphere_lever_arm = {0, 0.9, 0};

using Contour = std::vector<cv::Point>;

constexpr float SphereRadius = 0.045f / 2.f;
constexpr size_t MinPoints = 6;

/*!
 * Single camera.
 */
struct View
{
	cv::Mat undistort_rectify_map_x;
	cv::Mat undistort_rectify_map_y;

	cv::Matx33d intrinsics;
	cv::Mat distortion; // size may vary
	cv::Vec4d distortion_fisheye;
	bool use_fisheye;
	std::unique_ptr<NormalizedCoordsCache> norm_coords;
	ConeFitter cone_fitter;
	cv::Vec3f position;
	bool position_valid;

	xrt_pose calibration_transform;
	bool transform_valid;

	std::vector<cv::KeyPoint> keypoints;

	cv::Mat frame_undist_rectified;

	void
	populate_from_calib(t_camera_calibration &calib,
	                    const RemapPair &rectification)
	{
		CameraCalibrationWrapper wrap(calib);
		intrinsics = wrap.intrinsics_mat;
		distortion = wrap.distortion_mat.clone();
		distortion_fisheye = wrap.distortion_fisheye_mat;
		use_fisheye = wrap.use_fisheye;

		assert(!use_fisheye);
		norm_coords.reset(new NormalizedCoordsCache(
		    wrap.image_size_pixels_cv, wrap.intrinsics_mat,
		    wrap.distortion_mat));

		undistort_rectify_map_x = rectification.remap_x;
		undistort_rectify_map_y = rectification.remap_y;
	}
};

struct TrackerPSMV
{
	struct xrt_tracked_psmv base = {};
	struct xrt_frame_sink sink = {};
	struct xrt_frame_node node = {};

	//! Frame waiting to be processed.
	struct xrt_frame *frame;

	//! Thread and lock helper.
	struct os_thread_helper oth;

	bool tracked = false;

	HelperDebugSink debug = {HelperDebugSink::AllAvailable};

	//! Have we received a new IMU sample.
	bool has_imu = false;

	struct
	{
		struct xrt_vec3 pos = {};
		struct xrt_quat rot = {};
	} fusion;

	View view[2];

	bool calibrated;

	cv::Mat disparity_to_depth;
	cv::Vec3d r_cam_translation;
	cv::Matx33d r_cam_rotation;

	cv::Ptr<cv::SimpleBlobDetector> sbd;

	std::unique_ptr<xrt_fusion::PSMVFusionInterface> filter;

	xrt_vec3 tracked_object_position;
};

static Contour
combineContours(std::vector<Contour> &contours)
{
	Contour allPixels;
	if (contours.size() == 1) {
		allPixels = std::move(contours[0]);
		// avoid undef behavior in case we accidentally access it
		// outside this function.
		contours[0].clear();
	} else if (contours.size() > 1) {
		auto totalPixels =
		    std::accumulate(contours.begin(), contours.end(), size_t(0),
		                    [](size_t prevSize, Contour const &c) {
			                    return prevSize + c.size();
		                    });
		allPixels.reserve(totalPixels);
		for (const auto &contour : contours) {
			allPixels.insert(allPixels.end(), contour.begin(),
			                 contour.end());
		}
		std::cout << "We found " << contours.size()
		          << " contours with a total of " << totalPixels
		          << " pixels.\n";
	}
	return allPixels;
}
static inline std::vector<Contour>
getContours(cv::Mat frame)
{
	std::vector<Contour> output;
	// Not actually used, but can't pass a temporary.
	cv::Mat hierarchy;
	//! @todo we can probably do better than this generic algorithm.
	//! @todo we also need to return the associated "outside" pixel so we
	//! can get the "between" vector
	cv::findContours(frame, output, hierarchy, cv::RETR_EXTERNAL,
	                 cv::CHAIN_APPROX_NONE);
	return output;
}

static std::vector<cv::Vec2f>
makeContourFloat(Contour const &allPixels)
{

	std::vector<cv::Vec2f> points;
	points.reserve(allPixels.size());
	for (const auto &pt : allPixels) {
		points.emplace_back(pt.x, pt.y);
	}
	return points;
}


static void
plot_points_by_index(cv::Mat &frame,
                     Contour const &allPixels,
                     std::vector<size_t> const &indices,
                     cv::Vec3b color)
{
	for (auto index : indices) {
		auto px = allPixels[index];
		frame.at<cv::Vec3b>(px.y, px.x) = color;
	}
}

/*!
 * @brief Perform per-view (two in a stereo camera image) processing on an
 * image, before tracking math is performed.
 *
 * This runs the bulk of the tracking because of cone-fitting.
 */
static bool
do_view_cone(TrackerPSMV &t, View &view, cv::Mat &grey, cv::Mat &rgb)
{

	std::vector<Contour> output;

	auto contours = getContours(grey);

	auto allPixels = combineContours(contours);
	if (allPixels.empty()) {
		return false;
	}
	auto points = makeContourFloat(allPixels);

	std::vector<cv::Vec3f> directions;
	for (const auto &point : points) {
		cv::Vec3d normalizedVec =
		    view.norm_coords->getNormalizedVector(point);
		directions.emplace_back(normalizedVec);
	}

	cv::Vec3f position;
	std::vector<size_t> indices;
	if (view.cone_fitter.fit_cone_and_get_inlier_indices(
	        directions, MinPoints, SphereRadius, position, indices)) {
		// Debug is wanted, draw the keypoints.
		if (rgb.cols > 0) {
			plot_points_by_index(rgb, allPixels, indices,
			                     cv::Vec3b(0, 255, 0));
		}
		// y axis in world is inverted from camera-land
		position[1] = -position[1];
		view.position = position;
		return true;
	}
	return false;
}


static void
incorporate_cone_fit_tracking(TrackerPSMV &t,
                              const View &view,
                              timepoint_ns timestamp)
{
	xrt_vec3 pos = {view.position[0], view.position[1], view.position[2]};
	if (view.transform_valid) {
		math_pose_transform_point(&view.calibration_transform, &pos,
		                          &pos);
	}
	t.tracked_object_position = pos;
	// std::cout << "Transformed position is [" << pos.x << ", " << pos.y
	//           << ", " << pos.z << "]\n";
	//! @todo these are crude guesses.
	const xrt_vec3 variance = {0.001, 0.001, 0.001};
	t.filter->process_3d_vision_data(timestamp, &pos, &variance,
	                                 &sphere_lever_arm,
	                                 //! @todo tune cutoff for residual
	                                 //! arbitrarily "too large"
	                                 15);
}

/*!
 * @brief Perform tracking computations on a frame of video data.
 */
static void
process(TrackerPSMV &t, struct xrt_frame *xf)
{
	// Only IMU data: nothing to do
	if (xf == NULL) {
		return;
	}

	// Wrong type of frame: unreference and return?
	if (xf->format != XRT_FORMAT_L8) {
		xrt_frame_reference(&xf, NULL);
		return;
	}

	if (!t.calibrated) {
		return;
	}

	// Create the debug frame if needed.
	t.debug.refresh(xf);

	t.view[0].keypoints.clear();
	t.view[1].keypoints.clear();
	int cols = xf->width / 2;
	int rows = xf->height;
	int stride = xf->stride;


	if (t.debug.frame != NULL) {
		t.debug.rgb[0] = cv::Mat::zeros(t.debug.rgb[0].size(), CV_8UC3);
		t.debug.rgb[1] = cv::Mat::zeros(t.debug.rgb[1].size(), CV_8UC3);
		// .copyTo()
		// .copyTo(t.debug.rgb[1]);
	}
	cv::Mat l_grey(rows, cols, CV_8UC1, xf->data, stride);
	cv::Mat r_grey(rows, cols, CV_8UC1, xf->data + cols, stride);

	t.view[0].position_valid =
	    do_view_cone(t, t.view[0], l_grey, t.debug.rgb[0]);
	t.view[1].position_valid =
	    do_view_cone(t, t.view[1], r_grey, t.debug.rgb[1]);

	timepoint_ns timestamp = xf->timestamp;

	// Update debug view
	if (t.debug.frame != NULL) {
		t.debug.sink->push_frame(t.debug.sink, t.debug.frame);
		t.debug.rgb[0] = cv::Mat();
		t.debug.rgb[1] = cv::Mat();
	}

	// We are done with the debug frame.
	t.debug.submit();

	// We are done with the frame.
	xrt_frame_reference(&xf, NULL);

	// incorporate_stereo_blob_tracking(t, gotTracking);
	if (t.view[0].position_valid) {
		incorporate_cone_fit_tracking(t, t.view[0], timestamp);
	}
	if (t.view[1].position_valid) {
		incorporate_cone_fit_tracking(t, t.view[1], timestamp);
	}
}

/*!
 * @brief Tracker processing thread function
 */
static void
run(TrackerPSMV &t)
{
	struct xrt_frame *frame = NULL;

	os_thread_helper_lock(&t.oth);

	while (os_thread_helper_is_running_locked(&t.oth)) {
		// No data
		if (!t.has_imu || t.frame == NULL) {
			os_thread_helper_wait_locked(&t.oth);
		}

		if (!os_thread_helper_is_running_locked(&t.oth)) {
			break;
		}

		// Take a reference on the current frame, this keeps it alive
		// if it is replaced during the consumer processing it, but
		// we no longer need to hold onto the frame on the queue we
		// just move the pointer.
		frame = t.frame;
		t.frame = NULL;

		// Unlock the mutex when we do the work.
		os_thread_helper_unlock(&t.oth);

		process(t, frame);

		// Have to lock it again.
		os_thread_helper_lock(&t.oth);
	}

	os_thread_helper_unlock(&t.oth);
}

/*!
 * @brief Retrieves a pose from the filter.
 */
static void
get_pose(TrackerPSMV &t,
         enum xrt_input_name name,
         struct time_state *timestate,
         timepoint_ns when_ns,
         struct xrt_space_relation *out_relation)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	if (name == XRT_INPUT_PSMV_BALL_CENTER_POSE) {
		out_relation->pose.position = t.tracked_object_position;
		out_relation->pose.orientation.x = 0.0f;
		out_relation->pose.orientation.y = 0.0f;
		out_relation->pose.orientation.z = 0.0f;
		out_relation->pose.orientation.w = 1.0f;

		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);

		os_thread_helper_unlock(&t.oth);
		return;
	}

	t.filter->get_prediction(when_ns, out_relation);

	os_thread_helper_unlock(&t.oth);
}

static void
imu_data(TrackerPSMV &t,
         timepoint_ns timestamp_ns,
         struct xrt_tracking_sample *sample)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}
	t.filter->process_imu_data(timestamp_ns, sample, NULL);

	os_thread_helper_unlock(&t.oth);
}

static void
frame(TrackerPSMV &t, struct xrt_frame *xf)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	xrt_frame_reference(&t.frame, xf);
	// Wake up the thread.
	os_thread_helper_signal_locked(&t.oth);

	os_thread_helper_unlock(&t.oth);
}

static void
break_apart(TrackerPSMV &t)
{
	os_thread_helper_stop(&t.oth);
}


/*
 *
 * C wrapper functions.
 *
 */

extern "C" void
t_psmv_push_imu(struct xrt_tracked_psmv *xtmv,
                timepoint_ns timestamp_ns,
                struct xrt_tracking_sample *sample)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	imu_data(t, timestamp_ns, sample);
}

extern "C" void
t_psmv_get_tracked_pose(struct xrt_tracked_psmv *xtmv,
                        enum xrt_input_name name,
                        struct time_state *timestate,
                        timepoint_ns when_ns,
                        struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	get_pose(t, name, timestate, when_ns, out_relation);
}

extern "C" void
t_psmv_fake_destroy(struct xrt_tracked_psmv *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	(void)t;
	// Not the real destroy function
}

extern "C" void
t_psmv_sink_push_frame(struct xrt_frame_sink *xsink, struct xrt_frame *xf)
{
	auto &t = *container_of(xsink, TrackerPSMV, sink);
	frame(t, xf);
}

extern "C" void
t_psmv_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerPSMV, node);
	break_apart(t);
}

extern "C" void
t_psmv_node_destroy(struct xrt_frame_node *node)
{
	auto t_ptr = container_of(node, TrackerPSMV, node);
	os_thread_helper_destroy(&t_ptr->oth);

	// Tidy variable setup.
	u_var_remove_root(t_ptr);

	delete t_ptr;
}

extern "C" void *
t_psmv_run(void *ptr)
{
	auto &t = *(TrackerPSMV *)ptr;
	run(t);
	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

extern "C" int
t_psmv_start(struct xrt_tracked_psmv *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSMV, base);
	return os_thread_helper_start(&t.oth, t_psmv_run, &t);
}

extern "C" int
t_psmv_create(struct xrt_frame_context *xfctx,
              struct xrt_colour_rgb_f32 *rgb,
              struct t_stereo_camera_calibration *data,
              struct xrt_tracked_psmv **out_xtmv,
              struct xrt_frame_sink **out_sink)
{
	fprintf(stderr, "%s\n", __func__);

	auto &t = *(new TrackerPSMV());
	int ret;

	t.base.get_tracked_pose = t_psmv_get_tracked_pose;
	t.base.push_imu = t_psmv_push_imu;
	t.base.destroy = t_psmv_fake_destroy;
	t.base.colour = *rgb;
	t.sink.push_frame = t_psmv_sink_push_frame;
	t.node.break_apart = t_psmv_node_break_apart;
	t.node.destroy = t_psmv_node_destroy;
	t.fusion.rot.x = 0.0f;
	t.fusion.rot.y = 0.0f;
	t.fusion.rot.z = 0.0f;
	t.fusion.rot.w = 1.0f;
	t.filter = xrt_fusion::PSMVFusionInterface::create();

	ret = os_thread_helper_init(&t.oth);
	if (ret != 0) {
		delete (&t);
		return ret;
	}

	static int hack = 0;
	switch (hack++) {
	case 0:
		t.fusion.pos.x = -0.3f;
		t.fusion.pos.y = 1.3f;
		t.fusion.pos.z = -0.5f;
		break;
	case 1:
		t.fusion.pos.x = 0.3f;
		t.fusion.pos.y = 1.3f;
		t.fusion.pos.z = -0.5f;
		break;
	default:
		t.fusion.pos.x = 0.0f;
		t.fusion.pos.y = 0.8f + hack * 0.1f;
		t.fusion.pos.z = -0.5f;
		break;
	}

	StereoCameraCalibrationWrapper wrapped(*data);
	StereoRectificationMaps rectify(*data);
	t.view[0].populate_from_calib(data->view[0], rectify.view[0].rectify);
	t.view[1].populate_from_calib(data->view[1], rectify.view[1].rectify);
	t.disparity_to_depth = rectify.disparity_to_depth_mat;
	std::cout << t.disparity_to_depth << std::endl;
	t.r_cam_rotation = wrapped.camera_rotation_mat;
	t.r_cam_translation = wrapped.camera_translation_mat;
	{
		xrt_pose &transform = t.view[0].calibration_transform;
		t.view[0].transform_valid = true;
		transform.orientation.w = 1;
		transform.position.x = t.r_cam_translation[0];
		transform.position.y = t.r_cam_translation[1];
		transform.position.z = t.r_cam_translation[2];
		xrt_matrix_3x3 rot_mat;
		for (size_t i = 0; i < 3; ++i) {
			for (size_t j = 0; j < 3; ++j) {
				rot_mat.v[i * 3 + j] = t.r_cam_rotation(i, j);
			}
		}
		math_quat_from_matrix_3x3(&rot_mat, &transform.orientation);
	}
	t.calibrated = true;

	// clang-format off
	cv::SimpleBlobDetector::Params blob_params;
	blob_params.filterByArea = false;
	blob_params.filterByConvexity = true;
	blob_params.minConvexity = 0.8;
	blob_params.filterByInertia = false;
	blob_params.filterByColor = true;
	blob_params.blobColor = 255; // 0 or 255 - color comes from binarized image?
	blob_params.minArea = 1;
	blob_params.maxArea = 1000;
	blob_params.maxThreshold = 51; // using a wide threshold span slows things down bigtime
	blob_params.minThreshold = 50;
	blob_params.thresholdStep = 1;
	blob_params.minDistBetweenBlobs = 5;
	blob_params.minRepeatability = 1; // need this to avoid error?
	// clang-format on

	t.sbd = cv::SimpleBlobDetector::create(blob_params);
	xrt_frame_context_add(xfctx, &t.node);

	// Everything is safe, now setup the variable tracking.
	u_var_add_root(&t, "PSMV Tracker", true);
	u_var_add_vec3_f32(&t, &t.tracked_object_position, "last.ball.pos");
	u_var_add_sink(&t, &t.debug.sink, "Debug");

	*out_sink = &t.sink;
	*out_xtmv = &t.base;

	return 0;
}
