// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR tracker code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_tracking.h"

#include "tracking/t_tracking.h"
#include "tracking/t_calibration_opencv.h"
#include "tracking/t_tracker_psvr_fusion.h"

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

/*!
 * Single camera.
 */
struct View
{
	cv::Mat undistort_map_x;
	cv::Mat undistort_map_y;
	cv::Mat rectify_map_x;
	cv::Mat rectify_map_y;

	std::vector<cv::KeyPoint> keypoints;

	cv::Mat frame_undist;
	cv::Mat frame_rectified;
};

class TrackerPSVR
{
public:
	struct xrt_tracked_psvr base = {};
	struct xrt_frame_sink sink = {};
	struct xrt_frame_node node = {};

	//! Frame waiting to be processed.
	struct xrt_frame *frame;

	//! Thread and lock helper.
	struct os_thread_helper oth;

	bool tracked = false;

	struct
	{
		struct xrt_frame_sink *sink;
		struct xrt_frame *frame;

		cv::Mat rgb[2];
	} debug;

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

	cv::Ptr<cv::SimpleBlobDetector> sbd;

	std::unique_ptr<xrt_fusion::PSVRFusionInterface> filter;

	xrt_vec3 tracked_object_position;
};

static void
process(TrackerPSVR &t, struct xrt_frame *xf)
{
	// Only IMU data
	if (xf == NULL) {
		return;
	}

	// Wrong type of frame: unreference and return?
	if (xf->format != XRT_FORMAT_L8) {
		xrt_frame_reference(&xf, NULL);
		return;
	}

	if (!t.calibrated) {
		bool ok = calibration_get_stereo(
		    "PS4_EYE",                  // name
		    xf->width,                  // width
		    xf->height,                 // height
		    false,                      // use_fisheye
		    &t.view[0].undistort_map_x, // l_undistort_map_x
		    &t.view[0].undistort_map_y, // l_undistort_map_y
		    &t.view[0].rectify_map_x,   // l_rectify_map_x
		    &t.view[0].rectify_map_y,   // l_rectify_map_y
		    &t.view[1].undistort_map_x, // r_undistort_map_x
		    &t.view[1].undistort_map_y, // r_undistort_map_y
		    &t.view[1].rectify_map_x,   // r_rectify_map_x
		    &t.view[1].rectify_map_y,   // r_rectify_map_y
		    &t.disparity_to_depth);     // disparity_to_depth

		if (ok) {
			printf("loaded calibration for camera!\n");
			t.calibrated = true;
		} else {
			xrt_frame_reference(&xf, NULL);
			return;
		}
	}

	// Create the debug frame if needed.
	// refresh_gui_frame(t, xf);

	t.view[0].keypoints.clear();
	t.view[1].keypoints.clear();

	int cols = xf->width / 2;
	int rows = xf->height;
	int stride = xf->stride;

	cv::Mat l_grey(rows, cols, CV_8UC1, xf->data, stride);
	cv::Mat r_grey(rows, cols, CV_8UC1, xf->data + cols, stride);

	do_view(t, t.view[0], l_grey, t.debug.rgb[0]);
	do_view(t, t.view[1], r_grey, t.debug.rgb[1]);

	cv::Point3f last_point(t.tracked_object_position.x,
	                       t.tracked_object_position.y,
	                       t.tracked_object_position.z);
	auto nearest_world =
	    make_lowest_score_finder<cv::Point3f>([&](cv::Point3f world_point) {
		    //! @todo don't really need the square root to be done here.
		    return cv::norm(world_point - last_point);
	    });
	// do some basic matching to come up with likely disparity-pairs.

	const cv::Matx44d disparity_to_depth =
	    static_cast<cv::Matx44d>(t.disparity_to_depth);

	for (const cv::KeyPoint &l_keypoint : t.view[0].keypoints) {
		cv::Point2f l_blob = l_keypoint.pt;

		auto nearest_blob = make_lowest_score_finder<cv::Point2f>(
		    [&](cv::Point2f r_blob) { return l_blob.x - r_blob.x; });

		for (const cv::KeyPoint &r_keypoint : t.view[1].keypoints) {
			cv::Point2f r_blob = r_keypoint.pt;
			// find closest point on same-ish scanline
			if ((l_blob.y < r_blob.y + 3) &&
			    (l_blob.y > r_blob.y - 3)) {
				nearest_blob.handle_candidate(r_blob);
			}
		}
		//! @todo do we need to avoid claiming the same counterpart
		//! several times?
		if (nearest_blob.got_one) {
			cv::Point3f pt = world_point_from_blobs(
			    l_blob, nearest_blob.best, disparity_to_depth);
			nearest_world.handle_candidate(pt);
		}
	}

	if (nearest_world.got_one) {
		cv::Point3f world_point = nearest_world.best;
		// update internal state
		memcpy(&t.tracked_object_position, &world_point.x,
		       sizeof(t.tracked_object_position));
	} else {
		t.filter->clear_position_tracked_flag();
	}

	if (t.debug.frame != NULL) {
		t.debug.sink->push_frame(t.debug.sink, t.debug.frame);
		t.debug.rgb[0] = cv::Mat();
		t.debug.rgb[1] = cv::Mat();
	}

	xrt_frame_reference(&xf, NULL);
	xrt_frame_reference(&t.debug.frame, NULL);
	if (nearest_world.got_one) {
#if 0
		//! @todo something less arbitrary for the lever arm?
		//! This puts the origin approximately under the PS
		//! button.
		xrt_vec3 lever_arm{0.f, 0.09f, 0.f};
		//! @todo this should depend on distance
		// Weirdly, this is where *not* applying the
		// disparity-to-distance/rectification/etc would
		// simplify things, since the measurement variance is
		// related to the image sensor. 1.e-4 means 1cm std dev.
		// Not sure how to estimate the depth variance without
		// some research.
		xrt_vec3 variance{1.e-4f, 1.e-4f, 4.e-4f};
#endif
		t.filter->process_3d_vision_data(
		    0, &t.tracked_object_position, NULL, NULL,
		    //! @todo tune cutoff for residual arbitrarily "too large"
		    15);
	} else {
		t.filter->clear_position_tracked_flag();
	}
}

/*!
 * @brief Tracker processing thread function
 */
static void
run(TrackerPSVR &t)
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
get_pose(TrackerPSVR &t,
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

	out_relation->pose.position = t.fusion.pos;
	out_relation->pose.orientation = t.fusion.rot;

	//! @todo assuming that orientation is actually currently tracked.
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	os_thread_helper_unlock(&t.oth);
}

static void
imu_data(TrackerPSVR &t,
         time_duration_ns delta_ns,
         struct xrt_tracking_sample *sample)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	float dt = time_ns_to_s(delta_ns);
	// Super simple fusion.
	math_quat_integrate_velocity(&t.fusion.rot, &sample->gyro_rad_secs, dt,
	                             &t.fusion.rot);

	os_thread_helper_unlock(&t.oth);
}

static void
frame(TrackerPSVR &t, struct xrt_frame *xf)
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
break_apart(TrackerPSVR &t)
{
	os_thread_helper_stop(&t.oth);
}


/*
 *
 * C wrapper functions.
 *
 */

extern "C" void
t_psvr_push_imu(struct xrt_tracked_psvr *xtmv,
                time_duration_ns delta_ns,
                struct xrt_tracking_sample *sample)
{
	auto &t = *container_of(xtmv, TrackerPSVR, base);
	imu_data(t, delta_ns, sample);
}

extern "C" void
t_psvr_get_tracked_pose(struct xrt_tracked_psvr *xtmv,
                        struct time_state *timestate,
                        timepoint_ns when_ns,
                        struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xtmv, TrackerPSVR, base);
	get_pose(t, timestate, when_ns, out_relation);
}

extern "C" void
t_psvr_fake_destroy(struct xrt_tracked_psvr *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSVR, base);
	(void)t;
	// Not the real destroy function
}

extern "C" void
t_psvr_sink_push_frame(struct xrt_frame_sink *xsink, struct xrt_frame *xf)
{
	auto &t = *container_of(xsink, TrackerPSVR, sink);
	frame(t, xf);
}

extern "C" void
t_psvr_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerPSVR, node);
	break_apart(t);
}

extern "C" void
t_psvr_node_destroy(struct xrt_frame_node *node)
{
	auto t_ptr = container_of(node, TrackerPSVR, node);

	os_thread_helper_destroy(&t_ptr->oth);

	// Tidy variable setup.
	u_var_remove_root(t_ptr);

	delete t_ptr;
}

extern "C" void *
t_psvr_run(void *ptr)
{
	auto &t = *(TrackerPSVR *)ptr;
	run(t);
	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

extern "C" int
t_psvr_start(struct xrt_tracked_psvr *xtmv)
{
	auto &t = *container_of(xtmv, TrackerPSVR, base);
	return os_thread_helper_start(&t.oth, t_psvr_run, &t);
}

extern "C" int
t_psvr_create(struct xrt_frame_context *xfctx,
              struct xrt_tracked_psvr **out_xtmv,
              struct xrt_frame_sink **out_sink)
{
	fprintf(stderr, "%s\n", __func__);

	auto &t = *(new TrackerPSVR());
	int ret;

	t.base.get_tracked_pose = t_psvr_get_tracked_pose;
	t.base.push_imu = t_psvr_push_imu;
	t.base.destroy = t_psvr_fake_destroy;
	t.sink.push_frame = t_psvr_sink_push_frame;
	t.node.break_apart = t_psvr_node_break_apart;
	t.node.destroy = t_psvr_node_destroy;
	t.fusion.rot.w = 1.0f;
	t.filter = xrt_fusion::PSVRFusionInterface::create();

	ret = os_thread_helper_init(&t.oth);
	if (ret != 0) {
		delete (&t);
		return ret;
	}

	// HACK, to counter the tracking origin offset.
	t.fusion.pos.x = 0.0f;
	t.fusion.pos.y = 0.6f;
	t.fusion.pos.z = -2.0f;

	t.fusion.rot.x = 0.0f;
	t.fusion.rot.y = 1.0f;
	t.fusion.rot.z = 0.0f;
	t.fusion.rot.w = 0.0f;

	t.sbd = cv::SimpleBlobDetector::create(blob_params);
	xrt_frame_context_add(xfctx, &t.node);

	// Everything is safe, now setup the variable tracking.
	u_var_add_root(&t, "PSVR Tracker", true);
	u_var_add_sink(&t, &t.debug.sink, "Debug");

	*out_sink = &t.sink;
	*out_xtmv = &t.base;

	return 0;
}
