// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR tracker code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_tracking.h"

#include "tracking/t_tracking.h"

#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_format.h"

#include "math/m_api.h"

#include "os/os_threading.h"

#include <stdio.h>
#include <assert.h>
#include <pthread.h>

#include <permutations.hpp>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#define PSVR_NUM_LEDS 7

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

typedef enum led_tag
{
	NONE,
	TL,
	TR,
	C,
	BL,
	BR,
	SL,
	SR
} led_tag_t;

typedef struct model_vertex
{
	uint32_t model_index;
	Eigen::Vector3f position;
	led_tag_t tag;

	bool
	operator<(const model_vertex &mv) const
	{
		return (model_index < mv.model_index);
	}
	bool
	operator>(const model_vertex &mv) const
	{
		return (model_index > mv.model_index);
	}

} model_vertex_t;

typedef struct match_vertex
{
	float angle;
	float distance;
	uint32_t vertex_index;
} match_vertex_t;


typedef struct match
{
	std::vector<match_vertex_t> verts;
} match_t;



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

	//! Have we received a new IMU sample.
	bool has_imu = false;

	timepoint_ns last_imu{0};

	struct
	{
		struct xrt_vec3 pos = {};
		struct xrt_quat rot = {};
	} fusion;

	model_vertex_t model_vertices[PSVR_NUM_LEDS];
	cv::KalmanFilter track_filters[PSVR_NUM_LEDS];

	View view[2];

	bool calibrated;

	cv::Mat disparity_to_depth;

	cv::Ptr<cv::SimpleBlobDetector> sbd;
	std::vector<cv::KeyPoint> l_blobs, r_blobs;
	std::vector<match_t> matches;
};

static float
dist_3d(Eigen::Vector3f a, Eigen::Vector3f b)
{
	return sqrt(a[0] - b[0]) * (a[0] - b[0]) +
               (a[1] - b[1]) * (a[1] - b[1]) +
               (a[2] - b[2]) * (a[2] - b[2]);
}

static void
init_filter(cv::KalmanFilter &kf, float process_cov, float meas_cov, float dt)
{
	kf.init(6, 3);
	kf.transitionMatrix =
	    (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	     1.0);

	cv::setIdentity(kf.measurementMatrix, cv::Scalar::all(1.0f));
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.0f));

	// our filter parameters set the process and measurement noise
	// covariances.

	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(process_cov));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(meas_cov));
}

static void
filter_predict(model_vertex_t *pose, cv::KalmanFilter *filters, float dt)
{
	for (uint32_t i = 0; i < PSVR_NUM_LEDS; i++) {
		model_vertex_t *current_led = pose + i;
		cv::KalmanFilter *current_kf = filters + i;

		// set our dt components in the transition matrix
		current_kf->transitionMatrix.at<float>(0, 3) = dt;
		current_kf->transitionMatrix.at<float>(1, 4) = dt;
		current_kf->transitionMatrix.at<float>(2, 5) = dt;

		current_led->model_index = i;
		current_led->tag =
		    (led_tag_t)(i + 1); // increment, as 0 is TAG_NONE
		cv::Mat prediction = current_kf->predict();
		current_led->position[0] = prediction.at<float>(0, 0);
		current_led->position[1] = prediction.at<float>(1, 0);
		current_led->position[2] = prediction.at<float>(2, 0);
	}
}

static void
filter_update(model_vertex_t *pose, cv::KalmanFilter *filters, float dt)
{
	for (uint32_t i = 0; i < PSVR_NUM_LEDS; i++) {
		model_vertex_t *current_led = pose + i;
		cv::KalmanFilter *current_kf = filters + i;

		// set our dt components in the transition matrix
		current_kf->transitionMatrix.at<float>(0, 3) = dt;
		current_kf->transitionMatrix.at<float>(1, 4) = dt;
		current_kf->transitionMatrix.at<float>(2, 5) = dt;

		current_led->model_index = i;
		current_led->tag =
		    (led_tag_t)(i + 1); // increment, as 0 is TAG_NONE
		cv::Mat measurement = cv::Mat(3, 1, CV_32F);
		measurement.at<float>(0, 0) = current_led->position[0];
		measurement.at<float>(1, 0) = current_led->position[1];
		measurement.at<float>(2, 0) = current_led->position[2];
		current_kf->correct(measurement);
	}
}

static void
remove_outliers(std::vector<cv::Point3f> *orig_points,
                std::vector<Eigen::Vector3f> *pruned_points,
                float outlier_thresh)
{
	// immediately prune anything that is measured as
	// 'behind' the camera
	std::vector<Eigen::Vector3f> temp_points;

	for (uint32_t i = 0; i < orig_points->size(); i++) {
		cv::Point3f p = orig_points->at(i);
		if (p.z < 0) {
			temp_points.push_back(Eigen::Vector3f(p.x, p.y, p.z));
		}
	}
	std::vector<float> x_values;
	std::vector<float> y_values;
	std::vector<float> z_values;
	for (uint32_t i = 0; i < temp_points.size(); i++) {
		x_values.push_back(temp_points[i][0]);
		y_values.push_back(temp_points[i][1]);
		z_values.push_back(temp_points[i][2]);
	}

	std::nth_element(x_values.begin(),
	                 x_values.begin() + x_values.size() / 2,
	                 x_values.end());
	float median_x = x_values[x_values.size() / 2];
	std::nth_element(y_values.begin(),
	                 y_values.begin() + y_values.size() / 2,
	                 y_values.end());
	float median_y = y_values[y_values.size() / 2];
	std::nth_element(z_values.begin(),
	                 z_values.begin() + z_values.size() / 2,
	                 z_values.end());
	float median_z = z_values[z_values.size() / 2];

	for (uint32_t i = 0; i < temp_points.size(); i++) {
		float error_x = temp_points[i][0] - median_x;
		float error_y = temp_points[i][1] - median_y;
		float error_z = temp_points[i][2] - median_z;

		float rms_error =
		    sqrt((error_x * error_x) + (error_y * error_y) +
		         (error_z * error_z));

		if (rms_error < outlier_thresh) {
			pruned_points->push_back(temp_points[i]);
		}
	}
}

static void
match_triangles(Eigen::Matrix4f *t1_mat,
                Eigen::Matrix4f *t1_to_t2_mat,
                Eigen::Vector3f t1_a,
                Eigen::Vector3f t1_b,
                Eigen::Vector3f t1_c,
                Eigen::Vector3f t2_a,
                Eigen::Vector3f t2_b,
                Eigen::Vector3f t2_c)
{
	*t1_mat = Eigen::Matrix4f().Identity();
	Eigen::Matrix4f t2_mat = Eigen::Matrix4f().Identity();

	Eigen::Vector3f t1_x_vec = (t1_b - t1_a).normalized();
	Eigen::Vector3f t1_z_vec =
	    (t1_c - t1_a).cross(t1_b - t1_a).normalized();
	Eigen::Vector3f t1_y_vec = t1_x_vec.cross(t1_z_vec).normalized();

	Eigen::Vector3f t2_x_vec = (t2_b - t2_a).normalized();
	Eigen::Vector3f t2_z_vec =
	    (t2_c - t2_a).cross(t2_b - t2_a).normalized();
	Eigen::Vector3f t2_y_vec = t2_x_vec.cross(t2_z_vec).normalized();

	t1_mat->col(0) << t1_x_vec[0], t1_x_vec[1], t1_x_vec[2], 0.0f;
	t1_mat->col(1) << t1_y_vec[0], t1_y_vec[1], t1_y_vec[2], 0.0f;
	t1_mat->col(2) << t1_z_vec[0], t1_z_vec[1], t1_z_vec[2], 0.0f;
	t1_mat->col(3) << t1_x_vec[0], t1_x_vec[1], t1_x_vec[2], 1.0f;

	t2_mat.col(0) << t2_x_vec[0], t2_x_vec[1], t2_x_vec[2], 0.0f;
	t2_mat.col(1) << t2_y_vec[0], t2_y_vec[1], t2_y_vec[2], 0.0f;
	t2_mat.col(2) << t2_z_vec[0], t2_z_vec[1], t2_z_vec[2], 0.0f;
	t2_mat.col(3) << t2_x_vec[0], t2_x_vec[1], t2_x_vec[2], 1.0f;

	*t1_to_t2_mat = t1_mat->inverse() * t2_mat;
}

static void
create_model(TrackerPSVR &t)
{
	t.model_vertices[0] = {0, Eigen::Vector3f(-2.51408f, 3.77113f, 0.0f),
	                       TL};
	t.model_vertices[0] = {1, Eigen::Vector3f(-2.51408f, -3.77113f, 0.0f),
	                       TR};
	t.model_vertices[0] = {2, Eigen::Vector3f(0.0f, 0.0f, 1.07253f), C};
	t.model_vertices[0] = {3, Eigen::Vector3f(2.51408f, 3.77113f, 0.0f),
	                       BL};
	t.model_vertices[0] = {4, Eigen::Vector3f(2.51408f, -3.77113f, 0.0f),
	                       BR};
	t.model_vertices[0] = {5, Eigen::Vector3f(0.0f, 4.52535f, -3.36583f),
	                       SL};
	t.model_vertices[0] = {6, Eigen::Vector3f(0.0f, -4.52535f, -3.36584f),
	                       SR};
}

static void
create_match_list(TrackerPSVR &t)
{
	// create our permutation list
	for (auto &&vec : iter::permutations(t.model_vertices)) {
		match_t m;
		match_vertex_t mv;
		model_vertex_t ref_pt_a = vec[0];
		model_vertex_t ref_pt_b = vec[1];
		Eigen::Vector3f ref_vec = ref_pt_b.position - ref_pt_a.position;
		float normScale = dist_3d(ref_pt_a.position, ref_pt_b.position);
		for (auto &&i : vec) {
			mv.distance =
			    dist_3d(i.position, ref_pt_a.position) / normScale;
			Eigen::Vector3f plane_norm =
			    ref_vec.cross(i.position).normalized();
			if (ref_vec.normalized() != i.position.normalized()) {
				mv.vertex_index = i.model_index;
				if (plane_norm.normalized().z() < 0) {
					mv.angle =
					    -1 *
					    acos(i.position.normalized().dot(
					        ref_vec.normalized()));
				} else {
					mv.angle =
					    acos(i.position.normalized().dot(
					        ref_vec.normalized()));
				}
			} else {
				mv.angle = 0.0f;
			}
			m.verts.push_back(mv);
		}
		t.matches.push_back(m);
	}
}

static void
do_view(TrackerPSVR &t, View &view, cv::Mat &grey)
{
	// Undistort the whole image.
	cv::remap(grey,                 // src
	          view.frame_undist,    // dst
	          view.undistort_map_x, // map1
	          view.undistort_map_y, // map2
	          cv::INTER_LINEAR,     // interpolation
	          cv::BORDER_CONSTANT,  // borderMode
	          cv::Scalar(0, 0, 0)); // borderValue

	// Rectify the whole image.
	cv::remap(view.frame_undist,    // src
	          view.frame_rectified, // dst
	          view.rectify_map_x,   // map1
	          view.rectify_map_y,   // map2
	          cv::INTER_LINEAR,     // interpolation
	          cv::BORDER_CONSTANT,  // borderMode
	          cv::Scalar(0, 0, 0)); // borderValue

	cv::threshold(view.frame_rectified, // src
	              view.frame_rectified, // dst
	              32.0,                 // thresh
	              255.0,                // maxval
	              0);                   // type

	// tracker_measurement_t m = {};

	// Do blob detection with our masks.
	//! @todo Re-enable masks.
	t.sbd->detect(view.frame_rectified, // image
	              view.keypoints,       // keypoints
	              cv::noArray());       // mask
}

static void
process(TrackerPSVR &t, struct xrt_frame *xf)
{
	// Only IMU data
	if (xf == NULL) {
		return;
	}
	float dt = 1.0f;
	// get our predicted filtered led positions, if any
	model_vertex_t predicted_pose[PSVR_NUM_LEDS];
	filter_predict(predicted_pose, t.track_filters, dt);

	model_vertex_t measured_pose[PSVR_NUM_LEDS];

	// get our raw measurements

	t.view[0].keypoints.clear();
	t.view[1].keypoints.clear();
	t.l_blobs.clear();
	t.r_blobs.clear();

	int cols = xf->width / 2;
	int rows = xf->height;
	int stride = xf->stride;

	cv::Mat l_grey(rows, cols, CV_8UC1, xf->data, stride);
	cv::Mat r_grey(rows, cols, CV_8UC1, xf->data + cols, stride);


	do_view(t, t.view[0], l_grey);
	do_view(t, t.view[1], r_grey);

	// do some basic matching to come up with likely disparity-pairs.

	for (uint32_t i = 0; i < t.view[0].keypoints.size(); i++) {
		cv::KeyPoint l_blob = t.view[0].keypoints[i];
		int l_index = -1;
		int r_index = -1;

		for (uint32_t j = 0; j < t.view[1].keypoints.size(); j++) {
			float lowest_dist = 128;
			cv::KeyPoint r_blob = t.view[1].keypoints[j];
			// find closest point on same-ish scanline
			if ((l_blob.pt.y < r_blob.pt.y + 3) &&
			    (l_blob.pt.y > r_blob.pt.y - 3) &&
			    ((r_blob.pt.x - l_blob.pt.x) < lowest_dist)) {
				lowest_dist = r_blob.pt.x - l_blob.pt.x;
				r_index = j;
				l_index = i;
			}
		}

		if (l_index > -1 && r_index > -1) {
			t.l_blobs.push_back(t.view[0].keypoints.at(l_index));
			t.r_blobs.push_back(t.view[1].keypoints.at(r_index));
		}
	}

	// Convert our 2d point + disparities into 3d points.
	std::vector<cv::Point3f> world_points;
	if (t.l_blobs.size() > 0) {
		for (uint32_t i = 0; i < t.l_blobs.size(); i++) {
			float disp = t.r_blobs[i].pt.x - t.l_blobs[i].pt.x;
			cv::Vec4d xydw(t.l_blobs[i].pt.x, t.l_blobs[i].pt.y,
			               disp, 1.0f);
			// Transform
			cv::Vec4d h_world =
			    (cv::Matx44d)t.disparity_to_depth * xydw;

			// Divide by scale to get 3D vector from homogeneous
			// coordinate. invert x while we are here.
			world_points.push_back(cv::Point3f(
			    -h_world[0] / h_world[3], h_world[1] / h_world[3],
			    h_world[2] / h_world[3]));
		}
	}


	// remove outliers from our measurement list
	std::vector<Eigen::Vector3f> pruned_points;
	remove_outliers(&world_points, &pruned_points, 10);

	// try matching our leds against the predictions
	// if error low, fit model using raw and filtered positions

	// if error too large, brute-force disambiguate
	// fit model to raw measurements only

	// update filter

	filter_update(measured_pose, t.track_filters, dt);



	xrt_frame_reference(&xf, NULL);
}

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
         timepoint_ns timestamp_ns,
         struct xrt_tracking_sample *sample)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}
	if (t.last_imu != 0) {
		time_duration_ns delta_ns = timestamp_ns - t.last_imu;
		float dt = time_ns_to_s(delta_ns);
		// Super simple fusion.
		math_quat_integrate_velocity(
		    &t.fusion.rot, &sample->gyro_rad_secs, dt, &t.fusion.rot);
	}
	t.last_imu = timestamp_ns;

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
t_psvr_push_imu(struct xrt_tracked_psvr *xtvr,
                timepoint_ns timestamp_ns,
                struct xrt_tracking_sample *sample)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	imu_data(t, timestamp_ns, sample);
}

extern "C" void
t_psvr_get_tracked_pose(struct xrt_tracked_psvr *xtvr,
                        struct time_state *timestate,
                        timepoint_ns when_ns,
                        struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	get_pose(t, timestate, when_ns, out_relation);
}

extern "C" void
t_psvr_fake_destroy(struct xrt_tracked_psvr *xtvr)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
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
t_psvr_start(struct xrt_tracked_psvr *xtvr)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	int ret;



	// TODO: remove 'impossible' permutations

	ret = os_thread_helper_start(&t.oth, t_psvr_run, &t);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

extern "C" int
t_psvr_create(struct xrt_frame_context *xfctx,
              struct t_stereo_camera_calibration *data,
              struct xrt_tracked_psvr **out_xtvr,
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

	xrt_frame_context_add(xfctx, &t.node);

	*out_sink = &t.sink;
	*out_xtvr = &t.base;

	return 0;
}
