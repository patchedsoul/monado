#include <opencv2/opencv.hpp>

#include "tracker3D_sphere_stereo.h"
#include "common/opencv_utils.hpp"

#include "track_psvr.h"

#include <sys/stat.h>
#include <linux/limits.h>

#include <util/u_misc.h>

#define MAX_CALIBRATION_SAMPLES                                                \
	23 // mo' samples, mo' calibration accuracy, at the expense of time.

typedef struct tracker3D_sphere_stereo_instance
{
	bool configured;
	tracker_stereo_configuration_t configuration;
	measurement_consumer_callback_func measurement_target_callback;
	void* measurement_target_instance; // where we send our measurements
	event_consumer_callback_func event_target_callback;
	void* event_target_instance; // where we send our events

	tracked_object_t tracked_object;
	tracked_blob_t l_tracked_blob;
	tracked_blob_t r_tracked_blob;

	bool poses_consumed;
	cv::SimpleBlobDetector::Params blob_params;
	std::vector<cv::KeyPoint> l_keypoints;
	std::vector<cv::KeyPoint> r_keypoints;
	// these components hold no state so we can use a single instance for l
	// and r ?
	cv::Ptr<cv::SimpleBlobDetector> sbd;
	cv::Ptr<cv::BackgroundSubtractorMOG2> background_subtractor;
	cv::Mat l_frame_gray;
	cv::Mat r_frame_gray;
	cv::Mat l_mask_gray;
	cv::Mat r_mask_gray;

	cv::Mat l_frame_u;
	cv::Mat l_frame_v;
	cv::Mat r_frame_u;
	cv::Mat r_frame_v;


	cv::Mat debug_rgb;
	cv::Mat l_intrinsics;
	cv::Mat l_distortion;
	cv::Mat l_distortion_fisheye;
	cv::Mat l_translation;
	cv::Mat l_rotation;
	cv::Mat l_projection;
	cv::Mat r_intrinsics;
	cv::Mat r_distortion;
	cv::Mat r_distortion_fisheye;
	cv::Mat r_translation;
	cv::Mat r_rotation;
	cv::Mat r_projection;

	cv::Mat zero_distortion;
	cv::Mat zero_distortion_fisheye;

	cv::Matx44d disparity_to_depth;

	cv::Mat l_undistort_map_x;
	cv::Mat l_undistort_map_y;
	cv::Mat r_undistort_map_x;
	cv::Mat r_undistort_map_y;

	cv::Mat l_rectify_map_x;
	cv::Mat l_rectify_map_y;
	cv::Mat r_rectify_map_x;
	cv::Mat r_rectify_map_y;


	// calibration data structures
	std::vector<std::vector<cv::Point3f>> chessboards_model;
	std::vector<std::vector<cv::Point2f>> l_chessboards_measured;
	std::vector<std::vector<cv::Point2f>> r_chessboards_measured;

	bool calibrated;

	bool l_alloced_frames;
	bool r_alloced_frames;

	bool got_left;
	bool got_right;

} tracker3D_sphere_stereo_instance_t;

tracker3D_sphere_stereo_instance_t*
tracker3D_sphere_stereo_create(tracker_instance_t* inst)
{
	tracker3D_sphere_stereo_instance_t* i =
	    U_TYPED_CALLOC(tracker3D_sphere_stereo_instance_t);
	if (i) {
		i->blob_params.filterByArea = false;
		i->blob_params.filterByConvexity = false;
		i->blob_params.filterByInertia = false;
		i->blob_params.filterByColor = true;
		i->blob_params.blobColor =
		    255; // 0 or 255 - color comes from binarized image?
		i->blob_params.minArea = 1;
		i->blob_params.maxArea = 1000;
		i->blob_params.maxThreshold =
		    51; // using a wide threshold span slows things down bigtime
		i->blob_params.minThreshold = 50;
		i->blob_params.thresholdStep = 1;
		i->blob_params.minDistBetweenBlobs = 5;
		i->blob_params.minRepeatability = 1; // need this to avoid
		                                     // error?

		i->sbd = cv::SimpleBlobDetector::create(i->blob_params);
		i->background_subtractor =
		    cv::createBackgroundSubtractorMOG2(32, 16, false);

		i->poses_consumed = false;
		i->configured = false;
		i->l_alloced_frames = false;
		i->r_alloced_frames = false;
		i->got_left = false;
		i->got_right = false;


		// alloc our debug frame here - opencv is h,w, not w,h
		i->debug_rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

		i->zero_distortion =
		    cv::Mat(DISTORTION_SIZE, 1, CV_32F, cv::Scalar(0.0f));
		i->zero_distortion_fisheye = cv::Mat(DISTORTION_FISHEYE_SIZE, 1,
		                                     CV_32F, cv::Scalar(0.0f));

		return i;
	}
	return NULL;
}
bool
tracker3D_sphere_stereo_get_debug_frame(tracker_instance_t* inst,
                                        frame_t* frame)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;

	// wrap a frame struct around our debug cv::Mat and return it.

	frame->format = FORMAT_RGB_UINT8;
	frame->width = internal->debug_rgb.cols;
	frame->stride =
	    internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
	frame->height = internal->debug_rgb.rows;
	frame->data = internal->debug_rgb.data;
	frame->size_bytes = frame_size_in_bytes(frame);
	return true;
}
capture_parameters_t
tracker3D_sphere_stereo_get_capture_params(tracker_instance_t* inst)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	capture_parameters_t cp = {};
	switch (internal->configuration.calibration_mode) {
	case CALIBRATION_MODE_CHESSBOARD:
		cp.exposure = 0.3f;
		cp.gain = 0.01f;
		break;
	default:
		cp.exposure = 1.0 / 2048.0;
		cp.gain = 0.01f;
		break;
	}

	return cp;
}

bool
tracker3D_sphere_stereo_queue(tracker_instance_t* inst, frame_t* frame)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	if (!internal->configured) {
		printf(
		    "ERROR: you must configure this tracker before it can "
		    "accept frames\n");
		return false;
	}

	// alloc left if required - if we have a composite stereo frame, alloc
	// both left and right eyes.

	if (frame->source_id == internal->configuration.l_source_id &&
	    !internal->l_alloced_frames) {
		uint16_t eye_width = frame->width / 2;
		if (internal->configuration.split_left == true) {
			eye_width = frame->width / 2;
			internal->r_frame_gray =
			    cv::Mat(frame->height, eye_width, CV_8UC1,
			            cv::Scalar(0, 0, 0));
			internal->r_frame_u =
			    cv::Mat(frame->height, eye_width, CV_8UC1,
			            cv::Scalar(0, 0, 0));
			internal->r_frame_v =
			    cv::Mat(frame->height, eye_width, CV_8UC1,
			            cv::Scalar(0, 0, 0));
			internal->r_mask_gray =
			    cv::Mat(frame->height, eye_width, CV_8UC1,
			            cv::Scalar(0, 0, 0));
			internal->r_alloced_frames = true;
		}
		internal->l_frame_gray = cv::Mat(frame->height, eye_width,
		                                 CV_8UC1, cv::Scalar(0, 0, 0));
		internal->l_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
		                              cv::Scalar(0, 0, 0));
		internal->l_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
		                              cv::Scalar(0, 0, 0));

		internal->l_mask_gray = cv::Mat(frame->height, eye_width,
		                                CV_8UC1, cv::Scalar(0, 0, 0));
		internal->l_alloced_frames = true;
	}

	// if we have a right frame, alloc if required.

	if (frame->source_id == internal->configuration.r_source_id &&
	    !internal->r_alloced_frames) {
		uint16_t eye_width = frame->width / 2;
		internal->r_frame_gray = cv::Mat(frame->height, eye_width,
		                                 CV_8UC1, cv::Scalar(0, 0, 0));
		internal->r_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
		                              cv::Scalar(0, 0, 0));
		internal->r_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
		                              cv::Scalar(0, 0, 0));

		internal->r_mask_gray = cv::Mat(frame->height, eye_width,
		                                CV_8UC1, cv::Scalar(0, 0, 0));
		internal->r_alloced_frames = true;
	}

	// copy our data from our video buffer into our cv::Mats
	// TODO: initialise once
	cv::Mat l_chans[3];
	l_chans[0] = internal->l_frame_gray;
	l_chans[1] = internal->l_frame_u;
	l_chans[2] = internal->l_frame_v;

	cv::Mat r_chans[3];
	r_chans[0] = internal->r_frame_gray;
	r_chans[1] = internal->r_frame_u;
	r_chans[2] = internal->r_frame_v;


	if (frame->source_id == internal->configuration.l_source_id) {

		if (internal->configuration.split_left == true) {
			internal->got_left = true;
			internal->got_right = true;
			cv::Mat tmp(frame->height, frame->width, CV_8UC3,
			            frame->data);
			cv::Rect lr(internal->configuration.l_rect.tl.x,
			            internal->configuration.l_rect.tl.y,
			            internal->configuration.l_rect.br.x,
			            internal->configuration.l_rect.br.y);
			cv::Rect rr(internal->configuration.r_rect.tl.x,
			            internal->configuration.r_rect.tl.y,
			            internal->configuration.r_rect.br.x -
			                internal->configuration.r_rect.tl.x,
			            internal->configuration.r_rect.br.y);
			cv::split(tmp(lr), l_chans);
			cv::split(tmp(rr), r_chans);
		} else {
			internal->got_left = true;
			cv::Mat tmp(frame->height, frame->width, CV_8UC3,
			            frame->data);
			cv::split(tmp, l_chans);
		}
	}
	if (frame->source_id == internal->configuration.r_source_id &&
	    internal->configuration.split_left == false) {
		internal->got_right = true;
		cv::Mat tmp(frame->height, frame->width, CV_8UC3, frame->data);
		cv::split(tmp, r_chans);
	}

	// we have our pair of frames, now we can process them - we should do
	// this async, rather than in queue

	if (internal->got_left && internal->got_right) {
		switch (internal->configuration.calibration_mode) {
		case CALIBRATION_MODE_NONE:
			return tracker3D_sphere_stereo_track(inst);
			break;
		case CALIBRATION_MODE_CHESSBOARD:
			return tracker3D_sphere_stereo_calibrate(inst);
			break;
		default:
			printf("ERROR: unrecognised calibration mode\n");
			return false;
		}
	}
	return true;
}

bool
tracker3D_sphere_stereo_track(tracker_instance_t* inst)
{

	// printf("tracking...\n");
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	internal->l_keypoints.clear();
	internal->r_keypoints.clear();

	// DEBUG: dump all our planes out for inspection
	/*cv::imwrite("/tmp/l_out_y.jpg",internal->l_frame_gray);
	cv::imwrite("/tmp/r_out_y.jpg",internal->r_frame_gray);
	cv::imwrite("/tmp/l_out_u.jpg",internal->l_frame_u);
	cv::imwrite("/tmp/r_out_u.jpg",internal->r_frame_u);
	cv::imwrite("/tmp/l_out_v.jpg",internal->l_frame_v);
	cv::imwrite("/tmp/r_out_v.jpg",internal->r_frame_v);
*/

	// disabled channel combining in favour of using v plane directly - Y is
	// 2x resolution
	// so we do want to use that eventually

	// combine our yuv channels to isolate blue leds - y channel is all we
	// will use from now on
	// cv::subtract(internal->l_frame_u,internal->l_frame_v,internal->l_frame_u);
	// cv::subtract(internal->r_frame_u,internal->r_frame_v,internal->r_frame_u);

	// cv::subtract(internal->l_frame_u,internal->l_frame_gray,internal->l_frame_gray);
	// cv::subtract(internal->r_frame_u,internal->r_frame_gray,internal->r_frame_gray);

	// just use the u plane directly
	cv::bitwise_not(internal->l_frame_v, internal->l_frame_v);
	cv::bitwise_not(internal->r_frame_v, internal->r_frame_v);

	cv::threshold(internal->l_frame_v, internal->l_frame_gray, 150.0, 255.0,
	              0);
	cv::threshold(internal->r_frame_v, internal->r_frame_gray, 150.0, 255.0,
	              0);


	cv::Mat l_frame_undist;
	cv::Mat r_frame_undist;

	// undistort the whole image
	cv::remap(internal->l_frame_gray, l_frame_undist,
	          internal->l_undistort_map_x, internal->l_undistort_map_y,
	          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::remap(internal->r_frame_gray, r_frame_undist,
	          internal->r_undistort_map_x, internal->r_undistort_map_y,
	          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

	// rectify the whole image
	cv::remap(l_frame_undist, internal->l_frame_gray,
	          internal->l_rectify_map_x, internal->l_rectify_map_y,
	          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::remap(r_frame_undist, internal->r_frame_gray,
	          internal->r_rectify_map_x, internal->r_rectify_map_y,
	          cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));


	// block-match for disparity calculation - disabled

	cv::Mat disp;
	cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(128, 5);
	sbm->setNumDisparities(64);
	sbm->setBlockSize(5);
	sbm->setPreFilterCap(61);
	sbm->setPreFilterSize(15);
	sbm->setTextureThreshold(32);
	sbm->setSpeckleWindowSize(100);
	sbm->setSpeckleRange(32);
	sbm->setMinDisparity(2);
	sbm->setUniquenessRatio(10);
	sbm->setDisp12MaxDiff(0);
	//	sbm->compute(internal->l_frame_gray,
	// internal->r_frame_gray,disp); 	cv::normalize(disp, disp8, 0.1,
	// 255, CV_MINMAX, CV_8UC1);


	// disabled background subtraction for now

	// internal->background_subtractor->apply(internal->l_frame_gray,internal->l_mask_gray);
	// internal->background_subtractor->apply(internal->r_frame_gray,internal->r_mask_gray);

	xrt_vec2 lastPos = internal->l_tracked_blob.center;
	float offset = ROI_OFFSET;
	if (internal->l_tracked_blob.diameter > ROI_OFFSET) {
		offset = internal->l_tracked_blob.diameter;
	}

	// cv::rectangle(internal->l_mask_gray,
	// cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar(
	// 255 ),-1,0); lastPos = internal->r_tracked_blob.center;
	// cv::rectangle(internal->r_mask_gray,
	// cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar(
	// 255 ),-1,0);

	// cv::rectangle(internal->debug_rgb,
	// cv::Point2f(0,0),cv::Point2f(internal->debug_rgb.cols,internal->debug_rgb.rows),cv::Scalar(
	// 0,0,0 ),-1,0);

	cv::threshold(internal->l_frame_gray, internal->l_frame_gray, 32.0,
	              255.0, 0);
	cv::threshold(internal->r_frame_gray, internal->r_frame_gray, 32.0,
	              255.0, 0);

	// TODO: handle source images larger than debug_rgb
	cv::Mat debug_img;
	cv::cvtColor(internal->l_frame_gray, debug_img, CV_GRAY2BGR);
	cv::Mat dst_roi =
	    internal->debug_rgb(cv::Rect(0, 0, debug_img.cols, debug_img.rows));
	debug_img.copyTo(dst_roi);


	tracker_measurement_t m = {};

	// do blob detection with our masks
	internal->sbd->detect(internal->l_frame_gray,
	                      internal->l_keypoints); //,internal->l_mask_gray);
	internal->sbd->detect(internal->r_frame_gray,
	                      internal->r_keypoints); //,internal->r_mask_gray);

	// do some basic matching to come up with likely disparity-pairs
	std::vector<cv::KeyPoint> l_blobs, r_blobs;
	for (uint32_t i = 0; i < internal->l_keypoints.size(); i++) {
		cv::KeyPoint l_blob = internal->l_keypoints[i];
		int l_index = -1;
		int r_index = -1;
		for (uint32_t j = 0; j < internal->r_keypoints.size(); j++) {
			float lowest_dist = 128;
			cv::KeyPoint r_blob = internal->r_keypoints[j];
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
			l_blobs.push_back(internal->l_keypoints.at(l_index));
			r_blobs.push_back(internal->r_keypoints.at(r_index));
		}
	}

	// draw our disparity markers into our debug frame

	for (uint32_t i = 0; i < l_blobs.size(); i++) {
		cv::line(internal->debug_rgb, l_blobs[i].pt, r_blobs[i].pt,
		         cv::Scalar(255, 0, 0));
	}

	// convert our 2d point + disparities into 3d points.

	std::vector<cv::Point3f> world_points;
	if (l_blobs.size() > 0) {
		for (uint32_t i = 0; i < l_blobs.size(); i++) {
			float disp = r_blobs[i].pt.x - l_blobs[i].pt.x;
			cv::Vec4d xydw(l_blobs[i].pt.x, l_blobs[i].pt.y, disp,
			               1.0f);
			// Transform
			cv::Vec4d h_world = internal->disparity_to_depth * xydw;
			// Divide by scale to get 3D vector from homogeneous
			// coordinate
			world_points.push_back(cv::Point3f(
			    h_world[0] / h_world[3], h_world[1] / h_world[3],
			    h_world[2] / h_world[3]));
		}
	}

	int tracked_index = -1;
	float lowest_dist = 65535.0f;
	xrt_vec3 position = internal->tracked_object.pose.position;
	cv::Point3f last_point(position.x, position.y, position.z);

	for (uint32_t i = 0; i < world_points.size(); i++) {

		cv::Point3f world_point = world_points[i];

		// show our tracked world points (just x,y) in our debug output
		cv::Point2f img_point;
		img_point.x = world_point.x * internal->debug_rgb.cols / 2 +
		              internal->debug_rgb.cols / 2;
		img_point.y = world_point.y * internal->debug_rgb.rows / 2 +
		              internal->debug_rgb.rows / 2;
		cv::circle(internal->debug_rgb, img_point, 3,
		           cv::Scalar(0, 255, 0));

		float dist = dist_3d(world_point, last_point);
		if (dist < lowest_dist) {
			tracked_index = i;
			lowest_dist = dist;
		}
	}

	if (tracked_index != -1) {
		cv::Point3f world_point = world_points[tracked_index];

		// create our measurement for the filter
		m.has_position = true;
		m.has_rotation = false;
		m.pose.position.x = world_point.x;
		m.pose.position.y = world_point.y;
		m.pose.position.z = world_point.z;

		// update internal state
		cv::KeyPoint l_kp = l_blobs[tracked_index];
		cv::KeyPoint r_kp = l_blobs[tracked_index];

		internal->l_tracked_blob.center.x = l_kp.pt.x;
		internal->l_tracked_blob.center.y = l_kp.pt.y;
		internal->l_tracked_blob.diameter = l_kp.size;

		internal->r_tracked_blob.center.x = r_kp.pt.x;
		internal->r_tracked_blob.center.y = r_kp.pt.y;
		internal->r_tracked_blob.diameter = r_kp.size;

		internal->tracked_object.pose.position.x = world_point.x;
		internal->tracked_object.pose.position.y = world_point.y;
		internal->tracked_object.pose.position.z = world_point.z;
		internal->tracked_object.tracking_id = 1;

		char message[128];
		snprintf(message, 128, "X: %f Y: %f Z: %f", world_point.x,
		         world_point.y, world_point.z);

		cv::putText(internal->debug_rgb, message, cv::Point2i(10, 50),
		            0, 0.5f, cv::Scalar(96, 128, 192));
		if (internal->measurement_target_callback) {
			internal->measurement_target_callback(
			    internal->measurement_target_instance, &m);
		}
	}

	tracker_send_debug_frame(inst); // publish our debug frame


	return true;
}

bool
tracker3D_sphere_stereo_calibrate(tracker_instance_t* inst)
{

	printf("calibrating...\n");

	// check if we have saved calibration data. if so, just use it.
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;


	char path_string[256]; // TODO: 256 maybe not enough
	// TODO: use multiple env vars?
	char* config_path = secure_getenv("HOME");
	snprintf(path_string, 256, "%s/.config/monado/%s.calibration",
	         config_path,
	         internal->configuration
	             .configuration_filename); // TODO: hardcoded 256

	printf("TRY LOADING CONFIG FROM %s\n", path_string);
	FILE* calib_file = fopen(path_string, "rb");
	if (calib_file) {
		// read our calibration from this file
		read_mat(calib_file, &internal->l_intrinsics);
		read_mat(calib_file, &internal->r_intrinsics);
		read_mat(calib_file, &internal->l_distortion);
		read_mat(calib_file, &internal->r_distortion);
		read_mat(calib_file, &internal->l_distortion_fisheye);
		read_mat(calib_file, &internal->r_distortion_fisheye);
		read_mat(calib_file, &internal->l_rotation);
		read_mat(calib_file, &internal->r_rotation);
		read_mat(calib_file, &internal->l_translation);
		read_mat(calib_file, &internal->r_translation);
		read_mat(calib_file, &internal->l_projection);
		read_mat(calib_file, &internal->r_projection);
		cv::Mat disparity_to_depth;
		read_mat(calib_file, &disparity_to_depth);
		internal->disparity_to_depth = disparity_to_depth;
		cv::Size image_size(internal->l_frame_gray.cols,
		                    internal->l_frame_gray.rows);
		// TODO: save data indicating calibration image size
		// and multiply intrinsics accordingly

		cv::fisheye::initUndistortRectifyMap(
		    internal->l_intrinsics, internal->l_distortion_fisheye,
		    cv::noArray(), internal->l_intrinsics, image_size, CV_32FC1,
		    internal->l_undistort_map_x, internal->l_undistort_map_y);
		cv::fisheye::initUndistortRectifyMap(
		    internal->r_intrinsics, internal->r_distortion_fisheye,
		    cv::noArray(), internal->r_intrinsics, image_size, CV_32FC1,
		    internal->r_undistort_map_x, internal->r_undistort_map_y);

		cv::initUndistortRectifyMap(
		    internal->l_intrinsics, internal->zero_distortion,
		    internal->l_rotation, internal->l_projection, image_size,
		    CV_32FC1, internal->l_rectify_map_x,
		    internal->l_rectify_map_y);
		cv::initUndistortRectifyMap(
		    internal->r_intrinsics, internal->zero_distortion,
		    internal->r_rotation, internal->r_projection, image_size,
		    CV_32FC1, internal->r_rectify_map_x,
		    internal->r_rectify_map_y);

		printf("calibrated cameras! setting tracking mode\n");
		internal->calibrated = true;
		internal->configuration.calibration_mode =
		    CALIBRATION_MODE_NONE;
		// send an event to notify our driver of the switch into
		// tracking mode.
		driver_event_t e = {};
		e.type = EVENT_TRACKER_RECONFIGURED;
		internal->event_target_callback(internal->event_target_instance,
		                                e);
		return true;
	}

	// no saved file - perform interactive calibration.
	// try and find a chessboard in both images, and run the calibration.
	// - we need to define some mechanism for UI/user interaction.

	// TODO: initialise this on construction and move this to internal state
	cv::Size board_size(8, 6);
	std::vector<cv::Point3f> chessboard_model;

	for (uint32_t i = 0; i < board_size.width * board_size.height; i++) {
		cv::Point3f p(i / board_size.width, i % board_size.width, 0.0f);
		chessboard_model.push_back(p);
	}

	cv::Mat l_chessboard_measured;
	cv::Mat r_chessboard_measured;
	cv::Mat camera_rotation;
	cv::Mat camera_translation;
	cv::Mat camera_essential;
	cv::Mat camera_fundamental;

	// clear our debug image
	cv::rectangle(
	    internal->debug_rgb, cv::Point2f(0, 0),
	    cv::Point2f(internal->debug_rgb.cols, internal->debug_rgb.rows),
	    cv::Scalar(0, 0, 0), -1, 0);
	cv::Mat disp8;
	cv::cvtColor(internal->r_frame_gray, disp8, CV_GRAY2BGR);
	// disp8.copyTo(internal->debug_rgb);
	// we will collect samples continuously - the user should be able to
	// wave a chessboard around randomly while the system calibrates.. we
	// only add a sample when it increases the coverage area substantially,
	// to give the solver a decent dataset.

	bool found_left = cv::findChessboardCorners(
	    internal->l_frame_gray, board_size, l_chessboard_measured);
	bool found_right = cv::findChessboardCorners(
	    internal->r_frame_gray, board_size, r_chessboard_measured);
	char message[128];
	message[0] = 0x0;

	if (found_left && found_right) {
		std::vector<cv::Point2f> coverage;
		for (uint32_t i = 0;
		     i < internal->l_chessboards_measured.size(); i++) {
			cv::Rect brect = cv::boundingRect(
			    internal->l_chessboards_measured[i]);
			cv::rectangle(internal->debug_rgb, brect.tl(),
			              brect.br(), cv::Scalar(0, 64, 32));

			coverage.push_back(cv::Point2f(brect.tl()));
			coverage.push_back(cv::Point2f(brect.br()));
		}
		cv::Rect pre_rect = cv::boundingRect(coverage);
		cv::Rect brect = cv::boundingRect(l_chessboard_measured);
		coverage.push_back(cv::Point2f(brect.tl()));
		coverage.push_back(cv::Point2f(brect.br()));
		cv::Rect post_rect = cv::boundingRect(coverage);

		// std::cout << "COVERAGE: " << brect.area() << "\n";

		cv::rectangle(internal->debug_rgb, post_rect.tl(),
		              post_rect.br(), cv::Scalar(0, 255, 0));

		if (post_rect.area() > pre_rect.area() + 500) {
			// we will use the last n samples to calculate our
			// calibration

			if (internal->l_chessboards_measured.size() >
			    MAX_CALIBRATION_SAMPLES) {
				internal->l_chessboards_measured.erase(
				    internal->l_chessboards_measured.begin());
				internal->r_chessboards_measured.erase(
				    internal->r_chessboards_measured.begin());
			} else {
				internal->chessboards_model.push_back(
				    chessboard_model);
			}

			internal->l_chessboards_measured.push_back(
			    l_chessboard_measured);
			internal->r_chessboards_measured.push_back(
			    r_chessboard_measured);
		}

		if (internal->l_chessboards_measured.size() ==
		    MAX_CALIBRATION_SAMPLES) {
			cv::Size image_size(internal->l_frame_gray.cols,
			                    internal->l_frame_gray.rows);
			cv::Mat errors;

			// float rp_error =
			// cv::stereoCalibrate(internal->chessboards_model,internal->l_chessboards_measured,internal->r_chessboards_measured,internal->l_intrinsics,internal->l_distortion,internal->r_intrinsics,internal->r_distortion,image_size,camera_rotation,camera_translation,camera_essential,camera_fundamental,errors,0);

			float rp_error = cv::fisheye::stereoCalibrate(
			    internal->chessboards_model,
			    internal->l_chessboards_measured,
			    internal->r_chessboards_measured,
			    internal->l_intrinsics,
			    internal->l_distortion_fisheye,
			    internal->r_intrinsics,
			    internal->r_distortion_fisheye, image_size,
			    camera_rotation, camera_translation,
			    cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC);

			// we will generate undistort and rectify mappings
			// separately


			std::cout << "calibration rp_error" << rp_error << "\n";
			std::cout << "calibration camera_translation"
			          << camera_translation << "\n";

			// TODO: handle both fisheye and normal cameras -right
			// now I only have the fisheye

			cv::fisheye::initUndistortRectifyMap(
			    internal->l_intrinsics,
			    internal->l_distortion_fisheye, cv::noArray(),
			    internal->l_intrinsics, image_size, CV_32FC1,
			    internal->l_undistort_map_x,
			    internal->l_undistort_map_y);
			cv::fisheye::initUndistortRectifyMap(
			    internal->r_intrinsics,
			    internal->r_distortion_fisheye, cv::noArray(),
			    internal->r_intrinsics, image_size, CV_32FC1,
			    internal->r_undistort_map_x,
			    internal->r_undistort_map_y);

			cv::stereoRectify(
			    internal->l_intrinsics, internal->zero_distortion,
			    internal->r_intrinsics, internal->zero_distortion,
			    image_size, camera_rotation, camera_translation,
			    internal->l_rotation, internal->r_rotation,
			    internal->l_projection, internal->r_projection,
			    internal->disparity_to_depth,
			    cv::CALIB_ZERO_DISPARITY);

			cv::initUndistortRectifyMap(
			    internal->l_intrinsics, internal->zero_distortion,
			    internal->l_rotation, internal->l_projection,
			    image_size, CV_32FC1, internal->l_rectify_map_x,
			    internal->l_rectify_map_y);
			cv::initUndistortRectifyMap(
			    internal->r_intrinsics, internal->zero_distortion,
			    internal->r_rotation, internal->r_projection,
			    image_size, CV_32FC1, internal->r_rectify_map_x,
			    internal->r_rectify_map_y);

			char path_string[PATH_MAX];
			char file_string[PATH_MAX];
			// TODO: centralise this - use multiple env vars?
			char* config_path = secure_getenv("HOME");
			snprintf(path_string, PATH_MAX, "%s/.config/monado",
			         config_path);
			snprintf(
			    file_string, PATH_MAX,
			    "%s/.config/monado/%s.calibration", config_path,
			    internal->configuration.configuration_filename);

			printf("TRY WRITING CONFIG TO %s\n", file_string);
			FILE* calib_file = fopen(file_string, "wb");
			if (!calib_file) {
				// try creating it
				mkpath(path_string);
			}
			calib_file = fopen(file_string, "wb");
			if (!calib_file) {
				printf(
				    "ERROR. could not create calibration file "
				    "%s\n",
				    file_string);
			} else {
				write_mat(calib_file, &internal->l_intrinsics);
				write_mat(calib_file, &internal->r_intrinsics);
				write_mat(calib_file, &internal->l_distortion);
				write_mat(calib_file, &internal->r_distortion);
				write_mat(calib_file,
				          &internal->l_distortion_fisheye);
				write_mat(calib_file,
				          &internal->r_distortion_fisheye);
				write_mat(calib_file, &internal->l_rotation);
				write_mat(calib_file, &internal->r_rotation);
				write_mat(calib_file, &internal->l_translation);
				write_mat(calib_file, &internal->r_translation);
				write_mat(calib_file, &internal->l_projection);
				write_mat(calib_file, &internal->r_projection);
				cv::Mat disparity_to_depth(internal->disparity_to_depth);
				write_mat(calib_file,
				          &disparity_to_depth);


				fclose(calib_file);
			}

			printf("calibrated cameras! setting tracking mode\n");
			internal->calibrated = true;
			internal->configuration.calibration_mode =
			    CALIBRATION_MODE_NONE;
			// send an event to notify our driver of the switch into
			// tracking mode.
			driver_event_t e = {};
			e.type = EVENT_TRACKER_RECONFIGURED;
			internal->event_target_callback(
			    internal->event_target_instance, e);
		}

		snprintf(message, 128, "COLLECTING SAMPLE: %d/%d",
		         internal->l_chessboards_measured.size() + 1,
		         MAX_CALIBRATION_SAMPLES);
	}


	cv::drawChessboardCorners(internal->debug_rgb, board_size,
	                          l_chessboard_measured, found_left);
	cv::drawChessboardCorners(internal->debug_rgb, board_size,
	                          r_chessboard_measured, found_right);

	cv::putText(internal->debug_rgb, "CALIBRATION MODE",
	            cv::Point2i(160, 240), 0, 1.0f, cv::Scalar(192, 192, 192));
	cv::putText(internal->debug_rgb, message, cv::Point2i(160, 460), 0,
	            0.5f, cv::Scalar(192, 192, 192));

	// DEBUG: write out our image planes to confirm imagery is arriving as
	// expected
	/*cv::imwrite("/tmp/l_out_y.jpg",internal->l_frame_gray);
	cv::imwrite("/tmp/r_out_y.jpg",internal->r_frame_gray);
	cv::imwrite("/tmp/l_out_u.jpg",internal->l_frame_u);
	cv::imwrite("/tmp/r_out_u.jpg",internal->r_frame_u);
	cv::imwrite("/tmp/l_out_v.jpg",internal->l_frame_v);
	cv::imwrite("/tmp/r_out_v.jpg",internal->r_frame_v);
*/
	tracker_send_debug_frame(inst);
	printf("calibrating f end\n");
	return true;
}


bool
tracker3D_sphere_stereo_get_poses(tracker_instance_t* inst,
                                  tracked_object_t* objects,
                                  uint32_t* count)
{
	if (objects == NULL) {
		*count = TRACKED_POINTS; // tracking a single object
		return true;
	}

	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	for (uint32_t i = 0; i < 1; i++) {

		objects[i] = internal->tracked_object;
	}
	*count = 1;
	internal->poses_consumed = true;
	return true;
}

bool
tracker3D_sphere_stereo_new_poses(tracker_instance_t* inst)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	return internal->poses_consumed;
}

bool
tracker3D_sphere_stereo_configure(tracker_instance_t* inst,
                                  tracker_stereo_configuration_t* config)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	// return false if we cannot handle this config

	if (config->l_format != FORMAT_YUV444_UINT8) {
		internal->configured = false;
		return false;
	}
	internal->configuration = *config;
	internal->configured = true;
	return true;
}

void
tracker3D_sphere_stereo_register_measurement_callback(
    tracker_instance_t* inst,
    void* target_instance,
    measurement_consumer_callback_func target_func)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	internal->measurement_target_instance = target_instance;
	internal->measurement_target_callback = target_func;
}

void
tracker3D_sphere_stereo_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
	tracker3D_sphere_stereo_instance_t* internal =
	    (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	internal->event_target_instance = target_instance;
	internal->event_target_callback = target_func;
}
