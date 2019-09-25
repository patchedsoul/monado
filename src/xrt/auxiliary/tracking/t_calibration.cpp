// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Calibration code.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "util/u_sink.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_format.h"
#include "tracking/t_tracking.h"

#include <opencv2/opencv.hpp>
#include "tracking/t_calibration_opencv.h"
#include <sys/stat.h>

DEBUG_GET_ONCE_BOOL_OPTION(hsv_filter, "T_DEBUG_HSV_FILTER", false)
DEBUG_GET_ONCE_BOOL_OPTION(hsv_picker, "T_DEBUG_HSV_PICKER", false)
DEBUG_GET_ONCE_BOOL_OPTION(hsv_viewer, "T_DEBUG_HSV_VIEWER", false)

// calibration chessboard size - 7x9 'blocks' - the count
// of rows and cols refer to 'internal intersections'
#define CHESSBOARD_ROWS 6
#define CHESSBOARD_COLS 8

// we will use a number of samples spread across the frame
// to ensure a good calibration. must be > 9
#define CALIBRATION_SAMPLES 15

// set up our calibration rectangles, we will collect 9 chessboard samples
// that 'fill' these rectangular regions to get good coverage
#define COVERAGE_X 0.8f
#define COVERAGE_Y 0.8f

static cv::Rect2f calibration_rect[] = {
    cv::Rect2f(
        (1.0f - COVERAGE_X) / 2.0f, (1.0f - COVERAGE_Y) / 2.0f, 0.3f, 0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f,
               0.3f,
               0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + 2 * COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f,
               0.3f,
               0.3f),

    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f,
               (1.0f - COVERAGE_Y) / 2.0f + COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f + COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + 2 * COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f + COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),

    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f,
               (1.0f - COVERAGE_Y) / 2.0f + 2 * COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f + 2 * COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),
    cv::Rect2f((1.0f - COVERAGE_X) / 2.0f + 2 * COVERAGE_X / 3.0f,
               (1.0f - COVERAGE_Y) / 2.0f + 2 * COVERAGE_Y / 3.0f,
               0.3f,
               0.3f),
};


/*
 *
 * Structs
 *
 */

class Calibration
{
public:
	struct xrt_frame_sink base = {};

	struct
	{
		cv::Mat rgb = {};
		struct xrt_frame *frame = {};
		struct xrt_frame_sink *sink = {};
	} gui;

	std::vector<cv::Point3f> chessboard_model;
	cv::Size chessboard_size;

	struct
	{
		cv::Mat l_frame_grey;
		cv::Mat r_frame_grey;
		std::vector<std::vector<cv::Point3f>> chessboards_model;
		std::vector<std::vector<cv::Point2f>> l_chessboards_measured;
		std::vector<std::vector<cv::Point2f>> r_chessboards_measured;
		cv::Mat l_chessboard_measured;
		cv::Mat r_chessboard_measured;


		uint32_t calibration_count;
		bool calibrated;

	} state;

	cv::Mat grey;

	char text[512];
};

/*!
 * Holds `cv::Mat`s used during frame processing when processing a yuyv frame.
 */
struct t_frame_yuyv
{
public:
	//! Full frame size, each block is split across two cols.
	cv::Mat data_full = {};
	//! Half horizontal width covering a complete block of two pixels.
	cv::Mat data_half = {};
};


/*
 *
 * Small helpers.
 *
 */


static void
refresh_gui_frame(class Calibration &c, int rows, int cols)
{
	// Also dereferences the old frame.
	u_frame_create_one_off(XRT_FORMAT_R8G8B8, cols, rows, &c.gui.frame);

	c.gui.rgb = cv::Mat(rows, cols, CV_8UC3, c.gui.frame->data,
	                    c.gui.frame->stride);
}

static void
send_rgb_frame(class Calibration &c)
{
	c.gui.sink->push_frame(c.gui.sink, c.gui.frame);

	refresh_gui_frame(c, c.gui.rgb.rows, c.gui.rgb.cols);
}

static void
ensure_buffers_are_allocated(class Calibration &c, int rows, int cols)
{
	if (c.gui.rgb.cols == cols && c.gui.rgb.rows == rows) {
		return;
	}

	c.grey = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));
	c.state.l_frame_grey =
	    cv::Mat(rows, cols / 2, CV_8UC1, cv::Scalar(0, 0, 0));
	c.state.r_frame_grey =
	    cv::Mat(rows, cols / 2, CV_8UC1, cv::Scalar(0, 0, 0));

	refresh_gui_frame(c, rows, cols);
}

static void
print_txt(cv::Mat &rgb, const char *text, double fontScale)
{
	int fontFace = 0;
	int thickness = 2;
	cv::Size textSize =
	    cv::getTextSize(text, fontFace, fontScale, thickness, NULL);

	cv::Point textOrg((rgb.cols - textSize.width) / 2, textSize.height * 2);

	cv::putText(rgb, text, textOrg, fontFace, fontScale,
	            cv::Scalar(192, 192, 192), thickness);
}

static void
make_gui_str(class Calibration &c)
{
	auto &rgb = c.gui.rgb;

	int cols = 800;
	int rows = 100;
	ensure_buffers_are_allocated(c, rows, cols);

	cv::rectangle(rgb, cv::Point2f(0, 0), cv::Point2f(cols, rows),
	              cv::Scalar(0, 0, 0), -1, 0);

	print_txt(rgb, c.text, 1.0);

	send_rgb_frame(c);
}

static void
make_calibration_frame(class Calibration &c)
{
	auto &rgb = c.gui.rgb;
	if (rgb.rows == 0 || rgb.cols == 0) {
		ensure_buffers_are_allocated(c, 480, 640);
	}
	// clear  our gui frame
	cv::rectangle(c.gui.rgb, cv::Point2f(0, 0),
	              cv::Point2f(rgb.cols, rgb.rows), cv::Scalar(0, 0, 0), -1,
	              0);

	// split left and right eyes
	cv::Mat l_chans[] = {c.state.l_frame_grey};
	cv::Mat r_chans[] = {c.state.r_frame_grey};
	cv::Rect lr(0.0, 0.0, c.grey.cols / 2, c.grey.rows);
	cv::Rect rr(c.grey.cols / 2, 0.0, c.grey.cols / 2, c.grey.rows);
	cv::split(c.grey(lr), l_chans);
	cv::split(c.grey(rr), r_chans);

	bool found_left =
	    cv::findChessboardCorners(c.state.l_frame_grey, c.chessboard_size,
	                              c.state.l_chessboard_measured);
	bool found_right =
	    cv::findChessboardCorners(c.state.r_frame_grey, c.chessboard_size,
	                              c.state.r_chessboard_measured);

	// draw our current calibration guide box
	cv::Point2f bound_tl = calibration_rect[c.state.calibration_count].tl();
	bound_tl.x *= rgb.cols / 2;
	bound_tl.y *= rgb.rows;

	cv::Point2f bound_br = calibration_rect[c.state.calibration_count].br();
	bound_br.x *= rgb.cols / 2;
	bound_br.y *= rgb.rows;

	// compute our 'pre sample' coverage for this frame, and
	// display it
	std::vector<cv::Point2f> coverage;
	for (uint32_t i = 0; i < c.state.l_chessboards_measured.size(); i++) {
		cv::Rect brect =
		    cv::boundingRect(c.state.l_chessboards_measured[i]);
		cv::rectangle(rgb, brect.tl(), brect.br(),
		              cv::Scalar(0, 64, 32));
		coverage.push_back(cv::Point2f(brect.tl()));
		coverage.push_back(cv::Point2f(brect.br()));
	}
	cv::Rect pre_rect = cv::boundingRect(coverage);

	// What area of the camera have we calibrated.
	cv::rectangle(rgb, pre_rect.tl(), pre_rect.br(), cv::Scalar(0, 255, 0));

	// Draw the target rect last so it is the most visible.
	cv::rectangle(c.gui.rgb, bound_tl, bound_br, cv::Scalar(255, 0, 0));

	// if we have a valid sample (left and right), display it
	if (found_left && found_right) {
		cv::Rect brect =
		    cv::boundingRect(c.state.l_chessboard_measured);
		coverage.push_back(cv::Point2f(brect.tl()));
		coverage.push_back(cv::Point2f(brect.br()));
		cv::Rect post_rect = cv::boundingRect(coverage);

		cv::rectangle(rgb, post_rect.tl(), post_rect.br(),
		              cv::Scalar(0, 255, 0));

		// determine if we should add this sample to our list.
		// either we are still taking the first 9 samples and
		// the chessboard is in the box, or we have exceeded 9
		// samples and now want to 'push out the edges'

		bool add_sample = false;
		int coverage_threshold = c.state.l_frame_grey.cols * 0.3f *
		                         c.state.l_frame_grey.rows * 0.3f;

		// snprintf(message2, 128, "TRY TO 'PUSH OUT EDGES' WITH LARGE
		// BOARD IMAGES");

		if (c.state.calibration_count < 9) {
			// snprintf(message2, 128, "POSITION CHESSBOARD IN
			// BOX");
		}

		if (c.state.calibration_count < 9 &&
		    brect.tl().x >= bound_tl.x && brect.tl().y >= bound_tl.y &&
		    brect.br().x <= bound_br.x && brect.br().y <= bound_br.y) {
			add_sample = true;
		}
		// printf(" THRESH %d BRECT AREA %d coverage_diff
		// %d\n",coverage_threshold, brect.area(),post_rect.area() -
		// pre_rect.area());
		if (c.state.calibration_count >= 9 &&
		    brect.area() > coverage_threshold &&
		    post_rect.area() >
		        pre_rect.area() + coverage_threshold / 5) {
			add_sample = true;
		}

		if (add_sample) {
			// if we have more than our max calibration samples,
			// remove the oldest one.
			// TODO: we should not hit this condition, it is an
			// artifact of older calibration process
			if (c.state.l_chessboards_measured.size() >
			    CALIBRATION_SAMPLES) {
				c.state.l_chessboards_measured.erase(
				    c.state.l_chessboards_measured.begin());
				c.state.r_chessboards_measured.erase(
				    c.state.r_chessboards_measured.begin());
			} else {
				c.state.chessboards_model.push_back(
				    c.chessboard_model);
			}

			c.state.l_chessboards_measured.push_back(
			    c.state.l_chessboard_measured);
			c.state.r_chessboards_measured.push_back(
			    c.state.r_chessboard_measured);
			c.state.calibration_count++;
			printf("SAMPLE: %ld\n",
			       c.state.l_chessboards_measured.size());
		}
	}

	// draw our chessboards in the debug frame

	cv::drawChessboardCorners(rgb, c.chessboard_size,
	                          c.state.l_chessboard_measured, found_left);
	cv::drawChessboardCorners(rgb, c.chessboard_size,
	                          c.state.r_chessboard_measured, found_right);


	if (c.state.l_chessboards_measured.size() == CALIBRATION_SAMPLES) {
		cv::Size image_size(c.state.l_frame_grey.cols,
		                    c.state.l_frame_grey.rows);

		// we don't serialise these
		cv::Mat camera_rotation;
		cv::Mat camera_translation;
		cv::Mat camera_essential;
		cv::Mat camera_fundamental;

		struct opencv_calibration_params cp;

		cv::Mat zero_distortion =
		    cv::Mat(5, 1, CV_32F, cv::Scalar(0.0f));

		// TODO: handle both fisheye and normal cameras -right
		// now I only have the normal, for the PS4 camera

		/*float rp_error = cv::fisheye::stereoCalibrate(
		    internal->chessboards_model,
		    internal->l_chessboards_measured,
		    internal->r_chessboards_measured,
		    l_intrinsics,
		    l_distortion_fisheye,
		    r_intrinsics,
		    r_distortion_fisheye, image_size,
		    camera_rotation, camera_translation,
		    cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC);
		 */
		// non-fisheye version
		float rp_error = cv::stereoCalibrate(
		    c.state.chessboards_model, c.state.l_chessboards_measured,
		    c.state.r_chessboards_measured, cp.l_intrinsics,
		    cp.l_distortion, cp.r_intrinsics, cp.r_distortion,
		    image_size, camera_rotation, camera_translation,
		    camera_essential, camera_fundamental, 0);

		std::cout << "calibration rp_error: " << rp_error << "\n";
		std::cout << "calibration camera_translation:\n"
		          << camera_translation << "\n";

		cv::stereoRectify(cp.l_intrinsics, zero_distortion,
		                  cp.r_intrinsics, zero_distortion, image_size,
		                  camera_rotation, camera_translation,
		                  cp.l_rotation, cp.r_rotation, cp.l_projection,
		                  cp.r_projection, cp.disparity_to_depth,
		                  cv::CALIB_ZERO_DISPARITY);

		// snprintf(message2, 128, "CALIBRATION DONE RP ERROR
		// %f",rp_error);
		char path_string[PATH_MAX];
		char file_string[PATH_MAX];
		// TODO: centralise this - use multiple env vars?
		char *config_path = secure_getenv("HOME");
		snprintf(path_string, PATH_MAX, "%s/.config/monado",
		         config_path);
		snprintf(file_string, PATH_MAX,
		         "%s/.config/monado/%s.calibration", config_path,
		         "PS4_EYE");
		FILE *calib_file = fopen(file_string, "wb");
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
			write_cv_mat(calib_file, &cp.l_intrinsics);
			write_cv_mat(calib_file, &cp.r_intrinsics);
			write_cv_mat(calib_file, &cp.l_distortion);
			write_cv_mat(calib_file, &cp.r_distortion);
			write_cv_mat(calib_file, &cp.l_distortion_fisheye);
			write_cv_mat(calib_file, &cp.r_distortion_fisheye);
			write_cv_mat(calib_file, &cp.l_rotation);
			write_cv_mat(calib_file, &cp.r_rotation);
			write_cv_mat(calib_file, &cp.l_translation);
			write_cv_mat(calib_file, &cp.r_translation);
			write_cv_mat(calib_file, &cp.l_projection);
			write_cv_mat(calib_file, &cp.r_projection);
			write_cv_mat(calib_file, &cp.disparity_to_depth);

			cv::Mat mat_image_size;
			mat_image_size.create(1, 2, CV_32F);
			mat_image_size.at<float>(0, 0) = image_size.width;
			mat_image_size.at<float>(0, 1) = image_size.height;
			write_cv_mat(calib_file, &mat_image_size);

			fclose(calib_file);
		}

		printf("calibrated camera!\n");
		c.state.calibrated = true;
	}
	/*
	 * Draw text
	 */

	print_txt(rgb, "CALIBRATION MODE", 1.5);

	send_rgb_frame(c);
}


/*
 *
 * Main functions.
 *
 */

XRT_NO_INLINE static void
process_frame_yuv(class Calibration &c, struct xrt_frame *xf)
{

	int w = (int)xf->width;
	int h = (int)xf->height;

	cv::Mat data(h, w, CV_8UC3, xf->data, xf->stride);
	ensure_buffers_are_allocated(c, data.rows, data.cols);

	cv::cvtColor(data, c.gui.rgb, cv::COLOR_YUV2RGB);
	cv::cvtColor(c.gui.rgb, c.grey, cv::COLOR_RGB2GRAY);
}

XRT_NO_INLINE static void
process_frame_yuyv(class Calibration &c, struct xrt_frame *xf)
{
	/*
	 * Cleverly extract the different channels.
	 * Cr/Cb are extracted at half width.
	 */
	int w = (int)xf->width;
	int half_w = w / 2;
	int h = (int)xf->height;

	struct t_frame_yuyv f = {};

	f.data_half = cv::Mat(h, half_w, CV_8UC4, xf->data, xf->stride);
	f.data_full = cv::Mat(h, w, CV_8UC2, xf->data, xf->stride);
	ensure_buffers_are_allocated(c, f.data_full.rows, f.data_full.cols);

	cv::cvtColor(f.data_full, c.gui.rgb, cv::COLOR_YUV2RGB_YUYV);
	cv::cvtColor(f.data_full, c.grey, cv::COLOR_YUV2GRAY_YUYV);
}


/*
 *
 * Interface functions.
 *
 */

extern "C" void
t_calibration_frame(struct xrt_frame_sink *xsink, struct xrt_frame *xf)
{
	auto &c = *(class Calibration *)xsink;

#if 0
	if (xf->stereo_format != XRT_FS_STEREO_SBS) {
		snprintf(c.text, sizeof(c.text),
		         "ERROR: Not side by side stereo!");
		make_gui_str(c);
		return;
	}
#endif

	// Fill both c.gui.rgb and c.grey with the data we got.
	switch (xf->format) {
	case XRT_FORMAT_YUV888: process_frame_yuv(c, xf); break;
	case XRT_FORMAT_YUV422: process_frame_yuyv(c, xf); break;
	default:
		snprintf(c.text, sizeof(c.text), "ERROR: Bad format '%s'",
		         u_format_str(xf->format));
		make_gui_str(c);
		return;
	}

	make_calibration_frame(c);
}


/*
 *
 * Exported functions.
 *
 */

extern "C" int
t_calibration_create(struct xrt_frame_context *xfctx,
                     struct xrt_frame_sink *gui,
                     struct xrt_frame_sink **out_sink)
{

	auto &c = *(new Calibration());

	c.gui.sink = gui;

	c.base.push_frame = t_calibration_frame;

	*out_sink = &c.base;

	snprintf(c.text, sizeof(c.text), "Waiting for camera");
	make_gui_str(c);

	int ret = 0;
	if (debug_get_bool_option_hsv_filter()) {
		ret = t_debug_hsv_filter_create(xfctx, *out_sink, out_sink);
	}

	if (debug_get_bool_option_hsv_picker()) {
		ret = t_debug_hsv_picker_create(xfctx, *out_sink, out_sink);
	}

	if (debug_get_bool_option_hsv_viewer()) {
		ret = t_debug_hsv_viewer_create(xfctx, *out_sink, out_sink);
	}

	// Ensure we only get yuv or yuyv frames.
	u_sink_create_to_yuv_or_yuyv(xfctx, *out_sink, out_sink);

	c.chessboard_size = cv::Size(CHESSBOARD_COLS, CHESSBOARD_ROWS);
	for (int i = 0; i < c.chessboard_size.width * c.chessboard_size.height;
	     i++) {
		cv::Point3f p(i / c.chessboard_size.width,
		              i % c.chessboard_size.width, 0.0f);
		c.chessboard_model.push_back(p);
	}

	return ret;
}
