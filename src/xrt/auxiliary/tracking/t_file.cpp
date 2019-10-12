// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Handling of files and calibration data.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "t_calibration_opencv.h"


/*
 *
 * Pre-declar functions.
 *
 */

static bool
read_cv_mat(FILE *f, cv::Mat *m, const char *name);

static bool
write_cv_mat(FILE *f, cv::Mat *m);


/*
 *
 * Free functions.
 *
 */

extern "C" void
t_calibration_data_free(struct t_calibration_data *data)
{
	CalibrationData *d_ptr = (CalibrationData *)data;
	delete d_ptr;
}

extern "C" void
t_calibration_raw_data_free(struct t_calibration_raw_data *raw_data)
{
	CalibrationRawData *rd_ptr = (CalibrationRawData *)raw_data;
	delete rd_ptr;
}


/*
 *
 * Load functions.
 *
 */

extern "C" bool
t_file_load_stereo_calibration_v1(FILE *calib_file,
                                  struct t_calibration_data **out_data,
                                  struct t_calibration_raw_data **out_raw_data)
{
	CalibrationRawData &raw = *(new CalibrationRawData());
	CalibrationData &data = *(new CalibrationData());

	cv::Mat zero_distortion = cv::Mat(5, 1, CV_32F, cv::Scalar(0.0f));

	//! @todo Load from file.
	bool use_fisheye = false;

	// Read our calibration from this file
	// clang-format off
	read_cv_mat(calib_file, &raw.l_intrinsics, "l_intrinsics"); // 3 x 3
	read_cv_mat(calib_file, &raw.r_intrinsics, "r_intrinsics"); // 3 x 3
	read_cv_mat(calib_file, &raw.l_distortion, "l_distortion"); // 1 x 5
	read_cv_mat(calib_file, &raw.r_distortion, "r_distortion"); // 1 x 5
	read_cv_mat(calib_file, &raw.l_distortion_fisheye, "l_distortion_fisheye");
	read_cv_mat(calib_file, &raw.r_distortion_fisheye, "r_distortion_fisheye");
	read_cv_mat(calib_file, &raw.l_rotation, "l_rotation"); // 3 x 3
	read_cv_mat(calib_file, &raw.r_rotation, "r_rotation"); // 3 x 3
	read_cv_mat(calib_file, &raw.l_translation, "l_translation"); // empty
	read_cv_mat(calib_file, &raw.r_translation, "r_translation"); // empty
	read_cv_mat(calib_file, &raw.l_projection, "l_projection"); // 3 x 4
	read_cv_mat(calib_file, &raw.r_projection, "r_projection"); // 3 x 4
	read_cv_mat(calib_file, &raw.disparity_to_depth, "disparity_to_depth"); // 4 x 4
	cv::Mat mat_image_size = {};
	read_cv_mat(calib_file, &mat_image_size, "mat_image_size");

	raw.image_size_pixels.w = uint32_t(mat_image_size.at<float>(0, 0));
	raw.image_size_pixels.h = uint32_t(mat_image_size.at<float>(0, 1));
	cv::Size image_size(raw.image_size_pixels.w, raw.image_size_pixels.h);

	cv::Mat mat_new_image_size = {};
	if (read_cv_mat(calib_file, &mat_new_image_size, "mat_new_image_size")) {
		raw.new_image_size_pixels.w = uint32_t(mat_new_image_size.at<float>(0, 0));
		raw.new_image_size_pixels.h = uint32_t(mat_new_image_size.at<float>(0, 1));
	} else {
		raw.new_image_size_pixels.w = raw.image_size_pixels.w;
		raw.new_image_size_pixels.h = raw.image_size_pixels.h;
	}

	cv::Mat translation = {};
	if (read_cv_mat(calib_file, &translation, "translation")) {
		raw.translation.x = translation.at<float>(0, 0);
		raw.translation.y = translation.at<float>(0, 1);
		raw.translation.z = translation.at<float>(0, 2);
	}
	// clang-format on


	// No processing needed.
	data.disparity_to_depth = raw.disparity_to_depth;

	//! @todo Scale Our intrinsics if the frame size we request
	//              calibration for does not match what was saved

	// Generate undistortion maps - handle fisheye or rectilinear sources

	if (use_fisheye) {
		cv::fisheye::initUndistortRectifyMap(
		    raw.l_intrinsics, raw.l_distortion_fisheye, cv::noArray(),
		    raw.l_intrinsics, image_size, CV_32FC1,
		    data.l_undistort_map_x, data.l_undistort_map_y);
		cv::fisheye::initUndistortRectifyMap(
		    raw.r_intrinsics, raw.r_distortion_fisheye, cv::noArray(),
		    raw.r_intrinsics, image_size, CV_32FC1,
		    data.r_undistort_map_x, data.r_undistort_map_y);
	} else {
		cv::initUndistortRectifyMap(
		    raw.l_intrinsics, raw.l_distortion, cv::noArray(),
		    raw.l_intrinsics, image_size, CV_32FC1,
		    data.l_undistort_map_x, data.l_undistort_map_y);
		cv::initUndistortRectifyMap(
		    raw.r_intrinsics, raw.r_distortion, cv::noArray(),
		    raw.r_intrinsics, image_size, CV_32FC1,
		    data.r_undistort_map_x, data.r_undistort_map_y);
	}

	// Generate our rectification maps

	cv::initUndistortRectifyMap(
	    raw.l_intrinsics, zero_distortion, raw.l_rotation, raw.l_projection,
	    image_size, CV_32FC1, data.l_rectify_map_x, data.l_rectify_map_y);
	cv::initUndistortRectifyMap(
	    raw.r_intrinsics, zero_distortion, raw.r_rotation, raw.r_projection,
	    image_size, CV_32FC1, data.r_rectify_map_x, data.r_rectify_map_y);

	*out_data = &data;
	*out_raw_data = &raw;

	return true;
}


/*
 *
 * Save functions.
 *
 */

extern "C" bool
t_file_save_raw_data(FILE *calib_file, struct t_calibration_raw_data *raw_data)
{
	CalibrationRawData &raw = *(CalibrationRawData *)raw_data;

	write_cv_mat(calib_file, &raw.l_intrinsics);
	write_cv_mat(calib_file, &raw.r_intrinsics);
	write_cv_mat(calib_file, &raw.l_distortion);
	write_cv_mat(calib_file, &raw.r_distortion);
	write_cv_mat(calib_file, &raw.l_distortion_fisheye);
	write_cv_mat(calib_file, &raw.r_distortion_fisheye);
	write_cv_mat(calib_file, &raw.l_rotation);
	write_cv_mat(calib_file, &raw.r_rotation);
	write_cv_mat(calib_file, &raw.l_translation);
	write_cv_mat(calib_file, &raw.r_translation);
	write_cv_mat(calib_file, &raw.l_projection);
	write_cv_mat(calib_file, &raw.r_projection);
	write_cv_mat(calib_file, &raw.disparity_to_depth);

	cv::Mat mat_image_size;
	mat_image_size.create(1, 2, CV_32F);
	mat_image_size.at<float>(0, 0) = raw.image_size_pixels.w;
	mat_image_size.at<float>(0, 1) = raw.image_size_pixels.h;
	write_cv_mat(calib_file, &mat_image_size);

	cv::Mat mat_new_image_size;
	mat_new_image_size.create(1, 2, CV_32F);
	mat_new_image_size.at<float>(0, 0) = raw.new_image_size_pixels.w;
	mat_new_image_size.at<float>(0, 1) = raw.new_image_size_pixels.h;
	write_cv_mat(calib_file, &mat_new_image_size);

	cv::Mat mat_translation;
	mat_translation.create(1, 3, CV_32F);
	mat_translation.at<float>(0, 0) = raw.translation.x;
	mat_translation.at<float>(0, 1) = raw.translation.y;
	mat_translation.at<float>(0, 2) = raw.translation.z;
	write_cv_mat(calib_file, &mat_translation);

	return true;
}


/*
 *
 * Hack functions.
 *
 */

extern "C" bool
t_file_load_stereo_calibration_v1_hack(struct t_calibration_data **out_data)
{
	const char *configuration_filename = "PS4_EYE";

	char path_string[256]; //! @todo 256 maybe not enough
	//! @todo Use multiple env vars?
	char *config_path = secure_getenv("HOME");
	snprintf(path_string, 256, "%s/.config/monado/%s.calibration",
	         config_path, configuration_filename); //! @todo Hardcoded 256

	FILE *calib_file = fopen(path_string, "rb");
	if (calib_file == NULL) {
		return false;
	}

	t_calibration_raw_data *raw_data;
	bool ret =
	    t_file_load_stereo_calibration_v1(calib_file, out_data, &raw_data);

	t_calibration_raw_data_free(raw_data);

	fclose(calib_file);

	return ret;
}

extern "C" bool
t_file_save_raw_data_hack(struct t_calibration_raw_data *raw_data)
{
	char path_string[PATH_MAX];
	char file_string[PATH_MAX];
	// TODO: centralise this - use multiple env vars?
	char *config_path = secure_getenv("HOME");
	snprintf(path_string, PATH_MAX, "%s/.config/monado", config_path);
	snprintf(file_string, PATH_MAX, "%s/.config/monado/%s.calibration",
	         config_path, "PS4_EYE");
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
		return false;
	}

	t_file_save_raw_data(calib_file, raw_data);

	fclose(calib_file);

	return true;
}


/*
 *
 * Helpers
 *
 */

static bool
write_cv_mat(FILE *f, cv::Mat *m)
{
	uint32_t header[3];
	header[0] = static_cast<uint32_t>(m->elemSize());
	header[1] = static_cast<uint32_t>(m->rows);
	header[2] = static_cast<uint32_t>(m->cols);
	fwrite(static_cast<void *>(header), sizeof(uint32_t), 3, f);
	fwrite(static_cast<void *>(m->data), header[0], header[1] * header[2],
	       f);
	return true;
}

static bool
read_cv_mat(FILE *f, cv::Mat *m, const char *name)
{
	uint32_t header[3] = {};
	size_t read = 0;

	read = fread(static_cast<void *>(header), sizeof(uint32_t), 3, f);
	if (read != 3) {
		printf("Failed to read mat header: '%i' '%s'\n", (int)read,
		       name);
		return false;
	}

	//! @todo We may have written things other than CV_32F and CV_64F.
	if (header[0] == 4) {
		m->create(static_cast<int>(header[1]),
		          static_cast<int>(header[2]), CV_32F);
	} else {
		m->create(static_cast<int>(header[1]),
		          static_cast<int>(header[2]), CV_64F);
	}
	read = fread(static_cast<void *>(m->data), header[0],
	             header[1] * header[2], f);
	if (read != (header[1] * header[2])) {
		printf("Failed to read mat body: '%i' '%s'\n", (int)read, name);
		return false;
	}

	return true;
}
