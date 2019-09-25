// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  OpenCV calibration helpers.
 * @author Pete Black <pblack@collabora.com>
 */

#ifndef T_CALIBRATION_H
#define T_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#ifdef __cplusplus
extern "C" {
#endif


struct opencv_calibration_params
{
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
	cv::Mat disparity_to_depth;
	cv::Mat mat_image_size;
};

static bool
write_cv_mat(FILE* f, cv::Mat* m)
{
	uint32_t header[3];
	header[0] = static_cast<uint32_t>(m->elemSize());
	header[1] = static_cast<uint32_t>(m->rows);
	header[2] = static_cast<uint32_t>(m->cols);
	fwrite(static_cast<void*>(header), sizeof(uint32_t), 3, f);
	fwrite(static_cast<void*>(m->data), header[0], header[1] * header[2],
	       f);
	return true;
}

static bool
read_cv_mat(FILE* f, cv::Mat* m)
{
	uint32_t header[3];
	fread(static_cast<void*>(header), sizeof(uint32_t), 3, f);
	// TODO: we may have written things other than CV_32F and CV_64F.
	if (header[0] == 4) {
		m->create(static_cast<int>(header[1]),
		          static_cast<int>(header[2]), CV_32F);
	} else {
		m->create(static_cast<int>(header[1]),
		          static_cast<int>(header[2]), CV_64F);
	}
	fread(static_cast<void*>(m->data), header[0], header[1] * header[2], f);
	return true;
}

static bool
calibration_get_stereo(char* configuration_filename,
                       uint32_t frame_w,
                       uint32_t frame_h,
                       bool use_fisheye,
                       cv::Mat* l_undistort_map_x,
                       cv::Mat* l_undistort_map_y,
                       cv::Mat* l_rectify_map_x,
                       cv::Mat* l_rectify_map_y,
                       cv::Mat* r_undistort_map_x,
                       cv::Mat* r_undistort_map_y,
                       cv::Mat* r_rectify_map_x,
                       cv::Mat* r_rectify_map_y,
                       cv::Mat* disparity_to_depth)
{

	struct opencv_calibration_params cp;
	cv::Mat zero_distortion = cv::Mat(5, 1, CV_32F, cv::Scalar(0.0f));

	char path_string[256]; // TODO: 256 maybe not enough
	// TODO: use multiple env vars?
	char* config_path = secure_getenv("HOME");
	snprintf(path_string, 256, "%s/.config/monado/%s.calibration",
	         config_path, configuration_filename); // TODO: hardcoded 256

	FILE* calib_file = fopen(path_string, "rb");
	if (calib_file) {
		// read our calibration from this file
		read_cv_mat(calib_file, &cp.l_intrinsics);
		read_cv_mat(calib_file, &cp.r_intrinsics);
		read_cv_mat(calib_file, &cp.l_distortion);
		read_cv_mat(calib_file, &cp.r_distortion);
		read_cv_mat(calib_file, &cp.l_distortion_fisheye);
		read_cv_mat(calib_file, &cp.r_distortion_fisheye);
		read_cv_mat(calib_file, &cp.l_rotation);
		read_cv_mat(calib_file, &cp.r_rotation);
		read_cv_mat(calib_file, &cp.l_translation);
		read_cv_mat(calib_file, &cp.r_translation);
		read_cv_mat(calib_file, &cp.l_projection);
		read_cv_mat(calib_file, &cp.r_projection);
		read_cv_mat(calib_file,
		            disparity_to_depth); // provided by caller
		read_cv_mat(calib_file, &cp.mat_image_size);

		// TODO: scale our intrinsics if the frame size we request
		// calibration for does not match what was saved

		cv::Size image_size(int(cp.mat_image_size.at<float>(0, 0)),
		                    int(cp.mat_image_size.at<float>(0, 1)));

		// generate our undistortion maps - handle fisheye or
		// rectilinear sources

		if (use_fisheye) {
			cv::fisheye::initUndistortRectifyMap(
			    cp.l_intrinsics, cp.l_distortion_fisheye,
			    cv::noArray(), cp.l_intrinsics, image_size,
			    CV_32FC1, *l_undistort_map_x, *l_undistort_map_y);
			cv::fisheye::initUndistortRectifyMap(
			    cp.r_intrinsics, cp.r_distortion_fisheye,
			    cv::noArray(), cp.r_intrinsics, image_size,
			    CV_32FC1, *r_undistort_map_x, *r_undistort_map_y);
		} else {
			cv::initUndistortRectifyMap(
			    cp.l_intrinsics, cp.l_distortion, cv::noArray(),
			    cp.l_intrinsics, image_size, CV_32FC1,
			    *l_undistort_map_x, *l_undistort_map_y);
			cv::initUndistortRectifyMap(
			    cp.r_intrinsics, cp.r_distortion, cv::noArray(),
			    cp.r_intrinsics, image_size, CV_32FC1,
			    *r_undistort_map_x, *r_undistort_map_y);
		}

		// generate our rectification maps

		cv::initUndistortRectifyMap(cp.l_intrinsics, zero_distortion,
		                            cp.l_rotation, cp.l_projection,
		                            image_size, CV_32FC1,
		                            *l_rectify_map_x, *l_rectify_map_y);
		cv::initUndistortRectifyMap(cp.r_intrinsics, zero_distortion,
		                            cp.r_rotation, cp.r_projection,
		                            image_size, CV_32FC1,
		                            *r_rectify_map_x, *r_rectify_map_y);

		return true;
	}
	return false;
}

// TODO - move this as it is a generic helper
static int
mkpath(char* path)
{
	char tmp[PATH_MAX]; // TODO: PATH_MAX probably not strictly correct
	char* p = nullptr;
	size_t len;

	snprintf(tmp, sizeof(tmp), "%s", path);
	len = strlen(tmp) - 1;
	if (tmp[len] == '/') {
		tmp[len] = 0;
	}
	for (p = tmp + 1; *p; p++) {
		if (*p == '/') {
			*p = 0;
			if (mkdir(tmp, S_IRWXU) < 0 && errno != EEXIST)
				return -1;
			*p = '/';
		}
	}
	if (mkdir(tmp, S_IRWXU) < 0 && errno != EEXIST)
		return -1;
	return 0;
}


// TODO: templatise these 2?
static float
cv_dist3d_point(cv::Point3f& p, cv::Point3f& q)
{
	cv::Point3f d = p - q;
	return cv::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

static float
cv_dist3d_vec(cv::Vec3f& p, cv::Vec3f& q)
{
	cv::Point3f d = p - q;
	return cv::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

#ifdef __cplusplus
}
#endif


#endif // T_CALIBRATION_H
