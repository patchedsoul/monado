// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Handling of files and calibration data.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include "tracking/t_calibration_opencv.hpp"
#include "util/u_misc.h"
#include "util/u_json.h"
#include "math/m_eigen_interop.hpp"


/*
 *
 * Pre-declar functions.
 *
 */

static int
mkpath(char *path);

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
t_stereo_camera_calibration_free(struct t_stereo_camera_calibration **data_ptr)
{
	free(*data_ptr);
	*data_ptr = NULL;
}


/*
 *
 * Refine and create functions.
 *
 */

RemapPair
calibration_get_undistort_map(t_camera_calibration &calib,
                              cv::InputArray rectify_transform_optional,
                              cv::Mat new_camera_matrix_optional)
{
	RemapPair ret;
	CameraCalibrationWrapper wrap(calib);
	if (new_camera_matrix_optional.empty()) {
		new_camera_matrix_optional = wrap.intrinsics_mat;
	}

	//! @todo Scale Our intrinsics if the frame size we request
	//              calibration for does not match what was saved
	cv::Size image_size(calib.image_size_pixels.w,
	                    calib.image_size_pixels.h);

	if (calib.use_fisheye) {
		cv::fisheye::initUndistortRectifyMap(
		    wrap.intrinsics_mat,         // cameraMatrix
		    wrap.distortion_fisheye_mat, // distCoeffs
		    rectify_transform_optional,  // R
		    new_camera_matrix_optional,  // newCameraMatrix
		    image_size,                  // size
		    CV_32FC1,                    // m1type
		    ret.remap_x,                 // map1
		    ret.remap_y);                // map2
	} else {
		cv::initUndistortRectifyMap(
		    wrap.intrinsics_mat,        // cameraMatrix
		    wrap.distortion_mat,        // distCoeffs
		    rectify_transform_optional, // R
		    new_camera_matrix_optional, // newCameraMatrix
		    image_size,                 // size
		    CV_32FC1,                   // m1type
		    ret.remap_x,                // map1
		    ret.remap_y);               // map2
	}

	return ret;
}

StereoRectificationMaps::StereoRectificationMaps(
    t_stereo_camera_calibration &data)
{
	assert(data.view[0].image_size_pixels.w ==
	       data.view[1].image_size_pixels.w);
	assert(data.view[0].image_size_pixels.h ==
	       data.view[1].image_size_pixels.h);

	assert(data.view[0].use_fisheye == data.view[1].use_fisheye);

	cv::Size image_size(data.view[0].image_size_pixels.w,
	                    data.view[0].image_size_pixels.h);
	StereoCameraCalibrationWrapper wrapped(data);

	/*
	 * Generate our rectification maps
	 *
	 * Here cv::noArray() means zero distortion.
	 */
	if (data.view[0].use_fisheye) {
#if 0
		//! @todo for some reason this looks weird?
		// Alpha of 1.0 kinda works, not really.
		int flags = cv::CALIB_ZERO_DISPARITY;
		double balance = 0.0; // aka alpha.
		double fov_scale = 1.0;

		cv::fisheye::stereoRectify(
		    wrapped.view[0].intrinsics_mat,         // K1
		    wrapped.view[0].distortion_fisheye_mat, // D1
		    wrapped.view[1].intrinsics_mat,         // K2
		    wrapped.view[1].distortion_fisheye_mat, // D2
		    image_size,                             // imageSize
		    wrapped.camera_rotation_mat,            // R
		    wrapped.camera_translation_mat,         // tvec
		    view[0].rotation_mat,                   // R1
		    view[1].rotation_mat,                   // R2
		    view[0].projection_mat,                 // P1
		    view[1].projection_mat,                 // P2
		    disparity_to_depth_mat,                 // Q
		    flags,                                  // flags
		    cv::Size(),                             // newImageSize
		    balance,                                // balance
		    fov_scale);                             // fov_scale
#else
		// Regular stereoRectify function instead, without distortion.
		int flags = cv::CALIB_ZERO_DISPARITY;
		// The function performs the default scaling.
		float alpha = -1.0f;

		cv::stereoRectify(
		    wrapped.view[0].intrinsics_mat, // cameraMatrix1
		    cv::noArray(),                  // distCoeffs1
		    wrapped.view[1].intrinsics_mat, // cameraMatrix2
		    cv::noArray(),                  // distCoeffs2
		    image_size,                     // imageSize
		    wrapped.camera_rotation_mat,    // R
		    wrapped.camera_translation_mat, // T
		    view[0].rotation_mat,           // R1
		    view[1].rotation_mat,           // R2
		    view[0].projection_mat,         // P1
		    view[1].projection_mat,         // P2
		    disparity_to_depth_mat,         // Q
		    flags,                          // flags
		    alpha,                          // alpha
		    cv::Size(),                     // newImageSize
		    NULL,                           // validPixROI1
		    NULL);                          // validPixROI2
#endif
	} else {
		// Have the same principal point on both.
		int flags = cv::CALIB_ZERO_DISPARITY;
		// Get all of the pixels from the camera.
		float alpha = 1.0f;

		cv::stereoRectify(
		    wrapped.view[0].intrinsics_mat, // cameraMatrix1
		    /* cv::noArray(), */            // distCoeffs1
		    wrapped.view[0].distortion_mat, // distCoeffs1
		    wrapped.view[1].intrinsics_mat, // cameraMatrix2
		    /* cv::noArray(), */            // distCoeffs2
		    wrapped.view[1].distortion_mat, // distCoeffs2
		    image_size,                     // imageSize
		    wrapped.camera_rotation_mat,    // R
		    wrapped.camera_translation_mat, // T
		    view[0].rotation_mat,           // R1
		    view[1].rotation_mat,           // R2
		    view[0].projection_mat,         // P1
		    view[1].projection_mat,         // P2
		    disparity_to_depth_mat,         // Q
		    flags,                          // flags
		    alpha,                          // alpha
		    cv::Size(),                     // newImageSize
		    NULL,                           // validPixROI1
		    NULL);                          // validPixROI2
	}

	view[0].rectify = calibration_get_undistort_map(
	    data.view[0], view[0].rotation_mat, view[0].projection_mat);
	view[1].rectify = calibration_get_undistort_map(
	    data.view[1], view[1].rotation_mat, view[1].projection_mat);
}

/*
 *
 * Load functions.
 *
 */

extern "C" bool
t_stereo_camera_calibration_load_v1(
    FILE *calib_file, struct t_stereo_camera_calibration **out_data)
{
	t_stereo_camera_calibration &raw =
	    *U_TYPED_CALLOC(t_stereo_camera_calibration);
	StereoCameraCalibrationWrapper wrapped(raw);

	// Dummy matrix
	cv::Mat dummy;

	// Read our calibration from this file
	// clang-format off
	cv::Mat_<float> mat_image_size(2, 1);
	bool result = read_cv_mat(calib_file, &wrapped.view[0].intrinsics_mat, "l_intrinsics"); // 3 x 3
	result = result && read_cv_mat(calib_file, &wrapped.view[1].intrinsics_mat, "r_intrinsics"); // 3 x 3
	result = result && read_cv_mat(calib_file, &wrapped.view[0].distortion_mat, "l_distortion"); // 1 x 5
	result = result && read_cv_mat(calib_file, &wrapped.view[1].distortion_mat, "r_distortion"); // 1 x 5
	result = result && read_cv_mat(calib_file, &wrapped.view[0].distortion_fisheye_mat, "l_distortion_fisheye"); // 4 x 1
	result = result && read_cv_mat(calib_file, &wrapped.view[1].distortion_fisheye_mat, "r_distortion_fisheye"); // 4 x 1
	result = result && read_cv_mat(calib_file, &dummy, "l_rotation"); // 3 x 3
	result = result && read_cv_mat(calib_file, &dummy, "r_rotation"); // 3 x 3
	result = result && read_cv_mat(calib_file, &dummy, "l_translation"); // empty
	result = result && read_cv_mat(calib_file, &dummy, "r_translation"); // empty
	result = result && read_cv_mat(calib_file, &dummy, "l_projection"); // 3 x 4
	result = result && read_cv_mat(calib_file, &dummy, "r_projection"); // 3 x 4
	result = result && read_cv_mat(calib_file, &dummy, "disparity_to_depth");  // 4 x 4
	result = result && read_cv_mat(calib_file, &mat_image_size, "mat_image_size");

	if (!result) {
		fprintf(stderr, "\tRe-run calibration!\n");
		return false;
	}
	wrapped.view[0].image_size_pixels.w = uint32_t(mat_image_size(0, 0));
	wrapped.view[0].image_size_pixels.h = uint32_t(mat_image_size(0, 1));
	wrapped.view[1].image_size_pixels = wrapped.view[0].image_size_pixels;

	cv::Mat mat_new_image_size = mat_image_size.clone();
	if (read_cv_mat(calib_file, &mat_new_image_size, "mat_new_image_size")) {
		// do nothing particular here.
	}

	if (!read_cv_mat(calib_file, &wrapped.camera_translation_mat, "translation")) {
		fprintf(stderr, "\tRe-run calibration!\n");
	}
	if (!read_cv_mat(calib_file, &wrapped.camera_rotation_mat, "rotation")) {
		fprintf(stderr, "\tRe-run calibration!\n");
	}
	if (!read_cv_mat(calib_file, &wrapped.camera_essential_mat, "essential")) {
		fprintf(stderr, "\tRe-run calibration!\n");
	}
	if (!read_cv_mat(calib_file, &wrapped.camera_fundamental_mat, "fundamental")) {
		fprintf(stderr, "\tRe-run calibration!\n");
	}

	cv::Mat_<float> mat_use_fisheye(1, 1);
	if (!read_cv_mat(calib_file, &mat_use_fisheye, "use_fisheye")) {
		wrapped.view[0].use_fisheye = false;
		fprintf(stderr, "\tRe-run calibration! (Assuming not fisheye)\n");
	} else {
		wrapped.view[0].use_fisheye = mat_use_fisheye(0, 0) != 0.0f;
	}
	wrapped.view[1].use_fisheye = wrapped.view[0].use_fisheye;
	// clang-format on


	assert(wrapped.isDataStorageValid());
	*out_data = &raw;

	return true;
}


/*
 *
 * Save functions.
 *
 */

extern "C" bool
t_file_save_raw_data(FILE *calib_file, struct t_stereo_camera_calibration *data)
{
	StereoCameraCalibrationWrapper wrapped(*data);
	// Dummy matrix
	cv::Mat dummy;


	write_cv_mat(calib_file, &wrapped.view[0].intrinsics_mat);
	write_cv_mat(calib_file, &wrapped.view[1].intrinsics_mat);
	write_cv_mat(calib_file, &wrapped.view[0].distortion_mat);
	write_cv_mat(calib_file, &wrapped.view[1].distortion_mat);
	write_cv_mat(calib_file, &wrapped.view[0].distortion_fisheye_mat);
	write_cv_mat(calib_file, &wrapped.view[1].distortion_fisheye_mat);
	write_cv_mat(calib_file, &dummy); // view[0].rotation_mat
	write_cv_mat(calib_file, &dummy); // view[1].rotation_mat
	write_cv_mat(calib_file, &dummy); // l_translation
	write_cv_mat(calib_file, &dummy); // r_translation
	write_cv_mat(calib_file, &dummy); // view[0].projection_mat
	write_cv_mat(calib_file, &dummy); // view[1].projection_mat
	write_cv_mat(calib_file, &dummy); // disparity_to_depth_mat

	cv::Mat mat_image_size;
	mat_image_size.create(1, 2, CV_32F);
	mat_image_size.at<float>(0, 0) = wrapped.view[0].image_size_pixels.w;
	mat_image_size.at<float>(0, 1) = wrapped.view[0].image_size_pixels.h;
	write_cv_mat(calib_file, &mat_image_size);

	// "new" image size - we actually leave up to the caller now
	write_cv_mat(calib_file, &mat_image_size);

	write_cv_mat(calib_file, &wrapped.camera_translation_mat);
	write_cv_mat(calib_file, &wrapped.camera_rotation_mat);
	write_cv_mat(calib_file, &wrapped.camera_essential_mat);
	write_cv_mat(calib_file, &wrapped.camera_fundamental_mat);

	cv::Mat mat_use_fisheye;
	mat_use_fisheye.create(1, 1, CV_32F);
	mat_use_fisheye.at<float>(0, 0) = wrapped.view[0].use_fisheye;
	write_cv_mat(calib_file, &mat_use_fisheye);

	return true;
}


/*
 *
 * Hack functions.
 *
 */

extern "C" bool
t_stereo_camera_calibration_load_v1_hack(
    struct t_stereo_camera_calibration **out_data)
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

	bool ret = t_stereo_camera_calibration_load_v1(calib_file, out_data);

	fclose(calib_file);

	return ret;
}

extern "C" bool
t_file_save_raw_data_hack(struct t_stereo_camera_calibration *data)
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

	t_file_save_raw_data(calib_file, data);

	fclose(calib_file);

	return true;
}

// Named constants for key strings, to avoid typo mismatch between read and
// write
constexpr const char *HEIGHT_KEY = "height";
constexpr const char *WIDTH_KEY = "width";
constexpr const char *RESOLUTION_KEY = "resolution";
constexpr const char *INTRINSICS_KEY = "intrinsics";
constexpr const char *F_X_KEY = "f_x";
constexpr const char *F_Y_KEY = "f_y";
constexpr const char *SKEW_KEY = "s";
constexpr const char *C_X_KEY = "c_x";
constexpr const char *C_Y_KEY = "c_y";
constexpr const char *DISTORTION_KEY = "distortion";
constexpr const char *RADIAL_KEY = "radial";
constexpr const char *TANGENTIAL_KEY = "tangential";
constexpr const char *USE_FISHEYE_KEY = "use_fisheye";
constexpr const char *CALIBRATIONS_KEY = "calibrations";
constexpr const char *SENSORS_KEY = "sensors";
constexpr const char *ESSENTIAL_MATRIX_KEY = "essential_matrix";
constexpr const char *FUNDAMENTAL_MATRIX_KEY = "fundamental_matrix";
constexpr const char *TRANSFORM_KEY = "transform";
constexpr const char *TRANSLATION_KEY = "translation";
constexpr const char *ROTATION_KEY = "rotation";
constexpr const char *STEREO_CAMERA_KEY = "stereo_camera";
constexpr const char *CAMERA_KEY = "camera";

static bool
_parse_int(const cJSON *object, const char *key, int *out_val)
{
	if (!object || !key) {
		return false;
	}
	const cJSON *member = cJSON_GetObjectItemCaseSensitive(object, key);
	if (!member || !cJSON_IsNumber(member)) {
		return false;
	}
	*out_val = member->valueint;
	return true;
}

static bool
_parse_resolution(const cJSON *resolution, xrt_size *out_res)
{
	assert(resolution);
	xrt_size val{};

	bool result = _parse_int(resolution, HEIGHT_KEY, &(val.h)) &&
	              _parse_int(resolution, WIDTH_KEY, &(val.w));
	if (result) {
		out_res->h = val.h;
		out_res->w = val.w;
	}
	return result;
}

static double
_get_double_or_default(const cJSON *object, const char *key, double def)
{
	if (!object || !key) {
		return def;
	}
	const cJSON *member = cJSON_GetObjectItemCaseSensitive(object, key);
	if (!member || !cJSON_IsNumber(member)) {
		return def;
	}
	return member->valuedouble;
}

static bool
_calibration_element_in_place_from_json(const cJSON *calibration_json,
                                        t_camera_calibration *data)
{
	CameraCalibrationWrapper wrapped(*data);
	const cJSON *resolution =
	    cJSON_GetObjectItemCaseSensitive(calibration_json, RESOLUTION_KEY);
	if (!_parse_resolution(resolution, &(data->image_size_pixels))) {
		return false;
	}

	{
		const cJSON *intrinsics = cJSON_GetObjectItemCaseSensitive(
		    calibration_json, INTRINSICS_KEY);
		if (!intrinsics) {
			return false;
		}
		double f_x = _get_double_or_default(intrinsics, F_X_KEY, 1);
		double f_y = _get_double_or_default(intrinsics, F_Y_KEY, 1);
		/*!
		 * @todo use a sentinel value and estimate other focal length if
		 * missing one based on aspect ratio
		 */
		wrapped.intrinsics_mat = cv::Mat_<double>::eye(3, 3);

		wrapped.intrinsics_mat(0, 0) = f_x;
		// skew
		wrapped.intrinsics_mat(0, 1) =
		    _get_double_or_default(intrinsics, SKEW_KEY, 0);
		wrapped.intrinsics_mat(1, 1) = f_y;
		wrapped.intrinsics_mat(0, 2) = _get_double_or_default(
		    intrinsics, C_X_KEY,
		    ((double)data->image_size_pixels.w) / 2.0);
		wrapped.intrinsics_mat(1, 2) = _get_double_or_default(
		    intrinsics, C_Y_KEY,
		    ((double)data->image_size_pixels.h) / 2.0);
	}

	const cJSON *distortion =
	    cJSON_GetObjectItemCaseSensitive(calibration_json, DISTORTION_KEY);
	if (distortion) {
		if (data->use_fisheye) {
			assert(cJSON_IsArray(distortion));
			u_json_get_double_array(
			    distortion, data->distortion_fisheye,
			    ARRAY_SIZE(data->distortion_fisheye));

		} else {
			assert(cJSON_IsObject(distortion));
			static_assert(XRT_DISTORTION_MAX_DIM == 5,
			              "If this increases we can parse more "
			              "JSON parameters");
			const cJSON *radial = cJSON_GetObjectItemCaseSensitive(
			    distortion, RADIAL_KEY);

			std::array<double, 3> k{0, 0, 0};
			u_json_get_double_array(radial, k.data(), k.size());

			const cJSON *tangential =
			    cJSON_GetObjectItemCaseSensitive(distortion,
			                                     TANGENTIAL_KEY);

			std::array<double, 2> p{0, 0};
			u_json_get_double_array(tangential, p.data(), p.size());

			// Jumble the distortion coefficients the way OpenCV
			// likes.
			wrapped.distortion_mat(0) = k[0];
			wrapped.distortion_mat(1) = k[1];
			wrapped.distortion_mat(2) = p[0];
			wrapped.distortion_mat(3) = p[1];
			wrapped.distortion_mat(4) = k[2];
		}
	}
	return true;
}

static bool
t_camera_calibration_in_place_from_json(const cJSON *json,
                                        struct t_camera_calibration *data)
{
	if (!cJSON_IsObject(json)) {
		return false;
	}
	U_ZERO(data);

	CameraCalibrationWrapper wrapped(*data);
	cJSON *fisheye =
	    cJSON_GetObjectItemCaseSensitive(json, USE_FISHEYE_KEY);
	wrapped.use_fisheye = (fisheye && cJSON_IsTrue(fisheye));
	const cJSON *calibrations =
	    cJSON_GetObjectItemCaseSensitive(json, CALIBRATIONS_KEY);

	if (!cJSON_IsArray(calibrations)) {
		return false;
	}
	if (cJSON_GetArraySize(calibrations) < 1) {
		return false;
	}
	const cJSON *calibration{};
	cJSON_ArrayForEach(calibration, calibrations)
	{
		if (!_calibration_element_in_place_from_json(calibration,
		                                             data)) {
			return false;
		}
		// Using just the first calibration for now
		break;
	}

	return true;
}

bool
t_camera_calibration_from_json(const cJSON *json,
                               struct t_camera_calibration **out_data)
{
	assert(cJSON_IsObject(json));

	t_camera_calibration *raw = U_TYPED_CALLOC(t_camera_calibration);
	if (!t_camera_calibration_in_place_from_json(json, raw)) {
		free(raw);
		raw = NULL;
	}
	*out_data = raw;
	return raw != NULL;
}

static bool
_parse_matrix33(const cJSON *json, double matrix[3][3])
{
	if (!json) {
		return false;
	}
	if (!cJSON_IsArray(json)) {
		return false;
	}
	if (cJSON_GetArraySize(json) != 3) {
		return false;
	}
	size_t row_num = 0;
	const cJSON *row = NULL;
	cJSON_ArrayForEach(row, json)
	{
		if (!cJSON_IsArray(json)) {
			return false;
		}
		if (cJSON_GetArraySize(json) != 3) {
			return false;
		}
		size_t col_num = 0;

		const cJSON *elt = NULL;
		cJSON_ArrayForEach(elt, row)
		{
			if (!cJSON_IsNumber(elt)) {
				return false;
			}
			matrix[row_num][col_num] = elt->valuedouble;
			++col_num;
		}
		++row_num;
	}
	return true;
}

static bool
_parse_quat(const cJSON *json, xrt_quat &q)
{
	std::array<double, 4> qvec = {0, 0, 0, 1};
	if (qvec.size() !=
	    u_json_get_double_array(json, qvec.data(), qvec.size())) {
		return false;
	}
	q.x = qvec[0];
	q.y = qvec[1];
	q.z = qvec[2];
	q.w = qvec[3];
	return true;
}

static bool
t_stereo_camera_calibration_in_place_from_json(
    const cJSON *json, t_stereo_camera_calibration *data)
{
	U_ZERO(data);
	if (!cJSON_IsObject(json)) {
		return false;
	}

	StereoCameraCalibrationWrapper wrapped(*data);

	auto sensors = cJSON_GetObjectItemCaseSensitive(json, SENSORS_KEY);
	if (cJSON_GetArraySize(sensors) != 2) {
		return false;
	}

	{
		cJSON *sensor = NULL;
		int i = 0;
		cJSON_ArrayForEach(sensor, sensors)
		{
			if (!t_camera_calibration_in_place_from_json(
			        sensor, &(data->view[i]))) {
				return false;
			}
			++i;
		}
	}

	auto essential_matrix =
	    cJSON_GetObjectItemCaseSensitive(json, ESSENTIAL_MATRIX_KEY);
	if (!_parse_matrix33(essential_matrix, data->camera_essential)) {
		return false;
	}
	auto fundamental_matrix =
	    cJSON_GetObjectItemCaseSensitive(json, FUNDAMENTAL_MATRIX_KEY);
	if (!_parse_matrix33(fundamental_matrix, data->camera_fundamental)) {
		return false;
	}

	// init to identity
	wrapped.camera_rotation_mat = cv::Mat_<double>::eye(3, 3);
	auto transform = cJSON_GetObjectItemCaseSensitive(json, TRANSFORM_KEY);
	if (transform) {
		auto translation =
		    cJSON_GetObjectItemCaseSensitive(json, TRANSLATION_KEY);
		u_json_get_double_array(translation, data->camera_translation,
		                        ARRAY_SIZE(data->camera_translation));

		auto rotation =
		    cJSON_GetObjectItemCaseSensitive(json, ROTATION_KEY);
		xrt_quat q{};
		if (rotation && _parse_quat(rotation, q)) {
			Eigen::Matrix3d::Map(&(data->camera_rotation[0][0])) =
			    map_quat(q).toRotationMatrix().cast<double>();
		}
	}

	return true;
}
bool
t_stereo_camera_calibration_from_json(const cJSON *json,
                                      t_stereo_camera_calibration **out_data)
{

	t_stereo_camera_calibration *raw =
	    U_TYPED_CALLOC(t_stereo_camera_calibration);
	if (!t_stereo_camera_calibration_in_place_from_json(json, raw)) {
		free(raw);
		raw = NULL;
	}
	*out_data = raw;
	return raw != NULL;
}

bool
t_stereo_or_mono_camera_calibration_from_json(
    const cJSON *json,
    t_stereo_camera_calibration **out_stereo,
    t_camera_calibration **out_mono)
{
	// First, set things to null
	if (out_stereo) {
		*out_stereo = NULL;
	}
	if (out_mono) {
		*out_mono = NULL;
	}

	// Now see what's available.
	if (!json) {
		return false;
	}
	const cJSON *stereo =
	    cJSON_GetObjectItemCaseSensitive(json, STEREO_CAMERA_KEY);
	const cJSON *mono = cJSON_GetObjectItemCaseSensitive(json, CAMERA_KEY);
	if (stereo && mono) {
		// can't parse both
		return false;
	}

	if (stereo && out_stereo) {
		return t_stereo_camera_calibration_from_json(stereo,
		                                             out_stereo);
	}

	if (mono && out_mono) {
		return t_camera_calibration_from_json(mono, out_mono);
	}
	return false;
}
static unique_cJSON
array_to_json(double *arr, size_t size)
{
	assert(arr);
	unique_cJSON ret(cJSON_CreateDoubleArray(arr, size));
	return ret;
}

#define RETURN_NULL_IF_NULL(X)                                                 \
	do {                                                                   \
		if (X == nullptr) {                                            \
			return nullptr;                                        \
		}                                                              \
	} while (0)

static unique_cJSON
_calib_to_json(t_camera_calibration &calib)
{
	CameraCalibrationWrapper wrapped(calib);
	unique_cJSON root(cJSON_CreateObject());
	{
		auto calibrations =
		    cJSON_AddArrayToObject(root.get(), CALIBRATIONS_KEY);
		unique_cJSON calibration(cJSON_CreateObject());
		RETURN_NULL_IF_NULL(calibration);

		// resolution
		{
			auto resolution = cJSON_AddObjectToObject(
			    calibration.get(), RESOLUTION_KEY);
			RETURN_NULL_IF_NULL(resolution);

			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    resolution, HEIGHT_KEY, calib.image_size_pixels.h));
			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    resolution, WIDTH_KEY, calib.image_size_pixels.w));
		}

		// intrinsics
		{
			auto intrinsics = cJSON_AddObjectToObject(
			    calibration.get(), INTRINSICS_KEY);
			RETURN_NULL_IF_NULL(intrinsics);

			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    intrinsics, F_X_KEY, wrapped.intrinsics_mat(0, 0)));
			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    intrinsics, F_Y_KEY, wrapped.intrinsics_mat(1, 1)));
			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    intrinsics, C_X_KEY, wrapped.intrinsics_mat(0, 2)));
			RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
			    intrinsics, C_Y_KEY, wrapped.intrinsics_mat(1, 2)));

			if (wrapped.intrinsics_mat(0, 1) != 0.0) {
				RETURN_NULL_IF_NULL(cJSON_AddNumberToObject(
				    intrinsics, SKEW_KEY,
				    wrapped.intrinsics_mat(0, 1)));
			}
		}

		// distortion and fisheye
		if (calib.use_fisheye) {
			RETURN_NULL_IF_NULL(cJSON_AddTrueToObject(
			    calibration.get(), USE_FISHEYE_KEY));
			auto distortion_json =
			    array_to_json(calib.distortion_fisheye,
			                  ARRAY_SIZE(calib.distortion_fisheye));
			RETURN_NULL_IF_NULL(distortion_json);
			cJSON_AddItemToArray(calibration.get(),
			                     distortion_json.release());
		} else {
			auto distortion = cJSON_AddObjectToObject(
			    calibration.get(), DISTORTION_KEY);
			RETURN_NULL_IF_NULL(distortion);

			static_assert(XRT_DISTORTION_MAX_DIM == 5,
			              "If this fails, we aren't serializing "
			              "all the distortion coefficients.");

			// Unjumble the coefficients.
			double radial[] = {calib.distortion[0],
			                   calib.distortion[1],
			                   calib.distortion[4]};
			double tangential[] = {calib.distortion[2],
			                       calib.distortion[3]};

			unique_cJSON radial_json =
			    array_to_json(radial, ARRAY_SIZE(radial));
			RETURN_NULL_IF_NULL(radial_json);

			unique_cJSON tang_json =
			    array_to_json(tangential, ARRAY_SIZE(tangential));
			RETURN_NULL_IF_NULL(tang_json);
			cJSON_AddItemToObject(distortion, RADIAL_KEY,
			                      radial_json.release());
			cJSON_AddItemToObject(distortion, TANGENTIAL_KEY,
			                      tang_json.release());
		}
		cJSON_AddItemToArray(calibrations, calibration.release());
	}
	return root;
}

bool
t_camera_calibration_to_json(const t_camera_calibration *calib,
                             cJSON **out_json)
{
	assert(calib);
	assert(out_json);
	// to get a non-const object for the wrappers.
	t_camera_calibration copy(*calib);
	auto camera = _calib_to_json(copy);
	unique_cJSON ret(cJSON_CreateObject());
	if (camera && ret) {
		cJSON_AddItemToObject(ret.get(), CAMERA_KEY, camera.release());
		*out_json = ret.release();
		return true;
	}
	return false;
}

static unique_cJSON
_matrix33_to_json(double matrix[3][3])
{
	unique_cJSON mat_json{cJSON_CreateArray()};
	RETURN_NULL_IF_NULL(mat_json);
	for (size_t row = 0; row < 3; ++row) {
		unique_cJSON row_json{cJSON_CreateArray()};
		RETURN_NULL_IF_NULL(row_json);
		for (size_t col = 0; col < 3; ++col) {
			unique_cJSON elt(cJSON_CreateNumber(matrix[row][col]));
			RETURN_NULL_IF_NULL(elt);
			cJSON_AddItemToArray(row_json.get(), elt.release());
		}
		cJSON_AddItemToArray(mat_json.get(), row_json.release());
	}
	return mat_json;
}

static unique_cJSON
_stereo_calib_to_json(t_stereo_camera_calibration &calib)
{
	unique_cJSON ret(cJSON_CreateObject());
	RETURN_NULL_IF_NULL(ret);

	// Sensors
	auto sensors = cJSON_AddArrayToObject(ret.get(), SENSORS_KEY);
	RETURN_NULL_IF_NULL(sensors);
	for (auto &view : calib.view) {
		unique_cJSON view_json = _calib_to_json(view);
		RETURN_NULL_IF_NULL(view_json);
		cJSON_AddItemToArray(sensors, view_json.release());
	}

	// Transform
	auto transform = cJSON_AddObjectToObject(ret.get(), TRANSFORM_KEY);
	RETURN_NULL_IF_NULL(transform);
	{
		unique_cJSON vec_json =
		    array_to_json(calib.camera_translation,
		                  ARRAY_SIZE(calib.camera_translation));
		RETURN_NULL_IF_NULL(vec_json);
		cJSON_AddItemToObject(transform, TRANSLATION_KEY,
		                      vec_json.release());


		xrt_quat q{};
		map_quat(q) =
		    Eigen::Quaterniond(
		        Eigen::Matrix3d::Map(&(calib.camera_rotation[0][0])))
		        .cast<float>();
		double qvec[] = {q.x, q.y, q.z, q.w};
		unique_cJSON q_json = array_to_json(qvec, ARRAY_SIZE(qvec));
		RETURN_NULL_IF_NULL(q_json);
		cJSON_AddItemToObject(transform, ROTATION_KEY,
		                      q_json.release());
	}

	unique_cJSON fund_mat = _matrix33_to_json(calib.camera_fundamental);
	RETURN_NULL_IF_NULL(fund_mat);
	cJSON_AddItemToObject(ret.get(), FUNDAMENTAL_MATRIX_KEY,
	                      fund_mat.release());

	unique_cJSON essential_mat = _matrix33_to_json(calib.camera_essential);
	RETURN_NULL_IF_NULL(essential_mat);
	cJSON_AddItemToObject(ret.get(), ESSENTIAL_MATRIX_KEY,
	                      essential_mat.release());

	return ret;
}

bool
t_stereo_camera_calibration_to_json(const t_stereo_camera_calibration *calib,
                                    cJSON **out_json)
{

	assert(calib);
	assert(out_json);
	// to get a non-const object for the wrappers.
	t_stereo_camera_calibration copy(*calib);
	auto stereo_camera = _stereo_calib_to_json(copy);
	unique_cJSON ret(cJSON_CreateObject());
	if (stereo_camera && ret) {
		cJSON_AddItemToObject(ret.get(), STEREO_CAMERA_KEY,
		                      stereo_camera.release());
		*out_json = ret.release();
		return true;
	}
	return false;
}

/*
 *
 * Helpers
 *
 */

//! @todo Move this as it is a generic helper
static int
mkpath(char *path)
{
	char tmp[PATH_MAX]; //!< @todo PATH_MAX probably not strictly correct
	char *p = nullptr;
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

	if (mkdir(tmp, S_IRWXU) < 0 && errno != EEXIST) {
		return -1;
	}

	return 0;
}

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

	cv::Mat temp;
	read = fread(static_cast<void *>(header), sizeof(uint32_t), 3, f);
	if (read != 3) {
		printf("Failed to read mat header: '%i' '%s'\n", (int)read,
		       name);
		return false;
	}

	if (header[1] == 0 && header[2] == 0) {
		return true;
	}

	//! @todo We may have written things other than CV_32F and CV_64F.
	if (header[0] == 4) {
		temp.create(static_cast<int>(header[1]),
		            static_cast<int>(header[2]), CV_32F);
	} else {
		temp.create(static_cast<int>(header[1]),
		            static_cast<int>(header[2]), CV_64F);
	}
	read = fread(static_cast<void *>(temp.data), header[0],
	             header[1] * header[2], f);
	if (read != (header[1] * header[2])) {
		printf("Failed to read mat body: '%i' '%s'\n", (int)read, name);
		return false;
	}
	if (temp.type() != m->type()) {
		printf("Mat body type does not match: %i vs %i for '%s'\n",
		       (int)temp.type(), (int)m->type(), name);
		return false;
	}
	if (temp.total() != m->total()) {
		printf("Mat total size does not match: %i vs %i for '%s'\n",
		       (int)temp.total(), (int)m->total(), name);
		return false;
	}
	if (temp.size() == m->size()) {
		// Exact match
		temp.copyTo(*m);
		return true;
	}
	if (temp.size().width == m->size().height &&
	    temp.size().height == m->size().width) {
		printf("Mat transposing on load: '%s'\n", name);
		// needs transpose
		cv::transpose(temp, *m);
		return true;
	}
	// highly unlikely so minimally-helpful error message.
	printf("Mat dimension unknown mismatch: '%s'\n", name);
	return false;
}
