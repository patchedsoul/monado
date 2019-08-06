#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

//separate file to avoid polluting C code with C++/opencv classes

#include <xrt/xrt_defines.h>
#include <opencv/cv.hpp>

#ifdef __cplusplus
extern "C" {
#endif

bool calibration_get_stereo(char* configuration_filename,bool use_fisheye, cv::Mat* l_undistort_map_x,cv::Mat* l_undistort_map_y,cv::Mat* l_rectify_map_x,cv::Mat* l_rectify_map_y,cv::Mat* r_undistort_map_x,cv::Mat* r_undistort_map_y,cv::Mat* r_rectify_map_x,cv::Mat* r_rectify_map_y,cv::Mat* disparity_to_depth);
bool calibration_get_mono(char* configuration_filename,bool use_fisheye, cv::Mat* undistort_map_x,cv::Mat* undistort_map_y);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // CALIBRATION_HPP
