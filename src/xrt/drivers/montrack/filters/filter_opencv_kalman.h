#ifndef FILTER_OPENCV_KALMAN_H
#define FILTER_OPENCV_KALMAN_H

#include <xrt/xrt_defines.h>
#include "common/filter.h"

typedef struct opencv_filter_configuration
{
	float measurement_noise_cov;
	float process_noise_cov;
} opencv_filter_configuration_t;

typedef struct opencv_kalman_filter_state
{
	struct xrt_pose pose;
} opencv_kalman_filter_state_t;

#ifdef __cplusplus
extern "C" {
#endif

// forward declare this, as it contains C++ stuff
struct filter_opencv_kalman;


struct filter_opencv_kalman*
filter_opencv_kalman_create();

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FILTER_OPENCV_KALMAN_H
