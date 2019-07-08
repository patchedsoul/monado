#ifndef FILTER_OPENCV_KALMAN_H
#define FILTER_OPENCV_KALMAN_H

#include <xrt/xrt_defines.h>
#include "common/filter.h"
#include "common/measurementqueue.h"

typedef struct opencv_filter_configuration
{
	float measurement_noise_cov;
	float process_noise_cov;
} opencv_filter_configuration_t;

#ifdef __cplusplus
extern "C" {
#endif

// forward declare this, as it contains C++ stuff
typedef struct filter_opencv_kalman_instance_t filter_opencv_kalman_instance_t;


filter_opencv_kalman_instance_t*
filter_opencv_kalman_create(filter_instance_t* inst);
bool
filter_opencv_kalman_destroy(filter_instance_t* inst);
bool
filter_opencv_kalman_queue(filter_instance_t* inst,
                           tracker_measurement_t* measurement);

bool
filter_opencv_kalman_get_state(filter_instance_t* inst, filter_state_t* state);
bool
filter_opencv_kalman_set_state(filter_instance_t* inst, filter_state_t* state);
bool
filter_opencv_kalman_predict_state(filter_instance_t* inst,
                                   filter_state_t*,
                                   timepoint_ns time);
bool
filter_opencv_kalman_configure(filter_instance_t* inst,
                               filter_configuration_ptr config);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FILTER_OPENCV_KALMAN_H
