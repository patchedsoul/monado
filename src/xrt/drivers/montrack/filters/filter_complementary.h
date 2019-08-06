#ifndef FILTER_COMPLEMENTARY_H
#define FILTER_COMPLEMENTARY_H

#include <xrt/xrt_defines.h>
#include "common/filter.h"
#include "common/measurementqueue.h"
#include <stdlib.h>

typedef struct filter_complementary_configuration
{
	float bias;
	float scale;
    uint64_t max_timestamp; //wrap timestamp here
    float accel_to_g;
    float gyro_to_radians_per_second;
    float drift_z_to_zero;
} filter_complementary_configuration_t;

#ifdef __cplusplus
extern "C" {
#endif

// forward declare this, as it contains C++ stuff
typedef struct filter_complementary_instance_t filter_complementary_instance_t;


filter_complementary_instance_t*
filter_complementary_create(filter_instance_t* inst);
bool
filter_complementary_destroy(filter_instance_t* inst);
bool
filter_complementary_queue(filter_instance_t* inst,
                           tracker_measurement_t* measurement);

bool
filter_complementary_get_state(filter_instance_t* inst, filter_state_t* state);
bool
filter_complementary_set_state(filter_instance_t* inst, filter_state_t* state);
bool
filter_complementary_predict_state(filter_instance_t* inst,
                                   filter_state_t*,
                                   timepoint_ns time);
bool
filter_complementary_configure(filter_instance_t* inst,
                               filter_configuration_ptr config);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // FILTER_COMPLEMENTARY_H
