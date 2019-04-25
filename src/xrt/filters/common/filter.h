#ifndef FILTER_H
#define FILTER_H
#include <xrt/xrt_defines.h>
#include <../auxiliary/util/u_time.h>

typedef void* filter_instance_ptr;
typedef void* filter_internal_instance_ptr;
typedef void* filter_configuration_ptr;
typedef void* filter_state_ptr;

typedef struct filter_measurement {
	struct xrt_pose pose;
	bool has_position;
	bool has_rotation;
	timepoint_ns timestamp;
} filter_measurement_t;

typedef struct filter_state {
	struct xrt_pose pose;
	bool has_position;
	bool has_rotation;
	struct xrt_vec3 velocity;
	struct xrt_vec3 acceleration;
	struct xrt_quat angular_velocity;
	struct xrt_quat angular_accel;
	timepoint_ns timestamp;
} filter_state_t;


typedef enum filter_type {
	FILTER_TYPE_NONE,
	FILTER_TYPE_OPENCV_KALMAN
} filter_type_t;

typedef struct _filter_instance {
	 filter_type_t tracker_type;
	 bool (*filter_queue)(filter_instance_ptr inst,filter_measurement_t* measurement);
	 bool (*filter_set_state)(filter_instance_ptr inst,filter_state_ptr state);
	 bool (*filter_get_state)(filter_instance_ptr inst,filter_state_ptr state);
	 bool (*filter_predict_state)(filter_instance_ptr inst, filter_state_t* state, timepoint_ns time);
	 bool (*filter_configure)(filter_instance_ptr inst, filter_configuration_ptr config);
	 filter_internal_instance_ptr internal_instance;
} filter_instance_t;

filter_instance_t* filter_create(filter_type_t t);
bool filter_destroy(filter_instance_t* inst);
bool filters_test();

#endif //FILTER_H
