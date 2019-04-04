#ifndef FILTER_H
#define FILTER_H

#include <xrt/xrt_defines.h>
#include "util/u_time.h"

typedef void* filter_configuration_ptr;

struct filter_instance;

struct filter_state
{
	struct xrt_pose pose;
	bool has_position;
	bool has_rotation;
	struct xrt_vec3 velocity;
	struct xrt_vec3 acceleration;
	struct xrt_quat angular_velocity;
	struct xrt_quat angular_accel;
	timepoint_ns timestamp;
};


typedef enum filter_type
{
	FILTER_TYPE_NONE,
	FILTER_TYPE_OPENCV_KALMAN
} filter_type_t;

typedef struct tracker_measurement tracker_measurement_t;

struct filter_instance
{
	filter_type_t tracker_type;
	bool (*queue)(struct filter_instance* inst,
	              tracker_measurement_t* measurement);
	bool (*set_state)(struct filter_instance* inst,
	                  struct filter_state* state);
	bool (*get_state)(struct filter_instance* inst,
	                  struct filter_state* state);
	bool (*predict_state)(struct filter_instance* inst,
	                      struct filter_state* state,
	                      timepoint_ns time);
	bool (*configure)(struct filter_instance* inst,
	                  filter_configuration_ptr config);
	void (*destroy)(struct filter_instance* inst);
};

struct filter_instance*
filter_create(filter_type_t t);

bool
filters_test();

static inline void
filter_destroy(struct filter_instance* inst)
{
	inst->destroy(inst);
}

static inline bool
filter_queue(struct filter_instance* inst, tracker_measurement_t* measurement)
{
	return inst->queue(inst, measurement);
}
static inline bool
filter_set_state(struct filter_instance* inst, struct filter_state* state)
{
	return inst->set_state(inst, state);
}
static inline bool
filter_get_state(struct filter_instance* inst, struct filter_state* state)
{
	return inst->get_state(inst, state);
}
static inline bool
filter_predict_state(struct filter_instance* inst,
                     struct filter_state* state,
                     timepoint_ns time)
{
	return inst->predict_state(inst, state, time);
}
static inline bool
filter_configure(struct filter_instance* inst, filter_configuration_ptr config)
{
	return inst->configure(inst, config);
}
#endif // FILTER_H
