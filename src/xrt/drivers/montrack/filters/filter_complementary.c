
#include "filter_complementary.h"
#include "util/u_misc.h"

struct filter_complementary_instance_t
{
	bool configured;
	filter_complementary_configuration_t configuration;
	filter_state_t last_state;
	filter_state_t state;
	float alpha;
	bool running;
};

/*!
 * Casts the internal instance pointer from the generic opaque type to our
 * opencv_kalman internal type.
 */
static inline filter_complementary_instance_t*
filter_complementary_instance(filter_internal_instance_ptr ptr)
{
	return (filter_complementary_instance_t*)ptr;
}

bool
filter_complementary__destroy(filter_instance_t* inst)
{
	// do nothing
	return false;
}

bool
filter_complementary_queue(filter_instance_t* inst,
                           tracker_measurement_t* measurement)
{
	filter_complementary_instance_t* internal =
	    filter_complementary_instance(inst->internal_instance);
    measurement_queue_add(inst->measurement_queue,measurement);
	internal->running = true;
	return true;
}
bool
filter_complementary_get_state(filter_instance_t* inst, filter_state_t* state)
{
	filter_complementary_instance_t* internal =
	    filter_complementary_instance(inst->internal_instance);
	// printf("getting filtered pose\n");
	if (!internal->running) {
		return false;
	}
	tracker_measurement_t* measurement_array;
    uint64_t last_timestamp = internal->last_state.timestamp;
    if (internal->configuration.max_timestamp > 0)
    {
        if (last_timestamp >= internal->configuration.max_timestamp){
            //wrap timestamp
            last_timestamp = internal->configuration.max_timestamp - last_timestamp;
        }
    }
    uint32_t count = measurement_queue_get_since_timestamp(inst->measurement_queue,0,last_timestamp,&measurement_array);
	float one_minus_bias = 1.0f - internal->configuration.bias;
	for (uint32_t i=0;i<count;i++)
	{
		tracker_measurement_t* m = &measurement_array[i];
        float dt = (m->source_timestamp - internal->last_state.timestamp) * 0.0000001;
        float normAccel = sqrt(m->accel.x * m->accel.x + m->accel.y * m->accel.y + m->accel.z+m->accel.z);

		//calculate filtered euler angles
		internal->state.rotation_euler.z = internal->last_state.rotation_euler.z + m->gyro.z * dt;
		internal->state.rotation_euler.y = internal->configuration.bias * (internal->last_state.rotation_euler.y + m->gyro.y * dt) + one_minus_bias * (m->accel.y * internal->configuration.scale/normAccel);
		internal->state.rotation_euler.x = internal->configuration.bias * (internal->last_state.rotation_euler.x + m->gyro.x * dt) + one_minus_bias * (m->accel.x * internal->configuration.scale/normAccel);
        internal->state.timestamp = m->source_timestamp;
        internal->last_state = internal->state;
        printf("source tstamp: %lld\n",m->source_timestamp);
	    }
	//TODO: come up with a way to avoid alloc/free - use max length buffer?
	free(measurement_array);
	//convert to a quat for consumption as pose
    printf(" integrated %d measurements after %lld X %f Y %f Z %f\n",count,last_timestamp,internal->state.rotation_euler.x,internal->state.rotation_euler.y,internal->state.rotation_euler.z);
	math_euler_to_quat(internal->state.rotation_euler,&internal->state.pose.orientation);
	*state = internal->state;
	return true;
}
bool
filter_complementary_set_state(filter_instance_t* inst, filter_state_t* state)
{
	//TODO: implement this
	return false;
}
bool
filter_complementary_predict_state(filter_instance_t* inst,
                                   filter_state_t* state,
                                   timepoint_ns time)
{

	//dont do any prediction right now
	return filter_complementary_get_state(inst,state);
}



bool
filter_complementary_configure(filter_instance_t* inst,
                               filter_configuration_ptr config_generic)
{
	filter_complementary_instance_t* internal =
        filter_complementary_instance(inst->internal_instance);
	filter_complementary_configuration_t* config =
	    (filter_complementary_configuration_t*)config_generic;
	internal->configuration = *config;
	internal->configured = true;
	return true;
}



filter_complementary_instance_t*
filter_complementary_create(filter_instance_t* inst)
{
	filter_complementary_instance_t* i =
	    U_TYPED_CALLOC(filter_complementary_instance_t);
	if (i) {
		i->configured = false;
		i->running = false;
		// set up our state stuff that will not change.
		// we a are a rotational-only filter
		i->state.has_rotation = true;
		i->state.has_position = false;
		i->state.timestamp = 0;
		i->last_state = i->state;
		return i;
	}
	return NULL;
}
