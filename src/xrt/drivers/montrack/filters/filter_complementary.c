
#include "filter_complementary.h"
#include "util/u_misc.h"


struct filter_complementary_instance_t
{
	bool configured;
	filter_complementary_configuration_t configuration;
	filter_state_t last_state;
	filter_state_t state;
    float gyro_x_bias,gyro_y_bias,gyro_z_bias;
    uint16_t avg_count;
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
    if (!internal->running) {
		return false;
	}
	tracker_measurement_t* measurement_array;
    uint32_t count = measurement_queue_get_since_timestamp(inst->measurement_queue,0,internal->last_state.timestamp,&measurement_array);
	float one_minus_bias = 1.0f - internal->configuration.bias;
    int avg_max =8;
    for (uint32_t i=0;i<count;i++) {
		tracker_measurement_t* m = &measurement_array[i];

        if (m->flags & ( MEASUREMENT_OPTICAL | MEASUREMENT_POSITION) ) {
            //we can just stuff our position into our state for now, ignoring  timestamps.
            internal->state.pose.position = m->pose.position;
            internal->state.pose.orientation = m->pose.orientation;

            internal->state.timestamp = m->source_timestamp;
            internal->last_state = internal->state;
        }
        if (m->flags & ( MEASUREMENT_IMU | MEASUREMENT_RAW_ACCEL | MEASUREMENT_RAW_GYRO) ) {
            float dt = (m->source_timestamp - internal->last_state.timestamp) * 0.000001;
            //printf("dt %f\n",dt);
            if (dt > 1.0f) {
                internal->last_state.timestamp = m->source_timestamp;
                return false; //this is our first frame, or something has gone wrong... big dt will blow up calculations.
            }

        float raw_mag_accel =  sqrt(m->accel.x * m->accel.x + m->accel.y * m->accel.y + m->accel.z * m->accel.z);
        float mag_accel = raw_mag_accel * internal->configuration.accel_to_g;

        //determine roll/pitch angles with gravity as a reference
        float roll = atan2(-1.0f * m->accel.x,sqrt(m->accel.y * m->accel.y + m->accel.z * m->accel.z));
        float pitch = atan2(m->accel.y, m->accel.z);

        //assume that, if acceleration is only gravity, then any change in the gyro is drift and update compensation
        //if acceleration magnitude is close to 1.0f, we assume its just gravity, and can integrate our gyro reading
        //as drift compensation - we can  assume controller is static at startup, and gather data then, or continuously
        //sample


        if (fabs(1.0f - mag_accel) < 0.05 ) {             //looks like gravity only
            //fill up the running average count as fast as possible, but subsequently
            //only integrate measurements that are not outliers w/respect to the average
            if (internal->avg_count < avg_max || fabs(m->gyro.z - internal->gyro_z_bias) < (internal->gyro_z_bias / 4)) {
                if (internal->avg_count < avg_max) {
                    internal->avg_count++;
                }
                internal->gyro_x_bias = internal->gyro_x_bias + (m->gyro.x - internal->gyro_x_bias) / math_min(internal->avg_count, avg_max);
                internal->gyro_y_bias = internal->gyro_y_bias + (m->gyro.y - internal->gyro_y_bias) / math_min(internal->avg_count, avg_max);
                internal->gyro_z_bias = internal->gyro_z_bias + (m->gyro.z - internal->gyro_z_bias) / math_min(internal->avg_count, avg_max);
                //printf("yaw correction: %f %f %f\n",internal->gyro_x_bias,internal->gyro_y_bias,internal->gyro_z_bias);
            }
        }
        //printf("roll %f pitch %f yaw 0.0f,gbc,%f,%f,%f,axyz,%f,%f,%f,gxyz,%f,%f,%f,dt,%f\n",roll,pitch,internal->gyro_x_bias,internal->gyro_y_bias,internal->gyro_z_bias,m->accel.x,m->accel.y,m->accel.z,m->gyro.x,m->gyro.y,m->gyro.z,dt);;
        internal->state.rotation_euler.z = internal->last_state.rotation_euler.z + (m->gyro.z - internal->gyro_z_bias) * internal->configuration.gyro_to_radians_per_second * dt;
        //push back towards zero, as 'returning to 0 slowly' is better than 'just drift', probably
        internal->state.rotation_euler.z -=internal->state.rotation_euler.z * internal->configuration.drift_z_to_zero * dt;
        internal->state.rotation_euler.y = internal->last_state.rotation_euler.y + (one_minus_bias * ((m->gyro.y - internal->gyro_y_bias) * internal->configuration.gyro_to_radians_per_second * dt)) - internal->configuration.bias * (internal->last_state.rotation_euler.y - roll);
        internal->state.rotation_euler.x = internal->last_state.rotation_euler.x + (one_minus_bias * ((m->gyro.x - internal->gyro_x_bias) * internal->configuration.gyro_to_radians_per_second * dt)) - internal->configuration.bias * (internal->last_state.rotation_euler.x - pitch);

        //DISABLED FOR POS TRQCKER TESTING
        //internal->state.timestamp = m->source_timestamp;
        //internal->last_state = internal->state;
        //printf("source tstamp: %lld\n",m->source_timestamp);
        }
    }
    //TODO: come up with a way to avoid alloc/free - use max length buffer?
    free(measurement_array);
    //convert to a quat for consumption as pose
    //printf(" integrated %d measurements after %lld X %f Y %f Z %f\n",count,internal->last_state.timestamp,internal->state.rotation_euler.x,internal->state.rotation_euler.y,internal->state.rotation_euler.z);

    //removed for debugging pos tracking
    //math_euler_to_quat(internal->state.rotation_euler,&internal->state.pose.orientation);

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
    internal->gyro_x_bias=0.0f;
    internal->gyro_y_bias=0.0f;
    internal->gyro_z_bias=0.0f;
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
        i->last_state.rotation_euler.x=0.0f;
        i->last_state.rotation_euler.y=0.0f;
        i->last_state.rotation_euler.z=0.0f;

		i->last_state = i->state;
		return i;
	}
	return NULL;
}
