
#include <opencv2/opencv.hpp>

#include "filter_opencv_kalman.h"

#include "util/u_misc.h"

struct filter_opencv_kalman_instance_t
{
	bool configured;
	opencv_filter_configuration_t configuration;
	cv::KalmanFilter kalman_filter;
	cv::Mat observation;
	cv::Mat prediction;
	cv::Mat state;
	bool running;
};

/*!
 * Casts the internal instance pointer from the generic opaque type to our
 * opencv_kalman internal type.
 */
static inline filter_opencv_kalman_instance_t*
filter_opencv_kalman_instance(filter_internal_instance_ptr ptr)
{
	return (filter_opencv_kalman_instance_t*)ptr;
}

bool
filter_opencv_kalman__destroy(filter_instance_t* inst)
{
	// do nothing
	return false;
}

bool
filter_opencv_kalman_queue(filter_instance_t* inst,
                           tracker_measurement_t* measurement)
{
	filter_opencv_kalman_instance_t* internal =
	    filter_opencv_kalman_instance(inst->internal_instance);
	printf("queueing measurement in filter\n");
	measurement_queue_add(inst->measurement_queue,measurement);
	//internal->observation.at<float>(0, 0) = measurement->pose.position.x;
	//internal->observation.at<float>(1, 0) = measurement->pose.position.y;
	//internal->observation.at<float>(2, 0) = measurement->pose.position.z;
	//internal->kalman_filter.correct(internal->observation);
	internal->running = true;
	return false;
}
bool
filter_opencv_kalman_get_state(filter_instance_t* inst, filter_state_t* state)
{
	return false;
}
bool
filter_opencv_kalman_set_state(filter_instance_t* inst, filter_state_t* state)
{
	return false;
}
bool
filter_opencv_kalman_predict_state(filter_instance_t* inst,
                                   filter_state_t* state,
                                   timepoint_ns time)
{
	filter_opencv_kalman_instance_t* internal =
	    filter_opencv_kalman_instance(inst->internal_instance);
	// printf("getting filtered pose\n");
	if (!internal->running) {
		return false;
	}
	//get all our measurements including the last optical frame,
	//and run our filter on them to make a prediction

	tracker_measurement_t* measurement_array;

	internal->prediction = internal->kalman_filter.predict();
	state->has_position = true;
	state->pose.position.x = internal->prediction.at<float>(0, 0);
	state->pose.position.y = internal->prediction.at<float>(1, 0);
	state->pose.position.z = internal->prediction.at<float>(2, 0);
	return true;
}
bool
filter_opencv_kalman_configure(filter_instance_t* inst,
                               filter_configuration_ptr config_generic)
{
	filter_opencv_kalman_instance_t* internal =
	    filter_opencv_kalman_instance(inst->internal_instance);
	opencv_filter_configuration_t* config =
	    (opencv_filter_configuration_t*)config_generic;
	internal->configuration = *config;
	cv::setIdentity(
	    internal->kalman_filter.processNoiseCov,
	    cv::Scalar::all(internal->configuration.process_noise_cov));
	cv::setIdentity(
	    internal->kalman_filter.measurementNoiseCov,
	    cv::Scalar::all(internal->configuration.measurement_noise_cov));
	internal->configured = true;
	return true;
}



filter_opencv_kalman_instance_t*
filter_opencv_kalman_create(filter_instance_t* inst)
{
	filter_opencv_kalman_instance_t* i =
	    U_TYPED_CALLOC(filter_opencv_kalman_instance_t);
	if (i) {
		float dt = 1.0;
		i->kalman_filter.init(6, 3);
		i->observation = cv::Mat(3, 1, CV_32F);
		i->prediction = cv::Mat(6, 1, CV_32F);
		i->kalman_filter.transitionMatrix =
		    (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0,
		     1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
		     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		     0.0, 0.0, 0.0, 0.0, 1.0);

		cv::setIdentity(i->kalman_filter.measurementMatrix,
		                cv::Scalar::all(1.0f));
		cv::setIdentity(i->kalman_filter.errorCovPost,
		                cv::Scalar::all(0.0f));

		// our filter parameters set the process and measurement noise
		// covariances.

		cv::setIdentity(
		    i->kalman_filter.processNoiseCov,
		    cv::Scalar::all(i->configuration.process_noise_cov));
		cv::setIdentity(
		    i->kalman_filter.measurementNoiseCov,
		    cv::Scalar::all(i->configuration.measurement_noise_cov));

		i->configured = false;
		i->running = false;
		return i;
	}
	return NULL;
}
