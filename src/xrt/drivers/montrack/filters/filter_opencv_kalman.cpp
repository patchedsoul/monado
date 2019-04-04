
#include <opencv2/opencv.hpp>

#include "../optical_tracking/common/tracker.h"
#include "filter_opencv_kalman.h"

#include "util/u_misc.h"

struct filter_opencv_kalman
{
	struct filter_instance base;
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
static inline struct filter_opencv_kalman*
filter_opencv_kalman(struct filter_instance* ptr)
{
	return (struct filter_opencv_kalman*)ptr;
}

static void
filter_opencv_kalman_destroy(struct filter_instance* inst)
{
	free(inst);
}

static bool
filter_opencv_kalman_queue(struct filter_instance* inst,
                           tracker_measurement_t* measurement)
{
	struct filter_opencv_kalman* internal = filter_opencv_kalman(inst);
	printf("queueing measurement in filter\n");
	internal->observation.at<float>(0, 0) = measurement->pose.position.x;
	internal->observation.at<float>(1, 0) = measurement->pose.position.y;
	internal->observation.at<float>(2, 0) = measurement->pose.position.z;
	internal->kalman_filter.correct(internal->observation);
	internal->running = true;
	return false;
}
bool
filter_opencv_kalman_get_state(struct filter_instance* inst,
                               struct filter_state* state)
{
	return false;
}
bool
filter_opencv_kalman_set_state(struct filter_instance* inst,
                               struct filter_state* state)
{
	return false;
}
bool
filter_opencv_kalman_predict_state(struct filter_instance* inst,
                                   struct filter_state* state,
                                   timepoint_ns time)
{
	struct filter_opencv_kalman* internal = filter_opencv_kalman(inst);
	// printf("getting filtered pose\n");
	if (!internal->running) {
		return false;
	}
	internal->prediction = internal->kalman_filter.predict();
	state->has_position = true;
	state->pose.position.x = internal->prediction.at<float>(0, 0);
	state->pose.position.y = internal->prediction.at<float>(1, 0);
	state->pose.position.z = internal->prediction.at<float>(2, 0);
	return true;
}
bool
filter_opencv_kalman_configure(struct filter_instance* inst,
                               filter_configuration_ptr config_generic)
{
	struct filter_opencv_kalman* internal = filter_opencv_kalman(inst);
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



struct filter_opencv_kalman*
filter_opencv_kalman_create()
{
	struct filter_opencv_kalman* i =
	    U_TYPED_CALLOC(struct filter_opencv_kalman);
	if (!i) {
		return NULL;
	}
	i->base.queue = filter_opencv_kalman_queue;
	i->base.set_state = filter_opencv_kalman_set_state;
	i->base.get_state = filter_opencv_kalman_get_state;
	i->base.predict_state = filter_opencv_kalman_predict_state;
	i->base.configure = filter_opencv_kalman_configure;
	i->base.destroy = filter_opencv_kalman_destroy;
	float dt = 1.0;
	i->kalman_filter.init(6, 3);
	i->observation = cv::Mat(3, 1, CV_32F);
	i->prediction = cv::Mat(6, 1, CV_32F);
	i->kalman_filter.transitionMatrix =
	    (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	     1.0);

	cv::setIdentity(i->kalman_filter.measurementMatrix,
	                cv::Scalar::all(1.0f));
	cv::setIdentity(i->kalman_filter.errorCovPost, cv::Scalar::all(0.0f));

	// our filter parameters set the process and measurement noise
	// covariances.

	cv::setIdentity(i->kalman_filter.processNoiseCov,
	                cv::Scalar::all(i->configuration.process_noise_cov));
	cv::setIdentity(
	    i->kalman_filter.measurementNoiseCov,
	    cv::Scalar::all(i->configuration.measurement_noise_cov));

	i->configured = false;
	i->running = false;
	return i;
}
