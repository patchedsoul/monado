#include "filter_opencv_kalman.h"
#include "opencv4/opencv2/opencv.hpp"


typedef struct filter_opencv_kalman_instance {
	bool configured;
	opencv_filter_configuration_t configuration;
	cv::KalmanFilter kalman_filter;
	cv::Mat observation;
	cv::Mat prediction;
	cv::Mat state;

} filter_opencv_kalman_instance_t;


bool filter_opencv_kalman__destroy(filter_instance_t* inst) {
	//do nothing
	return false;
}

bool filter_opencv_kalman_queue(filter_instance_t* inst,filter_measurement_t* measurement) {
	filter_opencv_kalman_instance_t* internal = (filter_opencv_kalman_instance_t*)inst->internal_instance;
	internal->observation.at<float>(0,0) = measurement->sample.position.x;
	internal->observation.at<float>(1,0) = measurement->sample.position.y;
	internal->observation.at<float>(2,0) = measurement->sample.position.z;
	internal->kalman_filter.correct(internal->observation);

	return false;
}
bool filter_opencv_kalman_get_state(filter_instance_t* inst,opencv_kalman_filter_state_t* state) {
	return false;
}
bool filter_opencv_kalman_set_state(filter_instance_t* inst,opencv_kalman_filter_state_t* state) {
	return false;
}
bool filter_opencv_kalman_predict_state(filter_instance_t* inst, opencv_kalman_filter_state_t*, timepoint_ns time) {
	return false;
}
bool filter_opencv_kalman_configure(filter_instance_t* inst, opencv_filter_configuration_t* config) {
	return false;
}



filter_opencv_kalman_instance_t* filter_opencv_kalman_create(filter_instance_t* inst) {
	filter_opencv_kalman_instance_t* i = (filter_opencv_kalman_instance_t*)calloc(1,sizeof(filter_opencv_kalman_instance_t));
	if (i) {
		float dt=1.0;
		i->kalman_filter.init(6,3);
		i->observation = cv::Mat(3,1,CV_32F);
		i->kalman_filter.transitionMatrix = (cv::Mat_<float>(6, 6) <<  1.0, 0.0, 0.0, dt, 0.0, 0.0,
		                                                           0.0, 1.0, 0.0, 0.0, dt, 0.0,
		                                                           0.0, 0.0, 1.0, 0.0, 0.0, dt,
		                                                           0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		                                                           0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		                                                           0.0, 0.0, 0.0, 0.0, 0.0, 1.0 );

		cv::setIdentity(i->kalman_filter.measurementMatrix,cv::Scalar::all(1.0f));
		cv::setIdentity(i->kalman_filter.errorCovPost, cv::Scalar::all(0.0f));

		// our filter parameters set the process and measurement noise covariances.

		cv::setIdentity(i->kalman_filter.processNoiseCov, cv::Scalar::all(i->configuration.process_noise_cov));
		cv::setIdentity(i->kalman_filter.measurementNoiseCov, cv::Scalar::all(i->configuration.measurement_noise_cov));

		i->configured = false;
		return i;
	}
	return NULL;
}


