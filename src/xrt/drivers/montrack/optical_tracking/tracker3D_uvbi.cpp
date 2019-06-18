#include "tracker3D_uvbi.h"
#include "unifiedvideoinertial/TrackingSystem.h"
#include "unifiedvideoinertial/TrackingDebugDisplay.h"
#include "opencv2/opencv.hpp"
#include "util/u_misc.h"

typedef struct tracker3D_uvbi_instance
{
	bool configured;
	measurement_consumer_callback_func measurement_target_callback;
	void* measurement_target_instance; // where we send our measurements
	event_consumer_callback_func event_target_callback;
	void* event_target_instance; // where we send our measurements
	tracker_mono_configuration_t configuration;
	videotracker::uvbi::TrackingSystem* system;
	videotracker::uvbi::TrackedBody* hmd;
	videotracker::uvbi::TrackingDebugDisplay* debug;
	cv::Mat frame_gray;
	cv::Mat debug_rgb;
	bool alloced_frames;
	videotracker::CameraParameters camera_params;
	videotracker::util::TimeValue current_time;
} tracker3D_uvbi_instance_t;



capture_parameters_t
tracker3D_uvbi_get_capture_params(tracker_instance_t* inst)
{
	capture_parameters_t cp = {};
	cp.exposure = 0.5f;
	cp.gain = 0.5f;
	return cp;
}

tracker3D_uvbi_instance_t*
tracker3D_uvbi_create(tracker_instance_t* inst)
{

	tracker3D_uvbi_instance_t* i =
	    U_TYPED_CALLOC(tracker3D_uvbi_instance_t);
	if (i) {
		videotracker::uvbi::ConfigParams cp;
		i->system = new videotracker::uvbi::TrackingSystem(cp);
		i->debug = new videotracker::uvbi::TrackingDebugDisplay(cp);
		i->debug_rgb = cv::Mat(480, 1280, CV_8UC3, cv::Scalar(0, 0, 0));
		i->camera_params =
		    videotracker::getSimulatedHDKCameraParameters();
		i->hmd = i->system->createTrackedBody();
		i->alloced_frames = false;
		return i;
	}
	return NULL;
}


bool
tracker3D_uvbi_get_debug_frame(tracker_instance_t* inst, frame_t* frame)
{
	tracker3D_uvbi_instance_t* internal =
	    (tracker3D_uvbi_instance_t*)inst->internal_instance;
	cv::Mat rgbFrame;
	cv::cvtColor(internal->frame_gray, rgbFrame, CV_GRAY2BGR);
	cv::Mat uvbi_debug = internal->debug->createStatusImage(
	    *(internal->system), internal->camera_params, rgbFrame);

	uvbi_debug.copyTo(internal->debug_rgb);


	frame->format = FORMAT_RGB_UINT8;
	frame->width = internal->debug_rgb.cols;
	frame->stride =
	    internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
	frame->height = internal->debug_rgb.rows;
	frame->data = internal->debug_rgb.data;
	frame->size_bytes = frame_size_in_bytes(frame);

	return true;
}
bool
tracker3D_uvbi_queue(tracker_instance_t* inst, frame_t* frame)
{
	tracker3D_uvbi_instance_t* internal =
	    (tracker3D_uvbi_instance_t*)inst->internal_instance;
	printf("received frame, tracking!\n");
	if (!internal->alloced_frames) {
		internal->frame_gray = cv::Mat(frame->height, frame->stride,
		                               CV_8UC1, cv::Scalar(0, 0, 0));
		internal->debug_rgb = cv::Mat(frame->height, frame->width,
		                              CV_8UC3, cv::Scalar(0, 0, 0));
		internal->alloced_frames = true;
	}
	// we will just 'do the work' here.
	// TODO: asynchronous tracker thread

	memcpy(internal->frame_gray.data, frame->data, frame->size_bytes);
	internal->system->processFrame(
	    internal->current_time, internal->frame_gray, internal->frame_gray,
	    internal->camera_params);
	tracker_send_debug_frame(inst); // publish our debug frame
	return true;
}
bool
tracker3D_uvbi_get_poses(tracker_instance_t* inst,
                         tracked_object_t* objects,
                         uint32_t* count)
{
	return false;
}
bool
tracker3D_uvbi_new_poses(tracker_instance_t* inst)
{
	return false;
}
bool
tracker3D_uvbi_configure(tracker_instance_t* inst,
                         tracker_mono_configuration_t* config)
{
	tracker3D_uvbi_instance_t* internal =
	    (tracker3D_uvbi_instance_t*)inst->internal_instance;
	// return false if we cannot handle this config

	if (config->format != FORMAT_Y_UINT8) {
		internal->configured = false;
		return false;
	}
	internal->configuration = *config;
	internal->configured = true;
	return true;
}
void
tracker3D_uvbi_register_measurement_callback(
    tracker_instance_t* inst,
    void* target_instance,
    measurement_consumer_callback_func target_func)
{}
void
tracker3D_uvbi_register_event_callback(tracker_instance_t* inst,
                                       void* target_instance,
                                       event_consumer_callback_func target_func)
{}
