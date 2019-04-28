#ifndef TRACKER_H
#define TRACKER_H
#include <xrt/xrt_defines.h>
#include "../frameservers/common/frameserver.h"
#include "calibration.h"
#include "tracked_object.h"

#define MAX_FRAMESERVERS 8 //maximum number of cameras/sources that can be bound to a tracker

typedef struct tracker_measurement {
	struct xrt_pose pose;
	bool has_position;
	bool has_rotation;
	timepoint_ns timestamp;
} tracker_measurement_t;

typedef void* tracker_instance_ptr;
typedef void* tracker_internal_instance_ptr;
typedef void* tracker_configuration_ptr;

typedef void (*measurement_consumer_callback_func)(void* instance, tracker_measurement_t* measurement);

typedef struct tracker_mono_configuration {
	camera_calibration_t calibration;
	frame_format_t format;
	uint64_t source_id;
    } tracker_mono_configuration_t;

typedef struct tracker_stereo_configuration {
	camera_calibration_t l_calibration;
	frame_format_t l_format;
	uint64_t l_source_id;
	camera_calibration_t r_calibration;
	frame_format_t r_format;
	uint64_t r_source_id;
} tracker_stereo_configuration_t;

typedef enum tracker_type {
    TRACKER_TYPE_NONE,
    TRACKER_TYPE_2D_BLUE_LED,
    TRACKER_TYPE_SPHERE_MONO
} tracker_type_t;

typedef struct _tracker_instance {
     tracker_type_t tracker_type;
	 capture_parameters_t (*tracker_get_capture_params)(tracker_instance_ptr inst);
	 bool (*tracker_queue)(tracker_instance_ptr inst,frame_t* frame);
	 bool (*tracker_get_debug_frame)(tracker_instance_ptr inst,frame_t* frame);
	 bool (*tracker_get_poses)(tracker_instance_ptr inst,tracked_object_t* tracked_objects,uint32_t* count);
	 bool (*tracker_has_new_poses)(tracker_instance_ptr inst);
	 void (*tracker_register_measurement_callback)(tracker_instance_ptr inst, void* target_instance, measurement_consumer_callback_func target_func);
	 bool (*tracker_configure)(tracker_instance_ptr inst, tracker_configuration_ptr config);
	 tracker_internal_instance_ptr internal_instance;
} tracker_instance_t;

tracker_instance_t* tracker_create(tracker_type_t t);
bool tracker_destroy(tracker_instance_t* inst);
bool trackers_test();

#endif //TRACKER_H
