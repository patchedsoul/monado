#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <xrt/xrt_defines.h>
#include "tracked_object.h"
#include "tracker.h"

#define INTRINSICS_SIZE 9
#define DISTORTION_SIZE 5
#define DISTORTION_FISHEYE_SIZE 4

typedef struct room_setup {
    struct xrt_matrix_4x4 origin_transform;
    struct xrt_vec3 metric_scale;
} room_setup_t;
\
#ifdef __cplusplus
extern "C" {
#endif




// forward declare these
typedef struct tracker3D_calibration_stereo_instance tracker3D_calibration_stereo_instance_t;
typedef struct tracker3D_calibration_mono_instance tracker3D_calibration_mono_instance_t;

//shared (no-op) functions for both mono and stereo calibration
void tracker3D_calibration_register_measurement_callback(tracker_instance_t* inst, void* target_instance, measurement_consumer_callback_func target_func);
bool tracker3D_calibration_get_poses(tracker_instance_t* inst, tracked_object_t* objects, uint32_t* count);
bool tracker3D_calibration_new_poses(tracker_instance_t* inst);
void tracker3D_calibration_register_measurement_callback(tracker_instance_t* inst, void* target_instance, measurement_consumer_callback_func target_func);

tracker3D_calibration_stereo_instance_t* tracker3D_calibration_stereo_create(tracker_instance_t* inst);
bool tracker3D_calibration_stereo_destroy(tracker_instance_t* inst);
capture_parameters_t tracker3D_calibration_stereo_get_capture_params(tracker_instance_t* inst);
bool tracker3D_calibration_stereo_get_debug_frame(tracker_instance_t* inst, frame_t* frame);
bool tracker3D_calibration_stereo_queue(tracker_instance_t* inst, frame_t* frame);
bool tracker3D_calibration_stereo_configure(tracker_instance_t* inst, tracker_stereo_configuration_t* config);
void tracker3D_calibration_stereo_register_event_callback(tracker_instance_t* inst,void* target_instance,event_consumer_callback_func target_func);

tracker3D_calibration_mono_instance_t* tracker3D_calibration_mono_create(tracker_instance_t* inst);
bool tracker3D_calibration_mono_destroy(tracker_instance_t* inst);
capture_parameters_t tracker3D_calibration_mono_get_capture_params(tracker_instance_t* inst);
bool tracker3D_calibration_mono_get_debug_frame(tracker_instance_t* inst, frame_t* frame);
bool tracker3D_calibration_mono_queue(tracker_instance_t* inst, frame_t* frame);
bool tracker3D_calibration_mono_configure(tracker_instance_t* inst, tracker_mono_configuration_t* config);
void tracker3D_calibration_mono_register_event_callback( tracker_instance_t* inst, void* target_instance, event_consumer_callback_func target_func);

bool calibration_get_roomsetup(char* configuration_filename,room_setup_t* rs);


#ifdef __cplusplus
} // extern "C"
#endif

#endif // CALIBRATION_H
