#ifndef TRACKER3D_OSVR_UVBI_H
#define TRACKER3D_OSVR_UVBI_H

#include <xrt/xrt_defines.h>
#include "common/tracked_object.h"
#include "common/tracker.h"


#ifdef __cplusplus
extern "C" {
#endif


// forward declare this
typedef struct tracker3D_osvr_uvbi_instance tracker3D_osvr_uvbi_instance_t;


tracker3D_osvr_uvbi_instance_t*
tracker3D_osvr_uvbi_create(tracker_instance_t* inst);
bool
tracker3D_osvr_uvbi_destroy(tracker_instance_t* inst);

capture_parameters_t
tracker3D_osvr_uvbi_get_capture_params(tracker_instance_t* inst);

bool
tracker3D_osvr_uvbi_get_debug_frame(tracker_instance_t* inst, frame_t* frame);
bool
tracker3D_osvr_uvbi_queue(tracker_instance_t* inst, frame_t* frame);
bool
tracker3D_osvr_uvbi_get_poses(tracker_instance_t* inst,
                              tracked_object_t* objects,
                              uint32_t* count);
bool
tracker3D_osvr_uvbi_new_poses(tracker_instance_t* inst);
bool
tracker3D_osvr_uvbi_configure(tracker_instance_t* inst,
                              tracker_mono_configuration_t* config);
void
tracker3D_osvr_uvbi_register_measurement_callback(
    tracker_instance_t* inst,
    void* target_instance,
    measurement_consumer_callback_func target_func);
void
tracker3D_osvr_uvbi_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func);



#ifdef __cplusplus
}
#endif


#endif // TRACKER3D_OSVR_UVBI_H
