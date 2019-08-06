#pragma once
#ifndef TRACKER3D_SPHERE_PSVR_H
#define TRACKER3D_SPHERE_PSVR_H

#include <xrt/xrt_defines.h>
#include <common/calibration.h>
#include "common/tracked_object.h"
#include "common/tracker.h"

#define NUM_LEDS 5
#define TRACKED_POINTS 1
#define ROI_OFFSET 32.0f

static const char* LED_LABELS[] = {"LU", "RU", "C",  "LL", "RL",
                                   "LS", "RS", "LB", "RB"};

//simplified model, normalised to longest dist between pair of leds
static const struct xrt_vec3 physical_led_positions[] = {
    {-0.413f,-0.28f,-0.18f},
    {0.413f,0.-0.28f,-0.18f},
    {0.0f,0.0f,0.0f},
    {-0.413f,0.28f,-0.18f},
{0.413f,0.28f,-0.18f},};
/*    {0.0f,0.0f,0.0f},
    {0.0f,0.0f,0.0f},
    {0.0f,0.0f,0.0f},
    {0.0f,0.0f,0.0f},

};*/

#ifdef __cplusplus
extern "C" {
#endif





// forward declare this
typedef struct tracker3D_psvr_stereo_instance
    tracker3D_psvr_stereo_instance_t;


tracker3D_psvr_stereo_instance_t*
tracker3D_psvr_stereo_create(tracker_instance_t* inst);
bool
tracker3D_psvr_stereo_destroy(tracker_instance_t* inst);

capture_parameters_t
tracker3D_psvr_stereo_get_capture_params(tracker_instance_t* inst);

bool
tracker3D_psvr_stereo_get_debug_frame(tracker_instance_t* inst,
                                        frame_t* frame);
bool
tracker3D_psvr_stereo_queue(tracker_instance_t* inst, frame_t* frame);
bool
tracker3D_psvr_stereo_get_poses(tracker_instance_t* inst,
                                  tracked_object_t* objects,
                                  uint32_t* count);
bool
tracker3D_psvr_stereo_new_poses(tracker_instance_t* inst);
bool
tracker3D_psvr_stereo_configure(tracker_instance_t* inst,
                                  tracker_stereo_configuration_t* config);
void
tracker3D_psvr_stereo_register_measurement_callback(
    tracker_instance_t* inst,
    void* target_instance,
    measurement_consumer_callback_func target_func);
void
tracker3D_psvr_stereo_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func);



#ifdef __cplusplus
} // extern "C"
#endif


#endif // TRACKER3D_PSVR_STEREO_H
