#ifndef TRACKER3D_SPHERE_MONO_H
#define TRACKER3D_SPHERE_MONO_H

#include <xrt/xrt_defines.h>
#include "common/calibration.h"
#include "common/tracked_object.h"
#include "common/tracker.h"


typedef struct tracker3D_sphere_mono_instance {
    camera_calibration_t calibration;
    frameserver_instance_t frameserver_instance;
    tracked_object_t tracked_object;
} tracker3D_sphere_mono_instance_t;

tracker3D_sphere_mono_instance_t* tracker3D_sphere_mono_create(tracker_instance_t* inst);
bool tracker3D_sphere_mono_destroy(tracker_instance_t* inst);
bool tracker3D_sphere_mono_track(tracker_instance_t* inst,frame_t* frame);
bool tracker3D_sphere_mono_get_poses(tracker_instance_t* inst,tracked_object_t* objects,uint32_t* count);



#endif //TRACKER3D_SPHERE_MONO_H
