#include "tracker3D_sphere_mono.h"

bool tracker3D_sphere_mono_track(tracker_instance_t* inst,frame_t* frame) {
    tracker3D_sphere_mono_instance_t* instance = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
    return true;
}

bool tracker3D_sphere_mono_get_poses(tracker_instance_t* inst,tracked_object_t* objects, uint32_t* count) {
    if (*count = 0)
    {
        *count = 1; //tracking a single object
        return true;
    }

    tracker3D_sphere_mono_instance_t* instance = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
    for (uint32_t i = 0;i< *count;i++) {

        objects[i] = instance->tracked_object;
    }

    return true;
}
