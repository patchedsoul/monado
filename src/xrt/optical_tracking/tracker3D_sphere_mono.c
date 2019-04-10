#include "tracker3D_sphere_mono.h"

tracker3D_sphere_mono_instance_t* tracker3D_sphere_mono_create(tracker_instance_t* inst) {
	//TODO use macro here
	tracker3D_sphere_mono_instance_t* i = calloc(1,sizeof(tracker3D_sphere_mono_instance_t));
	if (i) {
		return i;
	}
	return NULL;
}

bool tracker3D_sphere_mono_track(tracker_instance_t* inst,frame_t* frame) {
    tracker3D_sphere_mono_instance_t* instance = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
    return true;
}

bool tracker3D_sphere_mono_get_poses(tracker_instance_t* inst,tracked_object_t* objects, uint32_t* count) {
	if (objects == NULL)
    {
        *count = 1; //tracking a single object
        return true;
    }

    tracker3D_sphere_mono_instance_t* instance = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	for (uint32_t i = 0;i< 1;i++) {

        objects[i] = instance->tracked_object;
    }
	*count=1;
    return true;
}
