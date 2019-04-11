#include "tracker3D_sphere_mono.h"

tracker3D_sphere_mono_instance_t* tracker3D_sphere_mono_create(tracker_instance_t* inst) {
	tracker3D_sphere_mono_instance_t* i = calloc(1,sizeof(tracker3D_sphere_mono_instance_t));
	if (i) {
		i->poses_consumed=false;
		i->configured=false;
		return i;
	}
	return NULL;
}

bool tracker3D_sphere_mono_queue(tracker_instance_t* inst,frame_t* frame) {
    tracker3D_sphere_mono_instance_t* instance = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	tracker3D_sphere_mono_instance_t*  internal = inst->internal_instance;
	if (! internal->configured){
		printf("ERROR: you must configure this tracker before it can accept frames\n");
		return false;
	}
	printf("received frame, tracking!\n");

	//we will just 'do the work' here, normally this frame
	//would be added to a queue and a tracker thread
	//would analyse it asynchronously.

	//we will just randomise the pose to show 'something'
	//is being done

	internal->tracked_object.pose.position.x = rand() % 1000 / 10.0;
	internal->tracked_object.pose.position.y = rand() % 1000 / 10.0;
	internal->tracked_object.pose.position.z = rand() % 1000 / 10.0;

	internal->poses_consumed=false;
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
	tracker3D_sphere_mono_instance_t*  internal = inst->internal_instance;
	internal->poses_consumed=true;
	return true;
}

bool tracker3D_sphere_mono_new_poses(tracker_instance_t* inst)
{
	tracker3D_sphere_mono_instance_t*  internal = inst->internal_instance;
	return internal->poses_consumed;
}

bool tracker3D_sphere_mono_configure(tracker_instance_t* inst,tracker_mono_configuration_t* config)
{
	tracker3D_sphere_mono_instance_t*  internal = inst->internal_instance;
	//return false if we cannot handle this config

	if (config->format != FORMAT_Y_UINT8) {
		internal->configured=false;
		return false;
	}

	internal->configured=true;
	return true;
}

