#include <common/frameserver.h>
#include "v4l2_frameserver.h"
#include <string.h>
#include <stdlib.h>

bool v4l2_source_create(v4l2_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool v4l2_source_destroy(v4l2_source_descriptor_t* desc) {
    if (desc->device_path) {
        free (desc->device_path);
    }
    return true;
}

v4l2_frameserver_instance_t* v4l2_frameserver_create(frameserver_instance_t* inst) {
    v4l2_frameserver_instance_t* i = calloc(sizeof(v4l2_frameserver_instance_t),1);
    if (i) {
		//inst->internal_instance = i;
        return i;
    }
    return false;
}
bool v4l2_frameserver_destroy(frameserver_instance_t* inst) {
    free(inst->internal_instance);
    return true;
}


bool v4l2_frameserver_enumerate_sources(frameserver_instance_t* inst, v4l2_source_descriptor_t* cameras, uint32_t* count)
{

	return true;
}
void v4l2_register_event_callback(frameserver_instance_t* inst, void* target_instance,void* target_func,frameserver_event_type_t event_type)
{
	//do nothing
}

bool v4l2_frame_get(frameserver_instance_t* inst, frame_t* frame) {
	return false;
}

bool v4l2_seek(frameserver_instance_t* inst, uint64_t timestamp) {
//do nothing
return false;
}

bool v4l2_stream_start(frameserver_instance_t* inst){
	return false;
}
bool v4l2_stream_stop(frameserver_instance_t* inst){
	return false;
}


bool v4l2_is_running(frameserver_instance_t* inst) {
//do nothing
return false;
}



bool v4l2_frameserver_test(){
	printf("Running V4L2 Frameserver Test\n");
    v4l2_frameserver_instance_t instance;
	if (! v4l2_frameserver_init(&instance))
	{
		printf("FAILURE: Could not init frameserver.\n");
		return false;
	}
	uint32_t camera_count =0;
	if (! v4l2_frameserver_enumerate_devices(&instance,NULL,&camera_count)) {
		printf("FAILURE: Could not get camera count.\n");
		return false;
	}
    v4l2_source_descriptor_t* camera_list = calloc(camera_count,sizeof(v4l2_source_descriptor_t));
	if (! v4l2_frameserver_enumerate_devices(&instance, camera_list,&camera_count)) {
		printf("FAILURE: Could not get camera descriptors\n");
		return false;
	}
	for (uint32_t i=0;i<camera_count;i++)
	{
		printf("%d camera name: %s\n",i,camera_list[i].name);
	}
	return true;
}
