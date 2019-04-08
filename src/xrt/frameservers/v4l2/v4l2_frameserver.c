#include <common/frameserver.h>
#include "v4l2_frameserver.h"
#include <string.h>
#include <stdlib.h>

bool v4l2_source_alloc(v4l2_source_descriptor_t* desc)
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

bool v4l2_frameserver_alloc(v4l2_frameserver_instance_t* inst) {
	return true;
}
bool v4l2_frameserver_destroy(v4l2_frameserver_instance_t* inst) {
    return true;
}


bool v4l2_frameserver_enumerate_sources(v4l2_frameserver_instance_t* inst, v4l2_source_descriptor_t* cameras, uint32_t* count)
{

	return true;
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
