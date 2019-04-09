#include <common/frameserver.h>
#include "ffmpeg_frameserver.h"
#include <string.h>
#include <stdlib.h>


bool ffmpeg_source_create(ffmpeg_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

ffmpeg_frameserver_instance_t* ffmpeg_frameserver_create(frameserver_instance_t* inst) {
    //TODO use macro here
    ffmpeg_frameserver_instance_t* i = calloc(1,sizeof(ffmpeg_frameserver_instance_t));
    if (i) {
        inst->internal_instance = i;
        return i;
    }
    return NULL;
}

bool ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst) {
    //TODO: cleanup
    free(inst);
    return true;
}
bool ffmpeg_frameserver_enumerate_sources(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* sources, uint32_t* count) {
	return true;
}

bool ffmpeg_frame_get(frameserver_instance_t* inst, frame_t* _frame) {
	return false;
}
void ffmpeg_register_event_callback(frameserver_instance_t* inst, void* func,frameserver_event_type_t event_type) {
	//do nothing
}
bool ffmpeg_seek(frameserver_instance_t* inst, uint64_t timestamp) {
	return false;
}
bool ffmpeg_stream_start(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* source) {
	return false;
}
bool ffmpeg_stream_stop(frameserver_instance_t* inst) {
	return false;
}
bool ffmpeg_is_running(frameserver_instance_t* inst) {
	return false;
}

bool ffmpeg_frameserver_test() {
    printf("Running FFMPEG Frameserver Test\n");
	frameserver_instance_t* ffm_server = frameserver_create(FRAMESERVER_TYPE_FFMPEG);
    if (! ffm_server)
    {
        printf("FAILURE: Could not init FFMPEG frameserver.\n");
        return false;
    }
    uint32_t source_count =0;
    if (! ffm_server->frameserver_enumerate_sources(ffm_server,NULL,&source_count)) {
        printf("FAILURE: Could not get source count.\n");
        return false;
    }
    ffmpeg_source_descriptor_t* source_list = calloc(source_count,sizeof(ffmpeg_source_descriptor_t));
    if (! ffm_server->frameserver_enumerate_sources(ffm_server, source_list,&source_count)) {
        printf("FAILURE: Could not get source descriptors\n");
        return false;
    }
    for (uint32_t i=0;i<source_count;i++)
    {
        printf("%d FFMPEG source name: %s\n",i,source_list[i].name);
    }
    return true;
}
