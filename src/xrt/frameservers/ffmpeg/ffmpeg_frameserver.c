#include <common/frameserver.h>
#include "ffmpeg_frameserver.h"
#include <string.h>
#include <stdlib.h>


bool ffmpeg_source_alloc(ffmpeg_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool ffmpeg_frameserver_alloc(ffmpeg_frameserver_instance_t* inst) {
	return true;
}

bool ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst) {
    return true;
}
bool ffmpeg_frameserver_enumerate_sources(ffmpeg_frameserver_instance_t* inst, ffmpeg_source_descriptor_t* sources, uint32_t* count)
{
	return true;
}

bool ffmpeg_frameserver_test(){
    printf("Running FFMPEG Frameserver Test\n");
    ffmpeg_frameserver_instance_t instance;
    if (! ffmpeg_frameserver_alloc(&instance))
    {
        printf("FAILURE: Could not init frameserver.\n");
        return false;
    }
    uint32_t source_count =0;
    if (! ffmpeg_frameserver_enumerate_sources(&instance,NULL,&source_count)) {
        printf("FAILURE: Could not get source count.\n");
        return false;
    }
    ffmpeg_source_descriptor_t* source_list = calloc(source_count,sizeof(ffmpeg_source_descriptor_t));
    if (! ffmpeg_frameserver_enumerate_sources(&instance, source_list,&source_count)) {
        printf("FAILURE: Could not get source descriptors\n");
        return false;
    }
    for (uint32_t i=0;i<source_count;i++)
    {
        printf("%d source name: %s\n",i,source_list[i].name);
    }
    return true;
}
