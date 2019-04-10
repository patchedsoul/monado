#ifndef FFMPEG_FRAMESERVER_H
#define FFMPEG_FRAMESERVER_H

/* Almost all of the ground covered here would be covered
 * by the v4l2 frameserver on linux, but uvc may be the
 * simplest approach for cross-platform e.g. OS X
*/

#include <stdint.h>
#include <stdio.h>

#include "../common/frameserver.h"
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libavutil/avutil.h"



static AVPacket emptyPacket;

typedef struct ffmpeg_frameserver_instance {
    int64_t videoCodecTimebase;
    AVFormatContext* formatContext;
    AVCodecContext* videoCodecContext;
    AVFrame* ffVideoCurrentFrame;
} ffmpeg_frameserver_instance_t;


typedef struct ffmpeg_source_descriptor {
    char name[128];
    char* filepath;
    uint32_t current_frame;
    uint32_t frame_count;
} ffmpeg_source_descriptor_t;


ffmpeg_frameserver_instance_t* ffmpeg_frameserver_create(frameserver_instance_t* inst);
bool ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst);
bool ffmpeg_source_create(ffmpeg_source_descriptor_t* desc);
bool ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc);

bool ffmpeg_frameserver_enumerate_sources(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* sources, uint32_t* count);
bool ffmpeg_frame_get(frameserver_instance_t* inst, frame_t* _frame);
void ffmpeg_register_event_callback(frameserver_instance_t* inst, void* target_instance,void* target_func,frameserver_event_type_t event_type);
bool ffmpeg_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool ffmpeg_stream_start(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* source);
bool ffmpeg_stream_stop(frameserver_instance_t* inst);
bool ffmpeg_is_running(frameserver_instance_t* inst);
bool ffmpeg_frameserver_test();


#endif //UVC_FRAMESERVER_H
