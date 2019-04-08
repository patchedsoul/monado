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

typedef enum event_type {EVENT_NONE,EVENT_FRAME} event_type_t;

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


bool ffmpeg_frameserver_alloc(ffmpeg_frameserver_instance_t* inst);
bool ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst);
bool ffmpeg_source_alloc(ffmpeg_source_descriptor* desc);
bool ffmpeg_source_destroy(ffmpeg_source_descriptor* desc);
bool ffmpeg_frameserver_enumerate_sources(ffmpeg_frameserver_instance_t*, ffmpeg_source_descriptor_t* sources, uint32_t* count);
bool ffmpeg_frame_get(frame_t* _frame);
void ffmpeg_register_event_callback(void* func,event_type_t event_type);
bool ffmpeg_seek(uint64_t timestamp);
bool ffmpeg_stream_start(ffmpeg_source_descriptor_t* source);
bool ffmpeg_stream_stop();
bool ffmpeg_is_running();
bool ffmpeg_frameserver_test();


#endif //UVC_FRAMESERVER_H
