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



static AVPacket empty_packet;

typedef struct ffmpeg_source_descriptor {
	char name[128];
	char* filepath;
	uint64_t source_id;
	uint32_t current_frame;
	uint32_t frame_count;
	frame_format_t format;
	uint32_t width;
	uint32_t height;
} ffmpeg_source_descriptor_t;

typedef struct ffmpeg_frameserver_instance {
    int64_t videoCodecTimebase;
	int32_t av_video_streamid;
	AVFormatContext* av_format_context;
	AVCodecContext* av_codec_context;
	AVCodec* av_video_codec;
	AVFrame* av_current_frame;
	frame_consumer_callback_t target_frame_callback;
	void* target_instance;
	pthread_t stream_thread;
	bool is_running;
	ffmpeg_source_descriptor_t source_descriptor;

} ffmpeg_frameserver_instance_t;





ffmpeg_frameserver_instance_t* ffmpeg_frameserver_create(frameserver_instance_t* inst);
bool ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst);
bool ffmpeg_source_create(ffmpeg_source_descriptor_t* desc);
bool ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc);

bool ffmpeg_frameserver_enumerate_sources(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* sources, uint32_t* count);
bool ffmpeg_frame_get(frameserver_instance_t* inst, frame_t* _frame);
void ffmpeg_register_event_callback(frameserver_instance_t* inst, void* target_instance,frame_consumer_callback_t target_func,frameserver_event_type_t event_type);
bool ffmpeg_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool ffmpeg_stream_start(frameserver_instance_t* inst, ffmpeg_source_descriptor_t* source);
bool ffmpeg_stream_stop(frameserver_instance_t* inst);
bool ffmpeg_is_running(frameserver_instance_t* inst);
bool ffmpeg_frameserver_test();

static void ffmpeg_stream_run(frameserver_instance_t* inst);


#endif //UVC_FRAMESERVER_H
