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

typedef struct ffmpeg_source_descriptor
{
	char name[128];
	char* filepath;
	uint64_t source_id;
	uint32_t current_frame;
	uint32_t frame_count;
	frame_format_t format;
	uint32_t width;
	uint32_t height;
} ffmpeg_source_descriptor_t;

typedef struct ffmpeg_frameserver_instance
{
	int64_t videoCodecTimebase;
	int32_t av_video_streamid;
	AVFormatContext* av_format_context;
	AVCodecContext* av_codec_context;
	AVCodec* av_video_codec;
	AVFrame* av_current_frame;
	frame_consumer_callback_func frame_target_callback;
	event_consumer_callback_func event_target_callback;
	void* frame_target_instance; // where we send our frames
	void* event_target_instance; // where we send our events
	pthread_t stream_thread;
	bool is_running;
	ffmpeg_source_descriptor_t source_descriptor;
	uint32_t sequence_counter;


} ffmpeg_frameserver_instance_t;



ffmpeg_frameserver_instance_t*
ffmpeg_frameserver_create(frameserver_instance_t* inst);
bool
ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst);

bool
ffmpeg_source_create(ffmpeg_source_descriptor_t* desc);
bool
ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc);

bool
ffmpeg_frameserver_configure_capture(frameserver_instance_t* inst,
                                     capture_parameters_t cp);
bool
ffmpeg_frameserver_enumerate_sources(
    frameserver_instance_t* inst,
    frameserver_source_descriptor_ptr sources_generic,
    uint32_t* count);
bool
ffmpeg_frameserver_get(frameserver_instance_t* inst, frame_t* _frame);
void
ffmpeg_frameserver_register_frame_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    frame_consumer_callback_func target_func);
void
ffmpeg_frameserver_register_event_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func);
bool
ffmpeg_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool
ffmpeg_frameserver_stream_start(
    frameserver_instance_t* inst,
    frameserver_source_descriptor_ptr source_generic);
bool
ffmpeg_frameserver_stream_stop(frameserver_instance_t* inst);
bool
ffmpeg_frameserver_is_running(frameserver_instance_t* inst);
bool
ffmpeg_frameserver_test();


#endif // UVC_FRAMESERVER_H
