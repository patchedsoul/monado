#include "../common/frameserver.h"
#include "ffmpeg_frameserver.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

#include <util/u_misc.h>

#define DUMMY_FILE "/home/pblack/tracker_test.avi"

bool
ffmpeg_source_create(ffmpeg_source_descriptor_t* desc)
{
	// do nothing right now
	return true;
}

bool
ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc)
{
	// do nothing right now
	return true;
}

ffmpeg_frameserver_instance_t*
ffmpeg_frameserver_create(frameserver_instance_t* inst)
{
	// TODO use macro here
	ffmpeg_frameserver_instance_t* i =
	    U_TYPED_CALLOC(ffmpeg_frameserver_instance_t);
	if (i) {
		i->is_running = false;
		return i;
	}
	return NULL;
}

bool
ffmpeg_frameserver_configure_capture(frameserver_instance_t* inst,
                                     capture_parameters_t cp)
{
	printf("ffmpeg is file-only, no capture params supported\n");
	return true;
}


bool
ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst)
{
	// TODO: cleanup
	free(inst);
	return true;
}
bool
ffmpeg_frameserver_enumerate_sources(frameserver_instance_t* inst,
                                     ffmpeg_source_descriptor_t* sources,
                                     uint32_t* count)
{
	// TODO: this is hardcoded, we need to query the source for its
	// properties
	if (sources == NULL) {
		*count = 2; // we advertise support for YUV420 or just a Y frame
		return true;
	}
	char* filepath = DUMMY_FILE;
	char* source_name = "FFMPEG Test Source";
	uint32_t source_id = 666;

	// this is all hardcoded but in a more developed implementation we would
	// extend and tidy this up.
	sources[0].current_frame = 0;
	sources[0].filepath =
	    calloc(1, strlen(filepath) + 1); // TODO: free this up somehow
	memcpy(sources[0].filepath, filepath, strlen(filepath) + 1);
	sources[0].frame_count = 99;
	memcpy(sources[0].name, source_name, strlen(source_name) + 1);
	sources[0].name[127] = 0; // unecessary in this context, but why not?
	sources[0].format = FORMAT_YUV420_UINT8;

	sources[1].current_frame = 0;
	sources[1].filepath = calloc(1, strlen(filepath) + 1);
	memcpy(sources[1].filepath, filepath, strlen(filepath) + 1);
	sources[1].frame_count = 99;
	memcpy(sources[1].name, source_name, strlen(source_name) + 1);
	sources[1].name[127] = 0; // unecessary in this context, but why not?
	sources[1].format = FORMAT_Y_UINT8;
	return true;
}

bool
ffmpeg_frameserver_get(frameserver_instance_t* inst, frame_t* _frame)
{
	return false;
}
void
ffmpeg_frameserver_register_frame_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    frame_consumer_callback_func target_func)
{
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	internal->frame_target_instance = target_instance;
	internal->frame_target_callback = target_func;
}

void
ffmpeg_frameserver_register_event_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	internal->event_target_instance = target_instance;
	internal->event_target_callback = target_func;
}
bool
ffmpeg_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp)
{
	return false;
}
bool
ffmpeg_frameserver_stream_start(frameserver_instance_t* inst,
                                ffmpeg_source_descriptor_t* source)
{
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	if (pthread_create(&internal->stream_thread, NULL, ffmpeg_stream_run,
	                   inst)) {
		printf("ERROR: could not createv thread\n");
		return false;
	}
	internal->source_descriptor = *source;
	internal->is_running = true;
	// we're off to the races!
	return true;
}

bool
ffmpeg_frameserver_stream_stop(frameserver_instance_t* inst)
{
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	// TODO: signal shutdown to thread
	pthread_join(&internal->stream_thread, NULL);
	return true;
}
bool
ffmpeg_frameserver_is_running(frameserver_instance_t* inst)
{
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	return internal->is_running;
}

bool
ffmpeg_frameserver_test()
{
	printf("Running FFMPEG Frameserver Test\n");
	frameserver_instance_t* ffm_server =
	    frameserver_create(FRAMESERVER_TYPE_FFMPEG);
	if (!ffm_server) {
		printf("FAILURE: Could not init FFMPEG frameserver.\n");
		return false;
	}
	uint32_t source_count = 0;
	if (!ffm_server->frameserver_enumerate_sources(ffm_server, NULL,
	                                               &source_count)) {
		printf("FAILURE: Could not get source count.\n");
		return false;
	}
	ffmpeg_source_descriptor_t* source_list =
	    calloc(source_count, sizeof(ffmpeg_source_descriptor_t));
	if (!ffm_server->frameserver_enumerate_sources(ffm_server, source_list,
	                                               &source_count)) {
		printf("FAILURE: Could not get source descriptors\n");
		return false;
	}
	for (uint32_t i = 0; i < source_count; i++) {
		printf("%d FFMPEG source name: %s\n", i, source_list[i].name);
	}
	return true;
}


void
ffmpeg_stream_run(frameserver_instance_t* inst)
{
	int ret;
	ffmpeg_frameserver_instance_t* internal = inst->internal_instance;
	internal->av_video_streamid = -1;
	av_register_all();
	internal->av_format_context = avformat_alloc_context();
	// TODO: check file exists - avformat_open_input just crashes if it does
	// not exist
	ret = avformat_open_input(&(internal->av_format_context),
	                          internal->source_descriptor.filepath, NULL,
	                          NULL);
	if (ret < 0) {
		printf("ERROR: could not open file! %s",
		       internal->source_descriptor.filepath);
		return;
	}
	ret = avformat_find_stream_info(internal->av_format_context, NULL);
	if (ret < 0) {
		printf("ERROR: could find stream info! %s",
		       internal->source_descriptor.filepath);
		return;
	}

	// find our video stream id
	for (uint32_t i = 0; i < internal->av_format_context->nb_streams; i++) {
		if (internal->av_format_context->streams[i]
		        ->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
			internal->av_video_streamid = i;
		}
	}

	if (internal->av_video_streamid == -1) {
		printf("ERROR: could find video stream in source! %s",
		       internal->source_descriptor.filepath);
		return;
	}

	internal->av_video_codec = NULL;
	internal->av_codec_context = avcodec_alloc_context3(NULL);
	avcodec_parameters_to_context(
	    internal->av_codec_context,
	    internal->av_format_context->streams[internal->av_video_streamid]
	        ->codecpar);
	// TODO: deprecated - there is probably a better way to do this
	av_codec_set_pkt_timebase(
	    internal->av_codec_context,
	    internal->av_format_context->streams[internal->av_video_streamid]
	        ->time_base);
	internal->av_video_codec =
	    avcodec_find_decoder(internal->av_codec_context->codec_id);

	if (!internal->av_video_codec) {
		printf("ERROR: unsupported video codec %d\n",
		       internal->av_codec_context->codec_id);
		return;
	}

	ret = avcodec_open2(internal->av_codec_context,
	                    internal->av_video_codec, NULL);
	if (ret < 0) {
		printf("ERROR: could not open codec %d\n",
		       internal->av_codec_context->codec_id);
		return;
	}

	while (1) {
		AVPacket packet;
		av_init_packet(&packet);
		if (!internal->av_current_frame) {
			internal->av_current_frame = av_frame_alloc();
		}
		// we have no guarantees the next packet will be a video packet,
		// and
		// we may need to read multiple packets to get a valid frame
		int video_frame_finished = 0;
		while (1) {
			while (1) {
				if (av_read_frame(internal->av_format_context,
				                  &packet) >= 0) {
					if (packet.stream_index ==
					    internal->av_video_streamid) {
						break;
					}
				} else {
					packet = empty_packet;
					break;
				}
			}
			ret = avcodec_decode_video2(internal->av_codec_context,
			                            internal->av_current_frame,
			                            &video_frame_finished,
			                            &packet);
			if (ret < 0) {
				printf("ERROR: decode problem!\n");
				return;
			}
			if (video_frame_finished > 0) {
				// we have our frame! w00t!
				// printf("got frame!\n");
				// now we need to invoke our callback with a
				// frame.
				frame_t f;
				f.source_id =
				    internal->source_descriptor.source_id;
				f.format = internal->source_descriptor.format;
				f.width = internal->av_current_frame->width;
				f.height = internal->av_current_frame->height;
				f.stride =
				    internal->av_current_frame->linesize[0];
				f.size_bytes = frame_size_in_bytes(&f);

				// since we are just PoCing, we can just pass
				// the Y plane.
				f.data = internal->av_current_frame->data[0];
				internal->frame_target_callback(
				    internal->frame_target_instance, &f);
				driver_event_t e = {};
				e.type = EVENT_FRAMESERVER_GOTFRAME;
				if (internal->event_target_callback) {
					internal->event_target_callback(
					    internal->event_target_instance, e);
				}
			}
		}
	}
}
