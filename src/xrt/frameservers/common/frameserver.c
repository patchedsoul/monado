#include "frameserver.h"
#include "ffmpeg/ffmpeg_frameserver.h";
#include "v4l2/v4l2_frameserver.h";
#include "uvc/uvc_frameserver.h";


int32_t frame_size_in_bytes(frame_t* fd) {
	printf("ERROR: Not implemented");
	return -1;
}

int32_t frame_bytes_per_pixel(frame_t* fd){
	printf("ERROR: Not implemented\n");
	return -1;
}

bool split_stereo_frame(frame_t* source, frame_t* left,  frame_t* right){
	printf("ERROR: Not implemented!\n");
	return false;
}

bool extract_plane(frame_t* source, plane_t plane, frame_t* out) {
	printf("ERROR: Not implemented!\n");
	return false;
}

bool frameserver_create(frameserver_type_t t,frameserver_instance_t* inst)
{
    switch (t) {
    case FRAMESERVER_FFMPEG:
        inst->frameserver_enumerate_sources = ffmpeg_frameserver_enumerate_sources;
        inst->frameserver_frame_get = ffmpeg_frame_get;
        inst->frameserver_is_running = ffmpeg_is_running;
        inst->frameserver_register_event_callback = ffmpeg_register_event_callback;
        inst->frameserver_seek = ffmpeg_seek;
        inst->frameserver_stream_stop =ffmpeg_stream_stop;
        inst->framesrerver_stream_start=ffmpeg_stream_start;
        break;
    case FRAMESERVER_V4L2:
        inst->frameserver_enumerate_sources = v4l2_frameserver_enumerate_sources;
        inst->frameserver_frame_get = v4l2_frame_get;
        inst->frameserver_is_running = v4l2_is_running;
        inst->frameserver_register_event_callback = NULL;
        inst->frameserver_seek = NULL;
        inst->frameserver_stream_stop =v4l2_stream_stop;
        inst->framesrerver_stream_start=v4l2_stream_start;
        break;
    case FRAMESERVER_UVC:
        inst->frameserver_enumerate_sources = uvc_frameserver_enumerate_sources;
        inst->frameserver_frame_get = uvc_frame_get;
        inst->frameserver_is_running = uvc_is_running;
        inst->frameserver_register_event_callback = NULL;
        inst->frameserver_seek = NULL;
        inst->frameserver_stream_stop = uvc_stream_stop;
        inst->framesrerver_stream_start = uvc_stream_start;
        break;
    case FRAMESERVER_NULL:
    default:
        //do nothing
        break;
    }
}
