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

frameserver_instance_t* frameserver_create(frameserver_type_t t) {
    frameserver_instance_t* i = calloc(1,sizeof(frameserver_instance_t));
    if (i) {
        switch (t) {
		    case FRAMESERVER_TYPE_FFMPEG:
                i->frameserver_enumerate_sources = ffmpeg_frameserver_enumerate_sources;
                i->frameserver_frame_get = ffmpeg_frame_get;
                i->frameserver_is_running = ffmpeg_is_running;
                i->frameserver_register_event_callback = ffmpeg_register_event_callback;
                i->frameserver_seek = ffmpeg_seek;
                i->frameserver_stream_stop =ffmpeg_stream_stop;
                i->frameserver_stream_start=ffmpeg_stream_start;
                i->internal_instance = (void*) ffmpeg_frameserver_create(i);
                break;
		    case FRAMESERVER_TYPE_UVC:
                i->frameserver_enumerate_sources = uvc_frameserver_enumerate_sources;
				i->frameserver_frame_get = uvc_frameserver_get;
				i->frameserver_is_running = uvc_frameserver_is_running;
				i->frameserver_register_event_callback = uvc_frameserver_register_event_callback;
				i->frameserver_seek = uvc_frameserver_seek;
				i->frameserver_stream_stop =uvc_frameserver_stream_stop;
				i->frameserver_stream_start=uvc_frameserver_stream_start;
                i->internal_instance = (void*) uvc_frameserver_create(i);
                break;
		    case FRAMESERVER_TYPE_V4L2:
                i->frameserver_enumerate_sources = v4l2_frameserver_enumerate_sources;
                i->frameserver_frame_get = v4l2_frame_get;
                i->frameserver_is_running = v4l2_is_running;
                i->frameserver_register_event_callback = v4l2_register_event_callback;
                i->frameserver_seek = v4l2_seek;
                i->frameserver_stream_stop =v4l2_stream_stop;
                i->frameserver_stream_start=v4l2_stream_start;
                i->internal_instance = (void*) v4l2_frameserver_create(i);
                break;
		    case FRAMESERVER_TYPE_NONE:
            default:
                free(i);
                return NULL;
            break;
		}
        return i;
    }
    return NULL;
}

bool frameservers_test() {

    ffmpeg_frameserver_test();
   // uvc_frameserver_test();
   // v4l2_frameserver_test();
    return true;
}
