#include "frameserver.h"
#include "ffmpeg/ffmpeg_frameserver.h";
#include "v4l2/v4l2_frameserver.h";
#include "uvc/uvc_frameserver.h";


float format_bytes_per_pixel(frame_format_t f){
	switch (f){
	case FORMAT_Y_UINT8:
		return 1.0f;
	case FORMAT_YUV420_UINT8:
		return 1.5f;
	case FORMAT_Y_UINT16:
	case FORMAT_YUV422_UINT8:
	case FORMAT_YUYV_UINT8:
		return 2.0f;
	case FORMAT_BGR_UINT8:
	case FORMAT_RGB_UINT8:
	case FORMAT_YUV444_UINT8:
		return 3.0f;
	case FORMAT_RAW:
	case FORMAT_JPG:
	default:
		printf("cannot compute format size\n");
		return -1.0f;
	}
	return -1.0f;
}


int32_t frame_size_in_bytes(frame_t* f) {
	if (f) {
		int32_t frame_bytes = -1;
		//TODO: alpha formats, padding etc.
		switch (f->format){
		case FORMAT_Y_UINT8:
		case FORMAT_YUV420_UINT8:
		case FORMAT_Y_UINT16:
		case FORMAT_YUV422_UINT8:
		case FORMAT_BGR_UINT8:
		case FORMAT_RGB_UINT8:
		case FORMAT_YUV444_UINT8:
		case FORMAT_YUYV_UINT8:
			frame_bytes = f->stride * f->height * format_bytes_per_pixel(f->format);
			break;
		case FORMAT_RAW:
		case FORMAT_JPG:
		default:
			printf("cannot compute frame size for this format\n");
		}
	return frame_bytes;
	}
	return -1;
}

int32_t frame_bytes_per_pixel(frame_t* f){
	printf("ERROR: Not implemented\n");
	return -1;
}


bool frame_split_stereo(frame_t* source, frame_t* left,  frame_t* right){
	printf("ERROR: Not implemented!\n");
	return false;
}

bool frame_extract_plane(frame_t* source, plane_t plane, frame_t* out) {
	//only handle splitting Y out of YUYV for now
	if (source->format != FORMAT_YUYV_UINT8 && plane != PLANE_Y){
		printf("ERROR: unhandled plane extraction\n");
		return false;
	}
	if (! source->data)
	{
		printf("ERROR: no frame data!\n");
		return false;
	}

	//Y from YUYV

	if (! out->data)
	{
		printf("allocating data for plane - someone needs to free this!\n");
		out->data = malloc(source->width*source->height);
	}

	uint8_t* source_ptr;
	uint8_t* dest_ptr;
	uint8_t source_pixel_bytes = format_bytes_per_pixel(source->format);	
	uint32_t source_line_bytes = source->stride * format_bytes_per_pixel(source->format);
	uint8_t dest_pixel_bytes=1;
	uint32_t dest_line_bytes = source->width;
	for (uint32_t i=0;i< source->height;i++) {
		for (uint32_t j=0;j<source->stride;j++) {
			source_ptr = source->data + (j * source_pixel_bytes) + (i * source_line_bytes); 
			dest_ptr = out->data + (j * dest_pixel_bytes) + (i * dest_line_bytes);
			*dest_ptr = *source_ptr;
		}
	}
	return true;
}

frameserver_instance_t* frameserver_create(frameserver_type_t t) {
    frameserver_instance_t* i = calloc(1,sizeof(frameserver_instance_t));
    if (i) {
        switch (t) {
		    case FRAMESERVER_TYPE_FFMPEG:
                i->frameserver_enumerate_sources = ffmpeg_frameserver_enumerate_sources;
				i->frameserver_configure_capture = ffmpeg_frameserver_configure_capture;
				i->frameserver_frame_get = ffmpeg_frameserver_get;
				i->frameserver_is_running = ffmpeg_frameserver_is_running;
				i->frameserver_register_frame_callback = ffmpeg_frameserver_register_frame_callback;
				i->frameserver_register_event_callback = ffmpeg_frameserver_register_event_callback;
				i->frameserver_seek = ffmpeg_frameserver_seek;
				i->frameserver_stream_stop =ffmpeg_frameserver_stream_stop;
				i->frameserver_stream_start=ffmpeg_frameserver_stream_start;
                i->internal_instance = (void*) ffmpeg_frameserver_create(i);
                break;
		    case FRAMESERVER_TYPE_UVC:
                i->frameserver_enumerate_sources = uvc_frameserver_enumerate_sources;
				i->frameserver_configure_capture = uvc_frameserver_configure_capture;
				i->frameserver_frame_get = uvc_frameserver_get;
				i->frameserver_is_running = uvc_frameserver_is_running;
				i->frameserver_register_frame_callback = uvc_frameserver_register_frame_callback;
				i->frameserver_register_event_callback = uvc_frameserver_register_event_callback;
				i->frameserver_seek = uvc_frameserver_seek;
				i->frameserver_stream_stop =uvc_frameserver_stream_stop;
				i->frameserver_stream_start=uvc_frameserver_stream_start;
                i->internal_instance = (void*) uvc_frameserver_create(i);
                break;
		    case FRAMESERVER_TYPE_V4L2:
                i->frameserver_enumerate_sources = v4l2_frameserver_enumerate_sources;
				i->frameserver_configure_capture = v4l2_frameserver_configure_capture;
				i->frameserver_frame_get = v4l2_frameserver_get;
				i->frameserver_is_running = v4l2_frameserver_is_running;
				i->frameserver_register_frame_callback = v4l2_frameserver_register_frame_callback;
				i->frameserver_register_event_callback = v4l2_frameserver_register_event_callback;
				i->frameserver_seek = v4l2_frameserver_seek;
				i->frameserver_stream_stop =v4l2_frameserver_stream_stop;
				i->frameserver_stream_start=v4l2_frameserver_stream_start;
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
	//uvc_frameserver_test();
   // v4l2_frameserver_test();
    return true;
}
