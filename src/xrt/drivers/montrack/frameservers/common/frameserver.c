#include "frameserver.h"

#ifdef XRT_HAVE_FFMPEG
#include "ffmpeg/ffmpeg_frameserver.h"
#endif // XRT_HAVE_FFMPEG

#ifdef XRT_HAVE_LIBUVC
#include "uvc/uvc_frameserver.h"
#endif // XRT_HAVE_LIBUVC

#include "v4l2/v4l2_frameserver.h"

#include "util/u_misc.h"

float
format_bytes_per_pixel(frame_format_t f)
{
	switch (f) {
	case FORMAT_Y_UINT8: return 1.0f;
	case FORMAT_YUV420_UINT8: return 1.5f;
	case FORMAT_Y_UINT16:
	case FORMAT_YUV422_UINT8:
	case FORMAT_YUYV_UINT8: return 2.0f;
	case FORMAT_BGR_UINT8:
	case FORMAT_RGB_UINT8:
	case FORMAT_YUV444_UINT8: return 3.0f;
	case FORMAT_RAW:
	case FORMAT_JPG:
	default:
		printf("cannot compute format bytes per pixel\n");
		return -1.0f;
	}
	return -1.0f;
}


int32_t
frame_size_in_bytes(frame_t* f)
{
	if (f) {
		int32_t frame_bytes = -1;
		// TODO: alpha formats, padding etc.
		switch (f->format) {
		case FORMAT_Y_UINT8:
		case FORMAT_YUV420_UINT8:
		case FORMAT_Y_UINT16:
		case FORMAT_YUV422_UINT8:
		case FORMAT_BGR_UINT8:
		case FORMAT_RGB_UINT8:
		case FORMAT_YUV444_UINT8:
		case FORMAT_YUYV_UINT8:
			frame_bytes = f->stride * f->height;
			break;
		case FORMAT_JPG:
			// this is a maximum (assuming YUV444)
			frame_bytes = f->width * f->height * 3;
		case FORMAT_RAW:
		case FORMAT_NONE:
		default: printf("cannot compute frame size for this format\n");
		}
		return frame_bytes;
	}
	return -1;
}

int32_t
frame_bytes_per_pixel(frame_t* f)
{
	printf("ERROR: Not implemented\n");
	return -1;
}


bool
frame_split_stereo(frame_t* source, frame_t* left, frame_t* right)
{
	printf("ERROR: Not implemented!\n");
	return false;
}

bool
frame_extract_plane(frame_t* source, plane_t plane, frame_t* out)
{
	// only handle splitting Y out of YUYV for now
	if (source->format != FORMAT_YUYV_UINT8 && plane != PLANE_Y) {
		printf("ERROR: unhandled plane extraction\n");
		return false;
	}
	if (!source->data) {
		printf("ERROR: no frame data!\n");
		return false;
	}

	uint8_t* source_ptr;
	uint8_t* dest_ptr;
	uint8_t source_pixel_bytes = format_bytes_per_pixel(source->format);
	uint32_t source_line_bytes = source->stride;
	uint8_t dest_pixel_bytes = format_bytes_per_pixel(out->format);
	uint32_t dest_line_bytes = out->width;

	if (!out->data) {
		printf(
		    "allocating data for NULL plane - someone needs to free "
		    "this!\n");
		out->data = malloc(frame_size_in_bytes(out));
	}

	switch (source->format) {
	case FORMAT_YUYV_UINT8:
	case FORMAT_YUV444_UINT8:
		for (uint32_t i = 0; i < source->height; i++) {
			for (uint32_t j = 0; j < source->width; j++) {
				source_ptr = source->data +
				             (j * source_pixel_bytes) +
				             (i * source_line_bytes);
				dest_ptr = out->data + (j * dest_pixel_bytes) +
				           (i * dest_line_bytes);
				*dest_ptr = *source_ptr;
			}
		}
		break;
	default: return false;
	}
	return true;
}

bool
frame_resample(frame_t* source, frame_t* out)
{
	// TODO: more complete resampling.
	if (source->format != FORMAT_YUYV_UINT8 &&
	    out->format != FORMAT_YUV444_UINT8) {
		printf("ERROR: unhandled resample operation\n");
		return false;
	}

	if (!source->data) {
		printf("ERROR: no frame data!\n");
		return false;
	}

	uint8_t* source_ptr;
	uint8_t* dest_ptr;
	uint8_t source_pixel_bytes = format_bytes_per_pixel(source->format);
	uint32_t source_line_bytes = source->stride;
	uint8_t dest_pixel_bytes = format_bytes_per_pixel(out->format);
	uint32_t dest_line_bytes = out->stride;

	if (!out->data) {
		printf(
		    "allocating data for NULL plane - someone needs to free "
		    "this!\n");
		out->data = (uint8_t*)malloc(frame_size_in_bytes(out));
	}
	uint8_t lastU = 0;
	switch (source->format) {
	case FORMAT_YUYV_UINT8:
		for (uint32_t i = 0; i < source->height; i++) {
			for (uint32_t j = 0; j < source->width; j++) {
				source_ptr = source->data +
				             (j * source_pixel_bytes) +
				             (i * source_line_bytes);
				dest_ptr = out->data + (j * dest_pixel_bytes) +
				           (i * dest_line_bytes);
				*dest_ptr = *source_ptr; // Y
				if (j % 2 == 0) {
					*(dest_ptr + 1) =
					    *(source_ptr + 1); // U
					*(dest_ptr + 2) =
					    *(source_ptr +
					      3); // V from next source pixel
					lastU = *(dest_ptr + 1);
				} else {
					*(dest_ptr + 1) = lastU;
					*(dest_ptr + 2) = *(source_ptr + 1);
				}
			}
		}
		return true;
		break;
	default: return false;
	}
	return false;
}


frameserver_instance_t*
frameserver_create(frameserver_type_t t)
{
	frameserver_instance_t* i = U_TYPED_CALLOC(frameserver_instance_t);
	frameserver_internal_instance_ptr internal = NULL;
	if (i == NULL) {
		return NULL;
	}
	/*
	 * Each implementation constructor should set up the members of the
	 * frameserver instance, as well as return a pointer to itself. If it
	 * fails, it should return NULL without de-allocating the frameserver
	 * instance: that is the responsibility of this function.
	 */
	switch (t) {
#ifdef XRT_HAVE_FFMPEG
	case FRAMESERVER_TYPE_FFMPEG:
		internal = (frameserver_internal_instance_ptr)
		    ffmpeg_frameserver_create(i);
		break;
#endif // XRT_HAVE_FFMPEG

#ifdef XRT_HAVE_LIBUVC
	case FRAMESERVER_TYPE_UVC:
		internal =
		    (frameserver_internal_instance_ptr)uvc_frameserver_create(
		        i);
		break;
#endif // XRT_HAVE_LIBUVC

	case FRAMESERVER_TYPE_V4L2:
		internal =
		    (frameserver_internal_instance_ptr)v4l2_frameserver_create(
		        i);
		break;

	case FRAMESERVER_TYPE_NONE:
	default:
		free(i);
		return NULL;
		break;
	}
	if (internal == NULL) {
		/* Failed to allocate/create internal implementation */
		free(i);
		return NULL;
	}
	return i;
}

bool
frameservers_test()
{

	ffmpeg_frameserver_test();
	// uvc_frameserver_test();
	// v4l2_frameserver_test();
	return true;
}
