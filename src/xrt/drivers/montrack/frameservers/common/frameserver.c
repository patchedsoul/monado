// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of frameserver interface and shared functions.
 * @author Pete Black <pblack@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */


#include "frameserver.h"

#ifdef XRT_HAVE_FFMPEG
#include "ffmpeg/ffmpeg_frameserver.h"
#endif // XRT_HAVE_FFMPEG

#ifdef XRT_HAVE_LIBUVC
#include "uvc/uvc_frameserver.h"
#endif // XRT_HAVE_LIBUVC

#include "v4l2/v4l2_frameserver.h"

#include <stdio.h>
#include <stdlib.h>

struct frameserver*
frameserver_create(enum frameserver_type t)
{
	/*
	 * Each implementation constructor should set up the members of the
	 * frameserver instance, as well as return a pointer to itself. If it
	 * fails, it should return NULL without de-allocating the frameserver
	 * instance: that is the responsibility of this function.
	 */
	switch (t) {
#ifdef XRT_HAVE_FFMPEG
	case FRAMESERVER_TYPE_FFMPEG: return ffmpeg_frameserver_create();
#endif // XRT_HAVE_FFMPEG

#ifdef XRT_HAVE_LIBUVC
	case FRAMESERVER_TYPE_UVC: return uvc_frameserver_create();
#endif // XRT_HAVE_LIBUVC

	case FRAMESERVER_TYPE_V4L2: return v4l2_frameserver_create();

	case FRAMESERVER_TYPE_NONE:
	default: return NULL;
	}
}

float
fs_format_bytes_per_pixel(enum fs_frame_format f)
{
	switch (f) {
	case FS_FORMAT_Y_UINT8: return 1.0f;
	case FS_FORMAT_YUV420_UINT8: return 1.5f;
	case FS_FORMAT_Y_UINT16:
	case FS_FORMAT_YUV422_UINT8:
	case FS_FORMAT_YUYV_UINT8: return 2.0f;
	case FS_FORMAT_BGR_UINT8:
	case FS_FORMAT_RGB_UINT8:
	case FS_FORMAT_YUV444_UINT8: return 3.0f;
	case FS_FORMAT_RAW:
	case FS_FORMAT_JPG:
	default:
		printf("cannot compute format bytes per pixel\n");
		return -1.0f;
	}
	return -1.0f;
}


int32_t
fs_frame_size_in_bytes(struct fs_frame* f)
{
	if (f) {
		int32_t frame_bytes = -1;
		// TODO: alpha formats, padding etc.
		switch (f->format) {
		case FS_FORMAT_Y_UINT8:
		case FS_FORMAT_YUV420_UINT8:
		case FS_FORMAT_Y_UINT16:
		case FS_FORMAT_YUV422_UINT8:
		case FS_FORMAT_BGR_UINT8:
		case FS_FORMAT_RGB_UINT8:
		case FS_FORMAT_YUV444_UINT8:
		case FS_FORMAT_YUYV_UINT8:
			frame_bytes = f->stride * f->height;
			break;
		case FS_FORMAT_JPG:
			// this is a maximum (assuming YUV444)
			frame_bytes = f->width * f->height * 3;
			break;
		case FS_FORMAT_RAW:
		case FS_FORMAT_NONE:
		default: printf("cannot compute frame size for this format\n");
		}
		return frame_bytes;
	}
	return -1;
}

int32_t
fs_frame_bytes_per_pixel(struct fs_frame* f)
{
	printf("ERROR: Not implemented\n");
	return -1;
}


bool
fs_frame_split_stereo(struct fs_frame* source,
                      struct fs_frame* left,
                      struct fs_frame* right)
{
	printf("ERROR: Not implemented!\n");
	return false;
}

bool
fs_frame_extract_plane(struct fs_frame* source,
                       enum fs_plane plane,
                       struct fs_frame* out)
{
	// only handle splitting Y out of YUYV for now
	if (source->format != FS_FORMAT_YUYV_UINT8 && plane != FS_PLANE_Y) {
		printf("ERROR: unhandled plane extraction\n");
		return false;
	}
	if (!source->data) {
		printf("ERROR: no frame data!\n");
		return false;
	}

	uint8_t* source_ptr;
	uint8_t* dest_ptr;
	uint8_t source_pixel_bytes = fs_format_bytes_per_pixel(source->format);
	uint32_t source_line_bytes = source->stride;
	uint8_t dest_pixel_bytes = fs_format_bytes_per_pixel(out->format);
	uint32_t dest_line_bytes = out->width;

	if (!out->data) {
		printf(
		    "allocating data for NULL plane - someone needs to free "
		    "this!\n");
		out->data = malloc(fs_frame_size_in_bytes(out));
	}

	switch (source->format) {
	case FS_FORMAT_YUYV_UINT8:
	case FS_FORMAT_YUV444_UINT8:
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
fs_frame_resample(struct fs_frame* source, struct fs_frame* out)
{
	// TODO: more complete resampling.
	if (source->format != FS_FORMAT_YUYV_UINT8 &&
	    out->format != FS_FORMAT_YUV444_UINT8) {
		printf("ERROR: unhandled resample operation\n");
		return false;
	}

	if (!source->data) {
		printf("ERROR: no frame data!\n");
		return false;
	}

	uint8_t* source_ptr;
	uint8_t* dest_ptr;
	uint8_t source_pixel_bytes = fs_format_bytes_per_pixel(source->format);
	uint32_t source_line_bytes = source->stride;
	uint8_t dest_pixel_bytes = fs_format_bytes_per_pixel(out->format);
	uint32_t dest_line_bytes = out->stride;

	if (!out->data) {
		printf(
		    "allocating data for NULL plane - someone needs to free "
		    "this!\n");
		out->data = (uint8_t*)malloc(fs_frame_size_in_bytes(out));
	}
	uint8_t lastU = 0;
	switch (source->format) {
	case FS_FORMAT_YUYV_UINT8:
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

bool
frameservers_test()
{

	ffmpeg_frameserver_test();
	// uvc_frameserver_test();
	// v4l2_frameserver_test();
	return true;
}
