// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  @ref xrt_frame_sink converters and other helpers.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_format.h"

#include <stdio.h>

#ifdef XRT_HAVE_JPEG
#include "jpeglib.h"
#endif


/*
 *
 * Structs
 *
 */

struct u_sink_converter
{
	struct xrt_frame_sink base;
	struct xrt_frame_node node;

	struct xrt_frame_sink *downstream;

	struct xrt_frame *frame;

	enum xrt_format format;
};


/*
 *
 * YUV functions.
 *
 */

static inline int
clamp_to_byte(int v)
{
	if (v < 0) {
		return 0;
	}
	if (v >= 255) {
		return 255;
	}
	return v;
}

static inline uint32_t
YUV444_to_RGBX8888(int y, int u, int v)
{
	int C = y - 16;
	int D = u - 128;
	int E = v - 128;

	int R = clamp_to_byte((298 * C + 409 * E + 128) >> 8);
	int G = clamp_to_byte((298 * C - 100 * D - 209 * E + 128) >> 8);
	int B = clamp_to_byte((298 * C + 516 * D + 128) >> 8);

	return B << 16 | G << 8 | R;
}

#undef USE_TABLE
#ifdef USE_TABLE
static uint32_t lookup_YUV_to_RGBX[256][256][256] = {0};

static void
generate_lookup_YUV_to_RGBX()
{
	if (lookup_YUV_to_RGBX[255][255][255] != 0) {
		return;
	}

	int y;
	int u;
	int v;

	for (y = 0; y < 256; y++) {
		for (u = 0; u < 256; u++) {
			for (v = 0; v < 256; v++) {
				lookup_YUV_to_RGBX[y][u][v] =
				    YUV444_to_RGBX8888(y, u, v);
			}
		}
	}
}
#endif

inline static void
YUV422_to_R8G8B8X8(const uint8_t *input, uint32_t *rgb1, uint32_t *rgb2)
{
	uint8_t y0 = input[0];
	uint8_t u = input[1];
	uint8_t y1 = input[2];
	uint8_t v = input[3];

#ifdef USE_TABLE
	*rgb1 = lookup_YUV_to_RGBX[y0][u][v];
	*rgb2 = lookup_YUV_to_RGBX[y1][u][v];
#else
	*rgb1 = YUV444_to_RGBX8888(y0, u, v);
	*rgb2 = YUV444_to_RGBX8888(y1, u, v);
#endif
}

inline static void
YUV422_to_R8G8B8(const uint8_t *input, uint8_t *dst)
{
	uint8_t y0 = input[0];
	uint8_t u = input[1];
	uint8_t y1 = input[2];
	uint8_t v = input[3];

#ifdef USE_TABLE
	uint8_t *rgb1 = (uint8_t *)&lookup_YUV_to_RGBX[y0][u][v];
	uint8_t *rgb2 = (uint8_t *)&lookup_YUV_to_RGBX[y1][u][v];
#else
	uint32_t rgb1v = YUV444_to_RGBX8888(y0, u, v);
	uint32_t rgb2v = YUV444_to_RGBX8888(y1, u, v);
	uint8_t *rgb1 = (uint8_t *)&rgb1v;
	uint8_t *rgb2 = (uint8_t *)&rgb2v;
#endif

	dst[0] = rgb1[0];
	dst[1] = rgb1[1];
	dst[2] = rgb1[2];
	dst[3] = rgb2[0];
	dst[4] = rgb2[1];
	dst[5] = rgb2[2];
}

inline static void
YUV444_to_R8G8B8(const uint8_t *input, uint8_t *dst)
{
	uint8_t y = input[0];
	uint8_t u = input[1];
	uint8_t v = input[2];

#ifdef USE_TABLE
	uint8_t *rgb = (uint8_t *)&lookup_YUV_to_RGBX[y][u][v];
#else
	uint32_t rgbv = YUV444_to_RGBX8888(y, u, v);
	uint8_t *rgb = (uint8_t *)&rgbv;
#endif

	dst[0] = rgb[0];
	dst[1] = rgb[1];
	dst[2] = rgb[2];
}

static void
from_YUV422_to_R8G8B8(struct u_sink_converter *s,
                      uint32_t w,
                      uint32_t h,
                      size_t stride,
                      const uint8_t *data)
{
	for (uint32_t y = 0; y < h; y++) {
		for (uint32_t x = 0; x < w; x += 2) {
			const uint8_t *src = data;
			uint8_t *dst = s->frame->data;

			src = src + (y * stride) + (x * 2);
			dst = dst + (y * s->frame->stride) + (x * 3);
			YUV422_to_R8G8B8(src, dst);
		}
	}
}

static void
from_YUV888_to_R8G8B8(struct u_sink_converter *s,
                      uint32_t w,
                      uint32_t h,
                      size_t stride,
                      const uint8_t *data)
{
	for (uint32_t y = 0; y < h; y++) {
		for (uint32_t x = 0; x < w; x++) {
			const uint8_t *src = data;
			uint8_t *dst = s->frame->data;

			src = src + (y * stride) + (x * 3);
			dst = dst + (y * s->frame->stride) + (x * 3);
			YUV444_to_R8G8B8(src, dst);
		}
	}
}


/*
 *
 * MJPEG
 *
 */

#ifdef XRT_HAVE_JPEG
static bool
check_header(size_t size, const uint8_t *data)
{
	if (size < 16) {
		fprintf(stderr, "error: Invalid JPEG file size! %u\n",
		        (uint32_t)size);
		return false;
	}

	if (data[0] != 0xFF || data[1] != 0xD8) {
		fprintf(stderr, "error: Invalid file header! 0x%02X 0x%02X\n",
		        data[0], data[1]);
		return false;
	}

	return true;
}

static bool
from_MJPEG_to_R8G8B8(struct u_sink_converter *s,
                     size_t size,
                     const uint8_t *data)
{
	if (!check_header(size, data)) {
		return false;
	}

	struct jpeg_decompress_struct cinfo = {0};
	struct jpeg_error_mgr jerr = {0};

	cinfo.err = jpeg_std_error(&jerr);
	jerr.trace_level = 0;

	jpeg_create_decompress(&cinfo);
	jpeg_mem_src(&cinfo, data, size);

	int ret = jpeg_read_header(&cinfo, TRUE);
	if (ret != JPEG_HEADER_OK) {
		jpeg_destroy_decompress(&cinfo);
		return false;
	}

	cinfo.out_color_space = JCS_RGB;
	jpeg_start_decompress(&cinfo);

	uint8_t *moving_ptr = s->frame->data;

	uint32_t scanlines_read = 0;
	while (scanlines_read < cinfo.image_height) {
		int read_count = jpeg_read_scanlines(&cinfo, &moving_ptr, 16);
		moving_ptr += read_count * s->frame->stride;
		scanlines_read += read_count;
	}

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	return true;
}

static bool
from_MJPEG_to_YUV888(struct u_sink_converter *s,
                     size_t size,
                     const uint8_t *data)
{
	if (!check_header(size, data)) {
		return false;
	}

	struct jpeg_decompress_struct cinfo = {0};
	struct jpeg_error_mgr jerr = {0};

	cinfo.err = jpeg_std_error(&jerr);
	jerr.trace_level = 0;

	jpeg_create_decompress(&cinfo);
	jpeg_mem_src(&cinfo, data, size);

	int ret = jpeg_read_header(&cinfo, TRUE);
	if (ret != JPEG_HEADER_OK) {
		jpeg_destroy_decompress(&cinfo);
		return false;
	}

	cinfo.out_color_space = JCS_YCbCr;
	jpeg_start_decompress(&cinfo);

	uint8_t *moving_ptr = s->frame->data;

	uint32_t scanlines_read = 0;
	while (scanlines_read < cinfo.image_height) {
		int read_count = jpeg_read_scanlines(&cinfo, &moving_ptr, 16);
		moving_ptr += read_count * s->frame->stride;
		scanlines_read += read_count;
	}

	jpeg_finish_decompress(&cinfo);
	jpeg_destroy_decompress(&cinfo);

	return true;
}
#endif


/*
 *
 * Misc functions.
 *
 */

static void
ensure_data(struct u_sink_converter *s,
            enum xrt_format format,
            uint32_t w,
            uint32_t h)
{
	u_frame_create_one_off(format, w, h, &s->frame);
}

static void
push_data_downstream(struct u_sink_converter *s, struct xrt_frame *xf)
{
	// The frame has a single reference on it when it's on the converter
	// struct, we move it so no need to change the ref count.
	struct xrt_frame *frame = s->frame;
	s->frame = NULL;

	// Copy directly from original frame.
	frame->timestamp = xf->timestamp;
	frame->source_timestamp = xf->source_timestamp;
	frame->source_sequence = xf->source_sequence;
	frame->source_id = xf->source_id;
	frame->stereo_format = xf->stereo_format;

	s->downstream->push_frame(s->downstream, frame);

	// Refcount in case it's being held downstream.
	xrt_frame_reference(&frame, NULL);
}

static void
receive_frame_r8g8b8_or_l8(struct xrt_frame_sink *xs, struct xrt_frame *xf)
{
	struct u_sink_converter *s = (struct u_sink_converter *)xs;

	switch (xf->format) {
	case XRT_FORMAT_L8:
	case XRT_FORMAT_R8G8B8:
		s->downstream->push_frame(s->downstream, xf);
		return;
	case XRT_FORMAT_YUV422:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		from_YUV422_to_R8G8B8(s, xf->width, xf->height, xf->stride,
		                      xf->data);
		break;
	case XRT_FORMAT_YUV888:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		from_YUV888_to_R8G8B8(s, xf->width, xf->height, xf->stride,
		                      xf->data);
		break;
#ifdef XRT_HAVE_JPEG
	case XRT_FORMAT_MJPEG:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		if (!from_MJPEG_to_R8G8B8(s, xf->size, xf->data)) {
			return;
		}
		break;
#endif
	default:
		fprintf(stderr,
		        "error: Can not convert from '%s' to R8G8B8 or L8!\n",
		        u_format_str(xf->format));
		return;
	}

	push_data_downstream(s, xf);
}

static void
receive_frame_r8g8b8(struct xrt_frame_sink *xs, struct xrt_frame *xf)
{
	struct u_sink_converter *s = (struct u_sink_converter *)xs;

	switch (xf->format) {
	case XRT_FORMAT_R8G8B8:
		s->downstream->push_frame(s->downstream, xf);
		return;
	case XRT_FORMAT_YUV422:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		from_YUV422_to_R8G8B8(s, xf->width, xf->height, xf->stride,
		                      xf->data);
		break;
	case XRT_FORMAT_YUV888:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		from_YUV888_to_R8G8B8(s, xf->width, xf->height, xf->stride,
		                      xf->data);
		break;
#ifdef XRT_HAVE_JPEG
	case XRT_FORMAT_MJPEG:
		ensure_data(s, XRT_FORMAT_R8G8B8, xf->width, xf->height);
		if (!from_MJPEG_to_R8G8B8(s, xf->size, xf->data)) {
			return;
		}
		break;
#endif
	default:
		fprintf(stderr, "error: Can not convert from '%s' to R8G8B8!\n",
		        u_format_str(xf->format));
		return;
	}

	push_data_downstream(s, xf);
}

static void
receive_frame_yuv_yuyv_or_l8(struct xrt_frame_sink *xs, struct xrt_frame *xf)
{
	struct u_sink_converter *s = (struct u_sink_converter *)xs;


	switch (xf->format) {
	case XRT_FORMAT_L8:
	case XRT_FORMAT_YUV422:
	case XRT_FORMAT_YUV888:
		s->downstream->push_frame(s->downstream, xf);
		return;
#ifdef XRT_HAVE_JPEG
	case XRT_FORMAT_MJPEG:
		ensure_data(s, XRT_FORMAT_YUV888, xf->width, xf->height);
		if (!from_MJPEG_to_YUV888(s, xf->size, xf->data)) {
			return;
		}
		break;
#endif
	default:
		fprintf(stderr,
		        "error: Can not convert from '%s' to either YUV, YUYV "
		        "or L8!\n",
		        u_format_str(xf->format));
		return;
	}

	push_data_downstream(s, xf);
}

static void
receive_frame_yuv_or_yuyv(struct xrt_frame_sink *xs, struct xrt_frame *xf)
{
	struct u_sink_converter *s = (struct u_sink_converter *)xs;


	switch (xf->format) {
	case XRT_FORMAT_YUV422:
	case XRT_FORMAT_YUV888:
		s->downstream->push_frame(s->downstream, xf);
		return;
#ifdef XRT_HAVE_JPEG
	case XRT_FORMAT_MJPEG:
		ensure_data(s, XRT_FORMAT_YUV888, xf->width, xf->height);
		if (!from_MJPEG_to_YUV888(s, xf->size, xf->data)) {
			return;
		}
		break;
#endif
	default:
		fprintf(
		    stderr,
		    "error: Can not convert from '%s' to either YUV or YUYV!\n",
		    u_format_str(xf->format));
		return;
	}

	push_data_downstream(s, xf);
}

static void
break_apart(struct xrt_frame_node *node)
{}

static void
destroy(struct xrt_frame_node *node)
{
	struct u_sink_converter *s =
	    container_of(node, struct u_sink_converter, node);

	free(s);
}


/*
 *
 * "Exported" functions.
 *
 */

void
u_sink_create_format_converter(struct xrt_frame_context *xfctx,
                               enum xrt_format f,
                               struct xrt_frame_sink *downstream,
                               struct xrt_frame_sink **out_xfs)
{
	if (f != XRT_FORMAT_R8G8B8) {
		fprintf(stderr, "error: Format '%s' not supported\n",
		        u_format_str(f));
		return;
	}

#ifdef USE_TABLE
	generate_lookup_YUV_to_RGBX();
#endif

	struct u_sink_converter *s = U_TYPED_CALLOC(struct u_sink_converter);
	s->base.push_frame = receive_frame_r8g8b8;
	s->node.break_apart = break_apart;
	s->node.destroy = destroy;
	s->downstream = downstream;

	xrt_frame_context_add(xfctx, &s->node);

	*out_xfs = &s->base;
}

void
u_sink_create_to_r8g8b8_or_l8(struct xrt_frame_context *xfctx,
                              struct xrt_frame_sink *downstream,
                              struct xrt_frame_sink **out_xfs)
{
	struct u_sink_converter *s = U_TYPED_CALLOC(struct u_sink_converter);
	s->base.push_frame = receive_frame_r8g8b8_or_l8;
	s->node.break_apart = break_apart;
	s->node.destroy = destroy;
	s->downstream = downstream;

#ifdef USE_TABLE
	generate_lookup_YUV_to_RGBX();
#endif

	xrt_frame_context_add(xfctx, &s->node);

	*out_xfs = &s->base;
}

void
u_sink_create_to_yuv_yuyv_or_l8(struct xrt_frame_context *xfctx,
                                struct xrt_frame_sink *downstream,
                                struct xrt_frame_sink **out_xfs)
{
	struct u_sink_converter *s = U_TYPED_CALLOC(struct u_sink_converter);
	s->base.push_frame = receive_frame_yuv_yuyv_or_l8;
	s->node.break_apart = break_apart;
	s->node.destroy = destroy;
	s->downstream = downstream;

	xrt_frame_context_add(xfctx, &s->node);

	*out_xfs = &s->base;
}

void
u_sink_create_to_yuv_or_yuyv(struct xrt_frame_context *xfctx,
                             struct xrt_frame_sink *downstream,
                             struct xrt_frame_sink **out_xfs)
{
	struct u_sink_converter *s = U_TYPED_CALLOC(struct u_sink_converter);
	s->base.push_frame = receive_frame_yuv_or_yuyv;
	s->node.break_apart = break_apart;
	s->node.destroy = destroy;
	s->downstream = downstream;

	xrt_frame_context_add(xfctx, &s->node);

	*out_xfs = &s->base;
}
