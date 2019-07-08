// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header for frameserver interface
 * @author Pete Black <pblack@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */

#pragma once

#include "math/m_api.h"
#include "mt_events.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define FS_MAX_PLANES 3 // this is what we see currently in e.g. RGB,YUV

// frame
enum fs_frame_format
{
	FS_FORMAT_NONE,
	FS_FORMAT_RAW,
	FS_FORMAT_Y_UINT8,
	FS_FORMAT_Y_UINT16,
	FS_FORMAT_RGB_UINT8,
	FS_FORMAT_BGR_UINT8,
	FS_FORMAT_YUYV_UINT8,
	FS_FORMAT_YUV444_UINT8,
	FS_FORMAT_YUV422_UINT8,
	FS_FORMAT_YUV420_UINT8,
	FS_FORMAT_JPG
};

enum fs_stereo_format
{
	FS_STEREO_NONE,
	FS_STEREO_SBS,
	FS_STEREO_OAU
};

enum fs_plane
{
	FS_PLANE_NONE,
	FS_PLANE_R,
	FS_PLANE_G,
	FS_PLANE_B,
	FS_PLANE_Y,
	FS_PLANE_U,
	FS_PLANE_V
};

enum fs_chroma_sampling
{
	FS_CHROMA_SAMP_NONE,
	FS_CHROMA_SAMP_444,
	FS_CHROMA_SAMP_422,
	FS_CHROMA_SAMP_411
};

enum fs_plane_layout
{
	FS_PLANE_LAYOUT_COMPOSITE,
	FS_PLANE_LAYOUT_SEPARATE

};

enum fs_sampling
{
	FS_SAMPLING_NONE,
	FS_SAMPLING_UPSAMPLED,
	FS_SAMPLING_DOWNSAMPLED
};

// unnormalised pixel coordinates for clipping ROIs
struct fs_frame_rect
{
	struct xrt_vec2 tl;
	struct xrt_vec2 br;
};

// basic frame data structure - holds a pointer to buffer.
struct fs_frame
{
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	enum fs_frame_format format;
	enum fs_stereo_format stereo_format;
	uint32_t size_bytes;
	uint8_t* data;
	enum fs_chroma_sampling chroma_sampling; // unused
	enum fs_plane_layout plane_layout;       // unused
	uint8_t* u_data;                         // unused
	uint8_t* v_data;                         // unused
	uint64_t timestamp;
	uint64_t source_timestamp;
	uint64_t source_sequence; // sequence id
	uint64_t source_id;       // used to tag frames with the source they
	                          // originated from
};

struct fs_capture_parameters
{
	// used to configure cameras. since there is no guarantee every
	// frameserver will support any/all of these params, a 'best effort'
	// should be made to apply them. all numeric values are normalised
	// floats for broad applicability
	float gain;
	float exposure;
};


// frameserver

enum frameserver_type
{
	FRAMESERVER_TYPE_NONE,
	FRAMESERVER_TYPE_FFMPEG,
	FRAMESERVER_TYPE_UVC,
	FRAMESERVER_TYPE_V4L2
};

typedef void* fs_source_descriptor_ptr;
struct frameserver;

typedef void (*fs_frame_consumer_callback_func)(struct frameserver* instance,
                                                struct fs_frame* frame);

struct frameserver
{
	enum frameserver_type type;
	/*!
	 * Enumerate all available sources.
	 */
	bool (*enumerate_sources)(struct frameserver* inst,
	                          fs_source_descriptor_ptr sources,
	                          uint32_t* count);

	/*!
	 *
	 */
	bool (*configure_capture)(struct frameserver* inst,
	                          struct fs_capture_parameters cp);

	/*!
	 *
	 */
	bool (*frame_get)(struct frameserver* inst, struct fs_frame* frame);

	/*!
	 *
	 */
	void (*register_event_callback)(
	    struct frameserver* inst,
	    void* target_instance,
	    event_consumer_callback_func target_func);

	/*!
	 *
	 */
	bool (*seek)(struct frameserver* inst, uint64_t timestamp);

	/*!
	 *
	 */
	bool (*stream_start)(struct frameserver* inst,
	                     fs_source_descriptor_ptr source);

	/*!
	 *
	 */
	bool (*stream_stop)(struct frameserver* inst);

	/*!
	 *
	 */
	bool (*is_running)(struct frameserver* inst);
	void (*destroy)(struct frameserver* inst);
};

struct frameserver* frameserver_create(enum frameserver_type);

static inline bool
frameserver_enumerate_sources(struct frameserver* inst,
                              fs_source_descriptor_ptr sources,
                              uint32_t* count)
{
	return inst->enumerate_sources(inst, sources, count);
}

static inline bool
frameserver_configure_capture(struct frameserver* inst,
                              struct fs_capture_parameters cp)
{
	return inst->configure_capture(inst, cp);
}

static inline bool
frameserver_frame_get(struct frameserver* inst, struct fs_frame* _frame)
{
	return inst->frame_get(inst, _frame);
}


static inline void
frameserver_register_event_callback(struct frameserver* inst,
                                    void* target_instance,
                                    event_consumer_callback_func target_func)
{
	inst->register_event_callback(inst, target_instance, target_func);
}

static inline bool
frameserver_seek(struct frameserver* inst, uint64_t timestamp)
{
	return inst->seek(inst, timestamp);
}

static inline bool
frameserver_stream_start(struct frameserver* inst,
                         fs_source_descriptor_ptr source)
{
	return inst->stream_start(inst, source);
}

static inline bool
frameserver_stream_stop(struct frameserver* inst)
{
	return inst->stream_stop(inst);
}

static inline bool
frameserver_is_running(struct frameserver* inst)
{
	return inst->is_running(inst);
}
static inline void
frameserver_destroy(struct frameserver* inst)
{
	inst->destroy(inst);
}


int32_t
fs_frame_size_in_bytes(struct fs_frame* f);

int32_t
fs_frame_bytes_per_pixel(struct fs_frame* f);

float
fs_format_bytes_per_pixel(
    enum fs_frame_format f); // this is a float to support e.g. YUV420

bool
fs_frame_split_stereo(struct fs_frame* source,
                      struct fs_frame* left,
                      struct fs_frame* right);

bool
fs_frame_extract_plane(struct fs_frame* source,
                       enum fs_plane plane,
                       struct fs_frame* out);

bool
fs_frame_resample(struct fs_frame* source, struct fs_frame* out);

bool
frameservers_test();

#ifdef __cplusplus
}
#endif
