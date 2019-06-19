#ifndef FRAMESERVER_H
#define FRAMESERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math/m_api.h>

#include "../mt_events.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PLANES 3 // this is what we see currently in e.g. RGB,YUV

// frame
typedef enum frame_format
{
	FORMAT_NONE,
	FORMAT_RAW,
	FORMAT_Y_UINT8,
	FORMAT_Y_UINT16,
	FORMAT_RGB_UINT8,
	FORMAT_BGR_UINT8,
	FORMAT_YUYV_UINT8,
	FORMAT_YUV444_UINT8,
	FORMAT_YUV422_UINT8,
	FORMAT_YUV420_UINT8,
	FORMAT_JPG
} frame_format_t;
typedef enum stereo_format
{
	STEREO_NONE,
	STEREO_SBS,
	STEREO_OAU
} stereo_format_t;
typedef enum plane
{
	PLANE_NONE,
	PLANE_R,
	PLANE_G,
	PLANE_B,
	PLANE_Y,
	PLANE_U,
	PLANE_V
} plane_t;
typedef enum chroma_sampling
{
	CHROMA_SAMP_NONE,
	CHROMA_SAMP_444,
	CHROMA_SAMP_422,
	CHROMA_SAMP_411
} chroma_sampling_t;
typedef enum plane_layout
{
	PLANE_LAYOUT_COMPOSITE,
	PLANE_LAYOUT_SEPARATE
} plane_layout_t;
typedef enum sampling
{
	SAMPLING_NONE,
	SAMPLING_UPSAMPLED,
	SAMPLING_DOWNSAMPLED
} sampling_t;

// unnormalised pixel coordinates for clipping ROIs
typedef struct frame_rect
{
	struct xrt_vec2 tl;
	struct xrt_vec2 br;
} frame_rect_t;

// basic frame data structure - holds a pointer to buffer.
typedef struct frame
{
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	frame_format_t format;
	stereo_format_t stereo_format;
	uint32_t size_bytes;
	uint8_t* data;
	chroma_sampling_t chroma_sampling; // unused
	plane_layout_t plane_layout;       // unused
	uint8_t* u_data;                   // unused
	uint8_t* v_data;                   // unused
	uint64_t timestamp;
	uint64_t source_timestamp;
	uint64_t source_sequence; //sequence id
	uint64_t source_id; // used to tag frames with the source they
	                    // originated from
} frame_t;

typedef struct capture_parameters
{
	// used to configure cameras. since there is no guarantee every
	// frameserver will support any/all of these params, a 'best effort'
	// should be made to apply them. all numeric values are normalised
	// floats for broad applicability
	float gain;
	float exposure;
} capture_parameters_t;


// frameserver

typedef enum frameserver_type
{
	FRAMESERVER_TYPE_NONE,
	FRAMESERVER_TYPE_FFMPEG,
	FRAMESERVER_TYPE_UVC,
	FRAMESERVER_TYPE_V4L2
} frameserver_type_t;

// Interface types
typedef void* frameserver_internal_instance_ptr;
typedef void* frameserver_source_descriptor_ptr;
typedef void* frameserver_instance_ptr;



typedef void (*frame_consumer_callback_func)(void* instance, frame_t* frame);


// Frameserver API

typedef struct _frameserver_instance
{
	frameserver_type_t frameserver_type;
	bool (*frameserver_enumerate_sources)(
	    frameserver_instance_ptr inst,
	    frameserver_source_descriptor_ptr sources,
	    uint32_t* count);
	bool (*frameserver_configure_capture)(frameserver_instance_ptr,
	                                      capture_parameters_t cp);
	bool (*frameserver_frame_get)(frameserver_instance_ptr inst,
	                              frame_t* _frame);
	void (*frameserver_register_frame_callback)(
	    frameserver_instance_ptr inst,
	    void* target_instance,
	    frame_consumer_callback_func target_func);
	void (*frameserver_register_event_callback)(
	    frameserver_instance_ptr inst,
	    void* target_instance,
	    event_consumer_callback_func target_func);
	bool (*frameserver_seek)(frameserver_instance_ptr inst,
	                         uint64_t timestamp);
	bool (*frameserver_stream_start)(
	    frameserver_instance_ptr inst,
	    frameserver_source_descriptor_ptr source);
	bool (*frameserver_stream_stop)(frameserver_instance_ptr inst);
	bool (*frameserver_is_running)(frameserver_instance_ptr inst);
	frameserver_internal_instance_ptr internal_instance;
} frameserver_instance_t;

frameserver_instance_t*
frameserver_create(frameserver_type_t t);
bool
frameserver_destroy(frameserver_instance_t* inst);

// bool frame_data_alloc(frame_t*);
// bool frame_data_free(frame_t*);
int32_t
frame_size_in_bytes(frame_t* f);
int32_t
frame_bytes_per_pixel(frame_t* f);
float
format_bytes_per_pixel(
    frame_format_t f); // this is a float to support e.g. YUV420
bool
frame_split_stereo(frame_t* source, frame_t* left, frame_t* right);
bool
frame_extract_plane(frame_t* source, plane_t plane, frame_t* out);
bool
frame_resample(frame_t* source, frame_t* out);

bool
frameservers_test();

#ifdef __cplusplus
}
#endif

#endif // FRAMESERVER_H
