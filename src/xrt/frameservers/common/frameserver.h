#ifndef FRAMESERVER_H
#define FRAMESERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

//frame
typedef enum frame_format {FORMAT_RAW,FORMAT_Y_UINT8,FORMAT_Y_UINT16,FORMAT_RGB_UINT8,FORMAT_BGR_UINT8,FORMAT_YUYV_UINT8,FORMAT_YUV444_UINT8,FORMAT_YUV422_UINT8,FORMAT_YUV420_UINT8,FORMAT_JPG} frame_format_t;
typedef enum stereo_format {STEREO_NONE,STEREO_SBS,STEREO_OAU} stereo_format_t;
typedef enum plane {PLANE_R,PLANE_G,PLANE_B,PLANE_Y,PLANE_U,PLANE_V} plane_t;

typedef struct frame
{
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	frame_format_t format;
	stereo_format_t stereo_format;
	uint8_t* data;
	uint32_t size_bytes;
	uint64_t timestamp;
	uint64_t source_timestamp;
	uint64_t source_id; //used to tag frames with the source they originated from
} frame_t;

typedef struct capture_parameters{
	// used to configure cameras. since there is no guarantee every frameserver will support any/all of these
	// params, a 'best effort' should be made to apply them. all numeric values are normalised floats for broad applicability
	float gain;
	float exposure;
} capture_parameters_t;


// frameserver

typedef enum frameserver_type { FRAMESERVER_TYPE_NONE,FRAMESERVER_TYPE_FFMPEG,FRAMESERVER_TYPE_UVC,FRAMESERVER_TYPE_V4L2 } frameserver_type_t;
typedef enum frameserver_event_type {FRAMESERVER_EVENT_NONE, FRAMESERVER_EVENT_GOTFRAME} frameserver_event_type_t;

//Interface types
typedef void* frameserver_internal_instance_ptr;
typedef void* frameserver_source_descriptor_ptr;
typedef void* frameserver_instance_ptr;

typedef struct frameserver_event {
	frameserver_type_t type;
	// extra data to go along with events
	// can be added here
} frameserver_event_t;


typedef void (*frame_consumer_callback_func)(void* instance, frame_t* frame);
typedef void (*event_consumer_callback_func)(void* instance, frameserver_event_t event);

// Frameserver API

typedef struct _frameserver_instance {
    frameserver_type_t frameserver_type;
	bool (*frameserver_enumerate_sources)(frameserver_instance_ptr inst, frameserver_source_descriptor_ptr sources, uint32_t* count);
	bool (*frameserver_configure_capture)(frameserver_instance_ptr,capture_parameters_t cp);
	bool (*frameserver_frame_get)(frameserver_instance_ptr inst,frame_t* _frame);
	void (*frameserver_register_frame_callback)(frameserver_instance_ptr inst, void* target_instance, frame_consumer_callback_func target_func);
	void (*frameserver_register_event_callback)(frameserver_instance_ptr inst, void* target_instance, event_consumer_callback_func target_func);
	bool (*frameserver_seek)(frameserver_instance_ptr inst, uint64_t timestamp);
	bool (*frameserver_stream_start)(frameserver_instance_ptr inst, frameserver_source_descriptor_ptr source);
	bool (*frameserver_stream_stop)(frameserver_instance_ptr inst);
	bool (*frameserver_is_running)(frameserver_instance_ptr inst);
	frameserver_internal_instance_ptr internal_instance;
    } frameserver_instance_t;

frameserver_instance_t* frameserver_create(frameserver_type_t t);
bool frameserver_destroy(frameserver_instance_t* inst);

bool frame_data_alloc(frame_t*);
int32_t frame_size_in_bytes(frame_t* f);
int32_t frame_bytes_per_pixel(frame_t* f);
float format_bytes_per_pixel(frame_format_t f);
bool split_stereo_frame(frame_t* source, frame_t* left, frame_t* right);
bool extract_plane(frame_t* source,plane_t plane,frame_t* out);

bool frameservers_test();

#endif //FRAMESERVER_H
