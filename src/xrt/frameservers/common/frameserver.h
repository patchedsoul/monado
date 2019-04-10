#ifndef FRAMESERVER_H
#define FRAMESERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef enum frameserver_type { FRAMESERVER_TYPE_NONE,FRAMESERVER_TYPE_FFMPEG,FRAMESERVER_TYPE_UVC,FRAMESERVER_TYPE_V4L2 } frameserver_type_t;

typedef enum frame_format {FORMAT_RAW,FORMAT_Y_UINT8,FORMAT_Y_UINT16,FORMAT_RGB_UINT8,FORMAT_BGR_UINT8,FORMAT_YUYV_UINT8,FORMAT_YUV444_UINT8,FORMAT_JPG} frame_format_t;
typedef enum stereo_format {STEREO_NONE,STEREO_SBS,STEREO_OAU} stereo_format_t;
typedef enum plane {PLANE_R,PLANE_G,PLANE_B,PLANE_Y,PLANE_U,PLANE_V} plane_t;

typedef void* frameserver_internal_instance;
typedef void* frameserver_source_descriptor;
typedef void* frameserver_instance;

typedef enum frameserver_event_type {EVENT_NONE,EVENT_FRAME} frameserver_event_type_t;

typedef struct frame
{
    uint16_t width;
    uint16_t height;
    uint16_t stride;
    frame_format_t format;
    stereo_format_t stereo_format;
    uint8_t* data;
    uint64_t timestamp;
    uint64_t src_timestamp;
} frame_t;

typedef struct _frameserver_instance {
    frameserver_type_t frameserver_type;
    bool (*frameserver_enumerate_sources)(frameserver_instance* inst, frameserver_source_descriptor sources, uint32_t* count);
    bool (*frameserver_frame_get)(frameserver_instance* inst,frame_t* _frame);
	void (*frameserver_register_event_callback)(frameserver_instance* inst, void* target_instance, void* target_func,frameserver_event_type_t event_type);
    bool (*frameserver_seek)(frameserver_instance* inst, uint64_t timestamp);
    bool (*frameserver_stream_start)(frameserver_instance* inst, frameserver_source_descriptor source);
    bool (*frameserver_stream_stop)(frameserver_instance* inst);
    bool (*frameserver_is_running)(frameserver_instance* inst);
    frameserver_internal_instance internal_instance;
    } frameserver_instance_t;

frameserver_instance_t* frameserver_create(frameserver_type_t t);
bool frameserver_destroy(frameserver_instance_t* inst);

bool frame_data_alloc(frame_t*);
int32_t frame_size_in_bytes(frame_t* fd);
int32_t frame_bytes_per_pixel(frame_t* fd);
bool split_stereo_frame(frame_t* source, frame_t* left, frame_t* right);
bool extract_plane(frame_t* source,plane_t plane,frame_t* out);

bool frameservers_test();

#endif //FRAMESERVER_H
