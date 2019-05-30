#ifndef VL42_FRAMESERVER_H
#define Vl42_FRAMESERVER_H

#include <stdint.h>
#include <stdio.h>

#include "../common/frameserver.h"
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-common.h>

typedef struct v4l2_frameserver_instance {
    uint32_t padding;
    } v4l2_frameserver_instance_t;


//TODO: unify device descriptors across apis
typedef struct v4l2_source_descriptor {
	char device_path[256]; //TODO: might not be enough
    char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
	uint64_t source_id;
	frame_format_t format;
	uint32_t stream_format;
	sampling_t sampling;
	uint32_t width;
	uint32_t height;
	uint32_t rate;
} v4l2_source_descriptor_t;

v4l2_frameserver_instance_t* v4l2_frameserver_create(frameserver_instance_t* inst);
bool v4l2_frameserver_destroy(frameserver_instance_t* inst);
bool v4l2_frameserver_source_create(v4l2_source_descriptor_t*);
bool v4l2_frameserver_source_destroy(v4l2_source_descriptor_t*);
bool v4l2_frameserver_configure_capture(frameserver_instance_t* inst, capture_parameters_t cp);
bool v4l2_frameserver_enumerate_sources(frameserver_instance_t* inst, v4l2_source_descriptor_t* sources, uint32_t* count);
bool v4l2_frameserver_get(frameserver_instance_t* inst, frame_t* frame);
void v4l2_frameserver_register_event_callback(frameserver_instance_t* inst, void* target_instance,event_consumer_callback_func target_func);
void v4l2_frameserver_register_frame_callback(frameserver_instance_t* inst, void* target_instance,frame_consumer_callback_func target_func);
bool v4l2_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool v4l2_frameserver_stream_start(frameserver_instance_t* inst);
bool v4l2_frameserver_stream_stop(frameserver_instance_t* inst);
bool v4l2_frameserver_is_running(frameserver_instance_t* inst);
bool v4l2_frameserver_test();


#endif //V4L2_FRAMESERVER_H
