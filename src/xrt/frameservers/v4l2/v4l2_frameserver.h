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

typedef struct v4l2_source_descriptor {
    char* device_path;
    char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
} v4l2_source_descriptor_t;


bool v4l2_frameserver_alloc(v4l2_frameserver_instance_t*);
bool v4l2_frameserver_destroy(v4l2_frameserver_instance_t*);
bool v4l2_source_alloc(v4l2_source_descriptor_t*);
bool v4l2_source_destroy(v4l2_source_descriptor_t*);
bool v4l2_frameserver_enumerate_sources(v4l2_frameserver_instance_t*, v4l2_source_descriptor_t* cameras, uint32_t* count);
bool v4l2_frame_get(frame_t* _frame);
void v4l2_register_callback(void* func);
bool v4l2_stream_start();
bool v4l2_stream_stop();
bool v4l2_is_running();
bool v4l2_frameserver_test();


#endif //V4L2_FRAMESERVER_H
