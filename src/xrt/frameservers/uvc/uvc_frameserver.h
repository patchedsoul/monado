#ifndef UVC_FRAMESERVER_H
#define UVC_FRAMESERVER_H

/* Almost all of the ground covered here would be covered
 * by the v4l2 frameserver on linux, but uvc may be the
 * simplest approach for cross-platform e.g. OS X
*/

#include <stdint.h>
#include <stdio.h>

#include "../common/frameserver.h"
#include <libuvc/libuvc.h>

typedef struct uvc_frameserver_instance {
    uvc_device_t** device_list;
    uvc_context_t* context;
} uvc_frameserver_instance_t;

typedef struct uvc_source_descriptor {
	char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
	uvc_device_t* device;
	uvc_device_handle_t* device_handle;
	uvc_stream_handle_t* stream_handle;
	uvc_stream_ctrl_t stream_ctrl;
} uvc_source_descriptor_t;


uvc_frameserver_instance_t* uvc_frameserver_create(frameserver_instance_t* inst);
bool uvc_frameserver_destroy(uvc_frameserver_instance_t* inst);
bool uvc_source_alloc(uvc_source_descriptor_t* desc);
bool uvc_source_destroy(uvc_source_descriptor_t* desc);
bool uvc_frameserver_enumerate_sources(uvc_frameserver_instance_t*, uvc_source_descriptor_t* cameras, uint32_t* count);
bool uvc_frameserver_get(frameserver_instance_t* inst, frame_t* _frame);
void uvc_frameserver_register_event_callback(frameserver_instance_t* inst, void* target_instance,void* target_func,frameserver_event_type_t event_type);
bool uvc_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool uvc_frameserver_stream_start(frameserver_instance_t* inst);
bool uvc_frameserver_stream_stop(frameserver_instance_t* inst);
bool uvc_frameserver_is_running(frameserver_instance_t* inst);
bool uvc_frameserver_test();


#endif //UVC_FRAMESERVER_H
