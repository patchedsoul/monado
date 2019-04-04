#ifndef UVC_FRAMESERVER_H
#define UVC_FRAMESERVER_H

#include <stdint.h>
#include <stdio.h>

#include "../common/frameserver.h"
#include <libuvc/libuvc.h>

typedef struct uvc_camera_descriptor {
	char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
	uvc_device_t* device;
	uvc_device_handle_t* device_handle;
	uvc_stream_handle_t* stream_handle;
	uvc_stream_ctrl_t stream_ctrl;
} uvc_camera_descriptor_t;

typedef struct uvc_frameserver_instance {
	uvc_device_t** device_list;
	uvc_context_t* context;
} uvc_frameserver_instance;

bool uvc_frameserver_init(uvc_frameserver_instance*);
bool uvc_frameserver_enumerate_devices(uvc_frameserver_instance*, uvc_camera_descriptor_t* cameras, uint32_t* count);
bool uvc_frameserver_test();


#endif //UVC_FRAMESERVER_H
