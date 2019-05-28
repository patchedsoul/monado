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
#include <pthread.h>

//we need this to do a bit of hackery with multiple opens/closes
struct uvc_context {
  /** Underlying context for USB communication */
  struct libusb_context *usb_ctx;
  /** True iff libuvc initialized the underlying USB context */
  uint8_t own_usb_ctx;
  /** List of open devices in this context */
  uvc_device_handle_t *open_devices;
  pthread_t handler_thread;
  int kill_handler_thread;
};

typedef struct uvc_source_descriptor {
	char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
	uint64_t source_id;
	uint32_t uvc_device_index;
	enum uvc_frame_format stream_format;
	frame_format_t format;
    sampling_t sampling;
	uint32_t width;
	uint32_t height;
	uint32_t rate;
} uvc_source_descriptor_t;

typedef struct uvc_frameserver_instance {
    uvc_device_t** device_list;
    uvc_context_t* context;
	uvc_device_t* device;
	uvc_device_handle_t* device_handle;
	uvc_stream_handle_t* stream_handle;
	uvc_stream_ctrl_t stream_ctrl;
	frame_consumer_callback_func frame_target_callback;
	event_consumer_callback_func event_target_callback;
	void* frame_target_instance; //where we send our frames
	void* event_target_instance; //where we send our events
	uvc_source_descriptor_t source_descriptor;
	pthread_t stream_thread;
	capture_parameters_t capture_params;
	bool is_configured;
	bool is_running;
} uvc_frameserver_instance_t;



uvc_frameserver_instance_t* uvc_frameserver_create(frameserver_instance_t* inst);
bool uvc_frameserver_destroy(frameserver_instance_t* inst);
bool uvc_source_alloc(uvc_source_descriptor_t* desc);
bool uvc_source_destroy(uvc_source_descriptor_t* desc);
bool uvc_frameserver_configure_capture(frameserver_instance_t* inst, capture_parameters_t cp);
bool uvc_frameserver_enumerate_sources(frameserver_instance_t*, uvc_source_descriptor_t* sources, uint32_t* count);
bool uvc_frameserver_get(frameserver_instance_t* inst, frame_t* _frame);
void uvc_frameserver_register_event_callback(frameserver_instance_t* inst, void* target_instance,event_consumer_callback_func target_func);
void uvc_frameserver_register_frame_callback(frameserver_instance_t* inst, void* target_instance,frame_consumer_callback_func target_func);
bool uvc_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp);
bool uvc_frameserver_stream_start(frameserver_instance_t* inst,uvc_source_descriptor_t* source);
bool uvc_frameserver_stream_stop(frameserver_instance_t* inst);
bool uvc_frameserver_is_running(frameserver_instance_t* inst);
bool uvc_frameserver_test();

static void uvc_frameserver_stream_run(frameserver_instance_t* inst);  //streaming thread entrypoint
static uint32_t  uvc_frameserver_get_source_descriptors(uvc_source_descriptor_t** sds,uvc_device_t* device,uint32_t uvc_device_index);
static bool source_descriptor_from_uvc_descriptor(uvc_source_descriptor_t* source_descriptor, uvc_device_descriptor_t* uvc_device_descriptor, uvc_frame_desc_t* uvc_frame_descriptor);
#endif //UVC_FRAMESERVER_H
