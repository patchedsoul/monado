#include <common/frameserver.h>
#include "uvc_frameserver.h"
#include <string.h>
#include <stdlib.h>


static uvc_error_t res;

bool uvc_source_create(uvc_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool uvc_source_destroy(uvc_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}


uvc_frameserver_instance_t* uvc_frameserver_create(frameserver_instance_t* inst) {
    //TODO: calloc macro
    uvc_frameserver_instance_t* i = calloc(1,sizeof(uvc_frameserver_instance_t));
    if (i) {
        i->device_list =NULL;
        res = uvc_init(&(i->context), NULL);
        if (res < 0)
        {
            uvc_perror(res, "UVC Context init failed");
            return NULL;
        }
        return i;
    }

    return NULL;
}

bool uvc_frameserver_enumerate_sources(frameserver_instance_t* inst, uvc_source_descriptor_t* cameras, uint32_t* count)
{
	uvc_error_t res;
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	if (internal->device_list != NULL) {
		//uvc_free_device_list(inst->device_list,0);
	}
	uint32_t device_count = 0;
	res = uvc_get_device_list(internal->context, &(internal->device_list));
	if (res < 0)
	{
		printf("ERROR: %s\n",uvc_strerror(res));
		return false;
	}
	while (1)
	{
		uvc_device_t* uvc_device = internal->device_list[device_count];
		if (uvc_device == NULL)
		{
			break;
		}
		device_count++;
	}
	if (cameras == NULL)
	{
		//just return our count
		*count = device_count;
		uvc_free_device_list(internal->device_list,1);
		return true;
	}

	//if we were passed an array of camera descriptors, fill them in
	for (uint32_t i=0;i<device_count;i++)
	{
		uvc_device_t* uvc_device = internal->device_list[i];
		uvc_device_descriptor_t* uvc_device_descriptor;
		res = uvc_get_device_descriptor(uvc_device, &uvc_device_descriptor);
		if (res < 0)
		{
			printf("ERROR: %s\n",uvc_strerror(res));
			uvc_free_device_list(internal->device_list,1);
			return false;
		}
        uvc_source_descriptor_t* desc = &(cameras[i]);
		//TODO: check lengths
		snprintf(desc->name,128,"%s %s %s %04x:%04x",uvc_device_descriptor->manufacturer,uvc_device_descriptor->product,uvc_device_descriptor->serialNumber,uvc_device_descriptor->idProduct,uvc_device_descriptor->idVendor);
		desc->name[127]=0;
		desc->product_id = uvc_device_descriptor->idProduct;
		desc->vendor_id = uvc_device_descriptor->idVendor;
		//TODO check lengths
		if (uvc_device_descriptor->serialNumber){
			memcpy(desc->serial,uvc_device_descriptor->serialNumber,strlen(uvc_device_descriptor->serialNumber)+1);
		}
		desc->serial[127]=0;
		//desc->device = uvc_device;
		res = uvc_open(uvc_device,&internal->device_handle);
		if (res == UVC_SUCCESS)
		{
			const uvc_format_desc_t* format_desc = uvc_get_format_descs(internal->device_handle);
			while(format_desc->next != NULL)
			{
				printf("Found format: %d\n",format_desc->bFormatIndex);
				uvc_frame_desc_t* frame_desc = format_desc->frame_descs;
				while (frame_desc->next != NULL)
				{
					printf("W %d H %d\n",frame_desc->wWidth,frame_desc->wHeight);
					frame_desc=frame_desc->next;
				}
				format_desc=format_desc->next;
			}
			uvc_close(&internal->device_handle);
		}
		uvc_free_device_descriptor(uvc_device_descriptor);
	}
	//we can't free the device list since the device ptrs we hand out are contained in it
	//uvc_free_device_list(device_list,1);
	return true;
}


bool uvc_frameserver_configure_capture(frameserver_instance_t* inst, capture_parameters_t cp) {
	return true;
}

void uvc_frameserver_register_frame_callback(frameserver_instance_t* inst, void* target_instance, frame_consumer_callback_func target_func) {
	//do nothing
}

void uvc_frameserver_register_event_callback(frameserver_instance_t* inst, void* target_instance, event_consumer_callback_func target_func) {
	//do nothing
}

bool uvc_frameserver_get(frameserver_instance_t* inst, frame_t* _frame) {
	return false;
}
bool uvc_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp) {
	return false;
}
bool uvc_frameserver_stream_start(frameserver_instance_t* inst) {
	return false;
}
bool uvc_frameserver_stream_stop(frameserver_instance_t* inst) {
	return false;
}
bool uvc_frameserver_is_running(frameserver_instance_t* inst) {
	return false;
}


bool uvc_frameserver_test(){
	printf("Running UVC Frameserver Test\n");
	frameserver_instance_t* uvc_frameserver = frameserver_create(FRAMESERVER_TYPE_UVC);
	if (!uvc_frameserver )
	{
		printf("FAILURE: Could not create frameserver.\n");
		return false;
	}
	uint32_t camera_count =0;
	if (! uvc_frameserver->frameserver_enumerate_sources(uvc_frameserver,NULL,&camera_count)) {
		printf("FAILURE: Could not get camera count.\n");
		return false;
	}
    uvc_source_descriptor_t* camera_list = calloc(camera_count,sizeof(uvc_source_descriptor_t));
	if (! uvc_frameserver->frameserver_enumerate_sources(uvc_frameserver, camera_list,&camera_count)) {
		printf("FAILURE: Could not get camera descriptors\n");
		return false;
	}
	for (uint32_t i=0;i<camera_count;i++)
	{
		printf("%d camera name: %s\n",i,camera_list[i].name);
	}
	return true;
}
