#include <common/frameserver.h>
#include "uvc_frameserver.h"
#include <string.h>
#include <stdlib.h>


static uvc_error_t res;

bool uvc_source_alloc(uvc_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool uvc_source_destroy(uvc_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}


bool uvc_frameserver_alloc(uvc_frameserver_instance_t* inst) {
	inst->device_list =NULL;
	res = uvc_init(&(inst->context), NULL);
	if (res < 0)
	{
		uvc_perror(res, "UVC Context init failed");
		return false;
	}
	return true;
}

bool uvc_frameserver_enumerate_sources(uvc_frameserver_instance_t* inst, uvc_source_descriptor_t* cameras, uint32_t* count)
{
	if (inst->device_list != NULL) {
		//uvc_free_device_list(inst->device_list,0);
	}
	uint32_t device_count = 0;
	res = uvc_get_device_list(inst->context, &(inst->device_list));
	if (res < 0)
	{
		printf("ERROR: %s\n",uvc_strerror(res));
		return false;
	}
	while (1)
	{
		uvc_device_t* uvc_device = inst->device_list[device_count];
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
		uvc_free_device_list(inst->device_list,1);
		return true;
	}

	//if we were passed an array of camera descriptors, fill them in
	for (uint32_t i=0;i<device_count;i++)
	{
		uvc_device_t* uvc_device = inst->device_list[i];
		uvc_device_descriptor_t* uvc_device_descriptor;
		res = uvc_get_device_descriptor(uvc_device, &uvc_device_descriptor);
		if (res < 0)
		{
			printf("ERROR: %s\n",uvc_strerror(res));
			uvc_free_device_list(inst->device_list,1);
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
		desc->device = uvc_device;
		uvc_free_device_descriptor(uvc_device_descriptor);
	}
	//we can't free the device list since the device ptrs we hand out are contained in it
	//uvc_free_device_list(device_list,1);
	return true;
}

bool uvc_frameserver_test(){
	printf("Running UVC Frameserver Test\n");
    uvc_frameserver_instance_t instance;
    if (! uvc_frameserver_alloc(&instance))
	{
		printf("FAILURE: Could not init frameserver.\n");
		return false;
	}
	uint32_t camera_count =0;
	if (! uvc_frameserver_enumerate_devices(&instance,NULL,&camera_count)) {
		printf("FAILURE: Could not get camera count.\n");
		return false;
	}
    uvc_source_descriptor_t* camera_list = calloc(camera_count,sizeof(uvc_source_descriptor_t));
    if (! uvc_frameserver_enumerate_devices(&instance, camera_list,&camera_count)) {
		printf("FAILURE: Could not get camera descriptors\n");
		return false;
	}
	for (uint32_t i=0;i<camera_count;i++)
	{
		printf("%d camera name: %s\n",i,camera_list[i].name);
	}
	return true;
}
