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
	printf("creating uvc frameserver\n");
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
	//if (internal->device_list != NULL) {
	//	uvc_free_device_list(internal->device_list,0);
	//}
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

	uint32_t source_count=0;

	if (cameras == NULL)
	{
		printf("counting formats\n");
		for (uint32_t i=0;i<device_count;i++){
		uvc_source_descriptor_t* temp_sds_count= NULL;
		//we need to free the source descriptors, even though we only use the count
		uint32_t c = uvc_frameserver_get_source_descriptors(&temp_sds_count,internal->device_list[i],i);
		source_count += c;
		free(temp_sds_count);
		}

		*count = source_count;
		//uvc_free_device_list(internal->device_list,1);
		//internal->device_list = NULL;
		return true;
	}

	printf("returning formats\n");

	//if we were passed an array of camera descriptors, fill them in
	uvc_source_descriptor_t* temp_sds=NULL;

	uint32_t cameras_offset=0;
	for (uint32_t i=0;i<device_count;i++)
	{
		uint32_t c = uvc_frameserver_get_source_descriptors(&temp_sds,internal->device_list[i],i);
		printf("Got %d sources\n",c);
		if (c > 0) {
		source_count +=c;
		memcpy(cameras+cameras_offset,temp_sds,source_count * sizeof(uvc_source_descriptor_t));
		cameras_offset+=c;
		}
	}

	if (source_count==0)
	{
		return false;
	}

	free(temp_sds);
	//uvc_free_device_list(internal->device_list,1);
	//internal->device_list = NULL;
	return true;
}


bool uvc_frameserver_configure_capture(frameserver_instance_t* inst, capture_parameters_t cp) {
	return true;
}

void uvc_frameserver_register_frame_callback(frameserver_instance_t* inst, void* target_instance, frame_consumer_callback_func target_func) {
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	internal->frame_target_instance = target_instance;
	internal->frame_target_callback = target_func;
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
bool uvc_frameserver_stream_start(frameserver_instance_t* inst,uvc_source_descriptor_t* source) {

	uvc_frameserver_instance_t* internal = inst->internal_instance;
	internal->source_descriptor = *source;
	internal->is_running = true;
	if(pthread_create(&internal->stream_thread, NULL, uvc_frameserver_stream_run, inst)) {
	printf("ERROR: could not create thread\n");
	return false;
	}
	//we're off to the races!
	return true;
}
bool uvc_frameserver_stream_stop(frameserver_instance_t* inst) {
	return false;
}
bool uvc_frameserver_is_running(frameserver_instance_t* inst) {
	return false;
}


void uvc_frameserver_stream_run(frameserver_instance_t* inst)
{
	uvc_error_t res;
	uvc_frameserver_instance_t* internal = inst->internal_instance;

	enum uvc_frame_format uvc_format = UVC_FRAME_FORMAT_UNKNOWN;
	float uvc_bytes_per_pixel = 3.0; //assume 8 bit 444
	// convert our 'internal' format into a uvc format.
	// we only deal with YUYV format right now, for lowest latency, but this
	// will need work later
	switch (internal->source_descriptor.format){
	case FORMAT_YUYV_UINT8:
	case FORMAT_Y_UINT8:
		uvc_format = UVC_FRAME_FORMAT_YUYV;
		uvc_bytes_per_pixel=2.0;
		break;
	default:
		printf("Could not map source format to uvc format");
		uvc_format = UVC_FRAME_FORMAT_ANY;
	}
	//clear our kill_handler_thread flag, likely set when closing
	//devices during format enumeration
	internal->context->kill_handler_thread = 0;

	res = uvc_open(internal->device_list[internal->source_descriptor.uvc_device_index],&internal->device_handle);
	if (res < 0)
	{
		printf("ERROR: %s open %s\n",&internal->source_descriptor.name,uvc_strerror(res));
		return;
	}
	res = uvc_get_stream_ctrl_format_size(internal->device_handle, &internal->stream_ctrl,uvc_format,internal->source_descriptor.width, internal->source_descriptor.height,internal->source_descriptor.rate);
	if (res < 0)
	{
		printf("ERROR: %s get_stream_ctrl_format %s\n",&internal->source_descriptor.name,uvc_strerror(res));
		return;
	}


	uvc_print_stream_ctrl(&internal->stream_ctrl, stdout);

	//defaults
	uvc_set_ae_mode(internal->device_handle, 1);
	uvc_set_ae_priority(internal->device_handle,0);
	uvc_set_exposure_abs(internal->device_handle,10);
	uvc_set_gain(internal->device_handle,0);


	res = uvc_stream_open_ctrl(internal->device_handle, &internal->stream_handle, &internal->stream_ctrl);
	if (res < 0)
	{
		printf("ERROR: stream_open_ctrl %s\n",uvc_strerror(res));
		return;
	}

	res = uvc_stream_start(internal->stream_handle,NULL,NULL,0);
	if (res < 0)
	{
		printf("ERROR: stream_start %s\n",uvc_strerror(res));
		return;
	}


	int wat = internal->source_descriptor.width * internal->source_descriptor.height * uvc_bytes_per_pixel;
	frame_t f;
	uvc_frame_t* frame = uvc_allocate_frame(wat);
	{
		while (internal->is_running)
		{
			res =  uvc_stream_get_frame	(internal->stream_handle, &frame,0);
			if (res < 0)
			{
				printf("ERROR: stream_get_frame %s\n",uvc_strerror(res));
			} else {
				if (frame){
					printf("got frame\n");
					f.source_id = internal->source_descriptor.source_id;
					f.format = internal->source_descriptor.format;
					f.width = frame->width;
					f.height = frame->height;
					f.stride=frame->step;
					f.size_bytes = frame_size_in_bytes(&f);

					// since we are just PoCing, we can just pass the
					// Y plane.
					f.data = frame->data;
					if (internal->frame_target_callback){
					internal->frame_target_callback(internal->frame_target_instance,&f);
					}
					frameserver_event_t e ={};
					e.type = FRAMESERVER_EVENT_GOTFRAME;
					if (internal->event_target_callback){
						internal->event_target_callback(internal->event_target_instance,e);
					}
				}
			}
		}
		uvc_free_frame(frame);
	}


	return;

}


bool uvc_frameserver_test(){
	printf("Running UVC Frameserver Test\n");
	frameserver_instance_t* uvc_frameserver = frameserver_create(FRAMESERVER_TYPE_UVC);
	if (!uvc_frameserver )
	{
		printf("FAILURE: Could not create frameserver.\n");
		return false;
	}
	uint32_t source_count =0;
	if (! uvc_frameserver->frameserver_enumerate_sources(uvc_frameserver,NULL,&source_count)) {
		printf("FAILURE: Could not get source count.\n");
		return false;
	}
	uvc_source_descriptor_t* source_list = calloc(source_count,sizeof(uvc_source_descriptor_t));
	if (! uvc_frameserver->frameserver_enumerate_sources(uvc_frameserver, source_list,&source_count)) {
		printf("FAILURE: Could not get source descriptors\n");
		return false;
	}
	for (uint32_t i=0;i<source_count;i++)
	{
		printf("%d source name: %s\n",i,source_list[i].name);
	}
	return true;
}

uint32_t uvc_frameserver_get_source_descriptors(uvc_source_descriptor_t** sds,uvc_device_t* uvc_device, uint32_t device_index) {

	uint32_t sd_count=0;
	uvc_device_descriptor_t* uvc_device_descriptor;
	res = uvc_get_device_descriptor(uvc_device, &uvc_device_descriptor);
	if (res < 0) {
		printf("ERROR: %s\n",uvc_strerror(res));
	}
	uvc_device_handle_t* temp_handle;
	res = uvc_open(uvc_device,&temp_handle);
	if (res == UVC_SUCCESS)
	{
		const uvc_format_desc_t* format_desc = uvc_get_format_descs(temp_handle);
		uvc_source_descriptor_t* desc = *sds;
		while(format_desc  != NULL)
		{
			printf("Found format: %d FOURCC %c%c%c%c\n",format_desc->bFormatIndex,format_desc->fourccFormat[0],format_desc->fourccFormat[1],format_desc->fourccFormat[2],format_desc->fourccFormat[3]);
			uvc_frame_desc_t* frame_desc = format_desc->frame_descs;
			while (frame_desc != NULL)
			{
				printf("W %d H %d\n",frame_desc->wWidth,frame_desc->wHeight);
				uint32_t* frame_duration = frame_desc->intervals;
				while (*frame_duration != 0) {
					printf("rate: %d %f\n",*frame_duration,1.0/(*frame_duration / 10000000.0f));
					if (*frame_duration < 400000) { //anything quicker than 25fps
						// if we are a YUV mode, write out a descriptor.
						// also write out the Y-only descriptor
						if (format_desc->fourccFormat[0] == 'Y') {
							uvc_source_descriptor_t* temp_alloc = NULL;
							if (*sds == NULL || sd_count == 0) {
								temp_alloc= calloc(2,sizeof(uvc_source_descriptor_t));
							} else {
								temp_alloc = realloc(*sds,sd_count * 2 * sizeof(uvc_source_descriptor_t));
							}
							if (! temp_alloc) {
								printf("ERROR: could not allocate memory");
								exit(1);
							}
							*sds = temp_alloc;
							desc = temp_alloc + sd_count;

							snprintf(desc->name,128,"%s %s %s %04x:%04x",uvc_device_descriptor->manufacturer,uvc_device_descriptor->product,uvc_device_descriptor->serialNumber,uvc_device_descriptor->idProduct,uvc_device_descriptor->idVendor);
							desc->uvc_device_index=device_index;
							desc->name[127]=0;
							desc->product_id = uvc_device_descriptor->idProduct;
							desc->vendor_id = uvc_device_descriptor->idVendor;
							//TODO check lengths
							if (uvc_device_descriptor->serialNumber){
								memcpy(desc->serial,uvc_device_descriptor->serialNumber,strlen(uvc_device_descriptor->serialNumber)+1);
							}
							desc->serial[127]=0;
							desc->format = FORMAT_YUYV_UINT8;
							desc->width = frame_desc->wWidth;
							desc->height = frame_desc->wHeight;

							sd_count++;
							desc++;
							//also output our 'one plane Y' format
							snprintf(desc->name,128,"%s %s %s %04x:%04x",uvc_device_descriptor->manufacturer,uvc_device_descriptor->product,uvc_device_descriptor->serialNumber,uvc_device_descriptor->idProduct,uvc_device_descriptor->idVendor);
							desc->uvc_device_index=device_index;
							desc->name[127]=0;
							desc->product_id = uvc_device_descriptor->idProduct;
							desc->vendor_id = uvc_device_descriptor->idVendor;
							//TODO check lengths
							if (uvc_device_descriptor->serialNumber){
								memcpy(desc->serial,uvc_device_descriptor->serialNumber,strlen(uvc_device_descriptor->serialNumber)+1);
							}
							desc->serial[127]=0;
							desc->format = FORMAT_Y_UINT8;
							desc->width = frame_desc->wWidth;
							desc->height = frame_desc->wHeight;
							sd_count++;
							//desc++;
						}
					}
					frame_duration++;
				}
				frame_desc=frame_desc->next;
			}
			format_desc=format_desc->next;
		}
		uvc_close(temp_handle);
	}
	//this crashes - i guess we only care about closing if we have started streaming
	//
	//uvc_free_device_descriptor(uvc_device_descriptor);
	printf("RETURNING %d\n",sd_count);
	return sd_count;
}

