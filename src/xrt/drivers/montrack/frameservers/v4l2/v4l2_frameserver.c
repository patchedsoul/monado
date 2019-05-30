#include <common/frameserver.h>
#include "v4l2_frameserver.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "errno.h"
#include "jpeglib.h"
uint32_t v4l2_frameserver_get_source_descriptors(v4l2_source_descriptor_t** sds,char* v4l2_device, uint32_t device_index);

bool v4l2_source_create(v4l2_source_descriptor_t* desc)
{
   // do nothing right now
    return true;
}

bool v4l2_source_destroy(v4l2_source_descriptor_t* desc) {
    if (desc->device_path) {
        free (desc->device_path);
    }
    return true;
}

v4l2_frameserver_instance_t* v4l2_frameserver_create(frameserver_instance_t* inst) {
    v4l2_frameserver_instance_t* i = calloc(sizeof(v4l2_frameserver_instance_t),1);
    if (i) {
        return i;
    }
    return false;
}
bool v4l2_frameserver_destroy(frameserver_instance_t* inst) {
	if (inst->internal_instance){
		free(inst->internal_instance);
		return true;
	}
	return false;
}

bool v4l2_frameserver_enumerate_sources(frameserver_instance_t* inst, v4l2_source_descriptor_t* sources, uint32_t* count)
{
	v4l2_frameserver_instance_t* internal = inst->internal_instance;

	char device_files[64][256]; // max of 64 video4linux devices supported TODO: maybe 256 too small
	char* base_path="/dev/v4l/by-id"; //TODO: does this path work everywhere?
	DIR* dir;
	struct dirent* dentry;

	dir = opendir (base_path);
	if (! dir) {
	printf("ERROR: could not open %s\n",base_path);
	return false;
	}

	uint32_t device_count =0;
	while ((dentry = readdir (dir)) != NULL ) {
		if(strcmp(dentry->d_name,".") !=0 && strcmp(dentry->d_name,"..") !=0){
		snprintf (device_files[device_count],256,"%s/%s", base_path, dentry->d_name); //TODO: hardcoded 256
		device_count++;
		}
	}
	closedir (dir);

	uint32_t source_count = 0;

	if (sources == NULL)
	{
		for (uint32_t i=0;i<device_count;i++){
			v4l2_source_descriptor_t* temp_sds_count= NULL;
			uint32_t c = v4l2_frameserver_get_source_descriptors(&temp_sds_count,device_files[i],i);
			source_count+=c;
		}
		*count = source_count;
		printf("counting available source descriptors - %d\n",source_count);
		return true;
	}

	//our caller should now have alloced the array of source descriptors, fill them out

	for (uint32_t i=0;i<device_count;i++){
		v4l2_source_descriptor_t* device_sources = sources+source_count;
		uint32_t c = v4l2_frameserver_get_source_descriptors(&device_sources,device_files[i],i);
		source_count += c;
	}
	*count = source_count;

	return true;
}

bool v4l2_frameserver_configure_capture(frameserver_instance_t* inst, capture_parameters_t cp) {
	return true;
}

void v4l2_frameserver_register_event_callback(frameserver_instance_t* inst, void* target_instance,event_consumer_callback_func target_func)
{
	//do nothing
}


void v4l2_frameserver_register_frame_callback(frameserver_instance_t* inst, void* target_instance,frame_consumer_callback_func target_func)
{
	//do nothing
}


bool v4l2_frameserver_get(frameserver_instance_t* inst, frame_t* frame) {
	return false;
}

bool v4l2_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp) {
//do nothing
return false;
}

bool v4l2_frameserver_stream_start(frameserver_instance_t* inst, v4l2_source_descriptor_t* source){
	v4l2_frameserver_instance_t* internal = inst->internal_instance;
	internal->source_descriptor = *source;
	internal->is_running = true;
	if(pthread_create(&internal->stream_thread, NULL, v4l2_frameserver_stream_run, inst)) {
	printf("ERROR: could not create thread\n");
	return false;
	}
	//we're off to the races!
	return true;
}


void v4l2_frameserver_stream_run(frameserver_instance_t* inst) {
	v4l2_frameserver_instance_t* internal = inst->internal_instance;
	//our jpeg decoder stuff
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	int fd = open(internal->source_descriptor.device_path, O_RDWR, 0);
	if (fd== -1) {
		printf("ERROR Cannot open '%s %d %s\n",internal->source_descriptor.device_path,errno, strerror(errno));
		return;
	}

	/*res = uvc_open(internal->device_list[internal->source_descriptor.uvc_device_index],&internal->device_handle);
	if (res < 0)
	{
		printf("ERROR: %s open %s\n",&internal->source_descriptor.name,uvc_strerror(res));
		return;
	}
	int fps = 1.0 / (internal->source_descriptor.rate/10000000.0f);
	res = uvc_get_stream_ctrl_format_size(internal->device_handle, &internal->stream_ctrl,internal->source_descriptor.stream_format,internal->source_descriptor.width, internal->source_descriptor.height,fps);
	if (res < 0)
	{
		printf("ERROR: %s get_stream_ctrl_format %s\n",&internal->source_descriptor.name,uvc_strerror(res));
		return;
	}

	uvc_print_stream_ctrl(&internal->stream_ctrl, stdout);

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

	frame_t f = {}; //our buffer
	f.source_id = internal->source_descriptor.source_id;
	switch (internal->source_descriptor.stream_format) {
		case UVC_FRAME_FORMAT_YUYV:
			f.format = FORMAT_YUYV_UINT8;
			break;
		case UVC_FRAME_FORMAT_MJPEG:
			f.format = FORMAT_JPG; //this will get reset to YUV444
			cinfo.err = jpeg_std_error(&jerr);
			jpeg_create_decompress(&cinfo);
			break;
		default:
			printf("ERROR: unhandled format!\n");
	}

	frame_t sampled_frame = {};

	//replaced by sampled_frame but may be useful for planar output.
	frame_t plane_frame = {};
	uint8_t* plane_data[MAX_PLANES];

	uint8_t* temp_data = NULL;
	uint8_t* data_ptr = NULL;

	uvc_frame_t* frame = uvc_allocate_frame(internal->stream_ctrl.dwMaxVideoFrameSize);
	while (internal->is_running) {

		//if our config is invalidated at runtime, reconfigure
		if (! internal->is_configured) {
			//defaults - auto-anything off
			uvc_set_ae_mode(internal->device_handle, 1);
			uvc_set_ae_priority(internal->device_handle,0);
			//we may need to enumerate the control range..
			uvc_set_exposure_abs(internal->device_handle,internal->capture_params.exposure * 2048);
			uvc_set_gain(internal->device_handle,internal->capture_params.gain * 10);
			internal->is_configured = true;
		}

		res =  uvc_stream_get_frame	(internal->stream_handle, &frame,0);
		if (res < 0) {
			printf("ERROR: stream_get_frame %s\n",uvc_strerror(res));
		} else {
			if (frame) {
				//printf("got frame\n");

				f.width = frame->width;
				f.height = frame->height;
				f.stride= frame->step;
				f.size_bytes = frame->data_bytes;
				f.data = frame->data;

				switch (internal->source_descriptor.stream_format) {
					case UVC_FRAME_FORMAT_MJPEG:
						//immediately set this to YUV444 as this is what we decode to.
						f.format = FORMAT_YUV444_UINT8;
						f.stride = f.width * 3; //jpg format does not supply stride
						//decode our jpg frame.
						if (! temp_data) {
							temp_data=malloc(frame_size_in_bytes(&f));
						}
						jpeg_mem_src(&cinfo,frame->data,frame->data_bytes);
						jpeg_read_header(&cinfo, TRUE);
						//we will bypass colour conversion as we want YUV
						cinfo.out_color_space = cinfo.jpeg_color_space;
						jpeg_start_decompress(&cinfo);
						uint32_t scanlines_read = 0;
						data_ptr=temp_data;
						while (scanlines_read < cinfo.image_height ) {
							 int read_count = jpeg_read_scanlines(&cinfo,&data_ptr,16);
							data_ptr += read_count * frame->width*3;
							scanlines_read += read_count;
						}
						f.data = temp_data;
						jpeg_finish_decompress(&cinfo);

						switch (internal->source_descriptor.format) {
							case FORMAT_Y_UINT8:
								//split our Y plane out
								sampled_frame = f; //copy our buffer frames attributes
								sampled_frame.format = FORMAT_Y_UINT8;
								sampled_frame.stride = f.width;
								sampled_frame.size_bytes = frame_size_in_bytes(&sampled_frame);

								if (! sampled_frame.data) {
									sampled_frame.data = malloc(sampled_frame.size_bytes);
								}

								frame_extract_plane(&f,PLANE_Y,&sampled_frame);

								if (internal->frame_target_callback){
									internal->frame_target_callback(internal->frame_target_instance,&sampled_frame);
								}
								break;
							default:
								//supply our YUV444 directly
								if (internal->frame_target_callback){
									internal->frame_target_callback(internal->frame_target_instance,&f);
								}
						}
						break;
					case UVC_FRAME_FORMAT_YUYV:
						switch (internal->source_descriptor.format) {
							case FORMAT_Y_UINT8:
								//split our Y plane out
								sampled_frame = f; //copy our buffer frames attributes
								sampled_frame.format = FORMAT_Y_UINT8;
								sampled_frame.stride = f.width;
								sampled_frame.size_bytes = frame_size_in_bytes(&sampled_frame);

								if (! sampled_frame.data) {
									sampled_frame.data = malloc(sampled_frame.size_bytes);
								}

								frame_extract_plane(&f,PLANE_Y,&sampled_frame);

								if (internal->frame_target_callback){
									internal->frame_target_callback(internal->frame_target_instance,&sampled_frame);
								}
								break;
							case FORMAT_YUV444_UINT8:
								//upsample our YUYV to YUV444
								sampled_frame = f; //copy our buffer frames attributes
								sampled_frame.format = FORMAT_YUV444_UINT8;
								sampled_frame.stride = f.width * 3;
								sampled_frame.size_bytes = frame_size_in_bytes(&sampled_frame);
								//allocate on first access
								if (! sampled_frame.data) {
									sampled_frame.data = malloc(sampled_frame.size_bytes);
								}
								if (frame_resample(&f,&sampled_frame)) {
									if (internal->frame_target_callback) {
										internal->frame_target_callback(internal->frame_target_instance,&sampled_frame);
									}
								break;
								}
								printf("ERROR: could not resample frame from %d to %d\n",f.format,sampled_frame.format);
								break;
							default:
								//supply our YUYV directly
								if (internal->frame_target_callback){
									internal->frame_target_callback(internal->frame_target_instance,&f);
								}
							}
							break;
						default:
							printf("ERROR: Unknown stream format\n");
					}
				driver_event_t e ={};
				e.type =EVENT_FRAMESERVER_GOTFRAME;
				if (internal->event_target_callback){
					internal->event_target_callback(internal->event_target_instance,e);
				}
			}
		}
	}
	uvc_free_frame(frame);
	if (temp_data){
		free(temp_data);
		temp_data=NULL;
	}
	if (sampled_frame.data) {
		free (sampled_frame.data);
	}
	return;*/


	while(1) {
		printf("running v4l2 capture thread\n");
		sleep(1);
	}
}

bool v4l2_frameserver_stream_stop(frameserver_instance_t* inst){
	return false;
}


bool v4l2_frameserver_is_running(frameserver_instance_t* inst) {
//do nothing
return false;
}

uint32_t v4l2_frameserver_get_source_descriptors(v4l2_source_descriptor_t** sds,char* v4l2_device, uint32_t device_index) {

	uint32_t sd_count=0;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	//open the device, and check if is a video source

	int fd = open(v4l2_device, O_RDWR, 0);
	if (fd== -1) {
		printf("ERROR Cannot open '%s %d %s\n",v4l2_device,errno, strerror(errno));
		return 0;
	}

	if (ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
			return 0; //not a video device
		}
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			return 0; //cannot stream
		}
		if (!(cap.capabilities & V4L2_CAP_TIMEPERFRAME)) {
			printf("WARNING: device does not select setting frame intervals\n");
		}
	}

	struct v4l2_fmtdesc desc;
	memset(&desc,0,sizeof(desc));
	desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v4l2_source_descriptor_t* descriptor = *sds;
	while (ioctl(fd,VIDIOC_ENUM_FMT,&desc) == 0) {
		//printf("FORMAT: %s %04x\n", desc.description,desc.pixelformat);
		struct v4l2_frmsizeenum frame_size;
		struct v4l2_frmivalenum frame_interval;
		memset(&frame_size,0,sizeof(frame_size));
		memset(&frame_size,0,sizeof(frame_interval));
		frame_size.pixel_format = desc.pixelformat;
		frame_size.index = 0;
		while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frame_size) >= 0) {
			if (frame_size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
				    //printf("%dx%d\n", frame_size.discrete.width,frame_size.discrete.height);
				    frame_interval.pixel_format = frame_size.pixel_format;
					frame_interval.width = frame_size.discrete.width;
					frame_interval.height = frame_size.discrete.height;
					frame_interval.index=0;
					while(ioctl(fd,VIDIOC_ENUM_FRAMEINTERVALS,&frame_interval) >=0) {
						float fps = frame_interval.discrete.denominator / frame_interval.discrete.numerator;
						uint32_t rate = (frame_interval.discrete.numerator /  (float)frame_interval.discrete.denominator ) * 10000000;
						printf("FPS: %f %d\n", fps,rate);

						if (*sds) {
							//only fill in this struct if we were not passed NULL

							switch(desc.pixelformat){
							case 0x56595559: //YUYV stream format
								descriptor->format = FORMAT_YUYV_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								strncpy(descriptor->name,cap.driver,128);
								descriptor->device_path[127]=0x0;
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_NONE;
								descriptor++;

								descriptor->format = FORMAT_YUV444_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								strncpy(descriptor->name,cap.driver,128);
								descriptor->device_path[127]=0x0;
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_UPSAMPLED;
								descriptor++;

								descriptor->format = FORMAT_Y_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								strncpy(descriptor->name,cap.driver,128);
								descriptor->device_path[127]=0x0;
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_DOWNSAMPLED;
								descriptor++;
								sd_count += 3;
								break;
							case 0x47504a4d: //MJPEG stream format
								descriptor->format = FORMAT_YUV444_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								strncpy(descriptor->name,cap.driver,128);
								descriptor->device_path[127]=0x0;
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_UPSAMPLED;
								descriptor++;

								descriptor->format = FORMAT_Y_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								strncpy(descriptor->name,cap.driver,128);
								descriptor->device_path[127]=0x0;
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_DOWNSAMPLED;
								descriptor++;
								sd_count +=2;
								break;
							default:
								printf("ERROR: unknown pixelformat encountered\n");
							}
						} else {
							//we just need the count of the sources we would create
							switch(desc.pixelformat){
							case 0x56595559: //YUYV stream format
								sd_count += 3; //YUYV, YUV444, Y
								break;
							case 0x47504a4d: //MJPEG  stream format
								sd_count += 2; //YUV444,Y
								break;
							default:
								printf("ERROR: unknown pixelformat encountered\n");
							}
						}
						frame_interval.index++;
					}
			    }
			frame_size.index++;
		    }
		desc.index++;
	    }

	close(fd);
	return sd_count;
}

bool v4l2_frameserver_test(){
	printf("Running V4L2 Frameserver Test\n");
    v4l2_frameserver_instance_t instance;
	if (! v4l2_frameserver_init(&instance))
	{
		printf("FAILURE: Could not init frameserver.\n");
		return false;
	}
	uint32_t camera_count =0;
	if (! v4l2_frameserver_enumerate_devices(&instance,NULL,&camera_count)) {
		printf("FAILURE: Could not get camera count.\n");
		return false;
	}
    v4l2_source_descriptor_t* camera_list = calloc(camera_count,sizeof(v4l2_source_descriptor_t));
	if (! v4l2_frameserver_enumerate_devices(&instance, camera_list,&camera_count)) {
		printf("FAILURE: Could not get camera descriptors\n");
		return false;
	}
	for (uint32_t i=0;i<camera_count;i++)
	{
		printf("%d camera name: %s\n",i,camera_list[i].name);
	}
	return true;
}
