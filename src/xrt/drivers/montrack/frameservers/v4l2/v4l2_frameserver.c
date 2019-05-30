#include <common/frameserver.h>
#include "v4l2_frameserver.h"
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "errno.h"
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

	char device_files[64][256]; // max of 64 video4linux devices supported TODO: maaybe 256 too small
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
		source_count+=c;
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

bool v4l2_frameserver_stream_start(frameserver_instance_t* inst){
	return false;
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
						//printf("FPS: %f %d\n", fps,rate);

						if (*sds) {
							//only fill in this struct if we were not passed NULL
							v4l2_source_descriptor_t* descriptor = *sds;
							switch(desc.pixelformat){
							case 0x56595559: //YUYV stream format
								descriptor->format = FORMAT_YUYV_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_NONE;
								descriptor++;

								descriptor->format = FORMAT_YUV444_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_UPSAMPLED;
								descriptor++;

								descriptor->format = FORMAT_Y_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_DOWNSAMPLED;
								descriptor++;

								break;
							case 0x47504a4d: //MJPEG stream format
								descriptor->format = FORMAT_YUV444_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_UPSAMPLED;

								descriptor++;descriptor->format = FORMAT_Y_UINT8;
								strncpy(descriptor->device_path,v4l2_device,256); //TODO: hardcoded 256
								descriptor->device_path[255]=0x0; //TODO: hardcoded 256
								descriptor->width = frame_interval.width;
								descriptor->height = frame_interval.height;
								descriptor->stream_format = desc.pixelformat;
								descriptor->rate = rate;
								descriptor->sampling = SAMPLING_DOWNSAMPLED;
								descriptor++;
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
