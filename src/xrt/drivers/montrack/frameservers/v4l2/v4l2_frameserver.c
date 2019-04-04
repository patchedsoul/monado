// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation
 * @author Pete Black <pblack@collabora.com>
 */


#include "util/u_misc.h"

#include "v4l2_frameserver.h"
#include "frameserver.h"

#include "../mt_framequeue.h"

// Must be before jpeglib
#include <stdio.h>

#include "jpeglib.h"

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-common.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>

#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#define NUM_V4L2_BUFFERS 2

struct v4l2_frameserver
{
	struct frameserver base;
	event_consumer_callback_func event_target_callback;
	void* event_target_instance; // where we send our events
	struct v4l2_source_descriptor source_descriptor;
	pthread_t stream_thread;
	struct fs_capture_parameters capture_params;
	bool is_configured;
	bool is_running;
};


static uint32_t
v4l2_frameserver_get_source_descriptors(struct v4l2_source_descriptor** sds,
                                        char* v4l2_device,
                                        uint32_t device_index);
static bool
source_descriptor_from_v4l2(struct v4l2_source_descriptor* descriptor,
                            char* v4l2_device,
                            struct v4l2_capability* cap,
                            struct v4l2_fmtdesc* desc);

/*!
 * Streaming thread entrypoint
 */
static void*
v4l2_frameserver_stream_run(void* ptr);

/*!
 * Cast to derived type.
 */
static inline struct v4l2_frameserver*
v4l2_frameserver(struct frameserver* inst)
{
	return (struct v4l2_frameserver*)inst;
}
static bool
v4l2_frameserver_enumerate_sources(struct frameserver* inst,
                                   fs_source_descriptor_ptr sources_generic,
                                   uint32_t* count)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	struct v4l2_source_descriptor* sources =
	    (struct v4l2_source_descriptor*)sources_generic;
	char device_files[64][256]; // max of 64 video4linux devices supported
	                            // TODO: maybe 256 too small
	char* base_path =
	    "/dev/v4l/by-id"; // TODO: does this path work everywhere?
	DIR* dir;
	struct dirent* dentry;

	dir = opendir(base_path);
	if (!dir) {
		printf("ERROR: could not open %s\n", base_path);
		return false;
	}

	uint32_t device_count = 0;
	while ((dentry = readdir(dir)) != NULL) {
		if (strcmp(dentry->d_name, ".") != 0 &&
		    strcmp(dentry->d_name, "..") != 0) {
			snprintf(device_files[device_count], 256, "%s/%s",
			         base_path,
			         dentry->d_name); // TODO: hardcoded 256
			device_count++;
		}
	}
	closedir(dir);

	uint32_t source_count = 0;

	if (sources == NULL) {
		for (uint32_t i = 0; i < device_count; i++) {
			struct v4l2_source_descriptor* temp_sds_count = NULL;
			uint32_t c = v4l2_frameserver_get_source_descriptors(
			    &temp_sds_count, device_files[i], i);
			source_count += c;
		}
		*count = source_count;
		printf("counting available source descriptors - %d\n",
		       source_count);
		return true;
	}

	// our caller should now have allocated the array of source descriptors,
	// fill them out

	for (uint32_t i = 0; i < device_count; i++) {
		struct v4l2_source_descriptor* device_sources =
		    sources + source_count;
		uint32_t c = v4l2_frameserver_get_source_descriptors(
		    &device_sources, device_files[i], i);
		source_count += c;
	}
	*count = source_count;

	return true;
}

static bool
v4l2_frameserver_configure_capture(struct frameserver* inst,
                                   struct fs_capture_parameters cp)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
	return true;
}

static bool
v4l2_frameserver_frame_get(struct frameserver* inst, struct fs_frame* frame)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
	return false;
}

static void
v4l2_frameserver_register_event_callback(
    struct frameserver* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
}

static bool
v4l2_frameserver_seek(struct frameserver* inst, uint64_t timestamp)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
	return false;
}

static bool
v4l2_frameserver_stream_start(struct frameserver* inst,
                              fs_source_descriptor_ptr source_generic)
{
	struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	struct v4l2_source_descriptor* source =
	    (struct v4l2_source_descriptor*)source_generic;
	internal->source_descriptor = *source;
	internal->is_running = true;
	if (pthread_create(&internal->stream_thread, NULL,
	                   v4l2_frameserver_stream_run, inst)) {
		printf("ERROR: could not create thread\n");
		return false;
	}
	// we're off to the races!
	return true;
}

static bool
v4l2_frameserver_stream_stop(struct frameserver* inst)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
	return false;
}

static bool
v4l2_frameserver_is_running(struct frameserver* inst)
{
	// struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	//! @todo
	return false;
}
static void
v4l2_frameserver_destroy(struct frameserver* xinst)
{
	struct v4l2_frameserver* inst = v4l2_frameserver(xinst);

	//! @todo do any destruction-required things here.

	free(inst);
}

struct frameserver*
v4l2_frameserver_create()
{
	struct v4l2_frameserver* inst = U_TYPED_CALLOC(struct v4l2_frameserver);
	inst->base.type = FRAMESERVER_TYPE_V4L2;
	inst->base.enumerate_sources = v4l2_frameserver_enumerate_sources;
	inst->base.configure_capture = v4l2_frameserver_configure_capture;
	inst->base.frame_get = v4l2_frameserver_frame_get;
	inst->base.register_event_callback =
	    v4l2_frameserver_register_event_callback;
	inst->base.seek = v4l2_frameserver_seek;
	inst->base.stream_start = v4l2_frameserver_stream_start;
	inst->base.stream_stop = v4l2_frameserver_stream_stop;
	inst->base.is_running = v4l2_frameserver_is_running;
	inst->base.destroy = v4l2_frameserver_destroy;
	return &(inst->base);
}

void*
v4l2_frameserver_stream_run(void* ptr)
{
	struct frameserver* inst = (struct frameserver*)ptr;
	struct v4l2_frameserver* internal = v4l2_frameserver(inst);
	// our jpeg decoder stuff
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	int fd = open(internal->source_descriptor.device_path, O_RDWR, 0);
	if (fd == -1) {
		printf("ERROR Cannot open '%s %d %s\n",
		       internal->source_descriptor.device_path, errno,
		       strerror(errno));
		return NULL;
	}

	// set up our capture format

	struct v4l2_format v_format;
	U_ZERO(&v_format);
	v_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v_format.fmt.pix.width = internal->source_descriptor.width;
	v_format.fmt.pix.height = internal->source_descriptor.height;
	v_format.fmt.pix.pixelformat =
	    internal->source_descriptor.stream_format;
	v_format.fmt.pix.field = V4L2_FIELD_ANY;
	if (internal->source_descriptor.extended_format > 0) {
		v_format.fmt.pix.priv = V4L2_PIX_FMT_PRIV_MAGIC;
	}


	if (ioctl(fd, VIDIOC_S_FMT, &v_format) < 0) {
		printf("ERROR: could not set up format\n");
		return NULL;
	}

	// set up our buffers - prefer userptr (client alloc) vs mmap (kernel
	// alloc)
	// TODO: using buffer caps may be better than 'fallthrough to mmap'

	bool capture_userptr = true;

	struct v4l2_requestbuffers v_bufrequest;
	U_ZERO(&v_bufrequest);
	v_bufrequest.count = NUM_V4L2_BUFFERS;
	v_bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v_bufrequest.memory = V4L2_MEMORY_USERPTR;

	if (ioctl(fd, VIDIOC_REQBUFS, &v_bufrequest) < 0) {
		printf("INFO: driver does not handle userptr buffers\n");
		v_bufrequest.memory = V4L2_MEMORY_MMAP;
		capture_userptr = false;
		if (ioctl(fd, VIDIOC_REQBUFS, &v_bufrequest) < 0) {
			printf("ERROR: driver does not handle mmap buffers\n");
			return NULL;
		}
	}

	// set up our buffers

	void* mem[NUM_V4L2_BUFFERS];

	struct v4l2_buffer v_buf;
	U_ZERO(&v_buf);

	for (uint32_t i = 0; i < NUM_V4L2_BUFFERS; i++) {
		v_buf.index = i;
		v_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		v_buf.memory = V4L2_MEMORY_USERPTR;
		if (!capture_userptr) {
			v_buf.memory = V4L2_MEMORY_MMAP;
		}
		if (ioctl(fd, VIDIOC_QUERYBUF, &v_buf) < 0) {
			printf("ERROR: could not query buffers!\n");
			return NULL;
		}

		if (capture_userptr) {
			mem[i] = aligned_alloc(
			    getpagesize(),
			    v_buf.length); // align this to a memory page, v4l2
			                   // likes it that way
			// mem[i] = malloc(v_buf.length);
			if (!mem[i]) {
				printf(
				    "ERROR: could not alloc page-aligned "
				    "memory\n");
				return NULL;
			}

			// Silence valgrind.
			memset(mem[i], 0, v_buf.length);

			v_buf.m.userptr = mem[i];
		} else {
			mem[i] = mmap(0, v_buf.length, PROT_READ, MAP_SHARED,
			              fd, v_buf.m.offset);
			if (mem[i] == MAP_FAILED) {
				printf("ERROR: mmap failed!\n");
				return NULL;
			}
		}

		/// queue this buffer
		if (ioctl(fd, VIDIOC_QBUF, &v_buf) < 0) {
			printf("ERROR: queueing buffer failed!\n");
			return NULL;
		}
	}
	int start_capture = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd, VIDIOC_STREAMON, &start_capture) < 0) {
		printf("ERROR: could not start capture!\n");
		return NULL;
	}

	uint8_t* cropped_buffer = NULL;
	struct fs_frame f =
	    {}; // we dequeue buffers into this frame in our main loop
	if (internal->source_descriptor.crop_scanline_bytes_start > 0) {
		uint32_t alloc_size = internal->source_descriptor.crop_width *
		                      internal->source_descriptor.height *
		                      fs_format_bytes_per_pixel(
		                          internal->source_descriptor.format);
		cropped_buffer = malloc(alloc_size);
		if (!cropped_buffer) {
			printf("ERROR: could not alloc memory!");
			exit(0);
		}
	}
	switch (internal->source_descriptor.stream_format) {
	case V4L2_PIX_FMT_YUYV: f.format = FS_FORMAT_YUYV_UINT8; break;
	case V4L2_PIX_FMT_JPEG:
		f.format = FS_FORMAT_JPG; // this will get reset to YUV444
		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_decompress(&cinfo);
		break;
	default: printf("ERROR: unhandled format!\n");
	}

	struct fs_frame sampled_frame;
	sampled_frame.data = NULL;

	uint8_t* temp_data = NULL;
	uint8_t* data_ptr = NULL;

	frame_queue_t* fq = frame_queue_instance();
	uint64_t source_id = frame_queue_uniq_source_id(fq);
	while (internal->is_running) {

		// if our config is invalidated at runtime, reconfigure
		if (!internal->is_configured) {
			// defaults - auto-anything off
			// uvc_set_ae_mode(internal->device_handle, 1);
			// uvc_set_ae_priority(internal->device_handle,0);
			// we may need to enumerate the control range..
			// uvc_set_exposure_abs(internal->device_handle,internal->capture_params.exposure
			// * 2048);
			// uvc_set_gain(internal->device_handle,internal->capture_params.gain
			// * 10);
			internal->is_configured = true;
		}
		// dequeue our frame, process it, requeue it.

		if (ioctl(fd, VIDIOC_DQBUF, &v_buf) < 0) {
			printf("dequeue failed\n");
		} else {
			// printf("dequeue succeeded %d used %d of %d
			// \n",v_buf.index,v_buf.bytesused,v_buf.length);
			f.source_id =
			    source_id; // set this early so it is inherited by
			               // sampled or cropped frames
			if (internal->source_descriptor
			        .crop_scanline_bytes_start > 0) {
				// we need to crop our stream frame into a new
				// buffer
				uint32_t stream_bytes_per_pixel = 0;
				switch (
				    internal->source_descriptor.stream_format) {
				case V4L2_PIX_FMT_YUYV:
					stream_bytes_per_pixel = 2;
					break;
				default:
					printf(
					    "ERROR: No crop support for "
					    "non-YUYV stream formats\n");
					exit(0);
				}

				uint32_t raw_stride =
				    internal->source_descriptor.width *
				    stream_bytes_per_pixel;
				uint32_t cropped_stride =
				    internal->source_descriptor.crop_width *
				    stream_bytes_per_pixel;
				uint8_t* bufptr = mem[v_buf.index];
				for (uint32_t i = 0;
				     i < internal->source_descriptor.height;
				     i++) {
					uint8_t* dstptr = cropped_buffer +
					                  (i * cropped_stride);
					uint8_t* srcptr =
					    bufptr + (i * raw_stride) +
					    internal->source_descriptor
					        .crop_scanline_bytes_start;
#if 0
					printf(
					    "dstptr %d srcptr %d\n",
					    (i * cropped_stride),
					    (i * raw_stride) +
					        internal->source_descriptor
					            .crop_scanline_bytes_start);
#endif
					memcpy(dstptr, srcptr, cropped_stride);
				}
				// fix up the frame we supply to the consumer
				f.width =
				    internal->source_descriptor.crop_width;
				f.height = internal->source_descriptor.height;
				// reasonable default - will get reset
				f.stride =
				    internal->source_descriptor.crop_width *
				    fs_format_bytes_per_pixel(
				        internal->source_descriptor.format);
				f.size_bytes = cropped_stride * f.height;
				f.data = cropped_buffer;
				f.source_sequence = v_buf.sequence;
			} else {
				// process frame
				f.width = internal->source_descriptor.width;
				f.height = internal->source_descriptor.height;
				// reasonable default - will get reset
				f.stride =
				    internal->source_descriptor.width *
				    fs_format_bytes_per_pixel(
				        internal->source_descriptor.format);
				f.size_bytes = v_buf.bytesused;
				f.data = mem[v_buf.index];
				f.source_sequence = v_buf.sequence;
			}
			uint8_t* preserve_sample_data = sampled_frame.data;
			switch (internal->source_descriptor.stream_format) {
			case V4L2_PIX_FMT_JPEG:
				// immediately set this to YUV444 as this is
				// what we decode to.
				f.format = FS_FORMAT_YUV444_UINT8;
				f.stride =
				    f.width *
				    3; // jpg format does not supply stride
				// decode our jpg frame.
				if (!temp_data) {
					temp_data =
					    malloc(fs_frame_size_in_bytes(&f));
				}
				jpeg_mem_src(&cinfo, mem[v_buf.index],
				             v_buf.bytesused);
				jpeg_read_header(&cinfo, TRUE);
				// we will bypass colour conversion as we want
				// YUV
				cinfo.out_color_space = cinfo.jpeg_color_space;
				jpeg_start_decompress(&cinfo);
				uint32_t scanlines_read = 0;
				data_ptr = temp_data;
				while (scanlines_read < cinfo.image_height) {
					int read_count = jpeg_read_scanlines(
					    &cinfo, &data_ptr, 16);
					data_ptr +=
					    read_count *
					    internal->source_descriptor.width *
					    3;
					scanlines_read += read_count;
				}
				f.data = temp_data;
				jpeg_finish_decompress(&cinfo);

				switch (internal->source_descriptor.format) {
				case FS_FORMAT_Y_UINT8:
					// split our Y plane out
					sampled_frame = f;
					sampled_frame.data =
					    preserve_sample_data; // put our
					                          // correct
					                          // data back
					sampled_frame.format =
					    FS_FORMAT_Y_UINT8;
					sampled_frame.stride = f.width;
					sampled_frame.size_bytes =
					    fs_frame_size_in_bytes(
					        &sampled_frame);

					if (!sampled_frame.data) {
						sampled_frame.data = malloc(
						    sampled_frame.size_bytes);
					}

					fs_frame_extract_plane(&f, FS_PLANE_Y,
					                       &sampled_frame);

					frame_queue_add(fq, &sampled_frame);
					break;
				default:
					// supply our YUV444 directly
					frame_queue_add(fq, &f);
				}
				break;
			case V4L2_PIX_FMT_YUYV:
				f.stride =
				    f.width * 2; // 2 bytes per pixel for yuyv
				switch (internal->source_descriptor.format) {
				case FS_FORMAT_Y_UINT8:
					// split our Y plane out
					sampled_frame = f; // copy our buffer
					                   // frames attributes
					sampled_frame.data =
					    preserve_sample_data; // put our
					                          // correct
					                          // data back
					sampled_frame.format =
					    FS_FORMAT_Y_UINT8;
					sampled_frame.stride = f.width;
					sampled_frame.size_bytes =
					    fs_frame_size_in_bytes(
					        &sampled_frame);

					if (!sampled_frame.data) {
						sampled_frame.data = malloc(
						    sampled_frame.size_bytes);
					}

					fs_frame_extract_plane(&f, FS_PLANE_Y,
					                       &sampled_frame);

					frame_queue_add(fq, &sampled_frame);
					break;
				case FS_FORMAT_YUV444_UINT8:
					// upsample our YUYV to YUV444
					sampled_frame = f; // copy our buffer
					                   // frames attributes
					sampled_frame.data =
					    preserve_sample_data; // put our
					                          // correct
					                          // data back
					sampled_frame.format =
					    FS_FORMAT_YUV444_UINT8;
					sampled_frame.stride = f.width * 3;
					sampled_frame.size_bytes =
					    fs_frame_size_in_bytes(
					        &sampled_frame);
					// allocate on first access
					if (!sampled_frame.data) {
						sampled_frame.data = malloc(
						    sampled_frame.size_bytes);
					}
					if (fs_frame_resample(&f,
					                      &sampled_frame)) {
						frame_queue_add(fq,
						                &sampled_frame);
						break;
					}
					printf(
					    "ERROR: could not resample frame "
					    "from %d to %d\n",
					    f.format, sampled_frame.format);
					break;
				default:
					// supply our YUYV directly
					frame_queue_add(fq, &f);
				}
				break;
			default: printf("ERROR: Unknown stream format\n");
			}
			driver_event_t e;
			U_ZERO(&e);
			e.type = EVENT_FRAMESERVER_GOTFRAME;
			if (internal->event_target_callback) {
				internal->event_target_callback(
				    internal->event_target_instance, e);
			}


			if (ioctl(fd, VIDIOC_QBUF, &v_buf) < 0) {
				printf("requeue failed\n");
			}
		}
	}
	/*	res =  uvc_stream_get_frame	(internal->stream_handle,
	&frame,0); if (res < 0) { printf("ERROR: stream_get_frame
	%s\n",uvc_strerror(res)); } else { if (frame) {
	                        //printf("got frame\n");


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
	return NULL;
}


uint32_t
v4l2_frameserver_get_source_descriptors(struct v4l2_source_descriptor** sds,
                                        char* v4l2_device,
                                        uint32_t device_index)
{

	uint32_t sd_count = 0;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	// open the device, and check if is a video source

	int fd = open(v4l2_device, O_RDWR, 0);
	if (fd == -1) {
		printf("ERROR Cannot open '%s %d %s\n", v4l2_device, errno,
		       strerror(errno));
		return 0;
	}

	struct v4l2_source_descriptor* descriptor = *sds;
	if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
		if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
			return 0; // not a video device
		}
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			return 0; // cannot stream
		}
		if (!(cap.capabilities & V4L2_CAP_TIMEPERFRAME)) {
			printf(
			    "WARNING: device does not support setting frame "
			    "intervals\n");
		}
		if (*sds) { // skip this if we are just counting, descriptor
			    // will be NULL
			if ((cap.capabilities & V4L2_CAP_EXT_PIX_FORMAT)) {
				descriptor->extended_format =
				    1; // need to query for extended format info
			}
		}
	}
	struct v4l2_fmtdesc desc;
	U_ZERO(&desc);
	desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	while (ioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0) {
		printf("FORMAT: %s %04x %d\n", desc.description,
		       desc.pixelformat, desc.type);

		struct v4l2_frmsizeenum frame_size;
		U_ZERO(&frame_size);

		struct v4l2_frmivalenum frame_interval;
		U_ZERO(&frame_interval);

		frame_size.pixel_format = desc.pixelformat;
		frame_size.index = 0;
		while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frame_size) >= 0) {
			if (frame_size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
				printf("%dx%d\n", frame_size.discrete.width,
				       frame_size.discrete.height);
				frame_interval.pixel_format =
				    frame_size.pixel_format;
				frame_interval.width =
				    frame_size.discrete.width;
				frame_interval.height =
				    frame_size.discrete.height;
				frame_interval.index = 0;
				while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS,
				             &frame_interval) >= 0) {
					float fps =
					    frame_interval.discrete
					        .denominator /
					    frame_interval.discrete.numerator;
					uint32_t rate =
					    (frame_interval.discrete.numerator /
					     (float)frame_interval.discrete
					         .denominator) *
					    10000000;
					printf("FPS: %f %d\n", fps, rate);

					if (*sds) {
						// only fill in this struct if
						// we were not passed NULL

						switch (desc.pixelformat) {
						case V4L2_PIX_FMT_YUYV:
							descriptor->format =
							    FS_FORMAT_YUYV_UINT8;
							descriptor->width =
							    frame_interval
							        .width;
							descriptor->height =
							    frame_interval
							        .height;
							descriptor->rate = rate;
							descriptor->sampling =
							    FS_SAMPLING_NONE;
							source_descriptor_from_v4l2(
							    descriptor,
							    v4l2_device, &cap,
							    &desc);
							descriptor++;

							descriptor->format =
							    FS_FORMAT_YUV444_UINT8;
							descriptor->width =
							    frame_interval
							        .width;
							descriptor->height =
							    frame_interval
							        .height;
							descriptor->rate = rate;
							descriptor->sampling =
							    FS_SAMPLING_UPSAMPLED;
							source_descriptor_from_v4l2(
							    descriptor,
							    v4l2_device, &cap,
							    &desc);
							descriptor++;

							descriptor->format =
							    FS_FORMAT_Y_UINT8;
							descriptor->width =
							    frame_interval
							        .width;
							descriptor->height =
							    frame_interval
							        .height;
							descriptor->rate = rate;
							descriptor->sampling =
							    FS_SAMPLING_DOWNSAMPLED;
							source_descriptor_from_v4l2(
							    descriptor,
							    v4l2_device, &cap,
							    &desc);
							descriptor++;
							sd_count += 3;
							break;
						case V4L2_PIX_FMT_JPEG: // MJPEG
						                        // stream
						                        // format
							descriptor->format =
							    FS_FORMAT_YUV444_UINT8;
							descriptor->width =
							    frame_interval
							        .width;
							descriptor->height =
							    frame_interval
							        .height;
							descriptor->rate = rate;
							descriptor->sampling =
							    FS_SAMPLING_UPSAMPLED;
							source_descriptor_from_v4l2(
							    descriptor,
							    v4l2_device, &cap,
							    &desc);
							descriptor++;

							descriptor->format =
							    FS_FORMAT_Y_UINT8;
							descriptor->width =
							    frame_interval
							        .width;
							descriptor->height =
							    frame_interval
							        .height;
							descriptor->rate = rate;
							descriptor->sampling =
							    FS_SAMPLING_DOWNSAMPLED;
							source_descriptor_from_v4l2(
							    descriptor,
							    v4l2_device, &cap,
							    &desc);
							descriptor++;
							sd_count += 2;
							break;
						default:
							printf(
							    "ERROR: unknown "
							    "pixelformat "
							    "encountered\n");
						}
					} else {
						// we just need the count of the
						// sources we would create
						switch (desc.pixelformat) {
						case 0x56595559: // YUYV stream
						                 // format
							sd_count +=
							    3; // YUYV, YUV444,
							       // Y
							break;
						case 0x47504a4d: // MJPEG stream
						                 // format
							sd_count +=
							    2; // YUV444,Y
							break;
						default:
							printf(
							    "ERROR: unknown "
							    "pixelformat "
							    "encountered\n");
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

bool
v4l2_frameserver_test()
{
	printf("Running V4L2 Frameserver Test\n");
	struct frameserver* frameserver =
	    frameserver_create(FRAMESERVER_TYPE_V4L2);
	if (!frameserver) {
		printf("FAILURE: Could not create frameserver.\n");
		return false;
	}
	uint32_t camera_count = 0;
	if (!frameserver_enumerate_sources(frameserver, NULL, &camera_count)) {
		printf("FAILURE: Could not get camera count.\n");
		return false;
	}
	struct v4l2_source_descriptor* camera_list =
	    U_TYPED_ARRAY_CALLOC(struct v4l2_source_descriptor, camera_count);
	if (!frameserver_enumerate_sources(frameserver, camera_list,
	                                   &camera_count)) {
		printf("FAILURE: Could not get camera descriptors\n");
		return false;
	}
	for (uint32_t i = 0; i < camera_count; i++) {
		printf("%d camera name: %s\n", i, camera_list[i].name);
	}
	return true;
}

static bool
source_descriptor_from_v4l2(struct v4l2_source_descriptor* descriptor,
                            char* v4l2_device,
                            struct v4l2_capability* cap,
                            struct v4l2_fmtdesc* desc)
{

	strncpy(descriptor->device_path, v4l2_device,
	        256);                       // TODO: hardcoded 256
	descriptor->device_path[255] = 0x0; // TODO: hardcoded 256
	strncpy(descriptor->name, cap->driver, 32);
	descriptor->name[127] = 0x0;
	strncpy(descriptor->model, cap->card, 32);
	descriptor->model[127] = 0x0;
	descriptor->stream_format = desc->pixelformat;
	descriptor->crop_scanline_bytes_start = 0;
	// special-case the PS4 Eye camera  - need to crop the main stereo image
	// out of the composite (header+audio + main + interlaced) frame the
	// driver produces
	if (strcmp(cap->card, "USB Camera-OV580: USB Camera-OV") == 0) {
		descriptor->crop_scanline_bytes_start = 96;
		descriptor->crop_width = 2560; // assume highest res
		if (descriptor->width < 900) {
			descriptor->crop_width = 640;
		} else if (descriptor->width < 2000) {
			descriptor->crop_width = 1280;
		}
	}
	return true;
}
