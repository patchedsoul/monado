#include <common/frameserver.h>
#include "uvc_frameserver.h"
#include <string.h>
#include <stdlib.h>

#include <jpeglib.h>

#include "util/u_misc.h"


static void
uvc_frameserver_stream_run(
    frameserver_instance_t* inst); // streaming thread entrypoint
static uint32_t
uvc_frameserver_get_source_descriptors(uvc_source_descriptor_t** sds,
                                       uvc_device_t* device,
                                       uint32_t uvc_device_index);
static bool
source_descriptor_from_uvc_descriptor(
    uvc_source_descriptor_t* source_descriptor,
    uvc_device_descriptor_t* uvc_device_descriptor,
    uvc_frame_desc_t* uvc_frame_descriptor);


static uvc_error_t res;

bool
uvc_source_create(uvc_source_descriptor_t* desc)
{
	// do nothing right now
	return true;
}

bool
uvc_source_destroy(uvc_source_descriptor_t* desc)
{
	// do nothing right now
	return true;
}


uvc_frameserver_instance_t*
uvc_frameserver_create(frameserver_instance_t* inst)
{
	printf("creating uvc frameserver\n");
	uvc_frameserver_instance_t* i =
	    U_TYPED_CALLOC(uvc_frameserver_instance_t);
	if (i) {
		i->device_list = NULL;
		res = uvc_init(&(i->context), NULL);
		if (res < 0) {
			uvc_perror(res, "UVC Context init failed");
			return NULL;
		}

		return i;
	}

	return NULL;
}

bool
uvc_frameserver_enumerate_sources(frameserver_instance_t* inst,
                                  uvc_source_descriptor_t* cameras,
                                  uint32_t* count)
{
	uvc_error_t res;
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	// if (internal->device_list != NULL) {
	//	uvc_free_device_list(internal->device_list,0);
	//}
	uint32_t device_count = 0;
	res = uvc_get_device_list(internal->context, &(internal->device_list));
	if (res < 0) {
		printf("ERROR: %s\n", uvc_strerror(res));
		return false;
	}
	while (1) {
		uvc_device_t* uvc_device = internal->device_list[device_count];
		if (uvc_device == NULL) {
			break;
		}
		device_count++;
	}

	uint32_t source_count = 0;

	if (cameras == NULL) {
		printf("counting formats\n");
		for (uint32_t i = 0; i < device_count; i++) {
			uvc_source_descriptor_t* temp_sds_count = NULL;
			// we need to free the source descriptors, even though
			// we only use the count
			uint32_t c = uvc_frameserver_get_source_descriptors(
			    &temp_sds_count, internal->device_list[i], i);
			source_count += c;
			free(temp_sds_count);
		}

		*count = source_count;
		// uvc_free_device_list(internal->device_list,1);
		// internal->device_list = NULL;
		return true;
	}

	printf("returning formats\n");

	// if we were passed an array of camera descriptors, fill them in
	uvc_source_descriptor_t* temp_sds = NULL;

	uint32_t cameras_offset = 0;
	for (uint32_t i = 0; i < device_count; i++) {
		// last parameter will end up in source_id
		uint32_t c = uvc_frameserver_get_source_descriptors(
		    &temp_sds, internal->device_list[i], i);
		printf("Got %d sources\n", c);
		if (c > 0) {
			source_count += c;
			memcpy(cameras + cameras_offset, temp_sds,
			       c * sizeof(uvc_source_descriptor_t));
			cameras_offset += c;
		}
	}

	if (source_count == 0) {
		return false;
	}

	// free(temp_sds);
	// uvc_free_device_list(internal->device_list,1);
	// internal->device_list = NULL;
	return true;
}


bool
uvc_frameserver_configure_capture(frameserver_instance_t* inst,
                                  capture_parameters_t cp)
{
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	internal->capture_params = cp;
	internal->is_configured = false;
	return true;
}

void
uvc_frameserver_register_frame_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    frame_consumer_callback_func target_func)
{
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	internal->frame_target_instance = target_instance;
	internal->frame_target_callback = target_func;
}

void
uvc_frameserver_register_event_callback(
    frameserver_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
	// do nothing
}

bool
uvc_frameserver_get(frameserver_instance_t* inst, frame_t* _frame)
{
	return false;
}
bool
uvc_frameserver_seek(frameserver_instance_t* inst, uint64_t timestamp)
{
	return false;
}
bool
uvc_frameserver_stream_start(frameserver_instance_t* inst,
                             uvc_source_descriptor_t* source)
{
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	internal->source_descriptor = *source;
	internal->is_running = true;
	if (pthread_create(&internal->stream_thread, NULL,
	                   uvc_frameserver_stream_run, inst)) {
		printf("ERROR: could not create thread\n");
		return false;
	}
	// we're off to the races!
	return true;
}

bool
uvc_frameserver_stream_stop(frameserver_instance_t* inst)
{
	// TODO: stop the stream, join the thread, cleanup.
	return false;
}

bool
uvc_frameserver_is_running(frameserver_instance_t* inst)
{
	return false;
}

void
uvc_frameserver_stream_run(frameserver_instance_t* inst)
{
	uvc_error_t res;
	uvc_frameserver_instance_t* internal = inst->internal_instance;
	bool split_planes;
	plane_t planes[MAX_PLANES] = {};

	// clear our kill_handler_thread flag, likely set when closing
	// devices during format enumeration
	internal->context->kill_handler_thread = 0;

	// our jpeg decoder stuff
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;


	res = uvc_open(
	    internal->device_list[internal->source_descriptor.uvc_device_index],
	    &internal->device_handle);
	if (res < 0) {
		printf("ERROR: %s open %s\n", &internal->source_descriptor.name,
		       uvc_strerror(res));
		return;
	}
	int fps = 1.0 / (internal->source_descriptor.rate / 10000000.0f);
	res = uvc_get_stream_ctrl_format_size(
	    internal->device_handle, &internal->stream_ctrl,
	    internal->source_descriptor.stream_format,
	    internal->source_descriptor.width,
	    internal->source_descriptor.height, fps);
	if (res < 0) {
		printf("ERROR: %s get_stream_ctrl_format %s\n",
		       &internal->source_descriptor.name, uvc_strerror(res));
		return;
	}

	uvc_print_stream_ctrl(&internal->stream_ctrl, stdout);

	res = uvc_stream_open_ctrl(internal->device_handle,
	                           &internal->stream_handle,
	                           &internal->stream_ctrl);
	if (res < 0) {
		printf("ERROR: stream_open_ctrl %s\n", uvc_strerror(res));
		return;
	}

	res = uvc_stream_start(internal->stream_handle, NULL, NULL, 0);
	if (res < 0) {
		printf("ERROR: stream_start %s\n", uvc_strerror(res));
		return;
	}

	frame_t f = {}; // our buffer
	f.source_id = internal->source_descriptor.source_id;
	switch (internal->source_descriptor.stream_format) {
	case UVC_FRAME_FORMAT_YUYV: f.format = FORMAT_YUYV_UINT8; break;
	case UVC_FRAME_FORMAT_MJPEG:
		f.format = FORMAT_JPG; // this will get reset to YUV444
		cinfo.err = jpeg_std_error(&jerr);
		jpeg_create_decompress(&cinfo);
		break;
	default: printf("ERROR: unhandled format!\n");
	}

	frame_t sampled_frame = {};

	// replaced by sampled_frame but may be useful for planar output.
	frame_t plane_frame = {};
	uint8_t* plane_data[MAX_PLANES];

	uint8_t* temp_data = NULL;
	uint8_t* data_ptr = NULL;

	uvc_frame_t* frame =
	    uvc_allocate_frame(internal->stream_ctrl.dwMaxVideoFrameSize);
	while (internal->is_running) {

		// if our config is invalidated at runtime, reconfigure
		if (!internal->is_configured) {
			// defaults - auto-anything off
			uvc_set_ae_mode(internal->device_handle, 1);
			uvc_set_ae_priority(internal->device_handle, 0);
			// we may need to enumerate the control range..
			uint32_t exp_time =
			    internal->capture_params.exposure * 2048;
			uvc_set_exposure_abs(internal->device_handle, 50);
			uvc_set_gain(internal->device_handle,
			             internal->capture_params.gain * 10);

			internal->is_configured = true;
		}

		res = uvc_stream_get_frame(internal->stream_handle, &frame, 0);
		if (res < 0) {
			printf("ERROR: stream_get_frame %s\n",
			       uvc_strerror(res));
		} else {
			if (frame) {
				// printf("got frame\n");

				f.width = frame->width;
				f.height = frame->height;
				f.stride = frame->step;
				f.size_bytes = frame->data_bytes;
				f.data = frame->data;

				switch (
				    internal->source_descriptor.stream_format) {
				case UVC_FRAME_FORMAT_MJPEG:
					// immediately set this to YUV444 as
					// this is what we decode to.
					f.format = FORMAT_YUV444_UINT8;
					f.stride =
					    f.width * 3; // jpg format does not
					                 // supply stride
					// decode our jpg frame.
					if (!temp_data) {
						temp_data = malloc(
						    frame_size_in_bytes(&f));
					}
					jpeg_mem_src(&cinfo, frame->data,
					             frame->data_bytes);
					jpeg_read_header(&cinfo, TRUE);
					// we will bypass colour conversion as
					// we want YUV
					cinfo.out_color_space =
					    cinfo.jpeg_color_space;
					jpeg_start_decompress(&cinfo);
					uint32_t scanlines_read = 0;
					data_ptr = temp_data;
					while (scanlines_read <
					       cinfo.image_height) {
						int read_count =
						    jpeg_read_scanlines(
						        &cinfo, &data_ptr, 16);
						data_ptr += read_count *
						            frame->width * 3;
						scanlines_read += read_count;
					}
					f.data = temp_data;
					jpeg_finish_decompress(&cinfo);

					switch (internal->source_descriptor
					            .format) {
					case FORMAT_Y_UINT8:
						// split our Y plane out
						sampled_frame =
						    f; // copy our buffer frames
						       // attributes
						sampled_frame.format =
						    FORMAT_Y_UINT8;
						sampled_frame.stride = f.width;
						sampled_frame.size_bytes =
						    frame_size_in_bytes(
						        &sampled_frame);

						if (!sampled_frame.data) {
							sampled_frame
							    .data = malloc(
							    sampled_frame
							        .size_bytes);
						}

						frame_extract_plane(
						    &f, PLANE_Y,
						    &sampled_frame);

						if (internal
						        ->frame_target_callback) {
							internal->frame_target_callback(
							    internal
							        ->frame_target_instance,
							    &sampled_frame);
						}
						break;
					default:
						// supply our YUV444 directly
						if (internal
						        ->frame_target_callback) {
							internal->frame_target_callback(
							    internal
							        ->frame_target_instance,
							    &f);
						}
					}
					break;
				case UVC_FRAME_FORMAT_YUYV:
					switch (internal->source_descriptor
					            .format) {
					case FORMAT_Y_UINT8:
						// split our Y plane out
						sampled_frame =
						    f; // copy our buffer frames
						       // attributes
						sampled_frame.format =
						    FORMAT_Y_UINT8;
						sampled_frame.stride = f.width;
						sampled_frame.size_bytes =
						    frame_size_in_bytes(
						        &sampled_frame);

						if (!sampled_frame.data) {
							sampled_frame
							    .data = malloc(
							    sampled_frame
							        .size_bytes);
						}

						frame_extract_plane(
						    &f, PLANE_Y,
						    &sampled_frame);

						if (internal
						        ->frame_target_callback) {
							internal->frame_target_callback(
							    internal
							        ->frame_target_instance,
							    &sampled_frame);
						}
						break;
					case FORMAT_YUV444_UINT8:
						// upsample our YUYV to YUV444
						sampled_frame =
						    f; // copy our buffer frames
						       // attributes
						sampled_frame.format =
						    FORMAT_YUV444_UINT8;
						sampled_frame.stride =
						    f.width * 3;
						sampled_frame.size_bytes =
						    frame_size_in_bytes(
						        &sampled_frame);
						// allocate on first access
						if (!sampled_frame.data) {
							sampled_frame
							    .data = malloc(
							    sampled_frame
							        .size_bytes);
						}
						if (frame_resample(
						        &f, &sampled_frame)) {
							if (internal
							        ->frame_target_callback) {
								internal->frame_target_callback(
								    internal
								        ->frame_target_instance,
								    &sampled_frame);
							}
							break;
						}
						printf(
						    "ERROR: could not resample "
						    "frame from %d to %d\n",
						    f.format,
						    sampled_frame.format);
						break;
					default:
						// supply our YUYV directly
						if (internal
						        ->frame_target_callback) {
							internal->frame_target_callback(
							    internal
							        ->frame_target_instance,
							    &f);
						}
					}
					break;
				default:
					printf(
					    "ERROR: Unknown stream format\n");
				}
				driver_event_t e = {};
				e.type = EVENT_FRAMESERVER_GOTFRAME;
				if (internal->event_target_callback) {
					internal->event_target_callback(
					    internal->event_target_instance, e);
				}
			}
		}
	}
	uvc_free_frame(frame);
	if (temp_data) {
		free(temp_data);
		temp_data = NULL;
	}
	if (sampled_frame.data) {
		free(sampled_frame.data);
	}
	return;
}


bool
uvc_frameserver_test()
{
	printf("Running UVC Frameserver Test\n");
	frameserver_instance_t* uvc_frameserver =
	    frameserver_create(FRAMESERVER_TYPE_UVC);
	if (!uvc_frameserver) {
		printf("FAILURE: Could not create frameserver.\n");
		return false;
	}
	uint32_t source_count = 0;
	if (!uvc_frameserver->frameserver_enumerate_sources(
	        uvc_frameserver, NULL, &source_count)) {
		printf("FAILURE: Could not get source count.\n");
		return false;
	}
	uvc_source_descriptor_t* source_list =
	    U_TYPED_ARRAY_CALLOC(uvc_source_descriptor_t, source_count);
	if (!uvc_frameserver->frameserver_enumerate_sources(
	        uvc_frameserver, source_list, &source_count)) {
		printf("FAILURE: Could not get source descriptors\n");
		return false;
	}
	for (uint32_t i = 0; i < source_count; i++) {
		printf("%d source name: %s\n", i, source_list[i].name);
	}
	free(source_list);
	return true;
}

// TODO: fix this so we don't need to alloc?
uint32_t
uvc_frameserver_get_source_descriptors(uvc_source_descriptor_t** sds,
                                       uvc_device_t* uvc_device,
                                       uint32_t device_index)
{

	uint32_t sd_count = 0;
	uvc_device_descriptor_t* uvc_device_descriptor;
	res = uvc_get_device_descriptor(uvc_device, &uvc_device_descriptor);
	if (res < 0) {
		printf("ERROR: %s\n", uvc_strerror(res));
	}
	uvc_device_handle_t* temp_handle;
	res = uvc_open(uvc_device, &temp_handle);
	if (res == UVC_SUCCESS) {
		const uvc_format_desc_t* format_desc =
		    uvc_get_format_descs(temp_handle);
		uvc_source_descriptor_t* desc = *sds;
		uvc_source_descriptor_t* temp_alloc =
		    U_TYPED_CALLOC(uvc_device_descriptor_t);
		while (format_desc != NULL) {
			printf("Found format: %d FOURCC %c%c%c%c\n",
			       format_desc->bFormatIndex,
			       format_desc->fourccFormat[0],
			       format_desc->fourccFormat[1],
			       format_desc->fourccFormat[2],
			       format_desc->fourccFormat[3]);
			uvc_frame_desc_t* frame_desc = format_desc->frame_descs;
			while (frame_desc != NULL) {
				printf("W %d H %d\n", frame_desc->wWidth,
				       frame_desc->wHeight);
				uint32_t* frame_duration =
				    frame_desc->intervals;
				while (*frame_duration != 0) {
					printf("rate: %d %f\n", *frame_duration,
					       1.0 / (*frame_duration /
					              10000000.0f));
					if (*frame_duration <
					    400000) { // anything quicker than
						      // 25fps
						// if we are a YUV mode, write
						// out a descriptor + the Y-only
						// descriptor
						if (format_desc
						        ->fourccFormat[0] ==
						    'Y') {

							temp_alloc = realloc(
							    *sds,
							    (sd_count + 3) *
							        sizeof(
							            uvc_source_descriptor_t));
							if (!temp_alloc) {
								printf(
								    "ERROR: "
								    "could not "
								    "allocate "
								    "memory\n");
								exit(1);
							}

							*sds = temp_alloc;
							desc = temp_alloc +
							       sd_count;

							desc->uvc_device_index =
							    device_index;
							desc->rate =
							    *frame_duration;
							source_descriptor_from_uvc_descriptor(
							    desc,
							    uvc_device_descriptor,
							    frame_desc);
							desc->stream_format =
							    UVC_FRAME_FORMAT_YUYV;
							desc->format =
							    FORMAT_YUYV_UINT8;
							desc->sampling =
							    SAMPLING_NONE;
							sd_count++;
							desc++;

							// YUV444 format
							desc->uvc_device_index =
							    device_index;
							desc->rate =
							    *frame_duration;
							source_descriptor_from_uvc_descriptor(
							    desc,
							    uvc_device_descriptor,
							    frame_desc);
							desc->stream_format =
							    UVC_FRAME_FORMAT_YUYV;
							desc->format =
							    FORMAT_YUV444_UINT8;
							desc->sampling =
							    SAMPLING_UPSAMPLED;
							sd_count++;
							desc++;

							// also output our 'one
							// plane Y' format
							desc->uvc_device_index =
							    device_index;
							desc->rate =
							    *frame_duration;
							source_descriptor_from_uvc_descriptor(
							    desc,
							    uvc_device_descriptor,
							    frame_desc);
							desc->stream_format =
							    UVC_FRAME_FORMAT_YUYV;
							desc->format =
							    FORMAT_Y_UINT8;
							desc->sampling =
							    SAMPLING_DOWNSAMPLED;
							sd_count++;
							desc++;
						} else if (format_desc
						               ->fourccFormat
						                   [0] == 'M') {
							// MJPG, most likely -
							// TODO: check more than
							// the first letter

							temp_alloc = realloc(
							    *sds,
							    (sd_count + 2) *
							        sizeof(
							            uvc_source_descriptor_t));
							if (!temp_alloc) {
								printf(
								    "ERROR: "
								    "could not "
								    "allocate "
								    "memory\n");
								exit(1);
							}
							*sds = temp_alloc;
							desc = temp_alloc +
							       sd_count;

							desc->uvc_device_index =
							    device_index;
							desc->rate =
							    *frame_duration;
							source_descriptor_from_uvc_descriptor(
							    desc,
							    uvc_device_descriptor,
							    frame_desc);
							desc->stream_format =
							    UVC_FRAME_FORMAT_MJPEG;
							desc->format =
							    FORMAT_YUV444_UINT8;
							desc->sampling =
							    SAMPLING_UPSAMPLED;
							sd_count++;
							desc++;

							desc->uvc_device_index =
							    device_index;
							desc->rate =
							    *frame_duration;
							source_descriptor_from_uvc_descriptor(
							    desc,
							    uvc_device_descriptor,
							    frame_desc);
							desc->stream_format =
							    UVC_FRAME_FORMAT_MJPEG;
							desc->format =
							    FORMAT_Y_UINT8;
							desc->sampling =
							    SAMPLING_DOWNSAMPLED;
							sd_count++;
							desc++;
						}
					}
					frame_duration++; // incrementing
					                  // pointer
				}
				frame_desc = frame_desc->next;
			}
			format_desc = format_desc->next;
		}
		uvc_close(temp_handle);
	}
	// this crashes - i guess we only care about closing if we have started
	// streaming
	//
	// uvc_free_device_descriptor(uvc_device_descriptor);
	printf("RETURNING %d\n", sd_count);
	return sd_count;
}

bool
source_descriptor_from_uvc_descriptor(
    uvc_source_descriptor_t* source_descriptor,
    uvc_device_descriptor_t* uvc_device_descriptor,
    uvc_frame_desc_t* uvc_frame_descriptor)
{
	snprintf(
	    source_descriptor->name, 128, "%s %s %s %04x:%04x",
	    uvc_device_descriptor->manufacturer, uvc_device_descriptor->product,
	    uvc_device_descriptor->serialNumber,
	    uvc_device_descriptor->idProduct, uvc_device_descriptor->idVendor);
	source_descriptor->name[127] = 0;
	source_descriptor->product_id = uvc_device_descriptor->idProduct;
	source_descriptor->vendor_id = uvc_device_descriptor->idVendor;
	// TODO check lengths
	if (uvc_device_descriptor->serialNumber) {
		memcpy(source_descriptor->serial,
		       uvc_device_descriptor->serialNumber,
		       strlen(uvc_device_descriptor->serialNumber) + 1);
	} else {
		sprintf(source_descriptor->serial, "NONE");
	}
	source_descriptor->serial[127] = 0;
	source_descriptor->width = uvc_frame_descriptor->wWidth;
	source_descriptor->height = uvc_frame_descriptor->wHeight;
}
