// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Assembles and runs a Monado internal device.
 * @author Pete Black <pete.black@collabora.com>
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_time.h"

#include "mt_device.h"

#include "../optical_tracking/tracker3D_sphere_mono.h"

//#IFDEF have_opencv
#include "../filters/filter_opencv_kalman.h"
//#IFDEF have_uvc
#include "../frameservers/uvc/uvc_frameserver.h"


static void
mt_device_destroy(struct xrt_device *xdev)
{
	mt_device_t* md = mt_device(xdev);


	free(md);
}

static void
mt_device_get_tracked_pose(struct xrt_device *xdev,
                           struct time_state *timekeeping,
                           int64_t *out_timestamp,
                           struct xrt_space_relation *out_relation)
{
	mt_device_t* md = mt_device(xdev);
	struct xrt_pose pose;
	filter_state_t filtered;
	switch (md->tracker->tracker_type) {
	    case TRACKER_TYPE_SPHERE_MONO:
		    out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
			md->filter->filter_predict_state(md->filter,&filtered,0);
			out_relation->pose = filtered.pose;
		    break;
	    case TRACKER_TYPE_SPHERE_STEREO:
		    out_relation->relation_flags = (enum xrt_space_relation_flags)(
			XRT_SPACE_RELATION_POSITION_VALID_BIT |
			XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
			md->filter->filter_predict_state(md->filter,&filtered,0);
			out_relation->pose = filtered.pose;
		    break;

	    default:
		    printf("ERROR: Unknown tracker type\n");
	}


	// Update state within driver
	//md->last_update = *out_timestamp;
	//md->last_relation = *out_relation;
}

static void
mt_device_get_view_pose(struct xrt_device *xdev,
                        struct xrt_vec3 *eye_relation,
                        uint32_t view_index,
                        struct xrt_pose *out_pose)
{
	struct xrt_pose pose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	*out_pose = pose;
}



mt_device_t*
mt_device_create(char* device_name,bool log_verbose, bool log_debug) {
	mt_device_t* md = U_TYPED_CALLOC(mt_device_t);

	dummy_init_mt_device(md);

	if (strcmp(device_name,"MONO_LOGITECH_C270") == 0) {
		if (mt_create_mono_c270(md)) {
			return md;
		}
	}
	if (strcmp(device_name,"STEREO_ELP_60FPS") == 0) {
		if (mt_create_stereo_elp(md)) {
			return md;
		}
	}

	return NULL;



}
bool mt_create_mono_c270(mt_device_t* md) {
	// TODO - add IMU input source -> filter

	md->frameserver_count=1; // this driver uses a single camera source
	md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
	// ask our frameserver for available sources - note this will return a
	// type-specific struct that we need to deal with e.g. UVC-specific, FFMPEG-specific.
	uint32_t source_count=0;
	md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],NULL,&source_count);
	if (source_count == 0){
		//we have no sources, we cannot continue
		return false;
	}
	uvc_source_descriptor_t* descriptors = calloc(source_count,sizeof(uvc_source_descriptor_t));
	md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],descriptors,&source_count);
	// defer further configuration and stream start until the rest of our chain is set up.

	md->tracker = tracker_create(TRACKER_TYPE_SPHERE_MONO);
	tracker_mono_configuration_t tracker_config = {};

	// configure our logitech c270 when we find it during enumeration
	uint32_t source_index; //our frameserver config descriptor index
	for (uint32_t i=0; i< source_count;i++){
		if (descriptors[i].product_id == 0x0825 && descriptors[i].vendor_id == 0x046d && descriptors[i].format == FORMAT_Y_UINT8) {
			if (descriptors[i].width == 640 && descriptors[i].height == 480 && descriptors[i].rate == 333333) {
				tracker_config.format = descriptors[i].format;
				tracker_config.source_id =descriptors[i].source_id;
				float camera_size[2] = LOGITECH_C270_SIZE;
				float camera_intr[INTRINSICS_SIZE] = LOGITECH_C270_INTR;
				float camera_dist[DISTORTION_SIZE] = LOGITECH_C270_DIST;

				tracker_config.calibration.calib_capture_size[0]=camera_size[0];
				tracker_config.calibration.calib_capture_size[1]=camera_size[1];
				memcpy(tracker_config.calibration.intrinsics,camera_intr,sizeof(tracker_config.calibration.intrinsics));
				memcpy(tracker_config.calibration.distortion,camera_dist,sizeof(tracker_config.calibration.distortion));
				source_index =i;
			}
		}

	}
	// configure our tracker for this frame source
	bool configured = false;
	configured = md->tracker->tracker_configure(md->tracker,&tracker_config);

	if (! configured) {
		printf("ERROR: tracker rejected frameserver configuration!\n");
		return false;
	}

	// tracker is happy - connect our frameserver to our tracker
	md->frameservers[0]->frameserver_register_frame_callback(md->frameservers[0],md->tracker,md->tracker->tracker_queue);

	//create a filter for the trackers output
	opencv_filter_configuration_t filter_config = {};
	filter_config.measurement_noise_cov =0.1f;
	filter_config.process_noise_cov =0.1f;

	md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
	md->filter->filter_configure(md->filter,&filter_config);
	//connect our tracker to our filter
	md->tracker->tracker_register_measurement_callback(md->tracker,md->filter,md->filter->filter_queue);

	// now we can configure our frameserver and start the stream

	printf("INFO: frame source path: %s %d x %d interval: %d\n",&(descriptors[source_index].name), descriptors[source_index].width,descriptors[source_index].height,descriptors[source_index].format,descriptors[source_index].rate);
	md->frameservers[0]->frameserver_configure_capture(md->frameservers[0],md->tracker->tracker_get_capture_params(md->tracker));
	md->frameservers[0]->frameserver_stream_start(md->frameservers[0],&(descriptors[source_index]));

	return true;
}

bool mt_create_stereo_elp(mt_device_t* md) {

	// TODO - add IMU input source -> filter

	md->frameserver_count=1; // this driver uses a single, composite stereo, camera source
	md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
	// ask our frameserver for available sources - note this will return a
	// type-specific struct that we need to deal with e.g. UVC-specific, FFMPEG-specific.
	uint32_t source_count=0;
	md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],NULL,&source_count);
	if (source_count == 0){
		//we have no sources, we cannot continue
		return false;
	}
	uvc_source_descriptor_t* descriptors = calloc(source_count,sizeof(uvc_source_descriptor_t));
	md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],descriptors,&source_count);
	// defer further configuration and stream start until the rest of our chain is set up.

	md->tracker = tracker_create(TRACKER_TYPE_SPHERE_STEREO);
	tracker_stereo_configuration_t tracker_config = {};

	// configure our ELP camera when we find it during enumeration
	uint32_t source_index; //our frameserver config descriptor index - we would have an array for multiple devices
	for (uint32_t i=0; i< source_count;i++){
		uvc_source_descriptor_t s = descriptors[i];
		if (descriptors[i].product_id == 0x9750 && descriptors[i].vendor_id == 0x05a3 && descriptors[i].format == FORMAT_Y_UINT8) {
			if (descriptors[i].width == 1280 && descriptors[i].height == 480 && descriptors[i].rate == 166666) {
				tracker_config.l_format = descriptors[i].format;
				tracker_config.l_source_id =descriptors[i].source_id;

				//TODO: check if we have saved calibration data for this device
				//if not, we need to set the tracker to calibration mode
				tracker_config.calibration_mode = CALIBRATION_MODE_CHESSBOARD;

				//50/50 horizontal split - may need to put this in calibration data

				struct xrt_vec2 ltl = {0.0f,0.0f};
				struct xrt_vec2 lbr = {descriptors[i].width / 2.0f,descriptors[i].height};
				struct xrt_vec2 rtl = {descriptors[i].width / 2.0f,0.0f};
				struct xrt_vec2 rbr = {descriptors[i].width ,descriptors[i].height};

				tracker_config.l_rect.tl=ltl;
				tracker_config.l_rect.br=lbr;
				tracker_config.r_rect.tl=rtl;
				tracker_config.r_rect.br=rbr;

				tracker_config.split_left = true;

				source_index =i;
			}
		}

	}
	// configure our tracker for this frame source
	bool configured = false;
	configured = md->tracker->tracker_configure(md->tracker,&tracker_config);

	if (! configured) {
		printf("ERROR: tracker rejected frameserver configuration!\n");
		return false;
	}

	// tracker is happy - connect our frameserver to our tracker
	md->frameservers[0]->frameserver_register_frame_callback(md->frameservers[0],md->tracker,md->tracker->tracker_queue);

	//create a filter for the trackers output
	opencv_filter_configuration_t filter_config = {};
	filter_config.measurement_noise_cov =0.1f;
	filter_config.process_noise_cov =0.1f;

	md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
	md->filter->filter_configure(md->filter,&filter_config);
	//connect our tracker to our filter
	md->tracker->tracker_register_measurement_callback(md->tracker,md->filter,md->filter->filter_queue);

	// now we can configure our frameserver and start the stream

	printf("INFO: frame source path: %s %d x %d interval: %d\n",&(descriptors[source_index].name), descriptors[source_index].width,descriptors[source_index].height,descriptors[source_index].format,descriptors[source_index].rate);
	md->frameservers[0]->frameserver_configure_capture(md->frameservers[0],md->tracker->tracker_get_capture_params(md->tracker));
	md->frameservers[0]->frameserver_stream_start(md->frameservers[0],&(descriptors[source_index]));


	return true;
}




void dummy_init_mt_device(mt_device_t* md){
	//if this stuff isn't filled in we crash.

	md->base.destroy = mt_device_destroy;
	md->base.get_view_pose = mt_device_get_view_pose;
	md->base.get_tracked_pose = mt_device_get_tracked_pose;
	md->base.blend_mode=XRT_BLEND_MODE_OPAQUE;
	md->base.screens[0].w_pixels = 512;
	md->base.screens[0].h_pixels = 256;
	md->base.screens[0].nominal_frame_interval_ns = 11000;
	md->base.views[0].viewport.w_pixels = 256;
	md->base.views[0].viewport.h_pixels = 256;
	md->base.views[0].viewport.x_pixels=0;
	md->base.views[0].viewport.x_pixels=0;
	md->base.views[1].viewport.w_pixels = 256;
	md->base.views[1].viewport.h_pixels = 256;
	md->base.views[1].viewport.x_pixels= 256;
	md->base.views[1].viewport.y_pixels= 0;
	md->base.views[0].display.w_pixels = 256;
	md->base.views[0].display.w_meters = 0.1f;
	md->base.views[0].display.h_pixels=256;
	md->base.views[0].display.h_meters=0.1f;
	md->base.views[1].display.w_pixels = 256;
	md->base.views[1].display.w_meters = 0.1f;
	md->base.views[1].display.h_pixels=256;
	md->base.views[1].display.h_meters=0.1f;
}

