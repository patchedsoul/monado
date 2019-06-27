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

#include "optical_tracking/tracker3D_sphere_mono.h"

//#IFDEF have_opencv
#include "filters/filter_opencv_kalman.h"
//#IFDEF have_uvc
#include "frameservers/uvc/uvc_frameserver.h"
//#IFDEF have_v4l2
#include "frameservers/v4l2/v4l2_frameserver.h"

#include "mt_framequeue.h"


static void
mt_device_destroy(struct xrt_device* xdev)
{
	mt_device_t* md = mt_device(xdev);


	free(md);
}

static void
mt_device_get_tracked_pose(struct xrt_device* xdev,
                           enum xrt_input_name name,
                           struct time_state* timekeeping,
                           int64_t* out_timestamp,
                           struct xrt_space_relation* out_relation)
{
	mt_device_t* md = mt_device(xdev);
	struct xrt_pose pose;
	filter_state_t filtered;
	switch (md->tracker->tracker_type) {
	case TRACKER_TYPE_SPHERE_MONO:
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		md->filter->filter_predict_state(md->filter, &filtered, 0);
		out_relation->pose = filtered.pose;
		break;
	case TRACKER_TYPE_SPHERE_STEREO:
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		md->filter->filter_predict_state(md->filter, &filtered, 0);
		out_relation->pose = filtered.pose;
		break;
	case TRACKER_TYPE_UVBI:
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		// TODO: get pose from uvbi tracker
		// out_relation->pose = filtered.pose;
		break;
    case TRACKER_TYPE_CALIBRATION_STEREO:
        out_relation->relation_flags = 0;
        break;
	default: printf("ERROR: Unknown tracker type\n");
	}


	// Update state within driver
	// md->last_update = *out_timestamp;
	// md->last_relation = *out_relation;
}

static void
mt_device_get_view_pose(struct xrt_device* xdev,
                        struct xrt_vec3* eye_relation,
                        uint32_t view_index,
                        struct xrt_pose* out_pose)
{
	struct xrt_pose pose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	*out_pose = pose;
}

void
mt_device_update_inputs(struct xrt_device *xdev,
                          struct time_state *timekeeping)
{
    struct mt_device* md = mt_device(xdev);
    //we can try and ref a frame, process it, and unref it.
    frame_queue_t* fq = frame_queue_instance();
    if (!md->tracker->configured) {
        //configure our tracker to work with our source - hardcoded to first source
        frame_t source_config = fq->source_frames[0];
        tracker_stereo_configuration_t tc;
        snprintf(tc.configuration_filename,256,"PETE");
        tc.l_format= source_config.format;
        tc.split_left=true;
        tc.l_source_id = 0;
        tc.l_rect.tl.x=0;
        tc.l_rect.tl.y=0;
        tc.l_rect.br.x=source_config.width/2;
        tc.l_rect.br.y=source_config.height;
        tc.r_rect.tl.x=source_config.width/2;
        tc.r_rect.tl.y=0;
        tc.r_rect.br.x=source_config.width;
        tc.r_rect.br.y=source_config.height;
        md->tracker->tracker_configure(md->tracker,&tc);
    }
    //TODO: needs threading
    /*frame_t* frame;
    printf("mt_device ref\n");
    frame = frame_queue_ref_latest(fq,0);
    if (frame){
        printf("mt device track\n");
        md->tracker->tracker_queue(md->tracker,frame);
        printf("mt_device unref\n");
        frame_queue_unref(fq,frame);
    }*/

}

frameserver_instance_t*
mt_frameserver_create(char* model,frameserver_config_request_t config_req, bool log_verbose, bool log_debug)
{

    // TODO: load our device db - for now we are hardcoding this to only use the ps4 camera.
    // TODO: find the closest configuration to our request

    frameserver_instance_t* fs =  frameserver_create(FRAMESERVER_TYPE_V4L2);
    uint32_t source_count;
    fs->frameserver_enumerate_sources(fs,NULL, &source_count);
    if (source_count == 0) {
        // we have no sources, we cannot continue
        return NULL;
    }
    v4l2_source_descriptor_t* descriptors =
        U_TYPED_ARRAY_CALLOC(v4l2_source_descriptor_t, source_count);
    fs->frameserver_enumerate_sources(
        fs, descriptors, &source_count);
    for (uint32_t i = 0; i < source_count; i++) {
        v4l2_source_descriptor_t s = descriptors[i];
        if (strcmp(s.model,"USB Camera-OV580: USB Camera-OV") == 0 && s.format == FORMAT_YUV444_UINT8) {
            if (s.width == 1748 && s.height == 408 && s.rate == 166666) {
                //fs->frameserver_stream_start(fs,&s);
                return fs;
            }
        }

    }
    return NULL;
}

mt_device_t*
mt_device_create(char* device_name, bool log_verbose, bool log_debug)
{
	mt_device_t* md = U_TYPED_CALLOC(mt_device_t);

	dummy_init_mt_device(md);

    if (strcmp(device_name, "CALIBRATION_STEREO") == 0) {
        if (mt_create_calibration_stereo(md)) {
            md->base.tracking = calloc(1,sizeof(struct xrt_tracking));
            snprintf(md->base.tracking->name,256,"CALIBRATION_STEREO");
            md->base.tracking->type = XRT_TRACKING_TYPE_NONE;
            return md;
        }
    }
	return NULL;
}


bool
mt_create_calibration_stereo(mt_device_t* md)
{
    md->tracker = tracker_create(TRACKER_TYPE_CALIBRATION_STEREO);
    md->tracker->tracker_register_event_callback(md->tracker,md,mt_handle_event);

}

void
mt_handle_event(mt_device_t* md, driver_event_t e)
{
	switch (e.type) {
	case EVENT_TRACKER_RECONFIGURED:
		switch (md->tracker->tracker_type) {
		case TRACKER_TYPE_SPHERE_STEREO:
		case TRACKER_TYPE_SPHERE_MONO:
			for (uint32_t i = 0; i < md->frameserver_count; i++) {
				md->frameservers[i]
				    ->frameserver_configure_capture(
				        md->frameservers[i],
				        md->tracker->tracker_get_capture_params(
				            md->tracker));
			}
			break;
		default: break;
		}
		break;
	default: break;
	}
}

void
dummy_init_mt_device(mt_device_t* md)
{
	// if this stuff isn't filled in we crash.

	md->base.destroy = mt_device_destroy;
	md->base.get_view_pose = mt_device_get_view_pose;
	md->base.get_tracked_pose = mt_device_get_tracked_pose;
    md->base.update_inputs = mt_device_update_inputs;

	/*
	        md->base.blend_mode = XRT_BLEND_MODE_OPAQUE;
	        md->base.screens[0].w_pixels = 512;
	        md->base.screens[0].h_pixels = 256;
	        md->base.screens[0].nominal_frame_interval_ns = 11000;
	        md->base.views[0].viewport.w_pixels = 256;
	        md->base.views[0].viewport.h_pixels = 256;
	        md->base.views[0].viewport.x_pixels = 0;
	        md->base.views[0].viewport.x_pixels = 0;
	        md->base.views[1].viewport.w_pixels = 256;
	        md->base.views[1].viewport.h_pixels = 256;
	        md->base.views[1].viewport.x_pixels = 256;
	        md->base.views[1].viewport.y_pixels = 0;
	        md->base.views[0].display.w_pixels = 256;
	        md->base.views[0].display.w_meters = 0.1f;
	        md->base.views[0].display.h_pixels = 256;
	        md->base.views[0].display.h_meters = 0.1f;
	        md->base.views[1].display.w_pixels = 256;
	        md->base.views[1].display.w_meters = 0.1f;
	        md->base.views[1].display.h_pixels = 256;
	        md->base.views[1].display.h_meters = 0.1f;
	*/
}
