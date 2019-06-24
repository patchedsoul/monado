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



mt_device_t*
mt_device_create(char* device_name, bool log_verbose, bool log_debug)
{
	mt_device_t* md = U_TYPED_CALLOC(mt_device_t);

	dummy_init_mt_device(md);

	/*if (strcmp(device_name, "MONO_PS3EYE") == 0) {
	            if (mt_create_mono_ps3eye(md)) {
	                    return md;
	            }
	    }
	    if (strcmp(device_name, "MONO_LOGITECH_C270") == 0) {
	            if (mt_create_mono_c270(md)) {
	                    return md;
	            }
	    }
	    if (strcmp(device_name, "STEREO_ELP_60FPS") == 0) {
	            if (mt_create_stereo_elp(md)) {
	                    return md;
	            }
	    }
	    if (strcmp(device_name, "UVBI_ELP_60FPS") == 0) {
	            if (mt_create_uvbi_elp(md)) {
	                    return md;
	            }
	    }
	    if (strcmp(device_name, "UVBI_HDK") == 0) {
	            if (mt_create_uvbi_hdk(md)) {
	                    return md;
	            }
	}*/

	if (strcmp(device_name, "STEREO_PS4_60FPS") == 0) {
		if (mt_create_stereo_ps4(md)) {
			return md;
		}
	}


	return NULL;
}

/*bool
mt_create_mono_ps3eye(mt_device_t* md)
{
        md->frameserver_count = 1; // this driver uses a single camera source
        md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_V4L2);
        // ask our frameserver for available sources - note this will return a
        // type-specific struct that we need to deal with e.g. UVC-specific,
        // FFMPEG-specific.
        uint32_t source_count = 0;
        md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
                                                           NULL, &source_count);
        if (source_count == 0) {
                // we have no sources, we cannot continue
                return false;
        }
        v4l2_source_descriptor_t* descriptors =
            U_TYPED_ARRAY_CALLOC(v4l2_source_descriptor_t, source_count);
        md->frameservers[0]->frameserver_enumerate_sources(
            md->frameservers[0], descriptors, &source_count);
        // defer further configuration and stream start until the rest of our
        // chain is set up.

        md->tracker = tracker_create(TRACKER_TYPE_SPHERE_MONO);
        tracker_mono_configuration_t tracker_config = {};

        // start in calibration mode
        tracker_config.calibration_mode = CALIBRATION_MODE_CHESSBOARD;


        // configure our ps3 eye when we find it during enumeration
        uint32_t source_index; // our frameserver config descriptor index
        for (uint32_t i = 0; i < source_count; i++) {
                v4l2_source_descriptor_t temp = descriptors[i];
                if (strcmp(descriptors[i].name, "ov534") == 0 &&
                    descriptors[i].format == FORMAT_Y_UINT8) {
                        if (descriptors[i].width == 640 &&
                            descriptors[i].height == 480 &&
                            descriptors[i].rate == 166666) {
                                tracker_config.format = descriptors[i].format;
                                tracker_config.source_id =
                                    descriptors[i].source_id;
                                source_index = i;
                        }
                }
        }
        snprintf(tracker_config.configuration_filename, 128, "PS3eye_mono");

        // configure our tracker for this frame source
        bool configured = false;
        configured =
            md->tracker->tracker_configure(md->tracker, &tracker_config);

        if (!configured) {
                printf("ERROR: tracker rejected frameserver configuration!\n");
                return false;
        }

        // create a filter for the trackers output
        opencv_filter_configuration_t filter_config = {};
        filter_config.measurement_noise_cov = 0.1f;
        filter_config.process_noise_cov = 0.1f;

        md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
        md->filter->filter_configure(md->filter, &filter_config);
        // connect our tracker to our filter
        md->tracker->tracker_register_measurement_callback(
            md->tracker, md->filter, md->filter->filter_queue);
        // and our driver to tracker events
        md->tracker->tracker_register_event_callback(md->tracker, md,
                                                     mt_handle_event);

        // now we can configure our frameserver and start the stream

        printf("INFO: frame source path: %s %d x %d %d interval: %d\n",
               descriptors[source_index].name, descriptors[source_index].width,
               descriptors[source_index].height,
               descriptors[source_index].format,
               descriptors[source_index].rate);
        md->frameservers[0]->frameserver_configure_capture(
            md->frameservers[0],
            md->tracker->tracker_get_capture_params(md->tracker));
        md->frameservers[0]->frameserver_stream_start(
            md->frameservers[0], &(descriptors[source_index]));
        return true;
}

bool
mt_create_mono_c270(mt_device_t* md)
{
        // TODO - add IMU input source -> filter

        md->frameserver_count = 1; // this driver uses a single camera source
        md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
        // ask our frameserver for available sources - note this will return a
        // type-specific struct that we need to deal with e.g. UVC-specific,
        // FFMPEG-specific.
        uint32_t source_count = 0;
        md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
                                                           NULL, &source_count);
        if (source_count == 0) {
                // we have no sources, we cannot continue
                return false;
        }
        uvc_source_descriptor_t* descriptors =
            U_TYPED_ARRAY_CALLOC(uvc_source_descriptor_t, source_count);
        md->frameservers[0]->frameserver_enumerate_sources(
            md->frameservers[0], descriptors, &source_count);
        // defer further configuration and stream start until the rest of our
        // chain is set up.

        md->tracker = tracker_create(TRACKER_TYPE_SPHERE_MONO);
        tracker_mono_configuration_t tracker_config = {};

        // start in calibration mode
        tracker_config.calibration_mode = CALIBRATION_MODE_CHESSBOARD;


        // configure our logitech c270 when we find it during enumeration
        uint32_t source_index; // our frameserver config descriptor index
        for (uint32_t i = 0; i < source_count; i++) {
                if (descriptors[i].product_id == 0x0825 &&
                    descriptors[i].vendor_id == 0x046d &&
                    descriptors[i].format == FORMAT_Y_UINT8) {
                        if (descriptors[i].width == 640 &&
                            descriptors[i].height == 480 &&
                            descriptors[i].rate == 333333) {
                                tracker_config.format = descriptors[i].format;
                                tracker_config.source_id =
                                    descriptors[i].source_id;
                                source_index = i;
                        }
                }
        }
        snprintf(tracker_config.configuration_filename, 128, "C270_mono_%s",
                 descriptors[source_index].serial);

        // configure our tracker for this frame source
        bool configured = false;
        configured =
            md->tracker->tracker_configure(md->tracker, &tracker_config);

        if (!configured) {
                printf("ERROR: tracker rejected frameserver configuration!\n");
                return false;
        }

        // create a filter for the trackers output
        opencv_filter_configuration_t filter_config = {};
        filter_config.measurement_noise_cov = 0.1f;
        filter_config.process_noise_cov = 0.1f;

        md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
        md->filter->filter_configure(md->filter, &filter_config);
        // connect our tracker to our filter
        md->tracker->tracker_register_measurement_callback(
            md->tracker, md->filter, md->filter->filter_queue);
        // and our driver to tracker events
        md->tracker->tracker_register_event_callback(md->tracker, md,
                                                     mt_handle_event);

        // now we can configure our frameserver and start the stream

        printf("INFO: frame source path: %s %d x %d format: %d, interval: %d\n",
               descriptors[source_index].name, descriptors[source_index].width,
               descriptors[source_index].height,
               descriptors[source_index].format,
               descriptors[source_index].rate);
        md->frameservers[0]->frameserver_configure_capture(
            md->frameservers[0],
            md->tracker->tracker_get_capture_params(md->tracker));
        md->frameservers[0]->frameserver_stream_start(
            md->frameservers[0], &(descriptors[source_index]));

        return true;
}

bool
mt_create_stereo_elp(mt_device_t* md)
{

        // TODO - add IMU input source -> filter

        md->frameserver_count =
            1; // this driver uses a single, composite stereo, camera source
        md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
        // ask our frameserver for available sources - note this will return a
        // type-specific struct that we need to deal with e.g. UVC-specific,
        // FFMPEG-specific.
        uint32_t source_count = 0;
        md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
                                                           NULL, &source_count);
        if (source_count == 0) {
                // we have no sources, we cannot continue
                return false;
        }
        uvc_source_descriptor_t* descriptors =
            U_TYPED_ARRAY_CALLOC(uvc_source_descriptor_t, source_count);
        md->frameservers[0]->frameserver_enumerate_sources(
            md->frameservers[0], descriptors, &source_count);
        // defer further configuration and stream start until the rest of our
        // chain is set up.

        md->tracker = tracker_create(TRACKER_TYPE_SPHERE_STEREO);
        tracker_stereo_configuration_t tracker_config = {0};

        // configure our ELP camera when we find it during enumeration
        uint32_t source_index; // our frameserver config descriptor index - we
                               // would have an array for multiple devices
        for (uint32_t i = 0; i < source_count; i++) {
                uvc_source_descriptor_t s = descriptors[i];
                if (descriptors[i].product_id == 0x9750 &&
                    descriptors[i].vendor_id == 0x05a3 &&
                    descriptors[i].format == FORMAT_YUV444_UINT8) {
                        if (descriptors[i].width == 1280 &&
                            descriptors[i].height == 480 &&
                            descriptors[i].rate == 166666) {
                                tracker_config.l_format = descriptors[i].format;
                                tracker_config.l_source_id =
                                    descriptors[i].source_id;
                                const char* serial = descriptors[i].serial;
                                snprintf(tracker_config.configuration_filename,
                                         128, "ELP_60FPS_stereo_%s", serial);


                                // start in calibration mode

                                tracker_config.calibration_mode =
                                    CALIBRATION_MODE_CHESSBOARD;

                                // set up 50/50 horizontal stereo split - may
                                // need to put this in calibration data

                                struct xrt_vec2 ltl = {0.0f, 0.0f};
                                struct xrt_vec2 lbr = {descriptors[i].width /
                                                           2.0f,
                                                       descriptors[i].height};
                                struct xrt_vec2 rtl = {
                                    descriptors[i].width / 2.0f, 0.0f};
                                struct xrt_vec2 rbr = {descriptors[i].width,
                                                       descriptors[i].height};

                                tracker_config.l_rect.tl = ltl;
                                tracker_config.l_rect.br = lbr;
                                tracker_config.r_rect.tl = rtl;
                                tracker_config.r_rect.br = rbr;

                                tracker_config.split_left = true;

                                source_index = i;
                        }
                }
        }
        // configure our tracker for this frame source
        bool configured =
            md->tracker->tracker_configure(md->tracker, &tracker_config);

        if (!configured) {
                printf("ERROR: tracker rejected frameserver configuration!\n");
                return false;
        }


        // create a filter for the trackers output
        opencv_filter_configuration_t filter_config = {0};
        filter_config.measurement_noise_cov = 0.1f;
        filter_config.process_noise_cov = 0.1f;

        md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
        md->filter->filter_configure(md->filter, &filter_config);

        // connect our tracker to our filter
        md->tracker->tracker_register_measurement_callback(
            md->tracker, md->filter, md->filter->filter_queue);
        md->tracker->tracker_register_event_callback(md->tracker, md,
                                                     mt_handle_event);

        // nw our chain is setup up we can start streaming data through it
        printf("INFO: frame source path: %s %d x %d format: %d, interval: %d\n",
               descriptors[source_index].name, descriptors[source_index].width,
               descriptors[source_index].height,
               descriptors[source_index].format,
               descriptors[source_index].rate);
        md->frameservers[0]->frameserver_configure_capture(
            md->frameservers[0],
            md->tracker->tracker_get_capture_params(md->tracker));
        md->frameservers[0]->frameserver_stream_start(
            md->frameservers[0], &(descriptors[source_index]));

        return true;
}


bool
mt_create_uvbi_elp(mt_device_t* md)
{

        // TODO - add IMU input source -> filter

        md->frameserver_count =
            1; // this driver uses a single, mono camera source
        md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
        // ask our frameserver for available sources - note this will return a
        // type-specific struct that we need to deal with e.g. UVC-specific,
        // FFMPEG-specific.
        uint32_t source_count = 0;
        md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
                                                           NULL, &source_count);
        if (source_count == 0) {
                // we have no sources, we cannot continue
                return false;
        }
        uvc_source_descriptor_t* descriptors =
            U_TYPED_ARRAY_CALLOC(uvc_source_descriptor_t, source_count);
        md->frameservers[0]->frameserver_enumerate_sources(
            md->frameservers[0], descriptors, &source_count);
        // defer further configuration and stream start until the rest of our
        // chain is set up.

        md->tracker = tracker_create(TRACKER_TYPE_UVBI);
        tracker_mono_configuration_t tracker_config = {0};

        // configure our ELP camera when we find it during enumeration
        uint32_t source_index; // our frameserver config descriptor index - we
                               // would have an array for multiple devices
        for (uint32_t i = 0; i < source_count; i++) {
                uvc_source_descriptor_t s = descriptors[i];
                if (descriptors[i].product_id == 0x9750 &&
                    descriptors[i].vendor_id == 0x05a3 &&
                    descriptors[i].format == FORMAT_Y_UINT8) {
                        if (descriptors[i].width == 1280 &&
                            descriptors[i].height == 480 &&
                            descriptors[i].rate == 166666) {
                                tracker_config.format = descriptors[i].format;
                                tracker_config.source_id =
                                    descriptors[i].source_id;
                                snprintf(tracker_config.configuration_filename,
                                         128, "ELP_60FPS_uvbi_%s",
                                         descriptors[i].serial);
                                source_index = i;
                        }
                }
        }
        // configure our tracker for this frame source
        bool configured =
            md->tracker->tracker_configure(md->tracker, &tracker_config);

        if (!configured) {
                printf("ERROR: tracker rejected frameserver configuration!\n");
                return false;
        }

        // now our chain is setup up we can start streaming data through it
        printf(
            "INFO: frame source path: %s %d x %d interval: %d\n",
            &(descriptors[source_index].name), descriptors[source_index].width,
            descriptors[source_index].height, descriptors[source_index].format,
            descriptors[source_index].rate);
        md->frameservers[0]->frameserver_configure_capture(
            md->frameservers[0],
            md->tracker->tracker_get_capture_params(md->tracker));
        md->frameservers[0]->frameserver_stream_start(
            md->frameservers[0], &(descriptors[source_index]));

        return true;
}
bool
mt_create_uvbi_hdk(mt_device_t* md)
{

        // TODO - add IMU input source -> filter

        md->frameserver_count =
            1; // this driver uses a single, mono camera source
        md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_UVC);
        // ask our frameserver for available sources - note this will return a
        // type-specific struct that we need to deal with e.g. UVC-specific,
        // FFMPEG-specific.
        uint32_t source_count = 0;
        md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
                                                           NULL, &source_count);
        if (source_count == 0) {
                // we have no sources, we cannot continue
                return false;
        }
        uvc_source_descriptor_t* descriptors =
            U_TYPED_ARRAY_CALLOC(uvc_source_descriptor_t, source_count);
        md->frameservers[0]->frameserver_enumerate_sources(
            md->frameservers[0], descriptors, &source_count);
        // defer further configuration and stream start until the rest of our
        // chain is set up.

        md->tracker = tracker_create(TRACKER_TYPE_UVBI);
        tracker_mono_configuration_t tracker_config = {};

        uint32_t source_index;
        for (uint32_t i = 0; i < source_count; i++) {
                uvc_source_descriptor_t s = descriptors[i];

                if (s.product_id == 0x57e8 && s.vendor_id == 0x0bda &&
                    s.format == FORMAT_JPG) {
                        if (s.width == 640 && s.height == 480) {
                                tracker_config.format = s.format;
                                tracker_config.source_id = s.source_id;
                                snprintf(tracker_config.configuration_filename,
                                         128, "HDK_uvbi_%s", s.serial);
                                source_index = i;
                        }
                }
        }
        // configure our tracker for this frame source
        bool configured =
            md->tracker->tracker_configure(md->tracker, &tracker_config);

        if (!configured) {
                printf("ERROR: tracker rejected frameserver configuration!\n");
                return false;
        }


        // now our chain is setup up we can start streaming data through it
        printf(
            "INFO: frame source path: %s %d x %d format: %d rate: %d\n",
            &(descriptors[source_index].name), descriptors[source_index].width,
            descriptors[source_index].height, descriptors[source_index].format,
            descriptors[source_index].rate);
        md->frameservers[0]->frameserver_configure_capture(
            md->frameservers[0],
            md->tracker->tracker_get_capture_params(md->tracker));
        md->frameservers[0]->frameserver_stream_start(
            md->frameservers[0], &(descriptors[source_index]));

        return true;
}
*/
bool
mt_create_stereo_ps4(mt_device_t* md)
{

	// TODO - add IMU input source -> filter

	md->frameserver_count =
	    1; // this driver uses a single, composite stereo, camera source
	md->frameservers[0] = frameserver_create(FRAMESERVER_TYPE_V4L2);
	// ask our frameserver for available sources - note this will return a
	// type-specific struct that we need to deal with e.g. UVC-specific,
	// FFMPEG-specific.
	uint32_t source_count = 0;
	md->frameservers[0]->frameserver_enumerate_sources(md->frameservers[0],
	                                                   NULL, &source_count);
	if (source_count == 0) {
		// we have no sources, we cannot continue
		return false;
	}
	v4l2_source_descriptor_t* descriptors =
	    U_TYPED_ARRAY_CALLOC(v4l2_source_descriptor_t, source_count);
	md->frameservers[0]->frameserver_enumerate_sources(
	    md->frameservers[0], descriptors, &source_count);
	// defer further configuration and stream start until the rest of our
	// chain is set up.

	md->tracker = tracker_create(TRACKER_TYPE_SPHERE_STEREO);
	tracker_stereo_configuration_t tracker_config = {};

	// configure our PS4 camera when we find it during enumeration
	uint32_t source_index; // our frameserver config descriptor index - we
	                       // would have an array for multiple devices
	for (uint32_t i = 0; i < source_count; i++) {
		v4l2_source_descriptor_t s = descriptors[i];
		if (strcmp(descriptors[i].model,
		           "USB Camera-OV580: USB "
		           "Camera-OV                                          "
		           "                           ") == 0 &&
		    descriptors[i].format == FORMAT_YUV444_UINT8) {
			if (descriptors[i].width == 1748 &&
			    descriptors[i].height == 408 &&
			    descriptors[i].rate == 166666) {
				tracker_config.l_format = descriptors[i].format;
				tracker_config.l_source_id =
				    descriptors[i].source_id;
				snprintf(tracker_config.configuration_filename,
				         128, "PS4_60FPS_stereo_%s",
				         descriptors[i].model);


				// start in calibration mode

				tracker_config.calibration_mode =
				    CALIBRATION_MODE_CHESSBOARD;

				// set up 50/50 horizontal stereo split - may
				// need to put this in calibration data
				uint32_t effective_width = descriptors[i].width;
				if (descriptors[i].crop_width > 0) {
					effective_width =
					    descriptors[i].crop_width;
				}

				struct xrt_vec2 ltl = {0.0f, 0.0f};
				struct xrt_vec2 lbr = {effective_width / 2.0f,
				                       descriptors[i].height};
				struct xrt_vec2 rtl = {effective_width / 2.0f,
				                       0.0f};
				struct xrt_vec2 rbr = {effective_width,
				                       descriptors[i].height};

				tracker_config.l_rect.tl = ltl;
				tracker_config.l_rect.br = lbr;
				tracker_config.r_rect.tl = rtl;
				tracker_config.r_rect.br = rbr;

				tracker_config.split_left = true;

				source_index = i;
			}
		}
	}
	// configure our tracker for this frame source
	bool configured =
	    md->tracker->tracker_configure(md->tracker, &tracker_config);

	if (!configured) {
		printf("ERROR: tracker rejected frameserver configuration!\n");
		return false;
	}


	// create a filter for the trackers output
	opencv_filter_configuration_t filter_config = {};
	filter_config.measurement_noise_cov = 0.1f;
	filter_config.process_noise_cov = 0.1f;

	md->filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
	md->filter->filter_configure(md->filter, &filter_config);

	// connect our tracker to our filter
	md->tracker->tracker_register_measurement_callback(
	    md->tracker, md->filter, md->filter->filter_queue);
	md->tracker->tracker_register_event_callback(md->tracker, md,
	                                             mt_handle_event);

	// nw our chain is setup up we can start streaming data through it
	printf(
	    "INFO: frame source path: %s %d x %d format: %d rate: %d\n",
	    &(descriptors[source_index].name), descriptors[source_index].width,
	    descriptors[source_index].height, descriptors[source_index].format,
	    descriptors[source_index].rate);
	md->frameservers[0]->frameserver_configure_capture(
	    md->frameservers[0],
	    md->tracker->tracker_get_capture_params(md->tracker));
	md->frameservers[0]->frameserver_stream_start(
	    md->frameservers[0], &(descriptors[source_index]));

	return true;
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
