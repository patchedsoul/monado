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

	switch (md->tracker->tracker_type) {
	    case TRACKER_TYPE_SPHERE_MONO:
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		    filter_state_t filtered;
			md->filter->filter_predict_state(md->filter,&filtered,0);
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


mt_device_t *
mt_device_create(char* device_name,bool log_verbose, bool log_debug) {
	mt_device_t* md = U_TYPED_CALLOC(mt_device_t);
	md->base.destroy = mt_device_destroy;
	return md;
}


