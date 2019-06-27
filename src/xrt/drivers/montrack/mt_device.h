// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to internal Monado driver code.
 * @author Pete Black <pete.black@collabora.com>
 */

#pragma once

#include "math/m_api.h"
#include "xrt/xrt_device.h"

#include "optical_tracking/common/tracker.h"
#include "frameservers/common/frameserver.h"
#include "filters/common/filter.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct frameserver_model {
    char model_name[128];
    uint32_t vendor_id;
    uint32_t product_id;
    char driver_name[128];
} frameserver_model_t;

typedef struct frameserver_config_request{
    uint32_t width;
    uint32_t height;
    uint32_t fps;
} frameserver_config_request_t;

typedef struct mt_device
{
	struct xrt_device base;
    uint32_t frameserver_count;
    frameserver_instance_t** frameservers;
	tracker_instance_t* tracker;
	// TODO: merge these configurations to be descriptive of
	// n-source trackers
	tracker_mono_configuration_t config_mono;
	tracker_stereo_configuration_t config_stereo;
	filter_instance_t* filter;
	bool log_verbose;
	bool log_debug;
} mt_device_t;

XRT_MAYBE_UNUSED static inline mt_device_t*
mt_device(struct xrt_device* xdev)
{
	return (mt_device_t*)xdev;
}

mt_device_t*
mt_device_create(char* device_name, bool log_verbose, bool log_debug);

frameserver_instance_t*
mt_frameserver_create(char* model,frameserver_config_request_t config_req, bool log_verbose, bool log_debug);

static bool
mt_create_calibration_stereo(
    mt_device_t* md); // stereo tracker for calibration

void
mt_handle_event(mt_device_t* md, driver_event_t e);

void
dummy_init_mt_device(mt_device_t* md);

static void
mt_device_update_inputs(struct xrt_device *xdev,
                          struct time_state *timekeeping);


#ifdef __cplusplus
}
#endif
