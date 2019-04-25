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

#include "../optical_tracking/common/tracker.h"
#include "../frameservers/common/frameserver.h"
#include "../filters/common/filter.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef struct mt_device
{
	struct xrt_device base;
	frameserver_instance_t* frameservers[MAX_FRAMESERVERS];
	uint32_t frameserver_count;
	tracker_instance_t* tracker;
	// TODO: merge these configurations to be descriptive of
	// n-source trackers
	tracker_mono_configuration_t config_mono;
	tracker_stereo_configuration_t config_stereo;
	filter_instance_t* filter;
	bool log_verbose;
	bool log_debug;
} mt_device_t;

static inline mt_device_t*
mt_device(struct xrt_device *xdev)
{
	return (mt_device_t*) xdev;
}

mt_device_t *
mt_device_create(char* device_name,bool log_verbose, bool log_debug);



#ifdef __cplusplus
}
#endif
