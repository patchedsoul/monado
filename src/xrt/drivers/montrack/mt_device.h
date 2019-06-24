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


typedef struct mt_device
{
	struct xrt_device base;
	struct frameserver* frameservers[MAX_FRAMESERVERS];
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

XRT_MAYBE_UNUSED static inline mt_device_t*
mt_device(struct xrt_device* xdev)
{
	return (mt_device_t*)xdev;
}

mt_device_t*
mt_device_create(char* device_name, bool log_verbose, bool log_debug);

bool
mt_create_mono_ps3eye(mt_device_t* md); // mono blob tracker, ps3 60fps camera
bool
mt_create_mono_c270(
    mt_device_t* md); // mono blob tracker, logitech 30fps c270 camera
bool
mt_create_stereo_elp(
    mt_device_t* md); // stereo tracker, ELP 60fps stereo camera
bool
mt_create_uvbi_elp(mt_device_t* md); // uvbi tracker, ELP 60fps stereo camera
bool
mt_create_uvbi_hdk(mt_device_t* md); // uvbi tracker, OSVR HDK 100fps IR camera
bool
mt_create_stereo_ps4(
    mt_device_t* md); // stereo tracker, PS4 60fps stereo camera

void
mt_handle_event(mt_device_t* md, driver_event_t e);

void
dummy_init_mt_device(mt_device_t* md);


#ifdef __cplusplus
}
#endif
