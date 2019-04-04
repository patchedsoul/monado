// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header
 * @author Pete Black <pblack@collabora.com>
 */

#pragma once

#include "frameserver.h"

#ifdef __cplusplus
extern "C" {
#endif



// TODO: unify device descriptors across apis
struct uvc_source_descriptor
{
	char name[128];
	uint16_t vendor_id;
	uint16_t product_id;
	char serial[128];
	uint64_t source_id;
	uint32_t uvc_device_index;
	/*enum uvc_frame_format*/ int stream_format;
	enum fs_frame_format format;
	enum fs_sampling sampling;
	uint32_t width;
	uint32_t height;
	uint32_t rate;
};

struct frameserver*
uvc_frameserver_create();

#ifdef __cplusplus
}
#endif
