// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header
 * @author Pete Black <pblack@collabora.com>
 */

#pragma once

#include "../common/frameserver.h"

#ifdef __cplusplus
extern "C" {
#endif


struct v4l2_source_descriptor
{
	char device_path[256]; // TODO: might not be enough
	char name[128];
	char model[128];
	uint64_t source_id;
	enum fs_frame_format format;
	uint32_t stream_format;
	enum fs_sampling sampling;
	uint32_t width;
	uint32_t height;
	uint32_t rate;
	uint8_t extended_format;
	uint32_t crop_scanline_bytes_start; // byte offset - special case for
	                                    // ps4 camera
	uint32_t crop_width; // pixels - special case for ps4 camera
};

struct frameserver*
v4l2_frameserver_create();
