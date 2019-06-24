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


struct ffmpeg_source_descriptor
{
	char name[128];
	char* filepath;
	uint64_t source_id;
	uint32_t current_frame;
	uint32_t frame_count;
	enum fs_frame_format format;
	uint32_t width;
	uint32_t height;
};

struct frameserver*
ffmpeg_frameserver_create();

bool
ffmpeg_frameserver_test();
