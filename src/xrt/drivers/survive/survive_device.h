// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: Apache-2.0
/*!
 * @file
 * @brief  Interface to OpenHMD driver code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <survive_api.h>

struct survive_device
{
	struct xrt_device base;
	SurviveSimpleContext *ctx;

	bool print_spew;
	bool print_debug;
};

static inline struct survive_device *
survive_device(struct xrt_device *xdev)
{
	return (struct survive_device *)xdev;
}

struct survive_device *
survive_device_create(bool print_spew, bool print_debug);

#define SURVIVE_SPEW(c, ...)                                                   \
	do {                                                                   \
		if (c->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)
#define SURVIVE_DEBUG(c, ...)                                                  \
	do {                                                                   \
		if (c->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define SURVIVE_ERROR(c, ...)                                                  \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)

#ifdef __cplusplus
}
#endif
