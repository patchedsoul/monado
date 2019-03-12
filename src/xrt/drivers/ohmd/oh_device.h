// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to OpenHMD driver code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


struct oh_device
{
	struct xrt_device base;
	ohmd_context *ctx;
	ohmd_device *dev;

	bool skip_ang_vel;

	bool print_spew;
	bool print_debug;
};

static inline struct oh_device *
oh_device(struct xrt_device *xdev)
{
	return (struct oh_device *)xdev;
}

struct oh_device *
oh_device_create(ohmd_context *ctx,
                 ohmd_device *dev,
                 const char *prod,
                 bool print_spew,
                 bool print_debug);

#define OH_SPEW(c, ...)                                                        \
	do {                                                                   \
		if (c->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define OH_DEBUG(c, ...)                                                       \
	do {                                                                   \
		if (c->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define OH_ERROR(c, ...)                                                       \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)


#ifdef __cplusplus
}
#endif