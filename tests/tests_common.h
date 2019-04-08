// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Functions used in several tests.
 * @author Christoph Haag <christop.haag@collabora.com>
 */

#pragma once

#include <check.h>

/* Several internal xrt headers use a void* definition of GLX types, so we have
 * to do it too. */
#ifdef X11_INCLUDES
#include <X11/Xlib.h>

#include <GL/glx.h>
#else
typedef void *Display;
typedef void *GLXFBConfig;
typedef void *GLXDrawable;
typedef void *GLXContext;
#endif

#include <vulkan/vulkan.h>

#define XR_USE_GRAPHICS_API_OPENGL
#define XR_USE_GRAPHICS_API_VULKAN
#define XR_USE_PLATFORM_XLIB

#include "openxr_includes/openxr.h"
#include "openxr_includes/openxr_platform.h"

#include "openxr_includes/loader_interfaces.h"

#include <xrt/xrt_defines.h>

#include <oxr/oxr_logger.h>

#include <oxr/oxr_api_funcs.h>
#include <oxr/oxr_api_verify.h>

void init_glx(Display **display, GLXDrawable *w, GLXContext *ctx);
void destroy_glx();
