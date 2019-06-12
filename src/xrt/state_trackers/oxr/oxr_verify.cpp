// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  File for verifing app input into api functions.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_main
 * @ingroup oxr_api
 */

#include <cstdio>
#include <cstring>

#include "xrt/xrt_compiler.h"
#include "util/u_debug.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_api_verify.h"
#include "oxr_chain.h"


/*
 *
 * Path verification.
 *
 */

static bool
valid_path_char(const char c)
{
	if ('a' <= c && c <= 'z') {
		return true;
	}

	if ('0' <= c && c <= '9') {
		return true;
	}

	if (c == '-' || c == '_' || c == '.' || c == '/') {
		return true;
	}

	return false;
}

static bool
contains_zero(const char* path, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++) {
		if (path[i] == '\0') {
			return true;
		}
	}

	return false;
}

extern "C" XrResult
oxr_verify_fixed_size_single_level_path(struct oxr_logger* log,
                                        const char* path,
                                        uint32_t array_size,
                                        const char* name)
{
	if (array_size == 0) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
		                 "(%s) internal runtime error", name);
	}

	if (path[0] == '\0') {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) can not be empty", name);
	}

	if (!contains_zero(path, array_size)) {
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) must include zero termination '\\0'.",
		                 name);
	}

	size_t length = strlen(path);
	for (size_t i = 0; i < length; i++) {
		const char c = path[i];

		// Slashes are not valid in single level paths.
		if (valid_path_char(c) && c != '/') {
			continue;
		}

		return oxr_error(
		    log, XR_ERROR_PATH_FORMAT_INVALID,
		    "(%s) 0x%02x is not a valid character at position %u", name,
		    c, (uint32_t)i);
	}

	return XR_SUCCESS;
}

extern "C" XrResult
oxr_verify_localized_name(struct oxr_logger* log,
                          const char* string,
                          uint32_t array_size,
                          const char* name)
{
	if (array_size == 0) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
		                 "(%s) internal runtime error", name);
	}

	if (string[0] == '\0') {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) can not be empty", name);
	}

	if (!contains_zero(string, array_size)) {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) must include zero termination '\\0'.",
		                 name);
	}

	// Future work: validate well-formed UTF-8?
	return XR_SUCCESS;
}

enum class State
{
	Start,
	Middle,
	Slash,
	SlashDots,
};

extern "C" XrResult
oxr_verify_full_path_c(struct oxr_logger* log,
                       const char* path,
                       const char* name)
{
	// XR_MAX_PATH_LENGTH is max including null terminator,
	// length will not include null terminator
	size_t length = XR_MAX_PATH_LENGTH;
	for (size_t i = 0; i < XR_MAX_PATH_LENGTH; i++) {
		if (path[i] == '\0') {
			length = i;
			break;
		}
	}

	return oxr_verify_full_path(log, path, (uint32_t)length, name);
}

extern "C" XrResult
oxr_verify_full_path(struct oxr_logger* log,
                     const char* path,
                     size_t length,
                     const char* name)
{
	State state = State::Start;
	bool valid = true;

	if (length >= XR_MAX_PATH_LENGTH) {
		char formatted_path[XR_MAX_PATH_LENGTH + 6];
		snprintf(formatted_path, XR_MAX_PATH_LENGTH + 6, "%s[...]",
		         path);
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) is too long for a path, must be shorter "
		                 "than %u characters",
		                 name, XR_MAX_PATH_LENGTH);
	}

	for (uint32_t i = 0; i < length; i++) {
		const char c = path[i];
		switch (state) {
		case State::Start:
			if (c != '/') {
				return oxr_error(log,
				                 XR_ERROR_PATH_FORMAT_INVALID,
				                 "(%s) does not start with a "
				                 "fowrward slash",
				                 name);
			}
			state = State::Slash;
			break;
		case State::Slash:
			switch (c) {
			case '.':
				// Is valid and starts the SlashDot(s) state.
				state = State::SlashDots;
				break;
			case '/':
				return oxr_error(
				    log, XR_ERROR_PATH_FORMAT_INVALID,
				    "(%s) '//' is not a valid in a path", name);
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		case State::Middle:
			switch (c) {
			case '/': state = State::Slash; break;
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		case State::SlashDots:
			switch (c) {
			case '/':
				return oxr_error(
				    log, XR_ERROR_PATH_FORMAT_INVALID,
				    "(%s) '/.[.]*/' is not a valid in a path",
				    name);
			case '.':
				// It's valid, more ShashDot(s).
				break;
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		}

		if (!valid) {
			return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
			                 "(%s) 0x%02x is not a valid character "
			                 "at position %u",
			                 name, c, (uint32_t)length);
		}
	}

	switch (state) {
	case State::Start:
		// Empty string
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) a empty string is not a valid path",
		                 name);
	case State::Slash:
		// Is this '/foo/' or '/'
		if (length > 1) {
			// It was '/foo/'
			return XR_SUCCESS;
		}
		// It was '/'
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) the string '%s' is not a valid path",
		                 name, path);
	case State::SlashDots:
		// Does the path ends with '/..'
		return oxr_error(
		    log, XR_ERROR_PATH_FORMAT_INVALID,
		    "(%s) strings ending with '/.[.]*' is not a valid", name);

	case State::Middle:
		// '/foo/bar' okay!
		return XR_SUCCESS;
	default:
		// We should not end up here.
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "(%s) internal runtime error validating path (%s)", name,
		    path);
	}
}


/*
 *
 * Subaction path functions.
 *
 */

static XrResult
subaction_path_no_dups(struct oxr_logger* log,
                       struct oxr_instance* inst,
                       struct oxr_sub_paths& sub_paths,
                       XrPath path,
                       const char* variable,
                       uint32_t index)
{
	bool dub = false;

	if (path == XR_NULL_PATH) {
		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(%s[%u] == XR_NULL_PATH) not a "
		                 "valid subaction path.",
		                 variable, index);
	} else if (path == inst->path_cache.user) {
		if (sub_paths.user) {
			dub = true;
		} else {
			sub_paths.user = true;
		}
	} else if (path == inst->path_cache.head) {
		if (sub_paths.head) {
			dub = true;
		} else {
			sub_paths.head = true;
		}
	} else if (path == inst->path_cache.left) {
		if (sub_paths.left) {
			dub = true;
		} else {
			sub_paths.left = true;
		}
	} else if (path == inst->path_cache.right) {
		if (sub_paths.right) {
			dub = true;
		} else {
			sub_paths.right = true;
		}
	} else if (path == inst->path_cache.gamepad) {
		if (sub_paths.gamepad) {
			dub = true;
		} else {
			sub_paths.gamepad = true;
		}
	} else {
		const char* str = NULL;
		size_t length = 0;

		oxr_path_get_string(log, inst, path, &str, &length);
		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(%s[%u] == '%s') path is not a "
		                 "valid subaction path.",
		                 variable, index, str);
	}

	if (dub) {
		const char* str = NULL;
		size_t length = 0;

		oxr_path_get_string(log, inst, path, &str, &length);

		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(%s[%u] == '%s') dublicate paths", variable,
		                 index, str);
	}

	return XR_SUCCESS;
}


extern "C" XrResult
oxr_verify_subaction_paths_create(struct oxr_logger* log,
                                  struct oxr_instance* inst,
                                  uint32_t countSubactionPaths,
                                  const XrPath* subactionPaths,
                                  const char* variable)
{
	struct oxr_sub_paths sub_paths = {};

	for (uint32_t i = 0; i < countSubactionPaths; i++) {
		XrPath path = subactionPaths[i];

		XrResult ret = subaction_path_no_dups(log, inst, sub_paths,
		                                      path, variable, i);
		if (ret != XR_SUCCESS) {
			return ret;
		}
	}

	return XR_SUCCESS;
}

extern "C" XrResult
oxr_verify_subaction_path_sync(struct oxr_logger* log,
                               struct oxr_instance* inst,
                               XrPath path,
                               uint32_t index)
{
	if (path == XR_NULL_PATH || path == inst->path_cache.user ||
	    path == inst->path_cache.head || path == inst->path_cache.left ||
	    path == inst->path_cache.right ||
	    path == inst->path_cache.gamepad) {
		return XR_SUCCESS;
	} else {
		const char* str = NULL;
		size_t length = 0;

		oxr_path_get_string(log, inst, path, &str, &length);
		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(actionSets[%i].subactionPath == '%s') path "
		                 "is not a valid subaction path.",
		                 index, str);
	}

	return XR_SUCCESS;
}

extern "C" XrResult
oxr_verify_subaction_path_get(struct oxr_logger* log,
                              struct oxr_instance* inst,
                              XrPath path,
                              const struct oxr_sub_paths* act_sub_paths,
                              struct oxr_sub_paths* out_sub_paths,
                              const char* variable)
{
	struct oxr_sub_paths sub_paths = {};

	if (path == XR_NULL_PATH) {
		sub_paths.any = true;
	} else if (path == inst->path_cache.user) {
		sub_paths.user = true;
	} else if (path == inst->path_cache.head) {
		sub_paths.head = true;
	} else if (path == inst->path_cache.left) {
		sub_paths.left = true;
	} else if (path == inst->path_cache.right) {
		sub_paths.right = true;
	} else if (path == inst->path_cache.gamepad) {
		sub_paths.gamepad = true;
	} else {
		const char* str = NULL;
		size_t length = 0;

		oxr_path_get_string(log, inst, path, &str, &length);
		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(%s == '%s') path is not "
		                 "a valid subaction path.",
		                 variable, str);
	}

	if ((sub_paths.user && !act_sub_paths->user) ||
	    (sub_paths.head && !act_sub_paths->head) ||
	    (sub_paths.left && !act_sub_paths->left) ||
	    (sub_paths.right && !act_sub_paths->right) ||
	    (sub_paths.gamepad && !act_sub_paths->gamepad)) {
		const char* str = NULL;
		size_t length = 0;

		oxr_path_get_string(log, inst, path, &str, &length);

		return oxr_error(log, XR_ERROR_PATH_INVALID,
		                 "(%s == '%s') the subaction path was "
		                 "not specified at action creation",
		                 variable, str);
	}

	*out_sub_paths = sub_paths;

	return XR_SUCCESS;
}


/*
 *
 * Other verification.
 *
 */

extern "C" XrResult
oxr_verify_XrSessionCreateInfo(struct oxr_logger* log,
                               const struct oxr_instance* inst,
                               const XrSessionCreateInfo* createInfo)
{
	if (createInfo->type != XR_TYPE_SESSION_CREATE_INFO) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "(createInfo->type)");
	}

	XrResult result = oxr_system_verify_id(log, inst, createInfo->systemId);
	if (result != XR_SUCCESS) {
		return result;
	}

#ifdef XR_USE_PLATFORM_XLIB
	XrGraphicsBindingOpenGLXlibKHR const* opengl_xlib =
	    OXR_GET_INPUT_FROM_CHAIN(createInfo,
	                             XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR,
	                             XrGraphicsBindingOpenGLXlibKHR);
	if (opengl_xlib != NULL) {
		if (!inst->opengl_enable) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    " OpenGL "
			    "requires " XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
		}
		return oxr_verify_XrGraphicsBindingOpenGLXlibKHR(log,
		                                                 opengl_xlib);
	}
#endif

#ifdef XR_USE_GRAPHICS_API_VULKAN
	XrGraphicsBindingVulkanKHR const* vulkan = OXR_GET_INPUT_FROM_CHAIN(
	    createInfo, XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR,
	    XrGraphicsBindingVulkanKHR);
	if (vulkan != NULL) {
		if (!inst->vulkan_enable) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    " Vulkan "
			    "requires " XR_KHR_VULKAN_ENABLE_EXTENSION_NAME);
		}
		return oxr_verify_XrGraphicsBindingVulkanKHR(log, vulkan);
	}
#endif

	/*
	 * Add any new graphics binding structs here - before the headless
	 * check. (order for non-headless checks not specified in standard.)
	 * Add a new verify function below.
	 * Any new addition will also need to be added to
	 * oxr_session_create_impl.
	 */

	/* We didn't recognize any graphics binding structs in the chain - our
	 * last hope is headless. */

	if (inst->headless) {
		return XR_SUCCESS;
	}

	return oxr_error(log, XR_ERROR_GRAPHICS_DEVICE_INVALID,
	                 "(createInfo->next) Argument chain does not contain "
	                 "any known graphics bindings");
}


#ifdef XR_USE_PLATFORM_XLIB

extern "C" XrResult
oxr_verify_XrGraphicsBindingOpenGLXlibKHR(
    struct oxr_logger* log, const XrGraphicsBindingOpenGLXlibKHR* next)
{
	if (next->type != XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 " Graphics binding has invalid type");
	}

	return XR_SUCCESS;
}

#endif


#ifdef XR_USE_GRAPHICS_API_VULKAN

extern "C" XrResult
oxr_verify_XrGraphicsBindingVulkanKHR(struct oxr_logger* log,
                                      const XrGraphicsBindingVulkanKHR* next)
{
	if (next->type != XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 " Graphics binding has invalid type");
	}

	return XR_SUCCESS;
}

#endif
