// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Two call helper functions.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_main
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#define OXR_TWO_CALL_HELPER(log, cnt_input, cnt_output, output, count, data,   \
                            sval)                                              \
	do {                                                                   \
		if (cnt_output == NULL) {                                      \
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,     \
			                 #cnt_output);                         \
		}                                                              \
		*cnt_output = count;                                           \
                                                                               \
		if (cnt_input == 0) {                                          \
			return sval;                                           \
		}                                                              \
		if (cnt_input < count) {                                       \
			return oxr_error(log, XR_ERROR_SIZE_INSUFFICIENT,      \
			                 #cnt_input);                          \
		}                                                              \
		for (uint32_t i = 0; i < count; i++) {                         \
			(output)[i] = (data)[i];                               \
		}                                                              \
		return sval;                                                   \
	} while (false)

// The caller writes directly into the output memory and has to handle the
// cnt_input == 0 and count > cnt_input conditions
#define OXR_TWO_CALL_HELPER_DIRECT(log, cnt_input, cnt_output, count, sval)    \
	do {                                                                   \
		if (cnt_output == NULL) {                                      \
			return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,     \
			                 #cnt_output);                         \
		}                                                              \
		*cnt_output = count;                                           \
                                                                               \
		if (cnt_input == 0) {                                          \
			return sval;                                           \
		}                                                              \
		if (cnt_input < count) {                                       \
			return oxr_error(log, XR_ERROR_SIZE_INSUFFICIENT,      \
			                 #cnt_input);                          \
		}                                                              \
		return sval;                                                   \
	} while (false)

#ifdef __cplusplus
}
#endif
