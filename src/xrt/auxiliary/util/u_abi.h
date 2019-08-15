// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C ABI implementation helpers
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_util
 *
 */

#pragma once

#ifdef __cplusplus

#include <stdio.h>
#include <exception>

/*!
 * Wrapper over function-level try.
 *
 * Use with all functions exposing a C ABI and implemented in C++.
 */
#define XRT_ABI_TRY try
/*!
 * Wrapper over function-level catch with configurable return.
 *
 * Use this, or a related define, with all functions exposing a C ABI and
 * implemented in C++.
 */
#define XRT_ABI_CATCH_RETURN(RETVAL)                                           \
	catch (std::exception const &e)                                        \
	{                                                                      \
		fprintf(stderr, "%s: caught exception: %s\n", __func__,        \
		        e.what());                                             \
		return RETVAL;                                                 \
	}                                                                      \
	catch (...)                                                            \
	{                                                                      \
		fprintf(stderr, "%s: caught unrecognized exception\n",         \
		        __func__);                                             \
		return RETVAL;                                                 \
	}
/*!
 * Wrapper over function-level catch that returns false on an exception.
 *
 * Use this, or a related define, with all functions exposing a C ABI and
 * implemented in C++.
 */
#define XRT_ABI_CATCH_RETURN_FALSE XRT_ABI_CATCH_RETURN(false)

/*!
 * Wrapper over function-level catch that returns nothing.
 *
 * Use this, or a related define, with all functions exposing a C ABI and
 * implemented in C++.
 */
#define XRT_ABI_CATCH                                                          \
	catch (std::exception const &e)                                        \
	{                                                                      \
		fprintf(stderr, "%s: caught exception: %s\n", __func__,        \
		        e.what());                                             \
		return;                                                        \
	}                                                                      \
	catch (...)                                                            \
	{                                                                      \
		fprintf(stderr, "%s: caught unrecognized exception\n",         \
		        __func__);                                             \
		return;                                                        \
	}
#endif // __cplusplus
