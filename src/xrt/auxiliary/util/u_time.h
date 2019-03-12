// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Time-keeping: a clock that is steady, convertible to system time, and
 * ideally high-resolution.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 *
 * @see time_state
 */

#pragma once

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * Integer timestamp type.
 *
 * @see time_state
 */
typedef int64_t timepoint_ns;

/*!
 * @struct time_state util/u_time.h
 * @brief Time-keeping state structure.
 *
 * Exposed as an opaque pointer.
 *
 * @see timepoint_ns
 */
struct time_state;

/*!
 * Create a struct time_state.
 *
 * @public @memberof time_state
 */
struct time_state*
time_state_create();


/*!
 * Destroy a struct time_state.
 *
 * Should not be called simultaneously with any other time_state function.
 *
 * @public @memberof time_state
 */
void
time_state_destroy(struct time_state* state);

/*!
 * Get the current time as an integer timestamp.
 *
 * Does not update internal state for timekeeping.
 * Should not be called simultaneously with time_state_get_now_and_update.
 *
 * @public @memberof time_state
 */
timepoint_ns
time_state_get_now(struct time_state const* state);

/*!
 * Get the current time as an integer timestamp and update internal state.
 *
 * This should be called regularly, but only from one thread.
 * It updates the association between the timing sources.
 *
 * Should not be called simultaneously with any other time_state function.
 *
 * @public @memberof time_state
 */
timepoint_ns
time_state_get_now_and_update(struct time_state* state);

/*!
 * Convert an integer timestamp to a struct timespec (system time).
 *
 * Should not be called simultaneously with time_state_get_now_and_update.
 *
 * @public @memberof time_state
 */
void
time_state_to_timespec(struct time_state const* state,
                       timepoint_ns timestamp,
                       struct timespec* out);

/*!
 * Convert a struct timespec (system time) to an integer timestamp.
 *
 * Should not be called simultaneously with time_state_get_now_and_update.
 *
 * @public @memberof time_state
 */
timepoint_ns
time_state_from_timespec(struct time_state const* state,
                         const struct timespec* timespecTime);

#ifdef __cplusplus
}
#endif