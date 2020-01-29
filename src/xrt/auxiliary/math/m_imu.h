// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IMU helper struct.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"


#ifdef __cplusplus
extern "C" {
#endif


/*!
 * This is a common IMU pre-filter which takes raw 'ticks' from a 3 axis IMU
 * measurement and converts it into degrees per secs and meters per floats.
 *
 * One of these is used per gyro, accelerometer and magnometer.
 *
 * The formula used is: `v = ((V * ticks_to_float) - bias) * gain`. For
 * @ref ticks_to_float the same value used for all channels, where as for
 * @ref gain and @ref bias the value is per channel.
 */
struct m_imu_pre_filter_part
{
	//! Bias for the imu part.
	struct xrt_vec3 bias;
	float _pad;
	//! Gain for the imu part.
	struct xrt_vec3 gain;
	//! Going from IMU 'ticks' to a floating value.
	float ticks_to_float;
};

/*!
 * This is a common IMU pre-filter which takes raw 'ticks' from an IMU
 * measurement and converts it into degrees per secs and meters per floats.
 */
struct m_imu_pre_filter
{
	struct m_imu_pre_filter_part accel;
	struct m_imu_pre_filter_part gyro;

	/*!
	 * A 'pose' on how to transform the IMU values into head space.
	 */
	struct xrt_pose imu_to_head;
};

/*!
 * Pre-filters the values, taking them from ticks into float values.
 *
 * See description of @ref m_imu_pre_filter_part for formula used. Also rotates
 * values with the imu_to_head pose.
 */
void
m_imu_pre_filter_data(struct m_imu_pre_filter *imu,
                      struct xrt_vec3_i32 *accel,
                      struct xrt_vec3_i32 *gyro,
                      struct xrt_vec3 *out_accel,
                      struct xrt_vec3 *out_gyro);


#ifdef __cplusplus
}
#endif
