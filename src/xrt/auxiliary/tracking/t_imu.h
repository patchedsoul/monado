// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C interface to basic IMU fusion.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 *
 * @ingroup aux_tracking
 */

#include "math/m_api.h"

#ifdef __cplusplus
extern "C" {
#endif

struct imu_sensor_config
{
	struct xrt_vec3 initial_scale;
	struct xrt_vec3 initial_bias;
	bool present;
};

struct imu_sensor_measurement
{
	struct xrt_vec3 raw_measurement;
	struct xrt_vec3 scaled_variance;
};

struct imu_filter_config
{
	struct imu_sensor_config accel_config;
	struct imu_sensor_config gyro_config;
};

struct imu_filter;
/*!
 * Create a struct imu_filter.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
struct imu_filter *
imu_filter_create();


/*!
 * Destroy a struct imu_filter.
 *
 * Should not be called simultaneously with any other imu_filter function.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
void
imu_filter_destroy(struct imu_filter *filter);

/*!
 * Predict and correct filter with a gyroscope reading.
 *
 * Should not be called simultaneously with any other imu_filter function.
 *
 * Non-zero return means error.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
int
imu_filter_incorporate_gyros(struct imu_filter *filter,
                             float dt,
                             struct xrt_vec3 const *ang_vel,
                             struct xrt_vec3 const *variance);

/*!
 * Predict and correct filter with an accelerometer reading.
 *
 * Should not be called simultaneously with any other imu_filter function.
 *
 * Non-zero return means error.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
int
imu_filter_incorporate_accelerometer(struct imu_filter *filter,
                                     float dt,
                                     struct xrt_vec3 const *accel,
                                     float scale,
                                     struct xrt_vec3 const *reference,
                                     struct xrt_vec3 const *variance);

/*!
 * Predict and correct filter with a simultaneous accelerometer and gyroscope
 * reading.
 *
 * Should not be called simultaneously with any other imu_filter function.
 *
 * Non-zero return means error.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
int
imu_filter_incorporate_gyros_and_accelerometer(
    struct imu_filter *filter,
    float dt,
    struct xrt_vec3 const *ang_vel,
    struct xrt_vec3 const *ang_vel_variance,
    struct xrt_vec3 const *accel,
    float accel_scale,
    struct xrt_vec3 const *accel_reference,
    struct xrt_vec3 const *accel_variance);

/*!
 * Get the predicted state. Does not advance the internal state clock.
 *
 * Non-zero return means error.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
int
imu_filter_get_prediction(struct imu_filter const *filter,
                          float dt,
                          struct xrt_quat *out_quat,
                          struct xrt_vec3 *out_ang_vel);


/*!
 * Get the predicted state as a rotation vector. Does not advance the internal
 * state clock.
 *
 * Non-zero return means error.
 *
 * @public @memberof imu_filter
 * @ingroup aux_math
 */
int
imu_filter_get_prediction_rotation_vec(struct imu_filter const *filter,
                                       float dt,
                                       struct xrt_vec3 *out_rotation_vec);
#ifdef __cplusplus
}
#endif
