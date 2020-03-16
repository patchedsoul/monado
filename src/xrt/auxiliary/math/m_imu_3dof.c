// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A IMU fusion specially made for 3dof devices.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#include "util/u_misc.h"
#include "math/m_api.h"
#include "math/m_filter_fifo.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"

#include <stdio.h>

#define DUR_1S_IN_NS (1000 * 1000 * 1000)
#define DUR_300MS_IN_NS (300 * 1000 * 1000)
#define DUR_20MS_IN_NS (20 * 1000 * 1000)


void
m_imu_3dof_init(struct m_imu_3dof *f, int flags)
{
	U_ZERO(f);
	f->rot.w = 1.0f;

	m_ff_vec3_f32_alloc(&f->word_accel_ff, 1000);
	m_ff_vec3_f32_alloc(&f->gyro_ff, 1000);

	f->flags = flags;
}

void
m_imu_3dof_close(struct m_imu_3dof *f)
{
	m_ff_vec3_f32_free(&f->word_accel_ff);
	m_ff_vec3_f32_free(&f->gyro_ff);
}

static void
gravity_correction(struct m_imu_3dof *f,
                   uint64_t timepoint_ns,
                   const struct xrt_vec3 *accel,
                   const struct xrt_vec3 *gyro,
                   double dt,
                   float gyro_length)
{
	uint64_t dur_ns = 0;
	if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_20MS) {
		dur_ns = DUR_20MS_IN_NS;
	} else if (f->flags & M_IMU_3DOF_USE_GRAVITY_DUR_300MS) {
		dur_ns = DUR_300MS_IN_NS;
	} else {
		return;
	}

	const float gravity_tolerance = .4f, gyro_tolerance = .1f;
	const float min_tilt_error = 0.05f, max_tilt_error = 0.01f;

	/*
	 * If the device is within tolerance levels, count this
	 * as the device is level and add to the counter otherwise
	 * reset the counter and start over.
	 */

	bool is_accel = fabsf(m_vec3_len(*accel) - 9.82f) >= gravity_tolerance;
	bool is_rotating = gyro_length >= gyro_tolerance;
	if (is_accel || is_rotating) {
		f->grav.level_timepoint_ns = timepoint_ns;
	}

	/*
	 * Device has been level for long enough, grab mean from the
	 * accelerometer filter queue (last n values) and use for correction.
	 */
	uint64_t level_ns = f->grav.level_timepoint_ns + dur_ns;
	if (level_ns < timepoint_ns) {
		// Reset the timepoint
		f->grav.level_timepoint_ns = timepoint_ns;

		struct xrt_vec3 accel_mean;
		m_ff_vec3_f32_filter(f->word_accel_ff, timepoint_ns - 5e7,
		                     timepoint_ns, &accel_mean);
		if ((m_vec3_len(accel_mean) - 9.82f) < gravity_tolerance) {
			/*
			 * Calculate a cross product between what the device
			 * thinks is up and what gravity indicates is down.
			 * The values are optimized of what we would get out
			 * from the cross product.
			 */
			struct xrt_vec3 tilt = {
			    accel_mean.z,
			    0,
			    -accel_mean.x,
			};

			tilt = m_vec3_normalize(tilt);
			accel_mean = m_vec3_normalize(accel_mean);

			struct xrt_vec3 up = {0, 1.0f, 0};
			float tilt_angle = m_vec3_angle(up, accel_mean);

			if (tilt_angle > max_tilt_error) {
				f->grav.error_angle = tilt_angle;
				f->grav.error_axis = tilt;
			}
		}
	}

	// Perform gravity tilt correction.
	if (f->grav.error_angle > min_tilt_error) {
		// Correct 180° over 10 seconds, when moving.
		float max_factor = M_PI * dt / 10;
		// Correct 180° over 120 seconds, when stationary.
		float min_factor = M_PI * dt / 120;
		float use_angle = 0.5 * gyro_length * max_factor;

		use_angle = fmax(min_factor, use_angle);
		use_angle = fmin(max_factor, use_angle);
		use_angle = -fmin(use_angle, f->grav.error_angle);
		f->grav.error_angle += use_angle;

		// Perform the correction.
		struct xrt_quat corr_quat, old_orient;
		math_quat_from_angle_vector(use_angle, &f->grav.error_axis,
		                            &corr_quat);
		old_orient = f->rot;
		math_quat_rotate(&corr_quat, &old_orient, &f->rot);
	}
}

void
m_imu_3dof_update(struct m_imu_3dof *f,
                  uint64_t timepoint_ns,
                  const struct xrt_vec3 *accel,
                  const struct xrt_vec3 *gyro)
{
	//! Skip the first sample.
	if (f->state == M_IMU_3DOF_STATE_START) {
		f->state = M_IMU_3DOF_STATE_RUNNING;
		f->last.timepoint_ns = timepoint_ns;
		return;
	}

	f->last.gyro = *gyro;
	f->last.accel = *accel;

	struct xrt_vec3 world_accel = {0};
	math_quat_rotate_vec3(&f->rot, accel, &world_accel);

	uint64_t diff = timepoint_ns - f->last.timepoint_ns;
	double dt = (double)diff / DUR_1S_IN_NS;

	f->last.delta_ms = dt * 1000.0;
	f->last.timepoint_ns = timepoint_ns;

	m_ff_vec3_f32_push(f->word_accel_ff, &world_accel, timepoint_ns);
	m_ff_vec3_f32_push(f->gyro_ff, gyro, timepoint_ns);

	float gyro_length = m_vec3_len(*gyro);

	if (gyro_length > 0.0001f) {
#if 0
		math_quat_integrate_velocity(&f->rot, gyro, dt, &f->rot);
#else
		struct xrt_vec3 rot_axis = {
		    gyro->x / gyro_length,
		    gyro->y / gyro_length,
		    gyro->z / gyro_length,
		};

		float rot_angle = gyro_length * dt;

		struct xrt_quat delta_orient;
		math_quat_from_angle_vector(rot_angle, &rot_axis,
		                            &delta_orient);

		math_quat_rotate(&f->rot, &delta_orient, &f->rot);
#endif
	}

	// Gravity correction.
	gravity_correction(f, timepoint_ns, accel, gyro, dt, gyro_length);

	/*
	 * Mitigate drift due to floating point
	 * inprecision with quat multiplication.
	 */
	math_quat_normalize(&f->rot);
}
