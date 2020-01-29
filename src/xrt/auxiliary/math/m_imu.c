// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IMU helper struct.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#include "math/m_api.h"
#include "math/m_imu.h"


void
m_imu_pre_filter_data(struct m_imu_pre_filter *imu,
                      struct xrt_vec3_i32 *accel,
                      struct xrt_vec3_i32 *gyro,
                      struct xrt_vec3 *out_accel,
                      struct xrt_vec3 *out_gyro)
{
	struct m_imu_pre_filter_part fa, fg;
	struct xrt_vec3 a, g;
	struct xrt_pose p;

	fa = imu->accel;
	fg = imu->gyro;
	p = imu->imu_to_head;

	a.x = accel->x * fa.ticks_to_float;
	a.y = accel->y * fa.ticks_to_float;
	a.z = accel->z * fa.ticks_to_float;

	g.x = gyro->x * fg.ticks_to_float;
	g.y = gyro->y * fg.ticks_to_float;
	g.z = gyro->z * fg.ticks_to_float;

	a.x = (a.x - fa.bias.x) * fa.gain.x;
	a.y = (a.y - fa.bias.y) * fa.gain.y;
	a.z = (a.z - fa.bias.z) * fa.gain.z;

	g.x = (g.x - fg.bias.x) * fg.gain.x;
	g.y = (g.y - fg.bias.y) * fg.gain.y;
	g.z = (g.z - fg.bias.z) * fg.gain.z;

	math_quat_rotate_vec3(&p.orientation, &a, &a);
	math_quat_rotate_vec3(&p.orientation, &g, &g);

	*out_accel = a;
	*out_gyro = g;
}
