// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basic IMU on a arm fusion, used by PSMV.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#include "util/u_misc.h"
#include "math/m_api.h"
#include "math/m_fusion.h"


struct m_imu_on_arm
{
	struct m_ff_vec3_f32 *q_position;
	struct m_ff_vec3_f32 *q_accel;
	struct m_ff_vec3_f32 *q_gyro;

	struct xrt_vec3 offset;

	struct xrt_pose pose;
};


/*
 *
 * IMU on a arm.
 *
 */

void
m_imu_on_arm_alloc(struct m_imu_on_arm **f_out)
{
	struct m_imu_on_arm *f = U_TYPED_CALLOC(struct m_imu_on_arm);
	m_ff_vec3_f32_alloc(&f->q_position, 128);
	m_ff_vec3_f32_alloc(&f->q_accel, 128);
	m_ff_vec3_f32_alloc(&f->q_gyro, 128);
	f->pose.orientation.w = 1.0f;
	*f_out = f;
}

void
m_imu_on_arm_free(struct m_imu_on_arm **f_ptr)
{
	struct m_imu_on_arm *f = *f_ptr;
	if (f == NULL) {
		return;
	}

	m_ff_vec3_f32_free(&f->q_position);
	m_ff_vec3_f32_free(&f->q_accel);
	m_ff_vec3_f32_free(&f->q_gyro);
	free(f);
	*f_ptr = NULL;
}

void
m_imu_on_arm_position(struct m_imu_on_arm *f,
                      const struct xrt_vec3 *pos,
                      uint64_t timestamp_ns)
{
	m_ff_vec3_f32_push(f->q_position, pos, timestamp_ns);
}

void
m_imu_on_arm_accel(struct m_imu_on_arm *f,
                   const struct xrt_vec3 *accel,
                   uint64_t timestamp_ns)
{
	m_ff_vec3_f32_push(f->q_accel, accel, timestamp_ns);
}

void
m_imu_on_arm_gyro(struct m_imu_on_arm *f,
                  const struct xrt_vec3 *gyro,
                  uint64_t timestamp_ns)
{
	m_ff_vec3_f32_push(f->q_gyro, gyro, timestamp_ns);
}

void
m_imu_on_arm_latest(struct m_imu_on_arm *f, struct xrt_pose *pose_out)
{
	*pose_out = f->pose;
}
