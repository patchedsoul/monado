// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Basic IMU on a arm fusion, used by PSMV.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "math/m_filter_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif



struct m_queue_vec3_f32;
struct m_imu_on_arm;


void
m_queue_vec3_f32_alloc(struct m_queue_vec3_f32 **tq_out, size_t num);

void
m_queue_vec3_f32_free(struct m_queue_vec3_f32 **tq_ptr);

void
m_imu_on_arm_alloc(struct m_imu_on_arm **f_out);

void
m_imu_on_arm_free(struct m_imu_on_arm **f_ptr);

void
m_imu_on_arm_position(struct m_imu_on_arm *f,
                      const struct xrt_vec3 *pos,
                      uint64_t timestamp_ns);

void
m_imu_on_arm_accel(struct m_imu_on_arm *f,
                   const struct xrt_vec3 *accel,
                   uint64_t timestamp_ns);

void
m_imu_on_arm_gyro(struct m_imu_on_arm *f,
                  const struct xrt_vec3 *gyro,
                  uint64_t timestamp_ns);

void
m_imu_on_arm_latest(struct m_imu_on_arm *f, struct xrt_pose *pose_out);


#ifdef __cplusplus
}
#endif
