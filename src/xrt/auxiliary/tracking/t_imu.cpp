// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IMU fusion implementation - for inclusion into the single
 * kalman-incuding translation unit.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#include "t_imu.h"
#include "t_imu_fusion.h"
#include "math/m_eigen_interop.h"

#include <memory>

struct imu_fusion
{
	uint64_t time_ns{};

	xrt_fusion::SimpleIMUFusion simple_fusion;
};


/*
 * API functions
 */
struct imu_fusion *
imu_fusion_create()
{
	try {
		auto fusion = std::make_unique<imu_fusion>();
		return fusion.release();
	} catch (...) {
		return NULL;
	}
}

void
imu_fusion_destroy(struct imu_fusion *fusion)
{
	try {
		delete fusion;
	} catch (...) {
		assert(false && "Caught exception on destroy");
	}
}

int
imu_fusion_incorporate_gyros(struct imu_fusion *fusion,
                             float dt,
                             struct xrt_vec3 const *ang_vel,
                             struct xrt_vec3 const *variance)
{
	try {
		assert(fusion);
		assert(ang_vel);
		assert(variance);

		fusion->simple_fusion.handleGyro(
		    map_vec3(*ang_vel).cast<double>(), dt);
		return 0;
	} catch (...) {
		assert(false && "Caught exception on incorporate gyros");
		return -1;
	}
}

int
imu_fusion_incorporate_accelerometer(struct imu_fusion *fusion,
                                     float dt,
                                     struct xrt_vec3 const *accel,
                                     struct xrt_vec3 const *variance)
{
	try {
		assert(fusion);
		assert(accel);
		assert(variance);

		fusion->simple_fusion.handleAccel(
		    map_vec3(*accel).cast<double>(), dt);
		return 0;
	} catch (...) {
		assert(false &&
		       "Caught exception on incorporate accelerometer");
		return -1;
	}
}

int
imu_fusion_get_prediction(struct imu_fusion const *fusion,
                          float dt,
                          struct xrt_quat *out_quat,
                          struct xrt_vec3 *out_ang_vel)
{
	try {
		assert(fusion);
		assert(out_quat);
		assert(out_ang_vel);
		if (!fusion->simple_fusion.valid()) {
			return -2;
		}

		map_vec3(*out_ang_vel) =
		    fusion->simple_fusion.getAngVel().cast<float>();
		if (dt == 0) {
			// No need to predict here.
			map_quat(*out_quat) =
			    fusion->simple_fusion.getQuat().cast<float>();
			return 0;
		}
		Eigen::Quaterniond predicted_quat =
		    fusion->simple_fusion.getPredictedQuat(dt);
		map_quat(*out_quat) = predicted_quat.cast<float>();
		return 0;

	} catch (...) {
		assert(false && "Caught exception on getting prediction");
		return -1;
	}
}

int
imu_fusion_get_prediction_rotation_vec(struct imu_fusion const *fusion,
                                       float dt,
                                       struct xrt_vec3 *out_rotation_vec)
{
	try {
		assert(fusion);
		assert(out_rotation_vec);

		if (!fusion->simple_fusion.valid()) {
			return -2;
		}
		if (dt == 0) {
			// No need to predict here.
			map_vec3(*out_rotation_vec) =
			    fusion->simple_fusion.getRotationVec()
			        .cast<float>();
		} else {
			Eigen::Quaterniond predicted_quat =
			    fusion->simple_fusion.getPredictedQuat(dt);
			map_vec3(*out_rotation_vec) =
			    flexkalman::util::quat_ln(predicted_quat)
			        .cast<float>();
		}
		return 0;
	} catch (...) {
		assert(false && "Caught exception on getting prediction");
		return -1;
	}
}

int
imu_fusion_incorporate_gyros_and_accelerometer(
    struct imu_fusion *fusion,
    float dt,
    struct xrt_vec3 const *ang_vel,
    struct xrt_vec3 const *ang_vel_variance,
    struct xrt_vec3 const *accel,
    struct xrt_vec3 const *accel_variance)
{
	try {
		assert(fusion);
		assert(ang_vel);
		assert(ang_vel_variance);
		assert(accel);
		assert(accel_variance);

		Eigen::Vector3d accelVec = map_vec3(*accel).cast<double>();
		Eigen::Vector3d angVelVec = map_vec3(*ang_vel).cast<double>();
		fusion->simple_fusion.handleAccel(accelVec, dt);
		fusion->simple_fusion.handleGyro(angVelVec, dt);
		fusion->simple_fusion.postCorrect();
		return 0;
	} catch (...) {
		assert(
		    false &&
		    "Caught exception on incorporate gyros and accelerometer");
		return -1;
	}
}
