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
#include "t_fusion.h"
#include "math/m_eigen_interop.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../../external/flexkalman/OrientationConstantVelocity.h"
#include "../../external/flexkalman/AngularVelocityMeasurement.h"
#include "../../external/flexkalman/FlexibleUnscentedCorrect.h"
#include "../../external/flexkalman/FlexibleKalmanFilter.h"
#include "../../external/flexkalman/PureVectorState.h"
#include "../../external/flexkalman/ConstantProcess.h"
#include "../../external/flexkalman/AugmentedState.h"
#include "../../external/flexkalman/AugmentedProcessModel.h"

#include <memory>

using IMUProcessModel = flexkalman::OrientationConstantVelocityProcessModel;
using IMUState = IMUProcessModel::State;

using BiasState = flexkalman::PureVectorState<3>;
using flexkalman::types::Vector;


struct imu_fusion
{
	IMUState state{};
	IMUProcessModel processModel{};
	uint64_t time_ns{};
	struct
	{
		BiasState state{Vector<3>::Zero(),
		                Vector<3>::Constant(10).asDiagonal()};
		flexkalman::ConstantProcess<BiasState> processModel{};
	} gyro_bias;
};

/*
 * internal functions
 */

//! Predict the filter forward, then correct using the supplied measurement as
//! an "unscented" Kalman filter.
template <typename MeasurementType>
static int
unscentedPredictCorrect(imu_fusion &fusion,
                        float dt,
                        MeasurementType &&meas,
                        const char *meas_desc)
{
	if (dt != 0) {
		flexkalman::predict(fusion.state, fusion.processModel, dt);
	}
	auto correctionInProgress =
	    flexkalman::beginUnscentedCorrection(fusion.state, meas);
	if (!correctionInProgress.stateCorrectionFinite) {
		fprintf(stderr,
		        "Non-finite state correction in applying "
		        "%s\n",
		        meas_desc);
		return -2;
	}

	if (!correctionInProgress.finishCorrection(true)) {
		fprintf(stderr, "Non-finite covariance after applying %s\n",
		        meas_desc);
		return -3;
	}

	return 0;
}


static int
imu_fusion_output_prediction(IMUState const &state,
                             struct xrt_quat &out_quat,
                             struct xrt_vec3 &out_ang_vel)
{
	map_quat(out_quat) = state.getQuaternion().cast<float>();
	map_vec3(out_ang_vel) = state.angularVelocity().cast<float>();
	return 0;
}

static int
imu_fusion_output_rot_vec(IMUState const &state, struct xrt_vec3 &out_rot_vec)
{
	map_vec3(out_rot_vec) =
	    flexkalman::util::quat_ln(state.getQuaternion().cast<float>()) * 2;
	return 0;
}

/*
 * API functions
 */
struct imu_fusion *
imu_fusion_create()
{
	try {
		auto fusion = std::make_unique<imu_fusion>();
		// Arbitrarily high since we have no idea where we are.
		fusion->state.setErrorCovariance(
		    Vector<6>::Constant(10).asDiagonal());

		fusion->processModel.setNoiseAutocorrelation(
		    Vector<3>::Constant(1.3e-1));
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

		if (dt != 0) {
			flexkalman::predict(fusion->state, fusion->processModel,
			                    dt);
			// Advance the gyro bias state as well: just updates the
			// variance
			flexkalman::predict(fusion->gyro_bias.state,
			                    fusion->gyro_bias.processModel, dt);
		}
#if 0
	// the filter runs on doubles, not floats
	auto meas = flexkalman::AngularVelocityMeasurement(
	    map_vec3(*ang_vel).cast<double>(),
	    map_vec3(*variance).cast<double>());
	if (!flexkalman::correctUnscented(fusion->state, meas)) {
		fprintf(stderr,
		        "Non-finite results in correcting fusion using %s - "
		        "update canceled.\n",
		        meas_desc);
		return -2;
	}
#else
		auto meas = xrt_fusion::BiasedGyroMeasurement{
		    map_vec3(*ang_vel).cast<double>(),
		    map_vec3(*variance).cast<double>()};
		auto augmented_state = flexkalman::makeAugmentedState(
		    fusion->state, fusion->gyro_bias.state);
		if (!flexkalman::correctUnscented(augmented_state, meas)) {
			fprintf(stderr,
			        "Non-finite results in correcting fusion using "
			        "gyros - "
			        "update canceled.\n");
			return -2;
		}
#endif
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
                                     float scale,
                                     struct xrt_vec3 const *reference,
                                     struct xrt_vec3 const *variance)
{
	try {
		assert(fusion);
		assert(accel);
		assert(variance);

		if (dt != 0) {
			flexkalman::predict(fusion->state, fusion->processModel,
			                    dt);
		}
		// the fusion runs on doubles, not floats
		auto meas = xrt_fusion::WorldDirectionMeasurement<IMUState>(
		    map_vec3(*accel).cast<double>() * scale,
		    map_vec3(*reference).cast<double>(),
		    map_vec3(*variance).cast<double>());
		if (!flexkalman::correctUnscented(fusion->state, meas)) {
			fprintf(stderr,
			        "Non-finite results in correcting fusion using "
			        "accelerometers - update canceled.\n");
			return -2;
		}
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

		if (dt == 0) {
			// No need to predict here.
			return imu_fusion_output_prediction(
			    fusion->state, *out_quat, *out_ang_vel);
		}

		// Copy the state so we don't advance the internal clock.
		IMUState temp{fusion->state};
		// Copy the process model so we have one to modify.
		IMUProcessModel process{fusion->processModel};

		// Now, just predict + update the state itself, not the
		// covariance (save time)
		flexkalman::predictAndPostCorrectStateOnly(temp, process, dt);

		return imu_fusion_output_prediction(temp, *out_quat,
		                                    *out_ang_vel);

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

		if (dt == 0) {
			// No need to predict here.
			return imu_fusion_output_rot_vec(fusion->state,
			                                 *out_rotation_vec);
		}

		// Copy the state so we don't advance the internal clock.
		IMUState temp{fusion->state};
		// Copy the process model so we have one to modify.
		IMUProcessModel process{fusion->processModel};

		// Now, just predict + update the state itself, not the
		// covariance (save time)
		flexkalman::predictAndPostCorrectStateOnly(temp, process, dt);

		return imu_fusion_output_rot_vec(temp, *out_rotation_vec);
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
    float accel_scale,
    struct xrt_vec3 const *accel_reference,
    struct xrt_vec3 const *accel_variance)
{
	try {
		assert(fusion);
		assert(ang_vel);
		assert(ang_vel_variance);
		assert(accel);
		assert(accel_reference);
		assert(accel_variance);
		if (dt != 0) {
			flexkalman::predict(fusion->state, fusion->processModel,
			                    dt);
			// Advance the gyro bias state as well: just updates the
			// variance
			flexkalman::predict(fusion->gyro_bias.state,
			                    fusion->gyro_bias.processModel, dt);
		}
		{
			// the fusion runs on doubles, not floats
			auto meas =
			    xrt_fusion::WorldDirectionMeasurement<IMUState>(
			        map_vec3(*accel).cast<double>() * accel_scale,
			        map_vec3(*accel_reference).cast<double>(),
			        map_vec3(*accel_variance).cast<double>());
			if (!flexkalman::correctUnscented(fusion->state,
			                                  meas)) {
				fprintf(stderr,
				        "Non-finite results in correcting "
				        "fusion using "
				        "accelerometers - update canceled.\n");
				return -2;
			}
		}
		{
			auto meas = xrt_fusion::BiasedGyroMeasurement{
			    map_vec3(*ang_vel).cast<double>(),
			    map_vec3(*ang_vel_variance).cast<double>()};
			auto augmented_state = flexkalman::makeAugmentedState(
			    fusion->state, fusion->gyro_bias.state);
			if (!flexkalman::correctUnscented(augmented_state,
			                                  meas)) {
				fprintf(stderr,
				        "Non-finite results in correcting "
				        "fusion using "
				        "gyros - "
				        "update canceled.\n");
				return -2;
			}
		}
		return 0;
	} catch (...) {
		assert(
		    false &&
		    "Caught exception on incorporate gyros and accelerometer");
		return -1;
	}
}
