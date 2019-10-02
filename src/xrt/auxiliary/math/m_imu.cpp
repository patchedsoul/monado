// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  IMU fusion.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_math
 */

#include "m_imu.h"

#include "m_eigen_interop.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef FLEXKALMAN_COMBINED
#include "../../external/flexkalman/FlexKalman.h"
#else
#include "../../external/flexkalman/OrientationConstantVelocity.h"
#include "../../external/flexkalman/AngularVelocityMeasurement.h"
#include "../../external/flexkalman/FlexibleUnscentedCorrect.h"
#include "../../external/flexkalman/FlexibleKalmanFilter.h"
#include "../../external/flexkalman/PureVectorState.h"
#include "../../external/flexkalman/ConstantProcess.h"
#include "../../external/flexkalman/AugmentedState.h"
#include "../../external/flexkalman/AugmentedProcessModel.h"
#endif

#include <memory>

using ProcessModel = flexkalman::OrientationConstantVelocityProcessModel;
using State = ProcessModel::State;

namespace {
namespace types = flexkalman::types;
using flexkalman::types::Vector;

//! For things like accelerometers, which on some level measure the local vector
//! of a world direction.
class WorldDirectionMeasurement
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 3;
	using MeasurementVector = types::Vector<Dimension>;
	using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
	WorldDirectionMeasurement(types::Vector<3> const &direction,
	                          types::Vector<3> const &reference,
	                          types::Vector<3> const &variance)
	    : direction_(direction.normalized()),
	      reference_(reference.normalized()),
	      covariance_(variance.asDiagonal())
	{}

	// template <typename State>
	MeasurementSquareMatrix const &
	getCovariance(State const & /*s*/)
	{
		return covariance_;
	}

	// template <typename State>
	types::Vector<3>
	predictMeasurement(State const &s) const
	{
		return s.getCombinedQuaternion() * reference_;
	}

	// template <typename State>
	MeasurementVector
	getResidual(MeasurementVector const &predictedMeasurement,
	            State const &s) const
	{
		return predictedMeasurement - reference_;
	}

	template <typename State>
	MeasurementVector
	getResidual(State const &s) const
	{
		return getResidual(predictMeasurement(s), s);
	}

private:
	types::Vector<3> direction_;
	types::Vector<3> reference_;
	MeasurementSquareMatrix covariance_;
};
#if 0
//! For things like accelerometers, which on some level measure the local vector
//! of a world direction.
class LinAccelWithGravityMeasurement
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 3;
	using MeasurementVector = types::Vector<Dimension>;
	using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
	LinAccelWithGravityMeasurement(types::Vector<3> const &direction,
	                          types::Vector<3> const &reference,
	                          types::Vector<3> const &variance)
	    : direction_(direction), reference_(reference),
	      covariance_(variance.asDiagonal())
	{}

	// template <typename State>
	MeasurementSquareMatrix const &
	getCovariance(State const & /*s*/)
	{
		return covariance_;
	}

	// template <typename State>
	types::Vector<3>
	predictMeasurement(State const &s) const
	{
		return reference_;
	}

	// template <typename State>
	MeasurementVector
	getResidual(MeasurementVector const &predictedMeasurement,
	            State const &s) const
	{
		s.getQuaternion().conjugate() *
		        predictedMeasurement return predictedMeasurement -
		    reference_.normalized();
	}

	template <typename State>
	MeasurementVector
	getResidual(State const &s) const
	{
		MeasurementVector residual =
		    direction_ - reference_ * s.getQuaternion();
		return getResidual(predictMeasurement(s), s);
	}

private:
	types::Vector<3> direction_;
	types::Vector<3> reference_;
	MeasurementSquareMatrix covariance_;
};
#endif

class BiasedGyroMeasurement
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 3;
	using MeasurementVector = types::Vector<Dimension>;
	using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
	BiasedGyroMeasurement(types::Vector<3> const &angVel,
	                      types::Vector<3> const &variance)
	    : angVel_(angVel), covariance_(variance.asDiagonal())
	{}

	template <typename State>
	MeasurementSquareMatrix const &
	getCovariance(State const & /*s*/)
	{
		return covariance_;
	}

	template <typename State>
	types::Vector<3>
	predictMeasurement(State const &s) const
	{
		return s.b().stateVector() + angVel_;
	}

	template <typename State>
	MeasurementVector
	getResidual(MeasurementVector const &predictedMeasurement,
	            State const &s) const
	{
		return predictedMeasurement - s.a().angularVelocity();
	}

	template <typename State>
	MeasurementVector
	getResidual(State const &s) const
	{
		return getResidual(predictMeasurement(s), s);
	}

private:
	types::Vector<3> angVel_;
	MeasurementSquareMatrix covariance_;
};
} // namespace

using BiasState = flexkalman::PureVectorState<3>;
struct imu_filter
{
	State state{};
	ProcessModel processModel{};
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
unscentedPredictCorrect(imu_filter &filter,
                        float dt,
                        MeasurementType &&meas,
                        const char *meas_desc)
{


	return 0;
}


static int
imu_filter_output_prediction(State const &state,
                             struct xrt_quat &out_quat,
                             struct xrt_vec3 &out_ang_vel)
{
	map_quat(out_quat) = state.getQuaternion().cast<float>();
	map_vec3(out_ang_vel) = state.angularVelocity().cast<float>();
	return 0;
}

static int
imu_filter_output_rot_vec(State const &state, struct xrt_vec3 &out_rot_vec)
{
	map_vec3(out_rot_vec) =
	    flexkalman::util::quat_ln(state.getQuaternion().cast<float>()) * 2;
	return 0;
}

/*
 * API functions
 */
struct imu_filter *
imu_filter_create() try
{
	auto filter = std::make_unique<imu_filter>();
	// Arbitrarily high since we have no idea where we are.
	filter->state.setErrorCovariance(Vector<6>::Constant(10).asDiagonal());

	filter->processModel.setNoiseAutocorrelation(
	    Vector<3>::Constant(1.3e-1));
	return filter.release();
} catch (...) {
	return NULL;
}

void
imu_filter_destroy(struct imu_filter *filter) try {
	delete filter;
} catch (...) {
	assert(false && "Caught exception on destroy");
}


int
imu_filter_incorporate_gyros(struct imu_filter *filter,
                             float dt,
                             struct xrt_vec3 const *ang_vel,
                             struct xrt_vec3 const *variance) try {
	assert(filter);
	assert(ang_vel);
	assert(variance);

	if (dt != 0) {
		flexkalman::predict(filter->state, filter->processModel, dt);
	}
#if 0
	// the filter runs on doubles, not floats
	auto meas = flexkalman::AngularVelocityMeasurement(
	    map_vec3(*ang_vel).cast<double>(),
	    map_vec3(*variance).cast<double>());
	if (!flexkalman::correctUnscented(filter.state, meas)) {
		fprintf(stderr,
		        "Non-finite results in correcting filter using %s - "
		        "update canceled.\n",
		        meas_desc);
		return -2;
	}
#else
	auto meas = BiasedGyroMeasurement{map_vec3(*ang_vel).cast<double>(),
	                                  map_vec3(*variance).cast<double>()};
	auto augmented_state = flexkalman::makeAugmentedState(
	    filter->state, filter->gyro_bias.state);
	if (dt != 0) {
		// Advnace the gyro bias state as well: just updates the
		// variance
		flexkalman::predict(filter->gyro_bias.state,
		                    filter->gyro_bias.processModel, dt);
	}
	if (!flexkalman::correctUnscented(augmented_state, meas)) {
		fprintf(stderr,
		        "Non-finite results in correcting filter using gyros - "
		        "update canceled.\n");
		return -2;
	}
#endif
	return 0;
} catch (...) {
	assert(false && "Caught exception on incorporate gyros");
	return -1;
}

int
imu_filter_incorporate_accelerometer(struct imu_filter *filter,
                                     float dt,
                                     struct xrt_vec3 const *accel,
                                     float scale,
                                     struct xrt_vec3 const *reference,
                                     struct xrt_vec3 const *variance) try {
	assert(filter);
	assert(accel);
	assert(variance);

	// the filter runs on doubles, not floats
	auto meas =
	    WorldDirectionMeasurement(map_vec3(*accel).cast<double>() * scale,
	                              map_vec3(*reference).cast<double>(),
	                              map_vec3(*variance).cast<double>());
	return unscentedPredictCorrect(*filter, dt, meas, "accelerometers");
} catch (...) {
	assert(false && "Caught exception on incorporate accelerometer");
	return -1;
}

int
imu_filter_get_prediction(struct imu_filter const *filter,
                          float dt,
                          struct xrt_quat *out_quat,
                          struct xrt_vec3 *out_ang_vel) try {
	assert(filter);
	assert(out_quat);
	assert(out_ang_vel);

	if (dt == 0) {
		// No need to predict here.
		return imu_filter_output_prediction(filter->state, *out_quat,
		                                    *out_ang_vel);
	}

	// Copy the state so we don't advance the internal clock.
	State temp{filter->state};
	// Copy the process model so we have one to modify.
	ProcessModel process{filter->processModel};

	// Now, just predict + update the state itself, not the covariance (save
	// time)
	flexkalman::predictAndPostCorrectStateOnly(temp, process, dt);

	return imu_filter_output_prediction(temp, *out_quat, *out_ang_vel);
} catch (...) {
	assert(false && "Caught exception on getting prediction");
	return -1;
}

int
imu_filter_get_prediction_rotation_vec(struct imu_filter const *filter,
                                       float dt,
                                       struct xrt_vec3 *out_rotation_vec) try {
	assert(filter);
	assert(out_rotation_vec);

	if (dt == 0) {
		// No need to predict here.
		return imu_filter_output_rot_vec(filter->state,
		                                 *out_rotation_vec);
	}

	// Copy the state so we don't advance the internal clock.
	State temp{filter->state};
	// Copy the process model so we have one to modify.
	ProcessModel process{filter->processModel};

	// Now, just predict + update the state itself, not the covariance (save
	// time)
	flexkalman::predictAndPostCorrectStateOnly(temp, process, dt);

	return imu_filter_output_rot_vec(temp, *out_rotation_vec);
} catch (...) {
	assert(false && "Caught exception on getting prediction");
	return -1;
}
