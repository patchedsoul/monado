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

#include "../../external/flexkalman/FlexKalman.h"

#include <memory>

using ProcessModel = flexkalman::OrientationConstantVelocityProcessModel;
using State = ProcessModel::State;
using Filter = flexkalman::FlexibleKalmanFilter<ProcessModel>;

namespace {
namespace types = flexkalman::types;

//! For things like accelerometers, which on some level measure the local vector
//! of a world direction.
class WorldDirectionMeasurement
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static const types::DimensionType DIMENSION = 3;
	static const types::DimensionType STATE_DIMENSION =
	    types::Dimension<State>::value;
	using MeasurementVector = types::Vector<DIMENSION>;
	using MeasurementSquareMatrix = types::SquareMatrix<DIMENSION>;
	WorldDirectionMeasurement(types::Vector<3> const &direction,
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
		return s.getCombinedQuaternion() * reference_;
	}

	// template <typename State>
	MeasurementVector
	getResidual(MeasurementVector const &predictedMeasurement,
	            State const &s) const
	{
		return direction_ - reference_.normalized();
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

} // namespace

struct imu_filter
{
	State state{};
	ProcessModel processModel{};
};

/*
 * internal functions
 */

static inline double
ms_to_s(int64_t delta_t_ms)
{
	return double(delta_t_ms) / 1000.0;
	;
}

//! Predict the filter forward, then correct using the supplied measurement as
//! an "unscented" Kalman filter.
template <typename MeasurementType>
static int
unscentedPredictCorrect(imu_filter &filter,
                        float dt,
                        MeasurementType &&meas,
                        const char *meas_desc)
{
	if (dt != 0) {
		flexkalman::predict(filter.state, filter.processModel, dt);
	}
	auto correctionInProgress =
	    flexkalman::beginUnscentedCorrection(filter.state, meas);
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
imu_filter_output_prediction(State const &state,
                             struct xrt_quat &out_quat,
                             struct xrt_vec3 &out_ang_vel)
{
	map_quat(out_quat) = state.getQuaternion().cast<float>();
	map_vec3(out_ang_vel) = state.angularVelocity().cast<float>();
	return 0;
}

/*
 * API functions
 */
struct imu_filter *
imu_filter_create() try
{
	auto obj = std::make_unique<imu_filter>();
	return obj.release();
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

	auto meas = flexkalman::AngularVelocityMeasurement<State>(
	    map_vec3(*ang_vel).cast<double>(),
	    map_vec3(*variance).cast<double>());
	return unscentedPredictCorrect(*filter, dt, meas, "gyros");
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

	// Now, just predict + update the state itself, not the covariance (save
	// time)
	temp.setStateVector(filter->processModel.computeEstimate(temp, dt));
	// Post-correct to incorporate incremental orientation
	temp.postCorrect();

	return imu_filter_output_prediction(temp, *out_quat, *out_ang_vel);
} catch (...) {
	assert(false && "Caught exception on getting prediction");
	return -1;
}
