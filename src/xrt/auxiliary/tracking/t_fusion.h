// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C++ sensor fusion/filtering code that uses flexkalman
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 *
 * @ingroup aux_tracking
 */
#pragma once


#ifndef __cplusplus
#error "This header is C++-only."
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "flexkalman/OrientationConstantVelocity.h"
#include "flexkalman/AngularVelocityMeasurement.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"
#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/PureVectorState.h"
#include "flexkalman/ConstantProcess.h"
#include "flexkalman/AugmentedState.h"
#include "flexkalman/AugmentedProcessModel.h"
#include "flexkalman/MatrixExponentialMap.h"

namespace xrt_fusion {
namespace types = flexkalman::types;
using flexkalman::types::Vector;

//! For things like accelerometers, which on some level measure the local vector
//! of a world direction.
template <typename State>
class WorldDirectionMeasurement
    : public flexkalman::MeasurementBase<WorldDirectionMeasurement<State>>
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

	MeasurementSquareMatrix const &
	getCovariance(State const & /*s*/)
	{
		return covariance_;
	}

	types::Vector<3>
	predictMeasurement(State const &s) const
	{
		return s.getCombinedQuaternion() * reference_;
	}

	MeasurementVector
	getResidual(MeasurementVector const &predictedMeasurement,
	            State const &s) const
	{
		return predictedMeasurement - reference_;
	}

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
    : public flexkalman::MeasurementBase<LinAccelWithGravityMeasurement>
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
    : public flexkalman::MeasurementBase<BiasedGyroMeasurement>
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

template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
rot_matrix_ln(Eigen::MatrixBase<Derived> const &mat)
{
	EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
	using Scalar = typename Derived::Scalar;
	Eigen::AngleAxis<Scalar> angleAxis =
	    Eigen::AngleAxis<Scalar>{mat.derived()};
	return angleAxis.angle() * angleAxis.axis();
}

template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
rot_matrix_ln(Eigen::QuaternionBase<Derived> const &q)
{
	using Scalar = typename Derived::Scalar;
	Eigen::AngleAxis<Scalar> angleAxis{q.derived()};
	return angleAxis.angle() * angleAxis.axis();
}

using flexkalman::matrix_exponential_map::rodrigues;
/*!
 * Represents an orientation as a member of the "special orthogonal group in 3D"
 * SO3.
 *
 * This means we're logically using a 3D vector that can be converted to a
 * rotation matrix using the matrix exponential map aka "Rodrigues' formula".
 * We're actually storing both the rotation matrix and the vector for simplicity
 * right now.
 */
class SO3
{
public:
	SO3() = default;
	explicit SO3(Eigen::Vector3d const &v)
	    : matrix_(flexkalman::matrix_exponential_map::rodrigues(
	          flexkalman::matrix_exponential_map::singularitiesAvoided(v)))
	{}
	explicit SO3(Eigen::Matrix3d const &mat) : matrix_(mat) {}

	static SO3
	fromQuat(Eigen::Quaterniond const &q)
	{
		Eigen::AngleAxisd angleAxis{q};
		Eigen::Vector3d omega = angleAxis.angle() * angleAxis.axis();
		return SO3{omega};
	}
	Eigen::Vector3d
	getVector() const
	{
		double angle = getAngle();
		while (angle < -EIGEN_PI) {
			angle += 2 * EIGEN_PI;
		}
		while (angle > EIGEN_PI) {
			angle -= 2 * EIGEN_PI;
		}
		return angle * getAxis();
	}

	Eigen::Matrix3d const &
	getRotationMatrix() const noexcept
	{
		return matrix_;
	}

	Eigen::Quaterniond
	getQuat() const
	{
		return flexkalman::matrix_exponential_map::toQuat(getVector());
	}

	SO3
	getInverse() const
	{
		return SO3{getRotationMatrix(), InverseTag{}};
	}

	double
	getAngle() const
	{
		return Eigen::AngleAxisd{getRotationMatrix()}.angle();
	}

	Eigen::Vector3d
	getAxis() const
	{
		return Eigen::AngleAxisd{getRotationMatrix()}.axis();
	}

private:
	//! tag type to choose the inversion constructor.
	struct InverseTag
	{
	};

	//! Inversion constructor - fast
	SO3(Eigen::Matrix3d const &mat, InverseTag const & /*tag*/)
	    : //  omega_(Eigen::Vector3d::Zero()),
	      matrix_(mat.transpose())
	{
		// omega_ = rot_matrix_ln(matrix_);
	}

	// Eigen::Vector3d omega_{Eigen::Vector3d::Zero()};
	Eigen::Matrix3d matrix_{Eigen::Matrix3d::Identity()};
};

static inline SO3 operator*(SO3 const &lhs, SO3 const &rhs)
{
	Eigen::Matrix3d product =
	    lhs.getRotationMatrix() * rhs.getRotationMatrix();
	return SO3{product};
}

class SimpleIMUFilter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/*!
	 * @param gravity_rate Value in [0, 1] indicating how much the
	 * accelerometer should affect the orientation each second.
	 */
	explicit SimpleIMUFilter(double gravity_rate = 0.9)
	    : gravity_scale_(gravity_rate)
	{}

	bool
	valid() const noexcept
	{
		return started_;
	}

	Eigen::Quaterniond
	getQuat() const
	{
		return Eigen::Quaterniond(
		    Eigen::AngleAxisd(omega_.norm(), omega_.normalized()));
		// return Eigen::Quaterniond(rodrigues(omega_));
	}

	Eigen::Vector3d
	getRotationVec() const
	{
		return omega_;
	}

	//! in world space
	Eigen::Vector3d const &
	getAngVel() const
	{
		return angVel_;
	}

	bool
	filterGyro(Eigen::Vector3d const &gyro, float dt)
	{
		if (!started_) {
			return false;
		}
		Eigen::Vector3d incRot = gyro * dt;
		// Crude handling of "approximately zero"
		if (incRot.squaredNorm() < 1.e-8) {
			return false;
		}
		// Rotate from body-local to world space
		auto current = getRotationMatrix();
		Eigen::Matrix3d bodyToWorld = current.transpose();
		Eigen::Vector3d worldIncRot = bodyToWorld * incRot;
		angVel_ = bodyToWorld * gyro;

		// {
		// 	Eigen::Vector3d axis = worldIncRot.normalized();
		// 	fprintf(stderr,
		// 	        "Incremental rotation is %f radians about %f, "
		// 	        "%f, %f\n",
		// 	        worldIncRot.norm(), axis.x(), axis.y(),
		// 	        axis.z());
		// }
		// Update orientation
		omega_ =
		    rot_matrix_ln(rodrigues(worldIncRot) * current.transpose());

		return true;
	}

	bool
	filterAccel(Eigen::Vector3d const &accel, float dt)
	{
		auto diff = std::abs(accel.norm() - 9.81);
		if (!started_) {
			if (diff > 1.) {
				// We're moving, don't start it now.
				fprintf(stderr,
				        "Can't start yet because we're moving "
				        "- diff is %f\n",
				        diff);
				return false;
			}
			fprintf(stderr, "starting - diff is %f\n", diff);
			// Initially, just set it to totally trust gravity.
			started_ = true;
			omega_ =
			    rot_matrix_ln(Eigen::Quaterniond::FromTwoVectors(
			        Eigen::Vector3d::UnitY(), accel.normalized()));
			return true;
		}
		auto scale = 1. - diff;
		if (scale <= 0) {
			// Too far from gravity to be useful/trusted.
			fprintf(stderr,
			        "started but skipping accel: we're moving "
			        "- diff is %f\n",
			        diff);
			return false;
		}
		Eigen::Vector3d accelDir = accel.normalized();
		// fprintf(stderr, "Measured gravity is %f, %f, %f\n",
		//         accelDir.x(), accelDir.y(), accelDir.z());

		auto current = getRotationMatrix();
		// This should match the global gravity vector if the rotation
		// is right.
		Eigen::Vector3d measuredGravityDirection =
		    (current * accel).normalized();
		// fprintf(stderr, "Rotated measured gravity is %f, %f, %f\n",
		//         measuredGravityDirection.x(),
		//         measuredGravityDirection.y(),
		//         measuredGravityDirection.z());
		auto incremental =
		    Eigen::AngleAxisd(Eigen::Quaterniond::FromTwoVectors(
		        measuredGravityDirection, Eigen::Vector3d::UnitY()));

		Eigen::Vector3d scaledIncrementalVec = incremental.axis() *
		                                       incremental.angle() *
		                                       scale * gravity_scale_;
		// fprintf(stderr,
		//         "Gravity is causing incremental rotation per sec of "
		//         "%f, %f, %f\n",
		//         scaledIncrementalVec.x(), scaledIncrementalVec.y(),
		//         scaledIncrementalVec.z());
		// Update orientation
		omega_ = rot_matrix_ln(rodrigues(scaledIncrementalVec * dt) *
		                       current);

		flexkalman::matrix_exponential_map::avoidSingularities(omega_);
		return true;
	}

	Eigen::Matrix3d
	getRotationMatrix() const
	{
		return rodrigues(omega_);
	}

private:
	Eigen::Vector3d omega_;
	Eigen::Vector3d angVel_{Eigen::Vector3d::Zero()};
	double gravity_scale_;
	bool started_{false};
};
} // namespace xrt_fusion
