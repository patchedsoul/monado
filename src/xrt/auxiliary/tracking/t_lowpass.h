// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Low-pass IIR filter
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */
#pragma once


#ifndef __cplusplus
#error "This header is C++-only."
#endif

#include <Eigen/Core>
#include <type_traits>

namespace xrt_fusion {
/*!
 * A very simple low-pass filter, using a "one-pole infinite impulse response"
 * design (one-pole IIR).
 *
 * Configurable in dimension and scalar type.
 */
template <size_t Dim, typename Scalar = double> class LowPassIIR
{
public:
	static_assert(std::is_floating_point<Scalar>::value,
	              "Filter is designed only for floating-point values. If "
	              "you want fixed-point, you must reimplement it.");
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
	 * Constructor
	 *
	 * @param cutoff_hz A cutoff frequency in Hertz: signal changes much
	 * lower in frequency will be passed through the filter, while signal
	 * changes much higher in frequency will be blocked.
	 */
	explicit LowPassIIR(Scalar cutoff_hz)
	    : time_constant_(1.f / (2.f * EIGEN_PI * cutoff_hz))
	{}

	using Vector = Eigen::Matrix<Scalar, Dim, 1>;

	/*!
	 * Filter a sample, with an optional weight.
	 *
	 * @param sample The value to filter
	 * @param timestamp_ns The time that this sample was measured.
	 * @param weight An optional value between 0 and 1. The smaller this
	 * value, the less the current sample influences the filter state. For
	 * the first call, this is always assumed to be 1.
	 */
	void
	addSample(Vector const &sample,
	          std::uint64_t timestamp_ns,
	          Scalar weight = 1)
	{
		if (!initialized_) {
			initialized_ = true;
			state_ = sample;
			filter_timestamp_ns_ = timestamp_ns;
			return;
		}
		// get dt in seconds
		Scalar dt = (timestamp_ns - filter_timestamp_ns_) * 1.e-9f;
		//! @todo limit max dt?
		Scalar weighted = dt * weight;
		Scalar alpha = weighted / (time_constant_ * weighted);

		// The update step below is equivalent to
		// state_ = state_ * (1 - alpha) + alpha * sample;
		// -- it blends the current sample and the filter state using
		// alpha as the blending parameter.
		state_ += alpha * (sample - state_);
		filter_timestamp_ns_ = timestamp_ns;
	}

	/*!
	 * Access the filtered value.
	 */
	Vector const &
	getState() const
	{
		return state_;
	}

	/*!
	 * Access the time of last update.
	 */
	std::uint64_t
	getTimestampNs() const
	{
		return filter_timestamp_ns_;
	}

private:
	Scalar time_constant_;
	bool initialized_{false};
	uint64_t filter_timestamp_ns_;
	Vector state_;
};

} // namespace xrt_fusion
