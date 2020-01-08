// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move cone-fitting header
 *
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */
#pragma once

#include <opencv2/core.hpp>

#include <memory>
#include <vector>

namespace impl {
class ConeFitterImpl;
} // namespace impl

/*!
 * @brief Object for persisting some allocations between calls to fit a cone.
 *
 * All methods call private implementations.
 */
class ConeFitter
{
public:
	ConeFitter();
	~ConeFitter();

	/*!
	 * @brief Attempt to estimate the position of an imaged sphere. (basic
	 * functionality)
	 *
	 * Internally iterates to exclude extraneous points.
	 *
	 * @param boundaryDirections Normalized 3D vectors indicating the
	 * direction to a supposed boundary pixel of the sphere in the image.
	 * @param minPoints The minimum number of boundaryDirections that should
	 * be used to perform an estimate. 6 is a reasonable default "magic
	 * number".
	 * @param ballRadius The radius of the imaged sphere, in the same units
	 * you want outPosition in.
	 * @param[out] outPosition If estimation is successful, populated with
	 * the position estimate for the sphere center.
	 *
	 * @return true if estimation was successful and outPosition was
	 * populated.
	 */
	bool
	fit_cone(std::vector<cv::Vec3f> const &boundaryDirections,
	         size_t minPoints,
	         float ballRadius,
	         cv::Vec3f &outPosition);

	/*!
	 * @brief Like fit_cone(), but also returns the indices of the used
	 * boundary directions.
	 *
	 * May be useful if your boundaryDirections are a parallel array to
	 * original image points, etc.
	 *
	 * @see fit_cone()
	 *
	 * @param boundaryDirections Normalized 3D vectors indicating the
	 * direction to a supposed boundary pixel of the sphere in the image.
	 * @param minPoints The minimum number of boundaryDirections that should
	 * be used to perform an estimate. 6 is a reasonable default "magic
	 * number".
	 * @param ballRadius The radius of the imaged sphere, in the same units
	 * you want outPosition in.
	 * @param[out] outPosition If estimation is successful, populated with
	 * the position estimate for the sphere center.
	 * @param[out] outInlierIndices The indices of the boundaryDirections
	 * elements that were used in the last iteration.
	 */
	bool
	fit_cone_and_get_inlier_indices(
	    std::vector<cv::Vec3f> const &boundaryDirections,
	    size_t minPoints,
	    float ballRadius,
	    cv::Vec3f &outPosition,
	    std::vector<size_t> &outInlierIndices);

	/*!
	 * @brief Like fit_cone(), but also returns the indices of the used
	 * boundary directions at each step.
	 *
	 * May be useful if your boundaryDirections are a parallel array to
	 * original image points, etc.
	 *
	 * @see fit_cone()
	 *
	 * @param boundaryDirections Normalized 3D vectors indicating the
	 * direction to a supposed boundary pixel of the sphere in the image.
	 * @param minPoints The minimum number of boundaryDirections that should
	 * be used to perform an estimate. 6 is a reasonable default "magic
	 * number".
	 * @param ballRadius The radius of the imaged sphere, in the same units
	 * you want outPosition in.
	 * @param[out] outPosition If estimation is successful, populated with
	 * the position estimate for the sphere center.
	 * @param[out] outStepInlierIndices The indices of the
	 * boundaryDirections elements that were used in each iteration.
	 */
	bool
	fit_cone_and_get_step_inlier_indices(
	    std::vector<cv::Vec3f> const &boundaryDirections,
	    size_t minPoints,
	    float ballRadius,
	    cv::Vec3f &outPosition,
	    std::vector<std::vector<size_t>> &outStepInlierIndices);

	//! Move constructor
	ConeFitter(ConeFitter &&other) = default;
	//! Move assignment
	ConeFitter &
	operator=(ConeFitter &&) = default;

	// non-copyable
	ConeFitter(ConeFitter const &) = delete;
	ConeFitter &
	operator=(ConeFitter const &) = delete;

private:
	std::unique_ptr<impl::ConeFitterImpl> impl_;
};
