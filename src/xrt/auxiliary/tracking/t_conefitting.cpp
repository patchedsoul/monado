// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PS Move cone-fitting
 *
 * Based on the technique described in:
 * Kreylos, Oliver. “How to Track Glowing Balls in 3D.” Doc-Ok.Org (blog),
 * December 18, 2019. http://doc-ok.org/?p=1599. ———. Letter to Ryan Pavlik.
 * “PlayStation Move Tracking and Cone Fitting,” November 8, 2019.
 *
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 */

#include "t_conefitting.hpp"

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Cholesky> // for ldlt()

#include <vector>

namespace impl {

constexpr int maxIterations = 4;
class ConeFitterImpl
{

	using Vec3 = Eigen::Vector3f;
	using Matrix33 = Eigen::Matrix3f;
	using Scalar = float;
	Vec3 aPrime;
	Scalar aPrimeLength;
	Scalar cosAlpha;
	std::vector<bool> usedVertices;
	std::vector<bool> usedVerticesTemp;
	size_t numVerticesProcessed = 0;

	/*!
	 * @brief Shared implementation that processes (optionally some subset
	 * of) the boundary direction vectors to populate axis, axisLen, and
	 * cosAngle.
	 *
	 * @param boundaryDirections Normalized 3D vectors indicating the
	 * direction to a supposed boundary pixel of the sphere in the image.
	 * @param minPoints The minimum number of boundaryDirections that should
	 * be used to perform an estimate. 6 is a reasonable default "magic
	 * number".
	 * @param pred A predicate passed the vector p (aka $$p_i$$) as well as
	 * the length of vector p prime (px, py, -1)
	 *
	 * @return true if the number of vertices processed was sufficient, the
	 * solver ran, and all members are updated. If false, only
	 * numVerticesProcessed and usedVertices is updated.
	 */
	template <typename Predicate>
	size_t
	compute(std::vector<cv::Vec3f> const &boundaryDirections,
	        size_t minPoints,
	        Predicate &&pred)
	{
		Matrix33 MTM = Matrix33::Zero();
		Vec3 MTb = Vec3::Zero();
		size_t numVertices = 0;
		usedVerticesTemp.clear();
		usedVerticesTemp.reserve(boundaryDirections.size());

		// Solve the least-squares problem using normal equations.
		for (auto &pVec : boundaryDirections) {
			Scalar px = pVec[0];
			Scalar py = pVec[1];
			Scalar pz = pVec[2];
			Vec3 p(px, py, pz);
			// Note that length of p should be 1.
			if (std::abs(px * px + py * py + pz * pz - 1) >
			    0.0001) {
				throw std::runtime_error("p not normalized");
			}
			//! @todo why is this going in the MRow?
			Scalar pPrimeLen = std::sqrt(px * px + py * py + 1.0f);

			Vec3 MRow(px, py, -pPrimeLen);
			bool shouldUse = pred(p, pPrimeLen);
			usedVerticesTemp.push_back(shouldUse);
			if (shouldUse) {
				numVertices++;
				MTM += MRow * MRow.transpose();
				// Oliver's code appears to do the following:
				MTb += -MRow;
				// though this seems more logical
				// MTb += pz * MRow;
			}
		}
		numVerticesProcessed = numVertices;
		if (numVertices < minPoints) {
			return false;
		}
		// Solve the normal equations
		Vec3 solution = MTM.ldlt().solve(MTb);
		aPrime = Vec3(solution[0], solution[1], -1.0);
		aPrimeLength = aPrime.norm();
		cosAlpha = solution[2] / aPrimeLength;
		usedVertices.swap(usedVerticesTemp);
		return numVertices;
	}

	/*!
	 * @brief Wrapper around compute() for the first pass through the
	 * boundary (trivial predicate).
	 *
	 * @see compute()
	 *
	 * @return true if the number of vertices processed was sufficient, the
	 * solver ran, and all members are updated.
	 */
	size_t
	computeInitial(std::vector<cv::Vec3f> const &boundaryDirections,
	               size_t minPoints)
	{
		const auto pred = [](Vec3 const &, Scalar) { return true; };
		return compute(boundaryDirections, minPoints, pred);
	}

	/*!
	 * @brief Wrapper around compute() for the subsequent passes through the
	 * boundary (predicate is "things outside the current cone").
	 *
	 * Uses the current state of this object (left over from an earlier
	 * compute() call) to configure the predicate.
	 *
	 * @see compute()
	 *
	 * @return true if the number of vertices processed was sufficient, the
	 * solver ran, and all members are updated.
	 */
	size_t
	computeIteration(std::vector<cv::Vec3f> const &boundaryDirections,
	                 size_t minPoints)
	{
		Eigen::Vector2f aPrimeXY = aPrime.head<2>();
		const auto pred = [&](Vec3 const &p, Scalar pPrimeLen) {
			auto pPrimeDotAPrime = p.head<2>().dot(aPrimeXY) + 1;
			auto pPrimeLenTimesAPrimeLen = pPrimeLen * aPrimeLength;
			// Cutoff value found experimentally
			// Two-sided check, but not good enough to handle noise
			// on its own sadly. constexpr float maxDiffFrom1 =
			// 0.0003f; return std::abs(1.f - (pPrimeDotAPrime /
			// (pPrimeLenTimesAPrimeLen * cosAlpha))) <
			// maxDiffFrom1;

			// Original "upstream" predicate, rephrased
			return pPrimeDotAPrime <
			       (pPrimeLenTimesAPrimeLen * cosAlpha * 1.0001f);
		};
		return compute(boundaryDirections, minPoints, pred);
	}

public:
	/*!
	 * @brief Main iterative cone fitting function.
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
	 * @param iterationCallback gets called after each iteration (including
	 * the initial one), with the return value of that iteration.
	 */
	template <typename F>
	bool
	fitCone(std::vector<cv::Vec3f> const &boundaryDirections,
	        size_t minPoints,
	        float ballRadius,
	        cv::Vec3f &outPosition,
	        F &&iterationCallback)
	{

		if (boundaryDirections.size() < minPoints) {
			return false;
		}

		if (!computeInitial(boundaryDirections, minPoints)) {
			return false;
		}

		iterationCallback(true);
		auto lastNumVertices = numVerticesProcessed;
		for (int iteration = 0; iteration < maxIterations;
		     ++iteration) {
			bool iterSuccess =
			    computeIteration(boundaryDirections, minPoints);

			if (!iterSuccess) {
				// Ran out of points this time - Oliver Kreylos'
				// code just skips subsequent iterations without
				// any error.
				break;
			}
			if (lastNumVertices == numVerticesProcessed) {
				// No change in set of vertices processed, we've
				// converged.
				break;
			}
			lastNumVertices = numVerticesProcessed;
		}

		auto d = ballRadius / std::sin(std::acos(cosAlpha));
		Eigen::Vector3f::Map(outPosition.val) =
		    aPrime * (d / aPrimeLength);
		return true;
	}

	/*!
	 * @brief fitCone() overload with no callback argument.
	 * @overload
	 */
	bool
	fitCone(std::vector<cv::Vec3f> const &boundaryDirections,
	        size_t minPoints,
	        float ballRadius,
	        cv::Vec3f &outPosition)
	{
		return fitCone(boundaryDirections, minPoints, ballRadius,
		               outPosition, [](bool) {});
	}

	void
	getInlierIndices(std::vector<size_t> &outInlierIndices)
	{
		outInlierIndices.reserve(numVerticesProcessed);
		const auto n = usedVertices.size();
		for (size_t i = 0; i < n; ++i) {
			if (usedVertices[i]) {
				outInlierIndices.emplace_back(i);
			}
		}
	}
};
} // namespace impl

ConeFitter::ConeFitter() : impl_(new impl::ConeFitterImpl) {}

// Out of line for private impl idiom
ConeFitter::~ConeFitter() = default;

bool
ConeFitter::fit_cone(std::vector<cv::Vec3f> const &boundaryDirections,
                     size_t minPoints,
                     float ballRadius,
                     cv::Vec3f &outPosition)
{
	return impl_->fitCone(boundaryDirections, minPoints, ballRadius,
	                      outPosition);
}
bool
ConeFitter::fit_cone_and_get_inlier_indices(
    std::vector<cv::Vec3f> const &boundaryDirections,
    size_t minPoints,
    float ballRadius,
    cv::Vec3f &outPosition,
    std::vector<size_t> &outInlierIndices)
{
	if (impl_->fitCone(boundaryDirections, minPoints, ballRadius,
	                   outPosition)) {
		impl_->getInlierIndices(outInlierIndices);
		return true;
	}
	return false;
}

// Leaving this here as an example of how to make an iteration callback, since
// it is useful for getting debugging data at each iteration.
bool
ConeFitter::fit_cone_and_get_step_inlier_indices(
    std::vector<cv::Vec3f> const &boundaryDirections,
    size_t minPoints,
    float ballRadius,
    cv::Vec3f &outPosition,
    std::vector<std::vector<size_t>> &outStepInlierIndices)
{
	outStepInlierIndices.clear();
	outStepInlierIndices.reserve(impl::maxIterations + 1);
	auto callback = [&](bool success) {
		if (success) {
			outStepInlierIndices.emplace_back();
			impl_->getInlierIndices(outStepInlierIndices.back());
		}
	};
	return impl_->fitCone(boundaryDirections, minPoints, ballRadius,
	                      outPosition, callback);
}
