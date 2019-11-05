// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Shared tracker code between PS Move and PSVR.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup aux_tracking
 */

#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

/*!
 * @brief Helper struct that keeps the value that produces the lowest "score" as
 * computed by your functor.
 *
 * Having this as a struct with a method, instead of a single "algorithm"-style
 * function, allows you to keep your complicated filtering logic in your own
 * loop, just calling in when you have a new candidate for "best".
 *
 * @note Create by calling make_lowest_score_finder() with your
 * function/lambda that takes an element and returns the score, to deduce the
 * un-spellable typename of the lambda.
 *
 * @tparam ValueType The type of a single element value - whatever you want to
 * assign a score to.
 * @tparam FunctionType The type of your functor/lambda that turns a ValueType
 * into a float "score". Usually deduced.
 */
template <typename ValueType, typename FunctionType> struct FindLowestScore
{
	const FunctionType score_functor;
	bool got_one{false};
	ValueType best{};
	float best_score{0};

	void
	handle_candidate(ValueType val)
	{
		float score = score_functor(val);
		if (!got_one || score < best_score) {
			best = val;
			best_score = score;
			got_one = true;
		}
	}
};

//! Factory function for FindLowestScore to deduce the functor type.
template <typename ValueType, typename FunctionType>
static inline FindLowestScore<ValueType, FunctionType>
make_lowest_score_finder(FunctionType scoreFunctor)
{
	return FindLowestScore<ValueType, FunctionType>{scoreFunctor};
}
/*!
 * Single camera.
 */
struct CamViewProps
{
	cv::Mat undistort_map_x;
	cv::Mat undistort_map_y;
	cv::Mat rectify_map_x;
	cv::Mat rectify_map_y;
};

struct CamViewState
{
	std::vector<cv::KeyPoint> keypoints;

	cv::Mat frame_undist;
	cv::Mat frame_rectified;
	cv::Ptr<cv::SimpleBlobDetector> sbd;
};

void
tracker_blobs_do_view(CamViewProps const &view_props,
                      CamViewState &view_state,
                      cv::Mat &grey,
                      cv::Mat &rgb);
