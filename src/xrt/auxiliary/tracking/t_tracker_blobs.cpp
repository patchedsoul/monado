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
#include "t_tracker_blobs.h"

void
tracker_blobs_do_view(CamViewProps const &view_props,
                      CamViewState &view_state,
                      cv::Mat &grey,
                      cv::Mat &rgb)
{

	// Undistort the whole image.
	cv::remap(grey,                       // src
	          view_state.frame_undist,    // dst
	          view_props.undistort_map_x, // map1
	          view_props.undistort_map_y, // map2
	          cv::INTER_LINEAR,           // interpolation
	          cv::BORDER_CONSTANT,        // borderMode
	          cv::Scalar(0, 0, 0));       // borderValue

	// Rectify the whole image.
	cv::remap(view_state.frame_undist,    // src
	          view_state.frame_rectified, // dst
	          view_props.rectify_map_x,   // map1
	          view_props.rectify_map_y,   // map2
	          cv::INTER_LINEAR,           // interpolation
	          cv::BORDER_CONSTANT,        // borderMode
	          cv::Scalar(0, 0, 0));       // borderValue

	cv::threshold(view_state.frame_rectified, // src
	              view_state.frame_rectified, // dst
	              32.0,                       // thresh
	              255.0,                      // maxval
	              0);                         // type

	// tracker_measurement_t m = {};

	// Do blob detection with our masks.
	//! @todo Re-enable masks.
	view_state.sbd->detect(view_state.frame_rectified, // image
	                       view_state.keypoints,       // keypoints
	                       cv::noArray());             // mask


	// Debug is wanted, draw the keypoints.
	if (rgb.cols > 0) {
		cv::drawKeypoints(
		    view_state.frame_rectified,                 // image
		    view_state.keypoints,                       // keypoints
		    rgb,                                        // outImage
		    cv::Scalar(255, 0, 0),                      // color
		    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); // flags
	}
}
