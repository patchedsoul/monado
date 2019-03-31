// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  C interface to math library.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 *
 * @see xrt_vec3
 * @see xrt_quat
 * @see xrt_pose
 * @see xrt_space_relation
 */

#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * @dir auxiliary/math
 *
 * @brief C interface to some transform-related math functions.
 */

/*
 *
 * Vector functions
 *
 */

/*!
 * Accumulate a vector by adding in-place.
 *
 * Logically, *inAndOut += *additional
 * OK if the two arguments are the same addresses.
 *
 * @relates xrt_vec3
 */
void
math_vec3_accum(const struct xrt_vec3 *additional, struct xrt_vec3 *inAndOut);

/*
 *
 * Quat functions.
 *
 */

/*!
 * Rotate a vector.
 *
 * @relates xrt_quat
 * @relatesalso xrt_vec3
 */
void
math_quat_rotate_vec3(const struct xrt_quat *left,
                      const struct xrt_vec3 *right,
                      struct xrt_vec3 *result);

/*!
 * Rotate a quaternion (compose rotations).
 *
 * @relates xrt_quat
 */
void
math_quat_rotate(const struct xrt_quat *left,
                 const struct xrt_quat *right,
                 struct xrt_quat *result);


/*!
 * Integrate an angular velocity vector (exponential map) and apply to a
 * quaternion.
 *
 * ang_vel and dt should share the same units of time, and the ang_vel
 * vector should be in radians per unit of time.
 *
 * @relates xrt_quat
 * @relatesalso xrt_vec3
 */
void
math_quat_integrate_velocity(const struct xrt_quat *quat,
                             const struct xrt_vec3 *ang_vel,
                             const float dt,
                             struct xrt_quat *result);

/*!
 * Compute an angular velocity vector (exponential map format) by taking the
 * finite difference of two quaternions.
 *
 * quat1 is the orientation dt time after the orientation was quat0
 *
 * out_ang_vel and dt share the same units of time, and out_ang_vel is be in
 * radians per unit of time.
 *
 * @relates xrt_quat
 * @relatesalso xrt_vec3
 */
void
math_quat_finite_difference(const struct xrt_quat *quat0,
                            const struct xrt_quat *quat1,
                            const float dt,
                            struct xrt_vec3 *out_ang_vel);

/*
 *
 * Pose functions.
 *
 */

/*!
 * Check if this pose can be used in transformation operations.
 *
 * @relates xrt_pose
 */
bool
math_pose_validate(const struct xrt_pose *pose);

/*!
 * Invert pose.
 *
 * OK if input and output are the same addresses.
 *
 * @relates xrt_pose
 */
void
math_pose_invert(const struct xrt_pose *pose, struct xrt_pose *outPose);

/*!
 * Apply a rigid-body transformation to a pose.
 *
 * OK if input and output are the same addresses.
 *
 * @relates xrt_pose
 */
void
math_pose_transform(const struct xrt_pose *transform,
                    const struct xrt_pose *pose,
                    struct xrt_pose *outPose);

/*!
 * Combine the poses of the target and base space with the relative pose of
 * those spaces. In a way that OpenXR specifies in the function xrLocateSpace.
 *
 * Performs roughly outPose = spacePose * relativePose * baseSpacePose^-1
 *
 * OK if input and output are the same addresses.
 *
 * @relates xrt_pose
 */
void
math_pose_openxr_locate(const struct xrt_pose *space_pose,
                        const struct xrt_pose *relative_pose,
                        const struct xrt_pose *base_space_pose,
                        struct xrt_pose *result);

/*
 *
 * Space relation functions
 *
 */

/*!
 * Reset a relation to zero velocity, located at origin, and all validity flags.
 *
 * @relates xrt_space_relation
 */
void
math_relation_reset(struct xrt_space_relation *out);

/*!
 * Apply a static pose on top of an existing relation.
 *
 * Updates all valid pose and derivative fields. Does not modify the validity
 * mask. Treats both position and orientation of transform as valid.
 *
 * @relates xrt_space_relation
 * @see xrt_pose
 */
void
math_relation_accumulate_transform(const struct xrt_pose *transform,
                                   struct xrt_space_relation *in_out_relation);

/*!
 * Apply another step of space relation on top of an existing relation.
 *
 * Updates all valid pose and derivative fields, as well as the validity mask.
 *
 * @relates xrt_space_relation
 */
void
math_relation_accumulate_relation(
    const struct xrt_space_relation *additional_relation,
    struct xrt_space_relation *in_out_relation);

/*!
 * Combine the poses of the target and base space with the relative relation of
 * those spaces. In a way that OpenXR specifies in the function xrLocateSpace.
 *
 * Performs roughly `out_relation->pose = space_pose * relative_relation->pose *
 * base_space_pose^-1`  for the poses, and appropriate rotation
 *
 * OK if input and output are the same addresses.
 *
 * @relates xrt_space_relation
 * @see xrt_pose
 */
void
math_relation_openxr_locate(const struct xrt_pose *space_pose,
                            const struct xrt_space_relation *relative_relation,
                            const struct xrt_pose *base_space_pose,
                            struct xrt_space_relation *result);

/*!
 * Perform the computations from
 * "Computing Half-Fields-Of-View from Simpler Display Models",
 * to get half-FOVs from things we can retrieve from other APIs.
 * The origin is in the lower-left corner of the display, so w_1 is the width to
 * the left of CoP, and h_1 is the height below CoP.
 *
 * If vertfov_total is set to 0, it will be computed from h_total.
 *
 * Distances are in arbitrary but consistent units. Angles are in radians.
 *
 *
 * In the diagram below, treating it like a FOV for horizontal,
 * the top angle is horizfov_total, the length of the bottom
 * is w_total, and the distance between the vertical line and the left corner is
 * w_1. Vertical is similar - h_1 is above the center line.
 * The triangle need not be symmetrical, despite how the diagram looks.
 *
 * ```
 *               horizfov_total
 *                       *
 * angle_left (neg) -> / |  \ <- angle_right
 *                    /  |   \
 *                   /   |    \
 *                  /    |     \
 *                 -------------
 *                 [ w_1 ]
 *                 [ --- w  --- ]
 *
 * -------     --- |\
 *                 |   \
 *             h_1 |      \ angle_up
 * h_total     ___ |-------* vertfov_total
 *                 |      / angle_down (neg)
 *                 |    /
 *                 |  /
 * -------         |/
 * ```
 *
 * @return true if successful.
 */
bool
math_compute_fovs(double w_total,
                  double w_1,
                  double horizfov_total,
                  double h_total,
                  double h_1,
                  double vertfov_total,
                  struct xrt_fov *fov);

#ifdef __cplusplus
}
#endif
