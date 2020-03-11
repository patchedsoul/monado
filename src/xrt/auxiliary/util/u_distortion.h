// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Distortion utilities.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"

#define U_DIST_MAX_DISTORTION_KS (5)

/*!
 * What type of radial distortion.
 */
enum u_dist_type
{
	//! No distortion.
	U_DIST_TYPE_NONE = 0,
	//! rC = r * (1 + r2*k1)
	U_DIST_TYPE_POLY_1_COEFFS = 1,
	//! rC = r * (1 + r2*k1 + r4*k2)
	U_DIST_TYPE_POLY_2_COEFFS = 2,
	//! rC = r * (1 + r2*k1 + r4*k2 + ...)
	U_DIST_TYPE_POLY_3_COEFFS = 3,
	//! rC = r * (1 + r2*k1 + r4*k2 + ...)
	U_DIST_TYPE_POLY_4_COEFFS = 4,
	//! rC = r * (1 + r2*k1 + r4*k2 + ...)
	U_DIST_TYPE_POLY_5_COEFFS = 5,
	//! rC = r / (1 + r*k1 + r2*k2 + r3*k3)
	U_DIST_TYPE_POLY_DIVISION_3_COEFFS_SINGLE = 6,
	//! rC = r / (1 + r2*k1 + r4*k2 + r6*k3)
	U_DIST_TYPE_POLY_DIVISION_3_COEFFS_DOUBLE = 7,
};

/*!
 * Radial Distortion struct implementing many radial based distortions.
 *
 * Polynomial Radial Distortion, as used in OpenCV and Google Cardboard.
 *
 * Xcorrected = X(1 + k[0] * r^2 + k[1] * r^4 + k[2] * r^6 ...)
 * Ycorrected = Y(1 + k[0] * r^2 + k[1] * r^4 + k[2] * r^6 ...)
 */
struct u_dist
{
	double ks[U_DIST_MAX_DISTORTION_KS];
	enum u_dist_type type;
};

double
u_dist_distort_factor(struct u_dist *rd, double r_squared);

static inline double
u_dist_distort_radius(struct u_dist *rd, double r)
{
	return r * u_dist_distort_factor(rd, r * r);
}

struct xrt_vec2
u_dist_distort_vec2(struct u_dist *rd, struct xrt_vec2 p);



/*!
 * A lens distorted with configurable distortion.
 */
struct u_dist_lens
{
	struct u_dist dist;

	struct
	{
		struct xrt_vec2 length;
		struct xrt_vec2 center;
	} focal;

	struct
	{
		float left;
		float right;
		float top;
		float bottom;
	} extents;
};

struct xrt_fov
u_dist_lens_get_fov(struct u_dist_lens *rdl);
