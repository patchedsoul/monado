// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Distortion code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "util/u_distortion.h"
#include "math/m_vec2.h"

#include <stdio.h>
#include <assert.h>


/*
 *
 * Distortion functions.
 *
 */

static double
dist_1_factor(double *ks, double r2)
{
	double k1 = ks[0];

	return 1.0 + r2 * k1;
}

static double
dist_2_factor(double *ks, double r2)
{
	double k1 = ks[0];

	return 1.0 + r2 * k1;
}

static double
dist_3_factor(double *ks, double r2)
{
	double k1 = ks[0];
	double k2 = ks[1];
	double k3 = ks[2];

	double r4 = r2 * r2;
	double r6 = r4 * r2;

	return 1.0 + r2 * k1 + r4 * k2 + r6 * k3;
}

static double
dist_4_factor(double *ks, double r2)
{
	double k1 = ks[0];
	double k2 = ks[1];
	double k3 = ks[2];
	double k4 = ks[3];

	double r4 = r2 * r2;
	double r6 = r4 * r2;
	double r8 = r6 * r2;

	return 1.0 + r2 * k1 + r4 * k2 + r6 * k3 + r8 * k4;
}

static double
dist_5_factor(double *ks, double r2)
{
	double k1 = ks[0];
	double k2 = ks[1];
	double k3 = ks[2];
	double k4 = ks[3];
	double k5 = ks[4];

	double r4 = r2 * r2;
	double r6 = r4 * r2;
	double r8 = r4 * r4;
	double r10 = r8 * r2;

	return 1.0 + r2 * k1 + r4 * k2 + r6 * k3 + r8 * k4 + r10 * k5;
}

static double
dist_d3_single_factor(double *ks, double r2)
{
	double k1 = ks[0];
	double k2 = ks[1];
	double k3 = ks[2];

	double r1 = sqrt(r2);
	double r3 = r2 * r1;

	double inv = 1.0 + r1 * k1 + r2 * k2 + r3 * k3;

	return 1.0 / inv;
}

static double
dist_d3_double_factor(double *ks, double r2)
{
	return 1.0 / dist_3_factor(ks, r2);
}



/*
 *
 * Exported functions.
 *
 */

double
u_dist_distort_factor(struct u_dist *dist, double r_squared)
{
	switch (dist->type) {
	case U_DIST_TYPE_NONE: return 1.0;
	case U_DIST_TYPE_POLY_1_COEFFS:
		return dist_1_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_2_COEFFS:
		return dist_2_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_3_COEFFS:
		return dist_3_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_4_COEFFS:
		return dist_4_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_5_COEFFS:
		return dist_5_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_DIVISION_3_COEFFS_SINGLE:
		return dist_d3_single_factor(dist->ks, r_squared);
	case U_DIST_TYPE_POLY_DIVISION_3_COEFFS_DOUBLE:
		return dist_d3_double_factor(dist->ks, r_squared);
	default: assert(false);
	}
}

struct xrt_vec2
u_dist_distort_vec2(struct u_dist *dist, struct xrt_vec2 p)
{
	float r_squared = m_vec2_len_sqrd(p);
	float factor = u_dist_distort_factor(dist, r_squared);
	struct xrt_vec2 ret = {factor * p.x, factor * p.y};
	return ret;
}

struct xrt_fov
u_dist_lens_get_fov(struct u_dist_lens *distl)
{
	double left = distl->extents.left;
	double down = distl->extents.bottom;
	double right = distl->extents.right;
	double up = distl->extents.top;

	left = (left - distl->focal.center.x) / distl->focal.length.x;
	right = (right - distl->focal.center.x) / distl->focal.length.x;

	up = (up - distl->focal.center.y) / distl->focal.length.y;
	down = (down - distl->focal.center.y) / distl->focal.length.y;

	left = u_dist_distort_vec2(&distl->dist, (struct xrt_vec2){left, 0}).x;
	right =
	    u_dist_distort_vec2(&distl->dist, (struct xrt_vec2){right, 0}).x;
	up = u_dist_distort_vec2(&distl->dist, (struct xrt_vec2){0, up}).y;
	down = u_dist_distort_vec2(&distl->dist, (struct xrt_vec2){0, down}).y;

	struct xrt_fov ret;
	ret.angle_left = atan(left);
	ret.angle_right = atan(right);
	ret.angle_up = atan(up);
	ret.angle_down = atan(down);

	return ret;
}
