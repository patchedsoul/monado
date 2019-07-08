// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Functions related to tracking.
 * @author Pete Black <pete.black@collabora.com>
 * @ingroup aux_math
 */


#include "m_api.h"
#include "util/u_debug.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>


void
math_euler_to_quat(struct xrt_vec3 euler, struct xrt_quat* q)
{
	//STUB
	q->x=0.0f;
	q->y=0.0f;
	q->z=0.0f;
	q->w=1.0f;

}

int
math_min(int a, int b) {
    if (a > b) {
        return b;
    }
    return a;
}

int
math_max(int a, int b) {
    if (a > b) {
        return a;
    }
    return b;
}
