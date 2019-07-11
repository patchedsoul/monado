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

        double cy = cos(euler.z* 0.5);
        double sy = sin(euler.z * 0.5);
        double cp = cos(euler.x * 0.5);
        double sp = sin(euler.x * 0.5);
        double cr = cos(euler.y * 0.5);
        double sr = sin(euler.y * 0.5);

        q->w = cy * cp * cr + sy * sp * sr;
        q->x = cy * cp * sr - sy * sp * cr;
        q->y = sy * cp * sr + cy * sp * cr;
        q->z = sy * cp * cr - cy * sp * sr;

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
