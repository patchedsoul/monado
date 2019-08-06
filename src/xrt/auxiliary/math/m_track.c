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

        double c_yaw = cos(euler.z * 0.5);
        double s_yaw = sin(euler.z * 0.5);
        double c_pitch = cos(euler.x * 0.5);
        double s_pitch = sin(euler.x * 0.5);
        double c_roll = cos(euler.y * 0.5);
        double s_roll = sin(euler.y * 0.5);

        q->w = c_yaw * c_pitch * c_roll + s_yaw * s_pitch * s_roll;
        q->x = c_yaw * c_pitch * s_roll - s_yaw * s_pitch * c_roll;
        q->y = s_yaw * c_pitch * s_roll + c_yaw * s_pitch * c_roll;
        q->z = s_yaw * c_pitch * c_roll - c_yaw * s_pitch * s_roll;

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

float math_quat_magnitude(struct xrt_quat q) {
    return sqrt(q.x * q.x + q.y*q.y + q.z*q.z + q.w*q.w);
}

float
math_distance(struct xrt_vec3 a, struct xrt_vec3 b) {
    struct xrt_vec3 d;
    d.x = a.x - b.x;
    d.y = a.y - b.y;
    d.z = a.z - b.z;
    return sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}
