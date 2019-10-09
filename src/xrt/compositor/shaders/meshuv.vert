// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
// Author: Pete Black <pete.black@collabora.com>

#version 450

layout(location = 0) in vec4 pos_uv;
layout (location = 0) out vec2 outUV;

layout (binding = 2, std140) uniform ubo
{
	vec4 rot;
	int viewport_id;
	bool flip_y;
} ubo_vp;

out gl_PerVertex
{
	vec4 gl_Position;
};


void main()
{
        mat2x2 rot = {
                ubo_vp.rot.xy,
                ubo_vp.rot.zw,
        };

    outUV = pos_uv.zw;

    gl_Position = vec4(2.0 * rot * (pos_uv.xy-vec2(0.5,0.5)), 0.0f, 1.0f);

    if (ubo_vp.flip_y)
        outUV.y = 1.0 - outUV.y;
}
