// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
// Author: Pete Black <pete.black@collabora.com>

#version 450

layout (location = 0) out vec2 outUV;
layout (location = 1) out int  outViewIndex;
layout (binding = 1, std140) uniform uvUBO
{
    vec2[17][17] uvs;
} ubo;

layout (binding = 2, std140) uniform UBO
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

	outViewIndex = ubo_vp.viewport_id;
    float vertInc = 1.0/17.0;
    int vertRow = gl_VertexIndex/17;
    int vertCol = gl_VertexIndex % 17;
    vec2 vertPos = vec2(vertRow * vertInc, vertCol * vertInc);
    outUV = ubo.uvs[vertRow][vertCol];

    gl_Position = vec4(rot * (vertPos * 2.0f - 1.0f), 0.0f, 1.0f);


    if (ubo_vp.flip_y)
		outUV.y = 1.0 - outUV.y;
}
