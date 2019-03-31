// Copyright 2017, James Sarrett.
// Copyright 2017, Bastiaan Olij.
// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: James Sarrett <jsarrett@gmail.com>
// Author: Bastiaan Olij <mux213@gmail.com>
// Author: Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
// Author: Pete Black <pete.black@collabora.com>

#version 450

layout (binding = 0) uniform sampler2D texSampler;

layout (location = 0)      in vec2 inUV;
layout (location = 1) flat in int  inViewIndex;

layout (location = 0) out vec4 outColor;


void main()
{
	const int i = inViewIndex;


    vec3 color = texture(texSampler, inUV).xyz;


    outColor = vec4(1.0f,0.0f,0.0f, 1.0);
}
