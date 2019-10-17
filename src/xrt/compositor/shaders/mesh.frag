// Copyright 2017, James Sarrett.
// Copyright 2017, Bastiaan Olij.
// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
// Author: James Sarrett <jsarrett@gmail.com>
// Author: Bastiaan Olij <mux213@gmail.com>
// Author: Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
// Author: Pete Black <pete.black@collabora.com>

#version 450

layout (binding = 0) uniform sampler2D texSampler[2];
layout (binding = 1, std140) uniform UBO
{
	// Distoriton coefficients (PanoTools model) [a,b,c,d]
	vec4 HmdWarpParam;

	// chromatic distortion post scaling
	vec4 aberr;

	// Position of lens center in m (usually eye_w/2, eye_h/2)
	vec2 LensCenter[2];

	// Scale from texture co-ords to m (usually eye_w, eye_h)
	vec2 ViewportScale;

	// Distortion overall scale in m (usually ~eye_w/2)
	float WarpScale;
} ubo;

layout (location = 0) in vec2 in_ruv;
layout (location = 1) in vec2 in_guv;
layout (location = 2) in vec2 in_buv;

layout (location = 0) out vec4 out_color;

void main()
{
    vec2 ruv=in_ruv;
    ruv.y =1.0-ruv.y;
    vec2 guv=in_guv;
    guv.y =1.0-guv.y;
    vec2 buv=in_buv;
    buv.y =1.0-buv.y;


    vec3 rcolor = texture(texSampler[0], ruv).xyz;
    vec3 gcolor = texture(texSampler[0], guv).xyz;
    vec3 bcolor = texture(texSampler[0], buv).xyz;

#if 0
        if (in_ruv.x < 0.0 || in_ruv.x > 1.0 || in_ruv.y < 0.0 || in_ruv.y > 1.0) {
		color = vec3(1.0, 0.0, 1.0);
	} else {
                float t = floor(in_ruv.x * 16) + floor(in_ruv.y * 16);
		bool isEven = mod(t, 2.0) == 0.0;
		// color = color * float(isEven);
		color = vec3(isEven, isEven, isEven);
	}
#endif

        out_color = vec4(rcolor.x,gcolor.y,bcolor.z, 1.0);
}
