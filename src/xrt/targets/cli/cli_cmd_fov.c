// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Simple fov testing stuff.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "cli_common.h"
#include "util/u_distortion.h"

#include <string.h>
#include <stdio.h>
#include <math.h>


static void
dump_fov(struct u_dist_lens *l)
{
	struct xrt_fov fov = u_dist_lens_get_fov(l);
	fprintf(stderr, "\textents:\n");
	fprintf(stderr, "\t\tleft:   %+f\n", l->extents.left);
	fprintf(stderr, "\t\tright:  %+f\n", l->extents.right);
	fprintf(stderr, "\t\ttop:    %+f\n", l->extents.top);
	fprintf(stderr, "\t\tbottom: %+f\n", l->extents.bottom);
	fprintf(stderr, "\tfocal:\n");
	fprintf(stderr, "\t\tcenter.x: %+f\n", l->focal.center.x);
	fprintf(stderr, "\t\tcenter.y: %+f\n", l->focal.center.y);
	fprintf(stderr, "\t\tlength.x: %+f\n", l->focal.length.x);
	fprintf(stderr, "\t\tlength.y: %+f\n", l->focal.length.y);
#if 0
	if (l->dist.num_distortion_ks > 0) {
		fprintf(stderr, "\tdistortion:\n");
	}
	for (size_t i = 0; i < l->prd.num_distortion_ks; i++) {
		fprintf(stderr, "\t\tk%zi: %+f\n", i + 1,
		        l->dist.ks[i]);
	}
#endif
	fprintf(stderr, "\tfov:\n");
	fprintf(stderr, "\t\tleft:  %+f\n", fov.angle_left * (180 / M_PI));
	fprintf(stderr, "\t\tright: %+f\n", fov.angle_right * (180 / M_PI));
	fprintf(stderr, "\t\tup:    %+f\n", fov.angle_up * (180 / M_PI));
	fprintf(stderr, "\t\tdown:  %+f\n", fov.angle_down * (180 / M_PI));
}

int
cli_cmd_fov(int argc, const char **argv)
{
	struct u_dist_lens l = {0};


	/*
	 * Unit
	 */

	l.extents.left = -1.0;
	l.extents.right = 1.0;
	l.extents.top = 1.0;
	l.extents.bottom = -1.0;

	l.focal.center.x = 0.0;
	l.focal.center.y = 0.0;
	l.focal.length.x = 1.0;
	l.focal.length.y = 1.0;

	fprintf(stderr, "[-1, +1] (no distortion):\n");
	dump_fov(&l);


	/*
	 * PS4
	 */

	l.extents.left = 0;
	l.extents.right = 640;
	l.extents.top = 400;
	l.extents.bottom = 0;

	l.dist.ks[0] = -0.003932995695271172;
	l.dist.ks[1] = -0.01088207565904425;
	l.dist.ks[2] = 0.0002152222960531657;
	l.dist.type = U_DIST_TYPE_POLY_3_COEFFS;

	l.focal.center.x = 318.9401456053543;
	l.focal.center.y = 198.0158519744873;
	l.focal.length.x = 421.8661759175948;
	l.focal.length.y = 422.021647335717;

	fprintf(stderr, "PS4 lens:\n");
	dump_fov(&l);


	/*
	 * Left
	 */

	l.extents.left = -1.0;
	l.extents.right = 1.0;
	l.extents.top = 1.0;
	l.extents.bottom = -1.0;

	l.dist.ks[0] = -0.2249331765791355;
	l.dist.ks[1] = -0.01667879213687431;
	l.dist.ks[2] = -0.05274798719289359;
	l.dist.type = U_DIST_TYPE_POLY_DIVISION_3_COEFFS_DOUBLE;

	l.focal.center.x = 0.09460586309432983;
	l.focal.center.y = -0.0009955627610906959;
	l.focal.length.x = 1.212877750396729;
	l.focal.length.y = 1.090953826904297;

	fprintf(stderr, "Vive left lens:\n");
	dump_fov(&l);


	/*
	 * Right
	 */

	l.dist.ks[0] = -0.2249331765791355;
	l.dist.ks[1] = -0.01667879213687431;
	l.dist.ks[2] = -0.05274798719289359;
	l.dist.type = U_DIST_TYPE_POLY_DIVISION_3_COEFFS_DOUBLE;

	l.focal.center.x = -0.09292662143707275;
	l.focal.center.y = -0.00308517855592072;
	l.focal.length.x = 1.209185719490051;
	l.focal.length.y = 1.088273167610169;

	fprintf(stderr, "Vive right lens:\n");
	dump_fov(&l);


	return 0;
}
