// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A cli program to configure and test Monado.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "util/u_sink.h"
#include "util/u_format.h"

#include "tracking/t_tracking.h"

#include "cli_common.h"

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#define P(...) fprintf(stderr, __VA_ARGS__)

#if 0
static int
cli_print_help(int argc, const char **argv)
{
	if (argc >= 2) {
		fprintf(stderr, "Unknown command '%s'\n\n", argv[1]);
	}

	P("Monado-CLI 0.0.1\n");
	P("Usage: %s command [options]\n", argv[0]);
	P("\n");
	P("Commands:\n");
	P("  test       - List found devices, for prober testing.\n");
	P("  calibrate  - Calibrate a camera and save config.\n");

	return 1;
}
#endif

#include "v4l2/v4l2_interface.h"

static void
print_modes(struct xrt_fs *xfs)
{
	struct xrt_fs_mode *modes = NULL;
	uint32_t count = 0;

	xrt_fs_enumerate_modes(xfs, &modes, &count);

	P("Got %u mode%s\n", count, count == 1 ? "" : "s");

	for (uint32_t i = 0; i < count; i++) {
		P("%6i - %dx%d %s\n", i, modes[i].width, modes[i].height,
		  u_format_str(modes[i].format));
	}

	free(modes);
}

static void
push_frame(struct xrt_frame_sink *sink, struct xrt_frame *f)
{
	P("%s - Got frame #%u %ux%u\n", __func__, (uint32_t)f->source_sequence,
	  f->width, f->height);
}

int
main(int argc, const char **argv)
{
#if 0
	if (argc <= 1) {
		return cli_print_help(argc, argv);
	}

	if (strcmp(argv[1], "test") == 0) {
		return cli_cmd_test(argc, argv);
	}
	if (strcmp(argv[1], "calibrate") == 0) {
		return cli_cmd_calibrate(argc, argv);
	}

	return cli_print_help(argc, argv);
#endif

	if (argc != 3) {
		fprintf(stderr, "Must supply exactly two arguments!\n");
		fprintf(stderr, "\tdevice file\n");
		fprintf(stderr, "\tmode number\n");
	}

	struct xrt_frame_context xfctx = {0};
	struct xrt_frame_sink cli_sink = {0};
	cli_sink.push_frame = push_frame;

	struct xrt_frame_sink *xsink = &cli_sink;

	struct xrt_frame_sink *xsinks[4] = {&cli_sink, NULL, NULL, NULL};
	struct t_hsv_filter_params params = T_HSV_DEFAULT_PARAMS();
	t_hsv_filter_create(&xfctx, &params, xsinks, &xsink);

	// u_sink_queue_create(&xfctx, xsink, &xsink);
	// u_sink_create_format_converter(&xfctx, XRT_FORMAT_R8G8B8, xsink,
	//                                &xsink);
	u_sink_queue_create(&xfctx, xsink, &xsink);

	struct xrt_fs *xfs = v4l2_fs_create(&xfctx, argv[1]);
	if (xfs == NULL) {
		fprintf(stderr, "Failed to create frameserver!\n");
		return -1;
	}

	// Just to be helpful
	print_modes(xfs);

	// Start it up!
	xrt_fs_stream_start(xfs, xsink, atoi(argv[2]));

	// Sleep five seconds.
	usleep(3 * 1000 * 1000);

	// Stop it and destroy it.
	xrt_fs_stream_stop(xfs);

	// Tear everything down.
	xrt_frame_context_destroy_nodes(&xfctx);

	return 0;
}
