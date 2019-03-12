// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: Apache-2.0
/*!
 * @file
 * @brief  OpenHMD prober code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include <stdio.h>
#include <stdlib.h>

#include <survive_api.h>
#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "survive_device.h"

DEBUG_GET_ONCE_BOOL_OPTION(survive_spew, "SURVIVE_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(survive_debug, "SURVIVE_PRINT_DEBUG", false)

struct survive_prober
{
	struct xrt_prober base;
	SurviveSimpleContext *ctx;
};

static inline struct survive_prober *
survive_prober(struct xrt_prober *p)
{
	return (struct survive_prober *)p;
}

static void
survive_prober_destroy(struct xrt_prober *p)
{
	struct survive_prober *sp = survive_prober(p);

	if (sp->ctx != NULL) {
		/// @todo: This segfaults
		// survive_simple_close(ohp->ctx);
		sp->ctx = NULL;
	}

	free(sp);
}

static struct xrt_device *
survive_prober_autoprobe(struct xrt_prober *p)
{
	struct survive_prober *sp = survive_prober(p);


	bool print_spew = debug_get_bool_option_survive_spew();
	bool print_debug = debug_get_bool_option_survive_debug();

	/// @todo: with libsurvive, whether a context can be created tells us if
	/// a supported device is connected and readable.

	struct survive_device *sd =
	    survive_device_create(print_spew, print_debug);

	if (sd == NULL || sd->ctx == NULL)
		return NULL;

	sp->ctx = sd->ctx;

	return &sd->base;
}

struct xrt_prober *
survive_create_prober()
{
	struct survive_prober *sp = calloc(1, sizeof(struct survive_prober));
	sp->base.destroy = survive_prober_destroy;
	sp->base.lelo_dallas_autoprobe = survive_prober_autoprobe;

	return &sp->base;
}
