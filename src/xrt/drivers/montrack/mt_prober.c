// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Monado internal prober code.
 * @author Pete Black <pete.black@collabora.com>
 */

#include <stdio.h>
#include <stdlib.h>

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "mt_device.h"

DEBUG_GET_ONCE_BOOL_OPTION(mt_verbose, "MT_VERBOSE", false)
DEBUG_GET_ONCE_BOOL_OPTION(mt_debug, "MT_DEBUG", false)

typedef struct mt_prober
{
	struct xrt_auto_prober base;
	bool log_verbose;
	bool log_debug;
} mt_prober_t;

static inline mt_prober_t*
mt_prober(struct xrt_auto_prober* xp)
{
	return (mt_prober_t*)xp;
}

static void
mt_prober_destroy(struct xrt_auto_prober* xp)
{
	mt_prober_t* mp = mt_prober(xp);

	free(mp);
}

static struct xrt_device*
mt_prober_autoprobe(struct xrt_auto_prober* p)
{
	// struct mt_prober* mp = mt_prober(p);

	// here we would call functions to consult our config, check devices
	// are present etc. - for now we will attempt to create a mono blob
	// tracker, with any uvc camera we can use

	// mt_device_t* mtd = mt_device_create("MONO_LOGITECH_C270",true,true);
	// mt_device_t* mtd = mt_device_create("STEREO_ELP_60FPS",true,true);
	// mt_device_t* mtd = mt_device_create("MONO_PS3EYE",true,true);

	// mt_device_t* mtd =
	// mt_device_create("STEREO_LOGITECH_C270",true,true);
	mt_device_t* mtd = mt_device_create("STEREO_PS4_60FPS", true, true);

	return &mtd->base;
}

struct xrt_auto_prober*
mt_create_auto_prober()
{
	mt_prober_t* mp = U_TYPED_CALLOC(mt_prober_t);
	mp->base.destroy = mt_prober_destroy;
	mp->base.lelo_dallas_autoprobe = mt_prober_autoprobe;
	mp->log_verbose = debug_get_bool_option_mt_verbose();
	mp->log_debug = debug_get_bool_option_mt_debug();

	return &mp->base;
}
