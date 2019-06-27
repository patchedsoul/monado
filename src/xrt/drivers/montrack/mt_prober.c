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
    frameserver_instance_t* frameserver;
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
    //TODO: create a 'dummy' device that just holds frameserver(s)

    //just used to create a calibration tracker
    /*mt_device_t* mtd = mt_device_create("CALIBRATION_STEREO", true, true);

    mtd->frameserver_count= 1;
    mtd->frameservers = calloc(1,sizeof(void*));
    frameserver_config_request_t config_req;
    config_req.fps = 60;
    config_req.width = 1280;
    config_req.height = 480;
    mtd->frameservers[0] = mt_frameserver_create("PS4EYE",config_req,true,true);
*/
    return NULL;
    //return &mtd->base;
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
