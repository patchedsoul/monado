// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief low level libsurvive wrapper
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @ingroup drv_survive
 */

#define SURVIVE_ENABLE_FULL_API 1
#include "survive_api.h"

#include "survive_wrap.h"

// TODO: expose config values we need through actual survive API
#include "../../external/libsurvive/include/libsurvive/survive.h"

bool
survive_config_ready (const SurviveSimpleObject *sso)
{
	SurviveObject *so = survive_simple_get_survive_object(sso);
	return so->conf != 0;
}

char *
survive_get_json_config(const SurviveSimpleObject *sso)
{
	SurviveObject *so = survive_simple_get_survive_object(sso);
	return so->conf;
}
