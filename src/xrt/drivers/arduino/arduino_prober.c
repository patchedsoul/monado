// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Arduino felxable input device prober code.
 * @author Pete Black <pete.black@collabora.com>
 * @ingroup drv_arduino
 */

#include <stdio.h>
#include <stdlib.h>
#include <wchar.h>

#include "os/os_ble.h"

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"
#include "util/u_debug.h"

#include "arduino_interface.h"


/*
 *
 * Defines & structs.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(arduino_enable, "ARDUINO_ENABLE", true)
DEBUG_GET_ONCE_BOOL_OPTION(arduino_spew, "ARDUINO_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(arduino_debug, "ARDUINO_PRINT_DEBUG", false)

/*!
 * Arduino prober struct.
 *
 * @ingroup drv_arduino
 */
struct arduino_prober
{
	struct xrt_auto_prober base;

	bool print_spew;
	bool print_debug;
	bool enabled;
};


/*
 *
 * Static functions.
 *
 */

static inline struct arduino_prober *
arduino_prober(struct xrt_auto_prober *p)
{
	return (struct arduino_prober *)p;
}

static void
arduino_prober_destroy(struct xrt_auto_prober *p)
{
	struct arduino_prober *ap = arduino_prober(p);

	free(ap);
}

static struct xrt_device *
arduino_prober_autoprobe(struct xrt_auto_prober *xap,
                         bool no_hmds,
                         struct xrt_prober *xp)
{
	struct arduino_prober *ap = arduino_prober(xap);
	if (!ap->enabled) {
		return NULL;
	}

	const char *dev_uuid = "00004242-0000-1000-8000-004242424242";
	const char *char_uuid = "00000001-1000-1000-8000-004242424242";

	struct os_ble_device *ble = NULL;
	os_ble_notify_open(dev_uuid, char_uuid, &ble);
	if (ble == NULL) {
		return NULL;
	}

	return arduino_device_create(ble, ap->print_spew, ap->print_debug);
}


/*
 *
 * Exported functions.
 *
 */

struct xrt_auto_prober *
arduino_create_auto_prober()
{
	struct arduino_prober *ap = U_TYPED_CALLOC(struct arduino_prober);
	ap->base.destroy = arduino_prober_destroy;
	ap->base.lelo_dallas_autoprobe = arduino_prober_autoprobe;
	ap->enabled = debug_get_bool_option_arduino_enable();
	ap->print_spew = debug_get_bool_option_arduino_spew();
	ap->print_debug = debug_get_bool_option_arduino_debug();

	return &ap->base;
}
