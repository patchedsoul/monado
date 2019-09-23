// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to direct LG R100 driver code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup drv_lgr100
 */

#pragma once

#include "math/m_api.h"
#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif

struct os_hid_device;

struct lgr100_device *
lgr100_device_create(struct os_hid_device *dev,
                     bool print_spew,
                     bool print_debug);


#ifdef __cplusplus
}
#endif
