// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Wrapper around OS native BLE functions.
 * @author Pete Black <pete.black@collabora.com>
 *
 * @ingroup aux_os
 */

#pragma once

#include "xrt/xrt_config.h"
#include "xrt/xrt_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * Representing a single ble notify attribute on a device.
 */
struct os_ble_device
{
    int (*read)(struct os_ble_device *ble_dev,
	            uint8_t *data,
                size_t size);

    void (*destroy)(struct os_ble_device *ble_dev);
};

/*!
 * Read data from the ble file descriptor
 * immediately returns bytes read,
 *  -1 if no data avaialable
 */

XRT_MAYBE_UNUSED static inline int
os_ble_read(struct os_ble_device *ble_dev,
            uint8_t *data,
            size_t size)
{
    return ble_dev->read(ble_dev, data, size);
}


/*!
 * Close and free the given device.
 */
XRT_MAYBE_UNUSED static inline void
os_ble_destroy(struct os_ble_device *ble_dev)
{
    ble_dev->destroy(ble_dev);
}

#ifdef XRT_OS_LINUX
/*!
 * Open the given mac and path to device endpoint (Currently Linux/BlueZ specific).
 */
int
os_ble_notify_open(const char *mac, const char* endpoint,struct os_ble_device **out_ble);
#endif

#ifdef __cplusplus
} // extern "C"
#endif
