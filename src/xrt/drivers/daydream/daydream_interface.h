// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to @ref drv_daydream.
 * @author Pete Black <pete.black@collabora.com>
 * @ingroup drv_psmv
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_daydream Daydream Controller driver
 * @ingroup drv
 *
 * @brief Driver for the Google Daydream Controller.
 */


#define DAYDREAM_BLUEZ_PATH service002a/char002b

/*!
 * Probing function for the Daydream controller.
 *
 * @ingroup drv_daydream
 */
int
daydream_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t num_devices,
           size_t index,
           struct xrt_device **out_xdevs);

/*!
 * @dir drivers/daydream
 *
 * @brief @ref drv_daydream files.
 */


#ifdef __cplusplus
}
#endif
