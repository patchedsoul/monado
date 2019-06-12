// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to @ref drv_psmv.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_psmv
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_psmv PSVR driver
 * @ingroup drv
 *
 * @brief Driver for the Sony PlayStation Move Controller.
 */


#define PSMV_VID 0x054c
#define PSMV_PID 0x03d5

/*!
 * Probing function for the PSMove devices.
 *
 * @ingroup drv_psmv
 */
int
psmv_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t index,
           struct xrt_device **out_xdev);

/*!
 * @dir drivers/psmv
 *
 * @brief @ref drv_psmv files.
 */


#ifdef __cplusplus
}
#endif
