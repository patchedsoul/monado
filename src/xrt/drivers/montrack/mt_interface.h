// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Monado internal driver code.
 * @author Pete Black <pete.black@collabora.com>
 */

#pragma once

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_montrack "montrack" optical tracking driver
 * @ingroup drv
 *
 * @brief Driver for optical tracking.
 */

/*!
 * Probing function for optical tracking cameras.
 *
 * @ingroup drv_montrack
 */
struct xrt_auto_prober *
mt_create_auto_prober();


#define HDK_CAM_VID 0x0bda
#define HDK_CAM_PID 0x57e8

/*!
 * Probing function for UVC devices.
 *
 * @ingroup drv_montrack
 */
int
mt_uvc_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t index,
             struct xrt_device **out_xdev);


#ifdef __cplusplus
}
#endif
