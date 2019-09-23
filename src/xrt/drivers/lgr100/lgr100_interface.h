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

#include <stdlib.h>
#include <xrt/xrt_prober.h>

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * @defgroup drv_lgr100 LGR100 Driver
 * @ingroup drv
 *
 * @brief Driver for the LGR100 HMD.
 */

#define LGR100_VID 0x1004
#define LGR100_PID 0x6374

/*!
 * Probing function for LGR100 devices.
 *
 * @ingroup drv_lgr100
 */
int
lgr100_found(struct xrt_prober *xp,
          struct xrt_prober_device **devices,
          size_t index,
          struct xrt_device **out_xdev);

/*!
 * @dir drivers/lgr100
 *
 * @brief @ref drv_lgr100 files.
 */


#ifdef __cplusplus
}
#endif
