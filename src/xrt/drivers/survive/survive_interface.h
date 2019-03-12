// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: Apache-2.0
/*!
 * @file
 * @brief  Interface to OpenHMD driver code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define HTC_VID 0x0bb4
#define VALVE_VID 0x28de

#define VIVE_PID 0x2c87
#define VIVE_LIGHTHOUSE_FPGA_RX 0x2000

#define VIVE_PRO_MAINBOARD_PID 0x0309
#define VIVE_PRO_LHR_PID 0x2300

int
survive_found(struct xrt_prober *xp,
              struct xrt_prober_device **devices,
              size_t num_devices,
              size_t index,
              struct xrt_device **out_xdevs);

#ifdef __cplusplus
}
#endif
