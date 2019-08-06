// Copyright 2016, Joey Ferwerda.
// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR device header, imported from OpenHMD.
 * @author Joey Ferwerda <joeyferweda@gmail.com>
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_psvr
 */

#pragma once

#include "xrt/xrt_device.h"

#include <hidapi.h>

#include <optical_tracking/common/calibration.h>
#include <optical_tracking/common/tracker.h>
#include <filters/filter_complementary.h>


#include <mt_framequeue.h>
#include <pthread.h>
#include <sys/time.h>


#ifdef __cplusplus
extern "C" {
#endif


/*
 *
 * Defines
 *
 */

#define PSVR_VID 0x054c
#define PSVR_PID 0x09af

#define PSVR_HANDLE_IFACE 4
#define PSVR_CONTROL_IFACE 5

enum psvr_status_bits
{
	// clang-format off
	PSVR_STATUS_BIT_POWER                = (1 << 0),
	PSVR_STATUS_BIT_HMD_WORN             = (1 << 1),
	PSVR_STATUS_BIT_CINEMATIC_MODE       = (1 << 2),
	PSVR_STATUS_BIT_UNKNOWN_BIT_3        = (1 << 3),
	PSVR_STATUS_BIT_HEADPHONES_CONNECTED = (1 << 4),
	PSVR_STATUS_BIT_MUTE_ENABLED         = (1 << 5),
	PSVR_STATUS_BIT_UNKNOWN_BIT_6        = (1 << 6),
	PSVR_STATUS_BIT_UNKNOWN_BIT_7        = (1 << 7),
	// clang-format on
};

#define PSVR_STATUS_VR_MODE_OFF 0
#define PSVR_STATUS_VR_MODE_ON 1

#define PSVR_TICK_PERIOD (1.0f / 1000000.0f) // 1 MHz ticks

#define PSVR_PKG_STATUS 0xF0
#define PSVR_PKG_0xA0 0xA0


/*
 *
 * Structs
 *
 */

/*!
 * A single gyro, accel and tick sample.
 *
 * @ingroup drv_psvr
 */
struct psvr_sensor_sample
{
	int16_t accel[3];
	int16_t gyro[3];
	uint32_t tick;
};

/*!
 * A parsed sensor packet from the headset.
 *
 * @ingroup drv_psvr
 */
struct psvr_sensor_packet
{
	uint8_t buttons;
	uint8_t state;
	uint16_t volume;
	struct psvr_sensor_sample samples[2];
	uint16_t button_raw;
	uint16_t proximity;
	uint8_t seq;
};

/*!
 * A parsed status packet from the headset.
 *
 * @ingroup drv_psvr
 */
struct psvr_status_packet
{
	uint8_t status;
	uint8_t volume;
	uint8_t display_time;
	uint8_t vr_mode;
};


/*
 *
 * Functions
 *
 */

struct xrt_device *
psvr_device_create(struct hid_device_info *hmd_handle_info,
                   struct hid_device_info *hmd_control_info,
                   bool print_spew,
                   bool print_debug);

bool
psvr_parse_sensor_packet(struct psvr_sensor_packet *packet,
                         const uint8_t *buffer,
                         int size);

bool
psvr_parse_status_packet(struct psvr_status_packet *packet,
                         const uint8_t *buffer,
                         int size);


/*
 *
 * Printing functions.
 *
 */

#define PSVR_SPEW(p, ...)                                                      \
	do {                                                                   \
		if (p->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)
#define PSVR_DEBUG(p, ...)                                                     \
	do {                                                                   \
		if (p->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define PSVR_ERROR(p, ...)                                                     \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)


#ifdef __cplusplus
}
#endif
