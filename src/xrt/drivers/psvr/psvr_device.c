// Copyright 2016, Joey Ferwerda.
// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR device implementation, imported from OpenHMD.
 * @author Joey Ferwerda <joeyferweda@gmail.com>
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_psvr
 */

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_tracking.h"

#include "os/os_time.h"

#include "math/m_api.h"

#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "psvr_device.h"


/*
 *
 * Structs and defines.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(psvr_disco, "PSVR_DISCO", false)

#define FEATURE_BUFFER_SIZE 256

/*!
 * Private struct for the @ref drv_psvr device.
 *
 * @ingroup drv_psvr
 */
struct psvr_device
{
	struct xrt_device base;

	hid_device *hmd_handle;
	hid_device *hmd_control;

	struct xrt_tracked_psvr *tracker;

	struct psvr_parsed_sensor last;

	struct
	{
		uint8_t leds[9];
	} wants;

	struct
	{
		uint8_t leds[9];
	} state;

	struct
	{
		struct xrt_vec3 gyro;
		struct xrt_vec3 accel;
	} read;

	uint16_t buttons;

	bool powered_on;
	bool in_vr_mode;

	bool print_spew;
	bool print_debug;

	struct
	{
		bool last_frame;
		bool control;
	} gui;

	struct
	{
		struct xrt_quat rot;
	} fusion;
};


// Alternative way to turn on all of the leds.
XRT_MAYBE_UNUSED static const unsigned char psvr_tracking_on[12] = {
    0x11, 0x00, 0xaa, 0x08, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
};


#define PSVR_LED_POWER_OFF ((uint8_t)0x00)
#define PSVR_LED_POWER_MAX ((uint8_t)0xff)

#define PSVR_LED_POWER_WIRE_OFF ((uint8_t)0)
#define PSVR_LED_POWER_WIRE_MAX ((uint8_t)100)


enum psvr_leds
{
	PSVR_LED_A = (1 << 0),
	PSVR_LED_B = (1 << 1),
	PSVR_LED_C = (1 << 2),
	PSVR_LED_D = (1 << 3),
	PSVR_LED_E = (1 << 4),
	PSVR_LED_F = (1 << 5),
	PSVR_LED_G = (1 << 6),
	PSVR_LED_H = (1 << 7),
	PSVR_LED_I = (1 << 8),

	PSVR_LED_FRONT = PSVR_LED_A | PSVR_LED_B | PSVR_LED_C | PSVR_LED_D |
	                 PSVR_LED_E | PSVR_LED_F | PSVR_LED_G,

	PSVR_LED_BACK = PSVR_LED_H | PSVR_LED_I,

	PSVR_LED_ALL = PSVR_LED_FRONT | PSVR_LED_BACK,
};


/*
 *
 * Helpers and internal functions.
 *
 */

static inline struct psvr_device *
psvr_device(struct xrt_device *p)
{
	return (struct psvr_device *)p;
}

static int
open_hid(struct psvr_device *p,
         struct hid_device_info *dev_info,
         hid_device **out_dev)
{
	hid_device *dev = NULL;
	int ret;

	dev = hid_open_path(dev_info->path);
	if (dev == NULL) {
		PSVR_ERROR(p, "Failed to open '%s'", dev_info->path);
		return -1;
	}

	ret = hid_set_nonblocking(dev, 1);
	if (ret != 0) {
		PSVR_ERROR(p, "Failed to set non-blocking on device");
		hid_close(dev);
		return -1;
	}

	*out_dev = dev;
	return 0;
}

static int
send_to_control(struct psvr_device *psvr, const uint8_t *data, size_t size)
{
	return hid_write(psvr->hmd_control, data, size);
}


/*
 *
 * Packet reading code.
 *
 */

static uint8_t
scale_led_power(uint8_t power)
{
	return (uint8_t)((power / 255.0f) * 100.0f);
}

static void
accel_from_psvr_vec(const struct xrt_vec3_i32 *accel, struct xrt_vec3 *out_vec)
{
	out_vec->x = accel->x * (MATH_GRAVITY_M_S2 / 16384.0);
	out_vec->y = accel->y * (MATH_GRAVITY_M_S2 / 16384.0);
	out_vec->z = accel->z * (MATH_GRAVITY_M_S2 / 16384.0);
}

static void
gyro_from_psvr_vec(const struct xrt_vec3_i32 *gyro, struct xrt_vec3 *out_vec)
{
	out_vec->x = gyro->x * 0.00105;
	out_vec->y = gyro->y * 0.00105;
	out_vec->z = gyro->z * 0.00105;
}

static void
update_fusion(struct psvr_device *psvr,
              struct psvr_parsed_sample *sample,
              uint32_t tick_delta)
{
	struct xrt_vec3 mag = {0.0f, 0.0f, 0.0f};
	(void)mag;

	accel_from_psvr_vec(&sample->accel, &psvr->read.accel);
	gyro_from_psvr_vec(&sample->gyro, &psvr->read.gyro);

	if (psvr->tracker != NULL) {
		time_duration_ns delta_ns =
		    tick_delta * (1000000000.0 / PSVR_TICKS_PER_SECOND);

		struct xrt_tracking_sample sample;
		sample.accel_m_s2 = psvr->read.accel;
		sample.gyro_rad_secs = psvr->read.gyro;

		xrt_tracked_psvr_push_imu(psvr->tracker, delta_ns, &sample);
	} else {
		float delta_secs = tick_delta / PSVR_TICKS_PER_SECOND;

		math_quat_integrate_velocity(&psvr->fusion.rot,
		                             &psvr->read.gyro, delta_secs,
		                             &psvr->fusion.rot);
	}
}

static uint32_t
calc_delta_and_handle_rollover(uint32_t next, uint32_t last)
{
	uint32_t tick_delta = next - last;

	// The 24-bit tick counter has rolled over,
	// adjust the "negative" value to be positive.
	if (tick_delta > 0xffffff) {
		tick_delta += 0x1000000;
	}

	return tick_delta;
}

static void
handle_tracker_sensor_msg(struct psvr_device *psvr,
                          unsigned char *buffer,
                          int size)
{
	uint32_t last_sample_tick = psvr->last.samples[1].tick;

	if (!psvr_parse_sensor_packet(&psvr->last, buffer, size)) {
		PSVR_ERROR(psvr, "couldn't decode tracker sensor message");
	}

	struct psvr_parsed_sensor *s = &psvr->last;

	// Simplest is the buttons.
	psvr->buttons = s->buttons;

	uint32_t tick_delta = 500;

	// Startup correction, ignore last_sample_tick if zero.
	if (last_sample_tick > 0) {
		tick_delta = calc_delta_and_handle_rollover(s->samples[0].tick,
		                                            last_sample_tick);

		// The PSVR device can buffer sensor data from previous
		// sessions which we can get at the start of new sessions.
		// @todo Maybe just skip the first 10 sensor packets?
		// @todo Maybe reset sensor fusion?
		if (tick_delta < 400 || tick_delta > 600) {
			PSVR_DEBUG(psvr, "tick_delta = %u", tick_delta);
			tick_delta = 500;
		}
	}

	// Update the fusion with first sample.
	update_fusion(psvr, &s->samples[0], tick_delta);

	// New delta between the two samples.
	tick_delta = calc_delta_and_handle_rollover(s->samples[1].tick,
	                                            s->samples[0].tick);

	// Update the fusion with second sample.
	update_fusion(psvr, &s->samples[1], tick_delta);
}

static void
handle_control_status_msg(struct psvr_device *psvr,
                          unsigned char *buffer,
                          int size)
{
	struct psvr_parsed_status status;

	if (!psvr_parse_status_packet(&status, buffer, size)) {
		PSVR_ERROR(psvr, "couldn't decode tracker sensor message");
	}


	/*
	 * Power
	 */

	if (status.status & PSVR_STATUS_BIT_POWER) {
		if (!psvr->powered_on) {
			PSVR_DEBUG(psvr, "Device powered on! '%02x'",
			           status.status);
		}
		psvr->powered_on = true;
	} else {
		if (psvr->powered_on) {
			PSVR_DEBUG(psvr, "Device powered off! '%02x'",
			           status.status);
		}
		psvr->powered_on = false;
	}


	/*
	 * VR-Mode
	 */

	if (status.vr_mode == PSVR_STATUS_VR_MODE_OFF) {
		if (psvr->in_vr_mode) {
			PSVR_DEBUG(psvr, "Device not in vr-mode! '%02x'",
			           status.vr_mode);
		}
		psvr->in_vr_mode = false;
	} else if (status.vr_mode == PSVR_STATUS_VR_MODE_ON) {
		if (!psvr->in_vr_mode) {
			PSVR_DEBUG(psvr, "Device in vr-mode! '%02x'",
			           status.vr_mode);
		}
		psvr->in_vr_mode = true;
	} else {
		PSVR_ERROR(psvr, "Unknown vr_mode status!");
	}
}

static void
handle_control_0xA0(struct psvr_device *psvr, unsigned char *buffer, int size)
{
	if (size < 4) {
		return;
	}

	PSVR_DEBUG(psvr, "%02x %02x %02x %02x", buffer[0], buffer[1], buffer[2],
	           buffer[3]);
}

static int
read_handle_packets(struct psvr_device *psvr)
{
	uint8_t buffer[FEATURE_BUFFER_SIZE];
	int size = 0;

	do {
		size = hid_read(psvr->hmd_handle, buffer, FEATURE_BUFFER_SIZE);
		if (size == 0) {
			return 0;
		}
		if (size < 0) {
			return -1;
		}

		handle_tracker_sensor_msg(psvr, buffer, size);
	} while (true);
}

static int
read_control_packets(struct psvr_device *psvr)
{
	uint8_t buffer[FEATURE_BUFFER_SIZE];
	int size = 0;

	do {
		size = hid_read(psvr->hmd_control, buffer, FEATURE_BUFFER_SIZE);
		if (size == 0) {
			return 0;
		}
		if (size < 0) {
			return -1;
		}

		if (buffer[0] == PSVR_PKG_STATUS) {
			handle_control_status_msg(psvr, buffer, size);
		} else if (buffer[0] == PSVR_PKG_0xA0) {
			handle_control_0xA0(psvr, buffer, size);
		} else {
			PSVR_DEBUG(psvr, "Got report, 0x%02x", buffer[0]);
		}

	} while (true);
}


/*
 *
 * Control sending functions.
 *
 */

static int
wait_for_power(struct psvr_device *psvr, bool on)
{
	for (int i = 0; i < 5000; i++) {
		read_handle_packets(psvr);
		read_control_packets(psvr);

		if (psvr->powered_on == on) {
			return 0;
		}

		os_nanosleep(1000 * 1000);
	}

	return -1;
}

static int
wait_for_vr_mode(struct psvr_device *psvr, bool on)
{
	for (int i = 0; i < 5000; i++) {
		read_handle_packets(psvr);
		read_control_packets(psvr);

		if (psvr->in_vr_mode == on) {
			return 0;
		}

		os_nanosleep(1000 * 1000);
	}

	return -1;
}

static int
control_power_and_wait(struct psvr_device *psvr, bool on)
{
	const char *status = on ? "on" : "off";
	const uint8_t data[8] = {
	    0x17, 0x00, 0xaa, 0x04, on, 0x00, 0x00, 0x00,
	};

	int ret = send_to_control(psvr, data, sizeof(data));
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to switch %s the headset! '%i'",
		           status, ret);
	}

	ret = wait_for_power(psvr, on);
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to wait for headset power %s! '%i'",
		           status, ret);
		return ret;
	}

	return ret;
}

static int
control_vrmode_and_wait(struct psvr_device *psvr, bool on)
{
	const uint8_t data[8] = {
	    0x23, 0x00, 0xaa, 0x04, on, 0x00, 0x00, 0x00,
	};
	int ret;

	ret = send_to_control(psvr, data, sizeof(data));
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed %s vr-mode the headset! '%i'",
		           on ? "enable" : "disable", ret);
		return ret;
	}

	ret = wait_for_vr_mode(psvr, on);
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to wait for vr mode! '%i'", ret);
		return ret;
	}

	return 0;
}

static int
update_leds_if_changed(struct psvr_device *psvr)
{
	if (memcmp(psvr->wants.leds, psvr->state.leds,
	           sizeof(psvr->state.leds)) == 0) {
		return 0;
	}

	memcpy(psvr->state.leds, psvr->wants.leds, sizeof(psvr->state.leds));

	uint8_t data[20] = {
	    0x15,
	    0x00,
	    0xaa,
	    0x10,
	    (uint8_t)PSVR_LED_ALL,
	    (uint8_t)(PSVR_LED_ALL >> 8),
	    scale_led_power(psvr->state.leds[0]),
	    scale_led_power(psvr->state.leds[1]),
	    scale_led_power(psvr->state.leds[2]),
	    scale_led_power(psvr->state.leds[3]),
	    scale_led_power(psvr->state.leds[4]),
	    scale_led_power(psvr->state.leds[5]),
	    scale_led_power(psvr->state.leds[6]),
	    scale_led_power(psvr->state.leds[7]),
	    scale_led_power(psvr->state.leds[8]),
	    0,
	    0,
	    0,
	    0,
	    0,
	};

	return send_to_control(psvr, data, sizeof(data));
}

/*!
 * Control the leds on the headset, allowing you to turn on and off different
 * leds with a single call.
 *
 * @param[in] psvr   The PSVR to control leds on.
 * @param[in] adjust The leds to adjust with @p power.
 * @param[in] power  The power level to give to @p adjust leds.
 * @param[in] off    Leds that should be turned off,
 *                   @p adjust has higher priority.
 * @ingroup drv_psvr
 */
static int
control_leds(struct psvr_device *psvr,
             enum psvr_leds adjust,
             uint8_t power,
             enum psvr_leds off)
{
	// Get the leds we should control and remove any extra bits.
	enum psvr_leds all = (enum psvr_leds)((adjust | off) & PSVR_LED_ALL);
	if (all == 0) {
		// Nothing todo.
		return 0;
	}

	for (uint32_t i = 0; i < ARRAY_SIZE(psvr->wants.leds); i++) {
		uint32_t mask = (1 << i);
		if (adjust & mask) {
			psvr->wants.leds[i] = power;
		}
		if (off & mask) {
			psvr->wants.leds[i] = 0x00;
		}
	}

	return update_leds_if_changed(psvr);
}

static int
disco_leds(struct psvr_device *psvr)
{
	static const uint16_t leds[] = {
	    // First loop
	    PSVR_LED_A,
	    PSVR_LED_E,
	    PSVR_LED_B,
	    PSVR_LED_G,
	    PSVR_LED_D,
	    PSVR_LED_C,
	    PSVR_LED_F,
	    // Second loop
	    PSVR_LED_A,
	    PSVR_LED_E,
	    PSVR_LED_B,
	    PSVR_LED_G,
	    PSVR_LED_D,
	    PSVR_LED_C,
	    PSVR_LED_F,
	    // Blink loop
	    PSVR_LED_BACK,
	    PSVR_LED_FRONT,
	    PSVR_LED_BACK,
	    PSVR_LED_FRONT,
	    // All on after loop
	    PSVR_LED_ALL,
	};

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		int ret = control_leds(psvr, (enum psvr_leds)leds[i],
		                       PSVR_LED_POWER_MAX, PSVR_LED_ALL);
		if (ret < 0) {
			return ret;
		}

		// Sleep for a tenth of a second while polling for packages.
		for (int k = 0; k < 100; k++) {
			ret = read_handle_packets(psvr);
			if (ret < 0) {
				return ret;
			}

			ret = read_control_packets(psvr);
			if (ret < 0) {
				return ret;
			}

			os_nanosleep(1000 * 1000);
		}
	}

	return 0;
}

static void
teardown(struct psvr_device *psvr)
{
	// Stop the variable tracking.
	u_var_remove_root(psvr);

	// Includes null check, and sets to null.
	xrt_tracked_psvr_destroy(&psvr->tracker);

	if (psvr->hmd_control != NULL) {
		// Turn off VR-mode and power down headset.
		if (control_vrmode_and_wait(psvr, false) < 0 ||
		    control_power_and_wait(psvr, false) < 0) {
			PSVR_ERROR(psvr, "Failed to shut down the headset!");
		}

		hid_close(psvr->hmd_control);
		psvr->hmd_control = NULL;
	}

	if (psvr->hmd_handle != NULL) {
		hid_close(psvr->hmd_handle);
		psvr->hmd_handle = NULL;
	}
}


/*
 *
 * xrt_device functions.
 *
 */

static void
psvr_device_update_inputs(struct xrt_device *xdev,
                          struct time_state *timekeeping)
{
	struct psvr_device *psvr = psvr_device(xdev);

	read_handle_packets(psvr);
	update_leds_if_changed(psvr);
}

static void
psvr_device_get_tracked_pose(struct xrt_device *xdev,
                             enum xrt_input_name name,
                             struct time_state *timekeeping,
                             int64_t *out_timestamp,
                             struct xrt_space_relation *out_relation)
{
	struct psvr_device *psvr = psvr_device(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_RELATION) {
		PSVR_ERROR(psvr, "unknown input name");
		return;
	}

	// Read all packets.
	read_handle_packets(psvr);
	read_control_packets(psvr);

	// Clear out the relation.
	U_ZERO(out_relation);

	int64_t when = time_state_get_now(timekeeping);
	*out_timestamp = when;

	// We have no tracking, don't return a position.
	if (psvr->tracker == NULL) {
		out_relation->pose.orientation = psvr->fusion.rot;
		out_relation->relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	} else {
		psvr->tracker->get_tracked_pose(psvr->tracker, timekeeping,
		                                when, out_relation);
	}
}

static void
psvr_device_get_view_pose(struct xrt_device *xdev,
                          struct xrt_vec3 *eye_relation,
                          uint32_t view_index,
                          struct xrt_pose *out_pose)
{
	struct xrt_pose pose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	bool adjust = view_index == 0;

	pose.position.x = eye_relation->x / 2.0f;
	pose.position.y = eye_relation->y / 2.0f;
	pose.position.z = eye_relation->z / 2.0f;

	// Adjust for left/right while also making sure there aren't any -0.f.
	if (pose.position.x > 0.0f && adjust) {
		pose.position.x = -pose.position.x;
	}
	if (pose.position.y > 0.0f && adjust) {
		pose.position.y = -pose.position.y;
	}
	if (pose.position.z > 0.0f && adjust) {
		pose.position.z = -pose.position.z;
	}

	*out_pose = pose;
}

static void
psvr_device_destroy(struct xrt_device *xdev)
{
	struct psvr_device *psvr = psvr_device(xdev);
	teardown(psvr);
	free(psvr);
}


/*
 *
 * Exported functions.
 *
 */

struct xrt_device *
psvr_device_create(struct hid_device_info *hmd_handle_info,
                   struct hid_device_info *hmd_control_info,
                   struct xrt_prober *xp,
                   bool print_spew,
                   bool print_debug)
{
	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(
	    U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct psvr_device *psvr =
	    U_DEVICE_ALLOCATE(struct psvr_device, flags, 1, 0);
	int ret;

	psvr->print_spew = print_spew;
	psvr->print_debug = print_debug;
	psvr->base.update_inputs = psvr_device_update_inputs;
	psvr->base.get_tracked_pose = psvr_device_get_tracked_pose;
	psvr->base.get_view_pose = psvr_device_get_view_pose;
	psvr->base.destroy = psvr_device_destroy;
	psvr->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_RELATION;
	psvr->base.name = XRT_DEVICE_GENERIC_HMD;
	psvr->base.hmd->distortion.models = XRT_DISTORTION_MODEL_MESHUV;
	psvr->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_MESHUV;

	psvr->fusion.rot.w = 1.0f;

	snprintf(psvr->base.str, XRT_DEVICE_NAME_LEN, "PS VR Headset");

	ret = open_hid(psvr, hmd_handle_info, &psvr->hmd_handle);
	if (ret != 0) {
		goto cleanup;
	}

	ret = open_hid(psvr, hmd_control_info, &psvr->hmd_control);
	if (ret < 0) {
		goto cleanup;
	}

	if (control_power_and_wait(psvr, true) < 0 ||
	    control_vrmode_and_wait(psvr, true) < 0) {
		goto cleanup;
	}

	if (debug_get_bool_option_psvr_disco()) {
		ret = disco_leds(psvr);
	} else {
		ret = control_leds(psvr, PSVR_LED_ALL, PSVR_LED_POWER_MAX,
		                   (enum psvr_leds)0);
	}
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to control leds '%i'", ret);
		goto cleanup;
	}


	/*
	 * Device setup.
	 */

	struct u_device_simple_info info;
	info.display.w_pixels = 1920;
	info.display.h_pixels = 1080;
	info.display.w_meters = 0.126; // from calculated specs
	info.display.h_meters = 0.071;
	info.lens_horizontal_separation_meters = 0.0630999878f;
	info.lens_vertical_position_meters = 0.071 / 2.0f; // 94899882f;
	info.views[0].fov = 103.57f * (M_PI / 180.0f);
	info.views[1].fov = 103.57f * (M_PI / 180.0f);

	if (!u_device_setup_split_side_by_side(&psvr->base, &info)) {
		PSVR_ERROR(psvr, "Failed to setup basic device info");
		goto cleanup;
	}

	/*
	 * Setup variable.
	 */

	// clang-format off
	u_var_add_root(psvr, "PS VR Headset", true);
	u_var_add_gui_header(psvr, &psvr->gui.last_frame, "Last data");
	u_var_add_ro_vec3_i32(psvr, &psvr->last.samples[0].accel, "last.samples[0].accel");
	u_var_add_ro_vec3_i32(psvr, &psvr->last.samples[1].accel, "last.samples[1].accel");
	u_var_add_ro_vec3_i32(psvr, &psvr->last.samples[0].gyro, "last.samples[0].gyro");
	u_var_add_ro_vec3_i32(psvr, &psvr->last.samples[1].gyro, "last.samples[1].gyro");
	u_var_add_ro_vec3_f32(psvr, &psvr->read.accel, "read.accel");
	u_var_add_ro_vec3_f32(psvr, &psvr->read.gyro, "read.gyro");
	u_var_add_gui_header(psvr, &psvr->gui.control, "Control");
	u_var_add_u8(psvr, &psvr->wants.leds[0], "Led A");
	u_var_add_u8(psvr, &psvr->wants.leds[1], "Led B");
	u_var_add_u8(psvr, &psvr->wants.leds[2], "Led C");
	u_var_add_u8(psvr, &psvr->wants.leds[3], "Led D");
	u_var_add_u8(psvr, &psvr->wants.leds[4], "Led E");
	u_var_add_u8(psvr, &psvr->wants.leds[5], "Led F");
	u_var_add_u8(psvr, &psvr->wants.leds[6], "Led G");
	u_var_add_u8(psvr, &psvr->wants.leds[7], "Led H");
	u_var_add_u8(psvr, &psvr->wants.leds[8], "Led I");
	u_var_add_bool(psvr, &psvr->print_debug, "Debug");
	u_var_add_bool(psvr, &psvr->print_spew, "Spew");
	// clang-format on

	/*
	 * Finishing touches.
	 */

	if (psvr->print_debug) {
		u_device_dump_config(&psvr->base, __func__, "Sony PSVR");
	}

	// If there is a tracking factory use it.
	if (xp->tracking != NULL) {
		xp->tracking->create_tracked_psvr(xp->tracking, &psvr->base,
		                                  &psvr->tracker);
	}

	// Use the new origin if we got a tracking system.
	if (psvr->tracker != NULL) {
		psvr->base.tracking_origin = psvr->tracker->origin;
	}

	PSVR_DEBUG(psvr, "YES!");

	return &psvr->base;


cleanup:
	PSVR_DEBUG(psvr, "NO! :(");

	teardown(psvr);
	free(psvr);

	return NULL;
}
