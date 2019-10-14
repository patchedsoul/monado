// Copyright 2019, Collabora, Ltd.
// Copyright 2018, Joey Ferwerda
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for an LG R100 device.
 *
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Joey Ferwerda
 * @ingroup drv_lgr100
 */


#include "lgr100_device.h"
#include "lgr100_interface.h"

#include "math/m_api.h"

#include "os/os_hid.h"
#include "os/os_threading.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_device.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"

#include "xrt/xrt_device.h"
#include "xrt/xrt_prober.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



DEBUG_GET_ONCE_BOOL_OPTION(lgr100_spew, "LGR100_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(lgr100_debug, "LGR100_PRINT_DEBUG", false)

#define LGR100_SPEW(c, ...)                                                    \
	do {                                                                   \
		if (c->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)
#define LGR100_DEBUG(c, ...)                                                   \
	do {                                                                   \
		if (c->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define LGR100_ERROR(c, ...)                                                   \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)

enum lgr100_usb_cmd
{
	LGR100_IRQ_NULL = 0,
	LGR100_IRQ_UNKNOWN1 = 1,
	LGR100_IRQ_BUTTONS = 2,
	LGR100_IRQ_DEBUG1 = 3,
	LGR100_IRQ_DEBUG2 = 4,
	LGR100_IRQ_SENSORS = 5,
	LGR100_IRQ_DEBUG_SEQ1 = 32,
	LGR100_IRQ_DEBUG_SEQ2 = 33,
	LGR100_IRQ_UNKNOWN3 = 101,
	LGR100_IRQ_UNKNOWN2 = 255
};

/* Start command for the firmware */
static const unsigned char start_device[14] = {
    0x03, 0x0C, 'V', 'R', ' ', 'A', 'p', 'p', ' ', 'S', 't', 'a', 'r', 't'};

static const unsigned char start_accel[10] = {0x03, 0x08, 'A', 'c', 'c',
                                              'e',  'l',  ' ', 'O', 'n'};
static const unsigned char start_gyro[9] = {0x03, 0x07, 'G', 'y', 'r',
                                            'o',  ' ',  'O', 'n'};

/* Stay Awake (ignore proximity sensor and keep tracking) */
static const unsigned char keep_alive[15] = {0x03, 0x0D, 'S', 'l', 'e',
                                             'e',  'p',  ' ', 'D', 'i',
                                             's',  'a',  'b', 'l', 'e'};


/*!
 * Input package.
 */
struct lgr100_input_buttons
{
	uint8_t id;
	uint8_t value;
};
struct lgr100_input_sensors
{
	uint8_t id;
	float accel[3];
	float gyro[3];
	uint32_t tick;
};

struct lgr100_input
{
	union {
		uint8_t buffer[256];
		struct lgr100_input_buttons buttons;
		struct lgr100_input_sensors sensors;
	};
};

struct lgr100_parsed_input
{
	struct xrt_vec3 gyro;
	struct xrt_vec3 accel;
	uint16_t buttons;
	uint32_t tick;
};

struct lgr100_device
{
	struct xrt_device base;
	struct os_hid_device *dev;
	struct os_thread_helper oth;

	bool print_spew;
	bool print_debug;


	struct
	{
		bool last_frame;
	} gui;
	struct
	{
		struct xrt_vec3 gyro;
		struct xrt_vec3 accel;
		uint16_t buttons;
	} read;

	struct
	{
		//! Lock for last and fusion.
		struct os_mutex lock;

		//! Last sensor read.
		struct lgr100_parsed_input last;

		struct
		{
			struct xrt_quat rot;
		} fusion;
	};
};

/*
 *
 * Helpers and internal functions.
 *
 */

static inline struct lgr100_device *
lgr100_device(struct xrt_device *xdev)
{
	return (struct lgr100_device *)xdev;
}

/*!
 * Reads one packet from the device, handles time out, locking and checking if
 * the thread has been told to shut down.
 */
static bool
lgr100_read_one_packet(struct lgr100_device *lgd, uint8_t *buffer, size_t size)
{
	os_thread_helper_lock(&lgd->oth);

	while (os_thread_helper_is_running_locked(&lgd->oth)) {

		os_thread_helper_unlock(&lgd->oth);

		int ret = os_hid_read(lgd->dev, buffer, size, 1000);

		if (ret == 0) {
			fprintf(stderr, "%s\n", __func__);
			// Must lock thread before check in while.
			os_thread_helper_lock(&lgd->oth);
			continue;
		} else if (ret < 0) {
			LGR100_ERROR(lgd, "Failed to read device '%i'!", ret);
			return false;
		}

		return true;
	}

	return false;
}


static const float TICK_LEN = (1.0f / 200000.0f); // 200 Hz ticks
static enum lgr100_usb_cmd
lgr100_parse_input(struct lgr100_device *lgd,
                   struct lgr100_input *data,
                   struct lgr100_parsed_input *input)
{
	switch (data->buffer[0]) {
	case LGR100_IRQ_NULL: return LGR100_IRQ_NULL;
	case LGR100_IRQ_BUTTONS:
		input->buttons = 0;
		//! @todo
		// button 'OK' is buffer[1] state 01 and 04
		// button '<-' is buffer[1] state 02 and 03
		// if (data->buttons.value == LGR100_BUTTON_OK_ON)
		// 	input->buttons = 0x1 | 0x2;
		// else if (buffer[1] == LGR100_BUTTON_BACK_ON)
		// 	priv->controller_values[1] = 1;
		// else if (buffer[1] == LGR100_BUTTON_BACK_OFF)
		// 	priv->controller_values[1] = 0;
		// else if (buffer[1] == LGR100_BUTTON_OK_OFF)
		// 	priv->controller_values[0] = 0;
		return LGR100_IRQ_BUTTONS;
	case LGR100_IRQ_SENSORS:
		input->tick = data->sensors.tick;
		input->accel.x = data->sensors.accel[0];
		input->accel.y = data->sensors.accel[1];
		input->accel.z = data->sensors.accel[2];
		input->gyro.x = data->sensors.gyro[0] * 4.f;
		input->gyro.y = data->sensors.gyro[1] * 4.f;
		input->gyro.z = data->sensors.gyro[2] * 4.f;
		return LGR100_IRQ_SENSORS;
	case LGR100_IRQ_DEBUG1:
	case LGR100_IRQ_DEBUG2:
	case LGR100_IRQ_DEBUG_SEQ1:
	case LGR100_IRQ_DEBUG_SEQ2: LGR100_DEBUG(lgd, "%s\n", data->buffer + 1);
	}
	return LGR100_IRQ_NULL;
}

static const float G = 9.8f;
static const float ACCEL_RESTORE_RATE = 0.9f;

static void
update_fusion(struct lgr100_device *lgd,
              struct lgr100_parsed_input *sample,
              float dt)
{
	struct xrt_vec3 mag = {0.0f, 0.0f, 0.0f};
	(void)mag;

	lgd->read.accel = sample->accel;
	lgd->read.gyro = sample->gyro;
	struct xrt_quat rot = lgd->fusion.rot;
	struct xrt_quat inverse = {-rot.x, -rot.y, -rot.z, rot.w};
	struct xrt_vec3 xformed_vel;
	math_quat_rotate_vec3(&inverse, &(sample->gyro), &xformed_vel);

	math_quat_integrate_velocity(&rot, &lgd->read.gyro, dt,
	                             &lgd->fusion.rot);

	float diff_of_squares =
	    fabsf(G * G - math_vec3_squared_norm(&(sample->accel)));

	// If gravity is exact, this is 1. it gets smaller the further away from
	// gravity magnitude.
	float goodness = 1.f - diff_of_squares;
	if (goodness > 0.f) {
		float scale = ACCEL_RESTORE_RATE * goodness;
		struct xrt_vec3 normalized = sample->accel;
		math_vec3_normalize(&normalized);
		struct xrt_vec3 transformed_gravity;
		math_quat_rotate_vec3(&inverse, &normalized,
		                      &transformed_gravity);
		struct xrt_vec3 zero = {0.f, 0.f, 0.f};


		struct xrt_quat grav_adjust;
		math_quat_from_two_vecs(&transformed_gravity, &zero,
		                        &grav_adjust);
		struct xrt_quat identity = {0.f, 0.f, 0.f, 1.f};


		struct xrt_quat scaled_rotate;
		math_quat_slerp(&identity, &grav_adjust, goodness * dt, &scaled_rotate);
		math_quat_rotate(&scaled_rotate, &lgd->fusion.rot, &lgd->fusion.rot);
	}
}

static void *
lgr100_run_thread(void *ptr)
{
	struct lgr100_device *lgd = (struct lgr100_device *)ptr;
	struct time_state *time = time_state_create();

	struct lgr100_input data;

	while (os_hid_read(lgd->dev, data.buffer, sizeof(data), 0) > 0) {
		// Empty queue first
	}

	// Now wait for a package to sync up, it's discarded but that's okay.
	if (!lgr100_read_one_packet(lgd, data.buffer, sizeof(data))) {
		time_state_destroy(time);
		return NULL;
	}

	timepoint_ns then_ns = time_state_get_now(time);

	while (lgr100_read_one_packet(lgd, data.buffer, sizeof(data))) {

		timepoint_ns now_ns = time_state_get_now(time);
		struct lgr100_parsed_input input = {0};
		enum lgr100_usb_cmd cmd =
		    lgr100_parse_input(lgd, &data, &input);

		time_duration_ns delta_ns = now_ns - then_ns;
		then_ns = now_ns;

		// Lock last and the fusion.
		os_mutex_lock(&lgd->lock);

		// Copy to device.
		lgd->last = input;

		// Process the parsed data.
		if (cmd == LGR100_IRQ_NULL) {
			// not handled
		} else if (cmd == LGR100_IRQ_BUTTONS) {
			// ?
			lgd->read.buttons = input.buttons;
		} else if (cmd == LGR100_IRQ_SENSORS) {
			update_fusion(lgd, &input, delta_ns);

		} else {
			assert(false);
		}

		// Now done.
		os_mutex_unlock(&lgd->lock);
	}

	time_state_destroy(time);

	return NULL;
}
static void
teardown(struct lgr100_device *lgd)
{
	// Stop the variable tracking.
	u_var_remove_root(lgd);

	if (lgd->dev != NULL) {
		os_hid_destroy(lgd->dev);
		lgd->dev = NULL;
	}
}

/*
 *
 * xrt_device functions.
 *
 */

static void
lgr100_device_update_inputs(struct xrt_device *xdev,
                            struct time_state *timekeeping)
{
	struct lgr100_device *lgd = lgr100_device(xdev);
}

static void
lgr100_device_get_tracked_pose(struct xrt_device *xdev,
                               enum xrt_input_name name,
                               struct time_state *timekeeping,
                               int64_t *out_timestamp,
                               struct xrt_space_relation *out_relation)
{
	struct lgr100_device *lgd = lgr100_device(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_RELATION) {
		LGR100_ERROR(lgd, "unknown input name");
		return;
	}

	// // Read all packets.
	// read_dev_packets(lgd);
	// read_control_packets(lgd);

	// Clear out the relation.
	U_ZERO(out_relation);

	int64_t when = time_state_get_now(timekeeping);
	*out_timestamp = when;

	out_relation->pose.orientation = lgd->fusion.rot;
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
}

static void
lgr100_device_get_view_pose(struct xrt_device *xdev,
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
lgr100_device_destroy(struct xrt_device *xdev)
{
	struct lgr100_device *lgd = lgr100_device(xdev);

	teardown(lgd);
	free(lgd);
}



int
lgr100_found(struct xrt_prober *xp,
             struct xrt_prober_device **devices,
             size_t index,
             struct xrt_device **out_xdev)
{
	struct xrt_prober_device *dev = devices[index];

	bool print_spew = debug_get_bool_option_lgr100_spew();
	bool print_debug = debug_get_bool_option_lgr100_debug();

	struct os_hid_device *hid = NULL;
	int result = xrt_prober_open_hid_interface(xp, dev, 0, &hid);
	if (result != 0) {
		return -1;
	}
	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(
	    U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct lgr100_device *lgd =
	    U_DEVICE_ALLOCATE(struct lgr100_device, flags, 1, 0);

	lgd->base.hmd->blend_mode = XRT_BLEND_MODE_OPAQUE;
	lgd->base.update_inputs = lgr100_device_update_inputs;
	lgd->base.get_tracked_pose = lgr100_device_get_tracked_pose;
	lgd->base.get_view_pose = lgr100_device_get_view_pose;
	lgd->base.destroy = lgr100_device_destroy;
	lgd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_RELATION;
	lgd->base.name = XRT_DEVICE_GENERIC_HMD;
	lgd->dev = hid;
	lgd->print_spew = print_spew;
	lgd->print_debug = print_debug;
	lgd->fusion.rot.w = 1.f;

	snprintf(lgd->base.str, XRT_DEVICE_NAME_LEN, "LG R-100");
#if 1
	static const double DEGREES_TO_RADIANS = M_PI / 180.0;
	{
		/* right eye */
		math_compute_fovs(0.110000f, 0.063500f / 2.f,
		                  80.0f * DEGREES_TO_RADIANS, 0.038000f,
		                  0.038000f / 2.f, 0,
		                  &lgd->base.hmd->views[1].fov);
	}
	{
		/* left eye - just mirroring right eye now */
		lgd->base.hmd->views[0].fov.angle_up =
		    lgd->base.hmd->views[1].fov.angle_up;
		lgd->base.hmd->views[0].fov.angle_down =
		    lgd->base.hmd->views[1].fov.angle_down;

		lgd->base.hmd->views[0].fov.angle_left =
		    -lgd->base.hmd->views[1].fov.angle_right;
		lgd->base.hmd->views[0].fov.angle_right =
		    -lgd->base.hmd->views[1].fov.angle_left;
	}
#endif
#if 0
	/*
	 * Device setup.
	 */

	struct u_device_simple_info info;
	info.display.w_pixels = 1440;
	info.display.h_pixels = 720;
	info.display.w_meters = 0.110000f; // from calculated specs
	info.display.h_meters = 0.038000f;
	info.lens_horizontal_separation_meters = 0.063500f;
	info.lens_vertical_position_meters = 0.020000f;
	info.views[0].fov = 80.f * M_PI / 180.0f;
	info.views[1].fov = 80.f * M_PI / 180.0f;

	if (!u_device_setup_split_side_by_side(&lgd->base, &info)) {
		LGR100_ERROR(lgd, "Failed to setup basic device info");

		lgr100_device_destroy(&lgd->base);
		return -1;
	}
#endif
	// Mutex before thread.
	int ret = os_mutex_init(&lgd->lock);
	if (ret != 0) {
		LGR100_ERROR(lgd, "Failed to init mutex!");
		lgr100_device_destroy(&lgd->base);
		return ret;
	}

	// Thread and other state.
	ret = os_thread_helper_init(&lgd->oth);
	if (ret != 0) {
		LGR100_ERROR(lgd, "Failed to init threading!");
		lgr100_device_destroy(&lgd->base);
		return ret;
	}

	os_hid_write(lgd->dev, start_device, sizeof(start_device));
	os_hid_write(lgd->dev, keep_alive, sizeof(start_device));

	ret = os_thread_helper_start(&lgd->oth, lgr100_run_thread, lgd);
	if (ret != 0) {
		LGR100_ERROR(lgd, "Failed to start thread!");
		lgr100_device_destroy(&lgd->base);
		return ret;
	}

	u_var_add_root(lgd, "LG R-100 Headset", true);
	u_var_add_gui_header(lgd, &lgd->gui.last_frame, "Last data");
	u_var_add_ro_vec3_f32(lgd, &lgd->read.accel, "read.accel");
	u_var_add_ro_vec3_f32(lgd, &lgd->read.gyro, "read.gyro");

	if (lgd->print_debug) {
		u_device_dump_config(&lgd->base, __func__, lgd->base.str);
	}

	*out_xdev = &lgd->base;
	return 1;
}
