// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Daydreamcontroller code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup drv_psmv
 */

#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"

#include "os/os_threading.h"
#include "os/os_ble.h"

#include "math/m_api.h"
#include "tracking/t_imu.h"

#include "util/u_var.h"
#include "util/u_time.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include "daydream_interface.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>


/*!
 * @ingroup drv_daydream
 * @{
 */


/*
 *
 * Defines & structs.
 *
 */

#define DAYDREAM_SPEW(p, ...)                                                  \
	do {                                                                   \
		if (p->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define DAYDREAM_DEBUG(p, ...)                                                 \
	do {                                                                   \
		if (p->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define DAYDREAM_ERROR(p, ...)                                                 \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)

DEBUG_GET_ONCE_BOOL_OPTION(daydream_spew, "DAYDREAM_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(daydream_debug, "DAYDREAM_PRINT_DEBUG", false)

/*!
 * Indices where each input is in the input list.
 */
enum daydream_input_index
{
	DAYDREAM_TOUCHPAD_CLICK,
	DAYDREAM_BAR_CLICK,
	DAYDREAM_CIRCLE_CLICK,
	DAYDREAM_VOLUP_CLICK,
	DAYDREAM_VOLDN_CLICK,
	DAYDREAM_TOUCHPAD_POSE,
};

/*!
 * Mask for the button in the button uint32_t.
 */
// enum daydream_button_bit
//{
// clang-format off
    //TODO
// clang-format on
//};


/*!
 * Input package for Daydream.
 */
struct daydream_input_packet
{
	uint8_t data[20];
};

/*!
 * A parsed sample of accel and gyro.
 */
struct daydream_parsed_sample
{
	struct xrt_vec3_s16 accel;
	struct xrt_vec3_s16 gyro;
	struct xrt_vec3_s16 mag;
};

/*!
 * A parsed input packet.
 */
struct daydream_parsed_input
{
	uint32_t buttons;
	uint16_t timestamp;
	uint16_t timestamp_last;
	struct xrt_vec2_s16 touchpad;
	struct daydream_parsed_sample sample;
};

/*!
 * A single Daydream Controller.
 *
 */
struct daydream_device
{
	struct xrt_device base;

	struct os_ble_device *ble;

	struct os_thread_helper oth;
	char mac[128];

	struct
	{
		//! Lock for last and fusion.
		struct os_mutex lock;

		//! Last sensor read.
		struct daydream_parsed_input last;

		struct
		{
			struct xrt_quat rot;
			struct xrt_vec3 rotvec;
			struct imu_fusion *fusion;
			struct
			{
				struct xrt_vec3 accel;
				struct xrt_vec3 gyro;
				struct xrt_vec3 mag;

			} variance;
		} fusion;
	};


	struct
	{
		//! Last adjusted accelerator value.
		struct xrt_vec3 accel;
		//! Last adjusted gyro value.
		struct xrt_vec3 gyro;
		struct xrt_vec3 mag;

	} read;

	bool print_spew;
	bool print_debug;

	struct
	{
		bool control;
		bool calibration;
		bool last_frame;
		bool fusion;
	} gui;
};


/*
 *
 * Pre-declare some functions.
 *
 */

static int
daydream_parse_input(struct daydream_device *daydream,
                     void *data,
                     struct daydream_parsed_input *input);

static int
daydream_get_calibration(struct daydream_device *daydream);

/*
 *
 * Smaller helper functions.
 *
 */

static inline struct daydream_device *
daydream_device(struct xrt_device *xdev)
{
	return (struct daydream_device *)xdev;
}


static void
daydream_update_input_click(struct daydream_device *daydream,
                            int index,
                            int64_t now,
                            uint32_t bit)
{
	daydream->base.inputs[index].timestamp = now;
	daydream->base.inputs[index].value.boolean =
	    (daydream->last.buttons & bit) != 0;
}



/*
 *
 * Internal functions.
 *
 */

static void
update_fusion(struct daydream_device *daydream,
              struct daydream_parsed_sample *sample,
              timepoint_ns timestamp_ns,
              time_duration_ns delta_ns)
{}

/*!
 * Reads one packet from the device, handles time out, locking and checking if
 * the thread has been told to shut down.
 */
static bool
daydream_read_one_packet(struct daydream_device *daydream,
                         uint8_t *buffer,
                         size_t size)
{
	os_thread_helper_lock(&daydream->oth);

	while (os_thread_helper_is_running_locked(&daydream->oth)) {

		os_thread_helper_unlock(&daydream->oth);

		int ret = os_ble_read(daydream->ble, buffer, size);

		if (ret == 0) {
			fprintf(stderr, "%s\n", __func__);
			// Must lock thread before check in while.
			os_thread_helper_lock(&daydream->oth);
			continue;
		}
		if (ret < 0) {
			DAYDREAM_ERROR(daydream, "Failed to read device '%i'!",
			               ret);
			return false;
		}

		return true;
	}

	return false;
}

static void *
daydream_run_thread(void *ptr)
{
	struct daydream_device *daydream = (struct daydream_device *)ptr;
	//! @todo this should be injected at construction time
	struct time_state *time = time_state_create();

	uint8_t buffer[20];
	struct daydream_parsed_input input = {0};

	while (os_ble_read(daydream->ble, buffer, 20) > 0) {
		// Empty queue first
	}

	// Now wait for a package to sync up, it's discarded but that's okay.
	if (!daydream_read_one_packet(daydream, buffer, 20)) {
		// Does null checking and sets to null.
		time_state_destroy(&time);
		return NULL;
	}

	timepoint_ns then_ns = time_state_get_now(time);

	while (daydream_read_one_packet(daydream, buffer, 20)) {

		timepoint_ns now_ns = time_state_get_now(time);

		int num = daydream_parse_input(daydream, buffer, &input);

		time_duration_ns delta_ns = now_ns - then_ns;
		then_ns = now_ns;

		// Lock last and the fusion.
		os_mutex_lock(&daydream->lock);


		// Process the parsed data.
		update_fusion(daydream, &input.sample, now_ns, delta_ns);

		// Now done.
		os_mutex_unlock(&daydream->lock);
	}

	// Does null checking and sets to null.
	time_state_destroy(&time);

	return NULL;
}



static void
daydream_get_fusion_pose(struct daydream_device *daydream,
                         enum xrt_input_name name,
                         timepoint_ns when,
                         struct xrt_space_relation *out_relation)
{
	out_relation->pose.orientation = daydream->fusion.rot;

	//! @todo assuming that orientation is actually currently tracked.
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
}


/*
 *
 * Device functions.
 *
 */

static void
daydream_device_destroy(struct xrt_device *xdev)
{
	struct daydream_device *daydream = daydream_device(xdev);

	// Destroy the thread object.
	os_thread_helper_destroy(&daydream->oth);

	// Now that the thread is not running we can destroy the lock.
	os_mutex_destroy(&daydream->lock);

	// Destroy the IMU fusion.
	imu_fusion_destroy(daydream->fusion.fusion);

	// Remove the variable tracking.
	u_var_remove_root(daydream);

	if (daydream->ble != NULL) {

		os_ble_destroy(daydream->ble);
		daydream->ble = NULL;
	}

	free(daydream);
}

static void
daydream_device_update_inputs(struct xrt_device *xdev,
                              struct time_state *timekeeping)
{
	struct daydream_device *daydream = daydream_device(xdev);

	int64_t now = time_state_get_now(timekeeping);

	// Lock the data.
	os_mutex_lock(&daydream->lock);

	// clang-format off
    /*
    daydream_update_input_click(daydream, DAYDREAM_INDEX_TOUCHPAD_CLICK, now, PSMV_BUTTON_BIT_PS);
    daydream_update_input_click(daydream, PSMV_INDEX_MOVE_CLICK, now, PSMV_BUTTON_BIT_MOVE_ANY);
    daydream_update_input_click(daydream, PSMV_INDEX_START_CLICK, now, PSMV_BUTTON_BIT_START);
    daydream_update_input_click(daydream, PSMV_INDEX_SELECT_CLICK, now, PSMV_BUTTON_BIT_SELECT);
    daydream_update_input_click(daydream, PSMV_INDEX_SQUARE_CLICK, now, PSMV_BUTTON_BIT_SQUARE);
    daydream_update_input_click(daydream, PSMV_INDEX_CROSS_CLICK, now, PSMV_BUTTON_BIT_CROSS);
    daydream_update_input_click(daydream, PSMV_INDEX_CIRCLE_CLICK, now, PSMV_BUTTON_BIT_CIRCLE);
    daydream_update_input_click(daydream, PSMV_INDEX_TRIANGLE_CLICK, now, PSMV_BUTTON_BIT_TRIANGLE);
    \
	// Done now.
*/
os_mutex_unlock(&daydream->lock);
}

static void
daydream_device_get_tracked_pose(struct xrt_device *xdev,
                             enum xrt_input_name name,
                             struct time_state *timekeeping,
                             int64_t *out_timestamp,
                             struct xrt_space_relation *out_relation)
{
    struct daydream_device *daydream = daydream_device(xdev);

	timepoint_ns now = time_state_get_now(timekeeping);

        daydream_get_fusion_pose(daydream, name, now, out_relation);
}


/*
 *
 * Prober functions.
 *
 */

#define SET_INPUT(NAME)                                                        \
    (daydream->base.inputs[DAYDREAM_INDEX_##NAME].name = XRT_INPUT_DAYDREAM_##NAME)

int
daydream_found(struct xrt_prober *xp,
           struct xrt_prober_device **devices,
           size_t num_devices,
           size_t index,
           struct xrt_device **out_xdevs)
{
    struct os_ble_device *ble = NULL;
	int ret;


    ret = xrt_prober_open_ble_interface(xp, devices[index], 0, &ble);
	if (ret != 0) {
		return -1;
	}

	enum u_device_alloc_flags flags = U_DEVICE_ALLOC_TRACKING_NONE;
    struct daydream_device *daydream =
        U_DEVICE_ALLOCATE(struct daydream_device, flags, 12, 1);
    daydream->print_spew = debug_get_bool_option_daydream_spew();
    daydream->print_debug = debug_get_bool_option_daydream_debug();
    daydream->base.destroy = daydream_device_destroy;
    daydream->base.update_inputs = daydream_device_update_inputs;
    daydream->base.get_tracked_pose = daydream_device_get_tracked_pose;
    daydream->base.set_output = NULL;
    daydream->base.name = XRT_DEVICE_DAYDREAM;
    daydream->fusion.rot.w = 1.0f;
    daydream->fusion.fusion = imu_fusion_create();
    daydream->ble = ble;
    snprintf(daydream->base.str, XRT_DEVICE_NAME_LEN, "%s",
             "Daydream Controller");


	// Mutex before thread.
    ret = os_mutex_init(&daydream->lock);
	if (ret != 0) {
        DAYDREAM_ERROR(daydream, "Failed to init mutex!");
        daydream_device_destroy(&daydream->base);
		return ret;
	}

	// Thread and other state.
    ret = os_thread_helper_init(&daydream->oth);
	if (ret != 0) {
        DAYDREAM_ERROR(daydream, "Failed to init threading!");
        daydream_device_destroy(&daydream->base);
		return ret;
	}
	// Get calibration data.
    ret = daydream_get_calibration(daydream);
	if (ret != 0) {
        DAYDREAM_ERROR(daydream, "Failed to get calibration data!");
        daydream_device_destroy(&daydream->base);
		return ret;
	}

    ret = os_thread_helper_start(&daydream->oth, daydream_run_thread, daydream);
	if (ret != 0) {
        DAYDREAM_ERROR(daydream, "Failed to start thread!");
        daydream_device_destroy(&daydream->base);
		return ret;
	}

	// Start the variable tracking now that everything is in place.
	// clang-format off
    u_var_add_root(daydream, "Daydream Controller", true);
    u_var_add_gui_header(daydream, &daydream->gui.calibration, "Calibration");
	// clang-format on

	// And finally done
	*out_xdevs = &daydream->base;

	return 1;
}



/*
 *
 * Small dispatch functions.
 *
 */

static int
daydream_get_calibration(struct daydream_device *daydream)
{


	return 0;
}

static int
daydream_parse_input(struct daydream_device *daydream,
                     void *data,
                     struct daydream_parsed_input *input)
{
	U_ZERO(input);

	return 2;
}


/*!
 * @}
 */
