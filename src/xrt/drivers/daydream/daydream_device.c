// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Daydream controller code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @ingroup drv_psmv
 */

#include "xrt/xrt_prober.h"
#include "xrt/xrt_tracking.h"



#include "math/m_api.h"
#include "tracking/t_imu.h"

#include "util/u_var.h"
#include "util/u_time.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_bitwise.h"

#include "daydream_device.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>


/*!
 * @ingroup drv_daydream
 * @{
 */


DEBUG_GET_ONCE_BOOL_OPTION(daydream_spew, "DAYDREAM_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(daydream_debug, "DAYDREAM_PRINT_DEBUG", false)

static void
daydream_device_destroy(struct xrt_device *xdev);
static void
daydream_device_update_inputs(struct xrt_device *xdev, struct time_state *timekeeping);
static void
daydream_device_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           struct time_state *timekeeping,
                           int64_t *out_timestamp,
                           struct xrt_space_relation *out_relation);

static inline struct daydream_device *
daydream_device(struct xrt_device *xdev);
static void *
daydream_run_thread(void *ptr);

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
//enum daydream_button_bit
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
 * A single Daydream Controller.
 *
 */

struct daydream_device *
daydream_device_create(
                 bool print_spew,
                 bool print_debug)
{
    enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(
        U_DEVICE_ALLOC_TRACKING_NONE);
    struct daydream_device *dd =
        U_DEVICE_ALLOCATE(struct daydream_device, flags, 1, 0);
    dd->print_spew = debug_get_bool_option_daydream_spew();
    dd->print_debug = debug_get_bool_option_daydream_debug();
    dd->base.destroy = daydream_device_destroy;
    dd->base.update_inputs = daydream_device_update_inputs;
    dd->base.get_tracked_pose = daydream_device_get_tracked_pose;
    dd->base.inputs[0].name = XRT_INPUT_DAYDREAM_POSE;
    dd->base.name = XRT_DEVICE_DAYDREAM;
    dd->fusion.rot.w = 1.0f;
    dd->fusion.fusion = imu_fusion_create();
    dd->fusion.variance.accel.x =1.0f;
    dd->fusion.variance.accel.y =1.0f;
    dd->fusion.variance.accel.z =1.0f;
    dd->fusion.variance.gyro.x =1.0f;
    dd->fusion.variance.gyro.y =1.0f;
    dd->fusion.variance.gyro.z =1.0f;

    dd->calibration.accel.factor.x = 200.0;
    dd->calibration.accel.factor.y = 200.0;
    dd->calibration.accel.factor.z = 200.0;

    dd->calibration.accel.bias.x = 0.0;
    dd->calibration.accel.bias.y = 0.0;
    dd->calibration.accel.bias.z = 0.0;

    dd->calibration.gyro.factor.x = 120000000000.0;
    dd->calibration.gyro.factor.y = 120000000000.0;
    dd->calibration.gyro.factor.z = 120000000000.0;

    dd->calibration.gyro.bias.x = 0.0;
    dd->calibration.gyro.bias.y = 0.0;
    dd->calibration.gyro.bias.z = 0.0;


    os_ble_notify_open("A8_1E_84_5C_6C_28","service002a/char002b",&dd->ble);


    printf("creating daydream device!\n");

    int ret = os_thread_helper_start(&dd->oth, daydream_run_thread, dd);
    if (ret != 0) {
        DAYDREAM_ERROR(dd, "Failed to start thread!");
        daydream_device_destroy(&dd->base);
        //return ret;
    }

    return dd;
}





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

inline struct daydream_device *
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
{
    DAYDREAM_DEBUG(daydream,"fusion sample mx %d my %d mz %d ax %d ay %d az %d gx %d gy %d gz %d\n",sample->mag.x,sample->mag.y,sample->mag.z,sample->accel.x,sample->accel.y,sample->accel.z,sample->gyro.x,sample->gyro.y,sample->gyro.z);


    daydream->read.accel.x = (sample->accel.x - daydream->calibration.accel.bias.x) /
                         daydream->calibration.accel.factor.x *
                         MATH_GRAVITY_M_S2;
    daydream->read.accel.y = (sample->accel.y - daydream->calibration.accel.bias.y) /
                         daydream->calibration.accel.factor.y *
                         MATH_GRAVITY_M_S2;
    daydream->read.accel.z = (sample->accel.z - daydream->calibration.accel.bias.z) /
                         daydream->calibration.accel.factor.z *
                         MATH_GRAVITY_M_S2;

    daydream->read.gyro.x = (sample->gyro.x - daydream->calibration.gyro.bias.x) /
                        daydream->calibration.gyro.factor.x;
    daydream->read.gyro.y = (sample->gyro.y - daydream->calibration.gyro.bias.y) /
                        daydream->calibration.gyro.factor.y;
    daydream->read.gyro.z = (sample->gyro.z  - daydream->calibration.gyro.bias.z) /
                        daydream->calibration.gyro.factor.z;
    DAYDREAM_DEBUG(daydream,"fusion calibrated sample ax %f ay %f az %f gx %f gy %f gz %f\n",daydream->read.accel.x,daydream->read.accel.y,daydream->read.accel.z,daydream->read.gyro.x,daydream->read.gyro.y,daydream->read.gyro.z);

    math_quat_integrate_velocity(
        &daydream->fusion.rot, &daydream->read.gyro, delta_ns, &daydream->fusion.rot);
    return;

    imu_fusion_incorporate_gyros_and_accelerometer(
            daydream->fusion.fusion, timestamp_ns, &daydream->read.gyro,
            &daydream->fusion.variance.gyro, &daydream->read.accel,
            &daydream->fusion.variance.accel, NULL);
        struct xrt_vec3 angvel_dummy;
        imu_fusion_get_prediction(daydream->fusion.fusion, timestamp_ns,
                                  &daydream->fusion.rot, &angvel_dummy);
        imu_fusion_get_prediction_rotation_vec(
            daydream->fusion.fusion, timestamp_ns, &daydream->fusion.rotvec);
}

/*!
 * Reads one packet from the device,handles locking and checking if
 * the thread has been told to shut down.
 */
static bool
daydream_read_one_packet(struct daydream_device *daydream, uint8_t *buffer, size_t size)
{
    os_thread_helper_lock(&daydream->oth);

    while (os_thread_helper_is_running_locked(&daydream->oth)) {
        int retries = 5;
        int ret = -1;
        os_thread_helper_unlock(&daydream->oth);

        while (retries > 0) {
            ret = daydream->ble->read(daydream->ble, buffer, size);
            if (ret == size){
                break;
            }
            usleep(60000);
            retries--;
        }
        if (ret == 0) {
			fprintf(stderr, "%s\n", __func__);
			// Must lock thread before check in while.
            os_thread_helper_lock(&daydream->oth);
			continue;
		}
		if (ret < 0) {
            DAYDREAM_ERROR(daydream, "Failed to read device '%i'!", ret);
            return false;
		}
        return true;
	}

	return false;
}

void *
daydream_run_thread(void *ptr)
{
    struct daydream_device *daydream = (struct daydream_device *)ptr;
	//! @todo this should be injected at construction time
	struct time_state *time = time_state_create();

    uint8_t buffer[20];
    struct daydream_parsed_input input;// = {0};

    // wait for a package to sync up, it's discarded but that's okay.
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

 void
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

void
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

void
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
    unsigned char* b = (unsigned char*)data;
    DAYDREAM_DEBUG(daydream,"raw input: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12],b[13],b[14],b[15],b[16],b[17],b[18],b[19]);
    input->timestamp = get_bits(b,0,14);
    input->sample.mag.x = sign_extend_13(get_bits(b,14,13));
    input->sample.mag.y = sign_extend_13(get_bits(b,27,13));
    input->sample.mag.z = sign_extend_13(get_bits(b,40,13));
    input->sample.accel.x = sign_extend_13(get_bits(b,53,13));
    input->sample.accel.y = sign_extend_13(get_bits(b,66,13));
    input->sample.accel.z = sign_extend_13(get_bits(b,79,13));
    input->sample.gyro.x = sign_extend_13(get_bits(b,92,13));
    input->sample.gyro.y = sign_extend_13(get_bits(b,105,13));
    input->sample.gyro.z = sign_extend_13(get_bits(b,118,13));
    input->touchpad.x = get_bits(b,131,8);
    input->touchpad.y = get_bits(b,139,8);
    input->buttons.volup = get_bit(b,147);
    input->buttons.voldn = get_bit(b,148);
    input->buttons.app = get_bit(b,149);
    input->buttons.home = get_bit(b,150);
    input->buttons.touchpad = get_bit(b,151);

    return 1;
}


/*!
 * @}
 */
