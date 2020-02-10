// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: Apache-2.0
/*!
 * @file
 * @brief  Adapter to Libsurvive.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_device.h"

#include "xrt/xrt_prober.h"
#include "survive_interface.h"

#include "survive_api.h"

#define SURVIVE_SPEW(p, ...)                                                   \
	do {                                                                   \
		if (p->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define SURVIVE_DEBUG(p, ...)                                                  \
	do {                                                                   \
		if (p->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define SURVIVE_ERROR(p, ...)                                                  \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)

struct survive_system;

enum index_input_index
{
	INDEX_SYSTEM_CLICK    ,
	INDEX_SYSTEM_TOUCH    ,
	INDEX_A_CLICK         ,
	INDEX_A_TOUCH         ,
	INDEX_B_CLICK         ,
	INDEX_B_TOUCH         ,
	INDEX_SQUEEZE_VALUE   ,
	INDEX_SQUEEZE_FORCE   ,
	INDEX_TRIGGER_CLICK   ,
	INDEX_TRIGGER_VALUE   ,
	INDEX_TRIGGER_TOUCH   ,
	INDEX_THUMBSTICK_X    ,
	INDEX_THUMBSTICK_Y    ,
	INDEX_THUMBSTICK_XY   ,
	INDEX_THUMBSTICK_CLICK,
	INDEX_THUMBSTICK_TOUCH,
	INDEX_TRACKPAD_X      ,
	INDEX_TRACKPAD_Y      ,
	INDEX_TRACKPAD_XY     ,
	INDEX_TRACKPAD_FORCE  ,
	INDEX_TRACKPAD_TOUCH  ,
	INDEX_GRIP_POSE       ,
	INDEX_AIM_POSE        ,
	LAST_INPUT
};

// also used as index in sys->controllers[] array
typedef enum {
	SURVIVE_LEFT_CONTROLLER = 0,
	SURVIVE_RIGHT_CONTROLLER = 1,
	SURVIVE_HMD = 2,
} SurviveDeviceType;

static bool survive_already_initialized = false;

#define MAX_PENDING_EVENTS 30
struct survive_device
{
	struct xrt_device base;
	struct survive_system *sys;
	const SurviveSimpleObject *survive_obj;

	/* event needs to be processed if
	 * type != SurviveSimpleEventType_None */
	struct SurviveSimpleEvent pending_events[30];
	int num;
};

struct survive_system
{
	struct xrt_tracking_origin base;
	SurviveSimpleContext *ctx;
	struct survive_device *hmd;
	struct survive_device *controllers[2];
	bool print_spew;
	bool print_debug;
};

static void
survive_device_destroy(struct xrt_device *xdev)
{
	printf("destroying survive device\n");
	struct survive_device *survive = (struct survive_device *)xdev;

	if (survive == survive->sys->hmd)
		survive->sys->hmd = NULL;
	if (survive == survive->sys->controllers[SURVIVE_LEFT_CONTROLLER])
		survive->sys->controllers[SURVIVE_LEFT_CONTROLLER] = NULL;
	if (survive == survive->sys->controllers[SURVIVE_RIGHT_CONTROLLER])
		survive->sys->controllers[SURVIVE_RIGHT_CONTROLLER] = NULL;

	if (survive->sys->hmd == NULL &&
		survive->sys->controllers[SURVIVE_LEFT_CONTROLLER] == NULL &&
		survive->sys->controllers[SURVIVE_RIGHT_CONTROLLER] == NULL) {
			printf("Tearing down libsurvive context\n");
			survive_simple_close(survive->sys->ctx);

			free (survive->sys);
		}

	free(survive);
}

static void
_get_survive_pose(const SurviveSimpleObject *survive_object,
                  SurviveSimpleContext *ctx,
                  struct time_state *timekeeping,
                  int64_t *out_timestamp,
                  struct xrt_space_relation *out_relation)
{
	int64_t now = time_state_get_now(timekeeping);
	//! @todo adjust for latency here
	*out_timestamp = now;

	out_relation->relation_flags = 0;

	for (const SurviveSimpleObject *it =
		survive_simple_get_first_object(ctx);
	it != 0; it = survive_simple_get_next_object(ctx, it)) {
		//const char *codename = survive_simple_object_name(it);

		if (survive_simple_object_get_type(it) != SurviveSimpleObject_OBJECT && 
			survive_simple_object_get_type(it) != SurviveSimpleObject_HMD) {
			continue;
		}

		if (it != survive_object)
			continue;

		SurvivePose pose;

		uint32_t timecode =
		survive_simple_object_get_latest_pose(it, &pose);
		(void)timecode;

		struct xrt_quat out_rot = {
			.x = pose.Rot[1],
			.y = pose.Rot[2],
			.z = pose.Rot[3],
			.w = pose.Rot[0]
		};
		//printf ("quat %f %f %f %f\n", out_rot.x, out_rot.y, out_rot.z, out_rot.w);

		/* libsurvive looks down when it should be looking forward, so
		 * rotate the quat.
		 * because the HMD quat is the opposite of the in world
		 * rotation, we rotate down. */

		struct xrt_quat down_rot;
		down_rot.x = sqrtf(2) / 2.;
		down_rot.y = 0;
		down_rot.z = 0;
		down_rot.w = -sqrtf(2) / 2.;

		math_quat_rotate(&down_rot, &out_rot, &out_rot);


		math_quat_normalize(&out_rot);

		out_relation->pose.orientation = out_rot;

		/* because the quat is rotated, y and z axes are switched. */
		out_relation->pose.position.x = pose.Pos[0];
		out_relation->pose.position.y = pose.Pos[2];
		out_relation->pose.position.z = -pose.Pos[1];


		out_relation->relation_flags =
		(enum xrt_space_relation_flags)(
			XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
			XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) |
			XRT_SPACE_RELATION_POSITION_VALID_BIT |
			XRT_SPACE_RELATION_POSITION_TRACKED_BIT;

	}
}

static bool
_try_update_codenames(struct survive_system *sys)
{
	// TODO: better method

	if (sys->hmd->survive_obj && sys->controllers[0]->survive_obj && sys->controllers[1]->survive_obj)
		return true;

	SurviveSimpleContext *ctx = sys->ctx;

	for (const SurviveSimpleObject *it =
		survive_simple_get_first_object(ctx);
	it != 0; it = survive_simple_get_next_object(ctx, it)) {
		const char *codename = survive_simple_object_name(it);

		enum SurviveSimpleObject_type type = survive_simple_object_get_type(it);
		if (type == SurviveSimpleObject_HMD && sys->hmd->survive_obj == NULL) {
			printf("Found HMD: %s\n", codename);
			sys->hmd->survive_obj = it;
		}
		if (type == SurviveSimpleObject_OBJECT) {
			for (int i = 0; i < 2 /* TODO */; i++) {
				if (sys->controllers[i]->survive_obj == it) {
					break;
				}

				if (sys->controllers[i]->survive_obj == NULL) {
					printf("Found Controller %d: %s\n", i, codename);
					sys->controllers[i]->survive_obj = it;
					break;
				}
			}
		}
	}

	return true;
}

static void
survive_device_get_tracked_pose(struct xrt_device *xdev,
                                enum xrt_input_name name,
                                struct time_state *timekeeping,
                                int64_t *out_timestamp,
                                struct xrt_space_relation *out_relation)
{
	struct survive_device *survive = (struct survive_device *)xdev;
	if ((survive == survive->sys->hmd && name != XRT_INPUT_GENERIC_HEAD_POSE) ||
		((survive == survive->sys->controllers[0] ||
		  survive == survive->sys->controllers[1]) &&
		(name != XRT_INPUT_INDEX_AIM_POSE &&
		 name != XRT_INPUT_INDEX_GRIP_POSE))) {

		SURVIVE_ERROR(survive, "unknown input name");
		return;
	}

	_try_update_codenames(survive->sys);
	if (!survive->survive_obj) {
		//printf("Obj not set for %p\n", (void*)survive);
		return;
	}


	_get_survive_pose(survive->survive_obj, survive->sys->ctx, timekeeping, out_timestamp, out_relation);

	/*
	SURVIVE_SPEW(
		survive,
	      "GET_POSITION (%f %f %f) GET_ORIENTATION (%f, %f, %f, %f)",
		     out_relation->pose.position.x,
	      out_relation->pose.position.y,
	      out_relation->pose.position.z, out_rot.x, out_rot.y,
	      out_rot.z, out_rot.w);
	      */
	//printf("Get pose %f %f %f\n", out_relation->pose.position.x, out_relation->pose.position.y, out_relation->pose.position.z);
}

static void
survive_device_get_view_pose(struct xrt_device *xdev,
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

struct display_info
{
	float w_meters;
	float h_meters;
	int w_pixels;
	int h_pixels;
};

struct device_info
{
	struct display_info display;

	float lens_horizontal_separation;
	float lens_vertical_position;

	float pano_distortion_k[4];
	float pano_aberration_k[4];
	float pano_warp_scale;

	struct
	{
		float fov;

		struct display_info display;

		float lens_center_x_meters;
		float lens_center_y_meters;
	} views[2];
};

static struct device_info
get_info()
{
	struct device_info info = {0};

	//! @todo: hardcoded values for Vive 1 from openhmd vive driver
	info.display.w_meters = 0.122822f;
	info.display.h_meters = 0.068234f;
	info.lens_horizontal_separation = 0.056;
	info.lens_vertical_position = 0.032;
	info.views[0].fov =
	    2 * atan2f(0.122822f / 2. - 0.056 / 2., 0.023226876441867737);
	info.views[1].fov = info.views[0].fov;
	info.display.w_pixels = 2160;
	info.display.h_pixels = 1200;
	info.pano_distortion_k[0] = 1.318397;
	info.pano_distortion_k[1] = -1.490242;
	info.pano_distortion_k[2] = 0.663824;
	info.pano_distortion_k[3] = 0.508021;
	info.pano_aberration_k[0] = 1.00010147892f;
	info.pano_aberration_k[1] = 1.000f;
	info.pano_aberration_k[2] = 1.00019614479f;

	/*
	 * Assumptions made here:
	 *
	 * - There is a single, continuous, flat display serving both eyes, with
	 *   no dead space/gap between eyes.
	 * - This single panel is (effectively) perpendicular to the forward
	 *   (-Z) direction, with edges aligned with the X and Y axes.
	 * - Lens position is symmetrical about the center ("bridge of  nose").
	 * - Pixels are square and uniform across the entirety of the panel.
	 *
	 * If any of these are not true, then either the rendering will
	 * be inaccurate, or the properties will have to be "fudged" to
	 * make the math work.
	 */

	info.views[0].display.w_meters = info.display.w_meters / 2.0;
	info.views[0].display.h_meters = info.display.h_meters;
	info.views[1].display.w_meters = info.display.w_meters / 2.0;
	info.views[1].display.h_meters = info.display.h_meters;

	info.views[0].display.w_pixels = info.display.w_pixels / 2;
	info.views[0].display.h_pixels = info.display.h_pixels;
	info.views[1].display.w_pixels = info.display.w_pixels / 2;
	info.views[1].display.h_pixels = info.display.h_pixels;

	/*
	 * Assuming the lenses are centered vertically on the
	 * display. It's not universal, but 0.5 COP on Y is more
	 * common than on X, and it looked like many of the
	 * driver lens_vpos values were copy/pasted or marked
	 * with FIXME. Safer to fix it to 0.5 than risk an
	 * extreme geometry mismatch.
	 */

	const double cop_y = 0.5;
	const double h_1 = cop_y * info.display.h_meters;

	//! @todo This are probably all wrong!
	info.views[0].lens_center_x_meters =
	    info.views[0].display.w_meters -
	    info.lens_horizontal_separation / 2.0;
	info.views[0].lens_center_y_meters = h_1;

	info.views[1].lens_center_x_meters =
	    info.lens_horizontal_separation / 2.0;
	info.views[1].lens_center_y_meters = h_1;

	//! @todo This is most definitely wrong!
	//!       3Glasses likes the oposite better.
	info.pano_warp_scale = (info.views[0].lens_center_x_meters >
	                        info.views[0].lens_center_x_meters)
	                           ? info.views[0].lens_center_x_meters
	                           : info.views[0].lens_center_x_meters;
	return info;
}

static void
survive_hmd_update_inputs(struct xrt_device *xdev, struct time_state *timekeeping)
{
}

static void
_queue_event (struct survive_system *sys, SurviveSimpleObject *obj, SurviveSimpleEvent *event)
{
	struct survive_device *dev[] = {
		sys->hmd,
		sys->controllers[SURVIVE_LEFT_CONTROLLER],
		sys->controllers[SURVIVE_RIGHT_CONTROLLER],
	};
	for (int i = 0; i < 3; i++) {
		if (dev[i] && dev[i]->survive_obj == obj) {
			int queue_index = 0;
			for (queue_index = 0; queue_index < MAX_PENDING_EVENTS; queue_index++) {
				if (dev[i]->pending_events[queue_index].event_type == SurviveSimpleEventType_None) {
					break;
				}
			}
			if (queue_index == MAX_PENDING_EVENTS) {
				printf("Pending event queue full for device %d\n", i);
				return;
			}
			//printf("Queue event for device %d at index %d\n", i, queue_index);
			memcpy(&dev[i]->pending_events[queue_index], event, sizeof(SurviveSimpleEvent));
		}
	}

}

static void
_process_event (struct survive_device *survive,
		struct SurviveSimpleEvent *event,
		const SurviveSimpleObject *current,
		int64_t now)
{
	// ??
	const int survive_0_btn = 0;

	const int survive_trackpad_touch_btn = 1;

	const int survive_trigger_axis_id = 1;

	/* xy either thumbstick or trackpad */
	const int survive_xy_axis_id_x = 2;
	const int survive_xy_axis_id_y = 3;

	const int survive_a_btn = 4;
	const int survive_b_btn = 5;
	const int survive_trigger_click_btn = 24;

	switch (event->event_type) {
		case SurviveSimpleEventType_ButtonEvent: {
			const struct SurviveSimpleButtonEvent *e = survive_simple_get_button_event(event);

			if (e->object != current) {
				if (e->event_type == SurviveSimpleEventType_None) {
					// no need to queue empty event
					return;
				}
				_queue_event(survive->sys, e->object, event);
				return;
			}

			/*
			printf("Btn id %d type %d, axes %d  ", e->button_id, e->event_type, e->axis_count);
			for (int i = 0; i < e->axis_count; i++) {
				printf("axis id: %d val %hu    ", e->axis_ids[i], e->axis_val[i]);
			}
			printf("\n");
			*/


			if (e->button_id == survive_0_btn) {
				for (int i = 0; i < e->axis_count; i++) {
					if (e->axis_ids[i] == survive_trigger_axis_id) {
						uint16_t raw = e->axis_val[0];
						float scaled = (float) raw / 32768.;

						survive->base.inputs[INDEX_TRIGGER_VALUE].value.vec1.x = scaled;
						survive->base.inputs[INDEX_TRIGGER_VALUE].timestamp = now;
						//printf("Trigger value %f %lu\n", survive->base.inputs[INDEX_TRIGGER_VALUE].value.vec1.x, now);
					}
					if (e->axis_ids[i] == survive_xy_axis_id_x) {
						int input_x;
						int input_xy;
						if (survive->base.inputs[INDEX_TRACKPAD_TOUCH].value.boolean) {
							input_x = INDEX_TRACKPAD_X;
							input_xy = INDEX_TRACKPAD_XY;
						} else {
							input_x = INDEX_THUMBSTICK_X;
							input_xy = INDEX_THUMBSTICK_XY;
						}

						float x = (float)((int16_t) e->axis_val[i]) / 32768.;
						survive->base.inputs[input_x].value.vec1.x = x;
						survive->base.inputs[input_x].timestamp = now;

						survive->base.inputs[input_xy].value.vec2.x = x;
						survive->base.inputs[input_xy].timestamp = now;
						//printf("x: %f\n", x);
					}
					if (e->axis_ids[i] == survive_xy_axis_id_y) {
						int input_y;
						int input_xy;
						if (survive->base.inputs[INDEX_TRACKPAD_TOUCH].value.boolean) {
							input_y = INDEX_TRACKPAD_Y;
							input_xy = INDEX_TRACKPAD_XY;
						} else {
							input_y = INDEX_THUMBSTICK_Y;
							input_xy = INDEX_THUMBSTICK_XY;
						}

						float y = (float)((int16_t) e->axis_val[i]) / 32768.;
						survive->base.inputs[input_y].value.vec1.x = y;
						survive->base.inputs[input_y].timestamp = now;

						survive->base.inputs[input_xy].value.vec2.y = y;
						survive->base.inputs[input_xy].timestamp = now;
						//printf("y: %f\n", y);
					}

				}
			}

			if (e->button_id == survive_trigger_click_btn) {
				// 1 = pressed, 2 = released
				// printf("trigger click %d\n", e->event_type);

				survive->base.inputs[INDEX_TRIGGER_CLICK].timestamp = now;
				survive->base.inputs[INDEX_TRIGGER_CLICK].value.boolean = e->event_type == 1;
				//printf("Trigger click %d\n", survive->base.inputs[INDEX_TRIGGER_CLICK].value.boolean);
			}

			if (e->button_id == survive_a_btn) {
				survive->base.inputs[INDEX_A_CLICK].timestamp = now;
				survive->base.inputs[INDEX_A_CLICK].value.boolean = e->event_type == 1;
			}

			if (e->button_id == survive_b_btn) {
				survive->base.inputs[INDEX_B_CLICK].timestamp = now;
				survive->base.inputs[INDEX_B_CLICK].value.boolean = e->event_type == 1;
			}

			if (e->button_id == survive_trackpad_touch_btn) {
				survive->base.inputs[INDEX_TRACKPAD_TOUCH].timestamp = now;
				survive->base.inputs[INDEX_TRACKPAD_TOUCH].value.boolean = e->event_type == 1;
			}

			break;
		}
		case SurviveSimpleEventType_None:
			break;
	}
}

static void
survive_device_update_inputs(struct xrt_device *xdev, struct time_state *timekeeping)
{
	//printf("Update inputs\n");
	struct survive_device *survive = (struct survive_device*)xdev;

	int64_t now = time_state_get_now(timekeeping);

	const SurviveSimpleObject *current = survive->survive_obj;

	for (int i = 0; i < MAX_PENDING_EVENTS; i++) {
		struct SurviveSimpleEvent *event = &survive->pending_events[i];
		_process_event(survive, event, current, now);

		survive->pending_events[i].event_type = SurviveSimpleEventType_None;
	}

	struct SurviveSimpleEvent event = {0};
	while (survive_simple_next_event(survive->sys->ctx, &event) != SurviveSimpleEventType_None) {
		_process_event(survive, &event, current, now);
	}
}

static bool
_create_hmd_device(struct survive_system *sys)
{
	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(
	    U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	int inputs = 1;
	int outputs = 0;

	struct survive_device *survive =
	    U_DEVICE_ALLOCATE(struct survive_device, flags, inputs, outputs);
	sys->hmd = survive;
	survive->sys = sys;
	survive->survive_obj = NULL;

	survive->base.name = XRT_DEVICE_GENERIC_HMD;
	snprintf(survive->base.str, XRT_DEVICE_NAME_LEN, "Survive HMD");
	survive->base.destroy = survive_device_destroy;
	survive->base.update_inputs = survive_hmd_update_inputs;
	survive->base.get_tracked_pose = survive_device_get_tracked_pose;
	survive->base.get_view_pose = survive_device_get_view_pose;
	survive->base.tracking_origin = &sys->base;

	survive_simple_start_thread(sys->ctx);

	const struct device_info info = get_info();
	{
		/* right eye */
		if (!math_compute_fovs(info.views[1].display.w_meters,
		     info.views[1].lens_center_x_meters,
		     info.views[1].fov,
		     info.views[1].display.h_meters,
		     info.views[1].lens_center_y_meters, 0,
		     &survive->base.hmd->views[1].fov)) {
			SURVIVE_ERROR(survive,
				"Failed to compute the partial fields of view.");
			free(survive);
			return NULL;
		}
	}
	{
		/* left eye - just mirroring right eye now */
		survive->base.hmd->views[0].fov.angle_up =
		survive->base.hmd->views[1].fov.angle_up;
		survive->base.hmd->views[0].fov.angle_down =
		survive->base.hmd->views[1].fov.angle_down;

		survive->base.hmd->views[0].fov.angle_left =
		-survive->base.hmd->views[1].fov.angle_right;
		survive->base.hmd->views[0].fov.angle_right =
		-survive->base.hmd->views[1].fov.angle_left;
	}

	// clang-format off
	// Main display.
	survive->base.hmd->distortion.models = XRT_DISTORTION_MODEL_PANOTOOLS;
	survive->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_PANOTOOLS;
	survive->base.hmd->screens[0].w_pixels = info.display.w_pixels;
	survive->base.hmd->screens[0].h_pixels = info.display.h_pixels;
	survive->base.hmd->distortion.pano.distortion_k[0] = info.pano_distortion_k[0];
	survive->base.hmd->distortion.pano.distortion_k[1] = info.pano_distortion_k[1];
	survive->base.hmd->distortion.pano.distortion_k[2] = info.pano_distortion_k[2];
	survive->base.hmd->distortion.pano.distortion_k[3] = info.pano_distortion_k[3];
	survive->base.hmd->distortion.pano.aberration_k[0] = info.pano_aberration_k[0];
	survive->base.hmd->distortion.pano.aberration_k[1] = info.pano_aberration_k[1];
	survive->base.hmd->distortion.pano.aberration_k[2] = info.pano_aberration_k[2];

	// Left
	survive->base.hmd->views[0].display.w_meters = info.views[0].display.w_meters;
	survive->base.hmd->views[0].display.h_meters = info.views[0].display.h_meters;
	survive->base.hmd->views[0].lens_center.x_meters = info.views[0].lens_center_x_meters;
	survive->base.hmd->views[0].lens_center.y_meters = info.views[0].lens_center_y_meters;
	survive->base.hmd->views[0].display.w_pixels = info.views[0].display.w_pixels;
	survive->base.hmd->views[0].display.h_pixels = info.views[0].display.h_pixels;
	survive->base.hmd->views[0].viewport.x_pixels = 0;
	survive->base.hmd->views[0].viewport.y_pixels = 0;
	survive->base.hmd->views[0].viewport.w_pixels = info.views[0].display.w_pixels;
	survive->base.hmd->views[0].viewport.h_pixels = info.views[0].display.h_pixels;
	survive->base.hmd->views[0].rot = u_device_rotation_ident;

	// Right
	survive->base.hmd->views[1].display.w_meters = info.views[1].display.w_meters;
	survive->base.hmd->views[1].display.h_meters = info.views[1].display.h_meters;
	survive->base.hmd->views[1].lens_center.x_meters = info.views[1].lens_center_x_meters;
	survive->base.hmd->views[1].lens_center.y_meters = info.views[1].lens_center_y_meters;
	survive->base.hmd->views[1].display.w_pixels = info.views[1].display.w_pixels;
	survive->base.hmd->views[1].display.h_pixels = info.views[1].display.h_pixels;
	survive->base.hmd->views[1].viewport.x_pixels = info.views[0].display.w_pixels;
	survive->base.hmd->views[1].viewport.y_pixels = 0;
	survive->base.hmd->views[1].viewport.w_pixels = info.views[1].display.w_pixels;
	survive->base.hmd->views[1].viewport.h_pixels = info.views[1].display.h_pixels;
	survive->base.hmd->views[1].rot = u_device_rotation_ident;
	// clang-format on

	// Find any needed quirks.
	bool quirk_video_see_through = false;
	bool quirk_video_distortion_vive = false;

	quirk_video_distortion_vive = true;
	quirk_video_see_through = false;

	// Which blend modes does the device support.
	survive->base.hmd->blend_mode |= XRT_BLEND_MODE_OPAQUE;
	if (quirk_video_see_through) {
		survive->base.hmd->blend_mode |= XRT_BLEND_MODE_ALPHA_BLEND;
	}

	if (quirk_video_distortion_vive) {
		survive->base.hmd->distortion.models |= XRT_DISTORTION_MODEL_VIVE;
		survive->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_VIVE;

		// clang-format off
		// These need to be aquired from the vive config
		survive->base.hmd->distortion.vive.aspect_x_over_y = 0.8999999761581421f;
		survive->base.hmd->distortion.vive.grow_for_undistort = 0.6000000238418579f;
		survive->base.hmd->distortion.vive.undistort_r2_cutoff[0] = 1.11622154712677f;
		survive->base.hmd->distortion.vive.undistort_r2_cutoff[1] = 1.101870775222778f;
		survive->base.hmd->distortion.vive.center[0][0] = 0.08946027017045266f;
		survive->base.hmd->distortion.vive.center[0][1] = -0.009002181016260827f;
		survive->base.hmd->distortion.vive.center[1][0] = -0.08933516629552526f;
		survive->base.hmd->distortion.vive.center[1][1] = -0.006014565287238661f;

		// left
		// green
		survive->base.hmd->distortion.vive.coefficients[0][0][0] = -0.188236068524731f;
		survive->base.hmd->distortion.vive.coefficients[0][0][1] = -0.221086205321053f;
		survive->base.hmd->distortion.vive.coefficients[0][0][2] = -0.2537849057915209f;

		// blue
		survive->base.hmd->distortion.vive.coefficients[0][1][0] = -0.07316590815739493f;
		survive->base.hmd->distortion.vive.coefficients[0][1][1] = -0.02332400789561968f;
		survive->base.hmd->distortion.vive.coefficients[0][1][2] = 0.02469959434698275f;

		// red
		survive->base.hmd->distortion.vive.coefficients[0][2][0] = -0.02223805567703767f;
		survive->base.hmd->distortion.vive.coefficients[0][2][1] = -0.04931309279533211f;
		survive->base.hmd->distortion.vive.coefficients[0][2][2] = -0.07862881939243466f;

		// right
		// green
		survive->base.hmd->distortion.vive.coefficients[1][0][0] = -0.1906209981894497f;
		survive->base.hmd->distortion.vive.coefficients[1][0][1] = -0.2248896677207884f;
		survive->base.hmd->distortion.vive.coefficients[1][0][2] = -0.2721364516782803f;

		// blue
		survive->base.hmd->distortion.vive.coefficients[1][1][0] = -0.07346071902951497f;
		survive->base.hmd->distortion.vive.coefficients[1][1][1] = -0.02189527566250131f;
		survive->base.hmd->distortion.vive.coefficients[1][1][2] = 0.0581378652359256f;

		// red
		survive->base.hmd->distortion.vive.coefficients[1][2][0] = -0.01755850332081247f;
		survive->base.hmd->distortion.vive.coefficients[1][2][1] = -0.04517245633373419f;
		survive->base.hmd->distortion.vive.coefficients[1][2][2] = -0.0928909347763f;
		// clang-format on
	}
	return true;
}

static bool
_create_controller_device(struct survive_system *sys,
			  int controller_num) {

	enum u_device_alloc_flags flags = (enum u_device_alloc_flags)(
	    U_DEVICE_ALLOC_TRACKING_NONE);
	int inputs = LAST_INPUT;
	int outputs = 0;
	struct survive_device *controller =
	    U_DEVICE_ALLOCATE(struct survive_device, flags, inputs, outputs);
	sys->controllers[controller_num] = controller;
	controller->sys = sys;
	controller->survive_obj = NULL;
	for (int i = 0; i < MAX_PENDING_EVENTS; i++)
		controller->pending_events[i].event_type = SurviveSimpleEventType_None;

	controller->num = controller_num;
	controller->base.name = XRT_DEVICE_INDEX_CONTROLLER;
	snprintf(controller->base.str, XRT_DEVICE_NAME_LEN,
	         "Survive Controller %d", controller_num);
	controller->base.tracking_origin = &sys->base;

	controller->base.destroy = survive_device_destroy;
	controller->base.update_inputs = survive_device_update_inputs;
	controller->base.get_tracked_pose = survive_device_get_tracked_pose;

	controller->base.inputs[INDEX_AIM_POSE].name = XRT_INPUT_INDEX_AIM_POSE;
	controller->base.inputs[INDEX_GRIP_POSE].name = XRT_INPUT_INDEX_GRIP_POSE;

	controller->base.inputs[INDEX_TRIGGER_VALUE].name = XRT_INPUT_INDEX_TRIGGER_VALUE;

	controller->base.inputs[INDEX_TRIGGER_CLICK].name = XRT_INPUT_INDEX_TRIGGER_CLICK;
	controller->base.inputs[INDEX_TRIGGER_CLICK].value.boolean = false;

	controller->base.inputs[INDEX_A_CLICK].name = XRT_INPUT_INDEX_A_CLICK;
	controller->base.inputs[INDEX_A_CLICK].value.boolean = false;

	controller->base.inputs[INDEX_B_CLICK].name = XRT_INPUT_INDEX_B_CLICK;
	controller->base.inputs[INDEX_B_CLICK].value.boolean = false;


	controller->base.inputs[INDEX_TRIGGER_CLICK].name = XRT_INPUT_INDEX_TRIGGER_CLICK;
	controller->base.inputs[INDEX_TRIGGER_CLICK].value.boolean = false;
	controller->base.inputs[INDEX_TRIGGER_CLICK].name = XRT_INPUT_INDEX_TRIGGER_CLICK;
	controller->base.inputs[INDEX_TRIGGER_CLICK].value.boolean = false;


	controller->base.inputs[INDEX_THUMBSTICK_X].name = XRT_INPUT_INDEX_THUMBSTICK_X;
	controller->base.inputs[INDEX_THUMBSTICK_Y].name = XRT_INPUT_INDEX_THUMBSTICK_Y;
	controller->base.inputs[INDEX_THUMBSTICK_XY].name = XRT_INPUT_INDEX_THUMBSTICK_XY;

	controller->base.inputs[INDEX_TRACKPAD_X].name = XRT_INPUT_INDEX_TRACKPAD_X;
	controller->base.inputs[INDEX_TRACKPAD_Y].name = XRT_INPUT_INDEX_TRACKPAD_Y;
	controller->base.inputs[INDEX_TRACKPAD_XY].name = XRT_INPUT_INDEX_TRACKPAD_XY;


	controller->base.inputs[INDEX_TRACKPAD_TOUCH].name = XRT_INPUT_INDEX_TRACKPAD_TOUCH;
	/* TODO: maybe trackpad is touched initially... */
	controller->base.inputs[INDEX_TRACKPAD_TOUCH].value.boolean = false;

	return true;
}

//struct xrt_device *
//survive_device_create(bool print_spew, bool print_debug)

DEBUG_GET_ONCE_BOOL_OPTION(survive_spew, "SURVIVE_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(survive_debug, "SURVIVE_PRINT_DEBUG", false)

int
survive_found(struct xrt_prober *xp,
              struct xrt_prober_device **devices,
              size_t num_devices,
              size_t index,
              struct xrt_device **out_xdevs)
{
	if (survive_already_initialized) {
		printf("Skipping libsurvive initialization, already initialized\n");
		return 0;
	}

	SurviveSimpleContext *actx = NULL;
#if 1
	char *survive_args[] = {
		"Monado-libsurvive",
		//"--time-window", "1500000"
		//"--use-imu", "0",
		//"--use-kalman", "0"
	};
	actx = survive_simple_init(
		sizeof(survive_args) / sizeof(survive_args[0]), survive_args);
#else
	actx = survive_simple_init(0, 0);
#endif

	if (!actx) {
		SURVIVE_ERROR(survive, "failed to init survive");
		return false;
	}

	struct survive_system *ss = U_TYPED_CALLOC(struct survive_system);
	ss->ctx = actx;
	ss->base.type = XRT_TRACKING_TYPE_NONE; // ??
	snprintf(ss->base.name, XRT_TRACKING_NAME_LEN, "%s", "Libsurvive Tracking");
	ss->base.offset.position.y = 0.0f;
	ss->base.offset.position.z = 0.0f;
	ss->base.offset.orientation.w = 1.0f;

	ss->print_spew = debug_get_bool_option_survive_spew();
	ss->print_debug = debug_get_bool_option_survive_debug();

	_create_hmd_device(ss);
	_create_controller_device(ss, 0);
	_create_controller_device(ss, 1);

	//printf("Survive HMD %p, controller %p %p\n", ss->hmd, ss->controllers[0], ss->controllers[1]);

	if (ss->print_debug) {
		u_device_dump_config(&ss->hmd->base, __func__, "libsurvive");
	}

	out_xdevs[0] = &ss->hmd->base;
	out_xdevs[1] = &ss->controllers[0]->base;
	out_xdevs[2] = &ss->controllers[1]->base;

	survive_already_initialized = true;
	return 3;
}
