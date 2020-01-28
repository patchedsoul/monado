// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining the tracking system integration in Monado.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#define XRT_TRACKING_NAME_LEN 256

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#ifdef __cplusplus
extern "C" {
#endif


struct time_state;
struct xrt_device;
struct xrt_tracking;
struct xrt_tracking_factory;
struct xrt_tracked_psmv;
struct xrt_tracked_psvr;

/*!
 * @ingroup xrt_iface
 * @{
 */

/*!
 * What kind of tracking system is this.
 *
 * @todo Is none, Colour, IR, Magnetic the kind of type we need to know about?
 */
enum xrt_tracking_type
{
	// The device(s) are never tracked.
	XRT_TRACKING_TYPE_NONE,

	// The device(s) are tracked by RGB camera(s).
	XRT_TRACKING_TYPE_RGB,
};

/*!
 * A tracking system or device origin.
 */
struct xrt_tracking_origin
{
	//! For debugging.
	char name[XRT_TRACKING_NAME_LEN];

	//! What can the state tracker expect from this tracking system.
	enum xrt_tracking_type type;

	/*!
	 * Read and written to by the state-tracker using the device(s)
	 * this tracking system is tracking.
	 */
	struct xrt_pose offset;
};

/*!
 * Tracking factory.
 */
struct xrt_tracking_factory
{
	//! Internal frame context, exposed for debugging purposes.
	struct xrt_frame_context *xfctx;

	/*!
	 * Create a tracked PSMV ball.
	 */
	int (*create_tracked_psmv)(struct xrt_tracking_factory *,
	                           struct xrt_device *xdev,
	                           struct xrt_tracked_psmv **out_psmv);

	/*!
	 * Create a tracked PSVR ball.
	 */
	int (*create_tracked_psvr)(struct xrt_tracking_factory *,
	                           struct xrt_device *xdev,
	                           struct xrt_tracked_psvr **out_psvr);
};

/*!
 * IMU Sample.
 */
struct xrt_tracking_sample
{
	struct xrt_vec3 accel_m_s2;
	struct xrt_vec3 gyro_rad_secs;
};

/*!
 * A single tracked PS Move controller, camera and ball are not synced.
 *
 * @todo How do we communicate ball colour change?
 */
struct xrt_tracked_psmv
{
	//! The tracking system origin for this ball.
	struct xrt_tracking_origin *origin;

	//! Device owning this ball.
	struct xrt_device *xdev;

	//! Colour of the ball.
	struct xrt_colour_rgb_f32 colour;

	/*!
	 * Push a IMU sample into the tracking system.
	 */
	void (*push_imu)(struct xrt_tracked_psmv *,
	                 timepoint_ns timestamp_ns,
	                 struct xrt_tracking_sample *sample);

	/*!
	 * Called by the owning @ref xrt_device @ref xdev to get the pose of
	 * the ball in the tracking space at the given time.
	 *
	 * @todo Should we add a out_time argument as a way to signal min and
	 * maximum, and as such only do interpelation between different captured
	 * frames.
	 */
	void (*get_tracked_pose)(struct xrt_tracked_psmv *,
	                         enum xrt_input_name name,
	                         struct time_state *timekeeper,
	                         timepoint_ns when_ns,
	                         struct xrt_space_relation *out_relation);

	/*!
	 * Destroy this tracked ball.
	 */
	void (*destroy)(struct xrt_tracked_psmv *);
};

/*!
 * A tracked PSVR headset.
 *
 * @todo How do we communicate led lighting status?
 */
struct xrt_tracked_psvr
{
	//! The tracking system origin for this ball.
	struct xrt_tracking_origin *origin;

	//! Device owning this ball.
	struct xrt_device *xdev;

	/*!
	 * Push a IMU sample into the tracking system.
	 */
	void (*push_imu)(struct xrt_tracked_psvr *,
	                 timepoint_ns timestamp_ns,
	                 struct xrt_tracking_sample *sample);

	/*!
	 * Called by the owning @ref xrt_device @ref xdev to get the pose of
	 * the psvr in the tracking space at the given time.
	 */
	void (*get_tracked_pose)(struct xrt_tracked_psvr *,
	                         struct time_state *timekeeper,
	                         timepoint_ns when_ns,
	                         struct xrt_space_relation *out_relation);

	/*!
	 * Destroy this tracked psvr.
	 */
	void (*destroy)(struct xrt_tracked_psvr *);
};


/*!
 * A Leap Motion tracker - notional single tracked object
 */
struct xrt_tracked_leap
{
	//! The tracking system origin for this object.
	struct xrt_tracking_origin *origin;

	//! Device owning this object.
	struct xrt_device *xdev;

	/*!
	 * Push a IMU sample into the tracking system.
	 */
	void (*push_imu)(struct xrt_tracked_leap *,
	                 timepoint_ns timestamp_ns,
	                 struct xrt_tracking_sample *sample);

	/*!
	 * Called by the owning @ref xrt_device @ref xdev to get the pose of
	 * the object in the tracking space at the given time.
	 */
	void (*get_tracked_pose)(struct xrt_tracked_leap *,
	                         struct time_state *timekeeper,
	                         timepoint_ns when_ns,
	                         struct xrt_space_relation *out_relation);

	/*!
	 * Destroy this tracked leap.
	 */
	void (*destroy)(struct xrt_tracked_leap *);
};


/*
 *
 * Helper functions.
 *
 */

static inline void
xrt_tracked_psmv_get_tracked_pose(struct xrt_tracked_psmv *psmv,
                                  enum xrt_input_name name,
                                  struct time_state *timekeeper,
                                  timepoint_ns when_ns,
                                  struct xrt_space_relation *out_relation)
{
	psmv->get_tracked_pose(psmv, name, timekeeper, when_ns, out_relation);
}

static inline void
xrt_tracked_psmv_push_imu(struct xrt_tracked_psmv *psmv,
                          timepoint_ns timestamp_ns,
                          struct xrt_tracking_sample *sample)
{
	psmv->push_imu(psmv, timestamp_ns, sample);
}

static inline void
xrt_tracked_psmv_destroy(struct xrt_tracked_psmv **xtmv_ptr)
{
	struct xrt_tracked_psmv *xtmv = *xtmv_ptr;
	if (xtmv == NULL) {
		return;
	}

	xtmv->destroy(xtmv);
	*xtmv_ptr = NULL;
}

static inline void
xrt_tracked_psvr_get_tracked_pose(struct xrt_tracked_psvr *psvr,
                                  struct time_state *timekeeper,
                                  timepoint_ns when_ns,
                                  struct xrt_space_relation *out_relation)
{
	psvr->get_tracked_pose(psvr, timekeeper, when_ns, out_relation);
}

static inline void
xrt_tracked_psvr_push_imu(struct xrt_tracked_psvr *psvr,
                          timepoint_ns timestamp_ns,
                          struct xrt_tracking_sample *sample)
{
	psvr->push_imu(psvr, timestamp_ns, sample);
}

static inline void
xrt_tracked_psvr_destroy(struct xrt_tracked_psvr **xtvr_ptr)
{
	struct xrt_tracked_psvr *xtvr = *xtvr_ptr;
	if (xtvr == NULL) {
		return;
	}

	xtvr->destroy(xtvr);
	*xtvr_ptr = NULL;
}


static inline void
xrt_tracked_leap_get_tracked_pose(struct xrt_tracked_leap *leap,
                                  struct time_state *timekeeper,
                                  timepoint_ns when_ns,
                                  struct xrt_space_relation *out_relation)
{
	leap->get_tracked_pose(leap, timekeeper, when_ns, out_relation);
}

static inline void
xrt_tracked_leap_push_imu(struct xrt_tracked_leap *leap,
                          timepoint_ns timestamp_ns,
                          struct xrt_tracking_sample *sample)
{
	leap->push_imu(leap, timestamp_ns, sample);
}

static inline void
xrt_tracked_leap_destroy(struct xrt_tracked_leap **xtleap_ptr)
{
	struct xrt_tracked_leap *xtleap = *xtleap_ptr;
	if (xtleap == NULL) {
		return;
	}

	xtleap->destroy(xtleap);
	*xtleap_ptr = NULL;
}


/*!
 * @}
 */


#ifdef __cplusplus
}
#endif
