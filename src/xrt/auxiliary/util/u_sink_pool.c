// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pool for @ref xrt_frame.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include "util/u_sink.h"

#include <assert.h>
#include <pthread.h>


struct xrt_frame_pool
{
	pthread_mutex_t mutex;

	struct xrt_frame **frames;
	uint32_t num_frames;
	uint32_t num_pool_pop;
};


static void
free_frame(struct xrt_frame *xf, void *owner)
{
	struct xrt_frame_pool *p = (struct xrt_frame_pool *)owner;
	// Have to lock it again.
	pthread_mutex_lock(&p->mutex);

	assert(xf->reference.count == 0);
	assert(p->num_pool_pop < p->num_frames);

	p->frames[p->num_pool_pop++] = xf;

	pthread_mutex_unlock(&p->mutex);
}

bool
xrt_frame_pool_get(struct xrt_frame_pool *p, struct xrt_frame **out_frame)
{
	struct xrt_frame *xf = NULL;

	pthread_mutex_lock(&p->mutex);

	if (p->num_pool_pop == 0) {
		pthread_mutex_unlock(&p->mutex);
		return false;
	}

	xf = p->frames[--p->num_pool_pop];
	p->frames[p->num_pool_pop] = NULL;

	pthread_mutex_unlock(&p->mutex);

	// Make sure to reference without the lock being held.
	xrt_frame_reference(out_frame, xf);

	return true;
}
