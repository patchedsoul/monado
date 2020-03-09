// Copyright 2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A fifo that also allows you to dynamically filter.
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif


struct m_ft_f32
{
	float *values;

	size_t size;

	size_t index;

	double sum;
};

static inline void
m_ft_f32_init(struct m_ft_f32 *ft, size_t num)
{
	U_ZERO(ft);
	ft->size = num;
	ft->values = U_TYPED_ARRAY_CALLOC(float, num);
}

static inline void
m_ft_f32_destroy(struct m_ft_f32 *ft)
{
	free(ft->values);
	U_ZERO(ft);
}

static inline void
m_ft_f32_push(struct m_ft_f32 *ft, float timing)
{
	size_t index = (ft->index + 1) % ft->size;
	double sum = ft->sum - ft->values[index] + timing;

	ft->values[index] = timing;
	ft->sum = sum;
	ft->index = index;
}

static inline float
m_ft_f32_avg(struct m_ft_f32 *ft)
{
	return ft->sum / (double)ft->size;
}


#ifdef __cplusplus
}

/*!
 * Helper class to wrap a C filter timing.
 */
class FilterTimingF
{
private:
	m_ft_f32 ft;


public:
	FilterFifo3F() = delete;

	FilterFifo3F(size_t size)
	{
		m_ft_f32_init(&ft, size);
	}

	~FilterFifo3F()
	{
		m_ft_f32_destroy(&ft);
	}

	inline void
	push(float timing)
	{
		m_ft_f32_push(&ft, timing);
	}

	inline float
	avg()
	{
		return m_ft_f32_avg(&ft);
	}
};
#endif
