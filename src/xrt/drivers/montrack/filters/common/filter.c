#include "filter.h"
#include "filter_opencv_kalman.h"
#include <string.h>

#include "util/u_misc.h"

struct filter_instance*
filter_create(filter_type_t t)
{
	switch (t) {
	case FILTER_TYPE_OPENCV_KALMAN: return filter_opencv_kalman_create();
	case FILTER_TYPE_NONE:
	default: return NULL;
	}
}

bool
filters_test()
{

	// create a filter
	struct filter_instance* filter =
	    filter_create(FILTER_TYPE_OPENCV_KALMAN);
	if (!filter) {
		return false;
	}

	return true;
}
