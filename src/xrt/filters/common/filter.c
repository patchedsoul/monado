#include "filter.h"
#include <string.h>

filter_instance_t* filter_create(filter_type_t t) {
	filter_instance_t* i = calloc(1,sizeof(filter_instance_t));
	if (i) {
		switch (t) {
		    case FILTER_TYPE_OPENCV_KALMAN:
			    i->tracker_type = t;
				i->internal_instance = filter_opencv_kalman_create(i);
			    break;
		    case FILTER_TYPE_NONE:
		    default:
			    free(i);
			    return NULL;
			break;
		}
		return i;
	}
	return NULL;
}

bool filters_test(){

	//create a filter
	filter_instance_t* filter = filter_create(FILTER_TYPE_OPENCV_KALMAN);
	if (! filter)
	{
		return false;
	}

	return true;
}
