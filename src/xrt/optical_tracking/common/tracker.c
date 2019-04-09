#include "tracker.h"
#include "tracker3D_sphere_mono.h"

tracker_instance_t* tracker_create(tracker_type_t t) {
	tracker_instance_t* i = calloc(1,sizeof(tracker_instance_t));
	if (i) {
		switch (t) {
		    case TRACKER_TYPE_SPHERE_MONO:
			    i->internal_instance = tracker3D_sphere_mono_create(i);
			    break;
		    case TRACKER_TYPE_NONE:
		    default:
			    free(i);
			    return NULL;
			break;
		}
		return i;
	}
	return NULL;
}
