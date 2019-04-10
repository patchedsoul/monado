#ifndef TRACKER_H
#define TRACKER_H
#include <xrt/xrt_defines.h>
#include "../frameservers/common/frameserver.h"
#include "tracked_object.h"

typedef void* tracker_instance;
typedef void* tracker_internal_instance;

typedef enum tracker_type {
    TRACKER_TYPE_NONE,
    TRACKER_TYPE_2D_BLUE_LED,
    TRACKER_TYPE_SPHERE_MONO
} tracker_type_t;

typedef struct _tracker_instance {
     tracker_type_t tracker_type;
     bool (*tracker_track)(tracker_instance inst,frame_t* frame);
	 bool (*tracker_get_poses)(tracker_instance inst,tracked_object_t* tracked_objects,uint32_t* count);
     tracker_internal_instance internal_instance;
} tracker_instance_t;

tracker_instance_t* tracker_create(tracker_type_t t);
bool tracker_destroy(tracker_instance_t* inst);
bool trackers_test();

#endif //TRACKER_H
