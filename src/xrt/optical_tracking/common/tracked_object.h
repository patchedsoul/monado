#ifndef TRACKEDOBJECT_H
#define TRACKEDOBJECT_H
#include <xrt/xrt_defines.h>
#include "../auxiliary/util/u_time.h"

typedef struct tracked_object {
    struct xrt_pose pose;
    time_t pose_time;
    time_t frame_time;
	uint32_t tracking_tag; // a tracker may assign an opaque tag to denote object type
	uint32_t tracking_id; // a tracker may assign an opaque id to facilitate interframe correlation
} tracked_object_t;

#endif //TRACKEDOBJECT_H
