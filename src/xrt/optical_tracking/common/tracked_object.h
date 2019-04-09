#ifndef TRACKEDOBJECT_H
#define TRACKEDOBJECT_H
#include <xrt/xrt_defines.h>
#include "../auxiliary/util/u_time.h"
typedef struct tracked_object {
    struct xrt_pose pose;
    time_t pose_time;
    time_t frame_time;
} tracked_object_t;

#endif //CALIBRATION_H
