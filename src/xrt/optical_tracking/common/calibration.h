#ifndef CALIBRATIONN_H
#define DISTORTION_H
#include <xrt/xrt_defines.h>

#define INTRINSICS_SIZE 9

typedef struct camera_calibration {
     float intrinsics[INTRINSICS_SIZE];
     struct xrt_pose pose;
     float reprojection_error;
} camera_calibration_t;

#endif //DISTORTION_H
