#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <xrt/xrt_defines.h>

#define INTRINSICS_SIZE 9
#define DISTORTION_SIZE 5

typedef struct camera_calibration {
     float intrinsics[INTRINSICS_SIZE];
     float distortion[DISTORTION_SIZE];
	 struct xrt_pose pose; //camera position
     float reprojection_error;
} camera_calibration_t;

#endif //CALIBRATION_H
