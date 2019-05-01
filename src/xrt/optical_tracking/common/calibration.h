#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <xrt/xrt_defines.h>

#define INTRINSICS_SIZE 9
#define DISTORTION_SIZE 5

typedef struct camera_calibration {
     float intrinsics[INTRINSICS_SIZE];
     float distortion[DISTORTION_SIZE];
     float calib_capture_size[2];
	 struct xrt_pose pose; //camera position
     float reprojection_error;
} camera_calibration_t;

#endif //CALIBRATION_H

//measured with opencv checkerboard calibration and Distortion Tools

#define LOGITECH_C270_DIST {-0.0796f,0.6299f,0.0008f,0.0050f,-1.4538f}
#define LOGITECH_C270_INTR {1430.4364f,0.0f,644.6101f,0.0f,1434.7814f,476.8615f,0.0f,0.0f,1.0f}
#define LOGITECH_C270_SIZE {1280.0f,960.0f}
