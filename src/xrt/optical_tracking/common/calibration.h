#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <xrt/xrt_defines.h>

typedef enum camera_lens_type {CAMERA_LENS_NORMAL,CAMERA_LENS_FISHEYE} camera_lens_type_t;

#define INTRINSICS_SIZE 9
#define DISTORTION_SIZE 5
#define DISTORTION_FISHEYE_SIZE 4

typedef struct camera_calibration {
	camera_lens_type_t lens_type;
     float intrinsics[INTRINSICS_SIZE];
     float distortion[DISTORTION_SIZE];
	 float distortion_fisheye[DISTORTION_FISHEYE_SIZE];
	 float calib_capture_size[2]; //frame size at calibration time
	 struct xrt_pose pose; //camera position
     float reprojection_error;
} camera_calibration_t;

#endif //CALIBRATION_H

//measured with opencv checkerboard calibration and Distortion Tools

#define LOGITECH_C270_DIST {-0.0796f,0.6299f,0.0008f,0.0050f,-1.4538f}
#define LOGITECH_C270_INTR {1430.4364f,0.0f,644.6101f,0.0f,1434.7814f,476.8615f,0.0f,0.0f,1.0f}
#define LOGITECH_C270_SIZE {1280.0f,960.0f}

#define ELP_60FPS_DIST_L { -0.5850541915382547f, 0.2565158062087056f, 0.005209549905548239f, 0.001261134736355752f, -0.04273596601433997f}
#define ELP_60FPS_INTR_L { 714.0812692265652,0,698.920232470781,0,717.4516365199725,503.23559009954033,0,0,1}
#define ELP_60FPS_DIST_R { -0.5885807761934678,0.27743136288746995,0.005787057513468129,-0.0033120976560522155,-0.05103016878919477}
#define ELP_60FPS_INTR_R {668.3687642528068,0,695.2481699807906,0,672.5260709077188,531.9588298758522,0,0,1}
#define ELP_60FPS_SIZE {1280.0f,960.0f}
