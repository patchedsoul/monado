#ifndef DISTORTION_H
#define DISTORTION_H
#include <xrt/xrt_defines.h>

#define DISTORTION_SIZE 5

typedef struct camera_distortion {
     float intrinsics[5];
} camera_distortion_t;

#endif //DISTORTION_H
