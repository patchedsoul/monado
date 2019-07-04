#pragma once

#include <xrt/xrt_defines.h>
#include "common/tracked_object.h"
#include "common/tracker.h"
#include "opencv2/opencv.hpp"

#define NUM_LEDS 9

static XRT_MAYBE_UNUSED const char* LED_LABELS[] = {
    "LU", "RU", "C", "LL", "RL", "LS", "RS", "LB", "RB"};

#ifdef __cplusplus
extern "C" {
#endif


typedef struct psvr_track_data
{
	uint64_t timestamp;
	xrt_vec3 positions_3d[NUM_LEDS]; // x,y,z position for up to (currently)
	                                 // 9 points - LU,RU,C,LL,RL,LS,RS,LB,RB
	xrt_vec2
	    l_positions_2d[NUM_LEDS]; // 2d positions in left and right images
	xrt_vec2 r_positions_2d[NUM_LEDS];
	int8_t confidence[NUM_LEDS]; //-1 if point is not tracked, TODO: 0-128
	                             // for confidence
	xrt_matrix_4x4 rotation_matrix; // SVD-fitted head rotation matrix
	xrt_vec3 translation;           // head translation
} psvr_track_data_t;


typedef struct point_dist
{
	uint32_t index;
	xrt_vec3 translation;
	float length;
	xrt_vec3 scaled_translation;
	float scaled_length;
} point_dist_t;

typedef struct psvr_led
{
	xrt_vec2 l_position_2d;
	xrt_vec2 r_position_2d;
	float radius;
	cv::Point2f distance;
	xrt_vec3 position;
	xrt_vec3 scaled_position;
	std::vector<point_dist> point_dists;
	float rms_distance;
	int sign_x;
	int sign_y;
} psvr_led_t;

bool
psvr_disambiguate_5points(std::vector<psvr_led_t>* leds, psvr_track_data* t);

// bool psvr_computeSVD();


#ifdef __cplusplus
}
#endif
