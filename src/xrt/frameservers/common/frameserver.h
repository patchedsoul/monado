#ifndef FRAMESERVER_H
#define FRAMESERVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef enum frame_format {FORMAT_RAW,FORMAT_Y_UINT8,FORMAT_Y_UINT16,FORMAT_RGB_UINT8,FORMAT_BGR_UINT8,FORMAT_YUYV_UINT8,FORMAT_YUV444_UINT8,FORMAT_JPG} frame_format_t;
typedef enum stereo_format {STEREO_NONE,STEREO_SBS,STEREO_OAU} stereo_format_t;
typedef enum plane {PLANE_R,PLANE_G,PLANE_B,PLANE_Y,PLANE_U,PLANE_V} plane_t;

typedef struct frame_data
{
	uint16_t width;
	uint16_t height;
	uint16_t stride;
	frame_format_t format;
	stereo_format_t stereo_format;
	uint8_t* data;
	uint64_t timestamp;
	uint64_t src_timestamp;
} frame_data_t;

int32_t frame_size_in_bytes(frame_data_t fd);
int32_t frame_bytes_per_pixel(frame_data_t fd);
bool split_stereo_frame(frame_data_t source, frame_data_t* left, frame_data_t* right);
bool extract_plane(frame_data_t source,plane_t plane,frame_data_t* out);

#endif //FRAMESERVER_H
