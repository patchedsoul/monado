#include "frameserver.h"

int32_t frame_size_in_bytes(frame_data_t fd) {
	printf("ERROR: Not implemented");
	return -1;
}

int32_t frame_bytes_per_pixel(frame_data_t fd){
	printf("ERROR: Not implemented\n");
	return -1;
}

bool split_stereo_frame(frame_data_t source, frame_data_t* left,  frame_data_t* right){
	printf("ERROR: Not implemented!\n");
	return false;
}

bool extract_plane(frame_data_t source, plane_t plane, frame_data_t* out) {
	printf("ERROR: Not implemented!\n");
	return false;
}
