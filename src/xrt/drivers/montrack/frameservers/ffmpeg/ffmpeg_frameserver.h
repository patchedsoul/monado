#ifndef FFMPEG_FRAMESERVER_H
#define FFMPEG_FRAMESERVER_H

/* Almost all of the ground covered here would be covered
 * by the v4l2 frameserver on linux, but uvc may be the
 * simplest approach for cross-platform e.g. OS X
 */

#include <stdint.h>
#include <stdio.h>

#include "../common/frameserver.h"


typedef struct ffmpeg_source_descriptor
{
	char name[128];
	char* filepath;
	uint64_t source_id;
	uint32_t current_frame;
	uint32_t frame_count;
	frame_format_t format;
	uint32_t width;
	uint32_t height;
} ffmpeg_source_descriptor_t;

typedef struct ffmpeg_frameserver_instance ffmpeg_frameserver_instance_t;

ffmpeg_frameserver_instance_t*
ffmpeg_frameserver_create(frameserver_instance_t* inst);

bool
ffmpeg_frameserver_destroy(ffmpeg_frameserver_instance_t* inst);

/*! @todo are these candidates for static? are they unused? */
bool
ffmpeg_source_create(ffmpeg_source_descriptor_t* desc);
bool
ffmpeg_source_destroy(ffmpeg_source_descriptor_t* desc);

bool
ffmpeg_frameserver_test();


#endif // UVC_FRAMESERVER_H
