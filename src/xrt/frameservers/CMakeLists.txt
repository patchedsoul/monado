# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}/../auxiliary
	${CMAKE_CURRENT_SOURCE_DIR}
	)

set(FRAMESERVER_SOURCE_FILES
	ffmpeg/ffmpeg_frameserver.c
	ffmpeg/ffmpeg_frameserver.h
	uvc/uvc_frameserver.c
	uvc/uvc_frameserver.h
	v4l2/v4l2_frameserver.c
	v4l2/v4l2_frameserver.h
	common/frameserver.c
	common/frameserver.h
	)

# Use OBJECT to not create a archive, since it just gets in the way.
add_library(frameserver OBJECT ${FRAMESERVER_SOURCE_FILES})
set_property(TARGET frameserver PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(frameserver SYSTEM
	PRIVATE ${LIBUVC_INCLUDE_DIRS}
	)
