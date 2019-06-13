# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}/../auxiliary
	${CMAKE_CURRENT_SOURCE_DIR}
	)

set(FRAMESERVER_SOURCE_FILES
	common/frameserver.c
	common/frameserver.h
	)

if(BUILD_WITH_FFMPEG)
	list(APPEND FRAMESERVER_SOURCE_FILES
		ffmpeg/ffmpeg_frameserver.c
		ffmpeg/ffmpeg_frameserver.h
	)
endif()

if(BUILD_WITH_LIBUVC AND BUILD_WITH_JPEG)
	list(APPEND FRAMESERVER_SOURCE_FILES
		uvc/uvc_frameserver.c
		uvc/uvc_frameserver.h
	)
endif()

if(BUILD_WITH_JPEG)
	list(APPEND FRAMESERVER_SOURCE_FILES
		v4l2/v4l2_frameserver.c
		v4l2/v4l2_frameserver.h
	)
endif()

# Use OBJECT to not create a archive, since it just gets in the way.
add_library(frameserver OBJECT ${FRAMESERVER_SOURCE_FILES})
set_property(TARGET frameserver PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(frameserver SYSTEM
	PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/..
	)

if(BUILD_WITH_LIBUVC AND BUILD_WITH_JPEG)
	target_include_directories(frameserver SYSTEM
		PRIVATE
		${libuvc_INCLUDE_DIRS}
		${LIBUSB1_INCLUDE_DIRS}
		)
endif()

if(BUILD_WITH_JPEG)
	target_include_directories(frameserver SYSTEM
		PRIVATE
		${JPEG_INCLUDE_DIRS}
		)
endif()
