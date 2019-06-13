# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}/../auxiliary
	${CMAKE_CURRENT_SOURCE_DIR}
	)

set(FILTER_SOURCE_FILES
	common/filter.h
	common/filter.c
	filter_opencv_kalman.cpp
	filter_opencv_kalman.h
	)

# Use OBJECT to not create a archive, since it just gets in the way.
add_library(filter OBJECT ${FILTER_SOURCE_FILES})
set_property(TARGET filter PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(filter SYSTEM
	PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}../
	PRIVATE ${OpenCV_INCLUDE_DIRS}
	)
