# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
	)

set(OPTICAL_TRACKING_SOURCE_FILES
        common/calibration.cpp
	common/calibration.h
	common/tracked_object.h
	common/tracked_object.c
	common/tracker.h
	common/tracker.c
	common/opencv_utils.hpp
	tracker3D_sphere_stereo.cpp
	tracker3D_sphere_stereo.h
	tracker3D_sphere_mono.cpp
	tracker3D_sphere_mono.h
	track_psvr.h
	track_psvr.cpp
	)
if(BUILD_WITH_UVBI)
	list(APPEND OPTICAL_TRACKING_SOURCE_FILES
		tracker3D_uvbi.cpp
		tracker3D_uvbi.h
	)
endif()

# Use OBJECT to not create a archive, since it just gets in the way.
add_library(optical_tracking OBJECT ${OPTICAL_TRACKING_SOURCE_FILES})
set_property(TARGET optical_tracking PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(optical_tracking SYSTEM
	PRIVATE
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/..
	)
if(BUILD_WITH_UVBI)
	target_link_libraries(optical_tracking PRIVATE uvbi::uvbi_plugin_parts)
endif()
