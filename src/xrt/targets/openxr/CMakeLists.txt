# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

######
# Create a loadable OpenXR driver.

# set(RUNTIME_BARE_PREFIX xrt)
# set(RUNTIME_PREFIX ${RUNTIME_BARE_PREFIX}_)

set(RUNTIME_BARE_SUFFIX monado)
set(RUNTIME_SUFFIX _${RUNTIME_BARE_SUFFIX})

set(RUNTIME_TARGET ${RUNTIME_PREFIX}openxr${RUNTIME_SUFFIX} CACHE INTERNAL "" FORCE)

set(XR_API_MAJOR "0")


# $LIBPATH is a generator expression that is evaluated after configure_file, so we have to use file(GENERATE) instead

###
# Generate runtime manifest with absolute path to runtime intended for development without installing
file(
	GENERATE
	OUTPUT "${CMAKE_BINARY_DIR}/${RUNTIME_TARGET}-dev.json"
	CONTENT
"{
    \"file_format_version\": \"1.0.0\",
    \"runtime\": {
        \"library_path\": \"$<TARGET_SONAME_FILE:${RUNTIME_TARGET}>\"
    }
}
")

###
# Generate runtime manifest with relative path to runtime intendend for installed release build
# (assumes the runtime library is in the loader's search path)
file(
	GENERATE
	OUTPUT "${CMAKE_BINARY_DIR}/${RUNTIME_TARGET}.json"
	CONTENT
"{
    \"file_format_version\": \"1.0.0\",
    \"runtime\": {
        \"library_path\": \"$<TARGET_SONAME_FILE_NAME:${RUNTIME_TARGET}>\"
    }
}
")

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../../targets/common
	${CMAKE_CURRENT_SOURCE_DIR}/../../auxiliary
	${CMAKE_CURRENT_SOURCE_DIR}/../../include
	${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
	)

set(SOURCE_FILES
	target.c
	)

# depends on above generated files
add_library(${RUNTIME_TARGET} SHARED
	${MANIFEST_DEV_PATH}
	${MANIFEST_PATH}
	${SOURCE_FILES}
	$<TARGET_OBJECTS:aux_os>
	$<TARGET_OBJECTS:aux_util>
	$<TARGET_OBJECTS:aux_math>
	$<TARGET_OBJECTS:comp>
	$<TARGET_OBJECTS:st_oxr>
	$<TARGET_OBJECTS:st_prober>
	$<TARGET_OBJECTS:target_lists>
	)

target_link_libraries(${RUNTIME_TARGET}
	PUBLIC
	${LIBUVC_LIBRARIES}
	${LIBUSB_LIBRARIES}
	${Vulkan_LIBRARIES}
	${UDEV_LIBRARIES}
	${XCB_LIBRARIES}
	OpenGL::GLX
	)

if(DRIVER_OBJECTS)
	target_sources(${RUNTIME_TARGET} PRIVATE ${DRIVER_OBJECTS})
endif()

if(DRIVER_LIBRARIES)
	target_link_libraries(${RUNTIME_TARGET} PRIVATE ${DRIVER_LIBRARIES})
endif()

if(BUILD_DRIVER_MONTRACK)
	target_sources(${RUNTIME_TARGET} PRIVATE
		$<TARGET_OBJECTS:drv_montrack>
		$<TARGET_OBJECTS:frameserver>
		$<TARGET_OBJECTS:filter>
		$<TARGET_OBJECTS:optical_tracking>)
	if(BUILD_WITH_LIBUVC AND BUILD_WITH_JPEG)
		target_link_libraries(${RUNTIME_TARGET}
			PUBLIC
			${libuvc_LIBRARIES}
			${LIBUSB1_LIBRARIES})
	endif()
	if(BUILD_WITH_JPEG)
		target_link_libraries(${RUNTIME_TARGET}
			PUBLIC
			${JPEG_LIBRARIES})
	endif()
	if(BUILD_WITH_FFMPEG)
		target_link_libraries(${RUNTIME_TARGET}
			PUBLIC
			avformat # TODO find rather than hard code!
			avcodec # TODO find rather than hard code!
			avutil # TODO find rather than hard code!
		)
	endif()
	if(BUILD_WITH_UVBI)
		target_link_libraries(${RUNTIME_TARGET}
			PUBLIC
			uvbi::uvbi_plugin_parts)
	endif()
	target_link_libraries(${RUNTIME_TARGET}
		PUBLIC
		jpeg # TODO find rather than hard code!
		opencv_core
		opencv_calib3d
		opencv_highgui
		opencv_imgproc
		opencv_imgcodecs
		opencv_features2d
		opencv_video
		)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	include(GNUInstallDirs)
	install(TARGETS ${RUNTIME_TARGET}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
	install(FILES "${CMAKE_BINARY_DIR}/${RUNTIME_TARGET}.json" DESTINATION "share/openxr/${XR_API_MAJOR}/")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	# TODO: install target on windows
endif()
