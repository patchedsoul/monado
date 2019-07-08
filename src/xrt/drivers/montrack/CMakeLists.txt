set (MONTRACK_SOURCE_FILES
	mt_device.c
	mt_device.h
	mt_interface.h
	mt_prober.c
	mt_events.h
	mt_framequeue.h
	mt_framequeue.c
)


add_subdirectory(frameservers)
add_subdirectory(filters)
add_subdirectory(optical_tracking)


# Use OBJECT to not create a archive, since it just gets in the way.
add_library(drv_montrack OBJECT ${MONTRACK_SOURCE_FILES}
	$<TARGET_OBJECTS:frameserver>
	$<TARGET_OBJECTS:filter>
	$<TARGET_OBJECTS:optical_tracking>
	)

set_property(TARGET drv_montrack PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(drv_montrack SYSTEM
	PRIVATE frameservers
	PRIVATE optical_tracking
	PRIVATE filters)

target_link_libraries (drv_montrack frameserver filter optical_tracking)
