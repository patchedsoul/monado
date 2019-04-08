# Copyright 2018-2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

add_executable(tests
	tests_common.c
	tests_main.c
	tests_unit.c
	tests_system.c)

target_include_directories(tests PRIVATE
	"${PROJECT_SOURCE_DIR}/src"
	"${PROJECT_SOURCE_DIR}/src/external"
	"${PROJECT_SOURCE_DIR}/src/xrt/include"
	"${PROJECT_SOURCE_DIR}/src/xrt/state_trackers"
	"${PROJECT_SOURCE_DIR}/src/xrt/auxiliary"
)

target_link_libraries(tests ${RUNTIME_TARGET} libcheck::check OpenGL::GL glfw X11)

# TODO: adjust the runner so we can individually run test cases here.
add_test(NAME monado_tests COMMAND tests)
