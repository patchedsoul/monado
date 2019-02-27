
pkg_check_modules(CHECK check)

add_executable(tests
	tests_common.c
	tests_main.c
	tests_unit.c
	tests_system.c)

target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/src/external")
target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/src/xrt/include")
target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/src/xrt/state_trackers")
target_include_directories(tests PRIVATE "${CMAKE_SOURCE_DIR}/src/xrt/auxiliary")

find_package(glfw3 3.2 REQUIRED)
find_package(OpenGL REQUIRED)
target_include_directories(tests PRIVATE ${CHECK_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})

target_link_libraries(tests ${RUNTIME_TARGET} ${CHECK_LIBRARIES} ${OPENGL_gl_LIBRARY} glfw X11)

