# Copyright 2019-2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

######
# Create a small SDL2 based GUI for Monado.

set(SOURCE_FILES
	gui_sdl2.c
	gui_sdl2_imgui.c
	gui_sdl2_main.c
	gui_sdl2_prober.c
	)

add_executable(gui
	${SOURCE_FILES}
	)

set_target_properties(gui PROPERTIES
	OUTPUT_NAME monado-gui
	PREFIX ""
	)

target_link_libraries(gui PRIVATE
	aux_os
	aux_ogl
	aux_util
	aux_math
	st_gui
	st_prober
	target_lists
	imgui_impl_sdl
	)
target_include_directories(gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
