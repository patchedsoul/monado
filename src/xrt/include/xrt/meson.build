# Copyright 2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0


#####
# Drivers config
#

drivers_conf = configuration_data()
foreach driver : drivers
	drivers_conf.set('XRT_BUILD_DRIVER_' + driver.to_upper(), true)
endforeach

xrt_config_drivers_h = configure_file(
	output: 'xrt_config_drivers.h',
	configuration: drivers_conf,
)

xrt_config_drivers = declare_dependency(
	sources: xrt_config_drivers_h,
	include_directories: xrt_include,
)


#####
# Have config
#

have_conf = configuration_data()
if udev.found()
	have_conf.set('XRT_HAVE_LIBUDEV', true)
endif

if libusb.found()
	have_conf.set('XRT_HAVE_LIBUSB', true)
endif

if opencv.found()
	have_conf.set('XRT_HAVE_OPENCV', true)
endif

if libjpeg.found()
	have_conf.set('XRT_HAVE_JPEG', true)
endif

if libuvc.found()
	have_conf.set('XRT_HAVE_LIBUVC', true)
endif

if avcodec.found()
	have_conf.set('XRT_HAVE_FFMPEG', true)
endif

if sdl2.found()
	have_conf.set('XRT_HAVE_SDL2', true)
endif

xrt_config_have_h = configure_file(
	output: 'xrt_config_have.h',
	configuration: have_conf,
)

xrt_config_have = declare_dependency(
	sources: xrt_config_have_h,
	include_directories: xrt_include,
)


#####
# Vulkan config
#

vulkan_conf = configuration_data()
# Vulkan flags for the shared Vulkan code.
if build_wayland
	vulkan_conf.set('VK_USE_PLATFORM_WAYLAND_KHR', true)
endif
if build_xcb
	vulkan_conf.set('VK_USE_PLATFORM_XCB_KHR', true)
endif
if build_xcb_xrandr_direct
	vulkan_conf.set('VK_USE_PLATFORM_XLIB_XRANDR_EXT', true)
endif

xrt_config_vulkan_h = configure_file(
	output: 'xrt_config_vulkan.h',
	configuration: vulkan_conf,
)

xrt_config_vulkan = declare_dependency(
	sources: xrt_config_vulkan_h,
	include_directories: xrt_include,
)
