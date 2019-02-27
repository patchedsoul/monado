// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief System test.
 * @author Christoph Haag <christop.haag@collabora.com>
 */

#include "tests_common.h"

START_TEST(system_test) {
  XrInstance instance = XR_NULL_HANDLE;
  {
    const char *extensions[] = {XR_KHR_OPENGL_ENABLE_EXTENSION_NAME};

    XrInstanceCreateInfo ici = {0};
    ici.type = XR_TYPE_INSTANCE_CREATE_INFO;
    ici.enabledExtensionCount = sizeof(extensions) / sizeof(extensions[0]);
    ici.enabledExtensionNames = extensions;
    ck_assert(XR_SUCCESS == oxr_xrCreateInstance(&ici, &instance));
    ck_assert(XR_NULL_HANDLE != instance);
  }

  XrInstanceProperties instance_props = {0};
  instance_props.type = XR_TYPE_INSTANCE_PROPERTIES;
  ck_assert(XR_SUCCESS ==
            oxr_xrGetInstanceProperties(instance, &instance_props));

  XrSystemId system_id;
  {
    XrSystemGetInfo sgi = {0};
    sgi.type = XR_TYPE_SYSTEM_GET_INFO;
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    ck_assert(XR_SUCCESS == oxr_xrGetSystem(instance, &sgi, &system_id));
  }

  XrSystemProperties system_props = {0};
  system_props.type = XR_TYPE_SYSTEM_PROPERTIES;
  ck_assert(XR_SUCCESS ==
            oxr_xrGetSystemProperties(instance, system_id, &system_props));

  ck_assert(system_props.systemName[0] != '\0');
  ck_assert(system_props.systemId != XR_NULL_SYSTEM_ID);
  ck_assert(system_props.vendorId != 0);
  ck_assert_int_gt(system_props.graphicsProperties.maxSwapchainImageHeight, 0);
  ck_assert_int_gt(system_props.graphicsProperties.maxSwapchainImageWidth, 0);
  ck_assert_int_gt(system_props.graphicsProperties.maxLayerCount, 0);
  ck_assert_int_gt(system_props.graphicsProperties.maxViewCount, 0);

  uint32_t view_config_type_count = 0;
  {
    ck_assert(XR_SUCCESS ==
              oxr_xrEnumerateViewConfigurations(instance, system_id, 0,
                                                &view_config_type_count, NULL));
    ck_assert_int_gt(view_config_type_count, 0);
  }

  XrViewConfigurationType view_configuration_types[view_config_type_count];
  {
    ck_assert(XR_SUCCESS == oxr_xrEnumerateViewConfigurations(
                                instance, system_id, view_config_type_count,
                                &view_config_type_count,
                                view_configuration_types));
    ck_assert_int_gt(view_config_type_count, 0);
  }

  XrViewConfigurationType view_type_stereo =
      XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  bool stereo_supported = false;
  for (uint32_t i = 0; i < view_config_type_count; ++i) {
    XrViewConfigurationProperties view_configuration_props = {0};
    view_configuration_props.type = XR_TYPE_VIEW_CONFIGURATION_PROPERTIES;
    ck_assert(XR_SUCCESS ==
              oxr_xrGetViewConfigurationProperties(instance, system_id,
                                                   view_configuration_types[i],
                                                   &view_configuration_props));

    if (view_configuration_types[i] == view_type_stereo) {
      stereo_supported = true;
      ck_assert_int_eq(view_configuration_props.viewConfigurationType,
                       view_type_stereo);
    }
  }
  ck_assert(stereo_supported);

  uint32_t view_config_count = 0;
  ck_assert(XR_SUCCESS == oxr_xrEnumerateViewConfigurationViews(
                              instance, system_id, view_type_stereo, 0,
                              &view_config_count, NULL));
  ck_assert_int_eq(view_config_count, 2);

  XrViewConfigurationView view_configurations[view_config_count];
  ck_assert(XR_SUCCESS == oxr_xrEnumerateViewConfigurationViews(
                              instance, system_id, view_type_stereo,
                              view_config_count, &view_config_count,
                              view_configurations));
  ck_assert_int_eq(view_config_count, 2);

  for (int i = 0; i < 2; i++) {
    ck_assert_int_gt(view_configurations[i].recommendedImageRectHeight, 0);
    ck_assert_int_gt(view_configurations[i].recommendedImageRectWidth, 0);
    ck_assert_int_gt(view_configurations[i].maxImageRectHeight, 0);
    ck_assert_int_gt(view_configurations[i].maxImageRectWidth, 0);
    ck_assert_int_gt(view_configurations[i].recommendedSwapchainSampleCount, 0);
    ck_assert_int_gt(view_configurations[i].maxSwapchainSampleCount, 0);
  }

  XrGraphicsBindingOpenGLXlibKHR graphics_binding_gl = {0};
  graphics_binding_gl.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR;

  init_glx(&graphics_binding_gl.xDisplay, &graphics_binding_gl.glxDrawable,
           &graphics_binding_gl.glxContext);

  XrSessionCreateInfo sci = {0};
  sci.type = XR_TYPE_SESSION_CREATE_INFO;
  sci.next = &graphics_binding_gl;
  sci.systemId = system_id;

  XrSession session;
  ck_assert(XR_SUCCESS == oxr_xrCreateSession(instance, &sci, &session));

  ck_assert(XR_SUCCESS == oxr_xrDestroySession(session));
  ck_assert(XR_SUCCESS == oxr_xrDestroyInstance(instance));

  destroy_glx();
}
END_TEST

Suite *system_test_suite(void) {
  Suite *s = suite_create("Monado System Test Suite");
  TCase *system_test_case = tcase_create("System Test");

  tcase_add_test(system_test_case, system_test);

  suite_add_tcase(s, system_test_case);

  return s;
}
