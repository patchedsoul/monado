// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Functions used in several tests.
 * @author Christoph Haag <christop.haag@collabora.com>
 */

#include <glad/gl.c>
#include <glad/gl.h>

#define GLFW_EXPOSE_NATIVE_GLX
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#define X11_INCLUDES
#include "tests_common.h"

void error_callback(int error, const char *description) {
  fprintf(stderr, "Error: %s\n", description);
}

GLFWwindow *window;

void init_glx(Display **display, GLXDrawable *w, GLXContext *ctx) {

  if (!glfwInit()) {
    printf("GLFW: Failed to init!\n");
    exit(-1);
  }
  glfwSetErrorCallback(error_callback);

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

  window = glfwCreateWindow(640, 480, "Window Used for Tests", NULL, NULL);
  if (!window) {
    printf("GLFW: Failed to create window!\n");
    exit(-1);
  }

  glfwHideWindow(window);

  glfwMakeContextCurrent(window);

  if (!gladLoadGL(glfwGetProcAddress)) {
    printf("GLAD: Failed to load GL function pointers!\n");
    exit(-1);
  }

  // hope glfw uses GLX
  *display = glfwGetX11Display();
  *w = glfwGetGLXWindow(window);
  *ctx = glfwGetGLXContext(window);
}

void destroy_glx() {
  glfwDestroyWindow(window);
  glfwTerminate();
}
