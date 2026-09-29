#pragma once

#include <GLFW/glfw3.h>

namespace talus::platform {

class GLFWLibrary {
public:
  GLFWLibrary();
  ~GLFWLibrary();

  GLFWLibrary(const GLFWLibrary&) = delete;
  GLFWLibrary& operator=(const GLFWLibrary&) = delete;
  GLFWLibrary(GLFWLibrary&&) = delete;
  GLFWLibrary& operator=(GLFWLibrary&&) = delete;

private:
  static void errorCallback(int errorCode, const char* description);
};

}
