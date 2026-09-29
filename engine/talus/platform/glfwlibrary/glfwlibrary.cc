#include "glfwlibrary.ih"

namespace talus::platform {

GLFWLibrary::GLFWLibrary() {
  glfwSetErrorCallback(errorCallback);
  if (glfwInit() == GLFW_FALSE) {
    throw std::runtime_error("GLFW not initialized");
  }
}

GLFWLibrary::~GLFWLibrary() {
  glfwTerminate();
}

void GLFWLibrary::errorCallback(int errorCode, const char* description) {
  std::cerr << "GLFW Error [" << errorCode << "]: " << description << '\n';
}

}
