#pragma once

#include <GLFW/glfw3.h>

namespace talus::platform {

constexpr uint32_t WIDTH = 1280;
constexpr uint32_t HEIGHT = 720;

class Window {
public:
  Window();
  Window(uint32_t width, uint32_t height);
  Window(Window const&) = delete;
  Window& operator=(Window const&) = delete;
  ~Window();

  GLFWwindow* handle() const noexcept;
  [[nodiscard]]
  bool takeResized() noexcept;
  bool focused() const noexcept;

  void resize();
  void focus(bool focused);

private:
  void initialize();
  static void framebufferSizeCallback(
    GLFWwindow* window,
    int width,
    int height
  );
  static void windowFocusCallback(GLFWwindow* window, int focused);

  GLFWwindow* m_window = nullptr;
  uint32_t m_width;
  uint32_t m_height;

  bool m_resized;
  bool m_windowFocused;
};

}
