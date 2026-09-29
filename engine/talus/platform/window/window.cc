#include "window.ih"

namespace talus::platform {

Window::Window()
    : m_width(WIDTH),
      m_height(HEIGHT),
      m_resized(false),
      m_windowFocused(true) {
  initialize();
}

Window::Window(uint32_t width, uint32_t height)
    : m_width(width),
      m_height(height),
      m_resized(false),
      m_windowFocused(true) {
  initialize();
}

Window::~Window() {
  glfwDestroyWindow(m_window);
}

GLFWwindow* Window::handle() const noexcept {
  return m_window;
}

bool Window::focused() const noexcept {
  return m_windowFocused;
}

void Window::resize() {
  m_resized = true;
}

void Window::focus(bool focused) {
  m_windowFocused = focused;
}

bool Window::takeResized() noexcept {
  return std::exchange(m_resized, false);
}

void Window::initialize() {
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  m_window =
    glfwCreateWindow(m_width, m_height, "talus", nullptr, nullptr);

  if (m_window == nullptr) {
    throw std::runtime_error("Window could not be created");
  }

  glfwSetWindowUserPointer(m_window, this);
  glfwSetFramebufferSizeCallback(m_window, framebufferSizeCallback);
  glfwSetWindowFocusCallback(m_window, windowFocusCallback);
}

void Window::framebufferSizeCallback(
  GLFWwindow* window,
  int width,
  int height
) {
  (void)width;
  (void)height;
  Window* windowObj =
    reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
  if (windowObj) {
    windowObj->resize();
  }
}

void Window::windowFocusCallback(GLFWwindow* window, int focused) {
  Window* windowObj =
    reinterpret_cast<Window*>(glfwGetWindowUserPointer(window));
  if (windowObj) {
    windowObj->focus(focused);
  }
}

}
