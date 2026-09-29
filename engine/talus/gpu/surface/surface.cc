#include "surface.ih"

namespace talus::gpu {

Surface::Surface(const VkInstance& instance, const WindowSystem& windowSystem)
    : m_instance(instance), m_surface(windowSystem.createSurface(instance)) {
  if (m_surface == VK_NULL_HANDLE) {
    throw std::runtime_error("failed to create window surface!");
  }
}

Surface::~Surface() {
  vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
}

void Surface::recreate(const WindowSystem& windowSystem) {
  const VkSurfaceKHR replacement = windowSystem.createSurface(m_instance);

  if (replacement == VK_NULL_HANDLE) {
    throw std::runtime_error("failed to create window surface!");
  }

  vkDestroySurfaceKHR(m_instance, m_surface, nullptr);

  m_surface = replacement;
}

const VkSurfaceKHR& Surface::handle() const noexcept {
  return m_surface;
}

}
