#include "context.ih"

namespace talus::gpu {

Context::Context(const WindowSystem& windowSystem)
    : m_windowSystem(&windowSystem),
      m_instance(validationLayers, m_windowSystem->instanceExtensions()),
      m_surface(m_instance.handle(), *m_windowSystem),
      m_device(m_instance.handle(), m_surface.handle()) {}

void Context::retarget(const WindowSystem& windowSystem) {
  m_device.waitIdle();

  m_windowSystem = &windowSystem;

  m_surface.recreate(*m_windowSystem);
}

const WindowSystem& Context::windowSystem() const noexcept {
  return *m_windowSystem;
}

const Instance& Context::instance() const noexcept {
  return m_instance;
}

const Surface& Context::surface() const noexcept {
  return m_surface;
}

const Device& Context::device() const noexcept {
  return m_device;
}

Device& Context::device() {
  return m_device;
}

}
