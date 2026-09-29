#pragma once

#include <talus/gpu/instance/instance.h>
#include <talus/gpu/windowsystem/windowsystem.h>
#include <talus/gpu/surface/surface.h>
#include <talus/gpu/device/device.h>

namespace talus::gpu {

const std::vector<char const*> validationLayers = {
  "VK_LAYER_KHRONOS_validation"
};

class Context {
public:
  explicit Context(const WindowSystem& windowSystem);
  ~Context() = default;

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;
  Context(Context&&) = delete;
  Context& operator=(Context&&) = delete;

  void retarget(const WindowSystem& windowSystem);

  const WindowSystem& windowSystem() const noexcept;

  const Instance& instance() const noexcept;
  const Surface& surface() const noexcept;
  const Device& device() const noexcept;
  Device& device();

private:
  const WindowSystem* m_windowSystem;
  Instance m_instance;
  Surface m_surface;
  Device m_device;
};

}
