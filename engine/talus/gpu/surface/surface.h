#pragma once

#include <talus/gpu/windowsystem/windowsystem.h>

#include <vulkan/vulkan.h>

namespace talus::gpu {

class Surface {
public:
  Surface(const VkInstance& instance, const WindowSystem& windowSystem);
  Surface(Surface const&) = delete;
  Surface& operator=(Surface const&) = delete;
  ~Surface();

  void recreate(const WindowSystem& windowSystem);

  const VkSurfaceKHR& handle() const noexcept;

private:
  VkInstance m_instance = VK_NULL_HANDLE;
  VkSurfaceKHR m_surface = VK_NULL_HANDLE;
};

}
