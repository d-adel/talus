#pragma once

#include <talus/gpu/windowsystem/windowsystem.h>

#include <vector>

namespace talus::capi {

class Win32WindowSystem final : public talus::gpu::WindowSystem {
public:
  explicit Win32WindowSystem(void* windowHandle);

  Win32WindowSystem(const Win32WindowSystem&) = delete;
  Win32WindowSystem& operator=(const Win32WindowSystem&) = delete;
  Win32WindowSystem(Win32WindowSystem&&) = delete;
  Win32WindowSystem& operator=(Win32WindowSystem&&) = delete;

  std::vector<const char*> instanceExtensions() const override;
  VkSurfaceKHR createSurface(VkInstance instance) const override;
  VkExtent2D framebufferExtent() const override;
  void waitForNonZeroExtent() const override;

private:
  void* m_windowHandle;
};

}
