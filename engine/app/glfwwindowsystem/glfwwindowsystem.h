#pragma once

#include <vulkan/vulkan.h>

#include <talus/gpu/windowsystem/windowsystem.h>
#include <talus/platform/window/window.h>

#include <vector>

class GlfwWindowSystem final : public talus::gpu::WindowSystem {
public:
  explicit GlfwWindowSystem(const talus::platform::Window& window);

  GlfwWindowSystem(const GlfwWindowSystem&) = delete;
  GlfwWindowSystem& operator=(const GlfwWindowSystem&) = delete;
  GlfwWindowSystem(GlfwWindowSystem&&) = delete;
  GlfwWindowSystem& operator=(GlfwWindowSystem&&) = delete;

  std::vector<const char*> instanceExtensions() const override;
  VkSurfaceKHR createSurface(VkInstance instance) const override;
  VkExtent2D framebufferExtent() const override;
  void waitForNonZeroExtent() const override;

private:
  const talus::platform::Window& m_window;
};
