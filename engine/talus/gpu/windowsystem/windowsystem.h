#pragma once

#include <vulkan/vulkan.h>

#include <vector>

namespace talus::gpu {

class WindowSystem {
public:
  virtual ~WindowSystem() = default;

  virtual std::vector<const char*> instanceExtensions() const = 0;

  virtual VkSurfaceKHR createSurface(VkInstance instance) const = 0;

  virtual VkExtent2D framebufferExtent() const = 0;

  virtual void waitForNonZeroExtent() const = 0;
};

}
