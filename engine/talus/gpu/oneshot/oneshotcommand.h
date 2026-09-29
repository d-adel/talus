#pragma once

#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

namespace talus::gpu {

class OneShotCommand {
public:
  OneShotCommand() = default;
  ~OneShotCommand();

  void start(const Device& device);
  void submit(const Device& device) const;

  VkCommandBuffer cmdBuffer() const noexcept;

private:
  VkCommandPool m_commandPool;
  VkCommandBuffer m_commandBuffer;
};

}
