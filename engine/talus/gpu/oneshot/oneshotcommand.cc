#include "oneshotcommand.ih"

namespace talus::gpu {

OneShotCommand::~OneShotCommand() {}

VkCommandBuffer OneShotCommand::cmdBuffer() const noexcept {
  return m_commandBuffer;
}

void OneShotCommand::start(const Device& device) {
  VkCommandPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
  poolInfo.queueFamilyIndex = device.graphicsFamilyIndex();

  if (
    vkCreateCommandPool(device.vkDevice(), &poolInfo, nullptr, &m_commandPool)
    != VK_SUCCESS
  ) {
    throw std::runtime_error(
      "Failed to create command pool while copying buffer"
    );
  }

  VkCommandBufferAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandPool = m_commandPool;
  allocInfo.commandBufferCount = 1;

  if (
    vkAllocateCommandBuffers(device.vkDevice(), &allocInfo, &m_commandBuffer)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to allocate one shot command buffer");
  }

  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

  if (vkBeginCommandBuffer(m_commandBuffer, &beginInfo) != VK_SUCCESS) {
    throw std::runtime_error("Failed to begin one shot command buffer");
  }
}

void OneShotCommand::submit(const Device& device) const {
  if (vkEndCommandBuffer(m_commandBuffer) != VK_SUCCESS) {
    throw std::runtime_error("Failed to end one shot command buffer");
  }

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &m_commandBuffer;

  if (
    vkQueueSubmit(device.graphicsQueue(), 1, &submitInfo, VK_NULL_HANDLE)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to submit one shot command buffer");
  }

  if (vkQueueWaitIdle(device.graphicsQueue()) != VK_SUCCESS) {
    throw std::runtime_error("Failed waiting for the one shot command buffer");
  }

  vkFreeCommandBuffers(device.vkDevice(), m_commandPool, 1, &m_commandBuffer);
  vkDestroyCommandPool(device.vkDevice(), m_commandPool, nullptr);
}

}
