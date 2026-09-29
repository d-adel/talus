#include "uploader.ih"

namespace talus::gpu {

Uploader::Uploader(const Device& device) : m_device(device) {
  VkCommandPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;

  poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  poolInfo.queueFamilyIndex = device.graphicsFamilyIndex();

  if (
    vkCreateCommandPool(device.vkDevice(), &poolInfo, nullptr, &m_pool)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create the uploader command pool");
  }
}

Uploader::~Uploader() {
  if (m_recording) {
    vkEndCommandBuffer(m_open.commandBuffer);
    release(m_open);
  }

  if (!m_inFlight.empty()) {
    vkQueueWaitIdle(m_device.graphicsQueue());

    for (Batch& batch : m_inFlight) {
      release(batch);
    }

    m_inFlight.clear();
  }

  if (m_pool != VK_NULL_HANDLE) {
    vkDestroyCommandPool(m_device.vkDevice(), m_pool, nullptr);
    m_pool = VK_NULL_HANDLE;
  }
}

void Uploader::release(Batch& batch) noexcept {
  batch.staging.clear();

  if (batch.commandBuffer != VK_NULL_HANDLE) {
    vkFreeCommandBuffers(m_device.vkDevice(), m_pool, 1, &batch.commandBuffer);
    batch.commandBuffer = VK_NULL_HANDLE;
  }

  if (batch.fence != VK_NULL_HANDLE) {
    vkDestroyFence(m_device.vkDevice(), batch.fence, nullptr);
    batch.fence = VK_NULL_HANDLE;
  }
}

VkCommandBuffer Uploader::begin() {
  if (m_recording) {
    return m_open.commandBuffer;
  }

  VkCommandBufferAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandPool = m_pool;
  allocInfo.commandBufferCount = 1;

  if (
    vkAllocateCommandBuffers(
      m_device.vkDevice(),
      &allocInfo,
      &m_open.commandBuffer
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to allocate an upload command buffer");
  }

  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

  if (vkBeginCommandBuffer(m_open.commandBuffer, &beginInfo) != VK_SUCCESS) {
    throw std::runtime_error("Failed to begin an upload command buffer");
  }

  m_recording = true;

  return m_open.commandBuffer;
}

void Uploader::keep(Buffer&& staging) {
  m_open.staging.push_back(std::move(staging));
}

bool Uploader::pending() const noexcept {
  return m_recording;
}

void Uploader::submit() {
  if (!m_recording) {
    return;
  }

  if (vkEndCommandBuffer(m_open.commandBuffer) != VK_SUCCESS) {
    throw std::runtime_error("Failed to end an upload command buffer");
  }

  VkFenceCreateInfo fenceInfo{};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;

  if (
    vkCreateFence(m_device.vkDevice(), &fenceInfo, nullptr, &m_open.fence)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create an upload fence");
  }

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &m_open.commandBuffer;

  if (
    vkQueueSubmit(m_device.graphicsQueue(), 1, &submitInfo, m_open.fence)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to submit an upload command buffer");
  }

  m_inFlight.push_back(std::move(m_open));

  m_open = Batch{};
  m_recording = false;
}

void Uploader::retire() {
  size_t kept = 0;

  for (size_t i = 0; i != m_inFlight.size(); ++i) {
    Batch& batch = m_inFlight[i];

    if (vkGetFenceStatus(m_device.vkDevice(), batch.fence) == VK_SUCCESS) {
      release(batch);

      continue;
    }

    if (kept != i) {
      m_inFlight[kept] = std::move(batch);
    }

    ++kept;
  }

  m_inFlight.resize(kept);
}

}
