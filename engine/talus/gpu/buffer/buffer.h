#pragma once

#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

#include <cstring>
#include <vector>
#include <stdexcept>
#include <utility>

namespace talus::gpu {

class Buffer {
public:
  Buffer() = default;
  Buffer(
    const Device& device,
    VkDeviceSize size,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties
  );

  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;

  ~Buffer();

  VkBuffer buffer() const noexcept;
  VkDeviceMemory memory() const noexcept;

  void copyTo(const Device& device, VkBuffer dst, VkDeviceSize size) const;
  template<typename T>
  void fill(const std::vector<T>& data);

  template<typename T>
  static Buffer deviceLocal(
    const Device& device,
    const std::vector<T>& data,
    VkBufferUsageFlags usage
  );

private:
  void destroy() noexcept;

  void createBuffer(
    const Device& device,
    VkDeviceSize size,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties
  );

  VkBuffer m_buffer = VK_NULL_HANDLE;
  VkDeviceMemory m_bufferMemory = VK_NULL_HANDLE;
  VkDevice m_device = VK_NULL_HANDLE;
  VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
};

template<typename T>
void Buffer::fill(const std::vector<T>& data) {
  VkDeviceSize bufferSize = sizeof(data[0]) * data.size();

  void* objData;
  if (
    vkMapMemory(m_device, m_bufferMemory, 0, bufferSize, 0, &objData)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map buffer memory");
  }

  memcpy(objData, data.data(), (size_t)bufferSize);
  vkUnmapMemory(m_device, m_bufferMemory);
}

template<typename T>
Buffer Buffer::deviceLocal(
  const Device& device,
  const std::vector<T>& data,
  VkBufferUsageFlags usage
) {
  VkDeviceSize bufferSize = sizeof(T) * data.size();
  Buffer stagingBuffer(
    device,
    bufferSize,
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  stagingBuffer.fill(data);

  Buffer result(
    device,
    bufferSize,
    VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
  );

  stagingBuffer.copyTo(device, result.buffer(), bufferSize);

  return result;
}

}
