#include "buffer.ih"

namespace talus::gpu {

Buffer::Buffer(
  const Device& device,
  VkDeviceSize size,
  VkBufferUsageFlags usage,
  VkMemoryPropertyFlags properties
)
    : m_device(device.vkDevice()), m_physicalDevice(device.physicalDevice()) {
  createBuffer(device, size, usage, properties);
}

Buffer::Buffer(Buffer&& other) noexcept
    : m_buffer(std::exchange(other.m_buffer, VK_NULL_HANDLE)),
      m_bufferMemory(std::exchange(other.m_bufferMemory, VK_NULL_HANDLE)),
      m_device(std::exchange(other.m_device, VK_NULL_HANDLE)),
      m_physicalDevice(std::exchange(other.m_physicalDevice, VK_NULL_HANDLE)) {}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  destroy();

  m_buffer = std::exchange(other.m_buffer, VK_NULL_HANDLE);

  m_bufferMemory = std::exchange(other.m_bufferMemory, VK_NULL_HANDLE);

  m_device = std::exchange(other.m_device, VK_NULL_HANDLE);

  m_physicalDevice = std::exchange(other.m_physicalDevice, VK_NULL_HANDLE);

  return *this;
}

Buffer::~Buffer() {
  destroy();
}

void Buffer::destroy() noexcept {
  if (m_device == VK_NULL_HANDLE) {
    return;
  }

  if (m_buffer != VK_NULL_HANDLE) {
    vkDestroyBuffer(m_device, m_buffer, nullptr);
    m_buffer = VK_NULL_HANDLE;
  }

  if (m_bufferMemory != VK_NULL_HANDLE) {
    vkFreeMemory(m_device, m_bufferMemory, nullptr);
    m_bufferMemory = VK_NULL_HANDLE;
  }
}

VkBuffer Buffer::buffer() const noexcept {
  return m_buffer;
}

VkDeviceMemory Buffer::memory() const noexcept {
  return m_bufferMemory;
}

void Buffer::createBuffer(
  const Device& device,
  VkDeviceSize size,
  VkBufferUsageFlags usage,
  VkMemoryPropertyFlags properties
) {
  VkBufferCreateInfo bufferInfo{};
  bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bufferInfo.size = size;
  bufferInfo.usage = usage;
  bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &m_buffer) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create buffer");
  }

  VkMemoryRequirements memRequirements;
  vkGetBufferMemoryRequirements(m_device, m_buffer, &memRequirements);

  VkMemoryAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  allocInfo.memoryTypeIndex =
    device.findMemoryType(memRequirements.memoryTypeBits, properties);

  if (
    vkAllocateMemory(m_device, &allocInfo, nullptr, &m_bufferMemory)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("failed to allocate buffer memory!");
  }

  if (vkBindBufferMemory(m_device, m_buffer, m_bufferMemory, 0) != VK_SUCCESS) {
    throw std::runtime_error("Failed to bind buffer memory");
  }
}

void Buffer::copyTo(
  const Device& device,
  VkBuffer dst,
  VkDeviceSize size
) const {
  OneShotCommand cmd;
  cmd.start(device);

  VkBufferCopy copyRegion{};
  copyRegion.size = size;
  vkCmdCopyBuffer(cmd.cmdBuffer(), m_buffer, dst, 1, &copyRegion);

  cmd.submit(device);
}

}
