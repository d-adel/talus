#pragma once

#include <talus/gpu/device/device.h>
#include <talus/gpu/buffer/buffer.h>

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

namespace talus::gpu {

class DescriptorGroup {
public:
  enum class BufferUpdate {
    Static,
    PerFrame
  };

  struct BufferBinding {
    uint32_t binding = 0;
    VkDeviceSize size = 0;
    VkShaderStageFlags stages = 0;
    BufferUpdate update = BufferUpdate::PerFrame;
    VkDescriptorType type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  };

  struct SampledImageBinding {
    uint32_t binding = 0;
    VkShaderStageFlags stages = 0;
    std::vector<VkImageView> elements{};
    uint32_t capacity = 0;
  };

  struct StorageImageBinding {
    uint32_t binding = 0;
    VkShaderStageFlags stages = 0;
    std::vector<VkImageView> elements{};
    uint32_t capacity = 0;
  };

  struct SamplerBinding {
    uint32_t binding = 0;
    VkShaderStageFlags stages = 0;
    std::vector<VkSampler> elements{};
    uint32_t capacity = 0;
  };

  struct DeviceBufferBinding {
    uint32_t binding = 0;
    VkShaderStageFlags stages = 0;
  };

  DescriptorGroup(
    const Device& device,
    uint32_t frameCount,
    std::vector<BufferBinding> buffers,
    std::vector<SampledImageBinding> sampledImages = {},
    std::vector<SamplerBinding> samplers = {},
    std::vector<StorageImageBinding> storageImages = {},
    std::vector<DeviceBufferBinding> deviceBuffers = {}
  );

  DescriptorGroup(const DescriptorGroup&) = delete;
  DescriptorGroup& operator=(const DescriptorGroup&) = delete;
  DescriptorGroup(DescriptorGroup&&) = delete;
  DescriptorGroup& operator=(DescriptorGroup&&) = delete;

  ~DescriptorGroup();

  const VkDescriptorSetLayout& setLayout() const noexcept;
  const VkDescriptorSet& descriptorSet(uint32_t frame) const noexcept;

  void write(uint32_t binding, uint32_t frame, const void* data, size_t size);
  void writeImage(
    uint32_t binding,
    uint32_t index,
    VkImageView imageView,
    VkImageLayout layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  );

  void writeStorageImage(
    uint32_t binding,
    uint32_t index,
    VkImageView imageView
  );

  void writeBuffer(uint32_t binding, VkBuffer buffer, VkDeviceSize size);

  [[nodiscard]] VkDeviceSize capacity(uint32_t binding, uint32_t frame) const;

  void resize(uint32_t binding, uint32_t frame, VkDeviceSize size);

private:
  struct Slot {
    uint32_t binding = 0;
    BufferUpdate update = BufferUpdate::PerFrame;
    VkDeviceSize size = 0;
    VkDescriptorType type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    std::vector<Buffer> buffers;
    std::vector<void*> mapped;
    std::vector<VkDeviceSize> sizes;
  };

  [[nodiscard]] Slot& slotFor(uint32_t binding);

  [[nodiscard]] const Slot& slotFor(uint32_t binding) const;

  void mapInto(Slot& slot, uint32_t index);

  struct ImageSlot {
    uint32_t binding = 0;
    VkDescriptorType type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    uint32_t capacity = 0;
  };

  void createSetLayout(
    const std::vector<BufferBinding>& buffers,
    const std::vector<SampledImageBinding>& sampledImages,
    const std::vector<SamplerBinding>& samplers,
    const std::vector<StorageImageBinding>& storageImages,
    const std::vector<DeviceBufferBinding>& deviceBuffers
  );

  void createBuffers(
    const Device& device,
    const std::vector<BufferBinding>& buffers
  );
  void createPool(
    const std::vector<SampledImageBinding>& sampledImages,
    const std::vector<SamplerBinding>& samplers,
    const std::vector<StorageImageBinding>& storageImages,
    const std::vector<DeviceBufferBinding>& deviceBuffers
  );
  void createSets(
    const std::vector<SampledImageBinding>& sampledImages,
    const std::vector<SamplerBinding>& samplers,
    const std::vector<StorageImageBinding>& storageImages
  );

  const Device& m_owner;
  VkDevice m_device = VK_NULL_HANDLE;
  uint32_t m_frameCount = 0;

  VkDescriptorSetLayout m_setLayout = VK_NULL_HANDLE;
  VkDescriptorPool m_pool = VK_NULL_HANDLE;

  bool m_updateAfterBind = false;

  std::vector<VkDescriptorSet> m_sets;
  std::vector<Slot> m_slots;
  std::vector<ImageSlot> m_imageSlots;
  std::vector<uint32_t> m_deviceBufferSlots;
};

}
