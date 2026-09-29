#include "descriptorgroup.ih"

namespace talus::gpu {

DescriptorGroup::DescriptorGroup(
  const Device& device,
  uint32_t frameCount,
  std::vector<BufferBinding> buffers,
  std::vector<SampledImageBinding> sampledImages,
  std::vector<SamplerBinding> samplers,
  std::vector<StorageImageBinding> storageImages,
  std::vector<DeviceBufferBinding> deviceBuffers
)
    : m_owner(device), m_device(device.vkDevice()), m_frameCount(frameCount) {
  for (const auto& storageImage : storageImages) {
    if (storageImage.elements.size() > storageImage.capacity) {
      throw std::runtime_error(
        "Number of storage image bindings larger than the current capacity"
      );
    }
  }

  for (const auto& sampledImage : sampledImages) {
    if (sampledImage.elements.size() > sampledImage.capacity) {
      throw std::runtime_error(
        "Number of sampled image bindings larger than the current capacity"
      );
    }
  }

  for (const auto& sampler : samplers) {
    if (sampler.elements.size() > sampler.capacity) {
      throw std::runtime_error(
        "Number of sampler bindings larger than the current capacity"
      );
    }
  }

  createSetLayout(
    buffers,
    sampledImages,
    samplers,
    storageImages,
    deviceBuffers
  );
  createBuffers(device, buffers);

  if (
    buffers.empty() && sampledImages.empty() && samplers.empty()
    && storageImages.empty() && deviceBuffers.empty()
  ) {
    throw std::runtime_error("A descriptor group needs at least one binding");
  }

  createPool(sampledImages, samplers, storageImages, deviceBuffers);
  createSets(sampledImages, samplers, storageImages);
}

DescriptorGroup::~DescriptorGroup() {
  vkDestroyDescriptorPool(m_device, m_pool, nullptr);
  vkDestroyDescriptorSetLayout(m_device, m_setLayout, nullptr);
}

const VkDescriptorSetLayout& DescriptorGroup::setLayout() const noexcept {
  return m_setLayout;
}

const VkDescriptorSet& DescriptorGroup::descriptorSet(
  uint32_t frame
) const noexcept {
  return m_sets.at(frame);
}

DescriptorGroup::Slot& DescriptorGroup::slotFor(uint32_t binding) {
  for (Slot& slot : m_slots) {
    if (slot.binding == binding) {
      return slot;
    }
  }

  throw std::runtime_error("Write to a binding this group does not own");
}

const DescriptorGroup::Slot& DescriptorGroup::slotFor(uint32_t binding) const {
  for (const Slot& slot : m_slots) {
    if (slot.binding == binding) {
      return slot;
    }
  }

  throw std::runtime_error("Write to a binding this group does not own");
}

void DescriptorGroup::write(
  uint32_t binding,
  uint32_t frame,
  const void* data,
  size_t size
) {
  Slot& slot = slotFor(binding);

  const uint32_t index = slot.update == BufferUpdate::Static ? 0u : frame;

  if (size > slot.sizes.at(index)) {
    throw std::runtime_error("Write larger than the binding it targets");
  }

  memcpy(slot.mapped.at(index), data, size);
}

VkDeviceSize DescriptorGroup::capacity(uint32_t binding, uint32_t frame) const {
  const Slot& slot = slotFor(binding);

  return slot.sizes.at(slot.update == BufferUpdate::Static ? 0u : frame);
}

void DescriptorGroup::mapInto(Slot& slot, uint32_t index) {
  if (
    vkMapMemory(
      m_device,
      slot.buffers[index].memory(),
      0,
      slot.sizes[index],
      0,
      &slot.mapped[index]
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map uniform buffer memory");
  }
}

void DescriptorGroup::resize(
  uint32_t binding,
  uint32_t frame,
  VkDeviceSize size
) {
  Slot& slot = slotFor(binding);

  if (slot.update != BufferUpdate::PerFrame) {
    throw std::runtime_error("Only a per-frame binding can be resized");
  }

  const VkBufferUsageFlags usage =
    slot.type == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
      ? VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
      : VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

  slot.buffers.at(frame) = Buffer(
    m_owner,
    size,
    usage,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );
  slot.sizes.at(frame) = size;

  mapInto(slot, frame);

  VkDescriptorBufferInfo bufferInfo{};
  bufferInfo.buffer = slot.buffers[frame].buffer();
  bufferInfo.offset = 0;
  bufferInfo.range = size;

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.dstSet = m_sets.at(frame);
  write.dstBinding = binding;
  write.dstArrayElement = 0;
  write.descriptorType = slot.type;
  write.descriptorCount = 1;
  write.pBufferInfo = &bufferInfo;

  vkUpdateDescriptorSets(m_device, 1, &write, 0, nullptr);
}

void DescriptorGroup::writeImage(
  uint32_t binding,
  uint32_t index,
  VkImageView imageView,
  VkImageLayout layout
) {
  std::vector<VkWriteDescriptorSet> writes;
  writes.reserve(m_sets.size());

  auto it = std::find_if(
    m_imageSlots.begin(),
    m_imageSlots.end(),
    [binding](const ImageSlot& slot) { return slot.binding == binding; }
  );

  if (it == m_imageSlots.end()) {
    throw std::runtime_error("Failed to write image, non-existing binding");
  }

  if (index >= it->capacity) {
    throw std::runtime_error("Failed to write to image, index out of bounds");
  }

  if (it->type != VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE) {
    throw std::runtime_error(
      "Failed to write image, binding can only be a sampled image"
    );
  }

  VkDescriptorImageInfo imageInfo{};
  imageInfo.imageLayout = layout;
  imageInfo.imageView = imageView;

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;

  write.dstBinding = binding;
  write.dstArrayElement = index;
  write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
  write.descriptorCount = 1;
  write.pImageInfo = &imageInfo;

  for (auto set : m_sets) {
    write.dstSet = set;
    writes.push_back(write);
  }

  vkUpdateDescriptorSets(
    m_device,
    static_cast<uint32_t>(writes.size()),
    writes.data(),
    0,
    nullptr
  );
}

void DescriptorGroup::writeBuffer(
  uint32_t binding,
  VkBuffer buffer,
  VkDeviceSize size
) {
  const auto it =
    std::find(m_deviceBufferSlots.begin(), m_deviceBufferSlots.end(), binding);

  if (it == m_deviceBufferSlots.end()) {
    throw std::runtime_error(
      "Failed to write buffer, binding is not a device buffer"
    );
  }

  VkDescriptorBufferInfo bufferInfo{};
  bufferInfo.buffer = buffer;
  bufferInfo.offset = 0;
  bufferInfo.range = size;

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;

  write.dstBinding = binding;
  write.dstArrayElement = 0;
  write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  write.descriptorCount = 1;
  write.pBufferInfo = &bufferInfo;

  std::vector<VkWriteDescriptorSet> writes;
  writes.reserve(m_sets.size());

  for (auto set : m_sets) {
    write.dstSet = set;
    writes.push_back(write);
  }

  vkUpdateDescriptorSets(
    m_device,
    static_cast<uint32_t>(writes.size()),
    writes.data(),
    0,
    nullptr
  );
}

void DescriptorGroup::writeStorageImage(
  uint32_t binding,
  uint32_t index,
  VkImageView imageView
) {
  std::vector<VkWriteDescriptorSet> writes;
  writes.reserve(m_sets.size());

  auto it = std::find_if(
    m_imageSlots.begin(),
    m_imageSlots.end(),
    [binding](const ImageSlot& slot) { return slot.binding == binding; }
  );

  if (it == m_imageSlots.end()) {
    throw std::runtime_error(
      "Failed to write storage image, non-existing binding"
    );
  }

  if (index >= it->capacity) {
    throw std::runtime_error(
      "Failed to write to storage image, index out of bounds"
    );
  }

  if (it->type != VK_DESCRIPTOR_TYPE_STORAGE_IMAGE) {
    throw std::runtime_error(
      "Failed to write storage image, binding is not a storage image"
    );
  }

  VkDescriptorImageInfo imageInfo{};
  imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
  imageInfo.imageView = imageView;

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;

  write.dstBinding = binding;
  write.dstArrayElement = index;
  write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  write.descriptorCount = 1;
  write.pImageInfo = &imageInfo;

  for (auto set : m_sets) {
    write.dstSet = set;
    writes.push_back(write);
  }

  vkUpdateDescriptorSets(
    m_device,
    static_cast<uint32_t>(writes.size()),
    writes.data(),
    0,
    nullptr
  );
}

void DescriptorGroup::createSetLayout(
  const std::vector<BufferBinding>& buffers,
  const std::vector<SampledImageBinding>& sampledImages,
  const std::vector<SamplerBinding>& samplers,
  const std::vector<StorageImageBinding>& storageImages,
  const std::vector<DeviceBufferBinding>& deviceBuffers
) {
  const size_t bindingCount = buffers.size() + sampledImages.size()
                              + samplers.size() + storageImages.size()
                              + deviceBuffers.size();

  std::vector<VkDescriptorSetLayoutBinding> bindings;
  std::vector<VkDescriptorBindingFlags> bindingFlags;
  bindings.reserve(bindingCount);
  bindingFlags.reserve(bindingCount);
  m_imageSlots.reserve(
    sampledImages.size() + samplers.size() + storageImages.size()
  );

  for (const BufferBinding& buffer : buffers) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = buffer.binding;
    layoutBinding.descriptorType = buffer.type;
    layoutBinding.descriptorCount = 1;
    layoutBinding.stageFlags = buffer.stages;
    layoutBinding.pImmutableSamplers = nullptr;

    bindings.push_back(layoutBinding);
    bindingFlags.push_back(0);
  }

  for (const DeviceBufferBinding& deviceBuffer : deviceBuffers) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = deviceBuffer.binding;
    layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    layoutBinding.descriptorCount = 1;
    layoutBinding.stageFlags = deviceBuffer.stages;
    layoutBinding.pImmutableSamplers = nullptr;

    bindings.push_back(layoutBinding);
    bindingFlags.push_back(0);

    m_deviceBufferSlots.push_back(deviceBuffer.binding);
  }

  for (const SampledImageBinding& sampledImage : sampledImages) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = sampledImage.binding;
    layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    layoutBinding.descriptorCount = sampledImage.capacity;
    layoutBinding.stageFlags = sampledImage.stages;
    layoutBinding.pImmutableSamplers = nullptr;

    bindings.push_back(layoutBinding);
    bindingFlags.push_back(
      VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT
      | VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT
    );

    ImageSlot slot{};
    slot.binding = sampledImage.binding;
    slot.type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    slot.capacity = sampledImage.capacity;
    m_imageSlots.push_back(slot);
  }

  for (const SamplerBinding& sampler : samplers) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = sampler.binding;
    layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
    layoutBinding.descriptorCount = sampler.capacity;
    layoutBinding.stageFlags = sampler.stages;
    layoutBinding.pImmutableSamplers = nullptr;

    bindings.push_back(layoutBinding);
    bindingFlags.push_back(
      VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT
      | VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT
    );

    ImageSlot slot{};
    slot.binding = sampler.binding;
    slot.type = VK_DESCRIPTOR_TYPE_SAMPLER;
    slot.capacity = sampler.capacity;
    m_imageSlots.push_back(slot);
  }

  for (const StorageImageBinding& storageImage : storageImages) {
    VkDescriptorSetLayoutBinding layoutBinding{};
    layoutBinding.binding = storageImage.binding;
    layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    layoutBinding.descriptorCount = storageImage.capacity;
    layoutBinding.stageFlags = storageImage.stages;
    layoutBinding.pImmutableSamplers = nullptr;

    bindings.push_back(layoutBinding);

    bindingFlags.push_back(VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT);

    ImageSlot slot{};
    slot.binding = storageImage.binding;
    slot.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    slot.capacity = storageImage.capacity;
    m_imageSlots.push_back(slot);
  }

  VkDescriptorSetLayoutBindingFlagsCreateInfo bindingFlagsInfo{};
  bindingFlagsInfo.sType =
    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO;
  bindingFlagsInfo.pBindingFlags = bindingFlags.data();
  bindingFlagsInfo.bindingCount = static_cast<uint32_t>(bindingFlags.size());

  m_updateAfterBind = std::any_of(
    bindingFlags.begin(),
    bindingFlags.end(),
    [](VkDescriptorBindingFlags n) {
      return (n & VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT) != 0;
    }
  );

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
  layoutInfo.pBindings = bindings.data();
  layoutInfo.flags =
    m_updateAfterBind
      ? VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT
      : 0;
  layoutInfo.pNext = &bindingFlagsInfo;

  if (
    vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &m_setLayout)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create descriptor set layout");
  }
}

void DescriptorGroup::createBuffers(
  const Device& device,
  const std::vector<BufferBinding>& buffers
) {
  m_slots.reserve(buffers.size());

  for (const BufferBinding& binding : buffers) {
    const uint32_t count =
      binding.update == BufferUpdate::Static ? 1 : m_frameCount;

    Slot slot;
    slot.binding = binding.binding;
    slot.update = binding.update;
    slot.size = binding.size;
    slot.type = binding.type;
    slot.buffers.reserve(count);
    slot.mapped.resize(count);
    slot.sizes.assign(count, binding.size);

    const VkBufferUsageFlags usage =
      binding.type == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER
        ? VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
        : VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

    for (uint32_t i = 0; i < count; ++i) {
      slot.buffers.emplace_back(
        device,
        binding.size,
        usage,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT
          | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
      );

      mapInto(slot, i);
    }

    m_slots.push_back(std::move(slot));
  }
}

void DescriptorGroup::createPool(
  const std::vector<SampledImageBinding>& sampledImages,
  const std::vector<SamplerBinding>& samplers,
  const std::vector<StorageImageBinding>& storageImages,
  const std::vector<DeviceBufferBinding>& deviceBuffers
) {
  std::vector<VkDescriptorPoolSize> poolSizes;

  uint32_t uniformCount = 0;
  uint32_t storageCount = static_cast<uint32_t>(deviceBuffers.size());

  for (const Slot& slot : m_slots) {
    if (slot.type == VK_DESCRIPTOR_TYPE_STORAGE_BUFFER) {
      ++storageCount;
    } else {
      ++uniformCount;
    }
  }

  if (uniformCount > 0) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = uniformCount * m_frameCount;

    poolSizes.push_back(poolSize);
  }

  if (storageCount > 0) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSize.descriptorCount = storageCount * m_frameCount;

    poolSizes.push_back(poolSize);
  }

  uint32_t sampledImageCapacity = 0;
  for (const auto& item : sampledImages) {
    sampledImageCapacity += item.capacity;
  }

  if (sampledImageCapacity > 0) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    poolSize.descriptorCount = sampledImageCapacity * m_frameCount;
    poolSizes.push_back(poolSize);
  }

  uint32_t samplerCapacity = 0;
  for (const auto& item : samplers) {
    samplerCapacity += item.capacity;
  }

  if (samplerCapacity > 0) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_SAMPLER;
    poolSize.descriptorCount = samplerCapacity * m_frameCount;
    poolSizes.push_back(poolSize);
  }

  uint32_t storageImageCapacity = 0;
  for (const auto& item : storageImages) {
    storageImageCapacity += item.capacity;
  }

  if (storageImageCapacity > 0) {
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSize.descriptorCount = storageImageCapacity * m_frameCount;
    poolSizes.push_back(poolSize);
  }

  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
  poolInfo.pPoolSizes = poolSizes.data();
  poolInfo.maxSets = m_frameCount;

  poolInfo.flags =
    m_updateAfterBind ? VK_DESCRIPTOR_POOL_CREATE_UPDATE_AFTER_BIND_BIT : 0;

  if (
    vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_pool) != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create descriptor pool");
  }
}

void DescriptorGroup::createSets(
  const std::vector<SampledImageBinding>& sampledImages,
  const std::vector<SamplerBinding>& samplers,
  const std::vector<StorageImageBinding>& storageImages
) {
  const std::vector<VkDescriptorSetLayout> layouts(m_frameCount, m_setLayout);

  VkDescriptorSetAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  allocInfo.descriptorPool = m_pool;
  allocInfo.descriptorSetCount = m_frameCount;
  allocInfo.pSetLayouts = layouts.data();

  m_sets.resize(m_frameCount);

  if (
    vkAllocateDescriptorSets(m_device, &allocInfo, m_sets.data()) != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to allocate descriptor sets");
  }

  size_t sampledImageCapacity = 0;
  for (const auto& item : sampledImages) {
    sampledImageCapacity += item.capacity;
  }

  size_t samplerCapacity = 0;
  for (const auto& item : samplers) {
    samplerCapacity += item.capacity;
  }

  size_t storageImageCapacity = 0;
  for (const auto& item : storageImages) {
    storageImageCapacity += item.capacity;
  }

  for (uint32_t frame = 0; frame < m_frameCount; ++frame) {
    std::vector<VkDescriptorImageInfo> sampledImageInfos;
    std::vector<VkDescriptorImageInfo> samplerInfos;
    std::vector<VkDescriptorImageInfo> storageImageInfos;
    std::vector<VkWriteDescriptorSet> writes;
    std::vector<VkDescriptorBufferInfo> bufferInfos;

    bufferInfos.reserve(m_slots.size());
    sampledImageInfos.reserve(sampledImageCapacity);
    samplerInfos.reserve(samplerCapacity);
    storageImageInfos.reserve(storageImageCapacity);
    writes.reserve(
      m_slots.size() + sampledImages.size() + samplers.size()
      + storageImages.size()
    );

    for (const Slot& slot : m_slots) {
      VkDescriptorBufferInfo bufferInfo{};
      bufferInfo.buffer =
        slot.buffers.at(slot.update == BufferUpdate::Static ? 0 : frame)
          .buffer();
      bufferInfo.offset = 0;
      bufferInfo.range = slot.size;

      bufferInfos.push_back(bufferInfo);
    }

    for (size_t i = 0; i < m_slots.size(); ++i) {
      const Slot& slot = m_slots[i];

      VkWriteDescriptorSet write{};
      write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      write.dstSet = m_sets[frame];
      write.dstBinding = slot.binding;
      write.dstArrayElement = 0;
      write.descriptorType = slot.type;
      write.descriptorCount = 1;
      write.pBufferInfo = bufferInfos.data() + i;

      writes.push_back(write);
    }

    size_t sampledImageOffset = 0;
    for (const SampledImageBinding& sampledImage : sampledImages) {
      const size_t count = sampledImage.elements.size();

      if (count == 0u) {
        continue;
      }

      for (const auto& element : sampledImage.elements) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        imageInfo.imageView = element;

        sampledImageInfos.push_back(imageInfo);
      }

      VkWriteDescriptorSet write{};
      write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      write.dstSet = m_sets[frame];
      write.dstBinding = sampledImage.binding;
      write.dstArrayElement = 0;
      write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
      write.descriptorCount = static_cast<uint32_t>(count);
      write.pImageInfo = sampledImageInfos.data() + sampledImageOffset;

      writes.push_back(write);
      sampledImageOffset += count;
    }

    size_t samplerOffset = 0;
    for (const SamplerBinding& sampler : samplers) {
      const size_t count = sampler.elements.size();

      if (count == 0) {
        continue;
      }

      for (const auto& element : sampler.elements) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.sampler = element;

        samplerInfos.push_back(imageInfo);
      }

      VkWriteDescriptorSet write{};
      write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      write.dstSet = m_sets[frame];
      write.dstBinding = sampler.binding;
      write.dstArrayElement = 0;
      write.descriptorType = VK_DESCRIPTOR_TYPE_SAMPLER;
      write.descriptorCount = static_cast<uint32_t>(count);
      write.pImageInfo = samplerInfos.data() + samplerOffset;

      writes.push_back(write);
      samplerOffset += count;
    }

    size_t storageImageOffset = 0;
    for (const StorageImageBinding& storageImage : storageImages) {
      const size_t count = storageImage.elements.size();

      if (count == 0u) {
        continue;
      }

      for (const auto& element : storageImage.elements) {
        VkDescriptorImageInfo imageInfo{};
        imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
        imageInfo.imageView = element;

        storageImageInfos.push_back(imageInfo);
      }

      VkWriteDescriptorSet write{};
      write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
      write.dstSet = m_sets[frame];
      write.dstBinding = storageImage.binding;
      write.dstArrayElement = 0;
      write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
      write.descriptorCount = static_cast<uint32_t>(count);
      write.pImageInfo = storageImageInfos.data() + storageImageOffset;

      writes.push_back(write);
      storageImageOffset += count;
    }

    vkUpdateDescriptorSets(
      m_device,
      static_cast<uint32_t>(writes.size()),
      writes.data(),
      0,
      nullptr
    );
  }
}

}
