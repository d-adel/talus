#pragma once

#include <talus/gpu/descriptorgroup/descriptorgroup.h>
#include <talus/gpu/device/device.h>
#include <talus/renderer/materialrecord/materialrecord.h>
#include <talus/gpu/sampler/sampler.h>
#include <talus/gpu/texture/texture.h>

#include <cstdint>
#include <span>

namespace talus::renderer {

class ResourceTable {
public:
  static constexpr uint32_t MATERIALS = 0;
  static constexpr uint32_t TEXTURES = 1;
  static constexpr uint32_t SAMPLERS = 2;

  ResourceTable(
    const talus::gpu::Device& device,
    std::span<const talus::gpu::Texture> textures,
    std::span<const talus::gpu::Sampler> samplers,
    std::span<const talus::renderer::MaterialRecord> materials
  );

  ResourceTable(const ResourceTable&) = delete;
  ResourceTable& operator=(const ResourceTable&) = delete;
  ResourceTable(ResourceTable&&) = delete;
  ResourceTable& operator=(ResourceTable&&) = delete;

  ~ResourceTable() = default;

  void refresh(std::span<const talus::renderer::MaterialRecord> materials);

  const VkDescriptorSet& descriptorSet(uint32_t frame) const noexcept;
  const VkDescriptorSetLayout& setLayout() const noexcept;

private:
  talus::gpu::DescriptorGroup m_group;
  uint32_t m_textureCount;
  uint32_t m_samplerCount;
};

}
