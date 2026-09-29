#pragma once

#include <talus/gpu/texture/texture.h>
#include <talus/gpu/uploader/uploader.h>
#include <talus/core/config/config.h>
#include <talus/gpu/device/device.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <vulkan/vulkan.h>
#include <optional>
#include <array>
#include <cstddef>
#include <span>
#include <unordered_map>
#include <vector>

namespace talus::renderer {

struct VolumeImages {
  const talus::gpu::Image& distance;
  const talus::gpu::Image& material;
};

inline constexpr uint32_t INITIAL_SDF_VOLUMES = 1280u;

[[nodiscard]] uint32_t volumeCapacityFor(uint32_t needed);

struct VolumeKey {
  std::vector<std::byte> bytes;
  bool lattice = false;

  bool operator==(const VolumeKey& other) const = default;
};

VolumeKey volumeKey(
  std::span<const talus::field::Primitive> primitives,
  const glm::uvec3& resolution,
  float margin,
  const glm::vec3& origin,
  const glm::vec3& voxelSize
);

[[nodiscard]] VolumeKey latticeKey(const talus::field::SampledSdf& lattice);

[[nodiscard]] bool sameLattice(
  const talus::field::SampledSdf& first,
  const talus::field::SampledSdf& second
);

class VolumePool {
public:
  explicit VolumePool(const talus::gpu::Device& device);

  VolumePool(const VolumePool&) = delete;
  VolumePool& operator=(const VolumePool&) = delete;

  VolumePool(VolumePool&& other) = default;
  VolumePool& operator=(VolumePool&& other) = default;

  [[nodiscard]] uint32_t acquire(const VolumeKey& key);

  [[nodiscard]] uint32_t allocate(const talus::field::SampledSdf& grid);

  [[nodiscard]] uint32_t allocate(
    const talus::field::SampledSdf& grid,
    const VolumeKey& key
  );

  [[nodiscard]] uint32_t makeUnique(
    talus::gpu::Uploader& uploader,
    uint32_t index,
    const talus::field::SampledSdf& grid
  );

  void retain(uint32_t index);

  void release(uint32_t index);

  void forgetKey(uint32_t index);

  [[nodiscard]] const VolumeKey& key(uint32_t index) const;

  [[nodiscard]] uint32_t users(uint32_t index) const;

  [[nodiscard]] uint32_t allocated() const;

  [[nodiscard]] uint32_t capacity() const noexcept;

  void upload(
    talus::gpu::Uploader& uploader,
    uint32_t index,
    const talus::field::SampledSdf& grid,
    const talus::field::SampleBox& box
  );

  VolumeImages images(uint32_t index) const;

private:
  struct Volume {
    talus::gpu::Texture distance;
    talus::gpu::Texture material;
    uint32_t users = 0u;
    VolumeKey key;
  };

  [[nodiscard]] uint32_t fill(
    const talus::field::SampledSdf& grid,
    VolumeKey key
  );

  [[nodiscard]] bool takeFreeIndex(uint32_t& index);

  void writeInto(
    talus::gpu::Uploader& uploader,
    uint32_t index,
    const talus::field::SampledSdf& grid,
    const talus::field::SampleBox& box,
    VkImageLayout from
  );

  talus::gpu::Texture makeDistanceVolume(
    const talus::gpu::Device& device,
    const talus::field::SampledSdf& grid
  );
  talus::gpu::Texture makeMaterialVolume(
    const talus::gpu::Device& device,
    const talus::field::SampledSdf& grid
  );

  const talus::gpu::Device& m_device;

  std::vector<std::optional<Volume>> m_volumes;
  std::vector<uint32_t> m_freeIndices;
  std::unordered_map<size_t, std::vector<uint32_t>> m_shared;
};

}
