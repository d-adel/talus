#include <talus/renderer/volumepool/volumepool.ih>

namespace talus::renderer {

using talus::field::SampledSdf;

namespace {

constexpr uint64_t FNV_OFFSET = 14695981039346656037ull;

constexpr uint64_t FNV_PRIME = 1099511628211ull;

size_t hashKey(const VolumeKey& key) {
  size_t hash = FNV_OFFSET;

  for (std::byte value : key.bytes) {
    hash ^= static_cast<size_t>(value);
    hash *= FNV_PRIME;
  }

  hash ^= key.lattice ? 1u : 0u;
  hash *= FNV_PRIME;

  return hash;
}

template<typename T>
void appendBytes(std::vector<std::byte>& bytes, const T& value) {
  const size_t at = bytes.size();

  bytes.resize(at + sizeof(T));

  memcpy(bytes.data() + at, &value, sizeof(T));
}

template<typename T>
bool sameBytes(const std::vector<T>& first, const std::vector<T>& second) {
  return first.size() == second.size()
         && (first.empty() || memcmp(first.data(), second.data(), first.size() * sizeof(T)) == 0);
}

uint64_t latticeHash(const SampledSdf& lattice) {
  uint64_t hash = FNV_OFFSET;

  for (const float distance : lattice.distances) {
    hash ^= std::bit_cast<uint32_t>(distance);
    hash *= FNV_PRIME;
  }

  for (const uint8_t material : lattice.materials) {
    hash ^= material;
    hash *= FNV_PRIME;
  }

  return hash;
}

}

uint32_t volumeCapacityFor(uint32_t needed) {
  uint32_t capacity = INITIAL_SDF_VOLUMES;

  while (capacity < needed && capacity < talus::core::MAX_SDF_VOLUMES) {
    capacity *= 2u;
  }

  return std::min(capacity, talus::core::MAX_SDF_VOLUMES);
}

VolumeKey latticeKey(const SampledSdf& lattice) {
  VolumeKey key;
  key.lattice = true;
  key.bytes.reserve(
    sizeof(lattice.resolution) + sizeof(lattice.origin)
    + sizeof(lattice.voxelSize) + sizeof(uint64_t)
  );

  appendBytes(key.bytes, lattice.resolution);
  appendBytes(key.bytes, lattice.origin);
  appendBytes(key.bytes, lattice.voxelSize);
  appendBytes(key.bytes, latticeHash(lattice));

  return key;
}

bool sameLattice(const SampledSdf& first, const SampledSdf& second) {
  if (&first == &second) {
    return true;
  }

  return memcmp(&first.resolution, &second.resolution, sizeof(first.resolution))
           == 0
         && memcmp(&first.origin, &second.origin, sizeof(first.origin)) == 0
         && memcmp(&first.voxelSize, &second.voxelSize, sizeof(first.voxelSize))
              == 0
         && sameBytes(first.distances, second.distances)
         && sameBytes(first.materials, second.materials);
}

VolumeKey volumeKey(
  std::span<const talus::field::Primitive> primitives,
  const glm::uvec3& resolution,
  float margin,
  const glm::vec3& origin,
  const glm::vec3& voxelSize
) {
  const uint32_t dimensions[10] = {
    resolution.x,
    resolution.y,
    resolution.z,
    std::bit_cast<uint32_t>(margin),
    std::bit_cast<uint32_t>(origin.x),
    std::bit_cast<uint32_t>(origin.y),
    std::bit_cast<uint32_t>(origin.z),
    std::bit_cast<uint32_t>(voxelSize.x),
    std::bit_cast<uint32_t>(voxelSize.y),
    std::bit_cast<uint32_t>(voxelSize.z)
  };

  VolumeKey key;
  key.bytes.resize(primitives.size_bytes() + sizeof(dimensions));

  if (!primitives.empty()) {
    memcpy(key.bytes.data(), primitives.data(), primitives.size_bytes());
  }

  memcpy(
    key.bytes.data() + primitives.size_bytes(),
    dimensions,
    sizeof(dimensions)
  );

  return key;
}

VolumePool::VolumePool(const talus::gpu::Device& device) : m_device(device) {
  const uint32_t capacity = volumeCapacityFor(0u);

  m_volumes.resize(capacity);
  m_freeIndices.reserve(capacity);

  for (uint32_t i = 0; i < capacity; ++i) {
    m_freeIndices.push_back(capacity - 1 - i);
  }
}

bool VolumePool::takeFreeIndex(uint32_t& index) {
  if (m_freeIndices.empty()) {
    const uint32_t held = capacity();
    const uint32_t grown = volumeCapacityFor(held + 1u);

    if (grown <= held) {
      return false;
    }

    m_volumes.resize(grown);

    for (uint32_t fresh = grown; fresh != held; --fresh) {
      m_freeIndices.push_back(fresh - 1u);
    }
  }

  index = m_freeIndices.back();
  m_freeIndices.pop_back();

  return true;
}

uint32_t VolumePool::acquire(const VolumeKey& key) {
  if (key.bytes.empty()) {
    return UINT32_MAX;
  }

  const auto bucket = m_shared.find(hashKey(key));

  if (bucket == m_shared.end()) {
    return UINT32_MAX;
  }

  for (const uint32_t index : bucket->second) {
    if (m_volumes[index].has_value() && m_volumes[index]->key == key) {
      ++m_volumes[index]->users;

      return index;
    }
  }

  return UINT32_MAX;
}

uint32_t VolumePool::fill(const SampledSdf& grid, VolumeKey key) {
  uint32_t index = UINT32_MAX;

  if (!takeFreeIndex(index)) {
    return UINT32_MAX;
  }

  m_volumes[index].emplace(
    Volume{
      makeDistanceVolume(m_device, grid),
      makeMaterialVolume(m_device, grid),
      1u,
      std::move(key)
    }
  );

  return index;
}

uint32_t VolumePool::allocate(const SampledSdf& grid) {
  return fill(grid, VolumeKey{});
}

uint32_t VolumePool::allocate(const SampledSdf& grid, const VolumeKey& key) {
  const uint32_t shared = acquire(key);

  if (shared != UINT32_MAX) {
    return shared;
  }

  const uint32_t index = fill(grid, key);

  if (index != UINT32_MAX && !key.bytes.empty()) {
    m_shared[hashKey(key)].push_back(index);
  }

  return index;
}

uint32_t VolumePool::makeUnique(
  talus::gpu::Uploader& uploader,
  uint32_t index,
  const SampledSdf& grid
) {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Splitting a volume slot that is not allocated");
  }

  if (m_volumes[index]->users == 1u) {
    return index;
  }

  uint32_t fresh = UINT32_MAX;

  if (!takeFreeIndex(fresh)) {
    return UINT32_MAX;
  }

  m_volumes[fresh].emplace(
    Volume{
      talus::gpu::Texture(
        m_device,
        grid.resolution.x,
        grid.resolution.y,
        grid.resolution.z,
        VK_FORMAT_R32_SFLOAT,
        VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
      ),
      talus::gpu::Texture(
        m_device,
        grid.resolution.x,
        grid.resolution.y,
        grid.resolution.z,
        VK_FORMAT_R8_UINT,
        VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
      ),
      1u,
      VolumeKey{}
    }
  );

  writeInto(
    uploader,
    fresh,
    grid,
    talus::field::SampleBox{glm::uvec3(0u), grid.resolution},
    VK_IMAGE_LAYOUT_UNDEFINED
  );

  return fresh;
}

void VolumePool::retain(uint32_t index) {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Retaining a volume slot that is not allocated");
  }

  ++m_volumes[index]->users;
}

void VolumePool::release(uint32_t index) {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Releasing a volume slot that is not allocated");
  }

  Volume& volume = *m_volumes[index];

  if (volume.users > 1u) {
    --volume.users;

    return;
  }

  m_device.waitIdle();

  forgetKey(index);

  m_volumes[index].reset();
  m_freeIndices.push_back(index);
}

void VolumePool::forgetKey(uint32_t index) {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Forgetting the key of an unallocated volume");
  }

  VolumeKey& key = m_volumes[index]->key;

  if (key.bytes.empty()) {
    return;
  }

  const auto bucket = m_shared.find(hashKey(key));

  if (bucket != m_shared.end()) {
    std::erase(bucket->second, index);

    if (bucket->second.empty()) {
      m_shared.erase(bucket);
    }
  }

  key = VolumeKey{};
}

const VolumeKey& VolumePool::key(uint32_t index) const {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Reading the key of an unallocated volume");
  }

  return m_volumes[index]->key;
}

uint32_t VolumePool::users(uint32_t index) const {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    return 0u;
  }

  return m_volumes[index]->users;
}

uint32_t VolumePool::allocated() const {
  return capacity() - static_cast<uint32_t>(m_freeIndices.size());
}

uint32_t VolumePool::capacity() const noexcept {
  return static_cast<uint32_t>(m_volumes.size());
}

namespace {

template<typename T, typename Source>
std::vector<T> gather(
  const SampledSdf& grid,
  const talus::field::SampleBox& box,
  const std::vector<Source>& from
) {
  std::vector<T> packed;
  packed.reserve(
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z
  );

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const glm::uvec3 at = box.minimum + glm::uvec3(x, y, z);
        const size_t index =
          at.x + grid.resolution.x * (at.y + grid.resolution.y * at.z);

        packed.push_back(static_cast<T>(from[index]));
      }
    }
  }

  return packed;
}

void stageInto(
  const talus::gpu::Device& device,
  talus::gpu::Uploader& uploader,
  talus::gpu::Image& target,
  const void* data,
  VkDeviceSize sizeBytes,
  const talus::field::SampleBox& box,
  VkImageLayout from
) {
  talus::gpu::Buffer staging(
    device,
    sizeBytes,
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  void* mapped = nullptr;

  if (
    vkMapMemory(device.vkDevice(), staging.memory(), 0, sizeBytes, 0, &mapped)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map volume upload staging memory");
  }

  memcpy(mapped, data, static_cast<size_t>(sizeBytes));
  vkUnmapMemory(device.vkDevice(), staging.memory());

  const VkCommandBuffer commandBuffer = uploader.begin();

  target.recordTransition(
    commandBuffer,
    from,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL
  );

  target.recordCopyFrom(
    commandBuffer,
    staging,
    box.extent.x,
    box.extent.y,
    box.extent.z,
    VkOffset3D{
      static_cast<int32_t>(box.minimum.x),
      static_cast<int32_t>(box.minimum.y),
      static_cast<int32_t>(box.minimum.z)
    }
  );

  target.recordTransition(
    commandBuffer,
    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  );

  uploader.keep(std::move(staging));
}

}

void VolumePool::writeInto(
  talus::gpu::Uploader& uploader,
  uint32_t index,
  const SampledSdf& grid,
  const talus::field::SampleBox& box,
  VkImageLayout from
) {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error(
      "Uploading into a volume slot that is not allocated"
    );
  }

  const glm::uvec3 beyond = box.minimum + box.extent;

  if (glm::any(glm::greaterThan(beyond, grid.resolution))) {
    throw std::runtime_error("An upload must stay inside the lattice");
  }

  if (box.extent.x == 0u || box.extent.y == 0u || box.extent.z == 0u) {
    return;
  }

  Volume& volume = *m_volumes[index];

  const std::vector<float> distances = gather<float>(grid, box, grid.distances);

  stageInto(
    m_device,
    uploader,
    volume.distance.image(),
    distances.data(),
    distances.size() * sizeof(float),
    box,
    from
  );

  const std::vector<uint8_t> ids = gather<uint8_t>(grid, box, grid.materials);

  stageInto(
    m_device,
    uploader,
    volume.material.image(),
    ids.data(),
    ids.size(),
    box,
    from
  );
}

void VolumePool::upload(
  talus::gpu::Uploader& uploader,
  uint32_t index,
  const SampledSdf& grid,
  const talus::field::SampleBox& box
) {
  writeInto(
    uploader,
    index,
    grid,
    box,
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  );
}

VolumeImages VolumePool::images(uint32_t index) const {
  if (index >= m_volumes.size() || !m_volumes[index].has_value()) {
    throw std::runtime_error("Reading a volume slot that is not allocated");
  }

  const Volume& volume = *m_volumes[index];

  return {volume.distance.image(), volume.material.image()};
}

talus::gpu::Texture VolumePool::makeDistanceVolume(
  const talus::gpu::Device& device,
  const SampledSdf& grid
) {
  return talus::gpu::Texture(
    device,
    grid.resolution.x,
    grid.resolution.y,
    grid.resolution.z,
    VK_FORMAT_R32_SFLOAT,
    grid.distances.data(),
    grid.distances.size() * sizeof(float),
    VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
  );
}

talus::gpu::Texture VolumePool::makeMaterialVolume(
  const talus::gpu::Device& device,
  const SampledSdf& grid
) {
  return talus::gpu::Texture(
    device,
    grid.resolution.x,
    grid.resolution.y,
    grid.resolution.z,
    VK_FORMAT_R8_UINT,
    grid.materials.data(),
    grid.materials.size(),
    VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT
  );
}

}
