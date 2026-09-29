#include "sdfrecordtable.ih"

namespace talus::renderer {

namespace {

constexpr uint32_t INITIAL_RECORDS = 1280u;

constexpr VkBufferUsageFlags BRICK_BOUNDS_USAGE =
  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT
  | VK_BUFFER_USAGE_TRANSFER_DST_BIT;

VkDeviceSize brickBoundsBytes(uint32_t volumes) {
  return static_cast<VkDeviceSize>(volumes) * talus::core::BRICK_TABLE_STRIDE
         * sizeof(float);
}

void copyBrickBounds(
  VkCommandBuffer commandBuffer,
  VkBuffer from,
  VkBuffer to,
  VkDeviceSize sizeBytes
) {
  VkMemoryBarrier2 written{};
  written.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  written.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
  written.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
  written.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  written.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;

  VkDependencyInfo before{};
  before.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  before.memoryBarrierCount = 1;
  before.pMemoryBarriers = &written;

  vkCmdPipelineBarrier2(commandBuffer, &before);

  VkBufferCopy region{};
  region.size = sizeBytes;

  vkCmdCopyBuffer(commandBuffer, from, to, 1, &region);

  VkMemoryBarrier2 copied{};
  copied.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  copied.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  copied.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
  copied.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT
                        | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                        | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  copied.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT
                         | VK_ACCESS_2_SHADER_STORAGE_READ_BIT
                         | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;

  VkDependencyInfo after{};
  after.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  after.memoryBarrierCount = 1;
  after.pMemoryBarriers = &copied;

  vkCmdPipelineBarrier2(commandBuffer, &after);
}

}

SdfRecordTable::SdfRecordTable(const talus::gpu::Device& device)
    : m_device(device),
      m_group(
        device,
        talus::core::MAX_FRAMES_IN_FLIGHT,
        {{ENTRIES,
          INITIAL_RECORDS * sizeof(SdfRecord),
          VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT
            | VK_SHADER_STAGE_COMPUTE_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame,
          VK_DESCRIPTOR_TYPE_STORAGE_BUFFER},
         {PRIMITIVE_POOL,
          talus::core::MAX_PRIMITIVE_POOL * sizeof(talus::field::Primitive),
          VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame,
          VK_DESCRIPTOR_TYPE_STORAGE_BUFFER},
         {RECORD_BOUNDS,
          INITIAL_RECORDS * sizeof(RecordBounds),
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame,
          VK_DESCRIPTOR_TYPE_STORAGE_BUFFER},
         {SHADOW_CELLS,
          SHADOW_GRID_CELLS * SHADOW_GRID_CELLS * sizeof(ShadowCell),
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame,
          VK_DESCRIPTOR_TYPE_STORAGE_BUFFER},
         {SHADOW_INDICES,
          SHADOW_GRID_INITIAL_INDICES * sizeof(uint32_t),
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame,
          VK_DESCRIPTOR_TYPE_STORAGE_BUFFER},
         {SHADOW_GRID,
          sizeof(ShadowGridUniform),
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame}},
        {{DISTANCE_VOLUMES,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          {},
          talus::core::MAX_SDF_VOLUMES},
         {MATERIAL_VOLUMES,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          {},
          talus::core::MAX_SDF_VOLUMES}},
        {},
        {{DISTANCE_VOLUMES_STORAGE,
          VK_SHADER_STAGE_COMPUTE_BIT,
          {},
          talus::core::MAX_SDF_VOLUMES},
         {MATERIAL_VOLUMES_STORAGE,
          VK_SHADER_STAGE_COMPUTE_BIT,
          {},
          talus::core::MAX_SDF_VOLUMES}},
        {{BRICK_BOUNDS,
          VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT}}
      ),
      m_brickBounds(
        device,
        brickBoundsBytes(volumeCapacityFor(0u)),
        BRICK_BOUNDS_USAGE,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
      ),
      m_volumeCapacity(volumeCapacityFor(0u)) {
  m_records.resize(INITIAL_RECORDS);
  m_bakeQueued.resize(talus::core::MAX_SDF_VOLUMES, 0u);
  m_recordBounds.resize(INITIAL_RECORDS);

  m_group.writeBuffer(
    BRICK_BOUNDS,
    m_brickBounds.buffer(),
    brickBoundsBytes(m_volumeCapacity)
  );
}

uint32_t SdfRecordTable::volumeCapacity() const noexcept {
  return m_volumeCapacity;
}

void SdfRecordTable::fitVolumes(uint32_t needed) {
  if (needed <= m_volumeCapacity) {
    return;
  }

  const uint32_t grown = volumeCapacityFor(needed);

  if (grown < needed) {
    throw std::runtime_error("A volume index past the brick table ceiling");
  }

  m_device.waitIdle();

  talus::gpu::Buffer larger(
    m_device,
    brickBoundsBytes(grown),
    BRICK_BOUNDS_USAGE,
    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
  );

  talus::gpu::OneShotCommand cmd;
  cmd.start(m_device);

  copyBrickBounds(
    cmd.cmdBuffer(),
    m_brickBounds.buffer(),
    larger.buffer(),
    brickBoundsBytes(m_volumeCapacity)
  );

  cmd.submit(m_device);

  m_brickBounds = std::move(larger);
  m_volumeCapacity = grown;

  m_group.writeBuffer(
    BRICK_BOUNDS,
    m_brickBounds.buffer(),
    brickBoundsBytes(m_volumeCapacity)
  );
}

void SdfRecordTable::registerVolume(
  VkImageView distanceView,
  VkImageView materialView,
  uint32_t volumeIndex
) {
  fitVolumes(volumeIndex + 1u);

  m_group.writeImage(DISTANCE_VOLUMES, volumeIndex, distanceView);
  m_group.writeImage(MATERIAL_VOLUMES, volumeIndex, materialView);
  m_group.writeStorageImage(
    DISTANCE_VOLUMES_STORAGE,
    volumeIndex,
    distanceView
  );
  m_group.writeStorageImage(
    MATERIAL_VOLUMES_STORAGE,
    volumeIndex,
    materialView
  );
}

void SdfRecordTable::uploadBounds(
  talus::gpu::Uploader& uploader,
  uint32_t volumeIndex,
  const talus::field::SampledSdf& field
) {
  VkDeviceSize sizeBytes = 0;
  talus::gpu::Buffer staging = stageBounds(field, sizeBytes);

  recordBounds(uploader.begin(), staging, volumeIndex, sizeBytes);

  uploader.keep(std::move(staging));
}

void SdfRecordTable::uploadBounds(
  uint32_t volumeIndex,
  const talus::field::SampledSdf& field
) {
  VkDeviceSize sizeBytes = 0;
  const talus::gpu::Buffer staging = stageBounds(field, sizeBytes);

  talus::gpu::OneShotCommand cmd;
  cmd.start(m_device);

  recordBounds(cmd.cmdBuffer(), staging, volumeIndex, sizeBytes);

  cmd.submit(m_device);
}

talus::gpu::Buffer SdfRecordTable::stageBounds(
  const talus::field::SampledSdf& field,
  VkDeviceSize& sizeBytes
) const {
  std::vector<float> table;
  table.reserve(1u + field.brickLipschitz.size());
  table.push_back(field.globalLipschitz);
  table.insert(
    table.end(),
    field.brickLipschitz.begin(),
    field.brickLipschitz.end()
  );

  if (table.size() > talus::core::BRICK_TABLE_STRIDE) {
    throw std::runtime_error("A field has more bricks than its table slot");
  }

  sizeBytes = static_cast<VkDeviceSize>(table.size()) * sizeof(float);

  talus::gpu::Buffer staging(
    m_device,
    sizeBytes,
    VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  void* mapped = nullptr;

  if (
    vkMapMemory(m_device.vkDevice(), staging.memory(), 0, sizeBytes, 0, &mapped)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map brick bound staging memory");
  }

  memcpy(mapped, table.data(), static_cast<size_t>(sizeBytes));
  vkUnmapMemory(m_device.vkDevice(), staging.memory());

  return staging;
}

void SdfRecordTable::recordBounds(
  VkCommandBuffer commandBuffer,
  const talus::gpu::Buffer& staging,
  uint32_t volumeIndex,
  VkDeviceSize sizeBytes
) const {
  VkMemoryBarrier2 boundsFree{};
  boundsFree.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  boundsFree.srcStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                            | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  boundsFree.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
  boundsFree.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  boundsFree.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;

  VkDependencyInfo freeDependency{};
  freeDependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  freeDependency.memoryBarrierCount = 1;
  freeDependency.pMemoryBarriers = &boundsFree;

  vkCmdPipelineBarrier2(commandBuffer, &freeDependency);

  VkBufferCopy region{};
  region.srcOffset = 0;
  region.dstOffset = static_cast<VkDeviceSize>(volumeIndex)
                     * talus::core::BRICK_TABLE_STRIDE * sizeof(float);
  region.size = sizeBytes;

  vkCmdCopyBuffer(
    commandBuffer,
    staging.buffer(),
    m_brickBounds.buffer(),
    1,
    &region
  );

  VkMemoryBarrier2 boundsVisible{};
  boundsVisible.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  boundsVisible.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
  boundsVisible.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
  boundsVisible.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                               | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  boundsVisible.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;

  VkDependencyInfo visibleDependency{};
  visibleDependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  visibleDependency.memoryBarrierCount = 1;
  visibleDependency.pMemoryBarriers = &boundsVisible;

  vkCmdPipelineBarrier2(commandBuffer, &visibleDependency);
}

const talus::gpu::Buffer& SdfRecordTable::brickBounds() const noexcept {
  return m_brickBounds;
}

void SdfRecordTable::newFrame() {
  for (const uint32_t slot : m_bakeDispatch) {
    m_bakeQueued[m_records[slot].resolutionVolumeIndex.w] = 0u;
  }

  m_bakeList.clear();
  m_bakeDispatch.clear();
}

void SdfRecordTable::appendToBakeList(uint32_t slot) {
  m_bakeList.push_back(slot);

  const uint32_t volume = m_records.at(slot).resolutionVolumeIndex.w;

  if (volume >= m_bakeQueued.size() || m_bakeQueued[volume] != 0u) {
    return;
  }

  m_bakeQueued[volume] = 1u;
  m_bakeDispatch.push_back(slot);
}

void SdfRecordTable::setRecord(
  uint32_t recordIndex,
  uint32_t primitiveOffset,
  uint32_t primitiveCount,
  const talus::objectmodel::Pose& pose,
  const talus::objectmodel::SdfGrid& grid,
  const talus::objectmodel::BakedVolume& volume,
  uint32_t fieldRepresentation,
  std::span<const talus::field::Primitive> primitives
) {
  if (recordIndex >= m_records.size()) {
    const size_t grown =
      std::max<size_t>(size_t(recordIndex) + 1u, 2u * m_records.size());

    m_records.resize(grown);
    m_recordBounds.resize(grown);
  }

  m_records[recordIndex] = makeSdfRecord(
    pose,
    grid,
    volume,
    primitiveOffset,
    primitiveCount,
    fieldRepresentation,
    talus::objectmodel::gridMargin(grid, primitives)
  );

  m_recordBounds[recordIndex] = makeRecordBounds(
    talus::objectmodel::casterBox(primitives, grid),
    m_records[recordIndex].model
  );
}

void SdfRecordTable::update(
  uint32_t frame,
  uint32_t liveRecords,
  const glm::vec3& toLight
) {
  const size_t live = std::min<size_t>(liveRecords, m_records.size());

  const std::span<const SdfRecord> objects(m_records.data(), live);

  fit(ENTRIES, frame, objects.size_bytes());

  m_group.write(ENTRIES, frame, objects.data(), objects.size_bytes());

  const std::span<const RecordBounds> bounds(m_recordBounds.data(), live);

  fit(RECORD_BOUNDS, frame, bounds.size_bytes());

  m_group.write(RECORD_BOUNDS, frame, bounds.data(), bounds.size_bytes());

  const auto shadowBegan = std::chrono::steady_clock::now();

  m_shadowGrid.build(bounds, liveRecords, toLight);

  m_shadowBuildMs = 1000.0
                    * std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - shadowBegan
                    )
                        .count();

  const std::span<const ShadowCell> cells = m_shadowGrid.cells();
  const std::span<const uint32_t> indices = m_shadowGrid.indices();

  if (!cells.empty()) {
    m_group.write(SHADOW_CELLS, frame, cells.data(), cells.size_bytes());
  }

  if (!indices.empty()) {
    fit(SHADOW_INDICES, frame, indices.size_bytes());

    m_group.write(SHADOW_INDICES, frame, indices.data(), indices.size_bytes());
  }

  const ShadowGridUniform& grid = m_shadowGrid.uniform();

  m_group.write(SHADOW_GRID, frame, &grid, sizeof(grid));
}

void SdfRecordTable::fit(uint32_t binding, uint32_t frame, VkDeviceSize bytes) {
  VkDeviceSize held = m_group.capacity(binding, frame);

  if (bytes <= held) {
    return;
  }

  while (held < bytes) {
    held *= 2u;
  }

  m_group.resize(binding, frame, held);
}

void SdfRecordTable::updatePrimitives(
  uint32_t frame,
  std::span<const talus::field::Primitive> primitives
) {
  m_group.write(
    PRIMITIVE_POOL,
    frame,
    primitives.data(),
    primitives.size_bytes()
  );
}

const VkDescriptorSet& SdfRecordTable::descriptorSet(
  uint32_t frame
) const noexcept {
  return m_group.descriptorSet(frame);
}

const VkDescriptorSetLayout& SdfRecordTable::setLayout() const noexcept {
  return m_group.setLayout();
}

std::span<const SdfRecord> SdfRecordTable::records() const noexcept {
  return m_records;
}

std::span<const uint32_t> SdfRecordTable::bakeList() const noexcept {
  return m_bakeList;
}

std::span<const uint32_t> SdfRecordTable::bakeDispatch() const noexcept {
  return m_bakeDispatch;
}

const SdfRecord& SdfRecordTable::record(uint32_t recordIndex) const {
  return m_records.at(recordIndex);
}

const ShadowGrid& SdfRecordTable::shadowGrid() const noexcept {
  return m_shadowGrid;
}

double SdfRecordTable::shadowBuildMs() const noexcept {
  return m_shadowBuildMs;
}

}
