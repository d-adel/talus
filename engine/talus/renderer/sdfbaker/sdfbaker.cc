#include "sdfbaker.ih"

namespace talus::renderer {

using talus::field::SampledSdf;

namespace {

VkImageMemoryBarrier2 volumeBarrier(
  VkImage image,
  VkImageLayout from,
  VkImageLayout to
) {
  VkImageMemoryBarrier2 barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  barrier.oldLayout = from;
  barrier.newLayout = to;
  barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
  barrier.srcAccessMask =
    VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
  barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
  barrier.dstAccessMask =
    VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
  barrier.image = image;
  barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

  return barrier;
}

void transitionVolumes(
  VkCommandBuffer cmd,
  VkImage distances,
  VkImage materials,
  VkImageLayout from,
  VkImageLayout to
) {
  const std::array<VkImageMemoryBarrier2, 2> barriers{
    volumeBarrier(distances, from, to),
    volumeBarrier(materials, from, to)
  };

  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = static_cast<uint32_t>(barriers.size());
  dependency.pImageMemoryBarriers = barriers.data();

  vkCmdPipelineBarrier2(cmd, &dependency);
}

}

void SdfBaker::bake(
  const SdfRecord& gpu,
  uint32_t frame,
  VolumeImages images
) const {
  const auto start = std::chrono::steady_clock::now();

  talus::gpu::OneShotCommand cmd;
  cmd.start(m_device);

  transitionVolumes(
    cmd.cmdBuffer(),
    images.distance.image(),
    images.material.image(),
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
    VK_IMAGE_LAYOUT_GENERAL
  );

  vkCmdBindPipeline(
    cmd.cmdBuffer(),
    VK_PIPELINE_BIND_POINT_COMPUTE,
    m_bakePipeline.pipeline()
  );

  vkCmdBindDescriptorSets(
    cmd.cmdBuffer(),
    VK_PIPELINE_BIND_POINT_COMPUTE,
    m_bakePipeline.pipelineLayout(),
    0,
    1,
    &m_table.descriptorSet(frame),
    0,
    nullptr
  );

  glm::uvec3 resolution(
    gpu.resolutionVolumeIndex.x,
    gpu.resolutionVolumeIndex.y,
    gpu.resolutionVolumeIndex.z
  );

  const BakeParams params{
    gpu.localOrigin,
    gpu.voxelSize,
    glm::uvec4(resolution, gpu.config.x),
    glm::uvec4(
      gpu.resolutionVolumeIndex.w,
      gpu.config.w,
      gpu.resolutionVolumeIndex.w * talus::core::BRICK_TABLE_STRIDE,
      0u
    )
  };

  vkCmdPushConstants(
    cmd.cmdBuffer(),
    m_bakePipeline.pipelineLayout(),
    VK_SHADER_STAGE_COMPUTE_BIT,
    0,
    sizeof(params),
    &params
  );

  const glm::uvec3 groups = (resolution + glm::uvec3(3u)) / glm::uvec3(4u);

  vkCmdDispatch(cmd.cmdBuffer(), groups.x, groups.y, groups.z);

  transitionVolumes(
    cmd.cmdBuffer(),
    images.distance.image(),
    images.material.image(),
    VK_IMAGE_LAYOUT_GENERAL,
    VK_IMAGE_LAYOUT_GENERAL
  );

  vkCmdBindPipeline(
    cmd.cmdBuffer(),
    VK_PIPELINE_BIND_POINT_COMPUTE,
    m_lipschitzPipeline.pipeline()
  );

  vkCmdBindDescriptorSets(
    cmd.cmdBuffer(),
    VK_PIPELINE_BIND_POINT_COMPUTE,
    m_lipschitzPipeline.pipelineLayout(),
    0,
    1,
    &m_table.descriptorSet(frame),
    0,
    nullptr
  );

  vkCmdPushConstants(
    cmd.cmdBuffer(),
    m_lipschitzPipeline.pipelineLayout(),
    VK_SHADER_STAGE_COMPUTE_BIT,
    0,
    sizeof(params),
    &params
  );

  const glm::uvec3 bricks =
    (resolution - glm::uvec3(1u) + glm::uvec3(talus::core::BRICK_CELLS - 1u))
    / glm::uvec3(talus::core::BRICK_CELLS);

  const uint64_t brickCount =
    static_cast<uint64_t>(bricks.x) * bricks.y * bricks.z;

  if (brickCount > talus::core::MAX_BRICKS_PER_OBJECT) {
    throw std::runtime_error(
      "Sdf resolution needs more bricks than the bound table reserves"
    );
  }

  const glm::uvec3 brickGroups = (bricks + glm::uvec3(3u)) / glm::uvec3(4u);

  vkCmdDispatch(cmd.cmdBuffer(), brickGroups.x, brickGroups.y, brickGroups.z);

  VkMemoryBarrier2 boundsWritten{};
  boundsWritten.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  boundsWritten.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  boundsWritten.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
  boundsWritten.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  boundsWritten.dstAccessMask =
    VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;

  VkDependencyInfo boundsDependency{};
  boundsDependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  boundsDependency.memoryBarrierCount = 1;
  boundsDependency.pMemoryBarriers = &boundsWritten;

  vkCmdPipelineBarrier2(cmd.cmdBuffer(), &boundsDependency);

  BakeParams globalParams = params;
  globalParams.volume.w = 1u;

  vkCmdPushConstants(
    cmd.cmdBuffer(),
    m_lipschitzPipeline.pipelineLayout(),
    VK_SHADER_STAGE_COMPUTE_BIT,
    0,
    sizeof(globalParams),
    &globalParams
  );

  vkCmdDispatch(cmd.cmdBuffer(), 1, 1, 1);

  VkMemoryBarrier2 boundsVisible{};
  boundsVisible.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
  boundsVisible.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
  boundsVisible.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
  boundsVisible.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
  boundsVisible.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;

  VkDependencyInfo visibleDependency{};
  visibleDependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  visibleDependency.memoryBarrierCount = 1;
  visibleDependency.pMemoryBarriers = &boundsVisible;

  vkCmdPipelineBarrier2(cmd.cmdBuffer(), &visibleDependency);

  transitionVolumes(
    cmd.cmdBuffer(),
    images.distance.image(),
    images.material.image(),
    VK_IMAGE_LAYOUT_GENERAL,
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  );

  cmd.submit(m_device);

  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - start
  );

  std::cout << "field grid rebaked on GPU in " << elapsed.count() << " us  ("
            << gpu.config.x << " primitives)\n";
}

namespace {

std::vector<uint8_t> readVolume(
  const talus::gpu::Device& device,
  const talus::gpu::Image& image,
  const glm::uvec3& resolution,
  VkDeviceSize sizeBytes
) {
  talus::gpu::Buffer readback(
    device,
    sizeBytes,
    VK_BUFFER_USAGE_TRANSFER_DST_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  talus::gpu::OneShotCommand cmd;
  cmd.start(device);

  const std::array<VkImageMemoryBarrier2, 1> toSource{volumeBarrier(
    image.image(),
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
  )};

  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = toSource.data();

  vkCmdPipelineBarrier2(cmd.cmdBuffer(), &dependency);

  VkBufferImageCopy region{};
  region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
  region.imageExtent = {resolution.x, resolution.y, resolution.z};

  vkCmdCopyImageToBuffer(
    cmd.cmdBuffer(),
    image.image(),
    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
    readback.buffer(),
    1,
    &region
  );

  const std::array<VkImageMemoryBarrier2, 1> toRead{volumeBarrier(
    image.image(),
    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
  )};

  dependency.pImageMemoryBarriers = toRead.data();
  vkCmdPipelineBarrier2(cmd.cmdBuffer(), &dependency);

  cmd.submit(device);

  std::vector<uint8_t> bytes(static_cast<size_t>(sizeBytes));

  void* mapped = nullptr;
  if (
    vkMapMemory(device.vkDevice(), readback.memory(), 0, sizeBytes, 0, &mapped)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map the bake readback buffer");
  }

  std::memcpy(bytes.data(), mapped, static_cast<size_t>(sizeBytes));
  vkUnmapMemory(device.vkDevice(), readback.memory());

  return bytes;
}

std::vector<float> readBounds(
  const talus::gpu::Device& device,
  const talus::gpu::Buffer& bounds,
  uint32_t volumeIndex,
  uint32_t count
) {
  const VkDeviceSize sizeBytes =
    static_cast<VkDeviceSize>(count) * sizeof(float);

  talus::gpu::Buffer readback(
    device,
    sizeBytes,
    VK_BUFFER_USAGE_TRANSFER_DST_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );

  talus::gpu::OneShotCommand cmd;
  cmd.start(device);

  VkBufferCopy region{};
  region.srcOffset = static_cast<VkDeviceSize>(volumeIndex)
                     * talus::core::BRICK_TABLE_STRIDE * sizeof(float);
  region.dstOffset = 0;
  region.size = sizeBytes;

  vkCmdCopyBuffer(
    cmd.cmdBuffer(),
    bounds.buffer(),
    readback.buffer(),
    1,
    &region
  );

  cmd.submit(device);

  std::vector<float> values(count);

  void* mapped = nullptr;
  if (
    vkMapMemory(device.vkDevice(), readback.memory(), 0, sizeBytes, 0, &mapped)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to map the bound readback buffer");
  }

  std::memcpy(values.data(), mapped, static_cast<size_t>(sizeBytes));
  vkUnmapMemory(device.vkDevice(), readback.memory());

  return values;
}

}

BakeShaders bakeShaders() {
  return {"shaders/sdf-bake.comp.spv", "shaders/sdf-lipschitz.comp.spv"};
}

VkPushConstantRange bakePushConstantRange() {
  return {VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(BakeParams)};
}

SdfBaker::SdfBaker(
  const talus::gpu::Device& device,
  const SdfRecordTable& table
)
    : m_device(device),
      m_table(table),
      m_bakePipeline(
        device.vkDevice(),
        bakeShaders().distanceSpirv,
        std::vector<VkDescriptorSetLayout>{table.setLayout()},
        std::vector<VkPushConstantRange>{bakePushConstantRange()}
      ),
      m_lipschitzPipeline(
        device.vkDevice(),
        bakeShaders().lipschitzSpirv,
        std::vector<VkDescriptorSetLayout>{table.setLayout()},
        std::vector<VkPushConstantRange>{bakePushConstantRange()}
      ) {}

void SdfBaker::verifyBake(
  const talus::objectmodel::SdfGrid& grid,
  uint32_t volumeIndex,
  std::span<const talus::field::Primitive> primitives,
  VolumeImages images
) const {
  const talus::field::Aabb box = talus::objectmodel::gridBox(
    primitives,
    talus::objectmodel::gridMargin(grid, primitives)
  );

  const SampledSdf reference =
    talus::field::bake(primitives, box.minimum, box.maximum, grid.resolution);

  const std::vector<uint8_t> distanceBytes = readVolume(
    m_device,
    images.distance,
    grid.resolution,
    reference.distances.size() * sizeof(float)
  );

  const std::vector<uint8_t> materialBytes = readVolume(
    m_device,
    images.material,
    grid.resolution,
    reference.materials.size()
  );

  const float* baked = reinterpret_cast<const float*>(distanceBytes.data());

  float worstDistance = 0.0f;
  size_t materialMismatches = 0;

  for (size_t i = 0; i < reference.distances.size(); ++i) {
    worstDistance =
      std::max(worstDistance, std::abs(baked[i] - reference.distances[i]));

    if (materialBytes[i] != reference.materials[i]) {
      ++materialMismatches;
    }
  }

  std::cout << "bake check  worst distance " << std::scientific << worstDistance
            << "  material mismatches " << materialMismatches << " of "
            << reference.distances.size() << '\n';

  const std::vector<float> bounds = readBounds(
    m_device,
    m_table.brickBounds(),
    volumeIndex,
    1u + static_cast<uint32_t>(reference.brickLipschitz.size())
  );

  float worstBound = 0.0f;
  float gpuLargestBrick = 0.0f;

  for (size_t i = 0; i < reference.brickLipschitz.size(); ++i) {
    const float gpuBrick = bounds[i + 1u];

    worstBound =
      std::max(worstBound, std::abs(gpuBrick - reference.brickLipschitz[i]));
    gpuLargestBrick = std::max(gpuLargestBrick, gpuBrick);
  }

  const float gpuGlobal = bounds[0];

  std::cout << "bound check  worst brick " << std::scientific << worstBound
            << " over " << reference.brickLipschitz.size()
            << " bricks  global vs its own bricks "
            << std::abs(gpuGlobal - gpuLargestBrick) << "  vs cpu "
            << std::abs(gpuGlobal - reference.globalLipschitz) << '\n';
}

}
