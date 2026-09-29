#include "renderer.ih"

namespace talus::renderer {

namespace {

glm::vec3 lightDirection(
  const std::optional<talus::objectmodel::DirectionalLight>& light
) {
  return light.has_value() ? talus::objectmodel::toLight(*light)
                           : glm::vec3(0.0f);
}

}

Renderer::Renderer(
  const talus::gpu::Device& device,
  SdfRecordTable& sdfRecordTable,
  const SdfBaker& sdfBaker,
  const VolumePool& volumePool,
  FrameGlobals& frameGlobals,
  const talus::gpu::Pipeline& meshPipeline,
  const talus::gpu::Pipeline& sdfPipeline,
  const talus::gpu::Pipeline& meshDepthPipeline,
  const talus::gpu::Pipeline& sdfDepthPipeline,
  Lines& lines,
  const ResourceTable& resourceTable,
  const VkSurfaceKHR& surface,
  uint32_t imageCount
)
    : m_graphicsQueue(device.graphicsQueue()),
      m_presentQueue(device.presentQueue()),
      m_device(device.vkDevice()),
      m_meshPipeline(meshPipeline),
      m_sdfPipeline(sdfPipeline),
      m_meshDepthPipeline(meshDepthPipeline),
      m_sdfDepthPipeline(sdfDepthPipeline),
      m_lines(lines),
      m_resourceTable(resourceTable),
      m_recordTable(sdfRecordTable),
      m_sdfBaker(sdfBaker),
      m_volumePool(volumePool),
      m_frameGlobals(frameGlobals) {
  createCommandPool(device.physicalDevice(), surface);
  createCommandBuffer();
  createSyncObjects(imageCount);

  m_queries = talus::gpu::QueryPool(
    device,
    GPU_MARKS * talus::core::MAX_FRAMES_IN_FLIGHT
  );
}

Renderer::~Renderer() {
  vkDestroyCommandPool(m_device, m_commandPool, nullptr);
  for (size_t i = 0; i < talus::core::MAX_FRAMES_IN_FLIGHT; i++) {
    vkDestroySemaphore(m_device, m_imageAvailableSemaphores[i], nullptr);

    vkDestroyFence(m_device, m_inFlightFences[i], nullptr);
  }

  for (auto semaphore : m_renderFinishedSemaphores) {
    vkDestroySemaphore(m_device, semaphore, nullptr);
  }
}

void Renderer::createCommandPool(
  const VkPhysicalDevice& physicalDevice,
  const VkSurfaceKHR& surface
) {
  talus::gpu::QueueFamilyIndices queueFamilyIndices =
    talus::gpu::findQueueFamilies(physicalDevice, surface);

  VkCommandPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

  if (
    vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to create command pool");
  }
}

void Renderer::createCommandBuffer() {
  m_commandBuffers.resize(talus::core::MAX_FRAMES_IN_FLIGHT);
  VkCommandBufferAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.commandPool = m_commandPool;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandBufferCount = 1;

  allocInfo.commandBufferCount = (uint32_t)m_commandBuffers.size();

  if (
    vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data())
    != VK_SUCCESS
  ) {
    throw std::runtime_error("failed to allocate command buffers!");
  }
}

void Renderer::createSyncObjects(uint32_t imageCount) {
  m_imageAvailableSemaphores.resize(talus::core::MAX_FRAMES_IN_FLIGHT);
  m_renderFinishedSemaphores.resize(imageCount);
  m_inFlightFences.resize(talus::core::MAX_FRAMES_IN_FLIGHT);

  VkFenceCreateInfo fenceInfo{};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

  VkSemaphoreCreateInfo semaphoreInfo{};
  semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  for (size_t i = 0; i < talus::core::MAX_FRAMES_IN_FLIGHT; i++) {
    if (
      vkCreateFence(m_device, &fenceInfo, nullptr, &m_inFlightFences[i])
      != VK_SUCCESS
    ) {
      throw std::runtime_error("Failed to create fence for a frame");
    }

    if (
      vkCreateSemaphore(
        m_device,
        &semaphoreInfo,
        nullptr,
        &m_imageAvailableSemaphores[i]
      )
      != VK_SUCCESS
    ) {
      throw std::runtime_error(
        "Failed to create image available semaphore for a frame"
      );
    }
  }

  for (auto& semaphore : m_renderFinishedSemaphores) {
    if (
      vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &semaphore)
      != VK_SUCCESS
    ) {
      throw std::runtime_error(
        "Failed to create render finished semaphore for a frame"
      );
    }
  }
}

void Renderer::recordSelection(const Frame& frameContext) {
  const uint32_t selected = static_cast<uint32_t>(frameContext.selected);

  const std::array<VkDescriptorSet, 2> sharedSets = {
    m_frameGlobals.descriptorSet(m_currentFrame),
    m_resourceTable.descriptorSet(m_currentFrame)
  };

  vkCmdBindPipeline(
    m_commandBuffers[m_currentFrame],
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_meshDepthPipeline.pipeline()
  );

  vkCmdBindDescriptorSets(
    m_commandBuffers[m_currentFrame],
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_meshDepthPipeline.pipelineLayout(),
    0,
    static_cast<uint32_t>(sharedSets.size()),
    sharedSets.data(),
    0,
    nullptr
  );

  for (const auto& item : frameContext.meshRecords) {
    if (item.entity != selected || item.mesh >= frameContext.meshes.size()) {
      continue;
    }

    VkBuffer vertexBuffers[] = {
      frameContext.meshes[item.mesh].vertexBuffer().buffer()
    };
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(
      m_commandBuffers[m_currentFrame],
      0,
      1,
      vertexBuffers,
      offsets
    );
    vkCmdBindIndexBuffer(
      m_commandBuffers[m_currentFrame],
      frameContext.meshes[item.mesh].indexBuffer().buffer(),
      0,
      VK_INDEX_TYPE_UINT32
    );

    const talus::gpu::PushConstants pushConstants{item.model, item.material};

    vkCmdPushConstants(
      m_commandBuffers[m_currentFrame],
      m_meshDepthPipeline.pipelineLayout(),
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
      0,
      offsetof(talus::gpu::PushConstants, materialIndex)
        + sizeof(talus::gpu::PushConstants::materialIndex),
      &pushConstants
    );

    vkCmdDrawIndexed(
      m_commandBuffers[m_currentFrame],
      static_cast<uint32_t>(frameContext.meshes[item.mesh].indexCount()),
      1,
      0,
      0,
      0
    );
  }

  vkCmdBindPipeline(
    m_commandBuffers[m_currentFrame],
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_sdfDepthPipeline.pipeline()
  );

  vkCmdBindDescriptorSets(
    m_commandBuffers[m_currentFrame],
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_sdfDepthPipeline.pipelineLayout(),
    2,
    1,
    &m_recordTable.descriptorSet(m_currentFrame),
    0,
    nullptr
  );

  for (uint32_t slot = 0; slot != frameContext.sdfRecordCount; ++slot) {
    if (
      slot >= frameContext.sdfEntities.size()
      || static_cast<uint32_t>(frameContext.sdfEntities[slot]) != selected
    ) {
      continue;
    }

    const talus::gpu::PushConstants pushConstants{glm::mat4(1.0f), 0u, slot};

    vkCmdPushConstants(
      m_commandBuffers[m_currentFrame],
      m_sdfDepthPipeline.pipelineLayout(),
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
      0,
      offsetof(talus::gpu::PushConstants, recordIndex)
        + sizeof(talus::gpu::PushConstants::recordIndex),
      &pushConstants
    );

    vkCmdDraw(m_commandBuffers[m_currentFrame], 36, 1, 0, 0);
  }
}

void Renderer::runPasses(std::span<const ScenePass> passes, PassOrder order) {
  for (const ScenePass& pass : passes) {
    if (pass.order == order && pass.draw) {
      pass.draw(m_commandBuffers[m_currentFrame]);
    }
  }
}

void Renderer::recordCommandBuffer(
  const talus::gpu::SwapChain& swapChain,
  const Frame& frameContext,
  std::span<const ScenePass> passes
) {
  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

  if (
    vkBeginCommandBuffer(m_commandBuffers[m_currentFrame], &beginInfo)
    != VK_SUCCESS
  ) {
    throw std::runtime_error("failed to begin recording command buffer!");
  }

  const uint32_t base = m_currentFrame * GPU_MARKS;

  const auto stampGpu = [&](uint32_t at) {
    if (!m_queries.usable()) {
      return;
    }

    vkCmdWriteTimestamp2(
      m_commandBuffers[m_currentFrame],
      at == 0u ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
               : VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
      m_queries.handle(),
      base + at
    );
  };

  if (m_queries.usable()) {
    vkCmdResetQueryPool(
      m_commandBuffers[m_currentFrame],
      m_queries.handle(),
      base,
      GPU_MARKS
    );
  }

  stampGpu(0u);

  const VkExtent2D sceneExtent =
    m_target == nullptr ? swapChain.extent() : m_target->extent();

  const bool selecting = m_target != nullptr;

  const VkImage selectionImage =
    selecting ? m_target->selectionDepthImage() : VK_NULL_HANDLE;
  const VkImageView selectionView =
    selecting ? m_target->selectionDepthView() : VK_NULL_HANDLE;

  VkImageMemoryBarrier2 imageBarrier{};
  imageBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  imageBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imageBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  imageBarrier.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
  imageBarrier.srcAccessMask = 0;
  imageBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
  imageBarrier.dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
  imageBarrier.image = m_target == nullptr ? swapChain.image(m_imageIndex)
                                           : m_target->colourImage();
  imageBarrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  imageBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  imageBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

  VkImageMemoryBarrier2 depthBarrier{};
  depthBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  depthBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  depthBarrier.newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthBarrier.srcStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                              | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
  depthBarrier.srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  depthBarrier.dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                              | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
  depthBarrier.dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                               | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  depthBarrier.image = m_target == nullptr ? swapChain.depthImage().vkImage()
                                           : m_target->depthImage();
  depthBarrier.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
  depthBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  depthBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

  VkImageMemoryBarrier2 selectionBarrier = depthBarrier;
  selectionBarrier.image = selectionImage;

  std::array<VkImageMemoryBarrier2, 3> barriers = {
    imageBarrier,
    depthBarrier,
    selectionBarrier
  };
  VkDependencyInfo dependencyInfo{};
  dependencyInfo.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependencyInfo.imageMemoryBarrierCount = selecting ? 3 : 2;
  dependencyInfo.pImageMemoryBarriers = barriers.data();

  vkCmdPipelineBarrier2(m_commandBuffers[m_currentFrame], &dependencyInfo);

  stampGpu(1u);

  VkRenderingAttachmentInfo attachmentInfo{};
  attachmentInfo.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  attachmentInfo.imageView = m_target == nullptr
                               ? swapChain.imageView(m_imageIndex)
                               : m_target->colourView();
  attachmentInfo.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  attachmentInfo.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  attachmentInfo.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  attachmentInfo.clearValue = {{{0.0f, 0.0f, 0.0f, 1.0f}}};

  VkRenderingAttachmentInfo depthAttachmentInfo{};
  depthAttachmentInfo.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  depthAttachmentInfo.imageView = m_target == nullptr
                                    ? swapChain.depthImage().image().imageView()
                                    : m_target->depthView();
  depthAttachmentInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthAttachmentInfo.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAttachmentInfo.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  depthAttachmentInfo.clearValue.depthStencil = {1.0f, 0};

  VkRenderingInfo renderingInfo{};
  renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;

  renderingInfo.renderArea = {{0, 0}, sceneExtent};
  renderingInfo.layerCount = 1;
  renderingInfo.colorAttachmentCount = 1;
  renderingInfo.pColorAttachments = &attachmentInfo;
  renderingInfo.pDepthAttachment = &depthAttachmentInfo;

  vkCmdBeginRendering(m_commandBuffers[m_currentFrame], &renderingInfo);

  VkViewport viewport{};
  viewport.x = 0.0f;
  viewport.y = 0.0f;
  viewport.width = (float)sceneExtent.width;
  viewport.height = (float)sceneExtent.height;
  viewport.minDepth = 0.0f;
  viewport.maxDepth = 1.0f;
  vkCmdSetViewport(m_commandBuffers[m_currentFrame], 0, 1, &viewport);

  VkRect2D scissor{};
  scissor.offset = {0, 0};
  scissor.extent = sceneExtent;
  vkCmdSetScissor(m_commandBuffers[m_currentFrame], 0, 1, &scissor);

  runPasses(passes, PassOrder::BeforeScene);

  stampGpu(2u);

  const bool drawMeshes = frameContext.drawMeshes;
  const bool drawSdf = frameContext.drawSdf;

  const std::array<VkDescriptorSet, 2> sharedSets = {
    m_frameGlobals.descriptorSet(m_currentFrame),
    m_resourceTable.descriptorSet(m_currentFrame)
  };

  vkCmdBindDescriptorSets(
    m_commandBuffers[m_currentFrame],
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_meshPipeline.pipelineLayout(),
    0,
    static_cast<uint32_t>(sharedSets.size()),
    sharedSets.data(),
    0,
    nullptr
  );

  if (drawMeshes) {
    vkCmdBindPipeline(
      m_commandBuffers[m_currentFrame],
      VK_PIPELINE_BIND_POINT_GRAPHICS,
      m_meshPipeline.pipeline()
    );

    for (const auto& item : frameContext.meshRecords) {
      if (item.mesh >= frameContext.meshes.size()) {
        throw std::runtime_error(

          "A draw item names a mesh the frame does not carry"

        );
      }

      VkBuffer vertexBuffers[] = {
        frameContext.meshes[item.mesh].vertexBuffer().buffer()
      };
      VkDeviceSize offsets[] = {0};
      vkCmdBindVertexBuffers(
        m_commandBuffers[m_currentFrame],
        0,
        1,
        vertexBuffers,
        offsets
      );
      vkCmdBindIndexBuffer(
        m_commandBuffers[m_currentFrame],
        frameContext.meshes[item.mesh].indexBuffer().buffer(),
        0,
        VK_INDEX_TYPE_UINT32
      );

      const talus::gpu::PushConstants pushConstants{item.model, item.material};

      vkCmdPushConstants(
        m_commandBuffers[m_currentFrame],
        m_meshPipeline.pipelineLayout(),
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0,
        offsetof(talus::gpu::PushConstants, materialIndex)
          + sizeof(talus::gpu::PushConstants::materialIndex),
        &pushConstants
      );

      vkCmdDrawIndexed(
        m_commandBuffers[m_currentFrame],
        static_cast<uint32_t>(frameContext.meshes[item.mesh].indexCount()),
        1,
        0,
        0,
        0
      );
    }
  }

  stampGpu(3u);

  if (!drawSdf) {
    stampGpu(4u);
  }

  if (drawSdf) {
    const auto tableBegan = std::chrono::steady_clock::now();

    m_recordTable.update(
      m_currentFrame,
      frameContext.sdfRecordCount,
      lightDirection(frameContext.light)
    );
    m_recordTable.updatePrimitives(m_currentFrame, frameContext.primitives);

    m_timing.recordTableMs = 1000.0
                             * std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - tableBegan
                             )
                                 .count();
    m_timing.shadowBuildMs = m_recordTable.shadowBuildMs();
    m_timing.shadowIndices =
      static_cast<uint32_t>(m_recordTable.shadowGrid().indices().size());
    m_timing.shadowBinned = m_recordTable.shadowGrid().uniform().cell.z > 0.0f;

    stampGpu(4u);

    const auto bakeBegan = std::chrono::steady_clock::now();

    for (uint32_t slot : m_recordTable.bakeDispatch()) {
      ++m_timing.bakes;
      const SdfRecord& gpu = m_recordTable.record(slot);

      const uint32_t volumeIndex = gpu.resolutionVolumeIndex.w;

      VolumeImages images = m_volumePool.images(volumeIndex);

      m_sdfBaker.bake(gpu, m_currentFrame, images);
    }

    m_timing.bakeMs = 1000.0
                      * std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - bakeBegan
                      )
                          .count();

    vkCmdBindPipeline(
      m_commandBuffers[m_currentFrame],
      VK_PIPELINE_BIND_POINT_GRAPHICS,
      m_sdfPipeline.pipeline()
    );

    vkCmdBindDescriptorSets(
      m_commandBuffers[m_currentFrame],
      VK_PIPELINE_BIND_POINT_GRAPHICS,
      m_sdfPipeline.pipelineLayout(),
      2,
      1,
      &m_recordTable.descriptorSet(m_currentFrame),
      0,
      nullptr
    );

    for (uint32_t slot = 0; slot != frameContext.sdfRecordCount; ++slot) {
      const talus::gpu::PushConstants pushConstants{glm::mat4(1.0f), 0u, slot};

      vkCmdPushConstants(
        m_commandBuffers[m_currentFrame],
        m_sdfPipeline.pipelineLayout(),
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
        0,
        offsetof(talus::gpu::PushConstants, recordIndex)
          + sizeof(talus::gpu::PushConstants::recordIndex),
        &pushConstants
      );

      vkCmdDraw(m_commandBuffers[m_currentFrame], 36, 1, 0, 0);
    }
  }

  stampGpu(5u);

  vkCmdEndRendering(m_commandBuffers[m_currentFrame]);

  VkImageMemoryBarrier2 depthReadable{};
  depthReadable.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  depthReadable.oldLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthReadable.newLayout = VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL;
  depthReadable.srcStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                               | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
  depthReadable.srcAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
  depthReadable.dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT
                               | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT
                               | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
  depthReadable.dstAccessMask = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT
                                | VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
  depthReadable.image = m_target == nullptr ? swapChain.depthImage().vkImage()
                                            : m_target->depthImage();
  depthReadable.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
  depthReadable.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  depthReadable.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

  VkDependencyInfo readDepth{};
  readDepth.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  readDepth.imageMemoryBarrierCount = 1;
  readDepth.pImageMemoryBarriers = &depthReadable;

  vkCmdPipelineBarrier2(m_commandBuffers[m_currentFrame], &readDepth);

  stampGpu(6u);

  if (selecting) {
    VkRenderingAttachmentInfo selectionAttachment{};
    selectionAttachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    selectionAttachment.imageView = selectionView;
    selectionAttachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    selectionAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    selectionAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    selectionAttachment.clearValue.depthStencil = {1.0f, 0};

    VkRenderingInfo maskInfo{};
    maskInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    maskInfo.renderArea = {{0, 0}, sceneExtent};
    maskInfo.layerCount = 1;
    maskInfo.colorAttachmentCount = 0;
    maskInfo.pDepthAttachment = &selectionAttachment;

    vkCmdBeginRendering(m_commandBuffers[m_currentFrame], &maskInfo);

    vkCmdSetViewport(m_commandBuffers[m_currentFrame], 0, 1, &viewport);
    vkCmdSetScissor(m_commandBuffers[m_currentFrame], 0, 1, &scissor);

    if (frameContext.selected != talus::objectmodel::INVALID_ENTITY) {
      recordSelection(frameContext);
    }

    vkCmdEndRendering(m_commandBuffers[m_currentFrame]);

    VkImageMemoryBarrier2 selectionReadable = depthReadable;
    selectionReadable.image = selectionImage;

    VkDependencyInfo readSelection{};
    readSelection.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    readSelection.imageMemoryBarrierCount = 1;
    readSelection.pImageMemoryBarriers = &selectionReadable;

    vkCmdPipelineBarrier2(m_commandBuffers[m_currentFrame], &readSelection);
  }

  stampGpu(7u);

  attachmentInfo.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
  depthAttachmentInfo.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
  depthAttachmentInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL;

  renderingInfo.colorAttachmentCount = 1;
  renderingInfo.pColorAttachments = &attachmentInfo;

  vkCmdBeginRendering(m_commandBuffers[m_currentFrame], &renderingInfo);

  vkCmdSetViewport(m_commandBuffers[m_currentFrame], 0, 1, &viewport);
  vkCmdSetScissor(m_commandBuffers[m_currentFrame], 0, 1, &scissor);

  runPasses(passes, PassOrder::AfterScene);

  m_lines.record(
    m_commandBuffers[m_currentFrame],
    m_frameGlobals.descriptorSet(m_currentFrame),
    m_currentFrame
  );

  vkCmdEndRendering(m_commandBuffers[m_currentFrame]);

  stampGpu(8u);

  if (m_target != nullptr) {
    std::array<VkImageMemoryBarrier2, 2> toTransfer{};

    toTransfer[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    toTransfer[0].oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    toTransfer[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    toTransfer[0].srcStageMask =
      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
    toTransfer[0].srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
    toTransfer[0].dstStageMask = VK_PIPELINE_STAGE_2_BLIT_BIT;
    toTransfer[0].dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
    toTransfer[0].image = m_target->colourImage();
    toTransfer[0].subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    toTransfer[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    toTransfer[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    toTransfer[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
    toTransfer[1].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    toTransfer[1].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    toTransfer[1].srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    toTransfer[1].srcAccessMask = 0;
    toTransfer[1].dstStageMask = VK_PIPELINE_STAGE_2_BLIT_BIT;
    toTransfer[1].dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    toTransfer[1].image = swapChain.image(m_imageIndex);
    toTransfer[1].subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    toTransfer[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    toTransfer[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    VkDependencyInfo intoBlit{};
    intoBlit.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    intoBlit.imageMemoryBarrierCount = 2;
    intoBlit.pImageMemoryBarriers = toTransfer.data();

    vkCmdPipelineBarrier2(m_commandBuffers[m_currentFrame], &intoBlit);

    m_target->blitTo(
      m_commandBuffers[m_currentFrame],
      swapChain.image(m_imageIndex),
      swapChain.extent()
    );
  }

  stampGpu(9u);

  VkImageMemoryBarrier2 imageMemoryBarrier2Present{};
  imageMemoryBarrier2Present.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  imageMemoryBarrier2Present.oldLayout =
    m_target == nullptr ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
                        : VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  imageMemoryBarrier2Present.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  imageMemoryBarrier2Present.srcStageMask =
    m_target == nullptr ? VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT
                        : VK_PIPELINE_STAGE_2_BLIT_BIT;
  imageMemoryBarrier2Present.srcAccessMask =
    m_target == nullptr ? VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT
                        : VK_ACCESS_2_TRANSFER_WRITE_BIT;
  imageMemoryBarrier2Present.dstStageMask = VK_PIPELINE_STAGE_2_NONE;
  imageMemoryBarrier2Present.dstAccessMask = 0;
  imageMemoryBarrier2Present.image = swapChain.image(m_imageIndex);
  imageMemoryBarrier2Present.subresourceRange =
    {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
  imageMemoryBarrier2Present.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  imageMemoryBarrier2Present.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

  VkDependencyInfo dependencyInfoPresent{};
  dependencyInfoPresent.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependencyInfoPresent.imageMemoryBarrierCount = 1;
  dependencyInfoPresent.pImageMemoryBarriers = &imageMemoryBarrier2Present;

  vkCmdPipelineBarrier2(
    m_commandBuffers[m_currentFrame],
    &dependencyInfoPresent
  );

  stampGpu(10u);

  stampGpu(11u);

  if (vkEndCommandBuffer(m_commandBuffers[m_currentFrame]) != VK_SUCCESS) {
    throw std::runtime_error("failed to record command buffer!");
  }
}

uint32_t Renderer::currentFrame() const noexcept {
  return m_currentFrame;
}

void Renderer::useTarget(SceneTarget* target) noexcept {
  m_target = target;
}

void Renderer::submittedAt(uint64_t frame) noexcept {
  m_submitFrame = frame;
}

void Renderer::readTimestamps(uint32_t slot) {
  if (!m_queries.usable() || m_slotFrame[slot] == 0u) {
    return;
  }

  std::array<uint64_t, GPU_MARKS * 2u> raw{};

  const VkResult read = vkGetQueryPoolResults(
    m_device,
    m_queries.handle(),
    slot * GPU_MARKS,
    GPU_MARKS,
    sizeof(raw),
    raw.data(),
    2u * sizeof(uint64_t),
    VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT
  );

  if (read != VK_SUCCESS && read != VK_NOT_READY) {
    throw std::runtime_error("Failed to read the timestamp queries");
  }

  for (uint32_t at = 0u; at != GPU_MARKS; ++at) {
    if (raw[at * 2u + 1u] == 0u) {
      return;
    }
  }

  for (uint32_t at = 0u; at + 1u != GPU_MARKS; ++at) {
    const uint64_t from = raw[at * 2u];
    const uint64_t to = raw[(at + 1u) * 2u];

    m_timing.gpuSegmentMs[at] =
      to > from ? double(to - from) * m_queries.period() * 1.0e-6 : 0.0;
  }

  const uint64_t first = raw[0];
  const uint64_t last = raw[(GPU_MARKS - 1u) * 2u];

  m_timing.gpuTotalMs =
    last > first ? double(last - first) * m_queries.period() * 1.0e-6 : 0.0;
  m_timing.gpuFrame = m_slotFrame[slot];
  m_timing.gpuValid = true;
}

bool Renderer::drawFrame(
  const talus::gpu::SwapChain& swapChain,
  const Frame& frameContext,
  std::span<const ScenePass> passes,
  const std::function<void(VkImage)>& onFrameReady
) {
  const auto stamp = []() {
    return std::chrono::steady_clock::now();
  };
  const auto since = [](auto from) {
    return 1000.0
           * std::chrono::duration<double>(
               std::chrono::steady_clock::now() - from
           )
               .count();
  };

  m_timing = Timing{};

  auto mark = stamp();

  if (
    vkWaitForFences(
      m_device,
      1,
      &m_inFlightFences[m_currentFrame],
      VK_TRUE,
      UINT64_MAX
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed waiting for the in flight fence");
  }

  m_timing.fenceWaitMs = since(mark);

  readTimestamps(m_currentFrame);

  mark = stamp();

  VkResult result = vkAcquireNextImageKHR(
    m_device,
    swapChain.handle(),
    UINT64_MAX,
    m_imageAvailableSemaphores[m_currentFrame],
    VK_NULL_HANDLE,
    &m_imageIndex
  );

  m_timing.acquireMs = since(mark);

  if (result == VK_ERROR_OUT_OF_DATE_KHR) {
    return true;
  } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
    throw std::runtime_error("Failed to acquire swap chain image");
  }

  if (
    vkResetFences(m_device, 1, &m_inFlightFences[m_currentFrame]) != VK_SUCCESS
  ) {
    throw std::runtime_error("Failed to reset the in flight fence");
  }

  if (vkResetCommandBuffer(m_commandBuffers[m_currentFrame], 0) != VK_SUCCESS) {
    throw std::runtime_error("Failed to reset the command buffer");
  }

  m_lines.upload(
    m_currentFrame,
    frameContext.debugTested,
    frameContext.debugUntested,
    frameContext.overlayLines
  );

  const glm::mat4 cameraViewProj = frameContext.proj * frameContext.view;

  const CameraUniform camera{
    frameContext.view,
    frameContext.proj,
    cameraViewProj,
    glm::inverse(cameraViewProj),
    frameContext.cameraPos
  };

  const SceneCounts counts{frameContext.sdfRecordCount};

  LightUniform light{};

  if (frameContext.light.has_value()) {
    light.direction = glm::vec4(
      talus::objectmodel::toLight(*frameContext.light),
      frameContext.light->ambient
    );
  }

  if (frameContext.environment.has_value()) {
    const talus::objectmodel::Environment& environment =
      *frameContext.environment;

    light.skyTop = glm::vec4(environment.skyTop, environment.skyCurve);
    light.skyHorizon =
      glm::vec4(environment.skyHorizon, environment.groundCurve);
    light.groundBottom = glm::vec4(environment.groundBottom, 0.0f);
    light.shading = glm::vec4(
      environment.ambientEnergy,
      environment.occlusionStrength,
      environment.exposure,
      1.0f
    );
  }

  m_frameGlobals.update(
    m_currentFrame,
    camera,
    frameContext.march,
    counts,
    light
  );

  mark = stamp();

  recordCommandBuffer(swapChain, frameContext, passes);

  m_timing.recordMs = since(mark);
  mark = stamp();

  VkSubmitInfo2 submitInfo2{};
  submitInfo2.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;

  VkSemaphoreSubmitInfo waitSubmitInfo{};
  waitSubmitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
  waitSubmitInfo.semaphore = m_imageAvailableSemaphores[m_currentFrame];
  waitSubmitInfo.stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;

  VkCommandBufferSubmitInfo commandBufferInfo{};
  commandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
  commandBufferInfo.commandBuffer = m_commandBuffers[m_currentFrame];

  VkSemaphoreSubmitInfo signalSubmitInfo{};
  signalSubmitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
  signalSubmitInfo.semaphore = m_renderFinishedSemaphores.at(m_imageIndex);
  signalSubmitInfo.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

  submitInfo2.waitSemaphoreInfoCount = 1;
  submitInfo2.pWaitSemaphoreInfos = &waitSubmitInfo;
  submitInfo2.commandBufferInfoCount = 1;
  submitInfo2.pCommandBufferInfos = &commandBufferInfo;
  submitInfo2.signalSemaphoreInfoCount = 1;
  submitInfo2.pSignalSemaphoreInfos = &signalSubmitInfo;

  if (
    vkQueueSubmit2(
      m_graphicsQueue,
      1,
      &submitInfo2,
      m_inFlightFences[m_currentFrame]
    )
    != VK_SUCCESS
  ) {
    throw std::runtime_error("failed to submit draw command buffer!");
  }

  m_timing.submitMs = since(mark);
  mark = stamp();

  if (onFrameReady) {
    if (vkDeviceWaitIdle(m_device) != VK_SUCCESS) {
      throw std::runtime_error("Failed waiting for the frame to be readable");
    }

    onFrameReady(swapChain.image(m_imageIndex));
  }

  VkSemaphore signalSemaphores[] = {
    m_renderFinishedSemaphores.at(m_imageIndex)
  };
  VkPresentInfoKHR presentInfo{};
  presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

  presentInfo.waitSemaphoreCount = 1;
  presentInfo.pWaitSemaphores = signalSemaphores;

  VkSwapchainKHR swapChains[] = {swapChain.handle()};
  presentInfo.swapchainCount = 1;
  presentInfo.pSwapchains = swapChains;
  presentInfo.pImageIndices = &m_imageIndex;
  presentInfo.pResults = nullptr;

  result = vkQueuePresentKHR(m_presentQueue, &presentInfo);

  m_timing.presentMs = since(mark);

  m_slotFrame[m_currentFrame] = m_submitFrame;
  if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
    return true;
  } else if (result != VK_SUCCESS) {
    throw std::runtime_error("Failed to present swap chain image");
  }

  m_currentFrame = (m_currentFrame + 1) % talus::core::MAX_FRAMES_IN_FLIGHT;

  return false;
}

}
