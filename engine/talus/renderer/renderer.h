#pragma once

#include <vulkan/vulkan.h>
#include <talus/gpu/swapchain/swapchain.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/gpu/querypool/querypool.h>
#include <talus/renderer/resourcetable/resourcetable.h>
#include <talus/renderer/frameglobals/frameglobals.h>
#include <talus/renderer/frame/frame.h>
#include <talus/renderer/lines/lines.h>
#include <talus/renderer/scenetarget/scenetarget.h>
#include <talus/renderer/scenepass/scenepass.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>
#include <talus/renderer/volumepool/volumepool.h>
#include <talus/renderer/sdfbaker/sdfbaker.h>

#include <talus/core/config/config.h>

#include <array>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace talus::renderer {

class Renderer {
public:
  Renderer(
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
  );
  Renderer(Renderer const&) = delete;
  Renderer& operator=(Renderer const&) = delete;
  ~Renderer();

  void useTarget(SceneTarget* target) noexcept;

  [[nodiscard]] uint32_t currentFrame() const noexcept;

  static constexpr uint32_t GPU_MARKS = 12u;

  struct Timing {
    double gpuTotalMs = 0.0;
    std::array<double, GPU_MARKS - 1u> gpuSegmentMs{};
    uint64_t gpuFrame = 0u;
    bool gpuValid = false;
    double fenceWaitMs = 0.0;
    double acquireMs = 0.0;
    double recordMs = 0.0;
    double bakeMs = 0.0;
    double recordTableMs = 0.0;
    double shadowBuildMs = 0.0;
    double submitMs = 0.0;
    double presentMs = 0.0;
    uint32_t bakes = 0u;
    uint32_t shadowIndices = 0u;
    bool shadowBinned = false;
  };

  [[nodiscard]] const Timing& timing() const noexcept {
    return m_timing;
  }

  [[nodiscard]]
  bool drawFrame(
    const talus::gpu::SwapChain& swapChain,
    const Frame& frameContext,
    std::span<const ScenePass> passes = {},
    const std::function<void(VkImage)>& onFrameReady = {}
  );

  void submittedAt(uint64_t frame) noexcept;

private:
  Timing m_timing{};
  talus::gpu::QueryPool m_queries;
  std::array<uint64_t, talus::core::MAX_FRAMES_IN_FLIGHT> m_slotFrame{};
  uint64_t m_submitFrame = 0u;

  void readTimestamps(uint32_t slot);

  void createCommandPool(
    const VkPhysicalDevice& physicalDevice,
    const VkSurfaceKHR& surface
  );
  void createCommandBuffer();

  void createSyncObjects(uint32_t imageCount);
  void recordCommandBuffer(
    const talus::gpu::SwapChain& swapChain,
    const Frame& frameContext,
    std::span<const ScenePass> passes
  );

  void runPasses(std::span<const ScenePass> passes, PassOrder order);

  void recordSelection(const Frame& frameContext);

  VkCommandPool m_commandPool;
  std::vector<VkCommandBuffer> m_commandBuffers;
  std::vector<VkSemaphore> m_imageAvailableSemaphores;
  std::vector<VkSemaphore> m_renderFinishedSemaphores;
  std::vector<VkFence> m_inFlightFences;
  VkQueue m_graphicsQueue = VK_NULL_HANDLE;
  VkQueue m_presentQueue = VK_NULL_HANDLE;

  VkDevice m_device;
  uint32_t m_imageIndex = 0;
  uint32_t m_currentFrame = 0;

  SceneTarget* m_target = nullptr;

  const talus::gpu::Pipeline& m_meshPipeline;
  const talus::gpu::Pipeline& m_sdfPipeline;
  const talus::gpu::Pipeline& m_meshDepthPipeline;
  const talus::gpu::Pipeline& m_sdfDepthPipeline;
  Lines& m_lines;
  const ResourceTable& m_resourceTable;
  SdfRecordTable& m_recordTable;
  const SdfBaker& m_sdfBaker;
  const VolumePool& m_volumePool;
  FrameGlobals& m_frameGlobals;
};

}
