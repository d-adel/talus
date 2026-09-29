#pragma once

#include <talus/core/config/config.h>
#include <talus/debugdraw/debugdraw/debugdraw.h>
#include <talus/gpu/buffer/buffer.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/gpu/swapchain/swapchain.h>

#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace talus::renderer {

class Lines {
public:
  Lines(
    const talus::gpu::Device& device,
    const talus::gpu::SwapChain& swapChain,
    std::vector<VkDescriptorSetLayout> setLayouts
  );

  Lines(const Lines&) = delete;
  Lines& operator=(const Lines&) = delete;

  [[nodiscard]] static talus::gpu::PipelineDescription describe(
    bool depthTested
  );

  void upload(
    uint32_t frame,
    std::span<const talus::debugdraw::LineVertex> tested,
    std::span<const talus::debugdraw::LineVertex> untested,
    std::span<const talus::debugdraw::LineVertex> overlay
  );

  void record(
    VkCommandBuffer commands,
    VkDescriptorSet globals,
    uint32_t frame
  ) const;

private:
  struct Slot {
    talus::gpu::Buffer buffer;
    size_t capacity = 0u;
    uint32_t tested = 0u;
    uint32_t untested = 0u;
  };

  const talus::gpu::Device& m_device;

  talus::gpu::Pipeline m_tested;
  talus::gpu::Pipeline m_untested;

  std::array<Slot, talus::core::MAX_FRAMES_IN_FLIGHT> m_slots{};
};

}
