#pragma once

#include <talus/gpu/descriptorgroup/descriptorgroup.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/gpu/sampler/sampler.h>
#include <talus/gpu/swapchain/swapchain.h>

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace talus::gizmos {

struct OutlinePush {
  glm::uvec4 radius;
  glm::vec4 colour;
  glm::vec4 dimmed;
};

static_assert(
  offsetof(OutlinePush, radius) == 0,
  "OutlinePush must match its push constant block in the outline shader"
);
static_assert(
  offsetof(OutlinePush, colour) == 16,
  "OutlinePush must match its push constant block in the outline shader"
);
static_assert(
  offsetof(OutlinePush, dimmed) == 32,
  "OutlinePush must match its push constant block in the outline shader"
);

class Outline {
public:
  Outline(
    const talus::gpu::Device& device,
    const talus::gpu::SwapChain& swapChain,
    VkImageView selectionDepth,
    VkImageView sceneDepth
  );

  Outline(const Outline&) = delete;
  Outline& operator=(const Outline&) = delete;

  [[nodiscard]] static talus::gpu::PipelineDescription describe();

  void retarget(VkImageView selectionDepth, VkImageView sceneDepth);

  void show() noexcept;

  void hide() noexcept;

  [[nodiscard]] bool shown() const noexcept;

  void record(VkCommandBuffer commands) const;

private:
  talus::gpu::Sampler m_sampler;

  talus::gpu::DescriptorGroup m_group;

  talus::gpu::Pipeline m_pipeline;

  bool m_shown = false;
};

}
