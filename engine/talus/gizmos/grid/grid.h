#pragma once

#include <talus/gpu/buffer/buffer.h>
#include <talus/gpu/device/device.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/gpu/swapchain/swapchain.h>
#include <talus/view/grid/gridstyle.h>

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace talus::gizmos {

struct GridVertex {
  glm::vec3 position;
  float kind;

  static VkVertexInputBindingDescription bindingDescription();
  static std::vector<VkVertexInputAttributeDescription> attributeDescriptions();
};

static_assert(
  offsetof(GridVertex, position) == 0,
  "GridVertex must keep the stride the grid pipeline binds"
);
static_assert(
  offsetof(GridVertex, kind) == 16,
  "GridVertex must keep the stride the grid pipeline binds"
);
static_assert(
  sizeof(GridVertex) == 32,
  "GridVertex must keep the stride the grid pipeline binds"
);

class Grid {
public:
  Grid(
    const talus::gpu::Device& device,
    const talus::gpu::SwapChain& swapChain,
    std::vector<VkDescriptorSetLayout> setLayouts,
    const talus::view::GridPlane& plane = {},
    const talus::view::GridStyle& style = {}
  );

  Grid(Grid const&) = delete;
  Grid& operator=(Grid const&) = delete;

  [[nodiscard]] static talus::gpu::PipelineDescription describe();

  void update(const glm::vec3& cameraPosition);

  void record(VkCommandBuffer commands, VkDescriptorSet globals) const;

private:
  void rebuild();

  [[nodiscard]] float planeDistance(const glm::vec3& position) const;

  const talus::gpu::Device& m_device;

  talus::view::GridPlane m_plane;
  talus::view::GridStyle m_style;

  talus::gpu::Pipeline m_pipeline;

  talus::gpu::Buffer m_vertices;

  uint32_t m_vertexCount = 0;
  uint32_t m_capacity = 0;

  float m_level = 0.0f;
  float m_decimals = 0.0f;
  float m_floored = 0.0f;
  float m_radius = 0.0f;

  float m_centreU = 0.0f;
  float m_centreV = 0.0f;

  bool m_built = false;
};

}
