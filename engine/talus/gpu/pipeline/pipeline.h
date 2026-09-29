#pragma once

#include <talus/gpu/swapchain/swapchain.h>
#include <talus/gpu/shadermodule/shadermodule.h>

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>
#include <span>
#include <filesystem>

namespace talus::gpu {

struct PushConstants {
  glm::mat4 model;
  uint32_t materialIndex;
  uint32_t recordIndex;
};

static_assert(
  offsetof(PushConstants, model) == 0,
  "The push constant block must match its declaration in both mesh shaders"
);
static_assert(
  offsetof(PushConstants, materialIndex) == 64,
  "The push constant block must match its declaration in both mesh shaders"
);
static_assert(
  offsetof(PushConstants, recordIndex) == 68,
  "The push constant block must match its declaration in both mesh shaders"
);

static_assert(
  sizeof(PushConstants) == 80,
  "PushConstants must have one layout in every translation unit; only the "
  "first 72 bytes are ever pushed"
);

struct GridPush {
  glm::vec4 primary;
  glm::vec4 secondary;
  glm::vec4 axisColourU;
  glm::vec4 axisColourV;
  glm::vec4 normal;
  glm::vec4 fade;
};

static_assert(
  offsetof(GridPush, primary) == 0,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  offsetof(GridPush, secondary) == 16,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  offsetof(GridPush, axisColourU) == 32,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  offsetof(GridPush, axisColourV) == 48,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  offsetof(GridPush, normal) == 64,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  offsetof(GridPush, fade) == 80,
  "GridPush must match its push constant block in both grid shaders"
);
static_assert(
  sizeof(GridPush) == 96,
  "GridPush must match its push constant block in both grid shaders"
);

enum class PipelineType {
  Mesh,
  Sdf,
  Sky
};

struct PipelineShaders {
  const char* vertexSource;
  const char* fragmentSource;
  const char* vertexSpirv;
  const char* fragmentSpirv;
};

[[nodiscard]] PipelineShaders pipelineShaders(PipelineType type);

[[nodiscard]] std::vector<VkPushConstantRange> pushConstantRanges(
  PipelineType type
);

struct PipelineDescription {
  std::string name;
  std::string vertexSource;
  std::string fragmentSource;
  std::string vertexSpirv;
  std::string fragmentSpirv;

  std::vector<VkVertexInputBindingDescription> bindings{};
  std::vector<VkVertexInputAttributeDescription> attributes{};

  VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  VkCullModeFlags cullMode = VK_CULL_MODE_NONE;

  VkBool32 depthTest = VK_TRUE;
  VkBool32 depthWrite = VK_TRUE;
  VkBool32 blend = VK_FALSE;

  uint32_t colourAttachments = 1;

  std::vector<VkPushConstantRange> pushConstants{};
};

[[nodiscard]] PipelineDescription describeShaders(
  const char* name,
  const char* stem
);

[[nodiscard]] PipelineDescription describe(PipelineType type);

[[nodiscard]] PipelineDescription describeDepthOnly(PipelineType type);

struct PipelineConfig {
  std::string vert;
  std::string frag;
  std::string vertexShader;
  std::string fragmentShader;

  std::vector<VkVertexInputBindingDescription> bindingDescriptions{};
  std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};

  VkCullModeFlags cullMode = VK_CULL_MODE_NONE;

  VkBool32 depthTestEnable = VK_TRUE;
  VkBool32 depthWriteEnable = VK_TRUE;
  VkBool32 blendEnable = VK_FALSE;

  VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

  std::vector<VkDescriptorSetLayout> setLayouts{};
  std::vector<VkPushConstantRange> pushConstantRanges{};
};

class Pipeline {
public:
  Pipeline(
    PipelineType type,
    const VkDevice& device,
    const std::vector<VkDescriptorSetLayout>& setLayouts,
    const SwapChain& swapChain,
    std::span<const VkVertexInputBindingDescription> bindingDescriptions = {},
    std::span<const VkVertexInputAttributeDescription> attributeDescriptions =
      {}
  );
  Pipeline(
    PipelineDescription description,
    const VkDevice& device,
    const std::vector<VkDescriptorSetLayout>& setLayouts,
    const SwapChain& swapChain
  );
  Pipeline(Pipeline const&) = delete;
  Pipeline& operator=(Pipeline const&) = delete;
  ~Pipeline();

  const VkPipeline& pipeline() const noexcept;
  const VkPipelineLayout& pipelineLayout() const noexcept;

  void reload();
  [[nodiscard]]
  bool sourcesChanged() const;

private:
  void makeConfig();
  VkPipeline buildPipeline();
  VkPipelineLayout buildPipelineLayout();

  void create();
  void destroy() noexcept;

  static std::filesystem::file_time_type newestIncludeTime();

  VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
  VkPipeline m_pipeline = VK_NULL_HANDLE;

  VkDevice m_device = VK_NULL_HANDLE;
  VkFormat m_swapChainImageFormat;
  PipelineConfig m_config{};

  std::vector<VkDescriptorSetLayout> m_setLayouts;

  std::vector<VkVertexInputBindingDescription> m_bindingDescriptions;
  std::vector<VkVertexInputAttributeDescription> m_attributeDescriptions;
  VkFormat m_depthImageFormat;
  PipelineDescription m_description;

  std::filesystem::file_time_type m_vertTime;
  std::filesystem::file_time_type m_fragTime;
  std::filesystem::file_time_type m_includeTime;
};

}
