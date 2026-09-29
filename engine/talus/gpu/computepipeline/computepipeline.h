#pragma once

#include <talus/gpu/device/device.h>

#include <vulkan/vulkan.h>

#include <string>
#include <vector>

namespace talus::gpu {

class ComputePipeline {
public:
  ComputePipeline(
    const VkDevice& device,
    const std::string& spirvPath,
    const std::vector<VkDescriptorSetLayout>& setLayouts,
    const std::vector<VkPushConstantRange>& pushConstantRanges = {}
  );

  ComputePipeline(const ComputePipeline&) = delete;
  ComputePipeline& operator=(const ComputePipeline&) = delete;
  ComputePipeline(ComputePipeline&&) = delete;
  ComputePipeline& operator=(ComputePipeline&&) = delete;

  ~ComputePipeline();

  const VkPipeline& pipeline() const noexcept;
  const VkPipelineLayout& pipelineLayout() const noexcept;

private:
  void destroy() noexcept;

  VkDevice m_device = VK_NULL_HANDLE;
  VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
  VkPipeline m_pipeline = VK_NULL_HANDLE;
};

}
