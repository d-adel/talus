#include "computepipeline.ih"

namespace talus::gpu {

ComputePipeline::ComputePipeline(
  const VkDevice& device,
  const std::string& spirvPath,
  const std::vector<VkDescriptorSetLayout>& setLayouts,
  const std::vector<VkPushConstantRange>& pushConstantRanges
)
    : m_device(device) {
  try {
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
    layoutInfo.pSetLayouts = setLayouts.data();
    layoutInfo.pushConstantRangeCount =
      static_cast<uint32_t>(pushConstantRanges.size());
    layoutInfo.pPushConstantRanges = pushConstantRanges.data();

    if (
      vkCreatePipelineLayout(m_device, &layoutInfo, nullptr, &m_pipelineLayout)
      != VK_SUCCESS
    ) {
      throw std::runtime_error("Failed to create compute pipeline layout");
    }

    ShaderModule module(m_device, spirvPath);

    VkPipelineShaderStageCreateInfo stageInfo{};
    stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = module.handle();
    stageInfo.pName = "main";

    VkComputePipelineCreateInfo pipelineInfo{};
    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    pipelineInfo.stage = stageInfo;
    pipelineInfo.layout = m_pipelineLayout;

    if (
      vkCreateComputePipelines(
        m_device,
        VK_NULL_HANDLE,
        1,
        &pipelineInfo,
        nullptr,
        &m_pipeline
      )
      != VK_SUCCESS
    ) {
      throw std::runtime_error("Failed to create compute pipeline");
    }
  } catch (...) {
    destroy();
    throw;
  }
}

ComputePipeline::~ComputePipeline() {
  destroy();
}

const VkPipeline& ComputePipeline::pipeline() const noexcept {
  return m_pipeline;
}

const VkPipelineLayout& ComputePipeline::pipelineLayout() const noexcept {
  return m_pipelineLayout;
}

void ComputePipeline::destroy() noexcept {
  vkDestroyPipeline(m_device, m_pipeline, nullptr);
  m_pipeline = VK_NULL_HANDLE;

  vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
  m_pipelineLayout = VK_NULL_HANDLE;
}

}
