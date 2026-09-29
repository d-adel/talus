#include "pipeline.ih"

namespace talus::gpu {

static bool compileShader(
  const std::string& source,
  const std::string& output
) {
  const std::string command = "\"\"" GLSLC_PATH "\" " GLSLC_DEFINES
                              " -I\"" GLSLC_INCLUDE_DIR "\" \""
                              + source + "\" -o \"" + output + "\"\"";

  return std::system(command.c_str()) == 0;
}

PipelineShaders pipelineShaders(PipelineType type) {
  switch (type) {
    case PipelineType::Mesh:
      return {
        SHADER_SOURCE_DIR "/mesh-shader.vert",
        SHADER_SOURCE_DIR "/mesh-shader.frag",
        "shaders/mesh-shader.vert.spv",
        "shaders/mesh-shader.frag.spv"
      };
    case PipelineType::Sdf:
      return {
        SHADER_SOURCE_DIR "/sdf-shader.vert",
        SHADER_SOURCE_DIR "/sdf-shader.frag",
        "shaders/sdf-shader.vert.spv",
        "shaders/sdf-shader.frag.spv"
      };
    case PipelineType::Sky:
      return {
        SHADER_SOURCE_DIR "/sky-shader.vert",
        SHADER_SOURCE_DIR "/sky-shader.frag",
        "shaders/sky-shader.vert.spv",
        "shaders/sky-shader.frag.spv"
      };
  }

  return {};
}

std::vector<VkPushConstantRange> pushConstantRanges(PipelineType type) {
  VkPushConstantRange range{};
  range.offset = 0;
  range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

  switch (type) {
    case PipelineType::Mesh:
    case PipelineType::Sdf:
      range.size = offsetof(PushConstants, recordIndex)
                   + sizeof(PushConstants::recordIndex);

      return {range};
    case PipelineType::Sky:
      return {};
  }

  return {};
}

Pipeline::Pipeline(
  PipelineType type,
  const VkDevice& device,
  const std::vector<VkDescriptorSetLayout>& setLayouts,
  const SwapChain& swapChain,
  std::span<const VkVertexInputBindingDescription> bindingDescriptions,
  std::span<const VkVertexInputAttributeDescription> attributeDescriptions
)
    : m_device(device),
      m_swapChainImageFormat(swapChain.imageFormat()),
      m_setLayouts(setLayouts),
      m_bindingDescriptions(
        bindingDescriptions.begin(),
        bindingDescriptions.end()
      ),
      m_attributeDescriptions(
        attributeDescriptions.begin(),
        attributeDescriptions.end()
      ),
      m_depthImageFormat(swapChain.depthImage().format()),
      m_description(describe(type)) {
  if (!m_bindingDescriptions.empty()) {
    m_description.bindings = m_bindingDescriptions;
    m_description.attributes = m_attributeDescriptions;
  }

  try {
    create();
  } catch (...) {
    destroy();
    throw;
  }
}

Pipeline::Pipeline(
  PipelineDescription description,
  const VkDevice& device,
  const std::vector<VkDescriptorSetLayout>& setLayouts,
  const SwapChain& swapChain
)
    : m_device(device),
      m_swapChainImageFormat(swapChain.imageFormat()),
      m_setLayouts(setLayouts),
      m_bindingDescriptions(description.bindings),
      m_attributeDescriptions(description.attributes),
      m_depthImageFormat(swapChain.depthImage().format()),
      m_description(std::move(description)) {
  try {
    create();
  } catch (...) {
    destroy();
    throw;
  }
}

Pipeline::~Pipeline() {
  destroy();
}

void Pipeline::destroy() noexcept {
  vkDestroyPipeline(m_device, m_pipeline, nullptr);
  m_pipeline = VK_NULL_HANDLE;

  vkDestroyPipelineLayout(m_device, m_pipelineLayout, nullptr);
  m_pipelineLayout = VK_NULL_HANDLE;
}

const VkPipeline& Pipeline::pipeline() const noexcept {
  return m_pipeline;
}

const VkPipelineLayout& Pipeline::pipelineLayout() const noexcept {
  return m_pipelineLayout;
}

PipelineDescription describeShaders(const char* name, const char* stem) {
  PipelineDescription described{};
  described.name = name;
  described.vertexSource = std::string(SHADER_SOURCE_DIR "/") + stem + ".vert";
  described.fragmentSource =
    std::string(SHADER_SOURCE_DIR "/") + stem + ".frag";
  described.vertexSpirv = std::string("shaders/") + stem + ".vert.spv";
  described.fragmentSpirv = std::string("shaders/") + stem + ".frag.spv";

  return described;
}

PipelineDescription describe(PipelineType type) {
  const PipelineShaders shaders = pipelineShaders(type);

  PipelineDescription described{};
  described.vertexSource = shaders.vertexSource;
  described.fragmentSource = shaders.fragmentSource;
  described.vertexSpirv = shaders.vertexSpirv;
  described.fragmentSpirv = shaders.fragmentSpirv;
  described.pushConstants = pushConstantRanges(type);

  switch (type) {
    case PipelineType::Mesh:
      described.name = "mesh";
      described.cullMode = VK_CULL_MODE_BACK_BIT;
      break;
    case PipelineType::Sdf:
      described.name = "sdf";
      described.cullMode = VK_CULL_MODE_FRONT_BIT;
      break;
    case PipelineType::Sky:
      described.name = "sky";
      described.depthTest = VK_FALSE;
      described.depthWrite = VK_FALSE;
      break;
  }

  return described;
}

PipelineDescription describeDepthOnly(PipelineType type) {
  PipelineDescription described = describe(type);

  const char* stem = type == PipelineType::Mesh ? "mesh-depth" : "sdf-depth";

  described.name += "-depth";
  described.fragmentSource =
    std::string(SHADER_SOURCE_DIR "/") + stem + ".frag";
  described.fragmentSpirv = std::string("shaders/") + stem + ".frag.spv";
  described.colourAttachments = 0;
  described.blend = VK_FALSE;

  return described;
}

void Pipeline::makeConfig() {
  m_config.vert = m_description.vertexSource;
  m_config.frag = m_description.fragmentSource;
  m_config.vertexShader = m_description.vertexSpirv;
  m_config.fragmentShader = m_description.fragmentSpirv;
  m_config.bindingDescriptions = m_description.bindings;
  m_config.attributeDescriptions = m_description.attributes;
  m_config.topology = m_description.topology;
  m_config.cullMode = m_description.cullMode;
  m_config.depthTestEnable = m_description.depthTest;
  m_config.depthWriteEnable = m_description.depthWrite;
  m_config.blendEnable = m_description.blend;
  m_config.setLayouts = m_setLayouts;
  m_config.pushConstantRanges = m_description.pushConstants;
}

void Pipeline::create() {
  makeConfig();
  m_pipelineLayout = buildPipelineLayout();

  if (m_pipelineLayout == VK_NULL_HANDLE) {
    throw std::runtime_error("Failed to create pipeline layout");
  }

  m_pipeline = buildPipeline();
  if (m_pipeline == VK_NULL_HANDLE) {
    throw std::runtime_error("Failed to create pipeline");
  }

  std::error_code sources;

  m_vertTime = std::filesystem::last_write_time(m_config.vert, sources);
  m_fragTime = std::filesystem::last_write_time(m_config.frag, sources);
  m_includeTime = newestIncludeTime();
}

std::filesystem::file_time_type Pipeline::newestIncludeTime() {
  std::filesystem::file_time_type newest{};
  std::error_code ec;

  for (const auto& entry :
       std::filesystem::directory_iterator(GLSLC_INCLUDE_DIR, ec)) {
    if (ec) {
      break;
    }

    if (entry.path().extension() != ".glsl") {
      continue;
    }

    const auto written = std::filesystem::last_write_time(entry, ec);

    if (!ec && written > newest) {
      newest = written;
    }
  }

  return newest;
}

bool Pipeline::sourcesChanged() const {
  std::error_code ec;
  auto vertTime = std::filesystem::last_write_time(m_config.vert, ec);
  if (ec) {
    return false;
  }
  auto fragTime = std::filesystem::last_write_time(m_config.frag, ec);
  if (ec) {
    return false;
  }

  return vertTime != m_vertTime || fragTime != m_fragTime
         || newestIncludeTime() != m_includeTime;
}

void Pipeline::reload() {
  if (
    compileShader(m_config.vert, m_config.vertexShader) == false
    || compileShader(m_config.frag, m_config.fragmentShader) == false
  ) {
    std::cout << "Shader compilation failed\n";
    return;
  }

  makeConfig();

  VkPipeline rebuilt = VK_NULL_HANDLE;

  try {
    rebuilt = buildPipeline();
  } catch (const std::exception& e) {
    std::cout << "Pipeline reload failed: " << e.what() << '\n';
    return;
  }

  if (rebuilt == VK_NULL_HANDLE) {
    std::cout << "Pipeline build failed on reload\n";
    return;
  }

  vkDestroyPipeline(m_device, m_pipeline, nullptr);
  m_pipeline = rebuilt;

  m_vertTime = std::filesystem::last_write_time(m_config.vert);
  m_fragTime = std::filesystem::last_write_time(m_config.frag);
  m_includeTime = newestIncludeTime();
}

VkPipelineLayout Pipeline::buildPipelineLayout() {
  VkPipelineLayout pipelineLayout;

  VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
  pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pipelineLayoutInfo.setLayoutCount =
    static_cast<uint32_t>(m_config.setLayouts.size());
  pipelineLayoutInfo.pSetLayouts = m_config.setLayouts.data();
  pipelineLayoutInfo.pushConstantRangeCount =
    static_cast<uint32_t>(m_config.pushConstantRanges.size());
  pipelineLayoutInfo.pPushConstantRanges = m_config.pushConstantRanges.data();

  if (
    vkCreatePipelineLayout(
      m_device,
      &pipelineLayoutInfo,
      nullptr,
      &pipelineLayout
    )
    != VK_SUCCESS
  ) {
    return VK_NULL_HANDLE;
  }

  return pipelineLayout;
}

VkPipeline Pipeline::buildPipeline() {
  ShaderModule vertexShaderModule(m_device, m_config.vertexShader);
  ShaderModule fragShaderModule(m_device, m_config.fragmentShader);

  VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
  vertShaderStageInfo.sType =
    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vertShaderStageInfo.module = vertexShaderModule.handle();
  vertShaderStageInfo.pName = "main";

  VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
  fragShaderStageInfo.sType =
    VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  fragShaderStageInfo.module = fragShaderModule.handle();
  fragShaderStageInfo.pName = "main";

  std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages{
    vertShaderStageInfo,
    fragShaderStageInfo
  };

  std::vector<VkDynamicState> dynamicStates = {
    VK_DYNAMIC_STATE_VIEWPORT,
    VK_DYNAMIC_STATE_SCISSOR
  };

  VkPipelineDynamicStateCreateInfo dynamicState{};
  dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
  dynamicState.pDynamicStates = dynamicStates.data();

  VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
  vertexInputInfo.sType =
    VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

  vertexInputInfo.vertexBindingDescriptionCount =
    static_cast<uint32_t>(m_config.bindingDescriptions.size());
  vertexInputInfo.vertexAttributeDescriptionCount =
    static_cast<uint32_t>(m_config.attributeDescriptions.size());
  vertexInputInfo.pVertexBindingDescriptions =
    m_config.bindingDescriptions.data();
  vertexInputInfo.pVertexAttributeDescriptions =
    m_config.attributeDescriptions.data();

  VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
  inputAssembly.sType =
    VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  inputAssembly.topology = m_config.topology;
  inputAssembly.primitiveRestartEnable = VK_FALSE;

  VkPipelineViewportStateCreateInfo viewportState{};
  viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewportState.viewportCount = 1;
  viewportState.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo rasterizer{};
  rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  rasterizer.depthClampEnable = VK_FALSE;
  rasterizer.rasterizerDiscardEnable = VK_FALSE;
  rasterizer.lineWidth = 1.0f;
  rasterizer.cullMode = m_config.cullMode;
  rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  rasterizer.depthBiasEnable = VK_FALSE;
  rasterizer.depthBiasConstantFactor = 0.0f;
  rasterizer.depthBiasClamp = 0.0f;
  rasterizer.depthBiasSlopeFactor = 0.0f;

  VkPipelineMultisampleStateCreateInfo multisampling{};
  multisampling.sType =
    VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  multisampling.sampleShadingEnable = VK_FALSE;
  multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
  multisampling.minSampleShading = 1.0f;
  multisampling.pSampleMask = nullptr;
  multisampling.alphaToCoverageEnable = VK_FALSE;
  multisampling.alphaToOneEnable = VK_FALSE;

  VkPipelineColorBlendAttachmentState colorBlendAttachment{};
  colorBlendAttachment.colorWriteMask =
    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT
    | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  colorBlendAttachment.blendEnable = m_config.blendEnable;
  colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  colorBlendAttachment.dstColorBlendFactor =
    VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
  colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
  colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

  VkPipelineColorBlendStateCreateInfo colorBlending{};
  colorBlending.sType =
    VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  colorBlending.logicOpEnable = VK_FALSE;
  colorBlending.logicOp = VK_LOGIC_OP_COPY;
  colorBlending.attachmentCount = m_description.colourAttachments;
  colorBlending.pAttachments = &colorBlendAttachment;
  colorBlending.blendConstants[0] = 0.0f;
  colorBlending.blendConstants[1] = 0.0f;
  colorBlending.blendConstants[2] = 0.0f;
  colorBlending.blendConstants[3] = 0.0f;

  VkPipelineRenderingCreateInfo renderingCreateInfo{};
  renderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  renderingCreateInfo.colorAttachmentCount = m_description.colourAttachments;
  renderingCreateInfo.pColorAttachmentFormats = &m_swapChainImageFormat;
  renderingCreateInfo.depthAttachmentFormat = m_depthImageFormat;

  VkPipelineDepthStencilStateCreateInfo depthStencil{};
  depthStencil.sType =
    VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depthStencil.depthTestEnable = m_config.depthTestEnable;
  depthStencil.depthWriteEnable = m_config.depthWriteEnable;
  depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
  depthStencil.depthBoundsTestEnable = VK_FALSE;
  depthStencil.minDepthBounds = 0.0f;
  depthStencil.maxDepthBounds = 1.0f;
  depthStencil.stencilTestEnable = VK_FALSE;
  depthStencil.front = {};
  depthStencil.back = {};

  VkGraphicsPipelineCreateInfo pipelineInfo{};
  pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  pipelineInfo.pNext = &renderingCreateInfo;
  pipelineInfo.stageCount = 2;
  pipelineInfo.pStages = shaderStages.data();
  pipelineInfo.pVertexInputState = &vertexInputInfo;
  pipelineInfo.pInputAssemblyState = &inputAssembly;
  pipelineInfo.pViewportState = &viewportState;
  pipelineInfo.pRasterizationState = &rasterizer;
  pipelineInfo.pMultisampleState = &multisampling;
  pipelineInfo.pColorBlendState = &colorBlending;
  pipelineInfo.pDynamicState = &dynamicState;
  pipelineInfo.layout = m_pipelineLayout;
  pipelineInfo.subpass = 0;
  pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
  pipelineInfo.basePipelineIndex = -1;
  pipelineInfo.pDepthStencilState = &depthStencil;

  VkPipeline pipeline;
  if (
    vkCreateGraphicsPipelines(
      m_device,
      VK_NULL_HANDLE,
      1,
      &pipelineInfo,
      nullptr,
      &pipeline
    )
    != VK_SUCCESS
  ) {
    return VK_NULL_HANDLE;
  }

  return pipeline;
}

}
