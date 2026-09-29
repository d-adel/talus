#include "outline.ih"

namespace talus::gizmos {

namespace {

constexpr uint32_t SELECTION_BINDING = 0u;

constexpr uint32_t SAMPLER_BINDING = 1u;

constexpr uint32_t SCENE_BINDING = 2u;

constexpr uint32_t RADIUS = 2u;

constexpr glm::vec4 COLOUR(0.98f, 0.55f, 0.15f, 1.0f);

constexpr glm::vec4 DIMMED(0.98f, 0.55f, 0.15f, 0.30f);

talus::gpu::SamplerSettings pointSampling() {
  talus::gpu::SamplerSettings settings{};
  settings.filter = VK_FILTER_NEAREST;
  settings.addressMode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  settings.anisotropy = false;

  return settings;
}

}

Outline::Outline(
  const talus::gpu::Device& device,
  const talus::gpu::SwapChain& swapChain,
  VkImageView selectionDepth,
  VkImageView sceneDepth
)
    : m_sampler(device, pointSampling()),
      m_group(
        device,
        1u,
        {},
        {{SELECTION_BINDING,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          {selectionDepth},
          1u},
         {SCENE_BINDING, VK_SHADER_STAGE_FRAGMENT_BIT, {sceneDepth}, 1u}},
        {{SAMPLER_BINDING,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          {m_sampler.handle()},
          1u}}
      ),
      m_pipeline(
        describe(),
        device.vkDevice(),
        std::vector<VkDescriptorSetLayout>{m_group.setLayout()},
        swapChain
      ) {
  retarget(selectionDepth, sceneDepth);
}

talus::gpu::PipelineDescription Outline::describe() {
  talus::gpu::PipelineDescription described =
    talus::gpu::describeShaders("outline", "outline-shader");

  described.depthTest = VK_FALSE;
  described.depthWrite = VK_FALSE;
  described.blend = VK_TRUE;

  VkPushConstantRange range{};
  range.offset = 0;
  range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  range.size = sizeof(OutlinePush);

  described.pushConstants = {range};

  return described;
}

void Outline::retarget(VkImageView selectionDepth, VkImageView sceneDepth) {
  m_group.writeImage(
    SELECTION_BINDING,
    0u,
    selectionDepth,
    VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL
  );
  m_group.writeImage(
    SCENE_BINDING,
    0u,
    sceneDepth,
    VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL
  );
}

void Outline::show() noexcept {
  m_shown = true;
}

void Outline::hide() noexcept {
  m_shown = false;
}

bool Outline::shown() const noexcept {
  return m_shown;
}

void Outline::record(VkCommandBuffer commands) const {
  if (!m_shown) {
    return;
  }

  vkCmdBindPipeline(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_pipeline.pipeline()
  );

  const VkDescriptorSet set = m_group.descriptorSet(0u);

  vkCmdBindDescriptorSets(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_pipeline.pipelineLayout(),
    0,
    1,
    &set,
    0,
    nullptr
  );

  OutlinePush push{};
  push.radius = glm::uvec4(RADIUS, 0u, 0u, 0u);
  push.colour = COLOUR;
  push.dimmed = DIMMED;

  vkCmdPushConstants(
    commands,
    m_pipeline.pipelineLayout(),
    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
    0,
    sizeof(OutlinePush),
    &push
  );

  vkCmdDraw(commands, 3, 1, 0, 0);
}

}
