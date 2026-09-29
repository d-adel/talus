#include "frameglobals.ih"

namespace talus::renderer {

namespace {

constexpr VkDeviceSize MARCH_PARAMS_BLOCK_BYTES =
  (sizeof(MarchParams) + 15u) / 16u * 16u;

constexpr VkDeviceSize SCENE_COUNTS_BLOCK_BYTES =
  (sizeof(SceneCounts) + 15u) / 16u * 16u;

}

FrameGlobals::FrameGlobals(const talus::gpu::Device& device)
    : m_group(
        device,
        talus::core::MAX_FRAMES_IN_FLIGHT,
        {{CAMERA,
          sizeof(CameraUniform),
          VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame},
         {MARCH,
          MARCH_PARAMS_BLOCK_BYTES,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame},
         {COUNTS,
          SCENE_COUNTS_BLOCK_BYTES,
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame},
         {LIGHT,
          sizeof(LightUniform),
          VK_SHADER_STAGE_FRAGMENT_BIT,
          talus::gpu::DescriptorGroup::BufferUpdate::PerFrame}}
      ) {}

void FrameGlobals::update(
  uint32_t frame,
  const CameraUniform& camera,
  const MarchParams& march,
  const SceneCounts& counts,
  const LightUniform& light
) {
  m_group.write(CAMERA, frame, &camera, sizeof(camera));
  m_group.write(MARCH, frame, &march, sizeof(march));
  m_group.write(COUNTS, frame, &counts, sizeof(counts));
  m_group.write(LIGHT, frame, &light, sizeof(light));
}

const VkDescriptorSet& FrameGlobals::descriptorSet(
  uint32_t frame
) const noexcept {
  return m_group.descriptorSet(frame);
}

const VkDescriptorSetLayout& FrameGlobals::setLayout() const noexcept {
  return m_group.setLayout();
}

}
