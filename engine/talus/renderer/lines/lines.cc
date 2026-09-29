#include "lines.ih"

namespace talus::renderer {

namespace {

constexpr size_t MINIMUM_VERTICES = 1024u;

VkVertexInputBindingDescription binding() {
  VkVertexInputBindingDescription described{};
  described.binding = 0;
  described.stride = sizeof(talus::debugdraw::LineVertex);
  described.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  return described;
}

std::vector<VkVertexInputAttributeDescription> attributes() {
  std::vector<VkVertexInputAttributeDescription> described(2);

  described[0].binding = 0;
  described[0].location = 0;
  described[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  described[0].offset = offsetof(talus::debugdraw::LineVertex, position);

  described[1].binding = 0;
  described[1].location = 1;
  described[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
  described[1].offset = offsetof(talus::debugdraw::LineVertex, colour);

  return described;
}

}

Lines::Lines(
  const talus::gpu::Device& device,
  const talus::gpu::SwapChain& swapChain,
  std::vector<VkDescriptorSetLayout> setLayouts
)
    : m_device(device),
      m_tested(describe(true), device.vkDevice(), setLayouts, swapChain),
      m_untested(describe(false), device.vkDevice(), setLayouts, swapChain) {}

talus::gpu::PipelineDescription Lines::describe(bool depthTested) {
  talus::gpu::PipelineDescription described = talus::gpu::describeShaders(
    depthTested ? "lines" : "lines-overlay",
    "line-shader"
  );

  described.bindings = {binding()};
  described.attributes = attributes();
  described.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  described.depthTest = depthTested ? VK_TRUE : VK_FALSE;
  described.depthWrite = VK_FALSE;
  described.blend = VK_TRUE;

  return described;
}

void Lines::upload(
  uint32_t frame,
  std::span<const talus::debugdraw::LineVertex> tested,
  std::span<const talus::debugdraw::LineVertex> untested,
  std::span<const talus::debugdraw::LineVertex> overlay
) {
  Slot& slot = m_slots[frame];

  slot.tested = static_cast<uint32_t>(tested.size());
  slot.untested = static_cast<uint32_t>(untested.size() + overlay.size());

  const size_t total = tested.size() + untested.size() + overlay.size();

  if (total == 0u) {
    return;
  }

  if (total > slot.capacity) {
    size_t wanted = std::max(MINIMUM_VERTICES, slot.capacity);

    while (wanted < total) {
      wanted *= 2u;
    }

    slot.buffer = talus::gpu::Buffer(
      m_device,
      sizeof(talus::debugdraw::LineVertex) * wanted,
      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
    );

    slot.capacity = wanted;
  }

  std::vector<talus::debugdraw::LineVertex> joined;
  joined.reserve(total);
  joined.insert(joined.end(), tested.begin(), tested.end());
  joined.insert(joined.end(), untested.begin(), untested.end());
  joined.insert(joined.end(), overlay.begin(), overlay.end());

  slot.buffer.fill(joined);
}

void Lines::record(
  VkCommandBuffer commands,
  VkDescriptorSet globals,
  uint32_t frame
) const {
  const Slot& slot = m_slots[frame];

  if (slot.tested + slot.untested == 0u) {
    return;
  }

  const VkBuffer buffers[] = {slot.buffer.buffer()};
  const VkDeviceSize offsets[] = {0};

  vkCmdBindVertexBuffers(commands, 0, 1, buffers, offsets);

  const auto draw =
    [&](const talus::gpu::Pipeline& pipeline, uint32_t first, uint32_t count) {
      vkCmdBindPipeline(
        commands,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pipeline.pipeline()
      );

      vkCmdBindDescriptorSets(
        commands,
        VK_PIPELINE_BIND_POINT_GRAPHICS,
        pipeline.pipelineLayout(),
        0,
        1,
        &globals,
        0,
        nullptr
      );

      vkCmdDraw(commands, count, 1, first, 0);
    };

  if (slot.tested != 0u) {
    draw(m_tested, 0u, slot.tested);
  }

  if (slot.untested != 0u) {
    draw(m_untested, slot.tested, slot.untested);
  }
}

}
