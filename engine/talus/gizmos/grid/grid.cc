#include "grid.ih"

namespace talus::gizmos {

namespace {

constexpr float KIND_SECONDARY = 0.0f;
constexpr float KIND_PRIMARY = 1.0f;
constexpr float KIND_AXIS_U = 2.0f;
constexpr float KIND_AXIS_V = 3.0f;

float truncated(float value) {
  return static_cast<float>(static_cast<int64_t>(value));
}

}

VkVertexInputBindingDescription GridVertex::bindingDescription() {
  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(GridVertex);
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  return binding;
}

std::vector<VkVertexInputAttributeDescription> GridVertex::
  attributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributes(2);

  attributes[0].binding = 0;
  attributes[0].location = 0;
  attributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
  attributes[0].offset = offsetof(GridVertex, position);

  attributes[1].binding = 0;
  attributes[1].location = 1;
  attributes[1].format = VK_FORMAT_R32_SFLOAT;
  attributes[1].offset = offsetof(GridVertex, kind);

  return attributes;
}

Grid::Grid(
  const talus::gpu::Device& device,
  const talus::gpu::SwapChain& swapChain,
  std::vector<VkDescriptorSetLayout> setLayouts,
  const talus::view::GridPlane& plane,
  const talus::view::GridStyle& style
)
    : m_device(device),
      m_plane(plane),
      m_style(style),
      m_pipeline(
        describe(),
        device.vkDevice(),
        std::move(setLayouts),
        swapChain
      ) {
  m_capacity = static_cast<uint32_t>(4 * (2 * m_style.size + 1));

  m_vertices = talus::gpu::Buffer(
    m_device,
    sizeof(GridVertex) * m_capacity,
    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
    VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
  );
}

talus::gpu::PipelineDescription Grid::describe() {
  talus::gpu::PipelineDescription described =
    talus::gpu::describeShaders("grid", "grid-shader");

  described.bindings = {GridVertex::bindingDescription()};
  described.attributes = GridVertex::attributeDescriptions();
  described.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
  described.depthTest = VK_TRUE;
  described.depthWrite = VK_FALSE;
  described.blend = VK_TRUE;

  VkPushConstantRange range{};
  range.offset = 0;
  range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  range.size = sizeof(talus::gpu::GridPush);

  described.pushConstants = {range};

  return described;
}

float Grid::planeDistance(const glm::vec3& position) const {
  return std::abs(glm::dot(position, m_plane.normal));
}

void Grid::update(const glm::vec3& cameraPosition) {
  const float steps = static_cast<float>(m_style.steps);

  const float distance = std::max(planeDistance(cameraPosition), 1e-4f);

  m_level = std::log(distance) / std::log(steps) + m_style.levelBias;

  const float clamped = std::clamp(
    m_level,
    static_cast<float>(m_style.levelMin),
    static_cast<float>(m_style.levelMax)
  );

  const float floored = std::floor(clamped);

  m_decimals = clamped - floored;

  const float small = std::pow(steps, floored);
  const float large = small * steps;

  const float centreU =
    large * truncated(glm::dot(cameraPosition, m_plane.axisU) / large);
  const float centreV =
    large * truncated(glm::dot(cameraPosition, m_plane.axisV) / large);

  const float minFade = std::pow(steps, static_cast<float>(m_style.levelMin));
  const float maxFade = std::pow(steps, static_cast<float>(m_style.levelMax));

  const float fade =
    std::clamp(std::pow(steps, m_level - 1.0f), minFade, maxFade);

  m_radius = (static_cast<float>(m_style.size) - steps) * fade;

  const bool moved = !m_built || floored != m_floored || centreU != m_centreU
                     || centreV != m_centreV;

  m_floored = floored;
  m_centreU = centreU;
  m_centreV = centreV;

  if (moved) {
    rebuild();

    m_built = true;
  }
}

void Grid::rebuild() {
  const float steps = static_cast<float>(m_style.steps);
  const float small = std::pow(steps, m_floored);

  const float span = static_cast<float>(m_style.size) * small;

  const float beginU = m_centreU - span;
  const float endU = m_centreU + span;
  const float beginV = m_centreV - span;
  const float endV = m_centreV + span;

  std::vector<GridVertex> vertices;
  vertices.reserve(m_capacity);

  const auto line =
    [&vertices](const glm::vec3& from, const glm::vec3& to, float kind) {
      vertices.push_back({from, kind});
      vertices.push_back({to, kind});
    };

  for (int32_t i = -m_style.size; i <= m_style.size; ++i) {
    const float offset = static_cast<float>(i) * small;

    const float positionU = m_centreU + offset;
    const float positionV = m_centreV + offset;

    const bool primary = i % m_style.steps == 0;

    const float alongV = positionU == 0.0f ? KIND_AXIS_V
                         : primary         ? KIND_PRIMARY
                                           : KIND_SECONDARY;

    line(
      m_plane.axisU * positionU + m_plane.axisV * beginV,
      m_plane.axisU * positionU + m_plane.axisV * endV,
      alongV
    );

    const float alongU = positionV == 0.0f ? KIND_AXIS_U
                         : primary         ? KIND_PRIMARY
                                           : KIND_SECONDARY;

    line(
      m_plane.axisU * beginU + m_plane.axisV * positionV,
      m_plane.axisU * endU + m_plane.axisV * positionV,
      alongU
    );
  }

  m_vertexCount = static_cast<uint32_t>(vertices.size());

  m_vertices.fill(vertices);
}

void Grid::record(VkCommandBuffer commands, VkDescriptorSet globals) const {
  if (m_vertexCount == 0) {
    return;
  }

  vkCmdBindPipeline(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_pipeline.pipeline()
  );

  vkCmdBindDescriptorSets(
    commands,
    VK_PIPELINE_BIND_POINT_GRAPHICS,
    m_pipeline.pipelineLayout(),
    0,
    1,
    &globals,
    0,
    nullptr
  );

  const VkPipelineLayout layout = m_pipeline.pipelineLayout();

  if (m_vertexCount == 0) {
    return;
  }

  talus::gpu::GridPush push{};
  push.primary = m_style.primary;
  push.secondary = m_style.secondary;
  push.axisColourU = m_style.axisColourU;
  push.axisColourV = m_style.axisColourV;
  push.normal = glm::vec4(m_plane.normal, m_decimals);
  push.fade = glm::vec4(m_radius, 0.0f, 0.0f, 0.0f);

  vkCmdPushConstants(
    commands,
    layout,
    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
    0,
    sizeof(talus::gpu::GridPush),
    &push
  );

  const VkBuffer buffers[] = {m_vertices.buffer()};
  const VkDeviceSize offsets[] = {0};

  vkCmdBindVertexBuffers(commands, 0, 1, buffers, offsets);

  vkCmdDraw(commands, m_vertexCount, 1, 0, 0);
}

}
