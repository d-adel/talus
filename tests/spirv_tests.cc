#include <catch2/catch_test_macros.hpp>

#include <spirv_reflect.h>

#include <talus/core/config/config.h>
#include <talus/field/field.h>
#include <talus/gpu/pipeline/pipeline.h>
#include <talus/renderer/bindgroups/bindgroups.h>
#include <talus/renderer/frameglobals/frameglobals.h>
#include <talus/renderer/lines/lines.h>
#include <talus/renderer/materialrecord/materialrecord.h>
#include <talus/renderer/sdfbaker/sdfbaker.h>
#include <talus/renderer/vertex/vertex.h>
#include <talus/gizmos/grid/grid.h>
#include <talus/gizmos/handles/handles.h>
#include <talus/gizmos/outline/outline.h>
#include <talus/gizmos/pipelines/pipelines.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace {

using talus::gizmos::OutlinePush;
using talus::gpu::GridPush;
using talus::gpu::PipelineType;
using talus::gpu::PushConstants;
using talus::renderer::BindGroup;
using talus::renderer::CameraUniform;
using talus::renderer::LightUniform;
using talus::renderer::MarchParams;
using talus::renderer::MaterialRecord;
using talus::renderer::SceneCounts;

struct ExpectedMember {
  std::string name;
  uint32_t offset;
};

struct ExpectedBlock {
  uint32_t size;
  std::vector<ExpectedMember> members;
};

std::map<std::string, ExpectedBlock> blocks() {
  return {
    {"CameraUniform",
     {sizeof(CameraUniform),
      {{"view", offsetof(CameraUniform, view)},
       {"proj", offsetof(CameraUniform, proj)},
       {"viewProj", offsetof(CameraUniform, viewProj)},
       {"inverseViewProj", offsetof(CameraUniform, inverseViewProj)},
       {"position", offsetof(CameraUniform, position)}}}},
    {"MarchParams",
     {sizeof(MarchParams),
      {{"epsilon", offsetof(MarchParams, epsilon)},
       {"maxDistance", offsetof(MarchParams, maxDistance)},
       {"omega", offsetof(MarchParams, omega)},
       {"gradientEpsilon", offsetof(MarchParams, gradientEpsilon)},
       {"shadowMaxDistance", offsetof(MarchParams, shadowMaxDistance)},
       {"shadowSharpness", offsetof(MarchParams, shadowSharpness)},
       {"maxIterations", offsetof(MarchParams, maxIterations)}}}},
    {"SceneCounts",
     {sizeof(SceneCounts),
      {{"sdfRecords", offsetof(SceneCounts, sdfRecords)}}}},
    {"SceneLight",
     {sizeof(LightUniform),
      {{"direction", offsetof(LightUniform, direction)},
       {"skyTop", offsetof(LightUniform, skyTop)},
       {"skyHorizon", offsetof(LightUniform, skyHorizon)},
       {"groundBottom", offsetof(LightUniform, groundBottom)},
       {"shading", offsetof(LightUniform, shading)}}}},
    {"MaterialTable",
     {talus::core::MAX_MATERIALS * sizeof(MaterialRecord), {{"materials", 0}}}},
    {"PushConstants",
     {offsetof(PushConstants, recordIndex) + sizeof(PushConstants::recordIndex),
      {{"model", offsetof(PushConstants, model)},
       {"materialIndex", offsetof(PushConstants, materialIndex)},
       {"recordIndex", offsetof(PushConstants, recordIndex)}}}},
    {"GridPush",
     {sizeof(GridPush),
      {{"primary", offsetof(GridPush, primary)},
       {"secondary", offsetof(GridPush, secondary)},
       {"axisColourU", offsetof(GridPush, axisColourU)},
       {"axisColourV", offsetof(GridPush, axisColourV)},
       {"normal", offsetof(GridPush, normal)},
       {"fade", offsetof(GridPush, fade)}}}},
    {"OutlinePush",
     {sizeof(OutlinePush),
      {{"radius", offsetof(OutlinePush, radius)},
       {"colour", offsetof(OutlinePush, colour)},
       {"dimmed", offsetof(OutlinePush, dimmed)}}}}
  };
}

struct ShaderUnderTest {
  std::string spirv;
  std::string name;
  std::vector<BindGroup> groups;
  std::vector<VkPushConstantRange> pushConstants;
};

void addBoth(
  std::vector<ShaderUnderTest>& shaders,
  const talus::gpu::PipelineDescription& described,
  std::span<const BindGroup> groups
) {
  const std::vector<BindGroup> held(groups.begin(), groups.end());

  shaders.push_back(
    {described.vertexSpirv, described.name, held, described.pushConstants}
  );
  shaders.push_back(
    {described.fragmentSpirv, described.name, held, described.pushConstants}
  );
}

std::vector<ShaderUnderTest> graphicsShaders() {
  std::vector<ShaderUnderTest> shaders;

  for (const PipelineType type :
       {PipelineType::Mesh, PipelineType::Sdf, PipelineType::Sky}) {
    addBoth(
      shaders,
      talus::gpu::describe(type),
      talus::renderer::pipelineBindGroups(type)
    );
  }

  for (const PipelineType type : {PipelineType::Mesh, PipelineType::Sdf}) {
    addBoth(
      shaders,
      talus::gpu::describeDepthOnly(type),
      talus::renderer::pipelineBindGroups(type)
    );
  }

  for (const talus::gpu::PipelineDescription& described :
       talus::gizmos::pipelines()) {
    addBoth(shaders, described, talus::renderer::globalsBindGroups());
  }

  addBoth(
    shaders,
    talus::renderer::Lines::describe(true),
    talus::renderer::globalsBindGroups()
  );

  return shaders;
}

std::filesystem::path resolve(const std::string& spirv) {
  return std::filesystem::path(TALUS_SPIRV_DIR)
         / std::filesystem::path(spirv).filename();
}

std::vector<char> read(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);

  REQUIRE(file.is_open());

  const std::streamsize size = file.tellg();
  file.seekg(0);

  std::vector<char> bytes(static_cast<size_t>(size));
  file.read(bytes.data(), size);

  return bytes;
}

class Module {
public:
  explicit Module(const std::filesystem::path& path) {
    const std::vector<char> bytes = read(path);

    m_ok = spvReflectCreateShaderModule(bytes.size(), bytes.data(), &m_module)
           == SPV_REFLECT_RESULT_SUCCESS;
  }

  Module(const Module&) = delete;
  Module& operator=(const Module&) = delete;
  Module(Module&&) = delete;
  Module& operator=(Module&&) = delete;

  ~Module() {
    if (m_ok) {
      spvReflectDestroyShaderModule(&m_module);
    }
  }

  [[nodiscard]] bool ok() const noexcept {
    return m_ok;
  }

  [[nodiscard]] const SpvReflectShaderModule& get() const noexcept {
    return m_module;
  }

private:
  SpvReflectShaderModule m_module{};
  bool m_ok = false;
};

size_t checkBlock(
  const std::string& shader,
  const SpvReflectBlockVariable& block,
  const std::map<std::string, ExpectedBlock>& expected
) {
  if (
    block.type_description == nullptr
    || block.type_description->type_name == nullptr
  ) {
    return 0;
  }

  const std::string name = block.type_description->type_name;

  const auto found = expected.find(name);

  if (found == expected.end()) {
    return 0;
  }

  INFO("shader " << shader << " block " << name);

  REQUIRE(block.member_count == found->second.members.size());

  for (uint32_t at = 0; at != block.member_count; ++at) {
    const SpvReflectBlockVariable& member = block.members[at];

    INFO("member " << at);

    REQUIRE(member.name != nullptr);
    REQUIRE(std::string(member.name) == found->second.members[at].name);
    REQUIRE(member.offset == found->second.members[at].offset);
  }

  return 1;
}

}

TEST_CASE("every compiled shader module parses", "[spirv]") {
  size_t seen = 0;

  for (const ShaderUnderTest& shader : graphicsShaders()) {
    const std::filesystem::path path = resolve(shader.spirv);

    INFO("path " << path.string());
    REQUIRE(std::filesystem::exists(path));

    const Module module(path);

    REQUIRE(module.ok());

    ++seen;
  }

  const talus::renderer::BakeShaders bake = talus::renderer::bakeShaders();

  for (const char* spirv : {bake.distanceSpirv, bake.lipschitzSpirv}) {
    const std::filesystem::path path = resolve(spirv);

    INFO("path " << path.string());
    REQUIRE(std::filesystem::exists(path));

    const Module module(path);

    REQUIRE(module.ok());

    ++seen;
  }

  REQUIRE(seen == 18);
}

TEST_CASE("every declared uniform block matches its C++ struct", "[spirv]") {
  const std::map<std::string, ExpectedBlock> expected = blocks();

  size_t checked = 0;

  for (const ShaderUnderTest& shader : graphicsShaders()) {
    const Module module(resolve(shader.spirv));

    REQUIRE(module.ok());

    for (uint32_t at = 0; at != module.get().descriptor_binding_count; ++at) {
      checked += checkBlock(
        shader.spirv,
        module.get().descriptor_bindings[at].block,
        expected
      );
    }
  }

  REQUIRE(checked >= 10);
}

TEST_CASE("every push constant block matches its C++ struct", "[spirv]") {
  const std::map<std::string, ExpectedBlock> expected = blocks();

  size_t checked = 0;

  for (const ShaderUnderTest& shader : graphicsShaders()) {
    const Module module(resolve(shader.spirv));

    REQUIRE(module.ok());

    for (uint32_t at = 0; at != module.get().push_constant_block_count; ++at) {
      checked += checkBlock(
        shader.spirv,
        module.get().push_constant_blocks[at],
        expected
      );
    }
  }

  REQUIRE(checked == 8);
}

TEST_CASE("a shader binds no set its pipeline does not describe", "[spirv]") {
  size_t checked = 0;

  for (const ShaderUnderTest& shader : graphicsShaders()) {
    const Module module(resolve(shader.spirv));

    REQUIRE(module.ok());

    const std::span<const BindGroup> groups = shader.groups;

    for (uint32_t at = 0; at != module.get().descriptor_binding_count; ++at) {
      const uint32_t set = module.get().descriptor_bindings[at].set;

      INFO(
        "shader " << shader.spirv << " uses set " << set << " of "
                  << groups.size()
      );

      REQUIRE(set < groups.size());

      ++checked;
    }
  }

  REQUIRE(checked > 0);
}

TEST_CASE(
  "the push range covers every push block a shader declares",
  "[spirv]"
) {
  size_t checked = 0;

  for (const ShaderUnderTest& shader : graphicsShaders()) {
    const Module module(resolve(shader.spirv));

    REQUIRE(module.ok());

    const std::vector<VkPushConstantRange>& ranges = shader.pushConstants;

    for (uint32_t at = 0; at != module.get().push_constant_block_count; ++at) {
      const SpvReflectBlockVariable& block =
        module.get().push_constant_blocks[at];

      INFO("shader " << shader.spirv);

      REQUIRE(ranges.size() == 1);
      REQUIRE(block.offset >= ranges[0].offset);
      REQUIRE(block.offset + block.size <= ranges[0].offset + ranges[0].size);

      ++checked;
    }
  }

  REQUIRE(checked == 8);
}

TEST_CASE(
  "the vertex stages consume the attributes the host binds",
  "[spirv]"
) {
  const Module mesh(
    resolve(talus::gpu::pipelineShaders(PipelineType::Mesh).vertexSpirv)
  );

  REQUIRE(mesh.ok());

  const auto meshAttributes =
    talus::renderer::Vertex::getAttributeDescriptions();

  size_t meshInputs = 0;

  for (uint32_t at = 0; at != mesh.get().input_variable_count; ++at) {
    const SpvReflectInterfaceVariable& variable =
      *mesh.get().input_variables[at];

    if (variable.built_in != -1) {
      continue;
    }

    INFO("mesh input location " << variable.location);

    REQUIRE(variable.location < meshAttributes.size());

    ++meshInputs;
  }

  REQUIRE(meshInputs == meshAttributes.size());

  const Module grid(resolve(talus::gizmos::Grid::describe().vertexSpirv));

  REQUIRE(grid.ok());

  const auto gridAttributes =
    talus::gizmos::GridVertex::attributeDescriptions();

  size_t gridInputs = 0;

  for (uint32_t at = 0; at != grid.get().input_variable_count; ++at) {
    const SpvReflectInterfaceVariable& variable =
      *grid.get().input_variables[at];

    if (variable.built_in != -1) {
      continue;
    }

    INFO("grid input location " << variable.location);

    REQUIRE(variable.location < gridAttributes.size());

    ++gridInputs;
  }

  REQUIRE(gridInputs == gridAttributes.size());

  const talus::gpu::PipelineDescription lines =
    talus::renderer::Lines::describe(true);

  const Module line(resolve(lines.vertexSpirv));

  REQUIRE(line.ok());

  size_t lineInputs = 0;

  for (uint32_t at = 0; at != line.get().input_variable_count; ++at) {
    const SpvReflectInterfaceVariable& variable =
      *line.get().input_variables[at];

    if (variable.built_in != -1) {
      continue;
    }

    INFO("line input location " << variable.location);

    REQUIRE(variable.location < lines.attributes.size());

    ++lineInputs;
  }

  REQUIRE(lineInputs == lines.attributes.size());
}
