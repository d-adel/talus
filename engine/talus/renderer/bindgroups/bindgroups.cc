#include "bindgroups.ih"

namespace talus::renderer {

namespace {

constexpr std::array<BindGroup, 2> MESH_GROUPS{
  BindGroup::FrameGlobals,
  BindGroup::Resources
};

constexpr std::array<BindGroup, 3> SDF_GROUPS{
  BindGroup::FrameGlobals,
  BindGroup::Resources,
  BindGroup::SdfRecords
};

constexpr std::array<BindGroup, 1> GLOBALS_ONLY{BindGroup::FrameGlobals};

constexpr std::array<BindGroup, 1> RECORDS_ONLY{BindGroup::SdfRecords};

}

std::span<const BindGroup> pipelineBindGroups(talus::gpu::PipelineType type) {
  switch (type) {
    case talus::gpu::PipelineType::Mesh:
      return MESH_GROUPS;
    case talus::gpu::PipelineType::Sdf:
      return SDF_GROUPS;
    case talus::gpu::PipelineType::Sky:
      return GLOBALS_ONLY;
  }

  return {};
}

std::span<const BindGroup> bakeBindGroups() {
  return RECORDS_ONLY;
}

std::span<const BindGroup> globalsBindGroups() {
  return GLOBALS_ONLY;
}

std::vector<VkDescriptorSetLayout> bindGroupLayouts(
  std::span<const BindGroup> groups,
  const FrameGlobals& frameGlobals,
  const ResourceTable& resources,
  const SdfRecordTable& records
) {
  std::vector<VkDescriptorSetLayout> layouts;
  layouts.reserve(groups.size());

  for (const BindGroup group : groups) {
    switch (group) {
      case BindGroup::FrameGlobals:
        layouts.push_back(frameGlobals.setLayout());
        break;
      case BindGroup::Resources:
        layouts.push_back(resources.setLayout());
        break;
      case BindGroup::SdfRecords:
        layouts.push_back(records.setLayout());
        break;
    }
  }

  return layouts;
}

std::vector<VkDescriptorSetLayout> pipelineSetLayouts(
  talus::gpu::PipelineType type,
  const FrameGlobals& frameGlobals,
  const ResourceTable& resources,
  const SdfRecordTable& records
) {
  return bindGroupLayouts(
    pipelineBindGroups(type),
    frameGlobals,
    resources,
    records
  );
}

}
