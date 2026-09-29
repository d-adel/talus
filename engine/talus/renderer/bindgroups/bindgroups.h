#pragma once

#include <talus/gpu/pipeline/pipeline.h>
#include <talus/renderer/frameglobals/frameglobals.h>
#include <talus/renderer/resourcetable/resourcetable.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>

#include <vulkan/vulkan.h>

#include <span>
#include <vector>

namespace talus::renderer {

enum class BindGroup {
  FrameGlobals,
  Resources,
  SdfRecords
};

[[nodiscard]] std::span<const BindGroup> pipelineBindGroups(
  talus::gpu::PipelineType type
);

[[nodiscard]] std::span<const BindGroup> bakeBindGroups();

[[nodiscard]] std::span<const BindGroup> globalsBindGroups();

[[nodiscard]] std::vector<VkDescriptorSetLayout> bindGroupLayouts(
  std::span<const BindGroup> groups,
  const FrameGlobals& frameGlobals,
  const ResourceTable& resources,
  const SdfRecordTable& records
);

[[nodiscard]] std::vector<VkDescriptorSetLayout> pipelineSetLayouts(
  talus::gpu::PipelineType type,
  const FrameGlobals& frameGlobals,
  const ResourceTable& resources,
  const SdfRecordTable& records
);

}
