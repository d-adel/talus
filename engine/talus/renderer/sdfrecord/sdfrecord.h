#pragma once

#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/bakedvolume/bakedvolume.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace talus::renderer {

struct SdfRecord {
  glm::mat4 model;
  glm::mat4 inverseModel;
  glm::vec4 voxelSize;
  glm::uvec4 resolutionVolumeIndex;
  glm::uvec4 config;
  glm::vec4 localOrigin;
};

static_assert(
  offsetof(SdfRecord, model) == 0,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(SdfRecord, inverseModel) == 64,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(SdfRecord, voxelSize) == 128,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(SdfRecord, resolutionVolumeIndex) == 144,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(SdfRecord, config) == 160,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(SdfRecord, localOrigin) == 176,
  "SdfRecord must match its block in sdf-shader.frag"
);
static_assert(
  sizeof(SdfRecord) == 192,
  "SdfRecord must match its block in sdf-shader.frag"
);

struct RecordBounds {
  glm::vec4 minimum;
  glm::vec4 maximum;
};

static_assert(
  offsetof(RecordBounds, minimum) == 0,
  "RecordBounds must match its block in sdf-shader.frag"
);
static_assert(
  offsetof(RecordBounds, maximum) == 16,
  "RecordBounds must match its block in sdf-shader.frag"
);
static_assert(
  sizeof(RecordBounds) == 32,
  "RecordBounds must match its block in sdf-shader.frag"
);

RecordBounds makeRecordBounds(
  const talus::field::Aabb& localBox,
  const glm::mat4& model
);

SdfRecord makeSdfRecord(
  const talus::objectmodel::Pose& pose,
  const talus::objectmodel::SdfGrid& grid,
  const talus::objectmodel::BakedVolume& volume,
  uint32_t primitiveOffset,
  uint32_t primitiveCount,
  uint32_t fieldRepresentation,
  float gridMargin
);

}
