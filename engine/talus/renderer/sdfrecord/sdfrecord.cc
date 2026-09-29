#include "sdfrecord.ih"

namespace talus::renderer {

RecordBounds makeRecordBounds(
  const talus::field::Aabb& localBox,
  const glm::mat4& model
) {
  if (glm::any(glm::greaterThan(localBox.minimum, localBox.maximum))) {
    return {glm::vec4(1.0f), glm::vec4(-1.0f)};
  }

  glm::vec3 minimum(std::numeric_limits<float>::max());
  glm::vec3 maximum(std::numeric_limits<float>::lowest());

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::vec3 at(
      (corner & 1u) != 0u ? localBox.maximum.x : localBox.minimum.x,
      (corner & 2u) != 0u ? localBox.maximum.y : localBox.minimum.y,
      (corner & 4u) != 0u ? localBox.maximum.z : localBox.minimum.z
    );

    const glm::vec3 world = glm::vec3(model * glm::vec4(at, 1.0f));

    minimum = glm::min(minimum, world);
    maximum = glm::max(maximum, world);
  }

  return {glm::vec4(minimum, 0.0f), glm::vec4(maximum, 0.0f)};
}

SdfRecord makeSdfRecord(
  const talus::objectmodel::Pose& pose,
  const talus::objectmodel::SdfGrid& grid,
  const talus::objectmodel::BakedVolume& volume,
  uint32_t primitiveOffset,
  uint32_t primitiveCount,
  uint32_t fieldRepresentation,
  float gridMargin
) {
  SdfRecord record{};

  record.model = talus::objectmodel::model(pose);
  record.inverseModel = glm::inverse(record.model);

  record.voxelSize = glm::vec4(grid.voxelSize, gridMargin);

  record.resolutionVolumeIndex = glm::uvec4(grid.resolution, volume.index);

  record.config.x = primitiveCount;
  record.config.y = fieldRepresentation;
  record.config.z = 1u;
  record.config.w = primitiveOffset;

  record.localOrigin = grid.origin;

  record.localOrigin.w =
    volume.index == UINT32_MAX
      ? 0.0f
      : static_cast<float>(volume.index * talus::core::BRICK_TABLE_STRIDE);

  return record;
}

}
