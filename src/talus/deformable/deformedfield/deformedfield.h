#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/pose.h>

#include <glm/glm.hpp>

#include <span>

namespace talus::deformable {

[[nodiscard]] talus::field::WriteReport bakeDeformedField(
  const talus::field::SampledSdf& rest,
  const talus::PreparedMesh& mesh,
  std::span<const glm::vec3> position,
  const talus::Pose& pose,
  talus::field::SampledSdf& current,
  bool carveBeyondReach = false
);

}
