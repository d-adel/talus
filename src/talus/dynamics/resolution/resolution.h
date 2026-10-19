#pragma once

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <span>

namespace talus::dynamics {

inline constexpr double SHAPE_TOLERANCE = 0.1;

struct ResolvedMesh {
  std::shared_ptr<const talus::PreparedMesh> mesh;
  uint32_t coarsening = 1u;
};

[[nodiscard]] field::SampledSdf grownField(const field::SampledSdf& field);

[[nodiscard]] field::SampledSdf coarserField(
  const field::SampledSdf& field,
  uint32_t factor,
  const glm::uvec3& phase
);

[[nodiscard]] double shapeMismatch(
  const talus::PreparedMesh& authored,
  const talus::PreparedMesh& candidate,
  const field::SampledSdf& field,
  uint32_t factor,
  const glm::uvec3& phase
);

[[nodiscard]] uint32_t meshPieces(const talus::PreparedMesh& mesh);

[[nodiscard]] ResolvedMesh resolveMesh(
  const field::SampledSdf& field,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::mat4& localToWorld,
  std::span<const deformable::SupportVolume> supports,
  const IntegrationParameters& parameters
);

}
