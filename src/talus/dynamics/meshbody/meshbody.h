#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <memory>
#include <span>

namespace talus::dynamics {

struct MeshShapeSettings {
  double mass = 1.0;
  double voxel = 0.0;
  double spacing = 0.02;
  double friction = 0.5;
  double restitution = 0.0;
};

struct MeshShape {
  std::shared_ptr<const field::SampledSdf> field;
  RigidBody body;
};

struct MeshFrame {
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

[[nodiscard]] MeshShape meshShape(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const MeshShapeSettings& settings,
  const Workers& workers = Workers{}
);

uint32_t addMeshBody(
  BodySet& bodies,
  const MeshShape& shape,
  const glm::dvec3& position,
  const glm::dquat& rotation,
  const glm::dvec3& velocity
);

[[nodiscard]] MeshFrame meshFrame(const RigidBody& body);

[[nodiscard]] glm::dvec3 meshPoint(
  const RigidBody& body,
  const glm::dvec3& sample
);

}
