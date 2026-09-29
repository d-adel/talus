#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::deformable {

struct ShapeProbe {
  const talus::field::SampledSdf* field = nullptr;
  glm::vec3 centerOfMass{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 fieldOffset{0.0f};
  glm::vec3 linearVelocity{0.0f};
  glm::vec3 angularVelocity{0.0f};
};

struct ShapeAt {
  glm::vec3 centerOfMass{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

[[nodiscard]] ShapeAt advanceShape(const ShapeProbe& shape, float elapsed);

[[nodiscard]] glm::vec3 fieldPointOf(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
);

struct ShapeDistance {
  float distance = 0.0f;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

[[nodiscard]] ShapeDistance probeShape(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
);

struct PreciseDistance {
  double distance = 0.0;
  glm::dvec3 normal{0.0, 1.0, 0.0};
  glm::dvec3 gradient{0.0};
};

[[nodiscard]] PreciseDistance probeShapePrecise(
  const ShapeProbe& shape,
  const glm::dquat& rotation,
  const glm::dvec3& relative
);

struct SurfaceContact {
  glm::uvec3 nodes{0u};
  glm::vec3 weights{0.0f};
  int partner = -1;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
  glm::vec3 localPoint{0.0f};
  float gap = 0.0f;
  float bias = 0.0f;
  float friction = 0.5f;
  uint32_t triangle = 0u;
};

struct ContactGaps {
  float bias = 0.0f;
  float threshold = 0.0f;
};

[[nodiscard]] std::vector<SurfaceContact> findContacts(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const glm::vec3& gravity,
  std::span<const ShapeProbe> shapes,
  float deltaTime,
  const ContactGaps& gaps
);

}
