#pragma once

#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace fixture {

inline constexpr glm::dvec3 characterHalf{0.3, 0.8, 0.3};
inline constexpr double characterMass = 80.0;
inline constexpr double groundForce = 6000.0;

struct Character {
  uint32_t body = talus::dynamics::noBody;
  uint32_t planar = 0u;
  uint32_t pogo = 0u;
  uint32_t ground = talus::dynamics::noBody;
  bool onGround = false;
};

struct SoftBlock {
  talus::field::SampledSdf field;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

[[nodiscard]] const talus::field::SampledSdf& characterField();

[[nodiscard]] const talus::field::SampledSdf& groundField();

[[nodiscard]] Character addCharacter(
  talus::dynamics::World& world,
  const glm::dvec3& at,
  uint32_t planarBase = talus::dynamics::noBody
);

[[nodiscard]] SoftBlock softBlock(const glm::dvec3& half, double voxel);

void addSoftBlock(
  talus::dynamics::World& world,
  const SoftBlock& block,
  const glm::dvec3& at,
  const glm::dvec3& velocity = glm::dvec3(0.0)
);

void requireStrainstep(const talus::dynamics::World& world, uint32_t index);

[[nodiscard]] bool touching(
  const talus::dynamics::World& world,
  uint32_t a,
  uint32_t b
);

}
