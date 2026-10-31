#pragma once

#include <talus/dynamics/joints/joints.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/rays/worldray.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace fixture {

inline constexpr glm::dvec3 characterHalf{0.3, 0.8, 0.3};
inline constexpr double characterMass = 80.0;
inline constexpr double groundForce = 6000.0;
inline constexpr glm::dvec3 footAnchor{0.0, -0.5, 0.0};
inline constexpr double pogoRest = 0.9;
inline constexpr double pogoHertz = 3.0;
inline constexpr double pogoDamping = 0.8;
inline constexpr double capScale = 100.0;
inline constexpr uint32_t capSubsteps = 4u;

enum class PogoUpdate : uint8_t { Recreate, InPlace };

struct MoverStep {
  glm::dvec3 velocity{0.0};
  double maxForce = groundForce;
  std::vector<talus::dynamics::RaySkip> skips;
  PogoUpdate update = PogoUpdate::Recreate;
};

struct Character {
  uint32_t body = talus::dynamics::noBody;
  uint32_t planar = 0u;
  uint32_t pogo = 0u;
  uint32_t ground = talus::dynamics::noBody;
  bool onGround = false;
};

struct Grip {
  uint32_t hand = talus::dynamics::noBody;
  uint32_t held = talus::dynamics::noBody;
  uint32_t joint = 0u;
};

struct SoftBlock {
  talus::field::SampledSdf field;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

[[nodiscard]] const talus::field::SampledSdf& characterField();

[[nodiscard]] const talus::field::SampledSdf& groundField();

[[nodiscard]] const talus::field::SampledSdf& stepField();

[[nodiscard]] Character addCharacter(
  talus::dynamics::World& world,
  const glm::dvec3& at,
  uint32_t planarBase = talus::dynamics::noBody
);

[[nodiscard]] uint32_t addUprightSpring(
  talus::dynamics::World& world,
  uint32_t body
);

[[nodiscard]] Grip addGrip(
  talus::dynamics::World& world,
  uint32_t body,
  const glm::dvec3& offset,
  const glm::dvec3& heldHalf,
  double heldMass
);

void driveGrip(
  talus::dynamics::World& world,
  const Grip& grip,
  uint32_t body,
  const glm::dvec3& offset
);

[[nodiscard]] double weightOf(const talus::dynamics::World& world);

[[nodiscard]] talus::dynamics::PogoSettings pogoFor(
  const talus::dynamics::World& world,
  const glm::dvec3& normal
);

std::optional<talus::dynamics::WorldRayHit> moveCharacter(
  talus::dynamics::World& world,
  Character& character,
  const MoverStep& step
);

[[nodiscard]] double springResponse(
  double hertz,
  double dampingRatio,
  double displacement,
  double time
);

[[nodiscard]] double springPeakSpeed(
  double hertz,
  double dampingRatio,
  double displacement
);

[[nodiscard]] double springOvershoot(double dampingRatio);

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
