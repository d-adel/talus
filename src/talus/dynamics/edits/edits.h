#pragma once

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

void dropContacts(World& world, Side side, uint32_t index, bool renumber);

void destroyBodyContacts(World& world, uint32_t id);

void resetContacts(World& world);

void forgetBody(World& world, Side side, uint32_t index, uint32_t id);

void eraseRigids(BodySet& bodies, std::span<const uint8_t> gone);

void eraseLattice(BodySet& bodies, uint32_t index);

bool removeBody(World& world, uint32_t id);

uint32_t removeBodies(World& world, std::span<const uint32_t> ids);

bool removeStatic(World& world, uint32_t id);

bool setRigidPose(
  World& world,
  uint32_t id,
  const glm::dvec3& position,
  const glm::dquat& rotation
);

bool setVelocity(World& world, uint32_t id, const glm::dvec3& velocity);

[[nodiscard]] bool applyImpulse(
  World& world,
  uint32_t id,
  const glm::dvec3& impulse,
  const glm::dvec3& point
);

[[nodiscard]] bool setAngularVelocity(World& world, uint32_t id, const glm::dvec3& spin);

bool setMass(World& world, uint32_t id, double mass);

bool freezeBody(World& world, uint32_t id);
bool freezeBody(World& world, const BodyRef& ref);

bool thawBody(World& world, uint32_t id);

[[nodiscard]] bool isFrozen(const RigidBody& body);

uint32_t thawStruck(
  World& world,
  double speed,
  const std::vector<uint32_t>* live = nullptr
);

bool replaceRigidShape(World& world, uint32_t id, const RigidBody& shape);

bool setStaticPose(
  World& world,
  uint32_t id,
  const glm::dvec3& position,
  const glm::dquat& rotation
);

[[nodiscard]] bool removeCompoundChildren(
  World& world,
  uint32_t id,
  std::span<const uint32_t> keys
);

[[nodiscard]] std::vector<uint32_t> splitCompound(
  World& world,
  uint32_t id,
  std::span<const std::vector<uint32_t>> groups
);

[[nodiscard]] bool replaceCompoundChildren(
  World& world,
  uint32_t id,
  std::span<const ChildReplacement> replacements
);

}
