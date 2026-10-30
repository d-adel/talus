#pragma once

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/joints.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <cstdint>
#include <optional>
#include <span>

namespace talus::dynamics {

struct BodyRebase {
  uint32_t body = noBody;
  JointFrame oldToNew{};
  std::span<const CompoundChild> children{};
};

[[nodiscard]] uint32_t addJoint(World& world, const JointDef& def);

bool removeJoint(World& world, uint32_t joint);

bool setJointDrive(World& world, uint32_t joint, const JointDrive& drive);

bool setSpringTarget(World& world, uint32_t joint, const JointFrame& target);

void wakeJointPartners(World& world, uint32_t id);

void rebaseJoints(
  World& world,
  uint32_t oldBody,
  std::span<const BodyRebase> pieces
);

[[nodiscard]] std::optional<JointReport> jointReport(
  const World& world,
  uint32_t joint
);

void pruneJoints(World& world);

}
