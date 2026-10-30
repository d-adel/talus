#pragma once

#include <talus/dynamics/joints/joints.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <cstdint>
#include <optional>

namespace talus::dynamics {

[[nodiscard]] uint32_t addJoint(World& world, const JointDef& def);

bool removeJoint(World& world, uint32_t joint);

bool setJointDrive(World& world, uint32_t joint, const JointDrive& drive);

bool setSpringTarget(World& world, uint32_t joint, const JointFrame& target);

bool setSpringSettings(
  World& world,
  uint32_t joint,
  const SpringSettings& spring
);

[[nodiscard]] std::optional<JointDef> jointDef(
  const World& world,
  uint32_t joint
);

[[nodiscard]] std::optional<JointReport> jointReport(
  const World& world,
  uint32_t joint
);

}
