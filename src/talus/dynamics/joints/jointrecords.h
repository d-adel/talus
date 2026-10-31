#pragma once

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/joints.h>

#include <cstdint>
#include <span>

namespace talus::dynamics {

struct World;

struct JointRecords {
  [[nodiscard]] static uint32_t add(JointSet& joints, const JointDef& def);
  static bool remove(JointSet& joints, uint32_t id);
  static bool rebind(
    JointSet& joints,
    uint32_t id,
    uint32_t bodyA,
    uint32_t bodyB
  );
  [[nodiscard]] static Joint* find(JointSet& joints, uint32_t id);
  [[nodiscard]] static std::span<Joint> all(JointSet& joints);
};

struct BodyRebase {
  uint32_t body = noBody;
  JointFrame oldToNew{};
  std::span<const CompoundChild> children{};
};

void wakeJointPartners(World& world, uint32_t id);

void clearPogoMotion(World& world, uint32_t body);

void rebaseJoints(
  World& world,
  uint32_t oldBody,
  std::span<const BodyRebase> pieces
);

void removeBodyJoints(World& world, std::span<const uint32_t> bodies);

}
