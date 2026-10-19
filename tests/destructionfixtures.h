#pragma once

#include <talus/destruction/destructionworld/destructionworld.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>

#include <cstdint>

#include <string>

namespace destructionfixture {

struct Motion {
  glm::dvec3 momentum{0.0};
  glm::dvec3 angular{0.0};
  double energy = 0.0;
  double mass = 0.0;
};

[[nodiscard]] Motion motionOf(const talus::dynamics::World& world);

[[nodiscard]] double linearScale(const Motion& motion);

[[nodiscard]] double angularScale(const Motion& motion);

[[nodiscard]] double energyScale(const Motion& motion);

[[nodiscard]] std::string mismatch(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
);

[[nodiscard]] double heldEnergy(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
);

[[nodiscard]] uint64_t stateHash(
  const talus::destruction::DestructionWorld& destruction,
  const talus::dynamics::World& world
);

[[nodiscard]] bool sameOutcome(const talus::destruction::DestructionReport& a, const talus::destruction::DestructionReport& b);

}
