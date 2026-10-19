#pragma once

#include <talus/dynamics/bodyset/bodyset.h>

#include <glm/glm.hpp>

namespace talus::dynamics {

struct Energy {
  double kinetic = 0.0;
  double gravity = 0.0;
  double elastic = 0.0;

  [[nodiscard]] double total() const;
};

[[nodiscard]] Energy latticeEnergy(
  const LatticeBody& body,
  const glm::dvec3& gravity
);

[[nodiscard]] Energy rigidEnergy(
  const RigidBody& body,
  const glm::dvec3& gravity
);

[[nodiscard]] Energy energyOf(
  const BodySet& bodies,
  const glm::dvec3& gravity
);

}
