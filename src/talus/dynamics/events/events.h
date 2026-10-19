#pragma once

#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>

#include <vector>

namespace talus::dynamics {

struct StartVelocities {
  std::vector<glm::dvec3> linear;
  std::vector<glm::dvec3> angular;
  std::vector<std::vector<glm::dvec3>> lattice;
};

[[nodiscard]] StartVelocities startVelocities(const BodySet& bodies);

[[nodiscard]] std::vector<ContactHitEvent> gatherHits(
  const World& world,
  const StartVelocities& start,
  std::span<const uint8_t> latticeOwned,
  std::span<const uint32_t> candidates
);

}
