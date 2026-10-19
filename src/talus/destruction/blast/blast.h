#pragma once

#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/pipeline/pipeline.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace talus::destruction {

struct Blast {
  glm::dvec3 centre{0.0};
  double strength = 0.0;
  double radius = 0.0;
  uint64_t step = 0u;
};

struct BlastImpulse {
  uint32_t body = 0u;
  uint32_t child = dynamics::noChild;
  glm::dvec3 impulse{0.0};
  glm::dvec3 point{0.0};
};

[[nodiscard]] double blastStrength(double impulsePerArea, double distance, double radius);

[[nodiscard]] double blastFraction(double exposed, double distance, double radius);

[[nodiscard]] std::vector<BlastImpulse> blastImpulses(const dynamics::World& world, const Blast& blast);

}
