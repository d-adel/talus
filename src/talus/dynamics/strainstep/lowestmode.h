#pragma once

#include <talus/dynamics/strainstep/element.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>

namespace talus::dynamics {

[[nodiscard]] double lowestAngular(
  std::span<const StrainElement> elements,
  std::span<const double> mass,
  std::span<const uint8_t> pinned,
  std::span<const glm::dvec3> rest
);

}
