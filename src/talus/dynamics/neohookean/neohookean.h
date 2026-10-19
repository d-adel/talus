#pragma once

#include <talus/material.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>

namespace talus::dynamics {

struct Moduli {
  double shear = 0.0;
  double shifted = 0.0;
};

using Corners = std::array<glm::dvec3, 8>;
using GaussGradients = std::array<std::array<glm::dvec3, 8>, 8>;

struct NodeTerms {
  glm::dvec3 gradient{0.0};
  glm::dmat3 hessian{0.0};
};

[[nodiscard]] Moduli moduliOf(const talus::PhysicalMaterial& material);

[[nodiscard]] std::array<glm::dvec3, 8> shapeGradients(
  const glm::dvec3& at,
  const glm::dvec3& halfExtents
);

[[nodiscard]] GaussGradients gaussGradients(const glm::dvec3& halfExtents);

[[nodiscard]] double cellEnergy(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli
);

[[nodiscard]] NodeTerms nodeTerms(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli,
  uint32_t corner
);

[[nodiscard]] NodeTerms nodeTerms(
  const Corners& displacement,
  const GaussGradients& gradients,
  double volume,
  const Moduli& moduli,
  uint32_t corner
);

[[nodiscard]] std::array<std::array<glm::dmat3, 8>, 8> restStiffness(
  const glm::dvec3& halfExtents,
  double volume,
  const Moduli& moduli
);

[[nodiscard]] double volumeRatio(
  const Corners& deformed,
  const Corners& rest,
  const glm::dvec3& halfExtents
);

}
