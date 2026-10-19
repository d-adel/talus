#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/strainstep/element.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace talus::dynamics {

struct ReturnMap {
  glm::dmat3 elastic{1.0};
  double strain = 0.0;
  double trial = 0.0;
};

[[nodiscard]] double vonMises(const glm::dmat3& elastic, double shear);

[[nodiscard]] std::optional<ReturnMap> returnMap(
  const glm::dmat3& elastic,
  double shear,
  double yieldStress
);

double flowLattice(LatticeBody& lattice);

[[nodiscard]] bool flowed(const LatticeBody& lattice);

[[nodiscard]] ElementRest plasticRest(const LatticeBody& lattice, uint32_t cell);

[[nodiscard]] bool fitHolds(const ElementRest& rest);

void writeFitted(LatticeBody& lattice, uint32_t cell, const ElementRest& rest);

[[nodiscard]] glm::dquat latticeTurn(const LatticeBody& lattice);

[[nodiscard]] std::vector<glm::dvec3> plasticShape(const LatticeBody& lattice, uint32_t iterations);

}
