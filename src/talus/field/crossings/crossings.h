#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace talus::field {

struct Crossing {
  glm::vec3 lattice{0.0f};
  uint32_t axis = 0u;
};

struct LatticeSurface {
  std::vector<Crossing> crossings;
  std::vector<glm::uvec3> inside;
};

LatticeSurface latticeSurface(const SampledSdf& field);

glm::vec3 latticeToField(const SampledSdf& field, const glm::vec3& lattice);

}
