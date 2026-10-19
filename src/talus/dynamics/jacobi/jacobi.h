#pragma once

#include <glm/glm.hpp>

namespace talus::dynamics {

struct SymmetricEigen {
  glm::dvec3 values{0.0};
  glm::dmat3 vectors{1.0};
};

[[nodiscard]] SymmetricEigen jacobiEigen(const glm::dmat3& matrix);

}
