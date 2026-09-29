#pragma once

#include <glm/glm.hpp>

#include <concepts>

namespace talus::field {

template<typename T>
concept StepBounded =
  requires(const T& field, const glm::vec3& point, const glm::vec3& direction) {
    { stepBound(field, point, direction) } -> std::convertible_to<float>;
  };

}
