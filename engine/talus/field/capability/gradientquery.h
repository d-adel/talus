#pragma once

#include <glm/glm.hpp>

#include <concepts>

namespace talus::field {

template<typename T>
concept GradientQueryable = requires(const T& field, const glm::vec3& point) {
  { gradient(field, point) } -> std::convertible_to<glm::vec3>;
};

}
