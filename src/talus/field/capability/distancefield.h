#pragma once

#include <glm/glm.hpp>

#include <concepts>

namespace talus::field {

template<typename T>
concept DistanceField = requires(const T& field, const glm::vec3& point) {
  { distance(field, point) } -> std::convertible_to<float>;
};

}
