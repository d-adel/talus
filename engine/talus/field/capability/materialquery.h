#pragma once

#include <glm/glm.hpp>

#include <concepts>
#include <cstdint>

namespace talus::field {

template<typename T>
concept MaterialQueryable = requires(const T& field, const glm::vec3& point) {
  { material(field, point) } -> std::convertible_to<uint32_t>;
};

}
