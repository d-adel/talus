#pragma once

#include <glaze/glaze.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

template<>
struct glz::meta<glm::vec3> {
  using T = glm::vec3;
  static constexpr auto value = glz::array(&T::x, &T::y, &T::z);
};

template<>
struct glz::meta<glm::vec4> {
  using T = glm::vec4;
  static constexpr auto value = glz::array(&T::x, &T::y, &T::z, &T::w);
};

template<>
struct glz::meta<glm::uvec3> {
  using T = glm::uvec3;
  static constexpr auto value = glz::array(&T::x, &T::y, &T::z);
};

template<>
struct glz::meta<glm::uvec4> {
  using T = glm::uvec4;
  static constexpr auto value = glz::array(&T::x, &T::y, &T::z, &T::w);
};

template<>
struct glz::meta<glm::quat> {
  using T = glm::quat;
  static constexpr auto value = glz::array(&T::w, &T::x, &T::y, &T::z);
};
