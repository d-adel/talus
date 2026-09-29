#pragma once

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/field.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdint>

namespace fixture {

inline constexpr uint32_t SPHERE_KIND = 0u;
inline constexpr uint32_t BOX_KIND = 1u;

inline talus::field::Primitive sphere(
  const glm::vec3& centre,
  float radius,
  uint32_t operation = talus::core::FIELD_OP_UNION
) {
  talus::field::Primitive primitive{};

  primitive.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), centre));
  primitive.shape = glm::vec4(radius, 0.0f, 0.0f, 0.0f);
  primitive.shapeConfig = glm::uvec4(SPHERE_KIND, 1u, operation, 0u);

  talus::field::updateBounds(primitive);

  return primitive;
}

inline talus::field::Primitive box(
  const glm::vec3& centre,
  const glm::vec3& half,
  uint32_t operation = talus::core::FIELD_OP_UNION
) {
  talus::field::Primitive primitive{};

  primitive.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), centre));
  primitive.shape = glm::vec4(half, 0.0f);
  primitive.shapeConfig = glm::uvec4(BOX_KIND, 1u, operation, 0u);

  talus::field::updateBounds(primitive);

  return primitive;
}

}
