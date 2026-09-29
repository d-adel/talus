#include <talus/field/field.h>
#include <talus/field/analytic/analytic.h>

#include <glm/gtc/matrix_transform.hpp>

namespace talus::field {

Primitive makeSphere(
  glm::vec3 position,
  float radius,
  uint32_t material,
  uint32_t operation,
  float blendRadius
) {
  Primitive primitive{};

  const glm::mat4 model = glm::translate(glm::mat4(1.0f), position);

  primitive.inverseModel = glm::inverse(model);

  primitive.shape = glm::vec4(radius, 0.0f, 0.0f, blendRadius);

  primitive.shapeConfig = glm::uvec4(0, material, operation, 0);

  updateBounds(primitive);

  return primitive;
}

Primitive makeBox(
  glm::vec3 position,
  glm::vec3 halfExtents,
  float rotationRadians,
  glm::vec3 rotationAxis,
  uint32_t material,
  uint32_t operation,
  float blendRadius
) {
  Primitive primitive{};

  const glm::mat4 model =
    glm::translate(glm::mat4(1.0f), position)
    * glm::rotate(glm::mat4(1.0f), rotationRadians, rotationAxis);

  primitive.inverseModel = glm::inverse(model);

  primitive.shape = glm::vec4(halfExtents, blendRadius);

  primitive.shapeConfig = glm::uvec4(1, material, operation, 0);

  updateBounds(primitive);

  return primitive;
}

Primitive makePlane(glm::vec3 position, uint32_t material, uint32_t operation) {
  Primitive primitive{};

  const glm::mat4 model = glm::translate(glm::mat4(1.0f), position);

  primitive.inverseModel = glm::inverse(model);

  primitive.shape = glm::vec4(0.0f);

  primitive.shapeConfig = glm::uvec4(2, material, operation, 0);

  updateBounds(primitive);

  return primitive;
}

Primitive makeCylinder(
  glm::vec3 position,
  float radius,
  float halfHeight,
  float rotationRadians,
  glm::vec3 rotationAxis,
  uint32_t material,
  uint32_t operation,
  float blendRadius
) {
  Primitive primitive{};

  const glm::mat4 model =
    glm::translate(glm::mat4(1.0f), position)
    * glm::rotate(glm::mat4(1.0f), rotationRadians, rotationAxis);

  primitive.inverseModel = glm::inverse(model);

  primitive.shape = glm::vec4(radius, halfHeight, 0.0f, blendRadius);

  primitive.shapeConfig = glm::uvec4(3, material, operation, 0);

  updateBounds(primitive);

  return primitive;
}

}
