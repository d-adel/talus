#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>

namespace talus::field {

struct Primitive {
  glm::mat4 inverseModel;
  glm::vec4 shape;
  glm::uvec4 shapeConfig;
  glm::vec4 bounds;
};

static_assert(
  offsetof(Primitive, inverseModel) == 0,
  "Primitive must keep the layout the field shader indexes by"
);
static_assert(
  offsetof(Primitive, shape) == 64,
  "Primitive must keep the layout the field shader indexes by"
);
static_assert(
  offsetof(Primitive, shapeConfig) == 80,
  "Primitive must keep the layout the field shader indexes by"
);
static_assert(
  offsetof(Primitive, bounds) == 96,
  "Primitive must keep the layout the field shader indexes by"
);
static_assert(
  sizeof(Primitive) == 112,
  "Primitive must keep the layout the field shader indexes by"
);

Primitive makeSphere(
  glm::vec3 position,
  float radius,
  uint32_t material = 0,
  uint32_t operation = 0,
  float blendRadius = 0.0f
);

Primitive makeBox(
  glm::vec3 position,
  glm::vec3 halfExtents,
  float rotationRadians = 0.0f,
  glm::vec3 rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f),
  uint32_t material = 0,
  uint32_t operation = 0,
  float blendRadius = 0.0f
);

Primitive makeCylinder(
  glm::vec3 position,
  float radius,
  float halfHeight,
  float rotationRadians = 0.0f,
  glm::vec3 rotationAxis = glm::vec3(0.0f, 1.0f, 0.0f),
  uint32_t material = 0,
  uint32_t operation = 0,
  float blendRadius = 0.0f
);

Primitive makePlane(
  glm::vec3 position,
  uint32_t material = 0,
  uint32_t operation = 0
);

}
