#pragma once

#include <talus/objectmodel/entity/entity.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>

namespace talus::objectmodel {

struct Impact {
  Entity entity = INVALID_ENTITY;

  Entity other = INVALID_ENTITY;

  glm::vec3 point{0.0f};

  glm::vec3 outward{0.0f};

  float impulse = 0.0f;

  float speed = 0.0f;

  uint32_t material = 0u;

  uint32_t otherMaterial = 0u;
};

void sortStrongestFirst(std::span<Impact> impacts);

}
