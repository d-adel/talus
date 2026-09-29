#pragma once

#include <talus/field/raycast/raycast.h>
#include <talus/objectmodel/entity/entity.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace talus::objectmodel {
class World;
}

namespace talus::physics {
class PhysicsWorld;
}

namespace talus::query {

struct Context {
  const objectmodel::World& world;
  const physics::PhysicsWorld* physics = nullptr;
};

struct Hit {
  objectmodel::Entity entity = objectmodel::INVALID_ENTITY;
  glm::vec3 position{0.0f};
  glm::vec3 normal{0.0f};
  float distance = 0.0f;
  uint32_t material = 0u;
};

[[nodiscard]] std::optional<Hit> raycast(
  const Context& context,
  const field::Ray& ray,
  float maxDistance = field::MARCH_MAX_DISTANCE
);

[[nodiscard]] std::optional<Hit> sweep(
  const Context& context,
  objectmodel::Entity body,
  const glm::vec3& displacement
);

[[nodiscard]] std::vector<objectmodel::Entity> overlap(
  const Context& context,
  objectmodel::Entity body
);
}
