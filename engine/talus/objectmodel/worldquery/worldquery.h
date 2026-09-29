#pragma once

#include <talus/field/raycast/raycast.h>
#include <talus/objectmodel/world/world.h>

#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <glm/glm.hpp>

#include <optional>

namespace talus::objectmodel {

struct WorldExtent {
  glm::vec3 minimum{0.0f};
  glm::vec3 maximum{0.0f};

  bool empty = true;

  [[nodiscard]] glm::vec3 centre() const noexcept;
  [[nodiscard]] glm::vec3 span() const noexcept;
};

[[nodiscard]] WorldExtent sceneExtent(const talus::objectmodel::World& world);

[[nodiscard]] WorldExtent entityExtent(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity entity
);

template<typename... Ts>
[[nodiscard]] talus::objectmodel::Entity firstWith(
  const talus::objectmodel::World& world
) {
  const entt::registry& registry = world.registry();

  talus::objectmodel::Entity found = talus::objectmodel::INVALID_ENTITY;

  for (const talus::objectmodel::Entity entity : registry.view<const Ts...>()) {
    if (
      found == talus::objectmodel::INVALID_ENTITY
      || entt::to_entity(entity) < entt::to_entity(found)
    ) {
      found = entity;
    }
  }

  return found;
}

[[nodiscard]] talus::objectmodel::Entity mainCamera(
  const talus::objectmodel::World& world
);

[[nodiscard]] talus::objectmodel::Entity firstLens(
  const talus::objectmodel::World& world
);

[[nodiscard]] talus::objectmodel::Entity firstDeformable(
  const talus::objectmodel::World& world
);

struct Framing {
  glm::vec3 position{0.0f};
  float yaw = 0.0f;
  float pitch = 0.0f;
};

[[nodiscard]] Framing frameExtent(const WorldExtent& extent);

[[nodiscard]] Pose framingPose(const Framing& framing);

struct CameraView {
  glm::mat4 view{1.0f};
  glm::mat4 projection{1.0f};
  glm::vec3 position{0.0f};
  float nearPlane = 0.1f;
};

[[nodiscard]] CameraView cameraView(
  const Pose& pose,
  const Lens& lens,
  float aspect
);

[[nodiscard]] std::optional<CameraView> activeCamera(
  const talus::objectmodel::World& world,
  float aspect
);

[[nodiscard]] std::optional<talus::field::Ray> screenPointToRay(
  const talus::objectmodel::World& world,
  talus::objectmodel::Entity camera,
  const glm::vec2& screen,
  const glm::vec2& viewport
);

[[nodiscard]] CameraView framingCamera(
  const talus::objectmodel::World& world,
  float aspect
);

}
