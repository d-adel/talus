#include "defaultworld.ih"

namespace talus::objectmodel {

void addDefaultEntities(World& world) {
  const Entity camera = world.createAuthored();

  Pose seat{};
  seat.position = glm::vec3(0.0f, 1.0f, 6.0f);
  seat.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

  world.emplaceAuthored<Pose>(camera, seat);
  world.emplaceAuthored<Lens>(camera, Lens{});

  if (!world.setMainCamera(camera)) {
    throw std::runtime_error(
      "The default camera could not become the main one"
    );
  }

  const Entity sun = world.createAuthored();

  world.emplaceAuthored<Pose>(sun, Pose{});
  world.emplaceAuthored<DirectionalLight>(sun, DirectionalLight{});

  const Entity sky = world.createAuthored();

  world.emplaceAuthored<Pose>(sky, Pose{});
  world.emplaceAuthored<Environment>(sky, Environment{});
}

}
