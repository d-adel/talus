#include "lookthrough.ih"

namespace talus::view {

bool bindingIsLive(
  const Viewport& port,
  const talus::objectmodel::World& world
) {
  if (port.camera == talus::objectmodel::INVALID_ENTITY) {
    return false;
  }

  return world.registry()
    .all_of<talus::objectmodel::Pose, talus::objectmodel::Lens>(port.camera);
}

talus::objectmodel::CameraView lookThrough(
  const Viewport& port,
  const talus::objectmodel::World& world,
  float aspect
) {
  if (!bindingIsLive(port, world)) {
    return talus::objectmodel::cameraView(port.pose, port.lens, aspect);
  }

  const entt::registry& registry = world.registry();

  return talus::objectmodel::cameraView(
    registry.get<const talus::objectmodel::Pose>(port.camera),
    registry.get<const talus::objectmodel::Lens>(port.camera),
    aspect
  );
}

}
