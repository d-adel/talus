#include "renderpose.h"

namespace talus::objectmodel {

const Pose& drawnPose(const entt::registry& registry, Entity entity) {
  if (const auto* drawn = registry.try_get<RenderPose>(entity)) {
    return drawn->pose;
  }

  return registry.get<Pose>(entity);
}

}
