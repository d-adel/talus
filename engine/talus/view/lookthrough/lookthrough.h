#pragma once

#include <talus/objectmodel/world/world.h>
#include <talus/objectmodel/worldquery/worldquery.h>
#include <talus/view/viewport/viewport.h>

namespace talus::view {

[[nodiscard]] talus::objectmodel::CameraView lookThrough(
  const Viewport& port,
  const talus::objectmodel::World& world,
  float aspect
);

[[nodiscard]] bool bindingIsLive(
  const Viewport& port,
  const talus::objectmodel::World& world
);

}
