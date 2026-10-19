#include <talus/talus.h>

#include <array>
#include <cmath>
#include <iostream>

int main() {
  talus::World world;
  world.dynamics.workerCount = 8u;
  const std::array<talus::dynamics::CompoundChild, 1> floor{
    talus::dynamics::CompoundChild{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {8.0, 0.5, 8.0}, 1000.0, 0u}
  };
  const std::array<talus::dynamics::CompoundChild, 1> box{
    talus::dynamics::CompoundChild{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {0.5, 0.5, 0.5}, 500.0, 1u}
  };
  talus::dynamics::addCompoundBody(world.dynamics.bodies, floor, {0.0, -0.5, 0.0},
    {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.6, 0.0, true);
  const uint32_t body = talus::dynamics::addCompoundBody(world.dynamics.bodies, box,
    {0.0, 3.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.6, 0.0, false);
  for (uint32_t tick = 0u; tick < 360u; ++tick) {
    if (world.step().dynamics.nonFinite) {
      return 1;
    }
  }
  const auto& state = world.dynamics.bodies.rigids.at(body);
  std::cout << "box " << state.id << " position " << state.position.x << ' '
            << state.position.y << ' ' << state.position.z << '\n';
  return std::isfinite(state.position.y) && state.position.y >= 0.48 && state.position.y <= 0.53 ? 0 : 1;
}
