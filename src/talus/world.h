#pragma once

#include <talus/destruction/destructionworld/destructionworld.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <utility>

namespace talus {

struct StepReport {
  dynamics::StepReport dynamics;
  destruction::DestructionReport destruction;
};

// The host owns the clock and calls step once per fixed physics tick.
// Simulation data and events are independent of the host's renderer and ECS.
class World {
public:
  explicit World(const destruction::DestructionSettings& settings = {}) : destruction(settings) {}

  World(const World&) = delete;
  World& operator=(const World&) = delete;
  World(World&&) = default;
  World& operator=(World&&) = default;

  [[nodiscard]] StepReport step() {
    destruction.prepare(dynamics);
    auto motion = dynamics::step(dynamics);
    auto damage = destruction.update(dynamics);
    return {std::move(motion), std::move(damage)};
  }

  dynamics::World dynamics;
  destruction::DestructionWorld destruction;
};

}
