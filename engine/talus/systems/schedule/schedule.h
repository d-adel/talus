#pragma once

#include <talus/debugdraw/debugdraw/debugdraw.h>
#include <talus/objectmodel/impact/impact.h>
#include <talus/objectmodel/world/world.h>
#include <talus/systems/input/input.h>

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace talus::query {
struct Context;
}

namespace talus::systems {

struct Context {
  talus::objectmodel::World& world;
  const InputState& input;
  float deltaSeconds = 0.0f;
  uint32_t frameIndex = 0;
  std::span<const talus::objectmodel::Impact> impacts = {};
  talus::debugdraw::DebugDraw* debug = nullptr;
  const talus::query::Context* queries = nullptr;
};

using Function = std::function<void(Context&)>;

enum class Phase : uint32_t {
  PrePhysics = 0,
  PostPhysics = 1,
  PostUpdateWork = 2
};

inline constexpr std::array<Phase, 3> PHASES{
  Phase::PrePhysics,
  Phase::PostPhysics,
  Phase::PostUpdateWork
};

[[nodiscard]] std::optional<Phase> phaseFrom(uint32_t value) noexcept;

struct Entry {
  Phase phase = Phase::PostPhysics;
  int32_t order = 0;
  std::string name;
  Function function;
  bool enabled = true;
  double lastMilliseconds = 0.0;
};

class Schedule {
public:
  [[nodiscard]] bool add(
    Phase phase,
    int32_t order,
    std::string name,
    Function function
  );

  [[nodiscard]] bool remove(std::string_view name);

  [[nodiscard]] bool enable(std::string_view name, bool wanted);

  [[nodiscard]] bool enabled(std::string_view name) const noexcept;

  void clear() noexcept;

  void run(Phase phase, Context& context);

  void run(Context& context);

  [[nodiscard]] std::span<const Entry> systems() const noexcept;

  [[nodiscard]] size_t size() const noexcept;

  [[nodiscard]] double lastMilliseconds() const noexcept;

  [[nodiscard]] double lastMilliseconds(std::string_view name) const noexcept;

private:
  [[nodiscard]] Entry* find(std::string_view name) noexcept;
  [[nodiscard]] const Entry* find(std::string_view name) const noexcept;

  std::vector<Entry> m_systems;
  double m_lastMilliseconds = 0.0;
};

}
