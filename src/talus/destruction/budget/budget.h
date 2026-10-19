#pragma once

#include <talus/dynamics/pipeline/pipeline.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace talus::destruction {

struct StepCounts {
  uint32_t moving = 0u;
  uint32_t awakeChunks = 0u;
  uint32_t bodies = 0u;
  uint32_t visited = 0u;
  uint32_t gathered = 0u;
  uint32_t awakeManifolds = 0u;
  uint32_t fractures = 0u;
  uint32_t broken = 0u;
  uint64_t factorWork = 0u;
  uint64_t wideWork = 0u;
  uint32_t solvedChunks = 0u;
  uint32_t updates = 0u;
};

struct CostModel {
  std::array<double, 3> rigid{};
  std::array<double, 6> update{};
  std::array<double, 4> stress{};
  std::string_view source;
};

struct StepCost {
  double rigid = 0.0;
  double update = 0.0;
  double stress = 0.0;
  double total = 0.0;
};

inline constexpr double restSpeed = 0.3;
inline constexpr double restTime = 0.5;
inline constexpr double budgetTarget = 3.6;
inline constexpr double collapseCeiling = 12.37;
inline constexpr size_t queuedCases = 8u;

inline constexpr CostModel costModel{
  {0.0012981987195001723, 0.0011491332827081278, 5.7604754316642439e-05},
  {0.00013000681595005446, 0.0017382431390538483, 2.5306361988446604e-05, 0.16659290381227379, 0.0032315477604960411,
   0.00015620148616340832},
  {8.6437416596044472e-05, 8.1221456376982999e-06, 0.00069774740458902068, 0.0},
  "docs/physics-status.md: 2026-10-19 fit; model remains unqualified"
};

[[nodiscard]] double rigidCost(const StepCounts& counts, const CostModel& model = costModel);

[[nodiscard]] double updateCost(const StepCounts& counts, const CostModel& model = costModel);

[[nodiscard]] double stressCost(const StepCounts& counts, const CostModel& model = costModel);

[[nodiscard]] StepCost predictCost(const StepCounts& counts, const CostModel& model = costModel);

class RestTracker {
public:
  void update(const dynamics::World& world, const dynamics::Workers& workers);
  void reset(uint32_t body) noexcept;

  [[nodiscard]] bool resting(uint32_t body) const noexcept;
  [[nodiscard]] uint32_t restSteps(uint32_t body) const noexcept;

private:
  std::vector<uint32_t> m_steps;
  double m_dt = 0.0;
};

}
