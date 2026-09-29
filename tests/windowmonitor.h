#pragma once

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/runtime/runtime/runtime.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace windowmonitor {

inline constexpr double ANGULAR = 1.0e-4;
inline constexpr double MOMENTUM = 1.0e-3;
inline constexpr double KINETIC = 1.0e-6;
inline constexpr double HANDBACK = 1.0e-6;
inline constexpr double RESTING_DRIFT = 2.1e-8;

struct MonitoredBody {
  double mass = 0.0;
  double gyration = 0.0;
  double inertiaRatio = 1.0;
};

[[nodiscard]] MonitoredBody monitoredBody(
  const talus::objectmodel::PreparedMesh& mesh
);

[[nodiscard]] double linearFloor(const MonitoredBody& body);

[[nodiscard]] double angularFloor(const MonitoredBody& body);

[[nodiscard]] double kineticFloor(const MonitoredBody& body);

[[nodiscard]] double transitionBound(const MonitoredBody& body);

[[nodiscard]] double momentumApart(
  const glm::dvec3& before,
  const glm::dvec3& after,
  double floor
);

[[nodiscard]] double restingAllowance(double restingMass, uint32_t frames);

[[nodiscard]] std::vector<std::string> windowViolations(
  const talus::runtime::WindowRecord& record,
  uint32_t frame,
  double epsilon,
  const MonitoredBody& body,
  double restingMass = 0.0
);

[[nodiscard]] std::vector<std::string> frameViolations(
  std::span<const talus::runtime::WindowRecord> records,
  uint32_t frame,
  double epsilon,
  const std::function<MonitoredBody(talus::objectmodel::Entity)>& bodyOf,
  double restingMass = 0.0
);

[[nodiscard]] std::vector<std::string> handbackViolations(
  const talus::runtime::PartnerHandback& handback,
  uint32_t frame,
  double epsilon
);

[[nodiscard]] std::vector<std::string> transitionViolations(
  const talus::runtime::YieldTransition& transition,
  uint32_t frame,
  const MonitoredBody& body
);

}
