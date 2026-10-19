#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <span>

namespace talus::dynamics {

struct BoxPose {
  glm::dvec3 centre{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 half{0.0};
};

struct ManifoldPoint {
  glm::dvec3 point{0.0};
  double separation = 0.0;
  uint32_t feature = 0u;
};

struct Manifold {
  glm::dvec3 normal{0.0, 1.0, 0.0};
  std::array<ManifoldPoint, 4> points{};
  uint32_t count = 0u;
};

[[nodiscard]] Manifold collideSpheres(
  const glm::dvec3& centreA,
  double radiusA,
  const glm::dvec3& centreB,
  double radiusB,
  double speculative
);
[[nodiscard]] Manifold collideBoxAndSphere(
  const glm::dvec3& sphereCentre,
  double radius,
  const BoxPose& box,
  double speculative
);
[[nodiscard]] Manifold collideBoxes(
  const BoxPose& a,
  const BoxPose& b,
  double speculative
);
[[nodiscard]] uint32_t reduceManifold(
  std::span<const glm::dvec3> points,
  std::span<const double> separations,
  const glm::dvec3& normal,
  double cutoff,
  double tolerance,
  std::array<uint32_t, 4>& chosen
);

}
