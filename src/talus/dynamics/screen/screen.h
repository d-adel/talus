#pragma once

#include <talus/dynamics/neohookean/neohookean.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace talus::dynamics {

struct ScreenFactor;

struct RestBody {
  std::vector<double> mass;
  double totalMass = 0.0;
  glm::dvec3 centre{0.0};
  glm::dmat3 inertia{0.0};
  glm::dquat axes{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 moments{0.0};
  double thickness = 0.0;
};

struct ScreenResult {
  std::vector<glm::dvec3> displacement;
  double dent = 0.0;
};

[[nodiscard]] RestBody restBodyOf(const talus::PreparedMesh& mesh);

[[nodiscard]] std::shared_ptr<const ScreenFactor> factorScreen(
  const talus::PreparedMesh& mesh,
  std::span<const Moduli> moduli,
  std::span<const uint32_t> held = {}
);

[[nodiscard]] ScreenResult screenLoad(
  const ScreenFactor& factor,
  const RestBody& rest,
  const talus::PreparedMesh& mesh,
  std::span<const glm::dvec3> nodeForces
);

}
