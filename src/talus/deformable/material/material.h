#pragma once

#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <span>

namespace talus::deformable {

[[nodiscard]] const talus::PhysicalMaterial& laneMaterial(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
);

enum class ElementRotation : uint8_t {
  Polar,
  Invertible
};

struct ElasticParams {
  float youngsModulus = 0.0f;
  float poissonsRatio = 0.3f;
  float density = 0.0f;
};

struct TetStiffness {
  std::array<glm::mat3, 16> block{};
  std::array<glm::vec3, 4> gradient{};
  float first = 0.0f;
  float second = 0.0f;
};

struct HexStiffness {
  std::array<glm::mat3, 64> block{};
};

struct HexTrial {
  glm::mat3 rotation{1.0f};
  std::array<glm::vec3, 8> force{};
  float energy = 0.0f;
};

struct PlasticParams {
  float yieldStress = std::numeric_limits<float>::infinity();
  float hardeningModulus = 0.0f;
};

struct HexPointTrial {
  glm::mat3 rotation{1.0f};
  std::array<glm::vec3, 8> force{};
  std::array<talus::PlasticPoint, 8> points{};
  HexStiffness tangent;
  float energy = 0.0f;
  float plasticWork = 0.0f;
  uint32_t yielding = 0u;
  uint32_t loading = 0u;
};

struct NeoHookeanHex {
  std::array<glm::vec3, 8> force{};
  std::array<glm::vec3, 8> lossForce{};
  float energy = 0.0f;
};

struct TetPointTrial {
  glm::mat3 rotation{1.0f};
  std::array<glm::vec3, 4> force{};
  talus::PlasticPoint point{};
  TetStiffness tangent;
  float energy = 0.0f;
  float plasticWork = 0.0f;
  uint32_t yielding = 0u;
  uint32_t loading = 0u;
};

struct InitialStress {
  std::array<glm::mat3, 8> twist{};
  std::array<glm::mat3, 8> turn{};
};

struct TrialState {
  glm::mat3 rotation{1.0f};
  glm::mat3 stress{0.0f};
  std::array<glm::vec3, 4> force{};
  float energy = 0.0f;
};

[[nodiscard]] glm::mat3 polarRotation(const glm::mat3& gradient);

[[nodiscard]] std::array<glm::vec3, 4> shapeGradients(
  const glm::mat3& restInverse
);

[[nodiscard]] TetStiffness tetStiffness(
  const talus::PreparedTet& tet,
  const ElasticParams& params
);

[[nodiscard]] TrialState trial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const talus::PreparedTet& tet,
  const TetStiffness& stiffness,
  ElementRotation rotation = ElementRotation::Polar
);

void applyStiffness(
  const std::array<glm::vec3, 4>& direction,
  const glm::mat3& rotation,
  const TetStiffness& stiffness,
  std::array<glm::vec3, 4>& result
);

[[nodiscard]] std::array<glm::mat3, 4> stiffnessDiagonal(
  const TetStiffness& stiffness,
  const glm::mat3& rotation
);

[[nodiscard]] talus::CommittedTet commit(const TrialState& state);

[[nodiscard]] std::array<double, 8> hexShape(const glm::dvec3& local);

[[nodiscard]] std::array<glm::dvec3, 8> hexShapeGradient(
  const glm::dvec3& local
);

[[nodiscard]] HexStiffness hexStiffness(
  const talus::PreparedHex& hex,
  const ElasticParams& params
);

[[nodiscard]] glm::mat3 hexRotation(
  const std::array<glm::vec3, 8>& deformed,
  const glm::vec3& halfExtents,
  ElementRotation rotation = ElementRotation::Polar
);

[[nodiscard]] HexTrial hexTrial(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const talus::PreparedHex& hex,
  const HexStiffness& stiffness,
  ElementRotation rotation = ElementRotation::Polar
);

void applyHexStiffness(
  const std::array<glm::vec3, 8>& direction,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  std::array<glm::vec3, 8>& result
);

[[nodiscard]] std::array<glm::mat3, 8> hexStiffnessDiagonal(
  const HexStiffness& stiffness,
  const glm::mat3& rotation
);

[[nodiscard]] bool yields(const PlasticParams& params) noexcept;

[[nodiscard]] HexPointTrial hexPointTrial(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const talus::PreparedHex& hex,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const std::array<talus::PlasticPoint, 8>& committed,
  float margin = 0.0f,
  ElementRotation rotation = ElementRotation::Polar,
  bool returnMapping = true,
  bool tangent = true,
  bool rotationDerivative = false
);

[[nodiscard]] NeoHookeanHex neoHookeanHex(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const std::array<glm::vec3, 8>& velocity,
  const talus::PreparedHex& hex,
  const ElasticParams& elastic,
  float hysteresis,
  float step
);

[[nodiscard]] TetPointTrial tetPointTrial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const talus::PreparedTet& tet,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const talus::PlasticPoint& committed,
  float margin = 0.0f,
  ElementRotation rotation = ElementRotation::Polar,
  bool returnMapping = true,
  bool tangent = true
);

[[nodiscard]] InitialStress initialStress(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const talus::PreparedTet& tet,
  const TetStiffness& stiffness,
  const glm::mat3& rotation,
  const std::array<glm::vec3, 4>& force
);

[[nodiscard]] InitialStress initialStress(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const talus::PreparedHex& hex,
  const HexStiffness& stiffness,
  const glm::mat3& rotation,
  const std::array<glm::vec3, 8>& force
);

void applyInitialStress(
  const InitialStress& stress,
  std::span<const glm::vec3> direction,
  std::span<glm::vec3> result
) noexcept;

}
