#pragma once

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <span>

namespace talus::deformable {

[[nodiscard]] const objectmodel::PhysicalMaterial& laneMaterial(
  std::span<const objectmodel::PhysicalMaterial> materials,
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
  std::array<objectmodel::PlasticPoint, 8> points{};
  HexStiffness tangent;
  float energy = 0.0f;
  float plasticWork = 0.0f;
  uint32_t yielding = 0u;
  uint32_t loading = 0u;
};

struct TetPointTrial {
  glm::mat3 rotation{1.0f};
  std::array<glm::vec3, 4> force{};
  objectmodel::PlasticPoint point{};
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
  const objectmodel::PreparedTet& tet,
  const ElasticParams& params
);

[[nodiscard]] TrialState trial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
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

[[nodiscard]] objectmodel::CommittedTet commit(const TrialState& state);

[[nodiscard]] HexStiffness hexStiffness(
  const objectmodel::PreparedHex& hex,
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
  const objectmodel::PreparedHex& hex,
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
  const objectmodel::PreparedHex& hex,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const std::array<objectmodel::PlasticPoint, 8>& committed,
  float margin = 0.0f,
  ElementRotation rotation = ElementRotation::Polar,
  bool returnMapping = true,
  bool tangent = true,
  bool rotationDerivative = false
);

[[nodiscard]] TetPointTrial tetPointTrial(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const ElasticParams& elastic,
  const PlasticParams& plastic,
  const objectmodel::PlasticPoint& committed,
  float margin = 0.0f,
  ElementRotation rotation = ElementRotation::Polar,
  bool returnMapping = true,
  bool tangent = true
);

[[nodiscard]] InitialStress initialStress(
  const std::array<glm::vec3, 4>& deformed,
  const std::array<glm::vec3, 4>& rest,
  const objectmodel::PreparedTet& tet,
  const TetStiffness& stiffness,
  const glm::mat3& rotation,
  const std::array<glm::vec3, 4>& force
);

[[nodiscard]] InitialStress initialStress(
  const std::array<glm::vec3, 8>& deformed,
  const std::array<glm::vec3, 8>& rest,
  const objectmodel::PreparedHex& hex,
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
