#pragma once

#include <talus/mesh.h>
#include <talus/deformable/material/material.h>

#include <talus/material.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <array>
#include <span>
#include <unordered_map>
#include <vector>
#include <vector>

namespace talus::deformable {

struct FactorReference {
  std::vector<glm::vec3> positions;
  glm::vec3 centroid{0.0f};
  float step = 0.0f;
  float inertia = 1.0f;
  std::vector<float> damping;
};

struct SolverStats {
  uint32_t lastIterations = 0u;
  float lastResidual = 0.0f;
  uint32_t factorAge = 0u;
  uint32_t refactorCount = 0u;
  uint32_t retryCount = 0u;
  uint32_t substepsSinceRefresh = 0u;
  float meanIterations = 0.0f;
  bool refreshRequested = false;
  bool freshRetryFailed = false;
};

struct FactorTiming {
  double packSeconds = 0.0;
  double solveSeconds = 0.0;
  double unpackSeconds = 0.0;
  uint32_t applications = 0u;
};

class SystemFactor {
public:
  SystemFactor();
  ~SystemFactor();

  SystemFactor(SystemFactor&&) noexcept;
  SystemFactor& operator=(SystemFactor&&) noexcept;

  SystemFactor(const SystemFactor&) = delete;
  SystemFactor& operator=(const SystemFactor&) = delete;

  [[nodiscard]] bool analyzed() const noexcept;
  [[nodiscard]] bool empty() const noexcept;
  [[nodiscard]] uint32_t nodes() const noexcept;
  [[nodiscard]] uint64_t stored() const noexcept;
  [[nodiscard]] uint64_t generation() const noexcept;
  [[nodiscard]] const FactorReference& reference() const noexcept;
  [[nodiscard]] uint32_t rows() const noexcept;
  [[nodiscard]] uint64_t systemNonZeros() const noexcept;
  [[nodiscard]] uint64_t factorNonZeros() const noexcept;
  [[nodiscard]] double entry(uint32_t row, uint32_t column) const;

  struct Held;

private:
  friend void analyzeSystem(SystemFactor&, const talus::PreparedMesh&);
  friend void refreshFactor(
    SystemFactor&,
    const talus::PreparedMesh&,
    const talus::DeformableState&,
    std::span<const talus::PhysicalMaterial>,
    float,
    float,
    float
  );
  friend void applyFactor(
    const SystemFactor&,
    const glm::mat3&,
    const talus::PreparedMesh&,
    std::vector<glm::vec3>&,
    FactorTiming*
  );
  friend void applyFactorBatch(
    const SystemFactor&,
    const glm::mat3&,
    const talus::PreparedMesh&,
    std::span<std::vector<glm::vec3>>
  );

  std::unique_ptr<Held> m_held;
};

void analyzeSystem(
  SystemFactor& factor,
  const talus::PreparedMesh& mesh
);

void refreshFactor(
  SystemFactor& factor,
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  float step,
  float inertia = 1.0f,
  float damping = 0.0f
);

[[nodiscard]] glm::mat3 bodyRotation(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  const FactorReference& reference
);

void applyFactor(
  const SystemFactor& factor,
  const glm::mat3& frame,
  const talus::PreparedMesh& mesh,
  std::vector<glm::vec3>& vector,
  FactorTiming* timing = nullptr
);

void applyFactorBatch(
  const SystemFactor& factor,
  const glm::mat3& frame,
  const talus::PreparedMesh& mesh,
  std::span<std::vector<glm::vec3>> vectors
);

struct RigidFit {
  std::array<glm::vec3, 8> arms{};
  glm::mat3 inverse{0.0f};
};

[[nodiscard]] float stiffnessDamping(
  const talus::PhysicalMaterial& material
) noexcept;

[[nodiscard]] std::vector<float> laneDamping(
  std::span<const talus::PhysicalMaterial> materials,
  float scale
);

[[nodiscard]] RigidFit rigidFit(std::span<const glm::vec3> corners);

void removeRigid(const RigidFit& fit, std::span<glm::vec3> values);

void applyDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const TetStiffness& stiffness,
  const std::array<glm::vec3, 4>& direction,
  std::array<glm::vec3, 4>& result
);

void applyDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  const std::array<glm::vec3, 8>& direction,
  std::array<glm::vec3, 8>& result
);

void applyStiffnessDamping(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness,
  const std::array<glm::vec3, 8>& direction,
  std::array<glm::vec3, 8>& stiff,
  std::array<glm::vec3, 8>& damped
);

[[nodiscard]] std::array<glm::mat3, 16> dampingBlocks(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const TetStiffness& stiffness
);

[[nodiscard]] std::array<glm::mat3, 64> dampingBlocks(
  const RigidFit& fit,
  const glm::mat3& rotation,
  const HexStiffness& stiffness
);

struct ResponseBasis {
  std::unordered_map<uint32_t, std::array<std::vector<glm::vec3>, 3>> column;
  uint64_t generation = 0u;
};

struct ElementStiffness {
  std::vector<TetStiffness> tet;
  std::vector<HexStiffness> hex;
  std::vector<glm::vec3> elastic;
  const void* mesh = nullptr;
  size_t tets = 0u;
  size_t hexes = 0u;
  size_t nodes = 0u;
};

struct DeformableSolverState {
  SystemFactor factor;
  SolverStats stats;
  ResponseBasis basis;
  ElementStiffness stiffness;
};

}
