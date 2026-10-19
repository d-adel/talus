#pragma once

#include <talus/destruction/chunker/chunker.h>
#include <talus/destruction/stress/factor.h>
#include <talus/destruction/structure/structure.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace talus::destruction {

enum class MassWeighting : uint8_t { Real, Equal };

enum class LoadSharing : uint8_t { Equal, Area };

struct StressSettings {
  glm::dvec3 gravity{0.0, -9.81, 0.0};
  uint32_t crackRounds = 16u;
  uint32_t roundsPerCall = 0u;
  uint32_t wideChunks = 0u;
  dynamics::Workers workers;
  Lanes lanes = Lanes::Widest;
};

struct ChunkLoad {
  uint32_t chunk = 0u;
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
};

struct StressFactor;

struct StressState {
  std::vector<glm::dvec3> force;
  std::vector<glm::dvec3> moment;
  std::vector<double> inverseMass;
  std::vector<glm::dvec3> inverseInertia;
  std::vector<double> gravityScale;
  std::vector<double> forceShare;
  std::vector<double> momentShare;
  double length = 1.0;
  double area = 0.0;
  std::vector<ChunkLoad> loads;
  glm::dvec3 gravity{0.0};
  uint64_t revision = 0u;
  double utilization = 0.0;
  bool solved = false;
  bool eroding = false;
  bool pending = false;
  uint32_t rounds = 0u;
  std::shared_ptr<StressFactor> factor;
};

struct BondStress {
  double compression = 0.0;
  double tension = 0.0;
  double shear = 0.0;
  double pressure = 0.0;
  double rocking = 0.0;
};

struct BondAssessment {
  double damage = 0.0;
  bool cracks = false;
};

struct StressReport {
  bool ran = false;
  uint32_t rounds = 0u;
  uint32_t factorizations = 0u;
  uint32_t solvedChunks = 0u;
  uint32_t factorizedChunks = 0u;
  uint64_t factorWork = 0u;
  double factorizing = 0.0;
  bool pending = false;
  bool wide = false;
  std::vector<uint32_t> broken;
  std::vector<uint32_t> cracked;
};

[[nodiscard]] StressState makeStress(
  const ChunkedBuilding& building,
  MassWeighting weighting = MassWeighting::Real,
  LoadSharing sharing = LoadSharing::Area
);

[[nodiscard]] StressState makeStressLike(const ChunkedBuilding& building, const StressState& like);

void solveStress(
  const ChunkedBuilding& building,
  const StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
);

[[nodiscard]] uint64_t factorBytes(const StressState& stress);

[[nodiscard]] BondStress bondStress(
  const Bond& bond,
  double health,
  const glm::dvec3& force,
  const glm::dvec3& moment
);

[[nodiscard]] double damageOf(const Bond& bond, const BondStress& stress);

[[nodiscard]] BondAssessment assessBond(const Bond& bond, const BondStress& stress, bool cracked);

[[nodiscard]] StressReport updateStress(
  const ChunkedBuilding& building,
  StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
);

[[nodiscard]] double loadChange(
  std::span<const ChunkLoad> before,
  const glm::dvec3& beforeGravity,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
);

[[nodiscard]] double loadChange(const StressState& stress, std::span<const ChunkLoad> loads, const glm::dvec3& gravity);

[[nodiscard]] bool withinTier(double change, double utilization);

[[nodiscard]] uint64_t factorWorkBound(const StressState& stress, const StructureState& structure);

[[nodiscard]] bool tolerates(
  const StressState& stress,
  const StructureState& structure,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
);

}
