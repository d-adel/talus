#pragma once

#include <talus/deformable/contact/contact.h>
#include <talus/deformable/factor/factor.h>
#include <talus/deformable/material/material.h>

#include <talus/field/sampledsdf/sampledsdf.h>

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <vector>

namespace talus::deformable {

using VolumeElementType = objectmodel::VolumeElementType;

inline constexpr float EQUILIBRIUM_MOTION = 1.0e-5f;

enum class RigidSplit : uint8_t {
  Off,
  Unprojected,
  Projected
};

enum class ContactOperator : uint8_t {
  Dense,
  Unbuilt
};

struct Settings {
  uint32_t substeps = 1u;
  uint32_t cgIterations = 64u;
  uint32_t factorBudget = 32u;
  uint32_t contactIterations = 256u;
  uint32_t plasticPasses = 40u;
  float contactTolerance = 1.0e-3f;
  bool reportContactSystem = false;
  bool reportContactDiagnostics = false;
  bool reportSystem = false;
  ContactOperator contactOperator = ContactOperator::Unbuilt;
  VolumeElementType element = VolumeElementType::Tet;
  float tolerance = 1.0e-3f;
  float spectralRadius = 0.5f;
  RigidSplit rigidSplit = RigidSplit::Projected;
  bool rigidUntilYield = true;
  bool demoteAfterYield = true;
  double windowStepShare = 0.45;
  double windowPenaltyScale = 16.0;
  bool windowRotationDerivative = true;
  bool windowContactTransitions = true;
  uint32_t stepThreads = 0u;
};

inline constexpr uint32_t RIGID_PARTS = 8u;

struct RigidMotion {
  glm::dvec3 momentum{0.0};
  glm::dmat3 turn{1.0};
};

[[nodiscard]] RigidMotion rigidRate(
  const glm::dmat3& inverse,
  const RigidMotion& at
);

[[nodiscard]] RigidMotion rigidAdvance(
  const glm::dmat3& inverse,
  RigidMotion motion,
  double span,
  uint32_t count
);

struct RigidPartner {
  uint32_t body = 0u;
  glm::vec3 centerOfMass{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 linearVelocity{0.0f};
  glm::vec3 angularVelocity{0.0f};
  glm::vec3 originalLinearVelocity{0.0f};
  glm::vec3 originalAngularVelocity{0.0f};
  glm::mat3 inverseInertia{0.0f};
  float inverseMass = 0.0f;
  float friction = 0.5f;
  float restitution = 0.0f;
  bool dynamic = false;
  glm::vec3 impulse{0.0f};
  glm::vec3 nodeImpulse{0.0f};
};

struct RigidExchange {
  uint32_t body = 0u;
  glm::vec3 impulse{0.0f};
  glm::vec3 nodeImpulse{0.0f};
};

struct ContactCheck {
  float normalImpulse = 0.0f;
  float tangentImpulse = 0.0f;
  float friction = 0.0f;
  float gap = 0.0f;
  float advanceNormal = 0.0f;
  float advanceTangent = 0.0f;
  float freeAdvanceTangent = 0.0f;
  float freeVelocityTangent = 0.0f;
  float velocityNormal = 0.0f;
  float velocityTangent = 0.0f;
  float advanceOpposition = 0.0f;
  float velocityOpposition = 0.0f;
  float frictionWork = 0.0f;
  float area = 0.0f;
  float tangentCompliance = 0.0f;
  glm::vec2 tangentAdvance{0.0f};
  glm::vec2 solverTangentSlip{0.0f};
  float residualScale = 0.0f;
  float residualNorm = 0.0f;
  uint8_t dynamic = 0u;
};

struct ContactFrame {
  glm::uvec3 nodes{0u};
  glm::vec3 weights{0.0f};
  glm::vec3 normal{0.0f};
  glm::vec3 tangent{0.0f};
  glm::vec3 bitangent{0.0f};
  float gap = 0.0f;
  float friction = 0.5f;
  uint8_t dynamic = 0u;
};

struct ContactSample {
  uint32_t triangle = 0u;
  uint32_t partner = 0u;
  glm::uvec3 nodes{0u};
  glm::vec3 point{0.0f};
};

struct ContactSolve {
  uint32_t deformable = 0u;
  uint32_t contacts = 0u;
  uint32_t rows = 0u;
  uint32_t uniqueNodes = 0u;
  uint32_t cacheColumnsBefore = 0u;
  uint32_t cacheColumnsAfter = 0u;
  uint32_t cacheMisses = 0u;
  uint32_t newBasisSolves = 0u;
  uint32_t pgsPasses = 0u;
  double responseCacheSeconds = 0.0;
  double delassusSeconds = 0.0;
  double pgsSeconds = 0.0;
  double responseApplySeconds = 0.0;
  double totalContactSeconds = 0.0;
  std::vector<ContactSample> samples;
  std::vector<ContactCheck> checks;
};

struct StepReport {
  uint32_t nodes = 0u;
  uint32_t elements = 0u;
  uint32_t cgIterations = 0u;
  uint32_t cgCappedSolves = 0u;
  uint32_t plasticEvaluations = 0u;
  uint32_t plasticSolves = 0u;
  uint32_t plasticBacktracks = 0u;
  uint32_t plasticCapped = 0u;
  uint32_t plasticLinearReads = 0u;
  uint32_t plasticHandoffs = 0u;
  uint32_t plasticFallbacks = 0u;
  uint32_t plasticFallbackRejections = 0u;
  uint32_t primaryCgOperatorApplications = 0u;
  uint32_t contacts = 0u;
  float cgWorstRelativeResidual = 0.0f;
  float plasticWorstResidual = 0.0f;
  float plasticWorstRelativeResidual = 0.0f;
  double setupSeconds = 0.0;
  double setupScratchSeconds = 0.0;
  double setupStiffnessSeconds = 0.0;
  double setupIncidenceSeconds = 0.0;
  double setupAllocateSeconds = 0.0;
  double elasticSeconds = 0.0;
  double primaryCgSeconds = 0.0;
  double primaryCgMassSeconds = 0.0;
  double primaryCgTetSeconds = 0.0;
  double primaryCgAnchorSeconds = 0.0;
  double primaryCgConditionSeconds = 0.0;
  double factorPackSeconds = 0.0;
  double factorSolveSeconds = 0.0;
  double factorUnpackSeconds = 0.0;
  uint32_t factorApplications = 0u;
  uint32_t factorRows = 0u;
  uint64_t factorNonZeros = 0u;
  uint64_t systemNonZeros = 0u;
  double basisSeconds = 0.0;
  double responseCacheSeconds = 0.0;
  double delassusSeconds = 0.0;
  double contactSeconds = 0.0;
  double contactSweepSeconds = 0.0;
  double velocityContactSweepSeconds = 0.0;
  uint32_t contactSweeps = 0u;
  uint32_t contactSweepNodes = 0u;
  double responseApplySeconds = 0.0;
  uint32_t contactRows = 0u;
  float maxPenetration = 0.0f;
  float contactWorstRelativeChange = 0.0f;
  uint32_t contactPasses = 0u;
  uint32_t contactTotalPasses = 0u;
  uint32_t contactSettlePass = 0u;
  uint32_t contactTightPass = 0u;
  float contactResidual = 0.0f;
  uint32_t velocityContactPasses = 0u;
  uint32_t velocityContactTotalPasses = 0u;
  float velocityContactResidual = 0.0f;
  uint32_t velocityContactCapped = 0u;
  double velocityResponseApplySeconds = 0.0;
  uint32_t posedSubsteps = 0u;
  uint32_t posedSolves = 0u;
  uint32_t posedCapped = 0u;
  double posedKineticChange = 0.0;
  uint32_t basisSolves = 0u;
  float elasticEnergy = 0.0f;
  float kineticEnergy = 0.0f;
  double cpuSeconds = 0.0;
  uint32_t unsupportedPartners = 0u;
  bool nonFinite = false;
  bool plasticGrowth = false;
  uint32_t refactors = 0u;
  uint32_t stiffnessRebuilds = 0u;
  uint32_t retries = 0u;
  uint32_t refreshRequests = 0u;
  uint32_t retriesFailed = 0u;
  uint32_t refreshedAtAge = 0u;
  uint32_t wastedIterations = 0u;
  uint32_t worstSubstep = 0u;
  std::vector<uint32_t> cgPerSubstep;
  std::vector<float> cgStartResidual;
  std::vector<float> cgEndResidual;
  uint32_t spentBeforeRefresh = 0u;
  uint32_t spentAfterRefresh = 0u;
  double refactorSeconds = 0.0;
  std::vector<glm::vec3> contactImpulses;
  std::vector<glm::vec3> contactLoad;
  std::vector<glm::vec3> contactWarmStart;
  float contactResponseResidual = 0.0f;
  std::vector<float> contactOperator;
  std::vector<float> contactRight;
  std::vector<float> contactFriction;
  std::vector<ContactFrame> contactFrames;
  std::vector<ContactCheck> contactChecks;
  std::vector<ContactSolve> contactSolves;
  std::vector<RigidExchange> exchange;
  std::vector<glm::vec3> systemPositions;
  std::vector<glm::mat3> systemRotations;
  std::vector<float> systemOperator;
  std::vector<float> systemDamping;
  std::vector<glm::mat3> systemDiagonal;
  std::vector<float> systemSweep;
};

struct SupportVolume {
  glm::mat4 worldToSupport{1.0f};
  glm::vec3 halfExtents{0.0f};
};

[[nodiscard]] std::shared_ptr<const objectmodel::PreparedMesh> buildMesh(
  const talus::field::SampledSdf& lattice,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const glm::mat4& localToWorld,
  std::span<const SupportVolume> supports,
  VolumeElementType element
);

[[nodiscard]] objectmodel::DeformableState restState(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::Pose& pose
);

struct RigidMass {
  double mass = 0.0;
  glm::dvec3 centre{0.0};
  glm::dmat3 inertia{0.0};
};

template<typename Mass, typename Point>
[[nodiscard]] RigidMass rigidMass(
  std::span<const Mass> mass,
  std::span<const Point> position
) {
  RigidMass held;

  for (size_t node = 0u; node != mass.size(); ++node) {
    const double weight = static_cast<double>(mass[node]);

    held.mass += weight;
    held.centre += weight * glm::dvec3(position[node]);
  }

  if (!(held.mass > 0.0)) {
    return held;
  }

  held.centre /= held.mass;

  for (size_t node = 0u; node != mass.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(position[node]) - held.centre;

    held.inertia +=
      static_cast<double>(mass[node])
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  return held;
}

struct MeshFrame {
  glm::dvec3 restCentre{0.0};
  glm::dvec3 centre{0.0};
  glm::dmat3 rotation{1.0};
};

[[nodiscard]] MeshFrame meshFrame(
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> position
);

inline constexpr float UNLOAD_SPEED = 1.0e-6f;

inline constexpr uint32_t UNLOAD_STEPS = 40u;

struct Unloading {
  objectmodel::DeformableState state;
  uint32_t steps = 0u;
  float speed = std::numeric_limits<float>::infinity();
};

[[nodiscard]] Unloading unloadedState(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& from,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const Settings& settings,
  float deltaTime
);

[[nodiscard]] objectmodel::DeformableState solveEquilibrium(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const objectmodel::Pose& pose,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load = {}
);

[[nodiscard]] objectmodel::DeformableState solveEquilibrium(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const objectmodel::DeformableState& start,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load = {}
);

inline constexpr float EQUILIBRIUM_BALANCE = 1.0e-2f;

inline constexpr double EQUILIBRIUM_SHIFT = 1.0e-6;

struct EquilibriumCheck {
  float maxCorrection = 0.0f;
  bool cgCapped = false;
  bool unbalanced = false;
  bool factored = false;
  uint32_t freeModes = 0u;
  uint32_t cgIterations = 0u;
  float forceBalance = 0.0f;
  float torqueBalance = 0.0f;
  float shift = 0.0f;
};

[[nodiscard]] EquilibriumCheck checkEquilibrium(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& current,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const Settings& settings,
  const glm::vec3& gravity,
  std::span<const glm::vec3> load = {}
);

void step(
  const objectmodel::PreparedMesh& mesh,
  objectmodel::DeformableState& state,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const Settings& settings,
  const glm::vec3& gravity,
  float deltaTime,
  std::span<const SurfaceContact> contacts,
  std::span<RigidPartner> partners,
  DeformableSolverState* solver,
  StepReport& report
);

}
