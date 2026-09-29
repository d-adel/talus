#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <talus/deformable/impactwindow/impactwindow.h>
#include <talus/deformable/solver/solver.h>
#include <talus/objectmodel/component/renderpose/renderpose.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/impact/impact.h>
#include <talus/physics/physicsworld/physicsworld.h>
#include <talus/physics/joltlibrary/joltlibrary.h>

#include <atomic>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace talus::runtime {

inline constexpr float QUIESCENT_CORRECTION = 4.0e-4f;

struct YieldTransition {
  objectmodel::Entity entity = objectmodel::INVALID_ENTITY;
  bool promotion = false;
  glm::dvec3 linearBefore{0.0};
  glm::dvec3 linearAfter{0.0};
  glm::dvec3 angularBefore{0.0};
  glm::dvec3 angularAfter{0.0};
  double kineticBefore = 0.0;
  double kineticAfter = 0.0;
  double penaltyAfter = 0.0;
  float approach = 0.0f;
  float permanent = 0.0f;
};

struct PartnerHandback {
  uint32_t body = 0u;
  glm::dvec3 centre{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
  glm::dvec3 contactLinear{0.0};
  glm::dvec3 contactAngular{0.0};
  bool owned = false;
  double windowEnergy = 0.0;
  double forwardEnergy = 0.0;
  double groupInput = 0.0;
};

struct WindowRecord {
  objectmodel::Entity entity = objectmodel::INVALID_ENTITY;
  uint32_t frame = 0u;
  uint32_t group = 0u;
  deformable::WindowStep step;
  int64_t firstContact = -1;
  double lead = -1.0;
  uint32_t openingInside = 0u;
  bool closing = false;
  bool diverged = false;
  bool restored = false;
  std::vector<PartnerHandback> handbacks;
};

struct DeformableActivity {
  uint32_t armAttempts = 0u;
  uint32_t armAborts = 0u;
  uint32_t editDisarms = 0u;
  uint32_t confirms = 0u;
  uint32_t refusals = 0u;
  uint32_t freezes = 0u;
  uint32_t wakes = 0u;
  uint32_t editWakes = 0u;
  uint32_t moverWakes = 0u;
  uint32_t destroyWakes = 0u;
  uint32_t activationWakes = 0u;
  uint32_t globalWakes = 0u;
  uint32_t otherWakes = 0u;
  uint32_t confirmSteps = 0u;
  float worstCorrection = 0.0f;
  glm::vec3 confirmLoad{0.0f};
  deformable::EquilibriumCheck lastConfirm{};
  uint32_t promotions = 0u;
  uint32_t demotions = 0u;
  uint32_t hullFallbacks = 0u;
  std::vector<YieldTransition> transitions;
  uint32_t windows = 0u;
  uint32_t windowGroups = 0u;
  uint32_t windowTouches = 0u;
  double windowTouchDepth = 0.0;
  uint64_t windowSubsteps = 0u;
  uint32_t degeneratePartners = 0u;
  uint32_t windowRestores = 0u;
  std::vector<WindowRecord> windowFrames;
};

class Runtime {
public:
  Runtime(const objectmodel::World& source, physics::JoltLibrary&);

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;
  Runtime(Runtime&&) = delete;
  Runtime& operator=(Runtime&&) = delete;

  objectmodel::World& world() noexcept;
  const objectmodel::World& world() const noexcept;

  physics::PhysicsWorld& physics() noexcept;

  void refreshBody(objectmodel::Entity entity);

  void refreshBodies(std::span<const objectmodel::Entity> entities);

  void optimizeBroadPhase();

  void setDeformableSettings(const deformable::Settings& settings) noexcept;

  [[nodiscard]] const deformable::Settings& deformableSettings() const noexcept;

  [[nodiscard]] const deformable::StepReport& deformableReport() const noexcept;

  [[nodiscard]] const DeformableActivity& deformableActivity() const noexcept;

  [[nodiscard]] bool deformableQuiescent(objectmodel::Entity entity) const;

  [[nodiscard]] bool armedDeformable(objectmodel::Entity entity) const;

  [[nodiscard]] bool rigidUntilYield(objectmodel::Entity entity) const;

  [[nodiscard]] bool promotedDeformable(objectmodel::Entity entity) const;

  bool forcePromotion(objectmodel::Entity entity);

  [[nodiscard]] const deformable::ImpactWindow* impactWindow(
    objectmodel::Entity entity
  ) const;

  [[nodiscard]] std::span<const glm::vec3> storedShape(
    objectmodel::Entity entity
  ) const;

  [[nodiscard]] const deformable::DeformableSolverState* deformableSolver(
    objectmodel::Entity entity
  ) const;

  [[nodiscard]] bool prepareDeformable(objectmodel::Entity entity);

  void reshapeAfterDeform(
    objectmodel::Entity entity,
    const glm::uvec3& brickBegin,
    const glm::uvec3& brickEnd
  );

  void wake(const glm::vec3& minimum, const glm::vec3& maximum);

  void collectImpacts();

  [[nodiscard]] std::span<const objectmodel::Impact> impacts() const noexcept;

  void beginFrame() noexcept;

  void refreshDeformedBodies();

  void setBodyShape(objectmodel::Entity entity, const JPH::ShapeRefC& shape);

  void setLinearVelocity(objectmodel::Entity entity, const glm::vec3& velocity);

  bool destroy(objectmodel::Entity entity);

  void setMass(objectmodel::Entity entity, float mass);

  [[nodiscard]] size_t shapeCount() const noexcept;

  void applyPoseEdits();

  void step();

  void syncPoses(float alpha);

private:
  std::vector<objectmodel::Impact> m_impacts;

  void applyMassScale(objectmodel::Entity entity);

  void refreshInto(
    objectmodel::Entity entity,
    std::vector<JPH::BodyID>& created
  );

  void placeBody(
    objectmodel::Entity entity,
    const JPH::ShapeRefC& shape,
    std::vector<JPH::BodyID>& created
  );

  void addBodies(std::vector<JPH::BodyID>& created);

  struct SharedShape {
    std::weak_ptr<talus::field::SampledSdf> lattice;
    JPH::ShapeRefC shape;
  };

  [[nodiscard]] JPH::ShapeRefC shapeFor(const objectmodel::SharedSdf& held);

  void rememberShape(
    const objectmodel::SharedSdf& held,
    const JPH::ShapeRefC& shape
  );

  void stepDeformables();

  struct DeformableSlot {
    std::vector<JPH::BodyID> touching;
    std::vector<deformable::SurfaceContact> contacts;
    std::vector<deformable::RigidPartner> partners;
    std::vector<deformable::ShapeProbe> probes;
    std::vector<uint32_t> unsupported;
    std::vector<std::pair<uint32_t, bool>> wakes;
    deformable::StepReport report;
    uint32_t armAttempts = 0u;
    uint32_t armAborts = 0u;
    bool finite = true;
    std::exception_ptr failure;
  };

  void touchBodies(objectmodel::Entity entity, float pad, DeformableSlot& slot);

  void gatherContacts(
    objectmodel::Entity entity,
    const glm::vec3& gravity,
    std::span<const uint32_t> woken,
    DeformableSlot& slot
  );

  struct YieldTrack {
    std::vector<glm::vec3> shape;
    std::optional<deformable::ImpactWindow> window;
    float trigger = 0.0f;
    bool promoted = false;
    bool forced = false;
  };

  void applyYieldMotion(
    uint32_t body,
    const objectmodel::PreparedMesh& mesh,
    const YieldTrack& track
  );

  struct YieldFlight {
    uint32_t body = 0u;
    glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
    glm::vec3 spin{0.0f};
    glm::dmat3 inverseInertia{0.0};
  };

  void recordYieldFlights();

  void advanceYieldFlights();

  [[nodiscard]] bool yieldReachable(
    uint32_t body,
    const YieldTrack& track,
    const glm::vec3& gravity
  ) const;

  [[nodiscard]] objectmodel::DeformableState rigidState(
    objectmodel::Entity entity,
    uint32_t body,
    const objectmodel::PreparedMesh& mesh,
    const YieldTrack& track
  ) const;

  [[nodiscard]] float yieldApproach(
    objectmodel::Entity entity,
    const glm::vec3& gravity
  );

  void promote(objectmodel::Entity entity, uint32_t body, float approach);

  void demote(objectmodel::Entity entity, uint32_t body, YieldTrack& track);

  void stepYielding();

  struct WindowSlot {
    objectmodel::Entity entity = objectmodel::INVALID_ENTITY;
    const objectmodel::PreparedMesh* mesh = nullptr;
    YieldTrack* track = nullptr;
    glm::vec3 minimum{0.0f};
    glm::vec3 maximum{0.0f};
    std::vector<deformable::WindowPartner> partners;
  };

  void gatherWindowPartners(WindowSlot& slot);

  void gatherPartnerSupports(deformable::WindowGroup& group);

  void countWindowTouches(std::span<const WindowSlot> slots);

  void stepWindows();

  struct HandbackPrediction {
    size_t record = 0u;
    size_t handback = 0u;
    uint32_t body = 0u;
    glm::vec3 linear{0.0f};
    glm::vec3 angular{0.0f};
  };

  struct PartnerCarry {
    deformable::PartnerLoad load;
    deformable::PartnerLoad drag;
    std::vector<deformable::WindowContactTrack> supports;
  };

  void finishHandbacks();

  void reportUnsupported(objectmodel::Entity entity, const JPH::BodyID& id);

  enum class DeformableMode : uint8_t {
    Active,
    Arming,
    Quiescent
  };

  struct DeformableArming {
    std::vector<glm::vec3> anchor;
    std::vector<glm::vec3> load;
    std::vector<uint32_t> partners;
    uint32_t loadSteps = 0u;
    uint32_t quiet = 0u;
    uint32_t holdoff = 0u;
    uint32_t consecutiveAborts = 0u;
  };

  struct Bounds {
    glm::vec3 minimum{0.0f};
    glm::vec3 maximum{0.0f};
  };

  struct DeformableTrack {
    deformable::DeformableSolverState solver;
    DeformableArming arming;
    DeformableMode mode = DeformableMode::Active;
    Bounds region;
    std::vector<uint32_t> island;
  };

  struct SleepGate {
    float allowance = 0.0f;
    uint32_t window = 1u;
  };

  enum class WakeReason : uint8_t {
    Edit,
    Mover,
    Destroy,
    Activation,
    Global,
    Other
  };

  struct BodyMark {
    uint32_t body = 0u;
    WakeReason reason = WakeReason::Edit;
    std::optional<Bounds> before;
  };

  enum class IslandScan : uint8_t {
    Complete,
    Awake,
    Edited
  };

  [[nodiscard]] SleepGate sleepGate() const;

  [[nodiscard]] glm::vec3 gravity() const;

  void refreshStepping();

  void markBody(
    uint32_t body,
    WakeReason reason,
    const std::optional<Bounds>& before
  );

  [[nodiscard]] std::optional<Bounds> bodyBounds(uint32_t body) const;

  void countWake(WakeReason reason) noexcept;

  [[nodiscard]] IslandScan scanIsland(
    const objectmodel::PreparedMesh& mesh,
    std::span<const uint32_t> partners,
    bool stopAtAwake,
    std::span<const BodyMark> marks
  );

  void holdIsland(objectmodel::Entity entity, std::span<const uint32_t> island);

  void releaseIsland(objectmodel::Entity entity, DeformableTrack& track);

  void disarmEdited(bool global);

  void gatherMovers(const glm::vec3& gravity);

  [[nodiscard]] bool moverReaches(const Bounds& region) const;

  [[nodiscard]] bool partnersAsleep(std::span<const uint32_t> partners) const;

  [[nodiscard]] Bounds wakeRegion(
    const objectmodel::PreparedMesh& mesh,
    const objectmodel::DeformableState& state,
    std::span<const uint32_t> partners
  ) const;

  static void disarm(
    DeformableTrack& track,
    std::span<const glm::vec3> position,
    uint32_t holdoff,
    uint32_t& aborts
  );

  void groupDeformables(size_t threads);

  void stepGroup(
    size_t group,
    const glm::vec3& pull,
    const SleepGate& gate,
    std::vector<uint32_t>& woken,
    std::atomic<uint32_t>& limit
  );

  void stepDeformable(
    objectmodel::Entity entity,
    const glm::vec3& pull,
    const SleepGate& gate,
    std::span<const uint32_t> woken,
    DeformableSlot& slot
  );

  [[nodiscard]] bool exchangeDeformable(
    objectmodel::Entity entity,
    DeformableSlot& slot,
    std::vector<uint32_t>& woken
  );

  [[nodiscard]] bool applyDeformable(
    objectmodel::Entity entity,
    DeformableSlot& slot
  );

  void activatePartners(const DeformableSlot& slot);

  [[nodiscard]] bool wakeIsland(objectmodel::Entity entity);

  void wakeDeformables();

  void sleepDeformables();

  objectmodel::World m_world;
  physics::PhysicsWorld m_physicsWorld;

  deformable::Settings m_deformableSettings;
  std::unordered_map<objectmodel::Entity, DeformableTrack> m_deformableSolvers;
  deformable::StepReport m_deformableReport;
  DeformableActivity m_deformableActivity;

  std::vector<objectmodel::Entity> m_stepping;
  std::vector<objectmodel::Entity> m_quiescent;
  std::vector<Bounds> m_movers;
  float m_moverReach = 0.0f;

  std::unordered_map<objectmodel::Entity, YieldTrack> m_yielding;

  std::vector<YieldFlight> m_flights;

  std::vector<HandbackPrediction> m_handbackPredictions;
  std::unordered_map<uint32_t, PartnerCarry> m_partnerLoads;

  std::vector<BodyMark> m_marks;
  std::vector<uint32_t> m_activated;
  std::vector<std::pair<objectmodel::Entity, WakeReason>> m_waking;
  std::unordered_map<uint32_t, std::vector<objectmodel::Entity>> m_dependents;
  std::unordered_set<uint32_t> m_islandMembers;
  std::vector<uint32_t> m_islandPending;
  glm::vec3 m_loadGravity{0.0f};

  bool m_steppingStale = true;

  uint32_t m_pendingDestroyWakes = 0u;
  uint32_t m_pendingOtherWakes = 0u;

  std::vector<DeformableSlot> m_deformableSlots;
  std::vector<objectmodel::Entity> m_deformableRun;
  std::vector<uint32_t> m_groupParents;
  std::vector<uint32_t> m_groupMembers;
  std::vector<uint32_t> m_groupStarts;
  std::unordered_map<uint32_t, uint32_t> m_groupOwners;

  uint32_t m_unsupportedPartners = 0u;

  std::unordered_set<uint64_t> m_unsupportedSeen;

  std::vector<std::pair<objectmodel::Entity, objectmodel::Pose>> m_poseScratch;
  std::vector<std::pair<objectmodel::Entity, objectmodel::RenderPose>>
    m_renderScratch;

  std::unordered_map<uint32_t, objectmodel::Pose> m_previousPoses;

  std::unordered_map<const talus::field::SampledSdf*, SharedShape> m_shapes;

  bool m_masslessReported = false;

  bool m_broadPhaseStale = false;

  uint32_t m_addedSinceOptimize = 0u;

  uint32_t m_optimizeAfter = 0u;
};

}
