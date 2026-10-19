#pragma once

#include <talus/deformable/contact/contact.h>
#include <talus/deformable/solver/solver.h>

#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace talus::deformable {

struct WindowLaw {
  double wave = 0.0;
  double spacing = 0.0;
  double stepLimit = 0.0;
  uint32_t substeps = 1u;
  double step = 0.0;
  double referenceStep = 0.0;
  std::vector<double> penalty;
};

[[nodiscard]] WindowLaw windowLaw(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  double frameTime,
  const Settings& settings
);

inline constexpr uint32_t NO_WINDOW_BODY = 0xffffffffu;

struct WindowOffset {
  uint32_t node = 0u;
  uint32_t partner = 0u;
  double offset = 0.0;
  uint32_t supported = NO_WINDOW_BODY;
};

struct WindowContactTrack {
  uint32_t body = 0u;
  std::vector<double> depth;
  std::vector<glm::dvec3> push;
  std::vector<double> offset;
  std::vector<double> endDepth;
  std::vector<double> excess;
};

struct WindowLedger {
  double kinetic = 0.0;
  double gravity = 0.0;
  double elastic = 0.0;
  double plastic = 0.0;
  double hysteresis = 0.0;
  double penalty = 0.0;
  double friction = 0.0;
  double tangential = 0.0;
  double partners = 0.0;
  double injected = 0.0;
  double pending = 0.0;
  double total = 0.0;
  double transfer = 0.0;
};

inline constexpr uint64_t WINDOW_PARTNER_KEYS = 1ull << 32u;

[[nodiscard]] constexpr uint64_t windowPartnerKey(uint32_t entity) {
  return WINDOW_PARTNER_KEYS | entity;
}

inline constexpr uint64_t STORED_PARTNER_KEYS = 1ull << 33u;

[[nodiscard]] constexpr uint64_t storedPartnerKey(uint32_t entity) {
  return STORED_PARTNER_KEYS | entity;
}

inline constexpr uint32_t WINDOW_EDGE_POINTS = 16u;
inline constexpr double WINDOW_ONSET_STEPS = 4.0;
inline constexpr double WINDOW_ONSET_SPEED = 0.1635;
inline constexpr uint32_t EXIT_FRAMES = 2u;

struct WindowProbeTrack {
  uint64_t partner = 0u;
  uint32_t point = 0u;
  double depth = 0.0;
  double offset = 0.0;
  double stiffness = 0.0;
  double width = 0.0;
  double endDepth = 0.0;
  glm::dvec3 push{0.0};
  glm::dvec3 stretch{0.0};
  double tangentStiffness = 0.0;
  glm::dvec3 executed{0.0};
  uint32_t reversals = 0u;
  glm::dvec3 arm{0.0};
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> shape{};
  uint32_t first = 0u;
  uint32_t second = 0u;
  double along = 0.0;
  double approach = 0.0;
  bool capped = false;
  bool cappedBefore = false;
  bool reversed = false;
  uint32_t stuck = 0u;
  double excess = 0.0;
};

struct WindowPairStep {
  uint64_t partner = 0u;
  double stiffness = 0.0;
  double width = 0.0;
  double approach = 0.0;
  double deepest = 0.0;
  double load = 0.0;
  double loadMost = 0.0;
};

struct WindowReaction {
  uint64_t from = 0u;
  uint32_t node = 0u;
  glm::dvec3 force{0.0};
};

struct WindowPartnerLoad {
  uint32_t body = 0u;
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
};

struct ImpactWindow {
  glm::dvec3 origin{0.0};
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> velocity;
  std::vector<glm::dvec3> friction;
  std::vector<glm::dvec3> surface;
  std::vector<talus::PlasticPoint> plastic;
  std::vector<WindowOffset> offsets;
  std::vector<WindowContactTrack> contactTracks;
  bool started = false;
  double time = 0.0;
  uint32_t frames = 0u;
  int64_t substeps = 0;
  int64_t firstContact = -1;
  double lead = -1.0;
  uint32_t openingInside = 0u;
  double step = 0.0;
  bool closing = false;
  bool diverged = false;
  WindowLedger ledger;
  double openingTotal = 0.0;
  double openingPenalty = 0.0;
  double openingKinetic = 0.0;
  double openingElastic = 0.0;
  double openingGravity = 0.0;
  double partnerGravity = 0.0;
  double partnerEntry = 0.0;
  std::vector<uint32_t> partnersSeen;
  double minimumTotal = 0.0;
  bool contactSinceMinimum = false;
  double stepCreation = 0.0;
  bool stepCreationContact = false;
  double angularDrift = 0.0;
  std::vector<uint32_t> groupKeys;
  double groupMinimum = 0.0;
  double groupRise = 0.0;
  std::vector<WindowProbeTrack> probeTracks;
  std::vector<WindowReaction> reactions;
  std::vector<WindowReaction> kicks;
  double receivedWork = 0.0;
  double receivedPeak = 0.0;
  double memberWork = 0.0;
  double memberPeak = 0.0;
  double gradientBound = 0.0;
  std::vector<double> partnerEntries;
  uint32_t openingProbes = 0u;
  double openingOffset = 0.0;
  std::vector<WindowPartnerLoad> partnerLoads;
  double largestSlip = 0.0;
  double cappedSlip = 0.0;
  uint32_t cappedSubsteps = 0u;
  std::vector<uint64_t> tracedContacts;
  std::vector<uint64_t> probedKeys;
  uint32_t heldSubsteps = 0u;
  double heldFrameTime = 0.0;
};

[[nodiscard]] ImpactWindow openWindow(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& start,
  const glm::dvec3& centre
);

struct PartnerLoad {
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
};

struct WindowSupportPair {
  uint32_t partner = 0u;
  glm::dvec3 impulse{0.0};
  glm::dvec3 angularImpulse{0.0};
  glm::dvec3 partnerImpulse{0.0};
};

struct WindowPartner {
  ShapeProbe probe;
  uint32_t body = 0u;
  glm::dvec3 centerOfMass{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 linearVelocity{0.0};
  glm::dvec3 angularVelocity{0.0};
  double inverseMass = 0.0;
  glm::dmat3 inverseInertia{0.0};
  bool dynamic = false;
  std::vector<glm::vec3> seeds;
  bool touched = false;
  glm::dvec3 deltaLinear{0.0};
  glm::dvec3 deltaAngular{0.0};
  PartnerLoad previousLoad;
  PartnerLoad previousDrag;
  std::vector<WindowContactTrack> supportTracks;
  const talus::PreparedMesh* mesh = nullptr;
  std::vector<glm::dvec3> shape;
  uint64_t key = 0u;
  bool massive = false;
  std::vector<WindowSupportPair> supportPairs;
};

struct WindowMember {
  const talus::PreparedMesh* mesh = nullptr;
  ImpactWindow* window = nullptr;
  std::vector<uint32_t> partners;
  float trigger = 0.0f;
  bool permanent = false;
  uint32_t key = 0u;
  const talus::field::SampledSdf* field = nullptr;
  uint32_t body = NO_WINDOW_BODY;
};

struct WindowGroup {
  std::vector<WindowMember> members;
  std::vector<WindowPartner> partners;
  bool reflectStuckSlip = false;
  bool windowPassOncePerFrame = false;
  bool threadOrderedMembers = false;
  bool omitWindowReaction = false;
  bool oneWayProbing = false;
  bool zeroOnsetWidth = false;
  bool shrinkWithoutMargin = false;
  bool skipPushAbsorption = false;
  bool nodeOnlyProbing = false;
  bool proberMassStiffness = false;
  bool skipReturnMap = false;
  bool dissipationOnTotal = false;
  bool penaltyOnBoth = false;
  bool gridFromRest = false;
  bool clampBandWeights = false;
  bool lumpReaction = false;
  bool exitBlindToWindows = false;
  bool skipExcessExtension = false;
  bool skipRecentre = false;
  bool skipStepHold = false;
  bool skipShrink = false;
  bool skipPairRelease = false;
  bool skipUnfoundRelease = false;
  bool traceReleases = false;
  bool skipSupportOpening = false;
  bool doubleSubstepZeroBooking = false;
  bool supportOffsetOnFirstWindow = false;
  bool supportOnFirstWindow = false;
  bool recordSupportLedger = false;
  bool rectangleReleaseBooking = false;
  bool halfPlastic = false;
};

enum class WindowTraceKind : uint8_t {
  Static,
  Dynamic,
  Stored,
  StoredDynamic,
  Window,
  Support
};

inline constexpr uint32_t WINDOW_TRACE_KINDS = 6u;

struct WindowTraceContact {
  WindowTraceKind kind = WindowTraceKind::Static;
  uint64_t id = 0u;
  uint32_t points = 0u;
  double deepest = 0.0;
  double penalty = 0.0;
  double work = 0.0;
};

struct WindowTracePartner {
  WindowTraceKind kind = WindowTraceKind::Static;
  uint64_t id = 0u;
  uint32_t touchSubsteps = 0u;
  double deepest = 0.0;
};

struct WindowTraceJump {
  uint32_t substep = 0u;
  double rise = 0.0;
  double increase = 0.0;
  WindowLedger ledger;
  WindowLedger before;
  std::vector<WindowTraceContact> contacts;
  std::vector<WindowTracePartner> entered;
  std::vector<WindowTracePartner> left;
  uint32_t openings = 0u;
  uint32_t dropped = 0u;
  uint32_t transitions = 0u;
  uint32_t supportTransitions = 0u;
  double correction = 0.0;
  bool groupChange = false;
};

struct WindowTrace {
  bool groupChange = false;
  double creationStart = 0.0;
  double creep = 0.0;
  std::vector<WindowTracePartner> partners;
  std::vector<WindowTraceJump> jumps;
};

struct WindowReleaseEvent {
  uint32_t substep = 0u;
  uint32_t point = 0u;
  uint32_t first = 0u;
  uint32_t second = 0u;
  double along = 0.0;
  uint64_t partner = 0u;
  bool current = false;
  bool probed = false;
  bool found = false;
  double distance = 0.0;
  double priorDepth = 0.0;
  double priorOffset = 0.0;
  double priorStiffness = 0.0;
  double priorWidth = 0.0;
  glm::dvec3 priorPush{0.0};
  double energy = 0.0;
  double closing = 0.0;
  double opening = 0.0;
  double reach = 0.0;
  double lambda = 0.0;
  glm::dvec3 kick{0.0};
  double booked = 0.0;
  double kineticChange = 0.0;
  glm::dvec3 momentumChange{0.0};
  glm::dvec3 angularChange{0.0};
  double pointGap = 0.0;
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> shape{};
  int32_t againSubstep = -1;
  double againDistance = 0.0;
  double againOffset = 0.0;
  double againForce = 0.0;
  double againStiffness = 0.0;
  double againWidth = 0.0;
};

struct WindowStep {
  uint32_t substeps = 0u;
  uint32_t contactSubsteps = 0u;
  uint32_t dynamicPartners = 0u;
  double plasticAdded = 0.0;
  WindowLedger ledger;
  double stepCreation = 0.0;
  bool stepCreationContact = false;
  double creation = 0.0;
  bool restored = false;
  double groupRise = 0.0;
  double input = 0.0;
  double angularDrift = 0.0;
  glm::dvec3 angularStart{0.0};
  glm::dvec3 angularEnd{0.0};
  uint32_t windowPartners = 0u;
  uint32_t probeContacts = 0u;
  double probeStiffness = 0.0;
  double transferInput = 0.0;
  double repeatedEntry = 0.0;
  double gradientBound = 0.0;
  double stepScale = 1.0;
  uint32_t unconverged = 0u;
  uint32_t inverted = 0u;
  uint32_t engagedUnconverged = 0u;
  double tangentialStiffness = 0.0;
  uint32_t tangentialChecks = 0u;
  double dissipationLeast = 0.0;
  double capRatio = 0.0;
  uint32_t openingProbes = 0u;
  double openingOffset = 0.0;
  uint32_t pairLeaves = 0u;
  uint32_t pairUnfound = 0u;
  uint32_t pairReleases = 0u;
  double pairReleaseWork = 0.0;
  std::vector<WindowReleaseEvent> releaseEvents;
  glm::dvec3 pairReleaseImpulse{0.0};
  std::vector<uint64_t> touchedKeys;
  double exitApproach = 0.0;
  uint32_t exitProbes = 0u;
  double partnerChange = 0.0;
  double partnerWork = 0.0;
  uint32_t stuckSubsteps = 0u;
  uint32_t reversals = 0u;
  double largestSlip = 0.0;
  uint32_t engagedNotFound = 0u;
  std::vector<WindowPairStep> pairs;
  double cappedSlip = 0.0;
  uint32_t cappedSubsteps = 0u;
  std::vector<uint64_t> storedKeys;
  uint32_t shrinkSubsteps = 0u;
  double excessAbsorbed = 0.0;
  double correctionWork = 0.0;
  uint32_t excessTracks = 0u;
  WindowTrace trace;
  uint32_t stepChanges = 0u;
  double stepChangeEnergy = 0.0;
  uint32_t heldSubsteps = 0u;
  double supportBaseline = 0.0;
  double supportPenalty = 0.0;
  double supportCreation = 0.0;
  double supportRise = 0.0;
  double supportReceived = 0.0;
  double supportWork = 0.0;
  std::vector<std::array<double, 8>> supportSamples;
};

struct ProbePoint {
  uint32_t first = 0u;
  uint32_t second = 0u;
  double along = 0.0;
  double length = 1.0;
};

[[nodiscard]] std::vector<ProbePoint> probePoints(
  const talus::PreparedMesh& mesh,
  bool nodesOnly
);

[[nodiscard]] double supportStiffness(
  double mass,
  double pace,
  const Settings& settings
);

void endSupportDepths(WindowGroup& group);

[[nodiscard]] std::vector<WindowStep> stepWindows(
  WindowGroup& group,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& gravity,
  double frameTime,
  const Settings& settings
);

struct WindowExit {
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> velocity;
  std::vector<talus::PlasticPoint> plastic;
};

[[nodiscard]] WindowExit closeWindow(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const ImpactWindow& window,
  const Settings& settings
);

void editWindowVelocity(
  ImpactWindow& window,
  const talus::PreparedMesh& mesh,
  const glm::dvec3& change
);

void mirrorWindow(
  const ImpactWindow& window,
  talus::DeformableState& state
);

[[nodiscard]] double windowPlasticWork(
  const talus::PreparedMesh& mesh,
  std::span<const talus::PhysicalMaterial> materials,
  std::span<const talus::PlasticPoint> plastic
);

}
