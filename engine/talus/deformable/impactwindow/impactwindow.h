#pragma once

#include <talus/deformable/contact/contact.h>
#include <talus/deformable/solver/solver.h>

#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

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
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  double frameTime,
  const Settings& settings
);

struct WindowOffset {
  uint32_t node = 0u;
  uint32_t partner = 0u;
  double offset = 0.0;
};

struct WindowContactTrack {
  uint32_t body = 0u;
  std::vector<double> depth;
  std::vector<glm::dvec3> push;
};

struct WindowLedger {
  double kinetic = 0.0;
  double gravity = 0.0;
  double elastic = 0.0;
  double plastic = 0.0;
  double penalty = 0.0;
  double friction = 0.0;
  double partners = 0.0;
  double injected = 0.0;
  double pending = 0.0;
  double total = 0.0;
};

struct ImpactWindow {
  glm::dvec3 origin{0.0};
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> velocity;
  std::vector<glm::dvec3> friction;
  std::vector<glm::dvec3> surface;
  std::vector<objectmodel::PlasticPoint> plastic;
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
};

[[nodiscard]] ImpactWindow openWindow(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& start,
  const glm::dvec3& centre
);

struct PartnerLoad {
  glm::dvec3 force{0.0};
  glm::dvec3 torque{0.0};
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
};

struct WindowMember {
  const objectmodel::PreparedMesh* mesh = nullptr;
  ImpactWindow* window = nullptr;
  std::vector<uint32_t> partners;
  float trigger = 0.0f;
  bool permanent = false;
  uint32_t key = 0u;
};

struct WindowGroup {
  std::vector<WindowMember> members;
  std::vector<WindowPartner> partners;
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
};

[[nodiscard]] std::vector<WindowStep> stepWindows(
  WindowGroup& group,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const glm::dvec3& gravity,
  double frameTime,
  const Settings& settings
);

struct WindowExit {
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> velocity;
  std::vector<objectmodel::PlasticPoint> plastic;
};

[[nodiscard]] WindowExit closeWindow(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const ImpactWindow& window,
  const Settings& settings
);

void editWindowVelocity(
  ImpactWindow& window,
  const objectmodel::PreparedMesh& mesh,
  const glm::dvec3& change
);

void mirrorWindow(
  const ImpactWindow& window,
  objectmodel::DeformableState& state
);

[[nodiscard]] double windowPlasticWork(
  const objectmodel::PreparedMesh& mesh,
  std::span<const objectmodel::PhysicalMaterial> materials,
  std::span<const objectmodel::PlasticPoint> plastic
);

}
