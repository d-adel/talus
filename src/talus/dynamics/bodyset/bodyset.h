#pragma once

#include <talus/deformable/contact/contact.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/neohookean/neohookean.h>
#include <talus/dynamics/screen/screen.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace talus::dynamics {

struct LatticeBody {
  uint32_t id = 0u;
  std::shared_ptr<const talus::PreparedMesh> mesh;
  const field::SampledSdf* field = nullptr;
  std::vector<Moduli> moduli;
  std::vector<talus::PhysicalMaterial> materials;
  std::vector<glm::dmat3> plastic;
  std::vector<double> hardening;
  std::vector<GaussGradients> gaussGradients;
  std::vector<uint32_t> gaussGradientsOf;
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> velocity;
  std::vector<glm::dvec3> initial;
  std::vector<glm::dvec3> inertial;
  std::vector<glm::dvec3> previousVelocity;
  std::vector<uint8_t> colour;
  std::vector<uint32_t> cellStart;
  std::vector<uint32_t> cellOf;
  std::vector<uint8_t> cornerOf;
  double friction = 0.5;
  double cell = 0.0;
  double sleepTime = 0.0;
  bool asleep = false;
  uint32_t substeps = 0u;
};

enum class RigidShape : uint8_t { Sampled, Sphere, Box, Compound };

struct RigidBody {
  uint32_t id = 0u;
  const field::SampledSdf* field = nullptr;
  std::vector<glm::dvec3> samples;
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 velocity{0.0};
  glm::dvec3 angularVelocity{0.0};
  glm::dvec3 previousVelocity{0.0};
  glm::dvec3 initialPosition{0.0};
  glm::dvec3 inertialPosition{0.0};
  glm::dquat initialRotation{1.0, 0.0, 0.0, 0.0};
  glm::dquat inertialRotation{1.0, 0.0, 0.0, 0.0};
  double mass = 0.0;
  glm::dvec3 moment{0.0};
  double friction = 0.5;
  double restitution = 0.0;
  double radius = 0.0;
  RigidShape shape = RigidShape::Sampled;
  glm::dvec3 halfExtents{0.0};
  std::shared_ptr<const CompoundShape> compound;
  glm::dvec3 fieldOrigin{0.0};
  glm::dquat fieldRotation{1.0, 0.0, 0.0, 0.0};
  double sleepTime = 0.0;
  bool asleep = false;
  double frozenMass = 0.0;
  glm::dvec3 frozenMoment{0.0};
};

enum class Tier : uint8_t { Rigid, Lattice };

struct SwitchableBody {
  uint32_t id = 0u;
  Tier tier = Tier::Rigid;
  RigidBody rigid;
  LatticeBody lattice;
  RestBody rest;
  std::shared_ptr<const ScreenFactor> screen;
  std::shared_ptr<const ScreenFactor> supportScreen;
  std::vector<uint32_t> supportNodes;
  std::shared_ptr<const deformable::DeformedProbe> restProbe;
  std::shared_ptr<field::SampledSdf> baked;
  double dent = 0.0;
  std::vector<glm::dvec3> displacement;
  double checkedLoad = 0.0;
  double checkedTurn = 0.0;
  bool checked = false;
  uint32_t checks = 0u;
  uint32_t frames = 0u;
};

struct StaticField {
  const field::SampledSdf* field = nullptr;
  double friction = 0.5;
  double restitution = 0.0;
  glm::dvec3 position{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  uint32_t id = 0u;
};

struct BodySet {
  std::vector<LatticeBody> lattices;
  std::vector<RigidBody> rigids;
  std::vector<StaticField> statics;
  uint32_t nextId = 0u;
  std::vector<SwitchableBody> switchables;
  uint32_t nextStaticId = 0u;
  std::vector<uint32_t> stirred;
  uint64_t layout = 0u;
};

LatticeBody makeLattice(
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  const field::SampledSdf* restField
);

void rebuildAdjacency(LatticeBody& lattice);

uint32_t addLattice(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  const field::SampledSdf* restField = nullptr
);

uint32_t addRigidBody(
  BodySet& bodies,
  const field::SampledSdf& field,
  std::span<const glm::dvec3> samples,
  double mass,
  const glm::dvec3& moment,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  double restitution
);

uint32_t addRigidSphere(
  BodySet& bodies,
  const field::SampledSdf& field,
  double radius,
  double mass,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  double restitution
);

uint32_t addRigidBox(
  BodySet& bodies,
  const field::SampledSdf& field,
  const glm::dvec3& halfExtents,
  double mass,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double friction,
  uint32_t samplesPerEdge
);

uint32_t addCompoundBody(
  BodySet& bodies,
  std::span<const CompoundChild> children,
  const glm::dvec3& position,
  const glm::dquat& rotation,
  const glm::dvec3& velocity,
  double friction,
  double restitution,
  bool anchored
);

uint32_t addStatic(
  BodySet& bodies,
  const field::SampledSdf& field,
  double friction,
  double restitution = 0.0,
  const glm::dvec3& position = glm::dvec3(0.0),
  const glm::dquat& rotation = glm::dquat(1.0, 0.0, 0.0, 0.0)
);

[[nodiscard]] glm::dmat3 worldInertia(const RigidBody& body);

[[nodiscard]] double reachOf(const RigidBody& body);

[[nodiscard]] std::optional<uint32_t> findStatic(const BodySet& bodies, uint32_t id);

[[nodiscard]] glm::dvec3 staticLocal(const StaticField& body, const glm::dvec3& world);

[[nodiscard]] glm::dvec3 staticDirection(const StaticField& body, const glm::dvec3& local);

}
