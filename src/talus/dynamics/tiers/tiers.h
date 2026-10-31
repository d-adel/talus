#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace talus::dynamics {

[[nodiscard]] uint32_t addSwitchable(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const field::SampledSdf& field,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  double friction,
  double restitution
);

[[nodiscard]] uint32_t addSwitchable(
  BodySet& bodies,
  std::shared_ptr<const talus::PreparedMesh> mesh,
  std::span<const talus::PhysicalMaterial> materials,
  const field::SampledSdf& field,
  const glm::dvec3& offset,
  const glm::dvec3& velocity,
  const RigidBody& shape
);

[[nodiscard]] std::optional<BodyRef> findBody(const BodySet& bodies, uint32_t id);

[[nodiscard]] std::optional<BodyRef> locateBody(
  const World& world,
  uint32_t id
);

void switchTiers(World& world, StepReport& report);

bool promote(World& world, uint32_t id);

struct LatticeFit {
  glm::dvec3 centre{0.0};
  glm::dmat3 rotation{1.0};
  double dent = 0.0;
};

[[nodiscard]] LatticeFit fitLattice(const LatticeBody& lattice, const RestBody& rest);

void demote(World& world, uint32_t id);

[[nodiscard]] std::vector<uint8_t> shareCarriedForces(
  std::span<const CarriedForce> carried,
  std::span<Contact> contacts,
  const BodySet& bodies
);

}
