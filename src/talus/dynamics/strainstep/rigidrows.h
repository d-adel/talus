#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/strainstep/rigidstate.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::dynamics {

struct ManifoldRowPoint {
  uint32_t point = 0u;
  glm::dvec3 rA{0.0};
  glm::dvec3 rB{0.0};
  double baseSeparation = 0.0;
  double normalImpulse = 0.0;
  double totalNormalImpulse = 0.0;
  double restitutionImpulse = 0.0;
  double normalMass = 0.0;
  double relativeVelocity = 0.0;
  double leverArm = 0.0;
};

struct ManifoldRow {
  uint32_t a = 0u;
  uint32_t b = 0u;
  uint32_t slot = 0u;
  uint32_t manifold = 0u;
  uint32_t start = 0u;
  uint32_t count = 0u;
  glm::dvec3 normal{0.0, 1.0, 0.0};
  glm::dvec3 tangent1{1.0, 0.0, 0.0};
  glm::dvec3 tangent2{0.0, 0.0, 1.0};
  glm::dvec3 centerA{0.0};
  glm::dvec3 centerB{0.0};
  glm::dmat2 tangentMass{0.0};
  glm::dvec2 frictionImpulse{0.0};
  double twistMass = 0.0;
  double twistImpulse = 0.0;
  double friction = 0.0;
  double restitution = 0.0;
  Softness softness;
};

struct ManifoldSettings {
  double h = 0.0;
  double speculativeDistance = 0.02;
  Softness contact;
  Softness fixed;
};

void prepareManifolds(
  const BodySet& bodies,
  const ManifoldCache& cache,
  std::span<const uint32_t> slots,
  std::span<const RigidState> states,
  std::span<const uint32_t> stateOf,
  const ManifoldSettings& settings,
  std::vector<ManifoldRow>& rows,
  std::vector<ManifoldRowPoint>& points
);

void warmStartManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<const ManifoldRowPoint> points
);

void pushManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH,
  double contactSpeed
);

void relaxManifold(
  std::span<RigidState> states,
  ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH
);

void restituteManifold(
  std::span<RigidState> states,
  const ManifoldRow& row,
  std::span<ManifoldRowPoint> points,
  double invH,
  double threshold
);

void storeManifolds(
  ManifoldCache& cache,
  std::span<const ManifoldRow> rows,
  std::span<const ManifoldRowPoint> points,
  double h
);

}
