#pragma once

#include <talus/field/sampledsdf/sampledsdf.h>

#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace talus::deformable {

struct ShapeProbe {
  const talus::field::SampledSdf* field = nullptr;
  glm::vec3 centerOfMass{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 fieldOffset{0.0f};
  glm::vec3 linearVelocity{0.0f};
  glm::vec3 angularVelocity{0.0f};
};

struct ShapeAt {
  glm::vec3 centerOfMass{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
};

[[nodiscard]] ShapeAt advanceShape(const ShapeProbe& shape, float elapsed);

[[nodiscard]] glm::vec3 fieldPointOf(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
);

struct ShapeDistance {
  float distance = 0.0f;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
};

[[nodiscard]] ShapeDistance probeShape(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
);

struct PreciseDistance {
  double distance = 0.0;
  glm::dvec3 normal{0.0, 1.0, 0.0};
  glm::dvec3 gradient{0.0};
};

[[nodiscard]] PreciseDistance probeShapePrecise(
  const ShapeProbe& shape,
  const glm::dquat& rotation,
  const glm::dvec3& relative
);

struct SurfaceContact {
  glm::uvec3 nodes{0u};
  glm::vec3 weights{0.0f};
  int partner = -1;
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
  glm::vec3 localPoint{0.0f};
  float gap = 0.0f;
  float bias = 0.0f;
  float friction = 0.5f;
  uint32_t triangle = 0u;
};

struct ContactGaps {
  float bias = 0.0f;
  float threshold = 0.0f;
};

inline constexpr uint32_t SURFACE_CLUSTER = 32u;

void clusterSurface(talus::PreparedMesh& mesh);

[[nodiscard]] std::vector<SurfaceContact> findContacts(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::vec3& gravity,
  std::span<const ShapeProbe> shapes,
  float deltaTime,
  const ContactGaps& gaps,
  bool cull = true
);

inline constexpr double DEFORMED_TOLERANCE = 1.0e-10;
inline constexpr uint32_t DEFORMED_ITERATIONS = 8u;
inline constexpr double DEFORMED_PAD = 1.5;
inline constexpr double DEFORMED_PAD_FLOOR = 0.2;
inline constexpr double DEFORMED_PAD_SLACK = 3.0;
inline constexpr double DEFORMED_SCREEN = 0.5;

struct DeformedProbe {
  const talus::PreparedMesh* mesh = nullptr;
  const talus::field::SampledSdf* field = nullptr;
  std::vector<uint8_t> boundary;
  std::vector<uint8_t> cracked;
  std::vector<glm::dvec3> position;
  std::vector<glm::dvec3> low;
  std::vector<glm::dvec3> high;
  std::vector<glm::dvec3> centre;
  std::vector<glm::dmat3> inverse;
  double cell = 0.0;
  double padCells = DEFORMED_PAD;
  double pad = 0.0;
  glm::dvec3 minimum{0.0};
  glm::dvec3 maximum{0.0};
  glm::uvec3 cells{0u};
  std::vector<uint32_t> first;
  std::vector<uint32_t> members;
};

[[nodiscard]] DeformedProbe deformedProbe(
  const talus::PreparedMesh& mesh,
  const talus::field::SampledSdf& field,
  double reach = std::numeric_limits<double>::infinity()
);

void placeDeformedProbe(
  DeformedProbe& probe,
  std::span<const glm::dvec3> position
);

struct DeformedCounts {
  uint32_t unconverged = 0u;
  uint32_t inverted = 0u;
};

struct DeformedDistance {
  bool found = false;
  bool band = false;
  double distance = std::numeric_limits<double>::infinity();
  glm::dvec3 normal{0.0, 1.0, 0.0};
  glm::dvec3 gradient{0.0};
  glm::dvec3 local{0.0};
  glm::dvec3 rest{0.0};
  uint32_t element = 0u;
  std::array<uint32_t, 8> nodes{};
  std::array<double, 8> weights{};
};

[[nodiscard]] DeformedDistance probeDeformed(
  const DeformedProbe& probe,
  const glm::dvec3& point,
  DeformedCounts* counts = nullptr
);

}
