#include "meshbody.h"

#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/field/meshsdf/meshsdf.h>
#include <talus/material.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace talus::dynamics {

namespace {

constexpr double largestAutomaticVoxel = 0.01;
constexpr double smallestAutomaticVoxel = 0.004;
constexpr double voxelsAcrossThinnest = 12.0;
constexpr double paddingVoxels = 3.0;
constexpr double fewestSamples = 32.0;

void check(const MeshShapeSettings& settings) {
  if (!(settings.mass > 0.0) || !std::isfinite(settings.mass)) {
    throw std::invalid_argument("A mesh body needs a positive, finite mass");
  }

  if (!(settings.spacing > 0.0) || !std::isfinite(settings.spacing)) {
    throw std::invalid_argument(
      "A mesh body needs a positive, finite contact point spacing"
    );
  }

  if (!(settings.voxel >= 0.0) || !std::isfinite(settings.voxel)) {
    throw std::invalid_argument(
      "A mesh body's voxel size must be 0 (automatic) or positive and finite"
    );
  }

  if (!(settings.friction >= 0.0) || !std::isfinite(settings.friction)
      || !(settings.restitution >= 0.0 && settings.restitution <= 1.0)) {
    throw std::invalid_argument(
      "A mesh body needs a friction of at least zero and a restitution from 0 "
      "to 1"
    );
  }
}

double surfaceArea(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices
) {
  double area = 0.0;

  for (size_t face = 0u; face < indices.size(); face += 3u) {
    const glm::dvec3 a(vertices[indices[face]]);
    const glm::dvec3 b(vertices[indices[face + 1u]]);
    const glm::dvec3 c(vertices[indices[face + 2u]]);

    area += 0.5 * glm::length(glm::cross(b - a, c - a));
  }

  return area;
}

}

MeshShape meshShape(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices,
  const MeshShapeSettings& settings,
  const Workers& workers
) {
  check(settings);

  if (vertices.empty() || indices.empty()) {
    throw std::invalid_argument("A mesh needs vertices and triangles");
  }

  glm::dvec3 low(std::numeric_limits<double>::infinity());
  glm::dvec3 high(-std::numeric_limits<double>::infinity());

  for (const glm::vec3& vertex : vertices) {
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)
        || !std::isfinite(vertex.z)) {
      throw std::invalid_argument("Mesh vertices must be finite");
    }

    low = glm::min(low, glm::dvec3(vertex));
    high = glm::max(high, glm::dvec3(vertex));
  }

  const glm::dvec3 extent = high - low;
  const double voxel =
    settings.voxel > 0.0
      ? settings.voxel
      : std::max(
          smallestAutomaticVoxel,
          std::min(
            largestAutomaticVoxel,
            std::min({extent.x, extent.y, extent.z}) / voxelsAcrossThinnest
          )
        );
  const glm::dvec3 minimum = low - paddingVoxels * voxel;
  const glm::dvec3 maximum = high + paddingVoxels * voxel;
  const glm::dvec3 points =
    glm::ceil((maximum - minimum) / voxel) + glm::dvec3(1.0);

  if (!(points.x * points.y * points.z
        <= static_cast<double>(std::numeric_limits<uint32_t>::max()))) {
    throw std::invalid_argument(
      "A mesh field at this voxel size has too many lattice points"
    );
  }

  std::shared_ptr<const field::SampledSdf> baked =
    std::make_shared<const field::SampledSdf>(field::bakeMesh(
      vertices,
      indices,
      glm::vec3(minimum),
      glm::vec3(maximum),
      glm::uvec3(points),
      0u,
      workers
    ));

  const double budget = std::max(
    fewestSamples,
    std::ceil(
      surfaceArea(vertices, indices) / (settings.spacing * settings.spacing)
    )
  );
  talus::PhysicalMaterial unit;

  unit.density = 1.0f;

  const std::array<talus::PhysicalMaterial, 1> materials{unit};
  RigidBody body = rigidFromField(
    *baked,
    {},
    materials,
    static_cast<uint32_t>(std::min(
      budget,
      static_cast<double>(std::numeric_limits<uint32_t>::max())
    ))
  );

  if (!(body.mass > 0.0) || body.samples.empty()) {
    throw std::invalid_argument(
      "A mesh body needs a mesh that encloses lattice points at its voxel size"
    );
  }

  body.moment *= settings.mass / body.mass;
  body.mass = settings.mass;
  body.friction = settings.friction;
  body.restitution = settings.restitution;

  return MeshShape{std::move(baked), std::move(body)};
}

uint32_t addMeshBody(
  BodySet& bodies,
  const MeshShape& shape,
  const glm::dvec3& position,
  const glm::dquat& rotation,
  const glm::dvec3& velocity
) {
  if (shape.field == nullptr) {
    throw std::invalid_argument("A mesh body needs a shape made by meshShape");
  }

  const glm::dquat turn = glm::normalize(rotation);
  const uint32_t index = addRigidBody(
    bodies,
    *shape.field,
    shape.body.samples,
    shape.body.mass,
    shape.body.moment,
    position + turn * shape.body.fieldOrigin,
    velocity,
    shape.body.friction,
    shape.body.restitution
  );
  RigidBody& body = bodies.rigids[index];

  body.rotation = glm::normalize(turn * shape.body.fieldRotation);
  body.initialRotation = body.rotation;
  body.inertialRotation = body.rotation;
  body.fieldOrigin = shape.body.fieldOrigin;
  body.fieldRotation = shape.body.fieldRotation;

  return body.id;
}

MeshFrame meshFrame(const RigidBody& body) {
  const glm::dquat turn =
    glm::normalize(body.rotation * glm::conjugate(body.fieldRotation));

  return MeshFrame{body.position - turn * body.fieldOrigin, turn};
}

glm::dvec3 meshPoint(const RigidBody& body, const glm::dvec3& sample) {
  return body.fieldOrigin + body.fieldRotation * sample;
}

}
