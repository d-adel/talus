#include "meshbody.h"

#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/field/crossings/crossings.h>
#include <talus/field/meshsdf/meshsdf.h>
#include <talus/material.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr double largestAutomaticVoxel = 0.01;
constexpr double smallestAutomaticVoxel = 0.004;
constexpr double voxelsAcrossThinnest = 12.0;
constexpr double paddingVoxels = 3.0;
constexpr double fewestSamples = 32.0;
constexpr uint32_t projections = 2u;
constexpr size_t maximumSamples = size_t{1} << 18u;

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

double enclosedVolume(
  std::span<const glm::vec3> vertices,
  std::span<const uint32_t> indices
) {
  const glm::dvec3 origin(vertices[indices.front()]);
  double volume = 0.0;

  for (size_t face = 0u; face < indices.size(); face += 3u) {
    const glm::dvec3 a = glm::dvec3(vertices[indices[face]]) - origin;
    const glm::dvec3 b = glm::dvec3(vertices[indices[face + 1u]]) - origin;
    const glm::dvec3 c = glm::dvec3(vertices[indices[face + 2u]]) - origin;

    volume += glm::dot(a, glm::cross(b, c));
  }

  return std::abs(volume) / 6.0;
}

uint32_t sideOf(const glm::dvec3& normal) {
  const glm::dvec3 size = glm::abs(normal);
  const uint32_t axis = size.x >= size.y && size.x >= size.z ? 0u
                        : size.y >= size.z                   ? 1u
                                                             : 2u;

  return 2u * axis + (normal[axis] > 0.0 ? 1u : 0u);
}

bool projected(const field::SampledSdf& field, glm::dvec3& point) {
  for (uint32_t pass = 0u; pass != projections; ++pass) {
    const field::FieldProbeOf<double> probed =
      field::probe<double>(field, point);
    const double slope = glm::length(probed.gradient);

    if (!std::isfinite(probed.distance) || !(slope >= 1.0e-6)) {
      return false;
    }

    point -= probed.distance / (slope * slope) * probed.gradient;
  }

  return true;
}

std::vector<glm::dvec3> surfacePoints(
  const field::SampledSdf& field,
  double spacing
) {
  struct Bin {
    glm::dvec3 sum{0.0};
    glm::dvec3 first{0.0};
    double count = 0.0;
  };

  const glm::dvec3 origin(field.origin);
  const glm::dvec3 voxel(field.voxelSize);
  const double cell = std::max({spacing, voxel.x, voxel.y, voxel.z});
  std::map<std::array<uint32_t, 4>, size_t> slots;
  std::vector<Bin> bins;

  for (const field::Crossing& crossing :
       field::latticeSurface(field).crossings) {
    const glm::dvec3 point = origin + voxel * glm::dvec3(crossing.lattice);
    const glm::uvec3 at(glm::floor((point - origin) / cell));
    const uint32_t side = sideOf(field::probe<double>(field, point).normal);
    const auto [slot, added] = slots.try_emplace(
      std::array<uint32_t, 4>{at.x, at.y, at.z, side},
      bins.size()
    );

    if (added) {
      bins.push_back(Bin{glm::dvec3(0.0), point, 0.0});
    }

    bins[slot->second].sum += point;
    bins[slot->second].count += 1.0;
  }

  std::vector<glm::dvec3> points;

  points.reserve(bins.size());

  for (const Bin& bin : bins) {
    glm::dvec3 point = bin.sum / bin.count;

    points.push_back(projected(field, point) ? point : bin.first);
  }

  return points;
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

  talus::PhysicalMaterial unit;

  unit.density = 1.0f;

  const std::array<talus::PhysicalMaterial, 1> materials{unit};
  RigidBody body = rigidFromField(*baked, {}, materials, 0u);
  const double area = surfaceArea(vertices, indices);
  const glm::dquat back = glm::conjugate(body.fieldRotation);

  for (const glm::dvec3& point : surfacePoints(
         *baked,
         std::min(settings.spacing, std::sqrt(area / fewestSamples))
       )) {
    body.samples.push_back(back * (point - body.fieldOrigin));
  }

  if (!(body.mass > 0.0) || body.samples.empty()) {
    throw std::invalid_argument(
      "A mesh body needs a mesh that encloses lattice points at its voxel size"
    );
  }

  if (body.samples.size() > maximumSamples) {
    throw std::invalid_argument(
      "A mesh body can hold at most 262,144 contact points; use a coarser "
      "spacing"
    );
  }

  if (!(std::abs(body.mass - enclosedVolume(vertices, indices))
        <= 0.5 * area * voxel)) {
    throw std::invalid_argument(
      "A mesh body's shells must not overlap or nest with the same winding"
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
