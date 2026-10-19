#include "fieldbody.h"

#include <talus/field/constants.h>
#include <talus/field/crossings/crossings.h>
#include <talus/field/shapekind/shapekind.h>

#include <Eigen/Dense>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <unordered_map>
#include <vector>

namespace talus::dynamics {

namespace {

const talus::PhysicalMaterial& laneOf(
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t lane
) {
  static const talus::PhysicalMaterial fallback{};

  return lane < materials.size() ? materials[lane] : fallback;
}

bool unitAxes(const glm::mat4& model) {
  for (int column = 0; column != 3; ++column) {
    if (std::abs(glm::length(glm::vec3(model[column])) - 1.0f) > 1.0e-4f) {
      return false;
    }
  }

  return true;
}

std::vector<glm::dvec3> boxSamples(const glm::dvec3& half) {
  std::vector<glm::dvec3> samples;

  for (int i = 0; i != 3; ++i) {
    for (int j = 0; j != 3; ++j) {
      for (int k = 0; k != 3; ++k) {
        if (i == 1 && j == 1 && k == 1) {
          continue;
        }

        samples.push_back(glm::dvec3(i - 1, j - 1, k - 1) * half);
      }
    }
  }

  return samples;
}

RigidBody exactBody(
  const field::Primitive& primitive,
  std::span<const talus::PhysicalMaterial> materials
) {
  const glm::mat4 model = glm::inverse(primitive.inverseModel);
  const talus::PhysicalMaterial& material =
    laneOf(materials, primitive.shapeConfig.y);
  const double density = static_cast<double>(material.density);
  RigidBody body;

  body.fieldOrigin = glm::dvec3(glm::vec3(model[3]));
  body.friction = static_cast<double>(material.friction);
  body.restitution = static_cast<double>(material.restitution);

  if (field::shapeKindOf(primitive.shapeConfig.x) == field::ShapeKind::Sphere) {
    const double radius = static_cast<double>(primitive.shape.x);

    body.mass = density * 4.0 / 3.0 * std::numbers::pi * radius * radius * radius;
    body.moment = glm::dvec3(0.4 * body.mass * radius * radius);
    body.radius = radius;
    body.shape = RigidShape::Sphere;

    return body;
  }

  const glm::dvec3 half(glm::vec3(primitive.shape));
  const glm::dvec3 size = 2.0 * half;

  body.mass = density * size.x * size.y * size.z;
  body.moment = glm::dvec3(
    body.mass * (size.y * size.y + size.z * size.z) / 12.0,
    body.mass * (size.x * size.x + size.z * size.z) / 12.0,
    body.mass * (size.x * size.x + size.y * size.y) / 12.0
  );
  body.samples = boxSamples(half);
  body.halfExtents = half;
  body.shape = RigidShape::Box;
  body.fieldRotation =
    glm::normalize(glm::dquat(glm::quat_cast(glm::mat3(model))));

  return body;
}

bool exact(std::span<const field::Primitive> primitives) {
  if (primitives.size() != 1u
      || primitives.front().shapeConfig.z != field::FIELD_OP_UNION) {
    return false;
  }

  const field::ShapeKind kind = field::shapeKindOf(primitives.front().shapeConfig.x);

  return (kind == field::ShapeKind::Box || kind == field::ShapeKind::Sphere)
         && unitAxes(glm::inverse(primitives.front().inverseModel));
}

std::vector<glm::dvec3> thinned(
  const std::vector<glm::dvec3>& points,
  uint32_t budget
) {
  if (points.size() <= budget) {
    return points;
  }

  glm::dvec3 low(std::numeric_limits<double>::infinity());
  glm::dvec3 high(-std::numeric_limits<double>::infinity());

  for (const glm::dvec3& point : points) {
    low = glm::min(low, point);
    high = glm::max(high, point);
  }

  const double extent =
    std::max({high.x - low.x, high.y - low.y, high.z - low.z, 1.0e-9});

  uint32_t divisions = 64u;

  while (divisions != 0u) {
    const double size = extent / static_cast<double>(divisions);
    std::unordered_map<uint64_t, size_t> first;

    for (size_t index = 0u; index != points.size(); ++index) {
      const glm::dvec3 cell = glm::floor((points[index] - low) / size);
      const uint64_t key = static_cast<uint64_t>(cell.x)
                           | static_cast<uint64_t>(cell.y) << 21u
                           | static_cast<uint64_t>(cell.z) << 42u;

      first.try_emplace(key, index);
    }

    if (first.size() > budget) {
      const double shrink = std::sqrt(
        static_cast<double>(budget) / static_cast<double>(first.size())
      );

      divisions = std::min(
        divisions - 1u,
        std::max(1u, static_cast<uint32_t>(static_cast<double>(divisions) * shrink))
      );

      continue;
    }

    std::vector<size_t> kept;

    kept.reserve(first.size());

    for (const auto& [key, index] : first) {
      kept.push_back(index);
    }

    std::sort(kept.begin(), kept.end());

    std::vector<glm::dvec3> chosen;

    chosen.reserve(kept.size());

    for (const size_t index : kept) {
      chosen.push_back(points[index]);
    }

    return chosen;
  }

  return {points.front()};
}

RigidBody sampledBody(
  const field::SampledSdf& field,
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t budget
) {
  const glm::dvec3 voxel(field.voxelSize);
  const double cell = voxel.x * voxel.y * voxel.z;
  const double ramp = std::min({voxel.x, voxel.y, voxel.z});
  double mass = 0.0;
  double friction = 0.0;
  double restitution = 0.0;
  glm::dvec3 first(0.0);
  glm::dmat3 second(0.0);
  RigidBody body;

  body.shape = RigidShape::Sampled;

  for (uint32_t z = 0u; z != field.resolution.z; ++z) {
    for (uint32_t y = 0u; y != field.resolution.y; ++y) {
      for (uint32_t x = 0u; x != field.resolution.x; ++x) {
        const size_t index =
          x + static_cast<size_t>(field.resolution.x)
                * (y + static_cast<size_t>(field.resolution.y) * z);

        const double fill = std::clamp(
          0.5 - static_cast<double>(field.distances[index]) / ramp,
          0.0,
          1.0
        );

        if (!(fill > 0.0)) {
          continue;
        }

        const talus::PhysicalMaterial& material = laneOf(
          materials,
          field.materials.empty() ? 0u : field.materials[index]
        );
        const double weight =
          static_cast<double>(material.density) * cell * fill;
        const glm::dvec3 point =
          glm::dvec3(field.origin) + voxel * glm::dvec3(x, y, z);

        mass += weight;
        friction += weight * static_cast<double>(material.friction);
        restitution += weight * static_cast<double>(material.restitution);
        first += weight * point;
        second += weight * glm::outerProduct(point, point);
      }
    }
  }

  if (!(mass > 0.0)) {
    return body;
  }

  const glm::dvec3 centre = first / mass;
  const glm::dmat3 spread = second - mass * glm::outerProduct(centre, centre);
  const double trace = spread[0][0] + spread[1][1] + spread[2][2];
  Eigen::Matrix3d inertia;

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      inertia(row, column) =
        (row == column ? trace : 0.0) - spread[column][row];
    }
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solved(inertia);
  Eigen::Matrix3d axes = solved.eigenvectors();

  if (axes.determinant() < 0.0) {
    axes.col(2) = -axes.col(2);
  }

  glm::dmat3 frame(1.0);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      frame[column][row] = axes(row, column);
    }
  }

  body.mass = mass;
  body.moment = glm::dvec3(
    solved.eigenvalues()(0),
    solved.eigenvalues()(1),
    solved.eigenvalues()(2)
  );
  body.fieldOrigin = centre;
  body.fieldRotation = glm::normalize(glm::quat_cast(frame));
  body.friction = friction / mass;
  body.restitution = restitution / mass;

  if (budget == 0u) {
    return body;
  }

  const field::LatticeSurface surface = field::latticeSurface(field);
  std::vector<glm::dvec3> crossings;

  crossings.reserve(surface.crossings.size());

  for (const field::Crossing& crossing : surface.crossings) {
    crossings.push_back(glm::dvec3(field::latticeToField(field, crossing.lattice)));
  }

  const glm::dquat back = glm::conjugate(body.fieldRotation);

  for (const glm::dvec3& point : thinned(crossings, budget)) {
    body.samples.push_back(back * (point - centre));
  }

  return body;
}

}

RigidBody rigidFromField(
  const field::SampledSdf& field,
  std::span<const field::Primitive> primitives,
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t sampleBudget
) {
  RigidBody body = exact(primitives) ? exactBody(primitives.front(), materials)
                                     : sampledBody(field, materials, sampleBudget);

  body.field = &field;

  return body;
}

}
