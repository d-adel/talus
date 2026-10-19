#include "contact.ih"

#include <talus/deformable/material/material.h>

#include <unordered_map>
#include <utility>

namespace talus::deformable {

ShapeAt advanceShape(const ShapeProbe& shape, float elapsed) {
  ShapeAt at;

  at.centerOfMass = shape.centerOfMass + shape.linearVelocity * elapsed;

  const float spin = glm::length(shape.angularVelocity);

  at.rotation = spin > 0.0f
                  ? glm::angleAxis(spin * elapsed, shape.angularVelocity / spin)
                      * shape.rotation
                  : shape.rotation;

  return at;
}

glm::vec3 fieldPointOf(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
) {
  return glm::conjugate(at.rotation) * (world - at.centerOfMass)
         + shape.fieldOffset;
}

ShapeDistance probeShape(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const glm::vec3& world
) {
  const talus::field::FieldProbe probed =
    talus::field::probe(*shape.field, fieldPointOf(shape, at, world));

  const glm::vec3 outward = at.rotation * probed.normal;
  const float length = glm::length(outward);

  return ShapeDistance{
    probed.distance,
    length > 0.0f ? outward / length : glm::vec3(0.0f, 1.0f, 0.0f)
  };
}

PreciseDistance probeShapePrecise(
  const ShapeProbe& shape,
  const glm::dquat& rotation,
  const glm::dvec3& relative
) {
  const talus::field::FieldProbeOf<double> probed = talus::field::probe(
    *shape.field,
    glm::conjugate(rotation) * relative + glm::dvec3(shape.fieldOffset)
  );

  const glm::dvec3 outward = rotation * probed.normal;
  const double length = glm::length(outward);

  return PreciseDistance{
    probed.distance,
    length > 0.0 ? outward / length : glm::dvec3(0.0, 1.0, 0.0),
    rotation * probed.gradient
  };
}

namespace {

constexpr uint32_t ADVANCE_ITERATIONS = 4u;
constexpr uint32_t DESCENT_ITERATIONS = 8u;
constexpr uint32_t BACKTRACKS = 4u;
constexpr float IMPACT_RELATIVE_ERROR = 0.05f;
constexpr float CULL_TOLERANCE = 1.0e-5f;

template<typename Real>
struct TriangleOf {
  std::array<glm::vec<3, Real>, 3> corner{};
};

using Triangle = TriangleOf<float>;

template<typename Real>
glm::vec<3, Real> barycentricOf(
  const TriangleOf<Real>& triangle,
  const glm::vec<3, Real>& point
) {
  using Vector = glm::vec<3, Real>;

  const Real zero(0);
  const Real one(1);

  const Vector& a = triangle.corner[0];
  const Vector& b = triangle.corner[1];
  const Vector& c = triangle.corner[2];

  const Vector ab = b - a;
  const Vector ac = c - a;
  const Vector ap = point - a;

  const Real d1 = glm::dot(ab, ap);
  const Real d2 = glm::dot(ac, ap);

  if (d1 <= zero && d2 <= zero) {
    return Vector(one, zero, zero);
  }

  const Vector bp = point - b;
  const Real d3 = glm::dot(ab, bp);
  const Real d4 = glm::dot(ac, bp);

  if (d3 >= zero && d4 <= d3) {
    return Vector(zero, one, zero);
  }

  const Real vc = d1 * d4 - d3 * d2;

  if (vc <= zero && d1 >= zero && d3 <= zero) {
    const Real v = d1 / (d1 - d3);

    return Vector(one - v, v, zero);
  }

  const Vector cp = point - c;
  const Real d5 = glm::dot(ab, cp);
  const Real d6 = glm::dot(ac, cp);

  if (d6 >= zero && d5 <= d6) {
    return Vector(zero, zero, one);
  }

  const Real vb = d5 * d2 - d1 * d6;

  if (vb <= zero && d2 >= zero && d6 <= zero) {
    const Real w = d2 / (d2 - d6);

    return Vector(one - w, zero, w);
  }

  const Real va = d3 * d6 - d5 * d4;

  if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero) {
    const Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

    return Vector(zero, one - w, w);
  }

  const Real denominator = one / (va + vb + vc);
  const Real v = vb * denominator;
  const Real w = vc * denominator;

  return Vector(one - v - w, v, w);
}

template<typename Real>
glm::vec<3, Real> pointAt(
  const TriangleOf<Real>& triangle,
  const glm::vec<3, Real>& weights
) {
  return weights.x * triangle.corner[0] + weights.y * triangle.corner[1]
         + weights.z * triangle.corner[2];
}

template<typename Real>
Real longestEdge(const TriangleOf<Real>& triangle) {
  return glm::max(
    glm::length(triangle.corner[1] - triangle.corner[0]),
    glm::max(
      glm::length(triangle.corner[2] - triangle.corner[1]),
      glm::length(triangle.corner[0] - triangle.corner[2])
    )
  );
}

template<typename Real, typename Distance>
struct ClosestOf {
  glm::vec<3, Real> weights{Real(0)};
  glm::vec<3, Real> point{Real(0)};
  Distance probe;
};

using Closest = ClosestOf<float, ShapeDistance>;

template<typename Real, typename Distance, typename Probe>
ClosestOf<Real, Distance> closestOn(
  const TriangleOf<Real>& triangle,
  const Probe& probe,
  Real stop
) {
  using Vector = glm::vec<3, Real>;
  using Found = ClosestOf<Real, Distance>;

  const Real zero(0);
  const Real half(0.5f);
  const Real third = Real(1) / Real(3);

  const std::array<Vector, 7> starts{
    Vector(Real(1), zero, zero),
    Vector(zero, Real(1), zero),
    Vector(zero, zero, Real(1)),
    Vector(half, half, zero),
    Vector(zero, half, half),
    Vector(half, zero, half),
    Vector(third, third, third)
  };

  Found best;
  best.probe.distance = std::numeric_limits<Real>::max();

  for (const Vector& weights : starts) {
    Found candidate;
    candidate.weights = weights;
    candidate.point = pointAt(triangle, weights);
    candidate.probe = probe(candidate.point);

    if (candidate.probe.distance < best.probe.distance) {
      best = candidate;
    }
  }

  if (!(best.probe.distance < stop)) {
    return best;
  }

  const Vector across = glm::cross(
    triangle.corner[1] - triangle.corner[0],
    triangle.corner[2] - triangle.corner[0]
  );
  const Real area = glm::length(across);

  if (area <= zero) {
    return best;
  }

  const Vector facing = across / area;
  const Real edge = longestEdge(triangle);

  for (uint32_t iteration = 0u; iteration != DESCENT_ITERATIONS; ++iteration) {
    const Vector gradient = best.probe.normal;
    const Vector slide = -(gradient - facing * glm::dot(facing, gradient));
    const Real length = glm::length(slide);

    if (length <= Real(1.0e-4f)) {
      break;
    }

    const Vector direction = slide / length;

    Real stride = glm::clamp(
      std::abs(best.probe.distance),
      Real(0.05f) * edge,
      half * edge
    );

    bool improved = false;

    for (uint32_t attempt = 0u; attempt != BACKTRACKS; ++attempt) {
      Found candidate;
      candidate.weights =
        barycentricOf(triangle, best.point + direction * stride);
      candidate.point = pointAt(triangle, candidate.weights);
      candidate.probe = probe(candidate.point);

      if (candidate.probe.distance < best.probe.distance) {
        best = candidate;
        improved = true;

        break;
      }

      stride *= half;
    }

    if (!improved) {
      break;
    }
  }

  return best;
}

Closest closestOnTriangle(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const Triangle& triangle
) {
  return closestOn<float, ShapeDistance>(
    triangle,
    [&shape, &at](const glm::vec3& point) {
      return probeShape(shape, at, point);
    },
    std::numeric_limits<float>::max()
  );
}

struct Bounds {
  glm::vec3 minimum{std::numeric_limits<float>::max()};
  glm::vec3 maximum{std::numeric_limits<float>::lowest()};

  void include(const glm::vec3& point) {
    minimum = glm::min(minimum, point);
    maximum = glm::max(maximum, point);
  }

  [[nodiscard]] bool overlaps(const Bounds& other, float margin) const {
    return glm::all(glm::lessThanEqual(minimum - margin, other.maximum))
           && glm::all(glm::lessThanEqual(other.minimum - margin, maximum));
  }
};

Bounds sweptShapeBounds(const ShapeProbe& shape, float deltaTime) {
  const talus::field::SampledSdf& field = *shape.field;

  const glm::vec3 low = field.origin - shape.fieldOffset;
  const glm::vec3 high = field.origin
                         + field.voxelSize * glm::vec3(field.resolution - 1u)
                         - shape.fieldOffset;

  Bounds bounds;

  for (const float elapsed : {0.0f, deltaTime}) {
    const ShapeAt at = advanceShape(shape, elapsed);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::vec3 local(
        (corner & 1u) != 0u ? high.x : low.x,
        (corner & 2u) != 0u ? high.y : low.y,
        (corner & 4u) != 0u ? high.z : low.z
      );

      bounds.include(at.centerOfMass + at.rotation * local);
    }
  }

  return bounds;
}

float shapeRadius(const ShapeProbe& shape) {
  const talus::field::SampledSdf& field = *shape.field;

  const glm::vec3 low = field.origin - shape.fieldOffset;
  const glm::vec3 high = field.origin
                         + field.voxelSize * glm::vec3(field.resolution - 1u)
                         - shape.fieldOffset;

  return glm::length(glm::max(glm::abs(low), glm::abs(high)));
}

struct MovingTriangle {
  Triangle start;
  std::array<glm::vec3, 3> velocity{};

  [[nodiscard]] Triangle at(float elapsed) const {
    Triangle moved = start;

    for (size_t corner = 0u; corner != 3u; ++corner) {
      moved.corner[corner] += velocity[corner] * elapsed;
    }

    return moved;
  }

  [[nodiscard]] float speed() const {
    float fastest = 0.0f;

    for (const glm::vec3& held : velocity) {
      fastest = glm::max(fastest, glm::length(held));
    }

    return fastest;
  }
};

float impactBound(const ShapeProbe& shape, float speed, float radius) {
  return speed + glm::length(shape.linearVelocity)
         + glm::length(shape.angularVelocity) * radius;
}

float distanceBelow(
  const ShapeProbe& shape,
  const ShapeAt& origin,
  const glm::vec3& centre,
  float spread,
  float lipschitz
) {
  const glm::vec3 local = fieldPointOf(shape, origin, centre);
  const float sampled = talus::field::probeDistance(*shape.field, local);
  const float magnitude =
    1.0f + glm::length(centre) + glm::length(origin.centerOfMass)
    + glm::length(local) + glm::length(shape.field->origin) + std::abs(sampled)
    + spread;

  return sampled - lipschitz * spread
         - CULL_TOLERANCE * (1.0f + lipschitz) * magnitude;
}

bool outOfReach(
  const ShapeProbe& shape,
  float lower,
  float speed,
  float deltaTime,
  const ContactGaps& gaps,
  float radius
) {
  if (!(lower > gaps.threshold)) {
    return false;
  }

  const float bound = impactBound(shape, speed, radius);

  return bound <= 0.0f || (lower - gaps.bias) / bound > deltaTime;
}

bool farFromShape(
  const ShapeProbe& shape,
  const ShapeAt& origin,
  const MovingTriangle& moving,
  float deltaTime,
  const ContactGaps& gaps,
  float radius,
  float lipschitz
) {
  const Triangle& start = moving.start;
  const glm::vec3 centre =
    (start.corner[0] + start.corner[1] + start.corner[2]) / 3.0f;

  float spread = 0.0f;

  for (const glm::vec3& corner : start.corner) {
    spread = glm::max(spread, glm::distance(corner, centre));
  }

  return outOfReach(
    shape,
    distanceBelow(shape, origin, centre, spread, lipschitz),
    moving.speed(),
    deltaTime,
    gaps,
    radius
  );
}

struct ClusterReach {
  Bounds reach;
  glm::vec3 centre{0.0f};
  float spread = 0.0f;
  float speed = 0.0f;
  float thickness = 0.0f;
};

bool clusterFar(
  const ShapeProbe& shape,
  const ShapeAt& origin,
  const Bounds& swept,
  const ClusterReach& cluster,
  float deltaTime,
  const ContactGaps& gaps,
  float radius,
  float lipschitz
) {
  const ContactGaps skinned{
    gaps.bias + cluster.thickness,
    gaps.threshold + cluster.thickness
  };

  if (!cluster.reach.overlaps(swept, skinned.threshold)) {
    return true;
  }

  return outOfReach(
    shape,
    distanceBelow(shape, origin, cluster.centre, cluster.spread, lipschitz),
    cluster.speed,
    deltaTime,
    skinned,
    radius
  );
}

uint32_t spreadBits(uint32_t value) {
  value &= 0x3ffu;
  value = (value | (value << 16u)) & 0x030000ffu;
  value = (value | (value << 8u)) & 0x0300f00fu;
  value = (value | (value << 4u)) & 0x030c30c3u;
  value = (value | (value << 2u)) & 0x09249249u;

  return value;
}

bool advanceToImpact(
  const ShapeProbe& shape,
  const MovingTriangle& moving,
  float deltaTime,
  const ContactGaps& gaps,
  float radius,
  Closest& closest,
  float& impactTime
) {
  const float bound = impactBound(shape, moving.speed(), radius);

  if (bound <= 0.0f) {
    return false;
  }

  float elapsed = (closest.probe.distance - gaps.bias) / bound;

  if (elapsed > deltaTime) {
    return false;
  }

  const float reach = gaps.bias * (1.0f + IMPACT_RELATIVE_ERROR);

  for (uint32_t iteration = 0u; iteration != ADVANCE_ITERATIONS; ++iteration) {
    closest = closestOnTriangle(
      shape,
      advanceShape(shape, elapsed),
      moving.at(elapsed)
    );
    impactTime = elapsed;

    if (closest.probe.distance <= reach) {
      return true;
    }

    elapsed += (closest.probe.distance - gaps.bias) / bound;

    if (elapsed > deltaTime) {
      return false;
    }
  }

  return true;
}

}

void clusterSurface(talus::PreparedMesh& mesh) {
  mesh.surfaceClusterFaces.clear();
  mesh.surfaceClusterStarts.clear();

  const size_t faces = mesh.surfaceTriangles.size();

  if (faces == 0u) {
    return;
  }

  std::vector<glm::vec3> centres(faces);

  glm::vec3 low(std::numeric_limits<float>::max());
  glm::vec3 high(std::numeric_limits<float>::lowest());

  for (size_t face = 0u; face != faces; ++face) {
    const glm::uvec3& triangle = mesh.surfaceTriangles[face];

    centres[face] =
      (mesh.restPositions[triangle.x] + mesh.restPositions[triangle.y]
       + mesh.restPositions[triangle.z])
      / 3.0f;

    low = glm::min(low, centres[face]);
    high = glm::max(high, centres[face]);
  }

  const glm::vec3 span =
    glm::max(high - low, glm::vec3(std::numeric_limits<float>::min()));

  std::vector<std::pair<uint32_t, uint32_t>> keyed(faces);

  for (size_t face = 0u; face != faces; ++face) {
    const glm::uvec3 cell(
      glm::clamp((centres[face] - low) / span, 0.0f, 1.0f) * 1023.0f
    );

    keyed[face] = {
      spreadBits(cell.x) | (spreadBits(cell.y) << 1u)
        | (spreadBits(cell.z) << 2u),
      static_cast<uint32_t>(face)
    };
  }

  std::sort(keyed.begin(), keyed.end());

  mesh.surfaceClusterFaces.reserve(faces);

  for (const std::pair<uint32_t, uint32_t>& held : keyed) {
    mesh.surfaceClusterFaces.push_back(held.second);
  }

  for (size_t start = 0u; start < faces; start += SURFACE_CLUSTER) {
    mesh.surfaceClusterStarts.push_back(static_cast<uint32_t>(start));
  }

  mesh.surfaceClusterStarts.push_back(static_cast<uint32_t>(faces));
}

std::vector<SurfaceContact> findContacts(
  const talus::PreparedMesh& mesh,
  const talus::DeformableState& state,
  std::span<const talus::PhysicalMaterial> materials,
  const glm::vec3& gravity,
  std::span<const ShapeProbe> shapes,
  float deltaTime,
  const ContactGaps& gaps,
  bool cull
) {
  std::vector<SurfaceContact> contacts;

  const auto frictionOf = [&](uint32_t lane) {
    static const talus::PhysicalMaterial DEFAULT{};

    return (lane < materials.size() ? materials[lane] : DEFAULT).friction;
  };

  if (shapes.empty() || mesh.surfaceTriangles.empty()) {
    return contacts;
  }

  const auto velocityOf = [&](uint32_t node) {
    return mesh.anchored[node] != 0u
             ? glm::vec3(0.0f)
             : state.velocity[node] + gravity * deltaTime;
  };

  const float apart = 1.0e-2f * glm::compMax(mesh.voxelSize);

  const size_t faces = mesh.surfaceTriangles.size();

  thread_local std::vector<ClusterReach> clusters;
  thread_local std::vector<uint8_t> skipped;

  bool clustered = cull && mesh.surfaceClusterFaces.size() == faces
                   && mesh.surfaceClusterStarts.size() >= 2u
                   && mesh.surfaceClusterStarts.front() == 0u
                   && mesh.surfaceClusterStarts.back() == faces;

  if (clustered) {
    const size_t count = mesh.surfaceClusterStarts.size() - 1u;

    clusters.assign(count, ClusterReach{});

    for (size_t cluster = 0u; clustered && cluster != count; ++cluster) {
      ClusterReach& held = clusters[cluster];
      Bounds start;

      const uint32_t begin = mesh.surfaceClusterStarts[cluster];
      const uint32_t end = mesh.surfaceClusterStarts[cluster + 1u];

      if (end < begin) {
        clustered = false;

        break;
      }

      for (uint32_t at = begin; at != end; ++at) {
        const uint32_t face = mesh.surfaceClusterFaces[at];

        if (face >= faces) {
          clustered = false;

          break;
        }

        held.thickness = glm::max(
          held.thickness,
          face < mesh.surfaceThickness.size() ? mesh.surfaceThickness[face]
                                              : 0.0f
        );

        for (int corner = 0; corner != 3; ++corner) {
          const uint32_t node = mesh.surfaceTriangles[face][corner];
          const glm::vec3& position = state.position[node];
          const glm::vec3 velocity = velocityOf(node);

          start.include(position);
          held.reach.include(position);
          held.reach.include(position + velocity * deltaTime);
          held.speed = glm::max(held.speed, glm::length(velocity));
        }
      }

      held.centre = 0.5f * (start.minimum + start.maximum);

      for (uint32_t at = begin; clustered && at != end; ++at) {
        const uint32_t face = mesh.surfaceClusterFaces[at];

        for (int corner = 0; corner != 3; ++corner) {
          held.spread = glm::max(
            held.spread,
            glm::distance(
              state.position[mesh.surfaceTriangles[face][corner]],
              held.centre
            )
          );
        }
      }
    }
  }

  for (size_t index = 0u; index != shapes.size(); ++index) {
    const ShapeProbe& shape = shapes[index];

    if (shape.field == nullptr) {
      continue;
    }

    const Bounds swept = sweptShapeBounds(shape, deltaTime);
    const float radius = shapeRadius(shape);
    const size_t first = contacts.size();
    const ShapeAt origin = advanceShape(shape, 0.0f);
    const float lipschitz =
      cull ? talus::field::probeLipschitz(*shape.field) : 0.0f;

    if (clustered) {
      skipped.assign(faces, 0u);

      for (size_t cluster = 0u; cluster != clusters.size(); ++cluster) {
        if (!clusterFar(
              shape,
              origin,
              swept,
              clusters[cluster],
              deltaTime,
              gaps,
              radius,
              lipschitz
            )) {
          continue;
        }

        const uint32_t begin = mesh.surfaceClusterStarts[cluster];
        const uint32_t end = mesh.surfaceClusterStarts[cluster + 1u];

        for (uint32_t at = begin; at != end; ++at) {
          skipped[mesh.surfaceClusterFaces[at]] = 1u;
        }
      }
    }

    for (size_t face = 0u; face != faces; ++face) {
      if (clustered && skipped[face] != 0u) {
        continue;
      }

      const glm::uvec3& triangle = mesh.surfaceTriangles[face];
      const float thickness = face < mesh.surfaceThickness.size()
                                ? mesh.surfaceThickness[face]
                                : 0.0f;
      const ContactGaps skinned{
        gaps.bias + thickness,
        gaps.threshold + thickness
      };

      MovingTriangle moving;
      Bounds reach;

      for (size_t corner = 0u; corner != 3u; ++corner) {
        const uint32_t node = triangle[static_cast<int>(corner)];

        moving.start.corner[corner] = state.position[node];
        moving.velocity[corner] = velocityOf(node);

        reach.include(moving.start.corner[corner]);
        reach.include(
          moving.start.corner[corner] + moving.velocity[corner] * deltaTime
        );
      }

      if (!reach.overlaps(swept, skinned.threshold)) {
        continue;
      }

      if (
        cull
        && farFromShape(
          shape,
          origin,
          moving,
          deltaTime,
          skinned,
          radius,
          lipschitz
        )
      ) {
        continue;
      }

      Closest closest = closestOnTriangle(shape, origin, moving.start);
      float impactTime = 0.0f;

      if (
        closest.probe.distance > skinned.threshold
        && !advanceToImpact(
          shape,
          moving,
          deltaTime,
          skinned,
          radius,
          closest,
          impactTime
        )
      ) {
        continue;
      }

      const ShapeAt impact = advanceShape(shape, impactTime);

      if (closest.probe.distance <= 0.0f) {
        const Triangle now = moving.at(impactTime);
        const glm::vec3 across = glm::cross(
          now.corner[1] - now.corner[0],
          now.corner[2] - now.corner[0]
        );
        const float area = glm::length(across);

        if (area > 0.0f) {
          closest.probe.normal = -across / area;
        }
      }

      const glm::vec3 onShape =
        closest.point - closest.probe.distance * closest.probe.normal;
      const glm::vec3 local =
        glm::conjugate(impact.rotation) * (onShape - impact.centerOfMass);

      const glm::vec3 meshPoint = pointAt(moving.start, closest.weights);
      const glm::vec3 shapePoint =
        origin.centerOfMass + origin.rotation * local;

      bool duplicate = false;

      for (size_t held = first; held != contacts.size(); ++held) {
        const glm::vec3 other = pointAt(
          Triangle{
            {state.position[contacts[held].nodes.x],
             state.position[contacts[held].nodes.y],
             state.position[contacts[held].nodes.z]}
          },
          contacts[held].weights
        );

        if (glm::length(other - meshPoint) <= apart) {
          duplicate = true;

          break;
        }
      }

      if (duplicate) {
        continue;
      }

      SurfaceContact contact;
      contact.nodes = triangle;
      contact.weights = closest.weights;
      contact.partner = static_cast<int>(index);
      contact.normal = closest.probe.normal;
      contact.localPoint = local;
      contact.bias = skinned.bias;
      contact.friction = std::sqrt(
        frictionOf(face < mesh.surfaceLane.size() ? mesh.surfaceLane[face] : 0u)
        * frictionOf(
          talus::field::material(
            *shape.field,
            fieldPointOf(shape, impact, onShape)
          )
        )
      );
      contact.gap =
        glm::dot(contact.normal, meshPoint - shapePoint) - skinned.bias;
      contact.triangle = static_cast<uint32_t>(face);

      contacts.push_back(contact);
    }
  }

  return contacts;
}

namespace {

constexpr uint64_t CELL_BIAS = 1ull << 20u;

uint64_t cellKey(
  const glm::dvec3& rest,
  const glm::dvec3& smallest,
  const glm::dvec3& voxel
) {
  const glm::dvec3 scaled = glm::round((rest - smallest) / voxel);
  const auto packed = [](double value) {
    return static_cast<uint64_t>(
             static_cast<int64_t>(value) + int64_t(CELL_BIAS)
           )
           & 0x1fffffull;
  };

  return packed(scaled.x) | (packed(scaled.y) << 21u)
         | (packed(scaled.z) << 42u);
}

struct Inversion {
  glm::dvec3 local{0.5};
  bool converged = false;
  bool inverted = false;
};

Inversion invertHex(
  const DeformedProbe& probe,
  uint32_t element,
  const glm::dvec3& point
) {
  const talus::PreparedHex& hex = probe.mesh->hexes[element];

  Inversion found;

  double determinant = 0.0;

  for (uint32_t iteration = 0u; iteration != DEFORMED_ITERATIONS; ++iteration) {
    const std::array<double, 8> shape = hexShape(found.local);
    const std::array<glm::dvec3, 8> slope = hexShapeGradient(found.local);

    glm::dvec3 mapped(0.0);
    glm::dmat3 jacobian(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::dvec3& at = probe.position[hex.nodes[corner]];

      mapped += shape[corner] * at;
      jacobian += glm::outerProduct(at, slope[corner]);
    }

    determinant = glm::determinant(jacobian);

    if (determinant == 0.0 || !std::isfinite(determinant)) {
      break;
    }

    const glm::dvec3 step = glm::inverse(jacobian) * (mapped - point);

    found.local -= step;

    if (
      std::max({std::abs(step.x), std::abs(step.y), std::abs(step.z)})
      < DEFORMED_TOLERANCE
    ) {
      found.converged = true;

      break;
    }
  }

  found.inverted = found.converged && !(determinant > 0.0);

  return found;
}

DeformedDistance evaluateHex(
  const DeformedProbe& probe,
  uint32_t element,
  const glm::dvec3& local,
  bool band
) {
  const talus::PreparedHex& hex = probe.mesh->hexes[element];
  const std::array<double, 8> shape = hexShape(local);
  const std::array<glm::dvec3, 8> slope = hexShapeGradient(local);

  DeformedDistance made;

  made.found = true;
  made.band = band;
  made.element = element;
  made.local = local;

  glm::dmat3 deformed(0.0);
  glm::dmat3 rest(0.0);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 restCorner(probe.mesh->restPositions[hex.nodes[corner]]);

    made.nodes[corner] = hex.nodes[corner];
    made.weights[corner] = shape[corner];
    made.rest += shape[corner] * restCorner;
    deformed +=
      glm::outerProduct(probe.position[hex.nodes[corner]], slope[corner]);
    rest += glm::outerProduct(restCorner, slope[corner]);
  }

  const talus::field::FieldProbeOf<double> read =
    talus::field::probe(*probe.field, made.rest);

  const glm::dmat3 inverseDeformed = glm::inverse(glm::transpose(deformed));

  made.distance = read.distance;
  made.gradient = inverseDeformed * (glm::transpose(rest) * read.gradient);

  const uint8_t cracks =
    element < probe.cracked.size() ? probe.cracked[element] : 0u;

  for (int axis = 0; axis != 3 && cracks != 0u; ++axis) {
    for (int side = 0; side != 2; ++side) {
      if ((cracks & (1u << static_cast<uint32_t>(2 * axis + side))) == 0u) {
        continue;
      }

      const double extent = glm::length(rest[axis]);
      const double depth = side == 0 ? local[axis] : 1.0 - local[axis];
      const double face = -depth * extent;

      if (face > made.distance) {
        glm::dvec3 outward(0.0);

        outward[axis] = side == 0 ? -extent : extent;
        made.distance = face;
        made.gradient = inverseDeformed * outward;
      }
    }
  }

  const double length = glm::length(made.gradient);

  made.normal =
    length > 0.0 ? made.gradient / length : glm::dvec3(0.0, 1.0, 0.0);

  return made;
}

bool screened(
  const DeformedProbe& probe,
  uint32_t element,
  const glm::dvec3& point,
  uint8_t faces
) {
  const glm::dmat3& inverse = probe.inverse[element];

  if (inverse == glm::dmat3(0.0)) {
    return true;
  }

  const glm::dvec3 local =
    inverse * (point - probe.centre[element]) + glm::dvec3(0.5);

  for (int axis = 0; axis != 3; ++axis) {
    const double value = local[axis];

    if (value < -DEFORMED_SCREEN) {
      if (
        (faces & (1u << (2 * axis))) == 0u
        || value < -DEFORMED_SCREEN - probe.padCells
      ) {
        return false;
      }
    } else if (value > 1.0 + DEFORMED_SCREEN) {
      if (
        (faces & (1u << (2 * axis + 1))) == 0u
        || value > 1.0 + DEFORMED_SCREEN + probe.padCells
      ) {
        return false;
      }
    }
  }

  return true;
}

bool inside(
  const glm::dvec3& point,
  const glm::dvec3& low,
  const glm::dvec3& high,
  double margin
) {
  return point.x >= low.x - margin && point.y >= low.y - margin
         && point.z >= low.z - margin && point.x <= high.x + margin
         && point.y <= high.y + margin && point.z <= high.z + margin;
}

}

DeformedProbe deformedProbe(
  const talus::PreparedMesh& mesh,
  const talus::field::SampledSdf& field,
  double reach
) {
  DeformedProbe probe;

  probe.mesh = &mesh;
  probe.field = &field;
  probe.cell = static_cast<double>(glm::compMax(mesh.voxelSize));

  const double wanted =
    DEFORMED_PAD_SLACK * reach / static_cast<double>(glm::compMin(mesh.voxelSize));

  probe.padCells = wanted < DEFORMED_PAD
                     ? std::max(DEFORMED_PAD_FLOOR, wanted)
                     : DEFORMED_PAD;
  probe.pad = probe.padCells * probe.cell;

  const glm::dvec3 voxel(mesh.voxelSize);
  glm::dvec3 smallest(std::numeric_limits<double>::infinity());

  for (const glm::vec3& rest : mesh.restPositions) {
    smallest = glm::min(smallest, glm::dvec3(rest));
  }

  std::unordered_map<uint64_t, uint32_t> occupied;

  occupied.reserve(mesh.hexes.size());

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    occupied.emplace(
      cellKey(
        glm::dvec3(mesh.restPositions[mesh.hexes[element].nodes[0]]),
        smallest,
        voxel
      ),
      static_cast<uint32_t>(element)
    );
  }

  probe.boundary.assign(mesh.hexes.size(), 0u);

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const glm::dvec3 corner(mesh.restPositions[mesh.hexes[element].nodes[0]]);

    for (int axis = 0; axis != 3; ++axis) {
      for (int side = 0; side != 2; ++side) {
        glm::dvec3 neighbour = corner;

        neighbour[axis] += side == 0 ? -voxel[axis] : voxel[axis];

        if (!occupied.contains(cellKey(neighbour, smallest, voxel))) {
          probe.boundary[element] |=
            static_cast<uint8_t>(1u << (2 * axis + side));
        }
      }
    }
  }

  probe.cracked.assign(mesh.hexes.size(), 0u);

  for (size_t element = 0u;
       element != mesh.hexes.size() && element != mesh.crackFaces.size();
       ++element) {
    probe.cracked[element] = mesh.crackFaces[element];
    probe.boundary[element] |= mesh.crackFaces[element];
  }

  return probe;
}

void placeDeformedProbe(
  DeformedProbe& probe,
  std::span<const glm::dvec3> position
) {
  const talus::PreparedMesh& mesh = *probe.mesh;
  const size_t elements = mesh.hexes.size();

  probe.position.assign(position.begin(), position.end());
  probe.low.resize(elements);
  probe.high.resize(elements);
  probe.centre.resize(elements);
  probe.inverse.resize(elements);

  const std::array<glm::dvec3, 8> middle = hexShapeGradient(glm::dvec3(0.5));
  const glm::dvec3 voxel(mesh.voxelSize);
  const bool sized = probe.padCells < DEFORMED_PAD;
  double stretchSquared = 1.0;

  for (size_t element = 0u; element != elements; ++element) {
    glm::dvec3 low(std::numeric_limits<double>::infinity());
    glm::dvec3 high(-std::numeric_limits<double>::infinity());

    for (const uint32_t node : mesh.hexes[element].nodes) {
      low = glm::min(low, probe.position[node]);
      high = glm::max(high, probe.position[node]);
    }

    probe.low[element] = low;
    probe.high[element] = high;

    glm::dvec3 centre(0.0);
    glm::dmat3 jacobian(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::dvec3& at = probe.position[mesh.hexes[element].nodes[corner]];

      centre += 0.125 * at;
      jacobian += glm::outerProduct(at, middle[corner]);
    }

    const double determinant = glm::determinant(jacobian);

    probe.centre[element] = centre;
    probe.inverse[element] = determinant > 0.0 && std::isfinite(determinant)
                               ? glm::inverse(jacobian)
                               : glm::dmat3(0.0);

    if (sized && probe.boundary[element] != 0u) {
      for (int axis = 0; axis != 3; ++axis) {
        stretchSquared = std::max(
          stretchSquared,
          glm::dot(jacobian[axis], jacobian[axis]) / (voxel[axis] * voxel[axis])
        );
      }
    }
  }

  if (sized) {
    probe.pad = std::min(
      DEFORMED_PAD * probe.cell,
      probe.padCells * probe.cell * std::sqrt(stretchSquared)
    );
  }

  probe.minimum = glm::dvec3(std::numeric_limits<double>::infinity());
  probe.maximum = glm::dvec3(-std::numeric_limits<double>::infinity());

  for (size_t element = 0u; element != elements; ++element) {
    const double reach = probe.boundary[element] != 0u ? probe.pad : 0.0;

    probe.minimum = glm::min(probe.minimum, probe.low[element] - reach);
    probe.maximum = glm::max(probe.maximum, probe.high[element] + reach);
  }

  if (elements == 0u) {
    probe.cells = glm::uvec3(0u);
    probe.first.assign(1u, 0u);
    probe.members.clear();

    return;
  }

  const glm::dvec3 extent = (probe.maximum - probe.minimum) / probe.cell;

  probe.cells = glm::uvec3(glm::floor(extent)) + glm::uvec3(1u);

  const size_t total =
    static_cast<size_t>(probe.cells.x) * probe.cells.y * probe.cells.z;

  probe.first.assign(total + 1u, 0u);

  const auto cellOf = [&probe](const glm::dvec3& point) {
    const glm::dvec3 scaled = glm::floor((point - probe.minimum) / probe.cell);

    return glm::min(
      glm::uvec3(glm::max(scaled, glm::dvec3(0.0))),
      probe.cells - glm::uvec3(1u)
    );
  };

  const auto visit = [&](size_t element, const auto& action) {
    const double reach = probe.boundary[element] != 0u ? probe.pad : 0.0;
    const glm::uvec3 from = cellOf(probe.low[element] - reach);
    const glm::uvec3 to = cellOf(probe.high[element] + reach);

    for (uint32_t z = from.z; z <= to.z; ++z) {
      for (uint32_t y = from.y; y <= to.y; ++y) {
        for (uint32_t x = from.x; x <= to.x; ++x) {
          action(
            x + probe.cells.x * (static_cast<size_t>(y) + probe.cells.y * z)
          );
        }
      }
    }
  };

  for (size_t element = 0u; element != elements; ++element) {
    visit(element, [&probe](size_t index) { ++probe.first[index + 1u]; });
  }

  for (size_t index = 0u; index != total; ++index) {
    probe.first[index + 1u] += probe.first[index];
  }

  probe.members.resize(probe.first[total]);

  std::vector<uint32_t> cursor(probe.first.begin(), probe.first.end() - 1);

  for (size_t element = 0u; element != elements; ++element) {
    visit(element, [&](size_t index) {
      probe.members[cursor[index]++] = static_cast<uint32_t>(element);
    });
  }
}

DeformedDistance probeDeformed(
  const DeformedProbe& probe,
  const glm::dvec3& point,
  DeformedCounts* counts
) {
  if (
    probe.cells.x == 0u || !inside(point, probe.minimum, probe.maximum, 0.0)
  ) {
    return {};
  }

  const glm::uvec3 cell = glm::min(
    glm::uvec3(glm::floor((point - probe.minimum) / probe.cell)),
    probe.cells - glm::uvec3(1u)
  );
  const size_t index =
    cell.x
    + probe.cells.x * (static_cast<size_t>(cell.y) + probe.cells.y * cell.z);

  const auto count = [counts](const Inversion& found) {
    if (counts == nullptr) {
      return;
    }

    counts->unconverged += found.converged ? 0u : 1u;
    counts->inverted += found.inverted ? 1u : 0u;
  };

  const double margin = DEFORMED_TOLERANCE * probe.cell;
  const uint32_t begin = probe.first[index];
  const uint32_t end = probe.first[index + 1u];

  constexpr uint32_t KEPT = 16u;

  std::array<uint32_t, KEPT> keptElement{};
  std::array<Inversion, KEPT> keptInversion{};
  uint32_t kept = 0u;

  for (uint32_t at = begin; at != end; ++at) {
    const uint32_t element = probe.members[at];

    if (
      !inside(point, probe.low[element], probe.high[element], margin)
      || !screened(probe, element, point, 0u)
    ) {
      continue;
    }

    const Inversion found = invertHex(probe, element, point);

    count(found);

    if (kept != KEPT) {
      keptElement[kept] = element;
      keptInversion[kept] = found;
      ++kept;
    }

    if (!found.converged || found.inverted) {
      continue;
    }

    const glm::dvec3& local = found.local;

    if (
      local.x >= -DEFORMED_TOLERANCE && local.y >= -DEFORMED_TOLERANCE
      && local.z >= -DEFORMED_TOLERANCE && local.x <= 1.0 + DEFORMED_TOLERANCE
      && local.y <= 1.0 + DEFORMED_TOLERANCE
      && local.z <= 1.0 + DEFORMED_TOLERANCE
    ) {
      return evaluateHex(probe, element, local, false);
    }
  }

  bool accepted = false;
  uint32_t chosen = 0u;
  glm::dvec3 chosenLocal(0.0);
  double chosenExcess = std::numeric_limits<double>::infinity();

  for (uint32_t at = begin; at != end; ++at) {
    const uint32_t element = probe.members[at];
    const uint8_t faces = probe.boundary[element];

    if (
      faces == 0u
      || !inside(point, probe.low[element], probe.high[element], probe.pad)
      || !screened(probe, element, point, faces)
    ) {
      continue;
    }

    const auto held =
      std::find(keptElement.begin(), keptElement.begin() + kept, element);

    Inversion found;

    if (held != keptElement.begin() + kept) {
      found = keptInversion[static_cast<size_t>(held - keptElement.begin())];
    } else {
      found = invertHex(probe, element, point);

      count(found);
    }

    if (!found.converged || found.inverted) {
      continue;
    }

    bool across = true;
    double excess = 0.0;

    for (int axis = 0; axis != 3 && across; ++axis) {
      const double value = found.local[axis];

      if (value < -DEFORMED_TOLERANCE) {
        across = (faces & (1u << (2 * axis))) != 0u;
        excess = std::max(excess, -value);
      } else if (value > 1.0 + DEFORMED_TOLERANCE) {
        across = (faces & (1u << (2 * axis + 1))) != 0u;
        excess = std::max(excess, value - 1.0);
      }
    }

    if (across && excess < chosenExcess) {
      accepted = true;
      chosen = element;
      chosenLocal = found.local;
      chosenExcess = excess;
    }
  }

  if (!accepted) {
    return {};
  }

  return evaluateHex(probe, chosen, chosenLocal, true);
}

}
