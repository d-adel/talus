#include "contact.ih"

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

struct Triangle {
  std::array<glm::vec3, 3> corner{};
};

glm::vec3 barycentricOf(const Triangle& triangle, const glm::vec3& point) {
  const glm::vec3& a = triangle.corner[0];
  const glm::vec3& b = triangle.corner[1];
  const glm::vec3& c = triangle.corner[2];

  const glm::vec3 ab = b - a;
  const glm::vec3 ac = c - a;
  const glm::vec3 ap = point - a;

  const float d1 = glm::dot(ab, ap);
  const float d2 = glm::dot(ac, ap);

  if (d1 <= 0.0f && d2 <= 0.0f) {
    return glm::vec3(1.0f, 0.0f, 0.0f);
  }

  const glm::vec3 bp = point - b;
  const float d3 = glm::dot(ab, bp);
  const float d4 = glm::dot(ac, bp);

  if (d3 >= 0.0f && d4 <= d3) {
    return glm::vec3(0.0f, 1.0f, 0.0f);
  }

  const float vc = d1 * d4 - d3 * d2;

  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);

    return glm::vec3(1.0f - v, v, 0.0f);
  }

  const glm::vec3 cp = point - c;
  const float d5 = glm::dot(ab, cp);
  const float d6 = glm::dot(ac, cp);

  if (d6 >= 0.0f && d5 <= d6) {
    return glm::vec3(0.0f, 0.0f, 1.0f);
  }

  const float vb = d5 * d2 - d1 * d6;

  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);

    return glm::vec3(1.0f - w, 0.0f, w);
  }

  const float va = d3 * d6 - d5 * d4;

  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

    return glm::vec3(0.0f, 1.0f - w, w);
  }

  const float denominator = 1.0f / (va + vb + vc);
  const float v = vb * denominator;
  const float w = vc * denominator;

  return glm::vec3(1.0f - v - w, v, w);
}

glm::vec3 pointAt(const Triangle& triangle, const glm::vec3& weights) {
  return weights.x * triangle.corner[0] + weights.y * triangle.corner[1]
         + weights.z * triangle.corner[2];
}

float longestEdge(const Triangle& triangle) {
  return glm::max(
    glm::length(triangle.corner[1] - triangle.corner[0]),
    glm::max(
      glm::length(triangle.corner[2] - triangle.corner[1]),
      glm::length(triangle.corner[0] - triangle.corner[2])
    )
  );
}

struct Closest {
  glm::vec3 weights{0.0f};
  glm::vec3 point{0.0f};
  ShapeDistance probe;
};

Closest closestOnTriangle(
  const ShapeProbe& shape,
  const ShapeAt& at,
  const Triangle& triangle
) {
  constexpr std::array<glm::vec3, 7> STARTS{
    glm::vec3(1.0f, 0.0f, 0.0f),
    glm::vec3(0.0f, 1.0f, 0.0f),
    glm::vec3(0.0f, 0.0f, 1.0f),
    glm::vec3(0.5f, 0.5f, 0.0f),
    glm::vec3(0.0f, 0.5f, 0.5f),
    glm::vec3(0.5f, 0.0f, 0.5f),
    glm::vec3(1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f)
  };

  Closest best;
  best.probe.distance = std::numeric_limits<float>::max();

  for (const glm::vec3& weights : STARTS) {
    Closest candidate;
    candidate.weights = weights;
    candidate.point = pointAt(triangle, weights);
    candidate.probe = probeShape(shape, at, candidate.point);

    if (candidate.probe.distance < best.probe.distance) {
      best = candidate;
    }
  }

  const glm::vec3 across = glm::cross(
    triangle.corner[1] - triangle.corner[0],
    triangle.corner[2] - triangle.corner[0]
  );
  const float area = glm::length(across);

  if (area <= 0.0f) {
    return best;
  }

  const glm::vec3 facing = across / area;
  const float edge = longestEdge(triangle);

  for (uint32_t iteration = 0u; iteration != DESCENT_ITERATIONS; ++iteration) {
    const glm::vec3 gradient = best.probe.normal;
    const glm::vec3 slide = -(gradient - facing * glm::dot(facing, gradient));
    const float length = glm::length(slide);

    if (length <= 1.0e-4f) {
      break;
    }

    const glm::vec3 direction = slide / length;

    float stride =
      glm::clamp(std::abs(best.probe.distance), 0.05f * edge, 0.5f * edge);

    bool improved = false;

    for (uint32_t attempt = 0u; attempt != BACKTRACKS; ++attempt) {
      Closest candidate;
      candidate.weights =
        barycentricOf(triangle, best.point + direction * stride);
      candidate.point = pointAt(triangle, candidate.weights);
      candidate.probe = probeShape(shape, at, candidate.point);

      if (candidate.probe.distance < best.probe.distance) {
        best = candidate;
        improved = true;

        break;
      }

      stride *= 0.5f;
    }

    if (!improved) {
      break;
    }
  }

  return best;
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

bool advanceToImpact(
  const ShapeProbe& shape,
  const MovingTriangle& moving,
  float deltaTime,
  const ContactGaps& gaps,
  float radius,
  Closest& closest,
  float& impactTime
) {
  const float bound = moving.speed() + glm::length(shape.linearVelocity)
                      + glm::length(shape.angularVelocity) * radius;

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

std::vector<SurfaceContact> findContacts(
  const objectmodel::PreparedMesh& mesh,
  const objectmodel::DeformableState& state,
  std::span<const objectmodel::PhysicalMaterial> materials,
  const glm::vec3& gravity,
  std::span<const ShapeProbe> shapes,
  float deltaTime,
  const ContactGaps& gaps
) {
  std::vector<SurfaceContact> contacts;

  const auto frictionOf = [&](uint32_t lane) {
    static const objectmodel::PhysicalMaterial DEFAULT{};

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

  for (size_t index = 0u; index != shapes.size(); ++index) {
    const ShapeProbe& shape = shapes[index];

    if (shape.field == nullptr) {
      continue;
    }

    const Bounds swept = sweptShapeBounds(shape, deltaTime);
    const float radius = shapeRadius(shape);
    const size_t first = contacts.size();

    for (size_t face = 0u; face != mesh.surfaceTriangles.size(); ++face) {
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

      const ShapeAt origin = advanceShape(shape, 0.0f);

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

}
