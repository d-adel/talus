#include "worldray.h"

#include <talus/deformable/contact/contact.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace talus::dynamics {

namespace {

struct LocalHit {
  double fraction = 0.0;
  glm::dvec3 normal{0.0};
};

bool nearer(const WorldRayHit& a, const WorldRayHit& b) {
  if (a.fraction != b.fraction) {
    return a.fraction < b.fraction;
  }

  if (a.side != b.side) {
    return a.side < b.side;
  }

  if (a.body != b.body) {
    return a.body < b.body;
  }

  return a.child < b.child;
}

bool skipped(std::span<const RaySkip> skips, uint32_t body, uint32_t child) {
  return std::any_of(skips.begin(), skips.end(), [&](const RaySkip& skip) {
    return skip.body == body && skip.child == child;
  });
}

std::optional<LocalHit> sphereEntry(
  const glm::dvec3& centre,
  double radius,
  const glm::dvec3& from,
  const glm::dvec3& span
) {
  const double length = glm::dot(span, span);

  if (!(length > 0.0)) {
    return std::nullopt;
  }

  const glm::dvec3 offset = from - centre;
  const double closest = -glm::dot(offset, span) / length;
  const glm::dvec3 nearest = offset + closest * span;
  const double gap = radius * radius - glm::dot(nearest, nearest);

  if (gap < 0.0) {
    return std::nullopt;
  }

  const double fraction = closest - std::sqrt(gap / length);

  if (!(fraction > 0.0)) {
    return std::nullopt;
  }

  return LocalHit{fraction, (offset + fraction * span) / radius};
}

std::optional<LocalHit> boxEntry(
  const glm::dvec3& centre,
  const glm::dquat& rotation,
  const glm::dvec3& half,
  const glm::dvec3& from,
  const glm::dvec3& to
) {
  const glm::dquat back = glm::conjugate(rotation);
  const glm::dvec3 start = back * (from - centre);
  const glm::dvec3 span = back * (to - centre) - start;
  double enter = -std::numeric_limits<double>::infinity();
  double leave = std::numeric_limits<double>::infinity();
  int face = -1;

  for (int axis = 0; axis != 3; ++axis) {
    if (span[axis] == 0.0) {
      if (start[axis] < -half[axis] || start[axis] > half[axis]) {
        return std::nullopt;
      }

      continue;
    }

    const double first = (-half[axis] - start[axis]) / span[axis];
    const double second = (half[axis] - start[axis]) / span[axis];
    const double near = std::min(first, second);

    if (near > enter) {
      enter = near;
      face = axis;
    }

    leave = std::min(leave, std::max(first, second));
  }

  if (face < 0 || !(enter > 0.0) || enter > leave) {
    return std::nullopt;
  }

  glm::dvec3 normal(0.0);

  normal[face] = span[face] > 0.0 ? -1.0 : 1.0;

  return LocalHit{enter, rotation * normal};
}

constexpr double surfaceTolerance = 1.0e-9;
constexpr uint32_t bisectionLimit = 60u;
constexpr double fieldStepFloor = 0.05;
constexpr double latticeStepCells = 0.25;

std::optional<std::pair<double, double>> clipToBox(
  const glm::dvec3& start,
  const glm::dvec3& span,
  const glm::dvec3& low,
  const glm::dvec3& high,
  double reach
) {
  double enter = 0.0;
  double leave = reach;

  for (int axis = 0; axis != 3; ++axis) {
    if (span[axis] == 0.0) {
      if (start[axis] < low[axis] || start[axis] > high[axis]) {
        return std::nullopt;
      }

      continue;
    }

    const double first = (low[axis] - start[axis]) / span[axis];
    const double second = (high[axis] - start[axis]) / span[axis];

    enter = std::max(enter, std::min(first, second));
    leave = std::min(leave, std::max(first, second));
  }

  if (enter > leave) {
    return std::nullopt;
  }

  return std::make_pair(enter, leave);
}

template<typename Distance>
double bisect(Distance&& distance, double outside, double inside) {
  for (uint32_t pass = 0u; pass != bisectionLimit; ++pass) {
    const double middle = 0.5 * (outside + inside);
    const double gap = distance(middle);

    if (std::abs(gap) <= surfaceTolerance) {
      return middle;
    }

    if (gap <= 0.0) {
      inside = middle;
    } else {
      outside = middle;
    }
  }

  return inside;
}

std::optional<LocalHit> rayField(
  const field::SampledSdf& field,
  const glm::dvec3& start,
  const glm::dvec3& end,
  double reach
) {
  const glm::dvec3 span = end - start;
  const double length = glm::length(span);

  if (!(length > 0.0) || field.distances.empty()) {
    return std::nullopt;
  }

  const glm::dvec3 low(field.origin);
  const glm::dvec3 high = low
                          + glm::dvec3(field.voxelSize)
                              * glm::dvec3(field.resolution - glm::uvec3(1u));
  const std::optional<std::pair<double, double>> window =
    clipToBox(start, span, low, high, reach);

  if (!window) {
    return std::nullopt;
  }

  const glm::dvec3 direction = span / length;
  const auto distance = [&](double at) {
    return field::probe<double>(field, start + at * span).distance;
  };
  const auto normalAt = [&](double at) {
    return field::probe<double>(field, start + at * span).normal;
  };
  const double floor = fieldStepFloor
                       * static_cast<double>(std::min(
                         field.voxelSize.x,
                         std::min(field.voxelSize.y, field.voxelSize.z)
                       ));

  double at = window->first;

  if (distance(at) <= 0.0) {
    if (at > 0.0) {
      return LocalHit{at, normalAt(at)};
    }

    return std::nullopt;
  }

  while (at < window->second) {
    const double step = std::max(
      static_cast<double>(field::stepBound(
        field,
        glm::vec3(start + at * span),
        glm::vec3(direction)
      )),
      floor
    );
    const double next = std::min(at + step / length, window->second);

    if (distance(next) <= 0.0) {
      const double crossing = bisect(distance, at, next);

      return LocalHit{crossing, normalAt(crossing)};
    }

    at = next;
  }

  return std::nullopt;
}

std::optional<LocalHit> rayLattice(
  const LatticeBody& lattice,
  const glm::dvec3& from,
  const glm::dvec3& to,
  double reach
) {
  if (lattice.mesh == nullptr || lattice.field == nullptr) {
    return std::nullopt;
  }

  deformable::DeformedProbe probe =
    deformable::deformedProbe(*lattice.mesh, *lattice.field);

  deformable::placeDeformedProbe(probe, lattice.position);

  const glm::dvec3 span = to - from;
  const std::optional<std::pair<double, double>> window =
    clipToBox(from, span, probe.minimum, probe.maximum, reach);
  const double length = glm::length(span);

  if (!window || !(length > 0.0) || !(probe.cell > 0.0)) {
    return std::nullopt;
  }

  const auto distance = [&](double at) {
    const deformable::DeformedDistance found =
      deformable::probeDeformed(probe, from + at * span);

    return found.found && std::isfinite(found.distance)
             ? found.distance
             : std::numeric_limits<double>::infinity();
  };
  const auto normalAt = [&](double at) {
    return deformable::probeDeformed(probe, from + at * span).normal;
  };
  const double step = latticeStepCells * probe.cell / length;
  double at = window->first;

  if (distance(at) <= 0.0) {
    if (at > 0.0) {
      return LocalHit{at, normalAt(at)};
    }

    return std::nullopt;
  }

  while (at < window->second) {
    const double next = std::min(at + step, window->second);

    if (distance(next) <= 0.0) {
      const double crossing = bisect(distance, at, next);

      return LocalHit{crossing, normalAt(crossing)};
    }

    at = next;
  }

  return std::nullopt;
}

class Caster {
public:
  Caster(
    const World& world,
    const glm::dvec3& from,
    const glm::dvec3& to,
    std::span<const RaySkip> skips
  )
      : m_world(world), m_from(from), m_to(to), m_skips(skips) {}

  double visitMoving(uint32_t id) {
    const std::optional<BodyRef> ref = locateBody(m_world, id);

    if (ref && ref->side == Side::Rigid) {
      test(m_world.bodies.rigids[ref->index]);
    } else if (ref && ref->side == Side::Lattice) {
      testLattice(m_world.bodies.lattices[ref->index]);
    }

    return limit();
  }

  double visitStatic(uint32_t id) {
    const std::vector<StaticField>& statics = m_world.bodies.statics;
    const auto at = std::lower_bound(
      statics.begin(),
      statics.end(),
      id,
      [](const StaticField& body, uint32_t wanted) { return body.id < wanted; }
    );

    if (at != statics.end() && at->id == id) {
      testStatic(*at);
    }

    return limit();
  }

  void visitNew(const RigidBody& body) {
    test(body);
  }

  void visitNew(const LatticeBody& lattice) {
    testLattice(lattice);
  }

  void visitNew(const StaticField& body) {
    testStatic(body);
  }

  [[nodiscard]] std::optional<WorldRayHit> best() const {
    return m_best;
  }

private:
  double limit() const {
    return m_best ? m_best->fraction : 1.0;
  }

  void consider(
    Side side,
    uint32_t body,
    uint32_t child,
    const LocalHit& local
  ) {
    if (local.fraction > limit()) {
      return;
    }

    const WorldRayHit hit{
      side,
      body,
      child,
      m_from + local.fraction * (m_to - m_from),
      local.normal,
      local.fraction
    };

    if (!m_best || nearer(hit, *m_best)) {
      m_best = hit;
    }
  }

  void test(const RigidBody& body) {
    if (skipped(m_skips, body.id, noChild)) {
      return;
    }

    switch (body.shape) {
      case RigidShape::Sampled:
        testSampled(body);

        return;
      case RigidShape::Sphere:
        if (const std::optional<LocalHit> hit =
              sphereEntry(body.position, body.radius, m_from, m_to - m_from)) {
          consider(Side::Rigid, body.id, noChild, *hit);
        }

        return;
      case RigidShape::Box:
        if (const std::optional<LocalHit> hit = boxEntry(
              body.position,
              body.rotation,
              body.halfExtents,
              m_from,
              m_to
            )) {
          consider(Side::Rigid, body.id, noChild, *hit);
        }

        return;
      case RigidShape::Compound:
        if (body.compound != nullptr) {
          testCompound(body);
        }

        return;
    }
  }

  void testSampled(const RigidBody& body) {
    if (body.field == nullptr) {
      return;
    }

    const glm::dquat back = glm::conjugate(body.rotation);
    const glm::dvec3 start =
      body.fieldOrigin + body.fieldRotation * (back * (m_from - body.position));
    const glm::dvec3 end =
      body.fieldOrigin + body.fieldRotation * (back * (m_to - body.position));

    if (const std::optional<LocalHit> hit =
          rayField(*body.field, start, end, limit())) {
      consider(
        Side::Rigid,
        body.id,
        noChild,
        LocalHit{
          hit->fraction,
          body.rotation * (glm::conjugate(body.fieldRotation) * hit->normal)
        }
      );
    }
  }

  void testStatic(const StaticField& body) {
    if (body.field == nullptr) {
      return;
    }

    if (const std::optional<LocalHit> hit = rayField(
          *body.field,
          staticLocal(body, m_from),
          staticLocal(body, m_to),
          limit()
        )) {
      consider(
        Side::Static,
        body.id,
        noChild,
        LocalHit{hit->fraction, staticDirection(body, hit->normal)}
      );
    }
  }

  void testLattice(const LatticeBody& lattice) {
    if (skipped(m_skips, lattice.id, noChild)) {
      return;
    }

    if (const std::optional<LocalHit> hit =
          rayLattice(lattice, m_from, m_to, limit())) {
      consider(Side::Lattice, lattice.id, noChild, *hit);
    }
  }

  void testCompound(const RigidBody& body) {
    const CompoundShape& shape = *body.compound;

    if (shape.nodes.empty()) {
      return;
    }

    const glm::dquat back = glm::conjugate(body.rotation);
    const glm::dvec3 start = back * (m_from - body.position);
    const glm::dvec3 end = back * (m_to - body.position);
    const glm::dvec3 span = end - start;
    std::array<uint32_t, 64> stack{};
    uint32_t top = 0u;

    stack[top++] = 0u;

    while (top != 0u) {
      const ChildNode& node = shape.nodes[stack[--top]];

      if (!segmentMeets(node.bounds, start, span, limit())) {
        continue;
      }

      if (node.left != 0u) {
        stack[top++] = node.right;
        stack[top++] = node.left;
        continue;
      }

      for (uint32_t at = node.first; at != node.first + node.count; ++at) {
        const CompoundChild& child = shape.children[shape.leaves[at]];

        if (skipped(m_skips, body.id, child.key)) {
          continue;
        }

        if (const std::optional<LocalHit> hit =
              boxEntry(child.centre, child.rotation, child.half, start, end)) {
          consider(
            Side::Rigid,
            body.id,
            child.key,
            LocalHit{hit->fraction, body.rotation * hit->normal}
          );
        }
      }
    }
  }

  const World& m_world;
  glm::dvec3 m_from;
  glm::dvec3 m_to;
  std::span<const RaySkip> m_skips;
  std::optional<WorldRayHit> m_best;
};

}

std::optional<WorldRayHit> castRay(
  const World& world,
  const glm::dvec3& from,
  const glm::dvec3& to,
  std::span<const RaySkip> skips
) {
  Caster caster(world, from, to, skips);
  const BroadPhase& broadPhase = world.broadPhase;

  broadPhase.rayQuery(from, to, [&](bool fixed, uint32_t id, double) {
    return fixed ? caster.visitStatic(id) : caster.visitMoving(id);
  });

  const uint32_t seen = broadPhase.seenBodies();

  if (world.bodies.nextId > seen) {
    for (const RigidBody& body : world.bodies.rigids) {
      if (body.id >= seen) {
        caster.visitNew(body);
      }
    }

    for (const LatticeBody& lattice : world.bodies.lattices) {
      if (lattice.id >= seen) {
        caster.visitNew(lattice);
      }
    }
  }

  const uint32_t seenStatics = broadPhase.seenStatics();

  if (world.bodies.nextStaticId > seenStatics) {
    for (const StaticField& body : world.bodies.statics) {
      if (body.id >= seenStatics) {
        caster.visitNew(body);
      }
    }
  }

  return caster.best();
}

}
