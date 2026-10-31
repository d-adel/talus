#include "worldray.h"

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/tiers/tiers.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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
    }

    return limit();
  }

  void visitNew(const RigidBody& body) {
    test(body);
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
        return;
      case RigidShape::Sphere:
        if (
          const std::optional<LocalHit> hit =
            sphereEntry(body.position, body.radius, m_from, m_to - m_from)
        ) {
          consider(Side::Rigid, body.id, noChild, *hit);
        }

        return;
      case RigidShape::Box:
        if (
          const std::optional<LocalHit> hit = boxEntry(
            body.position,
            body.rotation,
            body.halfExtents,
            m_from,
            m_to
          )
        ) {
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

        if (
          const std::optional<LocalHit> hit =
            boxEntry(child.centre, child.rotation, child.half, start, end)
        ) {
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

  broadPhase.rayQuery(from, to, [&](bool fixed, uint32_t id, double clip) {
    return fixed ? clip : caster.visitMoving(id);
  });

  const uint32_t seen = broadPhase.seenBodies();

  if (world.bodies.nextId > seen) {
    for (const RigidBody& body : world.bodies.rigids) {
      if (body.id >= seen) {
        caster.visitNew(body);
      }
    }
  }

  return caster.best();
}

}
