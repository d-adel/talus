#include "rays.h"

#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/compound/compound.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <limits>
#include <stdexcept>

namespace talus::dynamics {

namespace {

struct Slab {
  double enter = -std::numeric_limits<double>::infinity();
  double leave = std::numeric_limits<double>::infinity();
};

Slab slabOf(const glm::dvec3& from, const glm::dvec3& span, const glm::dvec3& low, const glm::dvec3& high) {
  Slab slab;

  for (int axis = 0; axis != 3; ++axis) {
    if (span[axis] == 0.0) {
      if (from[axis] < low[axis] || from[axis] > high[axis]) {
        return Slab{1.0, 0.0};
      }

      continue;
    }

    const double first = (low[axis] - from[axis]) / span[axis];
    const double second = (high[axis] - from[axis]) / span[axis];

    slab.enter = std::max(slab.enter, std::min(first, second));
    slab.leave = std::min(slab.leave, std::max(first, second));
  }

  return slab;
}

bool meets(const Slab& slab) {
  return slab.enter <= slab.leave && slab.leave >= 0.0 && slab.enter <= 1.0;
}

std::optional<double> boxEntry(
  const glm::dvec3& centre,
  const glm::dquat& rotation,
  const glm::dvec3& half,
  const glm::dvec3& from,
  const glm::dvec3& to
) {
  const glm::dquat back = glm::conjugate(rotation);
  const glm::dvec3 start = back * (from - centre);
  const glm::dvec3 span = back * (to - centre) - start;
  const Slab slab = slabOf(start, span, -half, half);

  if (slab.enter > 0.0 && slab.enter <= 1.0 && slab.enter <= slab.leave) {
    return slab.enter;
  }

  return std::nullopt;
}

bool nearer(const RayHit& a, const RayHit& b) {
  if (a.fraction != b.fraction) {
    return a.fraction < b.fraction;
  }

  if (a.body != b.body) {
    return a.body < b.body;
  }

  return a.child < b.child;
}

}

std::vector<uint32_t> rayBlockers(const BodySet& bodies, const Bounds& region) {
  std::vector<uint32_t> blockers;

  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    const RigidBody& body = bodies.rigids[index];
    const bool compoundBody = body.shape == RigidShape::Compound && body.compound != nullptr;
    const bool fixedBox = body.shape == RigidShape::Box && !(body.mass > 0.0);

    if ((compoundBody || fixedBox) && overlaps(boundsOf(bodies, BodyRef{Side::Rigid, index}, 0.0), region)) {
      blockers.push_back(index);
    }
  }

  return blockers;
}

std::optional<RayHit> firstHit(
  const BodySet& bodies,
  std::span<const uint32_t> blockers,
  const glm::dvec3& from,
  const glm::dvec3& to,
  const RaySkip& skip
) {
  std::optional<RayHit> best;
  const auto consider = [&best](const RayHit& hit) {
    if (!best.has_value() || nearer(hit, *best)) {
      best = hit;
    }
  };

  for (const uint32_t index : blockers) {
    if (index >= bodies.rigids.size()) {
      throw std::out_of_range("A ray blocker names a body the world does not hold");
    }

    const RigidBody& body = bodies.rigids[index];

    if (body.shape != RigidShape::Compound || body.compound == nullptr) {
      if (body.id == skip.body && skip.child == noChild) {
        continue;
      }

      if (const std::optional<double> fraction = boxEntry(body.position, body.rotation, body.halfExtents, from, to)) {
        consider(RayHit{body.id, noChild, *fraction});
      }

      continue;
    }

    const CompoundShape& shape = *body.compound;

    if (shape.nodes.empty()) {
      continue;
    }

    const glm::dquat back = glm::conjugate(body.rotation);
    const glm::dvec3 start = back * (from - body.position);
    const glm::dvec3 end = back * (to - body.position);
    const glm::dvec3 span = end - start;
    std::array<uint32_t, 64> stack{};
    uint32_t top = 0u;

    stack[top++] = 0u;

    while (top != 0u) {
      const ChildNode& node = shape.nodes[stack[--top]];

      if (!meets(slabOf(start, span, node.bounds.low, node.bounds.high))) {
        continue;
      }

      if (node.left == 0u) {
        for (uint32_t at = node.first; at != node.first + node.count; ++at) {
          const CompoundChild& child = shape.children[shape.leaves[at]];

          if (body.id == skip.body && child.key == skip.child) {
            continue;
          }

          if (const std::optional<double> fraction = boxEntry(child.centre, child.rotation, child.half, start, end)) {
            consider(RayHit{body.id, child.key, *fraction});
          }
        }
      } else {
        stack[top++] = node.right;
        stack[top++] = node.left;
      }
    }
  }

  return best;
}

}
