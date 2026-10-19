#include "events.h"

#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/sleep/sleep.h>

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <unordered_map>

namespace talus::dynamics {

namespace {

uint32_t idOf(const BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Rigid) {
    return bodies.rigids[ref.index].id;
  }

  if (ref.side == Side::Lattice) {
    return bodies.lattices[ref.index].id;
  }

  return bodies.statics[ref.index].id;
}

uint32_t idOf(const BodySet& bodies, const ContactSide& side) {
  return idOf(bodies, BodyRef{side.side, side.body});
}

glm::dvec3 cachedPoint(
  const BodySet& bodies,
  const BodyRef& a,
  const BodyRef& b,
  const CachedManifoldPoint& point,
  double separation,
  const glm::dvec3& normal
) {
  if (a.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[a.index];

    return body.position + glm::mat3_cast(body.rotation) * point.localA
           - 0.5 * separation * normal;
  }

  const RigidBody& body = bodies.rigids[b.index];

  return body.position + glm::mat3_cast(body.rotation) * point.localB
         + 0.5 * separation * normal;
}

glm::dvec3 startVelocityAt(
  const BodySet& bodies,
  const StartVelocities& start,
  const ContactSide& side,
  const glm::dvec3& point
) {
  if (side.side == Side::Lattice) {
    const std::vector<glm::dvec3>& nodes = start.lattice[side.body];
    glm::dvec3 velocity(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      velocity += side.weights[corner] * nodes[side.nodes[corner]];
    }

    return velocity;
  }

  if (side.side == Side::Rigid) {
    return start.linear[side.body]
           + glm::cross(
             start.angular[side.body],
             point - bodies.rigids[side.body].position
           );
  }

  return glm::dvec3(0.0);
}

}

StartVelocities startVelocities(const BodySet& bodies) {
  StartVelocities start;

  start.linear.reserve(bodies.rigids.size());
  start.angular.reserve(bodies.rigids.size());
  start.lattice.reserve(bodies.lattices.size());

  for (const RigidBody& body : bodies.rigids) {
    start.linear.push_back(body.velocity);
    start.angular.push_back(body.angularVelocity);
  }

  for (const LatticeBody& body : bodies.lattices) {
    start.lattice.push_back(body.velocity);
  }

  return start;
}

std::vector<ContactHitEvent> gatherHits(
  const World& world,
  const StartVelocities& start,
  std::span<const uint8_t> latticeOwned,
  std::span<const uint32_t> candidates
) {
  const BodySet& bodies = world.bodies;
  const double threshold = world.parameters.hitEventThreshold;
  std::vector<ContactHitEvent> hits;
  std::vector<uint32_t> slots(candidates.begin(), candidates.end());
  const auto owned = [&](const BodyRef& ref) {
    return ref.side == Side::Rigid && ref.index < latticeOwned.size() && latticeOwned[ref.index] != 0u;
  };

  if (std::any_of(latticeOwned.begin(), latticeOwned.end(), [](uint8_t value) { return value != 0u; })) {
    std::vector<uint32_t> touching;

    world.manifolds.touchingPairs(touching);

    for (const uint32_t slot : touching) {
      const CachedBodyPair& pair = world.manifolds.at(slot);

      if (owned(pair.first) || owned(pair.second)) {
        slots.push_back(slot);
      }
    }
  }

  std::sort(slots.begin(), slots.end());
  slots.erase(std::unique(slots.begin(), slots.end()), slots.end());

  for (const uint32_t slot : slots) {
    const CachedBodyPair& pair = world.manifolds.at(slot);
    ContactHitEvent hit;
    bool strongest = false;
    double impulse = 0.0;

    for (const CachedManifold& manifold : pair.manifolds) {
      const BodyRef& endA = manifold.flipped ? pair.second : pair.first;
      const BodyRef& endB = manifold.flipped ? pair.first : pair.second;

      for (uint32_t index = manifold.start; index != manifold.start + manifold.count; ++index) {
        const CachedContactPoint& point = pair.points[index];

        impulse += point.memory.impulse;

        if (point.memory.approach > std::max(threshold, hit.approachSpeed)
            && point.memory.impulse > 0.0) {
          hit.firstSide = endA.side;
          hit.first = idOf(bodies, endA);
          hit.secondSide = endB.side;
          hit.second = idOf(bodies, endB);
          hit.point = cachedPoint(bodies, endA, endB, pair.manifoldPoints[index], point.separation, manifold.basis[0]);
          hit.normal = -manifold.basis[0];
          hit.approachSpeed = point.memory.approach;
          strongest = true;
        }
      }
    }

    if (strongest) {
      hit.impulse = impulse;
      hits.push_back(hit);
    }
  }

  if (start.linear.size() != bodies.rigids.size()
      || start.lattice.size() != bodies.lattices.size()) {
    return hits;
  }

  std::unordered_map<uint64_t, ContactHitEvent> strongest;
  std::unordered_map<uint64_t, double> total;

  for (const Contact& contact : world.contacts) {
    const glm::dvec3 point = sidePoint(bodies, contact.a);
    const glm::dvec3 normal = -contact.basis[0];
    const double approach = glm::dot(
      startVelocityAt(bodies, start, contact.a, point)
        - startVelocityAt(bodies, start, contact.b, point),
      normal
    );

    total[contact.pair] += contact.memory.impulse;

    if (!(approach > threshold && contact.memory.impulse > 0.0)) {
      continue;
    }

    const auto found = strongest.find(contact.pair);

    if (found != strongest.end() && found->second.approachSpeed >= approach) {
      continue;
    }

    ContactHitEvent hit;

    hit.firstSide = contact.a.side;
    hit.first = idOf(bodies, contact.a);
    hit.secondSide = contact.b.side;
    hit.second = idOf(bodies, contact.b);
    hit.point = point;
    hit.normal = normal;
    hit.approachSpeed = approach;
    strongest[contact.pair] = hit;
  }

  for (auto& [pair, hit] : strongest) {
    hit.impulse = total[pair];
    hits.push_back(hit);
  }

  return hits;
}

}
