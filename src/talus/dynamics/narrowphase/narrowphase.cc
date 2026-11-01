#include "narrowphase.h"

#include <talus/deformable/contact/contact.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/manifold/manifold.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/pipeline/pipeline.h>

#if defined(_M_X64) || defined(__x86_64__)
#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <x86intrin.h>
#endif
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

constexpr double recycleDistance = 0.05;
constexpr double recycleAngle = 0.99240388;

uint64_t ticks() {
#if defined(_M_X64) || defined(__x86_64__)
  return __rdtsc();
#else
  return static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
#endif
}

struct PairTicks {
  uint64_t recycle = 0u;
  uint64_t midphase = 0u;
  uint64_t narrowphase = 0u;
  uint64_t reduce = 0u;
  uint64_t build = 0u;
  uint32_t tested = 0u;
  uint32_t recycled = 0u;
  uint32_t childPairs = 0u;
  uint32_t redone = 0u;
};

struct PairClock {
  PairTicks* tally = nullptr;
  uint64_t last = 0u;

  void start() {
    if (tally != nullptr) {
      last = ticks();
    }
  }

  void charge(uint64_t PairTicks::*phase) {
    if (tally != nullptr) {
      const uint64_t now = ticks();

      tally->*phase += now - last;
      last = now;
    }
  }
};

void tangents(
  const glm::dvec3& normal,
  glm::dvec3& first,
  glm::dvec3& second
) {
  const glm::dvec3 helper = std::abs(normal.x) < 0.9
                              ? glm::dvec3(1.0, 0.0, 0.0)
                              : glm::dvec3(0.0, 1.0, 0.0);

  first = glm::normalize(glm::cross(normal, helper));
  second = glm::cross(normal, first);
}

uint32_t keyIndex(const BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Rigid) {
    return bodies.rigids[ref.index].id;
  }

  if (ref.side == Side::Lattice) {
    return bodies.lattices[ref.index].id;
  }

  return ref.index;
}

uint64_t contactKey(
  const BodySet& bodies,
  const BodyRef& points,
  const BodyRef& field,
  uint32_t point
) {
  return static_cast<uint64_t>(points.side) << 62u
         | static_cast<uint64_t>(field.side) << 60u
         | (static_cast<uint64_t>(keyIndex(bodies, points)) & 0x1FFFFFull) << 39u
         | (static_cast<uint64_t>(keyIndex(bodies, field)) & 0x1FFFFFull) << 18u
         | (static_cast<uint64_t>(point) & 0x3FFFFull);
}

struct Hit {
  double gap = 0.0;
  glm::dvec3 normal{0.0};
};

bool fieldHit(
  const field::SampledSdf& field,
  const glm::dvec3& point,
  double band,
  Hit& hit
) {
  const field::FieldProbeOf<double> probed = field::probe<double>(field, point);
  const double slope = glm::length(probed.gradient);

  if (!std::isfinite(probed.distance) || !(slope >= 1.0e-6)) {
    return false;
  }

  hit.gap = probed.distance / slope;
  hit.normal = probed.gradient / slope;

  return hit.gap < band;
}

bool primitive(const RigidBody& body) {
  return body.shape == RigidShape::Sphere || body.shape == RigidShape::Box;
}

bool compound(const RigidBody& body) {
  return body.shape == RigidShape::Compound;
}

bool movable(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid && bodies.rigids[ref.index].mass > 0.0;
}

bool primitiveHit(
  const RigidBody& target,
  const glm::dvec3& local,
  const glm::dvec3& toward,
  double band,
  Hit& hit
) {
  if (target.shape == RigidShape::Sphere) {
    const double distance = glm::length(local);

    hit.normal = distance > 0.0 ? local / distance : glm::dvec3(0.0, 1.0, 0.0);
    hit.gap = distance - target.radius;

    return hit.gap < band;
  }

  const glm::dvec3 half = target.halfExtents;
  const double tie = 0.01 * std::min({half.x, half.y, half.z});
  const glm::dvec3 offset = local - glm::clamp(local, -half, half);
  const double distance = glm::length(offset);

  if (distance > tie) {
    hit.normal = offset / distance;
    hit.gap = distance;

    return hit.gap < band;
  }

  double bestDepth = std::numeric_limits<double>::infinity();
  double bestFacing = -std::numeric_limits<double>::infinity();

  for (int axis = 0; axis != 3; ++axis) {
    for (const double sign : {-1.0, 1.0}) {
      const double depth = half[axis] - sign * local[axis];
      const double facing = sign * toward[axis];

      if (depth < bestDepth - tie
          || (depth <= bestDepth + tie && facing > bestFacing)) {
        bestDepth = depth;
        bestFacing = facing;
        hit.normal = glm::dvec3(0.0);
        hit.normal[axis] = sign;
      }
    }
  }

  hit.gap = -bestDepth;

  return hit.gap < band;
}

void addContact(
  std::vector<Contact>& contacts,
  const ContactSide& a,
  const ContactSide& b,
  const glm::dvec3& normal,
  double gap,
  double friction,
  uint64_t key
) {
  Contact contact;

  contact.a = a;
  contact.b = b;
  contact.basis[0] = normal;
  tangents(normal, contact.basis[1], contact.basis[2]);
  contact.gap = gap;
  contact.friction = friction;
  contact.key = key;
  contacts.push_back(contact);
}

ContactSide seedSide(const LatticeBody& body, uint32_t lattice, uint32_t seed) {
  ContactSide side;
  const talus::PreparedMesh& mesh = *body.mesh;
  const talus::PreparedHex& hex = mesh.hexes[mesh.surfaceSeedHexes[seed]];

  side.side = Side::Lattice;
  side.body = lattice;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    side.nodes[corner] = hex.nodes[corner];
    side.weights[corner] = mesh.surfaceSeedShapes[seed][corner];
  }

  return side;
}

void latticeSources(
  const LatticeBody& body,
  uint32_t lattice,
  std::vector<std::pair<ContactSide, glm::dvec3>>& sources
) {
  const talus::PreparedMesh& mesh = *body.mesh;
  std::vector<uint8_t> seeded(mesh.restPositions.size(), 0u);

  for (uint32_t seed = 0u; seed != mesh.surfaceSeeds.size(); ++seed) {
    const ContactSide side = seedSide(body, lattice, seed);
    glm::dvec3 world(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      world += side.weights[corner] * body.position[side.nodes[corner]];
      seeded[side.nodes[corner]] = 1u;
    }

    sources.emplace_back(side, world);
  }

  for (const uint32_t node : mesh.surfaceNodes) {
    if (seeded[node] != 0u) {
      continue;
    }

    ContactSide side;

    side.side = Side::Lattice;
    side.body = lattice;
    side.nodes.fill(node);
    side.weights.fill(0.0);
    side.weights[0] = 1.0;
    sources.emplace_back(side, body.position[node]);
  }
}

double restitutionOf(const BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Rigid) {
    return bodies.rigids[ref.index].restitution;
  }

  if (ref.side == Side::Static) {
    return bodies.statics[ref.index].restitution;
  }

  return 0.0;
}

double frictionOf(const BodySet& bodies, const BodyRef& ref) {
  if (ref.side == Side::Lattice) {
    return bodies.lattices[ref.index].friction;
  }

  if (ref.side == Side::Rigid) {
    return bodies.rigids[ref.index].friction;
  }

  return bodies.statics[ref.index].friction;
}

void pointsAgainst(
  const BodySet& bodies,
  const BodyRef& points,
  const BodyRef& field,
  double band,
  std::vector<Contact>& contacts
) {
  const double friction =
    std::sqrt(frictionOf(bodies, points) * frictionOf(bodies, field));
  std::vector<std::pair<ContactSide, glm::dvec3>> sources;

  if (points.side == Side::Lattice) {
    latticeSources(bodies.lattices[points.index], points.index, sources);
  } else {
    const RigidBody& body = bodies.rigids[points.index];
    const glm::dmat3 turn = glm::mat3_cast(body.rotation);

    for (const glm::dvec3& sample : body.samples) {
      ContactSide side;

      side.side = Side::Rigid;
      side.body = points.index;
      side.local = sample;
      sources.emplace_back(side, body.position + turn * sample);
    }

    if (body.radius > 0.0) {
      ContactSide side;

      side.side = Side::Rigid;
      side.body = points.index;
      sources.emplace_back(side, body.position);
    }
  }

  const double radius =
    points.side == Side::Rigid ? bodies.rigids[points.index].radius : 0.0;
  glm::dvec3 middle(0.0);

  if (points.side == Side::Lattice) {
    const LatticeBody& lattice = bodies.lattices[points.index];

    for (const glm::dvec3& position : lattice.position) {
      middle += position;
    }

    middle /=
      static_cast<double>(std::max<size_t>(lattice.position.size(), 1u));
  }

  for (uint32_t index = 0u; index != sources.size(); ++index) {
    const glm::dvec3& point = sources[index].second;
    const bool centre = radius > 0.0 && index + 1u == sources.size();
    const double inflate = centre ? radius : 0.0;
    Hit hit;
    ContactSide other;

    if (field.side == Side::Static) {
      const StaticField& fixed = bodies.statics[field.index];

      if (!fieldHit(
            *fixed.field,
            staticLocal(fixed, point),
            band + inflate,
            hit
          )) {
        continue;
      }

      hit.normal = staticDirection(fixed, hit.normal);
      other.side = Side::Static;
      other.body = field.index;
    } else {
      const RigidBody& target = bodies.rigids[field.index];
      const glm::dmat3 turn = glm::mat3_cast(target.rotation);
      const glm::dmat3 frame = glm::mat3_cast(target.fieldRotation);
      const glm::dvec3 local =
        glm::transpose(turn) * (point - target.position);

      const bool exact = points.side == Side::Lattice && primitive(target);
      const bool found =
        exact ? primitiveHit(
                  target,
                  local,
                  glm::transpose(turn) * (middle - target.position),
                  band + inflate,
                  hit
                )
              : fieldHit(
                  *target.field,
                  target.fieldOrigin + frame * local,
                  band + inflate,
                  hit
                );

      if (!found) {
        continue;
      }

      if (!exact) {
        hit.normal = glm::transpose(frame) * hit.normal;
      }

      other.side = Side::Rigid;
      other.body = field.index;
      other.local = local - hit.gap * hit.normal;
      hit.normal = turn * hit.normal;
    }

    if (centre) {
      const RigidBody& body = bodies.rigids[points.index];

      hit.gap -= radius;
      sources[index].first.local =
        glm::transpose(glm::mat3_cast(body.rotation)) * (-radius * hit.normal);
    }

    addContact(
      contacts,
      sources[index].first,
      other,
      hit.normal,
      hit.gap,
      friction,
      contactKey(bodies, points, field, index)
    );

    if (points.side != Side::Lattice && field.side != Side::Lattice) {
      contacts.back().restitution = std::max(
        restitutionOf(bodies, points),
        restitutionOf(bodies, field)
      );
    }
  }
}


Manifold collidePair(const RigidBody& a, const RigidBody& b, double band) {
  const BoxPose boxA{a.position, a.rotation, a.halfExtents};
  const BoxPose boxB{b.position, b.rotation, b.halfExtents};

  if (a.shape == RigidShape::Sphere && b.shape == RigidShape::Sphere) {
    return collideSpheres(a.position, a.radius, b.position, b.radius, band);
  }

  if (a.shape == RigidShape::Sphere) {
    return collideBoxAndSphere(a.position, a.radius, boxB, band);
  }

  if (b.shape == RigidShape::Sphere) {
    Manifold flipped = collideBoxAndSphere(b.position, b.radius, boxA, band);

    flipped.normal = -flipped.normal;

    return flipped;
  }

  return collideBoxes(boxA, boxB, band);
}

CachedPose poseOf(const RigidBody& a, const RigidBody& b, const glm::dmat3& turnA) {
  CachedPose pose;

  pose.rotationA = a.rotation;
  pose.rotationB = b.rotation;
  pose.relativeRotation = glm::conjugate(a.rotation) * b.rotation;
  pose.relativePosition = glm::transpose(turnA) * (b.position - a.position);
  pose.valid = true;

  return pose;
}

CachedPose poseOf(const RigidBody& a, const RigidBody& b) {
  return poseOf(a, b, glm::mat3_cast(a.rotation));
}

glm::dvec3 extentOf(const RigidBody& body) {
  if (body.shape == RigidShape::Box) {
    return body.halfExtents;
  }

  if (body.shape == RigidShape::Compound) {
    const Bounds& local = body.compound->bounds;

    return glm::max(glm::dvec3(body.radius), glm::max(glm::abs(local.low), glm::abs(local.high)));
  }

  glm::dvec3 extent(body.radius);

  for (const glm::dvec3& sample : body.samples) {
    extent = glm::max(extent, glm::abs(sample));
  }

  return extent;
}

glm::dvec3 turningExtentOf(const RigidBody& body, const glm::dquat& built) {
  return body.mass > 0.0 || body.rotation != built ? extentOf(body)
                                                    : glm::dvec3(0.0);
}

CachedPose staticPoseOf(const RigidBody& body) {
  CachedPose pose;

  pose.rotationA = body.rotation;
  pose.relativeRotation = body.rotation;
  pose.relativePosition = body.position;
  pose.valid = true;

  return pose;
}

uint32_t groupByNormal(
  const BodySet& bodies,
  std::vector<Contact>& contacts,
  size_t begin,
  double window,
  double spacing
) {
  thread_local std::vector<glm::dvec3> normals;
  thread_local std::vector<double> nearest;
  thread_local std::vector<uint8_t> keep;
  thread_local std::vector<size_t> members;
  thread_local std::vector<glm::dvec3> points;
  thread_local std::vector<double> separations;

  normals.clear();
  nearest.clear();

  for (size_t index = begin; index != contacts.size(); ++index) {
    const glm::dvec3& normal = contacts[index].basis[0];
    uint32_t group = 0u;

    while (group != normals.size()
           && glm::dot(normal, normals[group]) < sharedNormal) {
      group += 1u;
    }

    if (group == normals.size()) {
      normals.push_back(normal);
      nearest.push_back(contacts[index].gap);
    }

    nearest[group] = std::min(nearest[group], contacts[index].gap);
    contacts[index].manifold = group;
  }

  keep.assign(contacts.size() - begin, 0u);

  for (uint32_t group = 0u; group != normals.size(); ++group) {
    members.clear();
    points.clear();
    separations.clear();

    for (size_t index = begin; index != contacts.size(); ++index) {
      const Contact& contact = contacts[index];

      if (contact.manifold == group && contact.gap <= nearest[group] + window) {
        members.push_back(index);
        points.push_back(
          sidePoint(bodies, contact.a) - 0.5 * contact.gap * contact.basis[0]
        );
        separations.push_back(contact.gap);
      }
    }

    std::array<uint32_t, 4> chosen{};
    const uint32_t kept = reduceManifold(
      points,
      separations,
      normals[group],
      nearest[group] + window,
      spacing * spacing,
      chosen
    );

    for (uint32_t pick = 0u; pick != kept; ++pick) {
      keep[members[chosen[pick]] - begin] = 1u;
    }
  }

  size_t kept = begin;

  for (size_t index = begin; index != contacts.size(); ++index) {
    if (keep[index - begin] != 0u) {
      contacts[kept++] = contacts[index];
    }
  }

  contacts.resize(kept);

  return static_cast<uint32_t>(normals.size());
}

bool staticPoseHolds(
  const RigidBody& body,
  const CachedPose& cached,
  double tolerance
) {
  const double angle = glm::dot(body.rotation, cached.rotationA);
  const double distance = glm::length(body.position - cached.relativePosition);

  if (angle * angle <= recycleAngle || distance >= tolerance) {
    return false;
  }

  const glm::dquat turn =
    glm::conjugate(cached.relativeRotation) * body.rotation;
  const glm::dvec3 spin = glm::abs(glm::dvec3(turn.x, turn.y, turn.z));
  const glm::dvec3 extent = extentOf(body);
  const glm::dvec3 arc(
    spin.y * extent.z + spin.z * extent.y,
    spin.z * extent.x + spin.x * extent.z,
    spin.x * extent.y + spin.y * extent.x
  );
  const double slack = tolerance - distance;

  return 4.0 * glm::dot(arc, arc) < slack * slack;
}

bool pairPoseHolds(
  const RigidBody& a,
  const RigidBody& b,
  const CachedPose& cached,
  double tolerance,
  const glm::dmat3& turnA
) {
  const double angleA = glm::dot(a.rotation, cached.rotationA);
  const double angleB = glm::dot(b.rotation, cached.rotationB);
  const CachedPose now = poseOf(a, b, turnA);
  const double distance =
    glm::length(now.relativePosition - cached.relativePosition);

  if (std::min(angleA * angleA, angleB * angleB) <= recycleAngle
      || distance >= tolerance) {
    return false;
  }

  const glm::dquat turn =
    glm::conjugate(cached.relativeRotation) * now.relativeRotation;
  const glm::dvec3 spin = glm::abs(glm::dvec3(turn.x, turn.y, turn.z));
  const glm::dvec3 extent = glm::max(
    turningExtentOf(a, cached.rotationA),
    turningExtentOf(b, cached.rotationB)
  );
  const glm::dvec3 arc(
    spin.y * extent.z + spin.z * extent.y,
    spin.z * extent.x + spin.x * extent.z,
    spin.x * extent.y + spin.y * extent.x
  );
  const double slack = tolerance - distance;

  return 4.0 * glm::dot(arc, arc) < slack * slack;
}

bool pairPoseHolds(
  const RigidBody& a,
  const RigidBody& b,
  const CachedPose& cached,
  double tolerance
) {
  return pairPoseHolds(a, b, cached, tolerance, glm::mat3_cast(a.rotation));
}

bool recycleStaticRecords(
  const BodySet& bodies,
  const BodyRef& rigid,
  std::vector<Contact>& records,
  double travel
) {
  const CachedPose& cached = records.front().cached;

  if (!cached.valid || travel >= recycleDistance) {
    return false;
  }

  const RigidBody& body = bodies.rigids[rigid.index];

  if (!staticPoseHolds(body, cached, recycleDistance)) {
    return false;
  }

  const glm::dmat3 now = glm::mat3_cast(body.rotation);
  const glm::dmat3 then = glm::mat3_cast(cached.rotationA);

  for (Contact& contact : records) {
    const glm::dvec3 moved = body.position + now * contact.a.local
                             - cached.relativePosition
                             - then * contact.a.local;

    contact.gap =
      contact.cached.separation + glm::dot(moved, contact.basis[0]);
  }

  return true;
}

bool recycleStaticInPlace(const RigidBody& body, CachedBodyPair& pair, double window) {
  if (!staticPoseHolds(body, pair.pose, recycleDistance)) {
    return false;
  }

  const glm::dmat3 now = glm::mat3_cast(body.rotation);
  const glm::dmat3 then = glm::mat3_cast(pair.pose.rotationA);
  const auto separationOf = [&](const CachedManifold& manifold, const CachedManifoldPoint& point) {
    const glm::dvec3 moved = body.position + now * point.localA
                             - pair.pose.relativePosition - then * point.localA;

    return point.builtSeparation + glm::dot(moved, manifold.basis[0]);
  };

  for (const CachedManifold& manifold : pair.manifolds) {
    if (manifold.flipped) {
      return false;
    }

    double nearest = std::numeric_limits<double>::infinity();

    for (uint32_t index = 0u; index != manifold.count; ++index) {
      nearest = std::min(nearest, separationOf(manifold, pair.manifoldPoints[manifold.start + index]));
    }

    for (uint32_t index = 0u; index != manifold.count; ++index) {
      if (separationOf(manifold, pair.manifoldPoints[manifold.start + index]) > nearest + window) {
        return false;
      }
    }
  }

  for (const CachedManifold& manifold : pair.manifolds) {
    for (uint32_t index = 0u; index != manifold.count; ++index) {
      pair.points[manifold.start + index].separation =
        separationOf(manifold, pair.manifoldPoints[manifold.start + index]);
    }
  }

  return true;
}

bool recyclePair(const BodySet& bodies, CachedBodyPair& pair, double travel) {
  const CachedPose& cached = pair.pose;

  if (!cached.valid || travel >= recycleDistance) {
    return false;
  }

  const RigidBody& a = bodies.rigids[pair.first.index];
  const RigidBody& b = bodies.rigids[pair.second.index];
  const glm::dmat3 turnA = glm::mat3_cast(a.rotation);

  if (!pairPoseHolds(a, b, cached, recycleDistance, turnA)) {
    return false;
  }

  const glm::dmat3 turnB = glm::mat3_cast(b.rotation);

  for (const CachedManifold& manifold : pair.manifolds) {
    const RigidBody& onA = manifold.flipped ? b : a;
    const RigidBody& onB = manifold.flipped ? a : b;
    const glm::dmat3& turnOnA = manifold.flipped ? turnB : turnA;
    const glm::dmat3& turnOnB = manifold.flipped ? turnA : turnB;

    for (uint32_t index = 0u; index != manifold.count; ++index) {
      const CachedManifoldPoint& built = pair.manifoldPoints[manifold.start + index];

      pair.points[manifold.start + index].separation = glm::dot(
        (onA.position + turnOnA * built.localA) - (onB.position + turnOnB * built.localB),
        manifold.basis[0]
      );
    }
  }

  return true;
}

struct PairFrame {
  CachedPose pose;
  double friction = 0.0;
  double restitution = 0.0;
  glm::dmat3 turnA{1.0};
  glm::dmat3 turnB{1.0};
};

PairFrame frameOf(const BodySet& bodies, const BodyRef& first, const BodyRef& second) {
  const RigidBody& a = bodies.rigids[first.index];
  const RigidBody& b = bodies.rigids[second.index];
  PairFrame frame;

  frame.pose = poseOf(a, b);
  frame.friction = std::sqrt(frictionOf(bodies, first) * frictionOf(bodies, second));
  frame.restitution = std::max(restitutionOf(bodies, first), restitutionOf(bodies, second));
  frame.turnA = glm::mat3_cast(a.rotation);
  frame.turnB = glm::mat3_cast(b.rotation);

  return frame;
}

void addChildManifold(
  const BodySet& bodies,
  const BodyRef& first,
  const BodyRef& second,
  const PairFrame& frame,
  uint32_t childA,
  uint32_t childB,
  const Manifold& manifold,
  uint32_t group,
  std::vector<Contact>& contacts
) {
  const RigidBody& a = bodies.rigids[first.index];
  const RigidBody& b = bodies.rigids[second.index];
  const CachedPose& pose = frame.pose;
  const double friction = frame.friction;
  const double restitution = frame.restitution;
  const glm::dmat3& turnA = frame.turnA;
  const glm::dmat3& turnB = frame.turnB;

  for (uint32_t index = 0u; index != manifold.count; ++index) {
    const ManifoldPoint& point = manifold.points[index];
    const glm::dvec3 shift = (0.5 * point.separation) * manifold.normal;
    const glm::dvec3 onA = point.point + shift;
    const glm::dvec3 onB = point.point - shift;
    ContactSide sideA;
    ContactSide sideB;

    sideA.side = Side::Rigid;
    sideA.body = first.index;
    sideA.child = childA;
    sideA.local = glm::transpose(turnA) * (onA - a.position);
    sideB.side = Side::Rigid;
    sideB.body = second.index;
    sideB.child = childB;
    sideB.local = glm::transpose(turnB) * (onB - b.position);
    addContact(
      contacts,
      sideA,
      sideB,
      manifold.normal,
      point.separation,
      friction,
      contactKey(bodies, first, second, 0x20000u | point.feature)
    );
    contacts.back().restitution = restitution;
    contacts.back().cached = pose;
    contacts.back().cached.separation = point.separation;
    contacts.back().manifold = group;
  }
}

void primitivePair(
  const BodySet& bodies,
  const BodyRef& first,
  const BodyRef& second,
  double band,
  std::vector<Contact>& contacts
) {
  addChildManifold(
    bodies,
    first,
    second,
    frameOf(bodies, first, second),
    noChild,
    noChild,
    collidePair(bodies.rigids[first.index], bodies.rigids[second.index], band),
    0u,
    contacts
  );
}

using ProbeCache = std::vector<std::unique_ptr<deformable::DeformedProbe>>;

const deformable::DeformedProbe& placedProbe(
  const BodySet& bodies,
  uint32_t lattice,
  ProbeCache& probes,
  double reach
) {
  if (probes.size() != bodies.lattices.size()) {
    probes.resize(bodies.lattices.size());
  }

  if (!probes[lattice]) {
    const LatticeBody& body = bodies.lattices[lattice];

    probes[lattice] = std::make_unique<deformable::DeformedProbe>(
      deformable::deformedProbe(*body.mesh, *body.field, reach)
    );
    deformable::placeDeformedProbe(*probes[lattice], body.position);
  }

  return *probes[lattice];
}

void rigidAgainstLattice(
  const BodySet& bodies,
  const BodyRef& points,
  const BodyRef& target,
  double band,
  std::vector<Contact>& contacts,
  const ProbeCache& probes
) {
  const RigidBody& body = bodies.rigids[points.index];
  const LatticeBody& partner = bodies.lattices[target.index];

  if (partner.field == nullptr) {
    return;
  }

  const deformable::DeformedProbe& probe = *probes[target.index];
  const double friction = std::sqrt(body.friction * partner.friction);
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);

  for (uint32_t index = 0u; index != body.samples.size(); ++index) {
    const glm::dvec3 world = body.position + turn * body.samples[index];
    const deformable::DeformedDistance hit =
      deformable::probeDeformed(probe, world);

    if (!hit.found || !std::isfinite(hit.distance) || hit.distance >= band
        || !(glm::length(hit.normal) > 0.5)) {
      continue;
    }

    ContactSide side;
    ContactSide other;

    side.side = Side::Rigid;
    side.body = points.index;
    side.local = body.samples[index];
    other.side = Side::Lattice;
    other.body = target.index;
    other.nodes = hit.nodes;
    other.weights = hit.weights;
    addContact(
      contacts,
      side,
      other,
      glm::normalize(hit.normal),
      hit.distance,
      friction,
      contactKey(bodies, points, target, index)
    );
  }
}

void latticeAgainstLattice(
  const BodySet& bodies,
  const BodyRef& points,
  const BodyRef& target,
  double band,
  std::vector<Contact>& contacts,
  const ProbeCache& probes
) {
  const LatticeBody& source = bodies.lattices[points.index];
  const LatticeBody& partner = bodies.lattices[target.index];

  if (partner.field == nullptr) {
    return;
  }

  const deformable::DeformedProbe& probe = *probes[target.index];
  const double friction = std::sqrt(source.friction * partner.friction);
  std::vector<std::pair<ContactSide, glm::dvec3>> sources;

  latticeSources(source, points.index, sources);

  for (uint32_t index = 0u; index != sources.size(); ++index) {
    const ContactSide& side = sources[index].first;
    const glm::dvec3& world = sources[index].second;
    const deformable::DeformedDistance hit =
      deformable::probeDeformed(probe, world);

    if (!hit.found || !std::isfinite(hit.distance) || hit.distance >= band
        || !(glm::length(hit.normal) > 0.5)) {
      continue;
    }

    ContactSide other;

    other.side = Side::Lattice;
    other.body = target.index;
    other.nodes = hit.nodes;
    other.weights = hit.weights;
    addContact(
      contacts,
      side,
      other,
      glm::normalize(hit.normal),
      hit.distance,
      friction,
      contactKey(bodies, points, target, index)
    );
  }
}

}

namespace {

struct CoveredManifold {
  uint32_t childA = noChild;
  uint32_t childB = noChild;
  Manifold manifold;
};

struct ChildScratch {
  std::vector<uint32_t> found;
  std::vector<std::pair<uint32_t, uint32_t>> pairs;
  std::vector<CoveredManifold> covered;
};

Bounds boundsInBody(const RigidBody& body, const Bounds& world) {
  const glm::dmat3 back = glm::transpose(glm::mat3_cast(body.rotation));
  const glm::dvec3 middle = 0.5 * (world.low + world.high);
  const glm::dvec3 half = 0.5 * (world.high - world.low);

  return boxBoundsIn(back * (middle - body.position), back, half);
}

void compoundAgainstField(
  const BodySet& bodies,
  const BodyRef& points,
  const BodyRef& field,
  double band,
  std::vector<uint32_t>& candidates,
  std::vector<Contact>& contacts
) {
  const RigidBody& body = bodies.rigids[points.index];
  const CompoundShape& shape = *body.compound;
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  const double friction =
    std::sqrt(frictionOf(bodies, points) * frictionOf(bodies, field));
  const double restitution =
    std::max(restitutionOf(bodies, points), restitutionOf(bodies, field));

  overlappingChildren(shape, boundsInBody(body, boundsOf(bodies, field, band)), candidates);

  for (const uint32_t index : candidates) {
    const CompoundChild& child = shape.children[index];
    const std::array<glm::dvec3, childSampleCount> samples = childSamples(child);

    for (uint32_t sample = 0u; sample != childSampleCount; ++sample) {
      const glm::dvec3 point = body.position + turn * samples[sample];
      Hit hit;
      ContactSide side;
      ContactSide other;

      if (field.side == Side::Static) {
        const StaticField& fixed = bodies.statics[field.index];

        if (!fieldHit(*fixed.field, staticLocal(fixed, point), band, hit)) {
          continue;
        }

        hit.normal = staticDirection(fixed, hit.normal);
        other.side = Side::Static;
        other.body = field.index;
      } else {
        const RigidBody& target = bodies.rigids[field.index];
        const glm::dmat3 targetTurn = glm::mat3_cast(target.rotation);
        const glm::dmat3 frame = glm::mat3_cast(target.fieldRotation);
        const glm::dvec3 local = glm::transpose(targetTurn) * (point - target.position);

        if (!fieldHit(*target.field, target.fieldOrigin + frame * local, band, hit)) {
          continue;
        }

        hit.normal = glm::transpose(frame) * hit.normal;
        other.side = Side::Rigid;
        other.body = field.index;
        other.local = local - hit.gap * hit.normal;
        hit.normal = targetTurn * hit.normal;
      }

      side.side = Side::Rigid;
      side.body = points.index;
      side.child = child.key;
      side.local = samples[sample];
      addContact(
        contacts,
        side,
        other,
        hit.normal,
        hit.gap,
        friction,
        contactKey(bodies, points, field, sample)
      );
      contacts.back().restitution = restitution;
    }
  }
}

uint32_t childCount(const RigidBody& body) {
  return compound(body) ? static_cast<uint32_t>(body.compound->children.size()) : 1u;
}

CompoundChild childOf(const RigidBody& body, uint32_t index) {
  if (compound(body)) {
    return body.compound->children[index];
  }

  CompoundChild whole;

  whole.half = body.shape == RigidShape::Sphere ? glm::dvec3(body.radius) : body.halfExtents;
  whole.key = noChild;

  return whole;
}

BoxPose childPose(const RigidBody& body, const CompoundChild& child) {
  return BoxPose{
    body.position + glm::mat3_cast(body.rotation) * child.centre,
    body.rotation * child.rotation,
    child.half
  };
}

Manifold collideChildren(
  const RigidBody& a,
  const CompoundChild& childA,
  const RigidBody& b,
  const CompoundChild& childB,
  double band
) {
  if (a.shape == RigidShape::Sphere) {
    return collideBoxAndSphere(a.position, a.radius, childPose(b, childB), band);
  }

  if (b.shape == RigidShape::Sphere) {
    Manifold flipped = collideBoxAndSphere(b.position, b.radius, childPose(a, childA), band);

    flipped.normal = -flipped.normal;

    return flipped;
  }

  return collideBoxes(childPose(a, childA), childPose(b, childB), band);
}

void listChildPairs(
  const RigidBody& a,
  const RigidBody& b,
  double grow,
  ChildScratch& scratch
) {
  const bool treeOnB = compound(b) && (!compound(a) || childCount(b) >= childCount(a));
  const RigidBody& tree = treeOnB ? b : a;
  const RigidBody& probe = treeOnB ? a : b;
  const glm::dmat3 back = glm::transpose(glm::mat3_cast(tree.rotation));
  const glm::dmat3 probeTurn = glm::mat3_cast(probe.rotation);
  const glm::dmat3 relative = back * probeTurn;
  const glm::dquat unturned(1.0, 0.0, 0.0, 0.0);

  scratch.pairs.clear();

  for (uint32_t index = 0u; index != childCount(probe); ++index) {
    const CompoundChild child = childOf(probe, index);
    const glm::dvec3 centre =
      back * (probe.position + probeTurn * child.centre - tree.position);
    const glm::dmat3 turn = child.rotation == unturned ? relative : relative * glm::mat3_cast(child.rotation);

    overlappingChildren(
      *tree.compound,
      grown(boxBoundsIn(centre, turn, child.half), grow),
      scratch.found
    );

    for (const uint32_t other : scratch.found) {
      scratch.pairs.emplace_back(treeOnB ? index : other, treeOnB ? other : index);
    }
  }

  std::sort(scratch.pairs.begin(), scratch.pairs.end());
}

bool throughCoveredFace(const RigidBody& body, const glm::dmat3& turn, uint32_t index, const glm::dvec3& outward) {
  if (!compound(body) || body.compound->exposed[index] == allFaces) {
    return false;
  }

  const glm::dquat& rotation = body.compound->children[index].rotation;
  const glm::dvec3 local =
    glm::transpose(rotation == glm::dquat(1.0, 0.0, 0.0, 0.0) ? turn : glm::mat3_cast(body.rotation * rotation))
    * outward;

  for (int axis = 0; axis != 3; ++axis) {
    if (std::abs(local[axis]) > sharedNormal) {
      const uint32_t face = 2u * static_cast<uint32_t>(axis) + (local[axis] > 0.0 ? 1u : 0u);

      return (body.compound->exposed[index] & (1u << face)) == 0u;
    }
  }

  return false;
}

struct MovingPiece {
  glm::dvec3 centre{0.0};
  glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 half{0.0};
  glm::dvec3 velocity{0.0};
  double turn = 0.0;
  double arm = 0.0;
  double slack = 0.0;
  bool round = false;
};

MovingPiece pieceOf(
  const RigidBody& body,
  const glm::dvec3& centre,
  const glm::dquat& rotation,
  const glm::dvec3& half,
  double dt,
  double slack
) {
  if (body.shape == RigidShape::Sphere) {
    return MovingPiece{
      body.position,
      glm::dquat(1.0, 0.0, 0.0, 0.0),
      glm::dvec3(body.radius),
      slack > 0.0 ? glm::dvec3(0.0) : body.velocity,
      0.0,
      0.0,
      slack,
      true
    };
  }

  const glm::dvec3 arm = glm::mat3_cast(body.rotation) * centre;

  return MovingPiece{
    body.position + arm,
    body.rotation * rotation,
    half,
    slack > 0.0 ? glm::dvec3(0.0) : body.velocity + glm::cross(body.angularVelocity, arm),
    glm::length(body.angularVelocity) * dt,
    glm::length(arm),
    slack,
    false
  };
}

MovingPiece wholePiece(const RigidBody& body, double dt, double slack) {
  return pieceOf(body, glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), body.halfExtents, dt, slack);
}

bool segmentMeetsBox(const glm::dvec3& from, const glm::dvec3& to, const glm::dvec3& low, const glm::dvec3& high) {
  const glm::dvec3 along = to - from;
  double enter = 0.0;
  double leave = 1.0;

  for (int axis = 0; axis != 3; ++axis) {
    if (along[axis] == 0.0) {
      if (from[axis] < low[axis] || from[axis] > high[axis]) {
        return false;
      }

      continue;
    }

    const double first = (low[axis] - from[axis]) / along[axis];
    const double second = (high[axis] - from[axis]) / along[axis];

    enter = std::max(enter, std::min(first, second));
    leave = std::min(leave, std::max(first, second));

    if (enter > leave) {
      return false;
    }
  }

  return true;
}

bool pieceReaches(const MovingPiece& moving, const MovingPiece& target, double dt, double band) {
  const glm::dquat back = glm::conjugate(target.rotation);
  const glm::dvec3 offset = back * (moving.centre - target.centre);
  const glm::dvec3 shift = back * ((moving.velocity - target.velocity) * dt);
  const glm::dmat3 turn = glm::mat3_cast(back * moving.rotation);
  const glm::dvec3& half = moving.half;
  const glm::dvec3 span = moving.round ? half : glm::dvec3(
    std::abs(turn[0][0]) * half.x + std::abs(turn[1][0]) * half.y + std::abs(turn[2][0]) * half.z,
    std::abs(turn[0][1]) * half.x + std::abs(turn[1][1]) * half.y + std::abs(turn[2][1]) * half.z,
    std::abs(turn[0][2]) * half.x + std::abs(turn[1][2]) * half.y + std::abs(turn[2][2]) * half.z
  );
  const double arc = (moving.round ? 0.0 : (moving.turn + target.turn) * glm::length(half))
                     + moving.turn * moving.arm
                     + target.turn * (target.arm + glm::length(offset) + glm::length(shift));
  const glm::dvec3 grow = target.half + span + glm::dvec3(arc + band + moving.slack + target.slack);

  return segmentMeetsBox(offset, offset + shift, -grow, grow);
}

bool piecesMeet(const MovingPiece& a, const MovingPiece& b, double dt, double band) {
  return pieceReaches(a, b, dt, band) && pieceReaches(b, a, dt, band);
}

bool childrenMeet(
  const RigidBody& a,
  const CompoundChild& childA,
  double slackA,
  const RigidBody& b,
  const CompoundChild& childB,
  double slackB,
  double dt,
  double band
) {
  return piecesMeet(
    pieceOf(a, childA.centre, childA.rotation, childA.half, dt, slackA),
    pieceOf(b, childB.centre, childB.rotation, childB.half, dt, slackB),
    dt,
    band
  );
}

void compoundPair(
  const BodySet& bodies,
  const BodyRef& first,
  const BodyRef& second,
  double band,
  double reach,
  double dt,
  double slackA,
  double slackB,
  ChildScratch& scratch,
  std::vector<Contact>& contacts,
  PairClock& clock
) {
  const RigidBody& a = bodies.rigids[first.index];
  const RigidBody& b = bodies.rigids[second.index];

  if (a.shape == RigidShape::Sampled || b.shape == RigidShape::Sampled) {
    const bool onFirst = compound(a);
    const CachedPose pose = poseOf(a, b);

    compoundAgainstField(
      bodies,
      onFirst ? first : second,
      onFirst ? second : first,
      reach,
      scratch.found,
      contacts
    );

    for (Contact& contact : contacts) {
      contact.cached = pose;
      contact.cached.separation = contact.gap;
    }

    clock.charge(&PairTicks::narrowphase);
    (void)groupByNormal(bodies, contacts, 0u, reach, band);
    clock.charge(&PairTicks::reduce);

    return;
  }

  listChildPairs(a, b, reach, scratch);
  clock.charge(&PairTicks::midphase);
  scratch.covered.clear();

  const bool pathTested = reach > band && (slackA == 0.0 || slackB == 0.0);
  uint32_t group = 0u;
  PairFrame frame;
  bool framed = false;

  for (const auto& [onA, onB] : scratch.pairs) {
    const CompoundChild childA = childOf(a, onA);
    const CompoundChild childB = childOf(b, onB);

    if (pathTested && !childrenMeet(a, childA, slackA, b, childB, slackB, dt, band)) {
      continue;
    }

    if (clock.tally != nullptr) {
      clock.tally->childPairs += 1u;
    }

    const Manifold manifold = collideChildren(a, childA, b, childB, reach);
    const bool boxes = a.shape != RigidShape::Sphere && b.shape != RigidShape::Sphere;

    if (manifold.count == 0u) {
      continue;
    }

    if (!framed) {
      frame = frameOf(bodies, first, second);
      framed = true;
    }

    if (boxes && (throughCoveredFace(a, frame.turnA, onA, -manifold.normal)
                  || throughCoveredFace(b, frame.turnB, onB, manifold.normal))) {
      scratch.covered.push_back(CoveredManifold{childA.key, childB.key, manifold});
      continue;
    }

    addChildManifold(bodies, first, second, frame, childA.key, childB.key, manifold, group, contacts);
    group += 1u;
  }

  if (group == 0u) {
    for (const CoveredManifold& kept : scratch.covered) {
      addChildManifold(bodies, first, second, frame, kept.childA, kept.childB, kept.manifold, group, contacts);
      group += 1u;
    }
  }

  clock.charge(&PairTicks::narrowphase);

  if (group > 1u) {
    (void)groupByNormal(bodies, contacts, 0u, reach, band);
    std::sort(contacts.begin(), contacts.end(), [](const Contact& x, const Contact& y) {
      return std::tie(x.gap, x.manifold, x.a.child, x.b.child, x.key)
             < std::tie(y.gap, y.manifold, y.a.child, y.b.child, y.key);
    });
  }

  clock.charge(&PairTicks::reduce);
}

}

glm::dvec3 sidePoint(const BodySet& bodies, const ContactSide& side) {
  if (side.side == Side::Lattice) {
    const LatticeBody& body = bodies.lattices[side.body];
    glm::dvec3 point(0.0);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      point += side.weights[corner] * body.position[side.nodes[corner]];
    }

    return point;
  }

  if (side.side == Side::Rigid) {
    const RigidBody& body = bodies.rigids[side.body];

    return body.position + glm::mat3_cast(body.rotation) * side.local;
  }

  return glm::dvec3(0.0);
}

namespace {

std::vector<Contact> collide(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  std::span<const double> travel,
  const Workers& workers
);

}

std::vector<Contact> narrowphase(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  const Workers& workers
) {
  const std::vector<double> still(
    bodies.lattices.size() + bodies.rigids.size(),
    0.0
  );

  return collide(bodies, pairs, band, still, workers);
}

std::vector<Contact> narrowphase(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  std::span<const double> travel,
  const Workers& workers
) {
  return collide(bodies, pairs, band, travel, workers);
}

namespace {

std::vector<Contact> collide(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  std::span<const double> travel,
  const Workers& workers
) {
  ProbeCache probes(bodies.lattices.size());
  std::vector<double> probeReach(bodies.lattices.size(), 0.0);
  std::vector<uint8_t> probed(bodies.lattices.size(), 0u);

  const auto widenProbe = [&](const BodyRef& target, const BodyRef& other) {
    if (target.side == Side::Lattice && other.side != Side::Static
        && bodies.lattices[target.index].field != nullptr) {
      const double reach =
        band + travelOf(bodies, travel, target) + travelOf(bodies, travel, other);

      probed[target.index] = 1u;
      probeReach[target.index] = std::max(
        probeReach[target.index],
        std::isfinite(reach) ? reach : std::numeric_limits<double>::infinity()
      );
    }
  };

  for (const BodyPair& pair : pairs) {
    widenProbe(pair.first, pair.second);
    widenProbe(pair.second, pair.first);
  }

  for (uint32_t lattice = 0u; lattice != probed.size(); ++lattice) {
    if (probed[lattice] != 0u) {
      (void)placedProbe(bodies, lattice, probes, probeReach[lattice]);
    }
  }

  std::vector<std::vector<Contact>> found(pairs.size());
  std::vector<uint8_t> sharedPair(pairs.size(), 0u);

  parallelFor(workers, static_cast<uint32_t>(pairs.size()), 4u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      const BodyRef& first = pairs[index].first;
      const BodyRef& second = pairs[index].second;
      std::vector<Contact>& contacts = found[index];
      const bool fixedPair =
        first.side == Side::Static || second.side == Side::Static;
      const double reach =
        fixedPair ? band
                  : band + travelOf(bodies, travel, first)
                      + travelOf(bodies, travel, second);
      const bool holdsCompound =
        (first.side == Side::Rigid && compound(bodies.rigids[first.index]))
        || (second.side == Side::Rigid && compound(bodies.rigids[second.index]));

      if (holdsCompound) {
        continue;
      }

      if (first.side == Side::Static || second.side == Side::Static) {
        const BodyRef& moving = first.side == Side::Static ? second : first;
        const BodyRef& fixed = first.side == Side::Static ? first : second;

        pointsAgainst(bodies, moving, fixed, reach, contacts);

        if (moving.side == Side::Rigid) {
          const CachedPose pose = staticPoseOf(bodies.rigids[moving.index]);

          for (Contact& contact : contacts) {
            contact.cached = pose;
            contact.cached.separation = contact.gap;
          }
        }
      } else if (first.side == Side::Rigid && second.side == Side::Rigid
                 && primitive(bodies.rigids[first.index])
                 && primitive(bodies.rigids[second.index])) {
        const bool ordered =
          bodies.rigids[first.index].id < bodies.rigids[second.index].id;

        primitivePair(
          bodies,
          ordered ? first : second,
          ordered ? second : first,
          reach,
          contacts
        );
        sharedPair[index] = 1u;
      } else if (first.side == Side::Rigid && second.side == Side::Rigid) {
        pointsAgainst(bodies, first, second, reach, contacts);
        pointsAgainst(bodies, second, first, reach, contacts);
      } else if (first.side == Side::Lattice && second.side == Side::Rigid) {
        pointsAgainst(bodies, first, second, reach, contacts);
        rigidAgainstLattice(bodies, second, first, reach, contacts, probes);
      } else if (first.side == Side::Rigid && second.side == Side::Lattice) {
        pointsAgainst(bodies, second, first, reach, contacts);
        rigidAgainstLattice(bodies, first, second, reach, contacts, probes);
      } else if (first.side == Side::Lattice && second.side == Side::Lattice) {
        latticeAgainstLattice(bodies, first, second, reach, contacts, probes);
        latticeAgainstLattice(bodies, second, first, reach, contacts, probes);
      }

      const uint64_t key = pairKey(bodies, first, second);

      for (Contact& contact : contacts) {
        contact.pair = key;
      }
    }
  });

  size_t total = 0u;

  for (const std::vector<Contact>& list : found) {
    total += list.size();
  }

  std::vector<Contact> contacts;
  uint32_t manifold = 0u;

  contacts.reserve(total);

  for (size_t index = 0u; index != pairs.size(); ++index) {
    const size_t begin = contacts.size();

    contacts.insert(contacts.end(), found[index].begin(), found[index].end());

    if (sharedPair[index] != 0u && begin != contacts.size()) {
      for (size_t at = begin; at != contacts.size(); ++at) {
        contacts[at].manifold = manifold;
      }

      manifold += 1u;
    } else {
      for (size_t at = begin; at != contacts.size(); ++at) {
        contacts[at].manifold = manifold++;
      }
    }
  }

  return contacts;
}

}

uint64_t pairKey(
  const BodySet& bodies,
  const BodyRef& first,
  const BodyRef& second
) {
  if (first.side == Side::Static || second.side == Side::Static) {
    const BodyRef& moving = first.side == Side::Static ? second : first;
    const BodyRef& fixed = first.side == Side::Static ? first : second;

    return 1ull << 63u
           | static_cast<uint64_t>(keyIndex(bodies, moving)) << 32u
           | fixed.index;
  }

  const uint32_t a = keyIndex(bodies, first);
  const uint32_t b = keyIndex(bodies, second);

  return static_cast<uint64_t>(std::min(a, b)) << 32u | std::max(a, b);
}

double turningTravel(
  const BodySet& bodies,
  std::span<const double> travel,
  const BodyRef& ref,
  double dt
) {
  return std::max(0.0, travelOf(bodies, travel, ref) - glm::length(bodies.rigids[ref.index].velocity) * dt);
}

double relativeTravel(
  const BodySet& bodies,
  std::span<const double> travel,
  const BodyRef& first,
  const BodyRef& second,
  double dt
) {
  const glm::dvec3 apart = bodies.rigids[first.index].velocity - bodies.rigids[second.index].velocity;

  return glm::length(apart) * dt + turningTravel(bodies, travel, first, dt)
         + turningTravel(bodies, travel, second, dt);
}

namespace {

constexpr uint8_t firstHeld = 1u;
constexpr uint8_t secondHeld = 2u;
constexpr uint8_t countedTouch = 4u;
constexpr uint8_t rebuiltPair = 8u;
constexpr uint8_t startedEmpty = 16u;
constexpr uint8_t startedWide = 32u;
constexpr uint8_t visitedPair = 64u;
constexpr uint8_t queuedPair = 128u;

}

PairEvents updateCachedPairs(
  const BodySet& bodies,
  ManifoldCache& cache,
  double band,
  std::span<const double> travel,
  double dt,
  const Workers& workers,
  const BroadPhase* broadPhase,
  PairWork* work,
  const std::vector<uint32_t>* active,
  bool sameIndices
) {
  std::vector<std::vector<Contact>> scratch(workers.count);
  std::vector<ChildScratch> childScratch(workers.count);
  std::vector<std::vector<uint32_t>> separated(workers.count);
  std::vector<std::vector<uint32_t>> changed(workers.count);
  std::vector<std::vector<uint32_t>> risen(workers.count);
  std::vector<std::vector<uint32_t>> marked(workers.count);
  std::vector<uint32_t> kept;
  std::vector<PairTicks> tallies(work != nullptr ? workers.count : 0u);
  std::vector<uint8_t> touchedBefore(bodies.rigids.size(), 0u);
  std::vector<uint8_t> touched(bodies.rigids.size(), 0u);
  std::vector<uint8_t> flags(cache.size(), 0u);
  const uint64_t firstTick = work != nullptr ? ticks() : 0u;
  const std::chrono::steady_clock::time_point firstTime = std::chrono::steady_clock::now();

  if (sameIndices) {
    for (const uint32_t index : cache.touchedIndices()) {
      touchedBefore[index] = 1u;
    }
  } else {
    for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
      touchedBefore[index] =
        cache.touchedLast(bodies.rigids[index].id) ? 1u : 0u;
    }
  }

  if (broadPhase != nullptr) {
    for (const BodyPair& near : broadPhase->latticePairs()) {
      for (const BodyRef& ref : std::array<BodyRef, 2>{near.first, near.second}) {
        if (ref.side == Side::Rigid) {
          touchedBefore[ref.index] = 1u;

          if (touched[ref.index] == 0u) {
            touched[ref.index] = 1u;
            kept.push_back(ref.index);
          }
        }
      }
    }
  }

  const auto held = [&](std::span<const uint8_t> marks, const BodyRef& ref) {
    return movable(bodies, ref) && travelOf(bodies, travel, ref) > 0.0 && marks[ref.index] != 0u;
  };

  const auto touch = [&](uint32_t index, uint32_t worker) {
    std::atomic_ref<uint8_t> mark(touched[index]);

    if (mark.load(std::memory_order_relaxed) != 0u) {
      return;
    }

    if (mark.exchange(1u, std::memory_order_relaxed) != 0u) {
      return;
    }

    marked[worker].push_back(index);

    if (touchedBefore[index] == 0u) {
      risen[worker].push_back(index);
    }
  };

  const auto update = [&](uint32_t slot, uint32_t worker, PairClock& clock, bool checks) {
    std::vector<Contact>& records = scratch[worker];
    CachedBodyPair& pair = cache.at(slot);
    const bool heldFirst = (flags[slot] & firstHeld) != 0u;
    const bool heldSecond = (flags[slot] & secondHeld) != 0u;
    const double travelFirst = travelOf(bodies, travel, pair.first);
    const double travelSecond = travelOf(bodies, travel, pair.second);
    const bool fixedPair = pair.second.side == Side::Static;
    const double reach =
      fixedPair                  ? band
      : heldFirst || heldSecond ? band + travelFirst + travelSecond
                                : band + relativeTravel(bodies, travel, pair.first, pair.second, dt);
    const double slackFirst = heldFirst ? travelFirst : 0.0;
    const double slackSecond = heldSecond ? travelSecond : 0.0;

    records.clear();

    if (fixedPair) {
      const RigidBody& body = bodies.rigids[pair.first.index];
      const double tolerance = std::min(recycleDistance, band) - travelFirst;

      if (pair.points.empty() && pair.pose.valid && tolerance > 0.0
          && staticPoseHolds(body, pair.pose, tolerance)) {
        clock.charge(&PairTicks::recycle);

        if (clock.tally != nullptr) {
          clock.tally->recycled += 1u;
        }

        return;
      }

      if (!pair.points.empty() && pair.pose.valid
          && recycleStaticInPlace(body, pair, band)) {
        clock.charge(&PairTicks::recycle);

        if (clock.tally != nullptr) {
          clock.tally->recycled += 1u;
        }

        return;
      }

      bool reused = false;

      if (!pair.points.empty() && pair.pose.valid) {
        records = cache.records(slot);
        reused = recycleStaticRecords(bodies, pair.first, records, reach - band);
      }

      clock.charge(&PairTicks::recycle);

      if (reused && clock.tally != nullptr) {
        clock.tally->recycled += 1u;
      }

      if (!reused) {
        records.clear();

        if (compound(body)) {
          compoundAgainstField(bodies, pair.first, pair.second, reach, childScratch[worker].found, records);
        } else {
          pointsAgainst(bodies, pair.first, pair.second, reach, records);
        }

        const CachedPose pose = staticPoseOf(body);

        for (Contact& contact : records) {
          contact.cached = pose;
          contact.cached.separation = contact.gap;
        }

        clock.charge(&PairTicks::narrowphase);
      }

      (void)groupByNormal(bodies, records, 0u, band, band);
      clock.charge(&PairTicks::reduce);
      cache.rebuildManifolds(bodies, slot, records);
      flags[slot] |= rebuiltPair;
      clock.charge(&PairTicks::build);

      if (records.empty()) {
        pair.pose = staticPoseOf(body);
      }

      return;
    }

    const RigidBody& a = bodies.rigids[pair.first.index];
    const RigidBody& b = bodies.rigids[pair.second.index];

    if (compound(a) || compound(b) || (primitive(a) && primitive(b))) {
      const double tolerance = std::min(recycleDistance, band) - (reach - band);

      if (checks && pair.points.empty() && pair.pose.valid && tolerance > 0.0
          && pairPoseHolds(a, b, pair.pose, tolerance)) {
        clock.charge(&PairTicks::recycle);

        if (clock.tally != nullptr) {
          clock.tally->recycled += 1u;
        }

        return;
      }

      if (checks && !pair.points.empty() && recyclePair(bodies, pair, reach - band)) {
        clock.charge(&PairTicks::recycle);

        if (clock.tally != nullptr) {
          clock.tally->recycled += 1u;
        }

        return;
      }

      clock.charge(&PairTicks::recycle);

      if (compound(a) || compound(b)) {
        compoundPair(bodies, pair.first, pair.second, band, reach, dt, slackFirst, slackSecond, childScratch[worker], records, clock);
      } else {
        if (reach <= band || (slackFirst > 0.0 && slackSecond > 0.0)
            || piecesMeet(wholePiece(a, dt, slackFirst), wholePiece(b, dt, slackSecond), dt, band)) {
          primitivePair(bodies, pair.first, pair.second, reach, records);
        }

        for (Contact& contact : records) {
          contact.manifold = 0u;
        }

        clock.charge(&PairTicks::narrowphase);
      }

      cache.rebuildManifolds(bodies, slot, records);
      flags[slot] |= rebuiltPair;
      clock.charge(&PairTicks::build);

      if (records.empty()) {
        pair.pose = poseOf(a, b);
      }

      return;
    }

    clock.charge(&PairTicks::recycle);
    pointsAgainst(bodies, pair.first, pair.second, reach, records);
    pointsAgainst(bodies, pair.second, pair.first, reach, records);

    for (uint32_t index = 0u; index != records.size(); ++index) {
      records[index].manifold = index;
    }

    clock.charge(&PairTicks::narrowphase);
    cache.rebuildManifolds(bodies, slot, records);
    flags[slot] |= rebuiltPair;
    clock.charge(&PairTicks::build);
  };

  const auto count = [&](uint32_t slot, uint32_t worker) {
    const CachedBodyPair& pair = cache.at(slot);

    if (pair.points.empty() || (flags[slot] & countedTouch) != 0u) {
      return;
    }

    flags[slot] |= countedTouch;
    touch(pair.first.index, worker);

    if (pair.second.side == Side::Rigid) {
      touch(pair.second.index, worker);
    }
  };

  const uint32_t walked = active != nullptr ? static_cast<uint32_t>(active->size()) : cache.size();

  parallelFor(workers, walked, 16u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
    PairClock clock{work != nullptr ? &tallies[worker] : nullptr};

    for (uint32_t entry = begin; entry != end; ++entry) {
      const uint32_t slot = active != nullptr ? (*active)[entry] : entry;

      if (!cache.live(slot)) {
        continue;
      }

      CachedBodyPair& pair = cache.at(slot);

      if (resting(bodies, pair.first) && resting(bodies, pair.second)) {
        continue;
      }

      clock.start();

      if (broadPhase != nullptr
          && broadPhase->separated(bodies, pair.first, pair.second)) {
        separated[worker].push_back(slot);
        clock.charge(&PairTicks::recycle);
        continue;
      }

      if (!movable(bodies, pair.first) && !movable(bodies, pair.second)) {
        clock.charge(&PairTicks::recycle);
        continue;
      }

      if (clock.tally != nullptr) {
        clock.tally->tested += 1u;
      }

      const bool wasEmpty = pair.points.empty();
      const bool wasWide = fitsWideLane(pair);
      uint32_t state = (wasEmpty ? startedEmpty : 0u) | (wasWide ? startedWide : 0u);

      if (pair.second.side != Side::Static) {
        state |= visitedPair;
        state |= held(touchedBefore, pair.first) ? firstHeld : 0u;
        state |= held(touchedBefore, pair.second) ? secondHeld : 0u;
      }

      flags[slot] = static_cast<uint8_t>(state);
      update(slot, worker, clock, true);

      if (pair.points.empty() != wasEmpty || fitsWideLane(pair) != wasWide) {
        changed[worker].push_back(slot);
      }

      count(slot, worker);
    }
  });

  std::vector<uint32_t> newly;
  std::vector<uint32_t> again;
  std::vector<uint32_t> redone;

  for (;;) {
    newly.clear();

    for (std::vector<uint32_t>& list : risen) {
      newly.insert(newly.end(), list.begin(), list.end());
      list.clear();
    }

    std::sort(newly.begin(), newly.end());
    again.clear();

    for (const uint32_t index : newly) {
      const BodyRef ref{Side::Rigid, index};
      const uint32_t id = bodies.rigids[index].id;

      for (const uint32_t slot : cache.pairsOf(id)) {
        if ((flags[slot] & visitedPair) == 0u) {
          continue;
        }

        const uint8_t bit = cache.at(slot).firstId == id ? firstHeld : secondHeld;

        if ((flags[slot] & bit) != 0u || !held(touched, ref)) {
          continue;
        }

        flags[slot] |= bit;

        if ((flags[slot] & queuedPair) == 0u) {
          flags[slot] |= queuedPair;
          again.push_back(slot);
        }
      }
    }

    if (again.empty()) {
      break;
    }

    std::sort(again.begin(), again.end());

    parallelFor(workers, static_cast<uint32_t>(again.size()), 4u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
      PairClock clock{work != nullptr ? &tallies[worker] : nullptr};

      for (uint32_t at = begin; at != end; ++at) {
        const uint32_t slot = again[at];

        clock.start();
        flags[slot] &= static_cast<uint8_t>(~queuedPair);
        update(slot, worker, clock, (flags[slot] & rebuiltPair) == 0u);
        count(slot, worker);

        if (clock.tally != nullptr) {
          clock.tally->redone += 1u;
        }
      }
    });

    redone.insert(redone.end(), again.begin(), again.end());
  }

  for (const std::vector<uint32_t>& list : marked) {
    kept.insert(kept.end(), list.begin(), list.end());
  }

  std::sort(kept.begin(), kept.end());
  cache.keepTouched(bodies, kept);

  if (work != nullptr) {
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - firstTime).count();
    const uint64_t spanned = ticks() - firstTick;
    const double perTick = spanned != 0u ? elapsed / static_cast<double>(spanned) : 0.0;

    *work = PairWork{};

    for (const PairTicks& tally : tallies) {
      work->recycle += perTick * static_cast<double>(tally.recycle);
      work->midphase += perTick * static_cast<double>(tally.midphase);
      work->narrowphase += perTick * static_cast<double>(tally.narrowphase);
      work->reduce += perTick * static_cast<double>(tally.reduce);
      work->build += perTick * static_cast<double>(tally.build);
      work->tested += tally.tested;
      work->recycled += tally.recycled;
      work->childPairs += tally.childPairs;
      work->redone += tally.redone;
    }
  }

  PairEvents events;

  for (const std::vector<uint32_t>& list : separated) {
    events.separated.insert(events.separated.end(), list.begin(), list.end());
  }

  std::sort(events.separated.begin(), events.separated.end());

  for (const uint32_t slot : events.separated) {
    const CachedBodyPair& pair = cache.at(slot);

    if (!pair.points.empty() && pair.second.side == Side::Rigid
        && movable(bodies, pair.first) && movable(bodies, pair.second)) {
      events.lost.push_back(pair.firstId);
    }

    cache.destroyPair(slot);
  }

  for (const std::vector<uint32_t>& list : changed) {
    events.changed.insert(events.changed.end(), list.begin(), list.end());
  }

  if (!redone.empty()) {
    std::sort(redone.begin(), redone.end());
    redone.erase(std::unique(redone.begin(), redone.end()), redone.end());
    std::erase_if(events.changed, [&](uint32_t slot) {
      return std::binary_search(redone.begin(), redone.end(), slot);
    });

    for (const uint32_t slot : redone) {
      const CachedBodyPair& pair = cache.at(slot);

      if (pair.points.empty() != ((flags[slot] & startedEmpty) != 0u)
          || fitsWideLane(pair) != ((flags[slot] & startedWide) != 0u)) {
        events.changed.push_back(slot);
      }
    }
  }

  std::sort(events.changed.begin(), events.changed.end());

  for (const uint32_t slot : events.changed) {
    const CachedBodyPair& pair = cache.at(slot);

    if (pair.points.empty() && pair.second.side == Side::Rigid
        && movable(bodies, pair.first) && movable(bodies, pair.second)) {
      events.lost.push_back(pair.firstId);
    }
  }

  return events;
}

double normalForceOn(const World& world, const BodyRef& ref) {
  double load = 0.0;

  for (const Contact& contact : contactsOf(world, ref)) {
    load += contact.memory.normal;
  }

  return load;
}

}
