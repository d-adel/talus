#include "edits.h"

#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/jointrecords.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/tiers/tiers.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace talus::dynamics {

namespace {

bool switchable(const World& world, uint32_t id) {
  return std::any_of(
    world.bodies.switchables.begin(),
    world.bodies.switchables.end(),
    [id](const SwitchableBody& body) { return body.id == id; }
  );
}

bool frozenAt(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid && bodies.rigids[ref.index].frozenMass > 0.0;
}

using Frozen = std::pair<uint32_t, BodyRef>;

bool thawFrom(World& world, std::vector<Frozen> pending) {
  bool thawed = false;

  while (!pending.empty()) {
    const auto [id, ref] = pending.back();

    pending.pop_back();

    if (!frozenAt(world.bodies, ref)) {
      continue;
    }

    RigidBody& body = world.bodies.rigids[ref.index];

    body.mass = body.frozenMass;
    body.moment = body.frozenMoment;
    body.frozenMass = 0.0;
    body.frozenMoment = glm::dvec3(0.0);
    body.velocity = glm::dvec3(0.0);
    body.angularVelocity = glm::dvec3(0.0);
    body.previousVelocity = glm::dvec3(0.0);
    body.sleepTime = 0.0;
    body.asleep = false;
    thawed = true;

    for (const uint32_t slot : world.manifolds.pairsOf(id)) {
      if (!world.manifolds.live(slot)) {
        continue;
      }

      const CachedBodyPair& pair = world.manifolds.at(slot);

      if (pair.points.empty()) {
        continue;
      }

      const BodyRef& other = pair.firstId == id ? pair.second : pair.first;

      if (frozenAt(world.bodies, other)) {
        pending.emplace_back(pair.firstId == id ? pair.secondId : pair.firstId, other);
      } else {
        wakeBody(world.bodies, other);
      }
    }
  }

  return thawed;
}

void thawTouching(World& world, const BodyRef& ref, uint32_t id) {
  std::vector<Frozen> frozen{Frozen{id, ref}};

  for (const uint32_t slot : world.manifolds.pairsOf(id)) {
    if (world.manifolds.live(slot)) {
      const CachedBodyPair& pair = world.manifolds.at(slot);

      if (pair.points.empty()) {
        continue;
      }

      const BodyRef& other = pair.firstId == id ? pair.second : pair.first;

      if (frozenAt(world.bodies, other)) {
        frozen.emplace_back(pair.firstId == id ? pair.secondId : pair.firstId, other);
      }
    }
  }

  static_cast<void>(thawFrom(world, std::move(frozen)));
}

void wakeTouching(World& world, const BodyRef& ref, uint32_t id) {
  BodySet& bodies = world.bodies;
  std::vector<Frozen> frozen;

  if (frozenAt(bodies, ref)) {
    frozen.emplace_back(id, ref);
  }

  for (const uint32_t slot : world.manifolds.pairsOf(id)) {
    if (world.manifolds.live(slot)) {
      const CachedBodyPair& pair = world.manifolds.at(slot);

      if (pair.points.empty()) {
        continue;
      }

      for (const auto& [end, endId] : {std::pair(pair.first, pair.firstId), std::pair(pair.second, pair.secondId)}) {
        if (frozenAt(bodies, end)) {
          frozen.emplace_back(endId, end);
        } else {
          wakeBody(bodies, end);
        }
      }
    }
  }

  static_cast<void>(thawFrom(world, std::move(frozen)));

  for (const Contact& contact : world.contacts) {
    if (contact.a.side == ref.side && contact.a.body == ref.index) {
      wakeBody(bodies, BodyRef{contact.b.side, contact.b.body});
    }

    if (contact.b.side == ref.side && contact.b.body == ref.index) {
      wakeBody(bodies, BodyRef{contact.a.side, contact.a.body});
    }
  }

  wakeBody(bodies, ref);
}

void thawStaticContacts(World& world, uint32_t index) {
  std::vector<Frozen> frozen;

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot)) {
      continue;
    }

    const CachedBodyPair& pair = world.manifolds.at(slot);

    if (!pair.points.empty() && pair.second.side == Side::Static && pair.second.index == index && frozenAt(world.bodies, pair.first)) {
      frozen.emplace_back(pair.firstId, pair.first);
    }
  }

  for (const Contact& contact : world.contacts) {
    for (const auto& [support, other] : {std::pair(contact.a, contact.b), std::pair(contact.b, contact.a)}) {
      const BodyRef ref{other.side, other.body};

      if (support.side == Side::Static && support.body == index && frozenAt(world.bodies, ref)) {
        frozen.emplace_back(world.bodies.rigids[ref.index].id, ref);
      }
    }
  }

  static_cast<void>(thawFrom(world, std::move(frozen)));
}

bool movable(const BodySet& bodies, const BodyRef& ref) {
  return ref.side == Side::Rigid && bodies.rigids[ref.index].mass > 0.0;
}

void reframe(
  RigidBody& body,
  std::span<const CompoundChild> children,
  const PrincipalFrame& frame
) {
  const glm::dvec3 shift = glm::mat3_cast(body.rotation) * frame.centre;

  body.compound = makeCompoundShape(childrenInFrame(children, frame.centre, frame.rotation), body.compound->revision + 1u);
  body.velocity += glm::cross(body.angularVelocity, shift);
  body.previousVelocity = body.velocity;
  body.position += shift;
  body.rotation = glm::normalize(body.rotation * frame.rotation);
  body.mass = frame.mass;
  body.moment = frame.moment;
  body.initialPosition = body.position;
  body.inertialPosition = body.position;
  body.initialRotation = body.rotation;
  body.inertialRotation = body.rotation;
}

JointFrame oldToNew(const PrincipalFrame& frame) {
  const glm::dquat back = glm::conjugate(frame.rotation);

  return {back * -frame.centre, back};
}

bool fills(const CompoundChild& child, std::span<const CompoundChild> parts) {
  if (parts.empty()) {
    return false;
  }

  const glm::dquat back = glm::conjugate(child.rotation);
  const double tolerance = 1.0e-9 * std::max({child.half.x, child.half.y, child.half.z});
  const double whole = 8.0 * child.half.x * child.half.y * child.half.z;
  std::vector<Bounds> boxes;
  double volume = 0.0;

  boxes.reserve(parts.size());

  for (const CompoundChild& part : parts) {
    if (!(part.half.x > 0.0 && part.half.y > 0.0 && part.half.z > 0.0) || part.density != child.density
        || part.rotation != child.rotation) {
      return false;
    }

    const glm::dvec3 offset = back * (part.centre - child.centre);
    const Bounds box{offset - part.half, offset + part.half};

    if (glm::any(glm::lessThan(box.low, -child.half - glm::dvec3(tolerance)))
        || glm::any(glm::greaterThan(box.high, child.half + glm::dvec3(tolerance)))) {
      return false;
    }

    boxes.push_back(box);
    volume += 8.0 * part.half.x * part.half.y * part.half.z;
  }

  if (!(std::abs(volume - whole) <= 1.0e-12 * whole)) {
    return false;
  }

  for (size_t a = 0u; a != boxes.size(); ++a) {
    for (size_t b = a + 1u; b != boxes.size(); ++b) {
      const glm::dvec3 shared = glm::max(glm::min(boxes[a].high, boxes[b].high) - glm::max(boxes[a].low, boxes[b].low), glm::dvec3(0.0));

      if (shared.x * shared.y * shared.z > 1.0e-12 * whole) {
        return false;
      }
    }
  }

  return true;
}

void dropChildManifolds(
  World& world,
  uint32_t id,
  std::span<const uint32_t> keys,
  bool reframed
) {
  const std::span<const uint32_t> listed = world.manifolds.pairsOf(id);
  const std::vector<uint32_t> slots(listed.begin(), listed.end());

  for (const uint32_t slot : slots) {
    const bool touched = !world.manifolds.at(slot).points.empty();

    if (world.manifolds.dropChildren(world.bodies, slot, id, keys, reframed) == 0u) {
      continue;
    }

    const CachedBodyPair& pair = world.manifolds.at(slot);

    wakeBody(world.bodies, pair.firstId == id ? pair.second : pair.first);

    if (touched && pair.points.empty() && pair.second.side == Side::Rigid
        && movable(world.bodies, pair.first) && movable(world.bodies, pair.second)) {
      world.islandStore.markRemoved(pair.firstId);
    }
  }
}

}

void dropContacts(World& world, Side side, uint32_t index, bool renumber) {
  std::vector<Contact> contacts;

  contacts.reserve(world.contacts.size());

  for (Contact contact : world.contacts) {
    const bool touches = (contact.a.side == side && contact.a.body == index)
                         || (contact.b.side == side && contact.b.body == index);

    if (touches) {
      continue;
    }

    if (renumber) {
      for (ContactSide* other : {&contact.a, &contact.b}) {
        if (other->side == side && other->body > index) {
          --other->body;
        }
      }
    }

    contacts.push_back(contact);
  }

  world.contacts = std::move(contacts);
}

void destroyBodyContacts(World& world, uint32_t id) {
  world.manifolds.removeBody(id);
  world.broadPhase.markMoved(id);
  world.islandStore.markRemoved(id);
}

void resetContacts(World& world) {
  world.manifolds = ManifoldCache{};
  world.broadPhase = BroadPhase{};
  world.islandStore = IslandStore{};
}

void forgetBody(World& world, Side side, uint32_t index, uint32_t id) {
  dropContacts(world, side, index, true);
  destroyBodyContacts(world, id);
  world.manifolds.renumber(side, index);
}

bool removeBody(World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref) {
    return false;
  }

  wakeTouching(world, *ref, id);
  removeBodyJoints(world, std::span<const uint32_t>(&id, 1u));

  if (ref->side == Side::Rigid) {
    world.bodies.rigids.erase(world.bodies.rigids.begin() + ref->index);
  } else {
    world.bodies.lattices.erase(world.bodies.lattices.begin() + ref->index);
  }

  forgetBody(world, ref->side, ref->index, id);
  std::erase_if(world.bodies.switchables, [id](const SwitchableBody& body) {
    return body.id == id;
  });

  return true;
}

uint32_t removeBodies(World& world, std::span<const uint32_t> ids) {
  if (ids.empty()) {
    return 0u;
  }

  std::vector<uint32_t> indexOf(world.bodies.nextId, noBody);

  for (uint32_t index = 0u; index != world.bodies.rigids.size(); ++index) {
    indexOf[world.bodies.rigids[index].id] = index;
  }

  std::vector<uint8_t> gone(world.bodies.rigids.size(), 0u);
  std::vector<uint32_t> removed;

  for (const uint32_t id : ids) {
    if (id < indexOf.size() && indexOf[id] != noBody && gone[indexOf[id]] == 0u) {
      gone[indexOf[id]] = 1u;
      removed.push_back(id);
    }
  }

  if (removed.empty()) {
    return 0u;
  }

  removeBodyJoints(world, removed);

  std::vector<uint8_t> pairs(world.manifolds.size(), 0u);
  std::vector<Frozen> thawing;
  const auto wakeRemaining = [&](const BodyRef& ref) {
    if (ref.side == Side::Rigid && gone[ref.index] != 0u) {
      return;
    }

    if (frozenAt(world.bodies, ref)) {
      thawing.emplace_back(world.bodies.rigids[ref.index].id, ref);
    } else {
      wakeBody(world.bodies, ref);
    }
  };

  for (const uint32_t id : removed) {
    for (const uint32_t slot : world.manifolds.pairsOf(id)) {
      pairs[slot] = 1u;
    }

    world.broadPhase.markMoved(id);
    world.islandStore.markRemoved(id);
  }

  for (uint32_t slot = 0u; slot != pairs.size(); ++slot) {
    if (pairs[slot] == 0u || !world.manifolds.live(slot)) {
      continue;
    }

    const CachedBodyPair& pair = world.manifolds.at(slot);

    if (!pair.points.empty()) {
      wakeRemaining(pair.first);
      wakeRemaining(pair.second);
    }

    world.manifolds.destroyPair(slot);
  }

  std::vector<uint32_t> moved(world.bodies.rigids.size(), noBody);
  uint32_t kept = 0u;

  for (uint32_t index = 0u; index != moved.size(); ++index) {
    if (gone[index] == 0u) {
      moved[index] = kept++;
    }
  }

  std::vector<Contact> contacts;

  contacts.reserve(world.contacts.size());

  for (Contact contact : world.contacts) {
    const bool touches = (contact.a.side == Side::Rigid && gone[contact.a.body] != 0u)
                         || (contact.b.side == Side::Rigid && gone[contact.b.body] != 0u);

    if (touches) {
      wakeRemaining(BodyRef{contact.a.side, contact.a.body});
      wakeRemaining(BodyRef{contact.b.side, contact.b.body});
      continue;
    }

    for (ContactSide* end : {&contact.a, &contact.b}) {
      if (end->side == Side::Rigid) {
        end->body = moved[end->body];
      }
    }

    contacts.push_back(contact);
  }

  world.contacts = std::move(contacts);
  static_cast<void>(thawFrom(world, std::move(thawing)));

  uint32_t at = 0u;

  std::erase_if(world.bodies.rigids, [&](const RigidBody&) {
    return gone[at++] != 0u;
  });
  world.manifolds.renumber(Side::Rigid, moved);
  std::erase_if(world.bodies.switchables, [&](const SwitchableBody& body) {
    return body.id < indexOf.size() && indexOf[body.id] != noBody && gone[indexOf[body.id]] != 0u;
  });

  return static_cast<uint32_t>(removed.size());
}

bool removeStatic(World& world, uint32_t id) {
  const std::optional<uint32_t> found = findStatic(world.bodies, id);

  if (!found) {
    return false;
  }

  const uint32_t index = *found;

  thawStaticContacts(world, index);
  world.bodies.statics.erase(world.bodies.statics.begin() + index);
  wakeAll(world.bodies);
  world.contacts.clear();
  resetContacts(world);

  return true;
}

bool setRigidPose(
  World& world,
  uint32_t id,
  const glm::dvec3& position,
  const glm::dquat& rotation
) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid) {
    return false;
  }

  RigidBody& body = world.bodies.rigids[ref->index];
  const glm::dquat turn = glm::normalize(rotation);

  body.position = position;
  body.rotation = turn;
  body.initialPosition = position;
  body.inertialPosition = position;
  body.initialRotation = turn;
  body.inertialRotation = turn;
  wakeTouching(world, *ref, id);
  wakeJointPartners(world, id);
  dropContacts(world, Side::Rigid, ref->index, false);
  destroyBodyContacts(world, id);

  return true;
}

bool setVelocity(World& world, uint32_t id, const glm::dvec3& velocity) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref) {
    return false;
  }

  wakeBody(world.bodies, *ref);
  wakeJointPartners(world, id);
  clearPogoMotion(world, id);

  if (ref->side == Side::Rigid) {
    if (frozenAt(world.bodies, *ref)) {
      static_cast<void>(thawBody(world, id));
    }

    RigidBody& body = world.bodies.rigids[ref->index];

    body.velocity = velocity;
    body.previousVelocity = velocity;

    return true;
  }

  LatticeBody& body = world.bodies.lattices[ref->index];
  const talus::PreparedMesh& mesh = *body.mesh;
  const auto held = [&mesh](size_t node) {
    return !mesh.anchored.empty() && mesh.anchored[node] != 0u;
  };
  double mass = 0.0;
  glm::dvec3 momentum(0.0);

  for (size_t node = 0u; node != body.velocity.size(); ++node) {
    if (held(node)) {
      continue;
    }

    const double weight = static_cast<double>(mesh.nodeMass[node]);

    mass += weight;
    momentum += weight * body.velocity[node];
  }

  if (!(mass > 0.0)) {
    return false;
  }

  const glm::dvec3 shift = velocity - momentum / mass;

  for (size_t node = 0u; node != body.velocity.size(); ++node) {
    if (!held(node)) {
      body.velocity[node] += shift;
    }
  }

  body.previousVelocity = body.velocity;

  return true;
}

bool applyImpulse(World& world, uint32_t id, const glm::dvec3& impulse, const glm::dvec3& point) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid) {
    return false;
  }

  if (frozenAt(world.bodies, *ref)) {
    static_cast<void>(thawBody(world, id));
  }

  RigidBody& body = world.bodies.rigids[ref->index];

  if (!(body.mass > 0.0)) {
    return false;
  }
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  glm::dmat3 inverse(0.0);

  for (int axis = 0; axis != 3; ++axis) {
    inverse[axis][axis] = body.moment[axis] > 0.0 ? 1.0 / body.moment[axis] : 0.0;
  }

  body.velocity += impulse / body.mass;
  body.angularVelocity += turn * inverse * glm::transpose(turn) * glm::cross(point - body.position, impulse);
  body.previousVelocity = body.velocity;
  wakeBody(world.bodies, *ref);

  return true;
}

bool setAngularVelocity(World& world, uint32_t id, const glm::dvec3& spin) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid) {
    return false;
  }

  if (frozenAt(world.bodies, *ref)) {
    static_cast<void>(thawBody(world, id));
  }

  if (!(world.bodies.rigids[ref->index].mass > 0.0)) {
    return false;
  }

  world.bodies.rigids[ref->index].angularVelocity = spin;
  wakeBody(world.bodies, *ref);

  return true;
}

bool setMass(World& world, uint32_t id, double mass) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid || switchable(world, id) || !(mass > 0.0)) {
    return false;
  }

  if (frozenAt(world.bodies, *ref)) {
    static_cast<void>(thawBody(world, id));
  }

  RigidBody& body = world.bodies.rigids[ref->index];

  if (!(body.mass > 0.0)) {
    return false;
  }

  wakeBody(world.bodies, *ref);
  body.moment *= mass / body.mass;
  body.mass = mass;

  return true;
}

bool freezeBody(World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  return ref.has_value() && freezeBody(world, *ref);
}

bool freezeBody(World& world, const BodyRef& ref) {
  if (ref.side != Side::Rigid || ref.index >= world.bodies.rigids.size() || switchable(world, world.bodies.rigids[ref.index].id)) {
    return false;
  }

  RigidBody& body = world.bodies.rigids[ref.index];

  if (!(body.mass > 0.0)) {
    return false;
  }

  body.frozenMass = body.mass;
  body.frozenMoment = body.moment;
  body.mass = 0.0;
  body.moment = glm::dvec3(0.0);
  body.velocity = glm::dvec3(0.0);
  body.angularVelocity = glm::dvec3(0.0);
  body.previousVelocity = glm::dvec3(0.0);
  body.sleepTime = 0.0;
  body.asleep = false;

  return true;
}

bool thawBody(World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  return ref.has_value() && thawFrom(world, {Frozen{id, *ref}});
}

bool isFrozen(const RigidBody& body) {
  return body.frozenMass > 0.0;
}

uint32_t thawStruck(World& world, double speed) {
  std::vector<Frozen> struck;

  for (uint32_t index = 0u; index != world.bodies.rigids.size(); ++index) {
    const RigidBody& frozen = world.bodies.rigids[index];

    if (!isFrozen(frozen)) {
      continue;
    }

    const double inverseFrozenMass = 1.0 / frozen.frozenMass;
    const glm::dvec3 inverseMoment{
      frozen.frozenMoment.x > 0.0 ? 1.0 / frozen.frozenMoment.x : 0.0,
      frozen.frozenMoment.y > 0.0 ? 1.0 / frozen.frozenMoment.y : 0.0,
      frozen.frozenMoment.z > 0.0 ? 1.0 / frozen.frozenMoment.z : 0.0
    };
    const double reach = frozen.compound != nullptr ? frozen.compound->reach : frozen.radius;
    bool hit = false;

    for (const uint32_t slot : world.manifolds.pairsOf(frozen.id)) {
      if (hit || !world.manifolds.live(slot)) {
        continue;
      }

      world.manifolds.visitPoints(slot, [&](const CachedPointView& view) {
        if (hit) {
          return;
        }

        const bool first = view.a.side == Side::Rigid && world.bodies.rigids[view.a.index].id == frozen.id;
        const BodyRef& other = first ? view.b : view.a;

        if (other.side != Side::Rigid) {
          return;
        }

        const RigidBody& coming = world.bodies.rigids[other.index];

        if (!(coming.mass > 0.0)) {
          return;
        }

        if (std::find(world.thawOnContact.begin(), world.thawOnContact.end(), coming.id) != world.thawOnContact.end()) {
          hit = true;
          return;
        }

        const glm::dvec3 local = first ? view.point.localB : view.point.localA;
        const glm::dvec3 offset = coming.rotation * local;
        const glm::dvec3 velocity = coming.velocity + glm::cross(coming.angularVelocity, offset);
        const double toward = glm::dot(velocity, view.manifold.basis[0]) * (first ? 1.0 : -1.0);

        if (!(toward > 0.0)) {
          return;
        }

        const double inverseTranslation = 1.0 / coming.mass + inverseFrozenMass;
        const double approach = (1.0 + world.manifolds.at(slot).restitution) * toward;
        const double translationSpeed = approach / inverseTranslation * inverseFrozenMass;
        const glm::dvec3 frozenOffset = frozen.rotation * (first ? view.point.localA : view.point.localB);
        const glm::dvec3 lever = glm::cross(frozenOffset, view.manifold.basis[0]);
        const glm::dvec3 angular = frozen.rotation * (inverseMoment * (glm::conjugate(frozen.rotation) * lever));
        const double impulse = approach / (inverseTranslation + glm::dot(lever, angular));
        const double pointSpeed = impulse * (inverseFrozenMass + glm::length(angular) * reach);

        // Centered manifolds transfer translation; off-center hits can move a beam's tip faster.
        // Both estimates omit the incoming body's rotational compliance, conservatively favoring thaw.
        hit = std::max(translationSpeed, pointSpeed) > speed;
      });
    }

    if (hit) {
      struck.emplace_back(frozen.id, BodyRef{Side::Rigid, index});
    }
  }

  const auto thawed = static_cast<uint32_t>(struck.size());

  static_cast<void>(thawFrom(world, std::move(struck)));

  return thawed;
}

bool replaceRigidShape(World& world, uint32_t id, const RigidBody& shape) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid || switchable(world, id)) {
    return false;
  }

  wakeTouching(world, *ref, id);
  RigidBody& body = world.bodies.rigids[ref->index];

  body.field = shape.field;
  body.samples = shape.samples;
  body.mass = shape.mass;
  body.moment = shape.moment;
  body.radius = shape.radius;
  body.shape = shape.shape;
  body.halfExtents = shape.halfExtents;
  body.compound = shape.compound;
  body.fieldOrigin = shape.fieldOrigin;
  body.fieldRotation = shape.fieldRotation;
  dropContacts(world, Side::Rigid, ref->index, false);
  destroyBodyContacts(world, id);

  return true;
}

bool setStaticPose(
  World& world,
  uint32_t id,
  const glm::dvec3& position,
  const glm::dquat& rotation
) {
  const std::optional<uint32_t> found = findStatic(world.bodies, id);

  if (!found) {
    return false;
  }

  const uint32_t index = *found;
  thawStaticContacts(world, index);
  StaticField& body = world.bodies.statics[index];

  body.position = position;
  body.rotation = glm::normalize(rotation);
  wakeAll(world.bodies);
  dropContacts(world, Side::Static, index, false);
  resetContacts(world);

  return true;
}

bool removeCompoundChildren(World& world, uint32_t id, std::span<const uint32_t> keys) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid || keys.empty()
      || world.bodies.rigids[ref->index].shape != RigidShape::Compound) {
    return false;
  }

  thawTouching(world, *ref, id);

  RigidBody& body = world.bodies.rigids[ref->index];
  const CompoundShape& before = *body.compound;
  std::vector<uint32_t> removed(keys.begin(), keys.end());

  std::sort(removed.begin(), removed.end());

  const bool unknown = std::any_of(removed.begin(), removed.end(), [&](uint32_t key) {
    return !findChild(before, key).has_value();
  });

  if (unknown || std::adjacent_find(removed.begin(), removed.end()) != removed.end()
      || removed.size() >= before.children.size()) {
    return false;
  }

  const bool reframed = body.mass > 0.0;

  if (reframed) {
    std::vector<CompoundChild> remaining;

    for (const CompoundChild& child : before.children) {
      if (!std::binary_search(removed.begin(), removed.end(), child.key)) {
        remaining.push_back(child);
      }
    }

    const PrincipalFrame frame = principalFrame(remaining);

    if (!(frame.mass > 0.0)) {
      return false;
    }

    reframe(body, remaining, frame);
    wakeBody(world.bodies, *ref);

    if (!world.joints.empty()) {
      const BodyRebase rebase{id, oldToNew(frame), body.compound->children};

      rebaseJoints(world, id, std::span<const BodyRebase>(&rebase, 1u));
    }
  } else {
    removeChildren(body.compound, removed);
  }

  dropChildManifolds(world, id, removed, reframed);
  world.broadPhase.markMoved(id);

  return true;
}

std::vector<uint32_t> splitCompound(
  World& world,
  uint32_t id,
  std::span<const std::vector<uint32_t>> groups
) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid || groups.size() < 2u
      || world.bodies.rigids[ref->index].shape != RigidShape::Compound) {
    return {};
  }

  thawTouching(world, *ref, id);

  const CompoundShape& before = *world.bodies.rigids[ref->index].compound;
  std::vector<std::vector<uint32_t>> sorted(groups.begin(), groups.end());
  std::vector<uint32_t> all;

  for (std::vector<uint32_t>& group : sorted) {
    if (group.empty()) {
      return {};
    }

    std::sort(group.begin(), group.end());
    all.insert(all.end(), group.begin(), group.end());
  }

  std::sort(all.begin(), all.end());

  if (all.size() != before.children.size()) {
    return {};
  }

  for (size_t index = 0u; index != all.size(); ++index) {
    if (all[index] != before.children[index].key) {
      return {};
    }
  }

  std::sort(sorted.begin(), sorted.end(), [](const std::vector<uint32_t>& a, const std::vector<uint32_t>& b) {
    return a.front() < b.front();
  });

  size_t kept = 0u;

  for (size_t index = 1u; index != sorted.size(); ++index) {
    if (sorted[index].size() > sorted[kept].size()) {
      kept = index;
    }
  }

  RigidBody& parent = world.bodies.rigids[ref->index];
  const bool moving = parent.mass > 0.0;
  std::vector<std::vector<CompoundChild>> parts;
  std::vector<PrincipalFrame> frames;

  for (size_t index = 0u; index != sorted.size(); ++index) {
    std::vector<CompoundChild> children;

    if (index == kept && !moving) {
      frames.emplace_back();
      parts.push_back(std::move(children));
      continue;
    }

    for (const uint32_t key : sorted[index]) {
      children.push_back(before.children[*findChild(before, key)]);
    }

    frames.push_back(principalFrame(children));
    parts.push_back(std::move(children));

    if (!(frames.back().mass > 0.0)) {
      return {};
    }
  }

  const glm::dvec3 position = parent.position;
  const glm::dquat rotation = parent.rotation;
  const glm::dmat3 turn = glm::mat3_cast(rotation);
  const glm::dvec3 velocity = parent.velocity;
  const glm::dvec3 spin = parent.angularVelocity;
  std::vector<RigidBody> pieces;
  std::vector<uint32_t> departed;

  for (size_t index = 0u; index != sorted.size(); ++index) {
    if (index == kept) {
      continue;
    }

    const PrincipalFrame& frame = frames[index];
    const glm::dvec3 shift = turn * frame.centre;
    RigidBody piece;

    piece.shape = RigidShape::Compound;
    piece.compound = makeCompoundShape(childrenInFrame(parts[index], frame.centre, frame.rotation));
    piece.position = position + shift;
    piece.rotation = glm::normalize(rotation * frame.rotation);
    piece.mass = frame.mass;
    piece.moment = frame.moment;
    piece.velocity = moving ? velocity + glm::cross(spin, shift) : glm::dvec3(0.0);
    piece.angularVelocity = moving ? spin : glm::dvec3(0.0);
    piece.previousVelocity = piece.velocity;
    piece.initialPosition = piece.position;
    piece.inertialPosition = piece.position;
    piece.initialRotation = piece.rotation;
    piece.inertialRotation = piece.rotation;
    piece.friction = parent.friction;
    piece.restitution = parent.restitution;
    pieces.push_back(std::move(piece));
    departed.insert(departed.end(), sorted[index].begin(), sorted[index].end());
  }

  std::sort(departed.begin(), departed.end());

  if (moving) {
    reframe(parent, parts[kept], frames[kept]);
    wakeBody(world.bodies, *ref);
  } else {
    removeChildren(parent.compound, departed);
  }

  dropChildManifolds(world, id, departed, moving);
  world.broadPhase.markMoved(id);

  std::vector<uint32_t> ids;

  for (RigidBody& piece : pieces) {
    piece.id = world.bodies.nextId++;
    ids.push_back(piece.id);
    world.bodies.rigids.push_back(std::move(piece));
  }

  if (!world.joints.empty()) {
    const RigidBody& keeper = world.bodies.rigids[ref->index];
    std::vector<BodyRebase> rebases{
      BodyRebase{id, oldToNew(frames[kept]), keeper.compound->children}
    };
    size_t made = world.bodies.rigids.size() - ids.size();

    for (size_t index = 0u; index != sorted.size(); ++index) {
      if (index != kept) {
        const RigidBody& piece = world.bodies.rigids[made++];
        const JointFrame shift = oldToNew(frames[index]);

        rebases.push_back({piece.id, shift, piece.compound->children});
      }
    }

    rebaseJoints(world, id, rebases);
  }

  return ids;
}

bool replaceCompoundChildren(World& world, uint32_t id, std::span<const ChildReplacement> replacements) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  if (!ref || ref->side != Side::Rigid || replacements.empty()
      || world.bodies.rigids[ref->index].shape != RigidShape::Compound) {
    return false;
  }

  thawTouching(world, *ref, id);

  RigidBody& body = world.bodies.rigids[ref->index];
  const CompoundShape& shape = *body.compound;
  std::vector<uint32_t> keys;
  std::vector<uint32_t> fresh;

  for (const ChildReplacement& replacement : replacements) {
    const std::optional<uint32_t> index = findChild(shape, replacement.key);

    if (!index.has_value() || !fills(shape.children[*index], replacement.parts)) {
      return false;
    }

    keys.push_back(replacement.key);

    for (const CompoundChild& part : replacement.parts) {
      fresh.push_back(part.key);
    }
  }

  std::sort(keys.begin(), keys.end());
  std::sort(fresh.begin(), fresh.end());

  const bool reused = std::any_of(fresh.begin(), fresh.end(), [&shape](uint32_t key) {
    return findChild(shape, key).has_value();
  });

  if (reused || std::adjacent_find(keys.begin(), keys.end()) != keys.end()
      || std::adjacent_find(fresh.begin(), fresh.end()) != fresh.end()) {
    return false;
  }

  replaceChildren(body.compound, replacements);
  dropChildManifolds(world, id, keys, false);

  return true;
}

}
