#include <talus/dynamics/joints/jointedits.h>

#include <talus/dynamics/joints/jointrecords.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/tiers/tiers.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace talus::dynamics {

namespace {

bool finite(double value) {
  return std::isfinite(value);
}

bool finite(const glm::dvec3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

bool finite(const glm::dvec2& value) {
  return finite(value.x) && finite(value.y);
}

bool validCap(double value) {
  return !std::isnan(value) && value >= 0.0;
}

bool validGain(double value) {
  return finite(value) && value >= 0.0;
}

bool validFrame(const JointFrame& frame) {
  const glm::dquat& q = frame.rotation;
  const double length =
    std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  return finite(frame.position) && finite(length) && length > 1e-12;
}

constexpr double swingReach = 0.89;

bool withinTurn(double angle, double reach) {
  return finite(angle) && std::abs(angle) <= reach * std::numbers::pi;
}

bool validAxis(const AxisDrive& axis) {
  return withinTurn(axis.targetAngle, 1.0) && finite(axis.targetVelocity)
         && validGain(axis.stiffness) && validGain(axis.damping)
         && axis.minTorque <= 0.0 && axis.maxTorque >= 0.0;
}

bool validDrive(const JointDrive& drive) {
  for (const AxisDrive& axis : drive.axes) {
    if (!validAxis(axis)) {
      return false;
    }
  }

  const glm::dvec2 swing(drive.axes[1].targetAngle, drive.axes[2].targetAngle);

  return withinTurn(glm::length(swing), swingReach) && validCap(drive.maxTorque)
         && finite(drive.feedForward);
}

bool validHalfAngle(double angle) {
  return angle >= 1.0e-3 && withinTurn(angle, swingReach);
}

bool validCone(const ConeLimit& cone) {
  return finite(cone.twistLower) && finite(cone.twistUpper) && finite(cone.swing)
    && cone.swing.x >= 0.0 && cone.swing.y >= 0.0 && finite(cone.centre)
    && (!cone.enabled
        || (cone.twistLower <= cone.twistUpper && withinTurn(cone.twistLower, 0.99)
            && withinTurn(cone.twistUpper, 0.99) && validHalfAngle(cone.swing.x)
            && validHalfAngle(cone.swing.y)
            && withinTurn(glm::length(cone.centre) + std::max(cone.swing.x, cone.swing.y), swingReach)));
}

bool validSpring(const SpringSettings& spring) {
  return validFrame(spring.target) && validGain(spring.linearStiffness)
         && validGain(spring.linearDamping) && validCap(spring.maxForce)
         && validGain(spring.angularStiffness)
         && validGain(spring.angularDamping) && validCap(spring.maxTorque);
}

bool validPlanar(const PlanarVelocitySettings& planar) {
  return finite(planar.targetVelocity) && validCap(planar.maxForce);
}

bool validPogo(const PogoSettings& pogo) {
  const double length = glm::length(pogo.normal);

  return finite(pogo.normal) && finite(length) && length > 1e-12
         && validGain(pogo.restLength) && validGain(pogo.hertz)
         && validGain(pogo.dampingRatio) && validCap(pogo.maxTension)
         && validCap(pogo.maxCompression) && finite(pogo.force)
         && finite(pogo.speed) && finite(pogo.released);
}

bool validCollision(const JointDef& def) {
  return (def.kind != JointKind::PlanarVelocity && def.kind != JointKind::Pogo)
         || def.collideConnected;
}

bool validDef(const JointDef& def) {
  return def.kind <= JointKind::Pogo && validFrame(def.frameA) && validFrame(def.frameB)
    && finite(def.hertz) && def.hertz > 0.0 && finite(def.dampingRatio)
    && def.dampingRatio >= 0.0 && finite(def.hinge.lower) && finite(def.hinge.upper)
    && (!def.hinge.enabled
        || (def.hinge.lower <= def.hinge.upper && withinTurn(def.hinge.lower, 0.99) && withinTurn(def.hinge.upper, 0.99)))
    && validCone(def.cone) && validDrive(def.drive) && validSpring(def.spring)
    && validPlanar(def.planar) && validPogo(def.pogo) && validCollision(def);
}

JointFrame normalised(const JointFrame& frame) {
  JointFrame result = frame;
  result.rotation = glm::normalize(frame.rotation);

  return result;
}

bool resolves(const World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  return ref && ref->side == Side::Rigid;
}

bool standsOn(const World& world, const JointDef& def) {
  if (def.kind != JointKind::Pogo) {
    return true;
  }

  const RigidBody& body =
    world.bodies.rigids[findBody(world.bodies, def.bodyB)->index];
  const glm::dvec3 axis =
    body.rotation * def.frameB.rotation * glm::dvec3(0.0, 1.0, 0.0);

  return glm::dot(def.pogo.normal, axis) >= pogoSteepestCosine;
}

void wakeJointBodies(World& world, const JointDef& def) {
  for (const uint32_t id : {def.bodyA, def.bodyB}) {
    if (id == noBody) {
      continue;
    }

    if (const std::optional<BodyRef> ref = findBody(world.bodies, id)) {
      wakeBody(world.bodies, *ref);
    }
  }
}

void dropJoint(World& world, uint32_t joint, const JointDef& def) {
  wakeJointBodies(world, def);
  JointRecords::remove(world.joints, joint);

  if (excludesPair(def) && !world.joints.excludes(def.bodyA, def.bodyB)) {
    world.broadPhase.markMoved(def.bodyA);
    world.broadPhase.markMoved(def.bodyB);
  }
}

glm::dvec3 carried(const JointFrame& transform, const glm::dvec3& point) {
  return transform.rotation * point + transform.position;
}

double childGap(const CompoundChild& child, const glm::dvec3& point) {
  const glm::dvec3 local =
    glm::conjugate(child.rotation) * (point - child.centre);
  const glm::dvec3 outside =
    glm::max(glm::abs(local) - child.half, glm::dvec3(0.0));

  return glm::dot(outside, outside);
}

const BodyRebase& holderOf(
  std::span<const BodyRebase> pieces,
  const glm::dvec3& anchor
) {
  if (pieces.size() == 1u) {
    return pieces.front();
  }

  const BodyRebase* holder = &pieces.front();
  double nearest = jointInfinity;

  for (const BodyRebase& piece : pieces) {
    const glm::dvec3 point = carried(piece.oldToNew, anchor);

    for (const CompoundChild& child : piece.children) {
      const double gap = childGap(child, point);

      if (gap < nearest) {
        nearest = gap;
        holder = &piece;
      }

      if (nearest == 0.0) {
        return *holder;
      }
    }
  }

  return *holder;
}

}

uint32_t addJoint(World& world, const JointDef& def) {
  if (def.bodyB == noBody || def.bodyA == def.bodyB || !validDef(def)) {
    return 0u;
  }

  if (def.bodyA != noBody && !resolves(world, def.bodyA)) {
    return 0u;
  }

  if (!resolves(world, def.bodyB)) {
    return 0u;
  }

  JointDef stored = def;
  stored.frameA = normalised(def.frameA);
  stored.frameB = normalised(def.frameB);
  stored.spring.target = normalised(def.spring.target);
  stored.pogo.normal = glm::normalize(def.pogo.normal);

  if (!standsOn(world, stored)) {
    return 0u;
  }

  wakeJointBodies(world, stored);

  if (excludesPair(stored)) {
    world.manifolds.forgetPair(stored.bodyA, stored.bodyB);
  }

  return JointRecords::add(world.joints, stored);
}

bool removeJoint(World& world, uint32_t joint) {
  const Joint* found = world.joints.find(joint);

  if (found == nullptr) {
    return false;
  }

  const JointDef def = found->def;

  dropJoint(world, joint, def);

  if (def.bodyA != noBody) {
    world.islandStore.markRemoved(def.bodyB);
    world.islandStore.requestSplit(def.bodyB);
  }

  return true;
}

bool setJointDrive(World& world, uint32_t joint, const JointDrive& drive) {
  Joint* found = JointRecords::find(world.joints, joint);

  if (found == nullptr || !validDrive(drive)) {
    return false;
  }

  found->def.drive = drive;
  wakeJointBodies(world, found->def);

  return true;
}

bool setSpringTarget(World& world, uint32_t joint, const JointFrame& target) {
  Joint* found = JointRecords::find(world.joints, joint);

  if (found == nullptr || !validFrame(target)) {
    return false;
  }

  found->def.spring.target = normalised(target);
  wakeJointBodies(world, found->def);

  return true;
}

bool setSpringSettings(
  World& world,
  uint32_t joint,
  const SpringSettings& spring
) {
  Joint* found = JointRecords::find(world.joints, joint);

  if (found == nullptr || !validSpring(spring)) {
    return false;
  }

  found->def.spring = spring;
  found->def.spring.target = normalised(spring.target);
  wakeJointBodies(world, found->def);

  return true;
}

bool setPlanarVelocity(
  World& world,
  uint32_t joint,
  const PlanarVelocitySettings& planar
) {
  Joint* found = JointRecords::find(world.joints, joint);

  if (found == nullptr || found->def.kind != JointKind::PlanarVelocity
      || !validPlanar(planar)) {
    return false;
  }

  found->def.planar = planar;
  wakeJointBodies(world, found->def);

  return true;
}

void clearPogoMotion(World& world, uint32_t body) {
  for (Joint& joint : JointRecords::all(world.joints)) {
    if (joint.def.kind == JointKind::Pogo && joint.def.bodyB == body) {
      joint.impulses.pogoSpeed = 0.0;
      joint.impulses.pogoReleased = 0.0;
      joint.report.pogoSpeed = 0.0;
      joint.report.pogoReleased = 0.0;
    }
  }
}

void wakeJointPartners(World& world, uint32_t id) {
  for (const Joint& joint : world.joints.all()) {
    if (joint.def.bodyA == id || joint.def.bodyB == id) {
      wakeJointBodies(world, joint.def);
    }
  }
}

void rebaseJoints(
  World& world,
  uint32_t oldBody,
  std::span<const BodyRebase> pieces
) {
  if (pieces.empty()) {
    return;
  }

  uint32_t split = noBody;

  for (Joint& joint : JointRecords::all(world.joints)) {
    const bool onA = joint.def.bodyA == oldBody;

    if (!onA && joint.def.bodyB != oldBody) {
      continue;
    }

    JointFrame& frame = onA ? joint.def.frameA : joint.def.frameB;
    const BodyRebase& piece = holderOf(pieces, frame.position);

    frame.position = carried(piece.oldToNew, frame.position);
    frame.rotation = glm::normalize(piece.oldToNew.rotation * frame.rotation);

    if (piece.body != oldBody) {
      const JointDef before = joint.def;
      const uint32_t other = onA ? before.bodyB : before.bodyA;

      JointRecords::rebind(
        world.joints,
        joint.id,
        onA ? piece.body : other,
        onA ? other : piece.body
      );

      if (excludesPair(before)
          && !world.joints.excludes(before.bodyA, before.bodyB)) {
        world.broadPhase.markMoved(before.bodyA);
        world.broadPhase.markMoved(before.bodyB);
      }

      if (excludesPair(joint.def)) {
        world.manifolds.forgetPair(joint.def.bodyA, joint.def.bodyB);
      }

      if (other != noBody) {
        world.islandStore.markRemoved(other);
        split = other;
      }
    }

    wakeJointBodies(world, joint.def);
  }

  if (split != noBody) {
    world.islandStore.requestSplit(split);
  }
}

std::optional<JointDef> jointDef(const World& world, uint32_t joint) {
  const Joint* found = world.joints.find(joint);

  if (found == nullptr) {
    return std::nullopt;
  }

  return found->def;
}

std::optional<JointReport> jointReport(const World& world, uint32_t joint) {
  const Joint* found = world.joints.find(joint);

  if (found == nullptr) {
    return std::nullopt;
  }

  return found->report;
}

void removeBodyJoints(World& world, std::span<const uint32_t> bodies) {
  if (world.joints.empty() || bodies.empty()) {
    return;
  }

  std::vector<uint32_t> gone(bodies.begin(), bodies.end());
  std::vector<uint32_t> dead;

  std::sort(gone.begin(), gone.end());

  for (const Joint& joint : world.joints.all()) {
    if (std::binary_search(gone.begin(), gone.end(), joint.def.bodyA)
        || std::binary_search(gone.begin(), gone.end(), joint.def.bodyB)) {
      dead.push_back(joint.id);
    }
  }

  for (const uint32_t id : dead) {
    const JointDef def = world.joints.find(id)->def;

    dropJoint(world, id, def);
  }
}

}
