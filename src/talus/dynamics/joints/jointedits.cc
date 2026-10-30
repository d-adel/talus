#include <talus/dynamics/joints/jointedits.h>

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
  const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

  return finite(frame.position) && finite(length) && length > 1e-12;
}

bool withinTurn(double angle, double reach) {
  return finite(angle) && std::abs(angle) <= reach * std::numbers::pi;
}

bool validAxis(const AxisDrive& axis) {
  return withinTurn(axis.targetAngle, 1.0) && finite(axis.targetVelocity) && validGain(axis.stiffness)
    && validGain(axis.damping) && !std::isnan(axis.minTorque) && !std::isnan(axis.maxTorque)
    && axis.minTorque <= axis.maxTorque;
}

bool validDrive(const JointDrive& drive) {
  for (const AxisDrive& axis : drive.axes) {
    if (!validAxis(axis)) {
      return false;
    }
  }

  const glm::dvec2 swing(drive.axes[1].targetAngle, drive.axes[2].targetAngle);

  return withinTurn(glm::length(swing), 0.99) && validCap(drive.maxTorque) && finite(drive.feedForward);
}

bool validHalfAngle(double angle) {
  return angle >= 1.0e-3 && withinTurn(angle, 0.99);
}

bool validCone(const ConeLimit& cone) {
  return finite(cone.twistLower) && finite(cone.twistUpper) && finite(cone.swing)
    && cone.swing.x >= 0.0 && cone.swing.y >= 0.0 && finite(cone.centre)
    && (!cone.enabled
        || (cone.twistLower <= cone.twistUpper && withinTurn(cone.twistLower, 0.99)
            && withinTurn(cone.twistUpper, 0.99) && validHalfAngle(cone.swing.x)
            && validHalfAngle(cone.swing.y)
            && withinTurn(glm::length(cone.centre) + std::max(cone.swing.x, cone.swing.y), 0.99)));
}

bool validDef(const JointDef& def) {
  const SpringSettings& spring = def.spring;

  return validFrame(def.frameA) && validFrame(def.frameB) && validFrame(spring.target)
    && finite(def.hertz) && def.hertz > 0.0 && finite(def.dampingRatio)
    && def.dampingRatio >= 0.0 && finite(def.hinge.lower) && finite(def.hinge.upper)
    && (!def.hinge.enabled
        || (def.hinge.lower <= def.hinge.upper && withinTurn(def.hinge.lower, 0.99) && withinTurn(def.hinge.upper, 0.99)))
    && validCone(def.cone) && validDrive(def.drive) && validGain(spring.linearStiffness)
    && validGain(spring.linearDamping) && validCap(spring.maxForce)
    && validGain(spring.angularStiffness) && validGain(spring.angularDamping)
    && validCap(spring.maxTorque);
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
  wakeJointBodies(world, stored);

  if (excludesPair(stored)) {
    world.manifolds.forgetPair(stored.bodyA, stored.bodyB);
  }

  return world.joints.add(stored);
}

bool removeJoint(World& world, uint32_t joint) {
  const Joint* found = world.joints.find(joint);

  if (found == nullptr) {
    return false;
  }

  const JointDef def = found->def;

  wakeJointBodies(world, def);
  world.joints.remove(joint);

  if (def.bodyA != noBody) {
    world.islandStore.markRemoved(def.bodyB);
    world.islandStore.requestSplit(def.bodyB);
  }

  if (excludesPair(def) && !world.joints.excludes(def.bodyA, def.bodyB)) {
    world.broadPhase.markMoved(def.bodyA);
    world.broadPhase.markMoved(def.bodyB);
  }

  return true;
}

bool setJointDrive(World& world, uint32_t joint, const JointDrive& drive) {
  Joint* found = world.joints.find(joint);

  if (found == nullptr || !validDrive(drive)) {
    return false;
  }

  found->def.drive = drive;
  wakeJointBodies(world, found->def);

  return true;
}

bool setSpringTarget(World& world, uint32_t joint, const JointFrame& target) {
  Joint* found = world.joints.find(joint);

  if (found == nullptr || !validFrame(target)) {
    return false;
  }

  found->def.spring.target = normalised(target);
  wakeJointBodies(world, found->def);

  return true;
}

std::optional<JointReport> jointReport(const World& world, uint32_t joint) {
  const Joint* found = world.joints.find(joint);

  if (found == nullptr) {
    return std::nullopt;
  }

  return found->report;
}

void pruneJoints(World& world) {
  if (world.joints.empty()) {
    return;
  }

  std::vector<uint32_t> dead;

  for (const Joint& joint : world.joints.all()) {
    const bool aLive = joint.def.bodyA == noBody || resolves(world, joint.def.bodyA);

    if (!aLive || !resolves(world, joint.def.bodyB)) {
      dead.push_back(joint.id);
    }
  }

  for (const uint32_t id : dead) {
    world.joints.remove(id);
  }
}

}
