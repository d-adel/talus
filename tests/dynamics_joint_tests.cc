#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace {

using namespace talus::dynamics;

talus::field::SampledSdf crateField() {
  const glm::vec3 reach(0.09f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(glm::vec3(0.0f), glm::vec3(0.05f))},
    -reach,
    reach,
    glm::uvec3(10u)
  );
}

uint32_t addCrate(World& world, const talus::field::SampledSdf& crate, const glm::dvec3& at) {
  const uint32_t index =
    addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.5, at, glm::dvec3(0.0), 0.5, 3u);

  return world.bodies.rigids[index].id;
}

JointDef between(uint32_t a, uint32_t b) {
  JointDef def;
  def.bodyA = a;
  def.bodyB = b;

  return def;
}

}

TEST_CASE("joint ids are stable and start at 1", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  const uint32_t c = addCrate(world, crate, glm::dvec3(0.4, 1.0, 0.0));

  CHECK(addJoint(world, between(a, b)) == 1u);
  CHECK(addJoint(world, between(b, c)) == 2u);
  CHECK(removeJoint(world, 1u));
  CHECK_FALSE(removeJoint(world, 1u));
  CHECK(addJoint(world, between(a, c)) == 3u);
  CHECK_FALSE(jointReport(world, 1u).has_value());
  CHECK(jointReport(world, 2u).has_value());
  CHECK(world.joints.all().size() == 2u);
  CHECK(world.joints.all()[0].id == 2u);
  CHECK(world.joints.all()[1].id == 3u);
}

TEST_CASE("joined pairs are excluded until their last joint goes", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  JointDef touching = between(a, b);
  touching.collideConnected = true;

  const uint32_t first = addJoint(world, between(a, b));
  const uint32_t second = addJoint(world, between(b, a));
  CHECK(world.joints.excludes(a, b));
  CHECK(world.joints.excludes(b, a));
  CHECK(removeJoint(world, first));
  CHECK(world.joints.excludes(a, b));
  CHECK(removeJoint(world, second));
  CHECK_FALSE(world.joints.excludes(a, b));

  const uint32_t third = addJoint(world, touching);
  CHECK(third != 0u);
  CHECK_FALSE(world.joints.excludes(a, b));
}

TEST_CASE("invalid joint definitions are rejected", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const auto rejects = [&](const JointDef& def) {
    CHECK(addJoint(world, def) == 0u);
    CHECK(world.joints.empty());
  };

  rejects(between(a, a));
  rejects(between(a, 4000u));
  rejects(between(4000u, b));
  rejects(between(noBody, noBody));
  rejects(between(a, noBody));

  JointDef def = between(a, b);
  def.frameA.rotation = glm::dquat(0.0, 0.0, 0.0, 0.0);
  rejects(def);

  def = between(a, b);
  def.frameB.position.y = nan;
  rejects(def);

  def = between(a, b);
  def.drive.axes[1].stiffness = -1.0;
  rejects(def);

  def = between(a, b);
  def.drive.axes[0].damping = -1.0;
  rejects(def);

  def = between(a, b);
  def.spring.linearStiffness = -1.0;
  rejects(def);

  def = between(a, b);
  def.spring.maxForce = -1.0;
  rejects(def);

  def = between(a, b);
  def.drive.axes[2].minTorque = 2.0;
  def.drive.axes[2].maxTorque = 1.0;
  rejects(def);

  def = between(a, b);
  def.hinge.enabled = true;
  def.hinge.lower = 1.0;
  def.hinge.upper = -1.0;
  rejects(def);

  def = between(a, b);
  def.hertz = 0.0;
  rejects(def);

  CHECK(addJoint(world, between(a, b)) == 1u);
  CHECK(addJoint(world, between(noBody, b)) == 2u);
}

TEST_CASE("setters reject unknown joints and invalid drives", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  const uint32_t joint = addJoint(world, between(a, b));
  REQUIRE(joint == 1u);

  CHECK_FALSE(setJointDrive(world, 99u, JointDrive{}));

  JointDrive bad;
  bad.axes[0].targetAngle = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(setJointDrive(world, joint, bad));

  JointDrive good;
  good.axes[0].stiffness = 5.0;
  CHECK(setJointDrive(world, joint, good));
  CHECK(world.joints.find(joint)->def.drive.axes[0].stiffness == 5.0);

  JointFrame target;
  target.position = glm::dvec3(0.0, 0.1, 0.0);
  CHECK(setSpringTarget(world, joint, target));
  CHECK_FALSE(setSpringTarget(world, 99u, target));
  target.rotation = glm::dquat(0.0, 0.0, 0.0, 0.0);
  CHECK_FALSE(setSpringTarget(world, joint, target));
}

TEST_CASE("editing a joint wakes both bodies", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  const auto sleepAll = [&] {
    for (RigidBody& body : world.bodies.rigids) {
      body.asleep = true;
    }
  };

  sleepAll();
  const uint32_t joint = addJoint(world, between(a, b));
  CHECK_FALSE(world.bodies.rigids[0].asleep);
  CHECK_FALSE(world.bodies.rigids[1].asleep);

  sleepAll();
  CHECK(setJointDrive(world, joint, JointDrive{}));
  CHECK_FALSE(world.bodies.rigids[0].asleep);
  CHECK_FALSE(world.bodies.rigids[1].asleep);

  sleepAll();
  CHECK(setSpringTarget(world, joint, JointFrame{}));
  CHECK_FALSE(world.bodies.rigids[0].asleep);
  CHECK_FALSE(world.bodies.rigids[1].asleep);
}

TEST_CASE("a removed body's joints disappear at the next step", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t a = addCrate(world, crate, glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t b = addCrate(world, crate, glm::dvec3(0.2, 1.0, 0.0));
  const uint32_t joint = addJoint(world, between(a, b));
  REQUIRE(joint != 0u);

  CHECK(removeBody(world, a));
  CHECK(jointReport(world, joint).has_value());

  const StepReport report = step(world);

  CHECK_FALSE(jointReport(world, joint).has_value());
  CHECK(world.joints.empty());
  CHECK_FALSE(world.joints.excludes(a, b));
  CHECK_FALSE(report.nonFinite);
}
