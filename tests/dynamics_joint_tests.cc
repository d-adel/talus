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

talus::field::SampledSdf floorField() {
  const glm::vec3 low(-1.04f, -0.14f, -1.04f);
  const glm::vec3 high(1.04f, 0.34f, 1.04f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(glm::vec3(0.0f), glm::vec3(1.0f, 0.1f, 1.0f))},
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );
}

const talus::field::SampledSdf& floor() {
  static const talus::field::SampledSdf field = floorField();

  return field;
}

void addFloor(World& world) {
  addStatic(world.bodies, floor(), 0.5);
}

uint32_t addResting(World& world, const talus::field::SampledSdf& crate, double x) {
  return addCrate(world, crate, glm::dvec3(x, 0.15, 0.0));
}

bool allAsleep(const World& world) {
  for (const RigidBody& body : world.bodies.rigids) {
    if (!body.asleep) {
      return false;
    }
  }

  return true;
}

bool anyAsleep(const World& world) {
  for (const RigidBody& body : world.bodies.rigids) {
    if (body.asleep) {
      return true;
    }
  }

  return false;
}

bool touching(const World& world, uint32_t a, uint32_t b) {
  const BodyRef ref = *findBody(world.bodies, a);

  for (const Contact& contact : contactsOf(world, ref)) {
    if (contact.a.side == Side::Rigid && contact.b.side == Side::Rigid) {
      const uint32_t idA = world.bodies.rigids[contact.a.body].id;
      const uint32_t idB = world.bodies.rigids[contact.b.body].id;

      if ((idA == a && idB == b) || (idA == b && idB == a)) {
        return true;
      }
    }
  }

  return false;
}

bool cached(const World& world, uint32_t a, uint32_t b) {
  for (const uint32_t slot : world.manifolds.pairsOf(a)) {
    const CachedBodyPair& pair = world.manifolds.at(slot);

    if (world.manifolds.live(slot) && ((pair.firstId == a && pair.secondId == b) || (pair.firstId == b && pair.secondId == a))) {
      return true;
    }
  }

  return false;
}

bool sleepsWithin(World& world, uint32_t steps) {
  for (uint32_t step = 0u; step != steps; ++step) {
    (void)talus::dynamics::step(world);

    if (allAsleep(world)) {
      return true;
    }
  }

  return false;
}

JointDef anchored(uint32_t a, uint32_t b) {
  JointDef def = between(a, b);
  def.frameA.position = glm::dvec3(0.125, 0.0, 0.0);
  def.frameB.position = glm::dvec3(-0.125, 0.0, 0.0);

  return def;
}

std::vector<uint32_t> sleepingChain(World& world, const talus::field::SampledSdf& crate) {
  addFloor(world);

  std::vector<uint32_t> ids;

  for (uint32_t index = 0u; index != 5u; ++index) {
    ids.push_back(addResting(world, crate, 0.25 * static_cast<double>(index) - 0.5));
  }

  for (uint32_t index = 0u; index + 1u != ids.size(); ++index) {
    REQUIRE(addJoint(world, anchored(ids[index], ids[index + 1u])) == index + 1u);
  }

  return ids;
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

TEST_CASE("a jointed chain sleeps and wakes together", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const std::vector<uint32_t> ids = sleepingChain(world, crate);

  REQUIRE(sleepsWithin(world, 600u));

  for (const uint32_t id : ids) {
    CHECK(world.islandStore.islandOf(id) == world.islandStore.islandOf(ids[0]));
  }

  REQUIRE(applyImpulse(world, ids[0], glm::dvec3(0.0, 0.05, 0.0), world.bodies.rigids[0].position));
  (void)talus::dynamics::step(world);

  CHECK_FALSE(anyAsleep(world));
}

TEST_CASE("removing a joint splits its island", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);

  const uint32_t a = addResting(world, crate, 0.0);
  const uint32_t b = addResting(world, crate, 0.25);
  const uint32_t joint = addJoint(world, anchored(a, b));

  for (uint32_t index = 0u; index != 3u; ++index) {
    (void)talus::dynamics::step(world);
  }

  CHECK(world.islandStore.islandOf(a) == world.islandStore.islandOf(b));
  REQUIRE(removeJoint(world, joint));

  for (uint32_t index = 0u; index != 5u; ++index) {
    (void)talus::dynamics::step(world);
  }

  CHECK(world.islandStore.islandOf(a) != world.islandStore.islandOf(b));
}

TEST_CASE("a new drive target wakes a sleeping jointed island", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const std::vector<uint32_t> ids = sleepingChain(world, crate);

  REQUIRE(sleepsWithin(world, 600u));

  JointDrive drive;
  drive.axes[0].stiffness = 5.0;
  REQUIRE(setJointDrive(world, 1u, drive));
  (void)talus::dynamics::step(world);

  CHECK_FALSE(anyAsleep(world));
}

TEST_CASE("joined pairs never collide unless asked", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();

  {
    World world;

    addFloor(world);

    const uint32_t a = addResting(world, crate, 0.0);
    const uint32_t b = addResting(world, crate, 0.08);

    REQUIRE(addJoint(world, between(a, b)) != 0u);

    for (uint32_t index = 0u; index != 60u; ++index) {
      (void)talus::dynamics::step(world);
      REQUIRE_FALSE(touching(world, a, b));
    }
  }

  World world;

  addFloor(world);

  const uint32_t a = addResting(world, crate, 0.0);
  const uint32_t b = addResting(world, crate, 0.08);
  JointDef def = between(a, b);
  def.collideConnected = true;
  REQUIRE(addJoint(world, def) != 0u);

  bool contact = false;

  for (uint32_t index = 0u; index != 2u; ++index) {
    (void)talus::dynamics::step(world);
    contact = contact || touching(world, a, b);
  }

  CHECK(contact);
}

TEST_CASE("adding an excluding joint forgets the existing manifold", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);

  const uint32_t a = addResting(world, crate, 0.0);
  const uint32_t b = addResting(world, crate, 0.1);

  for (uint32_t index = 0u; index != 3u; ++index) {
    (void)talus::dynamics::step(world);
  }

  REQUIRE(cached(world, a, b));
  REQUIRE(addJoint(world, between(a, b)) != 0u);
  (void)talus::dynamics::step(world);

  CHECK_FALSE(cached(world, a, b));
  CHECK_FALSE(touching(world, a, b));
}

TEST_CASE("removing a joint wakes a sleeping body", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);

  const uint32_t a = addResting(world, crate, 0.0);
  const uint32_t joint = addJoint(world, between(noBody, a));

  REQUIRE(joint != 0u);
  REQUIRE(sleepsWithin(world, 600u));
  REQUIRE(removeJoint(world, joint));
  (void)talus::dynamics::step(world);

  CHECK_FALSE(world.bodies.rigids[0].asleep);
}

TEST_CASE("removing an excluding joint lets overlapping bodies collide", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);

  const uint32_t a = addResting(world, crate, 0.0);
  const uint32_t b = addResting(world, crate, 0.08);
  const uint32_t joint = addJoint(world, between(a, b));

  REQUIRE(joint != 0u);

  for (uint32_t index = 0u; index != 10u; ++index) {
    (void)talus::dynamics::step(world);
  }

  REQUIRE_FALSE(touching(world, a, b));
  REQUIRE(removeJoint(world, joint));

  bool contact = false;

  for (uint32_t index = 0u; index != 2u; ++index) {
    (void)talus::dynamics::step(world);
    contact = contact || touching(world, a, b);
  }

  CHECK(contact);
}
