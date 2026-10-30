#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/softstep/softstep.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
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
  JointDef def = between(noBody, a);
  def.frameA.position = world.bodies.rigids[0].position;
  const uint32_t joint = addJoint(world, def);

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

namespace {

uint64_t mixBits(uint64_t hash, double value) {
  uint64_t bits = 0u;

  std::memcpy(&bits, &value, sizeof(bits));

  for (int byte = 0; byte != 8; ++byte) {
    hash ^= (bits >> (8 * byte)) & 0xFFu;
    hash *= 1099511628211ull;
  }

  return hash;
}

struct JointedRun {
  uint64_t hash = 0u;
  bool loaded = false;
};

JointedRun chainOnPile(uint32_t workers, uint32_t width) {
  const talus::field::SampledSdf crate = crateField();
  World world;

  world.workerCount = workers;
  world.simdWidth = width;
  addFloor(world);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      for (int level = 0; level != 3; ++level) {
        (void)addCrate(world, crate, glm::dvec3(-0.12 + 0.12 * row, 0.151 + 0.101 * level, -0.12 + 0.12 * column));
      }
    }
  }

  std::vector<uint32_t> chain;

  for (int link = 0; link != 8; ++link) {
    chain.push_back(addCrate(world, crate, glm::dvec3(-0.42 + 0.12 * link, 0.7, 0.03)));
  }

  for (size_t link = 0u; link + 1u != chain.size(); ++link) {
    JointDef def = between(chain[link], chain[link + 1u]);
    def.frameA.position = glm::dvec3(0.06, 0.0, 0.0);
    def.frameB.position = glm::dvec3(-0.06, 0.0, 0.0);
    REQUIRE(addJoint(world, def) != 0u);
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    if (talus::dynamics::step(world).nonFinite) {
      return {};
    }
  }

  JointedRun run;
  run.hash = 1469598103934665603ull;

  for (const RigidBody& body : world.bodies.rigids) {
    for (int axis = 0; axis != 3; ++axis) {
      run.hash = mixBits(run.hash, body.position[axis]);
      run.hash = mixBits(run.hash, body.velocity[axis]);
      run.hash = mixBits(run.hash, body.angularVelocity[axis]);
    }

    for (int axis = 0; axis != 4; ++axis) {
      run.hash = mixBits(run.hash, body.rotation[axis]);
    }
  }

  for (const Joint& joint : world.joints.all()) {
    for (int axis = 0; axis != 3; ++axis) {
      run.hash = mixBits(run.hash, joint.report.force[axis]);
    }

    run.loaded = run.loaded || joint.report.force != glm::dvec3(0.0);
  }

  return run;
}

}

TEST_CASE("a ball pendulum keeps its period and energy", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const double lever = 1.0;
  const double release = 0.1;
  const glm::dvec3 pivot(0.0, 2.0, 0.0);
  const glm::dvec3 start = pivot + lever * glm::dvec3(std::sin(release), -std::cos(release), 0.0);
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, start, glm::dvec3(0.0), 0.5, 3u);
  JointDef def = between(noBody, world.bodies.rigids[index].id);
  def.frameA.position = pivot;
  def.frameB.position = pivot - start;
  REQUIRE(addJoint(world, def) != 0u);

  const double dt = world.parameters.dt;
  const uint32_t steps = static_cast<uint32_t>(std::lround(60.0 / dt));
  double previous = release;
  double peak = 0.0;
  std::vector<double> crossings;

  for (uint32_t frame = 1u; frame <= steps; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec3 arm = world.bodies.rigids[index].position - pivot;
    const double angle = std::atan2(arm.x, -arm.y);

    if (previous < 0.0 && angle >= 0.0) {
      crossings.push_back(dt * (static_cast<double>(frame - 1u) + previous / (previous - angle)));
    }

    peak = std::max(peak, std::atan2(std::hypot(arm.x, arm.z), -arm.y));
    previous = angle;
  }

  REQUIRE(crossings.size() >= 10u);

  const double period = (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1u);
  const double expected = 2.0 * std::numbers::pi * std::sqrt(lever / std::abs(world.parameters.gravity.y));

  CHECK(std::abs(period - expected) <= 0.02 * expected);
  CHECK(peak <= 1.01 * release);
}

TEST_CASE("a hanging joint reports its load", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const double mass = 3.0;
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), mass, glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.0), 0.5, 3u);
  JointDef def = between(noBody, world.bodies.rigids[index].id);
  def.frameA.position = glm::dvec3(0.0, 2.0, 0.0);
  def.frameB.position = glm::dvec3(0.0, 0.5, 0.0);
  const uint32_t joint = addJoint(world, def);
  REQUIRE(joint != 0u);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const std::optional<JointReport> report = jointReport(world, joint);
  const double weight = mass * std::abs(world.parameters.gravity.y);

  REQUIRE(report.has_value());
  CHECK(std::abs(report->force.y - weight) <= 0.01 * weight);
}

TEST_CASE("a joint rides a moving mass-0 body", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const glm::dvec3 speed(1.0, 0.0, 0.0);
  const uint32_t platform = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 0.0, glm::dvec3(0.0, 2.0, 0.0), speed, 0.5, 3u);
  const uint32_t box = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.0, 1.4, 0.0), speed, 0.5, 3u);
  JointDef def = between(world.bodies.rigids[platform].id, world.bodies.rigids[box].id);
  def.frameA.position = glm::dvec3(0.0, -0.1, 0.0);
  def.frameB.position = glm::dvec3(0.0, 0.5, 0.0);
  REQUIRE(addJoint(world, def) != 0u);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const RigidBody& carrier = world.bodies.rigids[platform];
  const glm::dvec3 anchor = carrier.position + carrier.rotation * def.frameA.position;
  const glm::dvec3 offset = world.bodies.rigids[box].position - anchor;

  CHECK(std::abs(carrier.position.x - 2.0) < 1.0e-3);
  CHECK(std::hypot(offset.x, offset.z) < 0.01);
  CHECK(std::abs(offset.y + 0.5) < 0.01);
}

TEST_CASE("a joint follows a mass-0 body outside the solve", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  BodySet bodies;
  const glm::dvec3 speed(1.0, 0.0, 0.0);
  const uint32_t platform = addRigidBox(bodies, crate, glm::dvec3(0.05), 0.0, glm::dvec3(0.0, 2.0, 0.0), speed, 0.5, 3u);
  const uint32_t box = addRigidBox(bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.0, 1.4, 0.0), speed, 0.5, 3u);
  JointDef def = between(bodies.rigids[platform].id, bodies.rigids[box].id);
  def.frameA.position = glm::dvec3(0.0, -0.1, 0.0);
  def.frameB.position = glm::dvec3(0.0, 0.5, 0.0);
  JointSet joints;
  REQUIRE(joints.add(def) == 1u);

  IslandStore islands;
  ManifoldCache cache;
  const IntegrationParameters parameters;
  std::vector<uint32_t> stateOf;
  const std::vector<uint32_t> solved{box};

  islands.sync(bodies);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    (void)solveSoftstep(bodies, cache, solved, parameters, stateOf, Workers{}, 1u, nullptr, nullptr, nullptr, nullptr, &joints, &islands);
    bodies.rigids[platform].position += parameters.dt * speed;
  }

  const glm::dvec3 anchor = bodies.rigids[platform].position + def.frameA.position;
  const glm::dvec3 offset = bodies.rigids[box].position - anchor;

  CHECK(std::hypot(offset.x, offset.z) < 0.01);
  CHECK(std::abs(offset.y + 0.5) < 0.01);
}

namespace {

void addJointedPair(World& world, const talus::field::SampledSdf& crate, const glm::dvec3& at) {
  const uint32_t a = addCrate(world, crate, at);
  const uint32_t b = addCrate(world, crate, at + glm::dvec3(0.25, 0.0, 0.0));

  REQUIRE(addJoint(world, anchored(a, b)) != 0u);
}

uint64_t bodyHash(uint64_t hash, const RigidBody& body) {
  for (int axis = 0; axis != 3; ++axis) {
    hash = mixBits(hash, body.position[axis]);
    hash = mixBits(hash, body.velocity[axis]);
    hash = mixBits(hash, body.angularVelocity[axis]);
  }

  for (int axis = 0; axis != 4; ++axis) {
    hash = mixBits(hash, body.rotation[axis]);
  }

  return hash;
}

uint64_t freeBoxHash(bool besideJoint) {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.4, 0.0));

  if (besideJoint) {
    addJointedPair(world, crate, glm::dvec3(5.0, 2.0, 0.0));
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    (void)talus::dynamics::step(world);
  }

  return bodyHash(1469598103934665603ull, world.bodies.rigids[findBody(world.bodies, box)->index]);
}

uint64_t pendulumHash(uint32_t softstepSubsteps, uint32_t jointSubsteps) {
  const talus::field::SampledSdf crate = crateField();
  World world;
  world.parameters.softstepSubsteps = softstepSubsteps;
  world.parameters.jointSubsteps = jointSubsteps;
  addFloor(world);
  (void)addResting(world, crate, 0.0);

  const glm::dvec3 pivot(3.0, 2.0, 0.0);
  const glm::dvec3 start = pivot + glm::dvec3(0.3, -0.4, 0.0);
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, start, glm::dvec3(0.0), 0.5, 3u);
  JointDef def = between(noBody, world.bodies.rigids[index].id);
  def.frameA.position = pivot;
  def.frameB.position = pivot - start;
  REQUIRE(addJoint(world, def) != 0u);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    (void)talus::dynamics::step(world);
  }

  return bodyHash(1469598103934665603ull, world.bodies.rigids[index]);
}

struct DeckScene {
  uint32_t deck = 0u;
  std::vector<uint32_t> boxes;
  std::vector<glm::dvec3> offsets;
};

DeckScene addDeckScene(World& world, const talus::field::SampledSdf& crate) {
  const glm::dvec3 speed(1.0, 0.0, 0.0);
  DeckScene scene;

  world.parameters.sleep = false;
  scene.deck = addRigidBox(world.bodies, floor(), glm::dvec3(1.0, 0.1, 1.0), 0.0, glm::dvec3(0.0, 1.0, 0.0), speed, 0.8, 3u);

  for (const double x : {-0.3, -0.05, 0.4}) {
    const uint32_t id = addCrate(world, crate, glm::dvec3(x, 1.15, 0.0));
    RigidBody& body = world.bodies.rigids[findBody(world.bodies, id)->index];

    body.velocity = speed;
    scene.boxes.push_back(id);
    scene.offsets.push_back(body.position - world.bodies.rigids[scene.deck].position);
  }

  REQUIRE(addJoint(world, anchored(scene.boxes[0], scene.boxes[1])) != 0u);

  return scene;
}

uint64_t deckHash(uint32_t workers, uint32_t width) {
  const talus::field::SampledSdf crate = crateField();
  World world;

  world.workerCount = workers;
  world.simdWidth = width;
  (void)addDeckScene(world, crate);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    if (talus::dynamics::step(world).nonFinite) {
      return 0u;
    }
  }

  uint64_t hash = 1469598103934665603ull;

  for (const RigidBody& body : world.bodies.rigids) {
    hash = bodyHash(hash, body);
  }

  for (const Joint& joint : world.joints.all()) {
    for (int axis = 0; axis != 3; ++axis) {
      hash = mixBits(hash, joint.report.force[axis]);
    }
  }

  return hash;
}

}

TEST_CASE("jointed worlds step the same at widths 1, 4 and 8 and workers 1, 4 and 8", "[dynamics][joints]") {
  std::vector<uint32_t> widths{1u, 4u};

  if (hasAvx2()) {
    widths.push_back(8u);
  }

  const JointedRun serial = chainOnPile(1u, 1u);
  const uint64_t deck = deckHash(1u, 1u);

  REQUIRE(serial.hash != 0u);
  REQUIRE(serial.loaded);
  REQUIRE(deck != 0u);

  for (const uint32_t width : widths) {
    for (const uint32_t workers : {1u, 4u, 8u}) {
      CAPTURE(width, workers);
      CHECK(chainOnPile(workers, width).hash == serial.hash);
      CHECK(deckHash(workers, width) == deck);
    }
  }
}

TEST_CASE("jointed islands step at the joint substep count", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;

  addFloor(world);
  (void)addResting(world, crate, 0.0);
  addJointedPair(world, crate, glm::dvec3(3.0, 2.0, 0.0));

  const StepReport report = talus::dynamics::step(world);

  CHECK(world.parameters.jointSubsteps == 16u);
  CHECK(report.jointSubsteps == world.parameters.jointSubsteps);
  CHECK(report.softstepSubsteps == world.parameters.softstepSubsteps);

  World plain;

  addFloor(plain);
  (void)addResting(plain, crate, 0.0);

  const StepReport alone = talus::dynamics::step(plain);

  CHECK(alone.jointSubsteps == 0u);
  CHECK(alone.softstepSubsteps == plain.parameters.softstepSubsteps);
}

TEST_CASE("a free island steps the same beside a jointed island", "[dynamics][joints]") {
  CHECK(freeBoxHash(true) == freeBoxHash(false));
}

TEST_CASE("jointed islands move at the joint substep count whatever the world's", "[dynamics][joints]") {
  const uint64_t jointed = pendulumHash(4u, 16u);

  CHECK(jointed == pendulumHash(16u, 16u));
  CHECK(jointed != pendulumHash(4u, 4u));
}

TEST_CASE("jointed and free boxes ride the same moving mass-0 deck", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const DeckScene scene = addDeckScene(world, crate);
  const uint32_t deck = scene.deck;
  const std::vector<uint32_t>& boxes = scene.boxes;
  const std::vector<glm::dvec3>& offsets = scene.offsets;

  StepReport report;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    report = talus::dynamics::step(world);
    REQUIRE_FALSE(report.nonFinite);
  }

  REQUIRE(report.jointSubsteps == world.parameters.jointSubsteps);
  REQUIRE(report.softstepSubsteps == world.parameters.softstepSubsteps);

  for (size_t box = 0u; box != boxes.size(); ++box) {
    CAPTURE(box);

    const glm::dvec3 offset =
      world.bodies.rigids[findBody(world.bodies, boxes[box])->index].position - world.bodies.rigids[deck].position;

    CHECK(std::abs(offset.x - offsets[box].x) < 0.01);
  }
}

namespace {

glm::dquat jointFrameRotation(const World& world, uint32_t id, const JointFrame& frame) {
  if (id == noBody) {
    return frame.rotation;
  }

  return world.bodies.rigids[findBody(world.bodies, id)->index].rotation * frame.rotation;
}

double hingeAngle(const World& world, const JointDef& def) {
  glm::dquat relative =
    glm::conjugate(jointFrameRotation(world, def.bodyA, def.frameA)) * jointFrameRotation(world, def.bodyB, def.frameB);

  if (relative.w < 0.0) {
    relative = -relative;
  }

  return 2.0 * std::atan2(relative.x, relative.w);
}

JointDef worldHinge(const World& world, uint32_t index, const glm::dvec3& pivot) {
  const RigidBody& body = world.bodies.rigids[index];
  JointDef def = between(noBody, body.id);
  def.kind = JointKind::Hinge;
  def.frameA.position = pivot;
  def.frameB.position = glm::conjugate(body.rotation) * (pivot - body.position);

  return def;
}

const glm::dvec3 hingePivot(0.0, 2.0, 0.0);
const glm::dvec3 hingeLever(0.0, 0.1, 0.0);

uint32_t addHingedCrate(World& world, const talus::field::SampledSdf& crate) {
  return addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, hingePivot - hingeLever, glm::dvec3(0.0), 0.5, 3u);
}

std::vector<double> zeroCrossings(const std::vector<double>& samples, double dt) {
  std::vector<double> crossings;

  for (size_t index = 1u; index < samples.size(); ++index) {
    const double before = samples[index - 1u];
    const double after = samples[index];

    if ((before < 0.0) != (after < 0.0)) {
      crossings.push_back(dt * (static_cast<double>(index - 1u) + before / (before - after)));
    }
  }

  return crossings;
}

}

TEST_CASE("a hinge pendulum keeps its period", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const double lever = 1.0;
  const double release = 0.1;
  const glm::dvec3 start = hingePivot + lever * glm::dvec3(0.0, -std::cos(release), std::sin(release));
  const glm::dvec3 sideways(0.1, 0.0, 0.0);
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, start, sideways, 0.5, 3u);
  REQUIRE(addJoint(world, worldHinge(world, index, hingePivot)) != 0u);

  const double dt = world.parameters.dt;
  const uint32_t steps = static_cast<uint32_t>(std::lround(60.0 / dt));
  std::vector<double> angles;
  double peak = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= steps; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec3 arm = world.bodies.rigids[index].position - hingePivot;

    angles.push_back(std::atan2(arm.z, -arm.y));
    peak = std::max(peak, std::atan2(std::hypot(arm.x, arm.z), -arm.y));

    if (dt * frame > 10.0) {
      drift = std::max(drift, std::abs(arm.x));
    }
  }

  const std::vector<double> crossings = zeroCrossings(angles, dt);

  REQUIRE(crossings.size() >= 20u);

  const double period = 2.0 * (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1u);
  const double expected = 2.0 * std::numbers::pi * std::sqrt(lever / std::abs(world.parameters.gravity.y));

  CHECK(std::abs(period - expected) <= 0.02 * expected);
  CHECK(peak <= 1.01 * release);
  CHECK(drift < 1.0e-3);
}

TEST_CASE("a hinge drive deflects by torque over stiffness", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;
  const double stiffness = 200.0;
  const double torque = 10.0;
  const uint32_t index = addHingedCrate(world, crate);
  JointDef def = worldHinge(world, index, hingePivot);
  def.drive.axes[0].stiffness = stiffness;
  def.drive.axes[0].damping = 20.0;
  def.drive.feedForward = glm::dvec3(torque, 0.0, 0.0);
  const uint32_t joint = addJoint(world, def);
  REQUIRE(joint != 0u);

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const double deflection = torque / stiffness;
  const std::optional<JointReport> report = jointReport(world, joint);

  REQUIRE(report.has_value());
  CHECK(std::abs(hingeAngle(world, def) - deflection) <= 0.02 * deflection);
  CHECK(std::abs(report->driveTorque.x + torque) <= 0.02 * torque);
}

TEST_CASE("a hinge drive rings at sqrt(k over I) for any mass ratio", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double stiffness = 0.5;
  const double offset = 0.05;
  const double half = 0.05;
  const double leverA = 0.1;
  const double leverB = 0.1;

  for (const double mass : {1.0, 10.0, 75.0}) {
    CAPTURE(mass);

    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    const glm::dvec3 anchor(0.0, 2.0, 0.0);
    const glm::dquat turned = glm::angleAxis(offset, glm::dvec3(1.0, 0.0, 0.0));
    const glm::dvec3 parentAt = anchor + glm::dvec3(0.0, leverA, 0.0);
    const glm::dvec3 childAt = anchor - turned * glm::dvec3(0.0, leverB, 0.0);
    const uint32_t parent = addRigidBox(world.bodies, crate, glm::dvec3(half), mass, parentAt, glm::dvec3(0.0), 0.5, 3u);
    const uint32_t child = addRigidBox(world.bodies, crate, glm::dvec3(half), 1.0, childAt, glm::dvec3(0.0), 0.5, 3u);
    world.bodies.rigids[child].rotation = turned;
    JointDef def = between(world.bodies.rigids[parent].id, world.bodies.rigids[child].id);
    def.kind = JointKind::Hinge;
    def.frameA.position = glm::dvec3(0.0, -leverA, 0.0);
    def.frameB.position = glm::dvec3(0.0, leverB, 0.0);
    def.drive.axes[0].stiffness = stiffness;
    REQUIRE(addJoint(world, def) != 0u);
    REQUIRE(std::abs(hingeAngle(world, def) - offset) < 1.0e-9);

    const double dt = world.parameters.dt;
    std::vector<double> angles;

    for (uint32_t frame = 0u; frame != 360u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
      angles.push_back(hingeAngle(world, def));
    }

    std::vector<double> crossings = zeroCrossings(angles, dt);

    REQUIRE(crossings.size() >= 8u);
    crossings.resize(8u);

    const double side = 2.0 * half;
    const double inertiaA = mass * 2.0 * side * side / 12.0;
    const double inertiaB = 1.0 * 2.0 * side * side / 12.0;
    const double reduced = mass * 1.0 / (mass + 1.0);
    const double inertia = (inertiaA * inertiaB + reduced * (leverA * leverA * inertiaB + leverB * leverB * inertiaA))
      / (inertiaA + inertiaB + reduced * (leverA + leverB) * (leverA + leverB));
    const double expected = std::sqrt(stiffness / inertia) / (2.0 * std::numbers::pi);
    const double measured = static_cast<double>(crossings.size() - 1u) / (2.0 * (crossings.back() - crossings.front()));

    CHECK(std::abs(measured - expected) <= 0.05 * expected);
  }
}

TEST_CASE("hinge drive torque respects per-direction bounds and the cap", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double slack = 1.0e-4;

  const auto run = [&](double target, double cap, double lower, double upper, double reached) {
    CAPTURE(target, cap);

    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldHinge(world, index, hingePivot);
    def.drive.axes[0].targetAngle = target;
    def.drive.axes[0].stiffness = 100.0;
    def.drive.axes[0].minTorque = -5.0;
    def.drive.axes[0].maxTorque = 2.0;
    def.drive.maxTorque = cap;
    const uint32_t joint = addJoint(world, def);
    REQUIRE(joint != 0u);

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      const JointReport report = *jointReport(world, joint);

      CHECK(report.driveTorque.x >= lower - slack);
      CHECK(report.driveTorque.x <= upper + slack);

      if (frame == 0u) {
        CHECK(std::abs(report.driveTorque.x - reached) <= slack);
        CHECK(report.driveSaturated);
      }
    }
  };

  run(1.0, jointInfinity, -5.0, 2.0, 2.0);
  run(-1.0, jointInfinity, -5.0, 2.0, -5.0);
  run(1.0, 1.0, -1.0, 1.0, 1.0);
  run(-1.0, 1.0, -1.0, 1.0, -1.0);
}

TEST_CASE("a hinge limit holds at the drive cap", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double bound = 0.5;

  for (const double side : {1.0, -1.0}) {
    CAPTURE(side);

    World world;
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldHinge(world, index, hingePivot);
    def.hinge.enabled = true;
    def.hinge.lower = -bound;
    def.hinge.upper = bound;
    def.drive.axes[0].targetAngle = 2.0 * side;
    def.drive.axes[0].stiffness = 1.0e5;
    def.drive.maxTorque = 150.0;
    const uint32_t joint = addJoint(world, def);
    REQUIRE(joint != 0u);

    double furthest = 0.0;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
      furthest = std::max(furthest, side * hingeAngle(world, def));
    }

    const JointReport report = *jointReport(world, joint);

    CHECK(furthest <= 0.53);
    CHECK(side * hingeAngle(world, def) >= bound - 0.02);
    CHECK(report.limitActive);
    CHECK(report.driveSaturated);
  }
}
