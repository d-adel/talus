#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <fixtures/humanoid.h>

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/softstep/softstep.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
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

talus::field::SampledSdf boxOf(const glm::dvec3& half, double voxel, double outset = 0.0) {
  const glm::vec3 reach = glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half) + glm::vec3(static_cast<float>(outset)))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel))) + glm::uvec3(1u)
  );
}

talus::PhysicalMaterial jelly() {
  talus::PhysicalMaterial material;
  material.density = 500.0f;
  material.friction = 0.7f;
  material.youngsModulus = 5.0e4f;
  material.poissonsRatio = 0.4f;

  return material;
}

struct SoftBlock {
  talus::field::SampledSdf field;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

SoftBlock softBlock(const glm::dvec3& half, double voxel) {
  SoftBlock block;
  block.field = boxOf(half, voxel, 1.0e-4);
  block.lanes.assign(2u, jelly());
  block.mesh = talus::deformable::buildMesh(
    block.field,
    block.lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  return block;
}

void addSoftBlock(World& world, const SoftBlock& block, const glm::dvec3& at, const glm::dvec3& velocity = glm::dvec3(0.0)) {
  (void)addLattice(world.bodies, block.mesh, block.lanes, at, velocity, block.lanes.front().friction, &block.field);
}

const Island* islandWith(const World& world, uint32_t index) {
  for (const Island& island : world.islands) {
    for (const BodyRef& ref : island.bodies) {
      if (ref.side == Side::Rigid && ref.index == index) {
        return &island;
      }
    }
  }

  return nullptr;
}

void requireStrainstep(const World& world, uint32_t index) {
  const Island* island = islandWith(world, index);

  REQUIRE(island != nullptr);
  REQUIRE(island->owner == Owner::Strainstep);
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
  def.spring.angularDamping = nan;
  rejects(def);

  def = between(a, b);
  def.spring.maxTorque = -1.0;
  rejects(def);

  def = between(a, b);
  def.spring.target.rotation = glm::dquat(0.0, 0.0, 0.0, 0.0);
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
  uint32_t limitPassRows = 0u;
  bool loaded = false;
  bool limited = false;
  bool saturated = false;
};

void twistLink(JointDef& def) {
  def.kind = JointKind::SwingTwist;
  def.cone.enabled = true;
  def.cone.twistLower = -0.3;
  def.cone.twistUpper = 0.3;
  def.cone.swing = glm::dvec2(0.5, 0.25);
  def.cone.centre = glm::dvec2(0.05, 0.0);
  def.drive.axes[1].targetAngle = 0.2;
  def.drive.axes[2].targetAngle = -0.3;
  def.drive.maxTorque = 5.0;

  for (AxisDrive& axis : def.drive.axes) {
    axis.stiffness = 2.0;
    axis.damping = 0.1;
  }
}

void springLink(JointDef& def) {
  def.kind = JointKind::Spring;
  def.spring.linearStiffness = 2000.0;
  def.spring.linearDamping = 20.0;
  def.spring.maxForce = 20.0;
  def.spring.angularStiffness = 2.0;
  def.spring.angularDamping = 0.05;
  def.spring.maxTorque = 0.5;
}

JointedRun chainOnPile(uint32_t workers, uint32_t width, JointKind kind) {
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

    if (kind == JointKind::SwingTwist) {
      twistLink(def);
    } else if (kind == JointKind::Spring) {
      springLink(def);
    }

    REQUIRE(addJoint(world, def) != 0u);
  }

  JointedRun run;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const StepReport report = talus::dynamics::step(world);

    if (report.nonFinite) {
      return {};
    }

    run.limitPassRows += report.limitPassRows;

    for (const Joint& joint : world.joints.all()) {
      run.limited = run.limited || joint.report.limitActive;
      run.saturated = run.saturated || joint.report.forceSaturated || joint.report.torqueSaturated;
    }
  }

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
      run.hash = mixBits(run.hash, joint.report.torque[axis]);
      run.hash = mixBits(run.hash, joint.report.driveTorque[axis]);
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

struct LatticeChain {
  std::vector<uint32_t> links;
  std::vector<JointDef> defs;
};

LatticeChain addLatticeChain(World& world, const talus::field::SampledSdf& crate, const SoftBlock& slab) {
  const glm::dquat across = glm::angleAxis(-0.5 * std::numbers::pi, glm::dvec3(0.0, 1.0, 0.0));
  LatticeChain chain;

  world.parameters.sleep = false;
  addFloor(world);
  addSoftBlock(world, slab, glm::dvec3(-0.4, 0.15, 0.0));

  for (uint32_t link = 0u; link != 5u; ++link) {
    chain.links.push_back(addCrate(world, crate, glm::dvec3(-0.5 + 0.25 * static_cast<double>(link), 0.25, 0.0)));
  }

  for (uint32_t link = 0u; link + 1u != chain.links.size(); ++link) {
    JointDef def = anchored(chain.links[link], chain.links[link + 1u]);
    def.kind = JointKind::Hinge;
    def.frameA.rotation = across;
    def.frameB.rotation = across;
    REQUIRE(addJoint(world, def) != 0u);
    chain.defs.push_back(def);
  }

  return chain;
}

SoftBlock chainSlab() {
  return softBlock(glm::dvec3(0.2, 0.05, 0.15), 0.05);
}

uint64_t latticeChainHash(uint32_t workers, uint32_t width) {
  const talus::field::SampledSdf crate = crateField();
  const SoftBlock slab = chainSlab();
  World world;

  world.workerCount = workers;
  world.simdWidth = width;
  (void)addLatticeChain(world, crate, slab);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    const StepReport report = talus::dynamics::step(world);

    if (report.nonFinite || report.jointRows != 4u) {
      return 0u;
    }
  }

  uint64_t hash = 1469598103934665603ull;

  for (const RigidBody& body : world.bodies.rigids) {
    hash = bodyHash(hash, body);
  }

  for (const glm::dvec3& node : world.bodies.lattices[0].position) {
    for (int axis = 0; axis != 3; ++axis) {
      hash = mixBits(hash, node[axis]);
    }
  }

  for (const Joint& joint : world.joints.all()) {
    for (int axis = 0; axis != 3; ++axis) {
      hash = mixBits(hash, joint.report.force[axis]);
      hash = mixBits(hash, joint.report.torque[axis]);
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

  const JointedRun serial = chainOnPile(1u, 1u, JointKind::Ball);
  const JointedRun twisted = chainOnPile(1u, 1u, JointKind::SwingTwist);
  const JointedRun sprung = chainOnPile(1u, 1u, JointKind::Spring);
  const uint64_t deck = deckHash(1u, 1u);
  const uint64_t strained = latticeChainHash(1u, 1u);

  REQUIRE(serial.hash != 0u);
  REQUIRE(serial.loaded);
  REQUIRE(twisted.hash != 0u);
  REQUIRE(twisted.loaded);
  REQUIRE(twisted.limited);
  REQUIRE(twisted.limitPassRows != 0u);
  REQUIRE(sprung.hash != 0u);
  REQUIRE(sprung.loaded);
  REQUIRE(sprung.saturated);
  REQUIRE(deck != 0u);
  REQUIRE(strained != 0u);

  for (const uint32_t width : widths) {
    for (const uint32_t workers : {1u, 4u, 8u}) {
      CAPTURE(width, workers);
      CHECK(chainOnPile(workers, width, JointKind::Ball).hash == serial.hash);
      const JointedRun twistedRun = chainOnPile(workers, width, JointKind::SwingTwist);
      CHECK(twistedRun.hash == twisted.hash);
      CHECK(twistedRun.limitPassRows == twisted.limitPassRows);
      CHECK(chainOnPile(workers, width, JointKind::Spring).hash == sprung.hash);
      CHECK(deckHash(workers, width) == deck);
      CHECK(latticeChainHash(workers, width) == strained);
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
  double sway = 0.0;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= steps; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec3 arm = world.bodies.rigids[index].position - hingePivot;

    angles.push_back(std::atan2(arm.z, -arm.y));
    peak = std::max(peak, std::atan2(std::hypot(arm.x, arm.z), -arm.y));
    sway = std::max(sway, std::abs(arm.x));

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
  CHECK(sway <= 0.01);
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

namespace {

void checkRinging(double stiffness) {
  const talus::field::SampledSdf crate = crateField();
  const double offset = 0.05;
  const double half = 0.05;
  const double leverA = 0.1;
  const double leverB = 0.1;

  for (const double mass : {1.0, 10.0, 75.0}) {
    CAPTURE(stiffness, mass);

    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    world.parameters.dt = 1.0 / 240.0;
    world.parameters.jointSubsteps = 4u;
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

    for (uint32_t frame = 0u; frame != 240u; ++frame) {
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

    CAPTURE(expected, measured);
    CHECK(std::abs(measured - expected) <= 0.05 * expected);
  }
}

}

TEST_CASE("a hinge drive rings at sqrt(k over I) for any mass ratio", "[dynamics][joints][!mayfail]") {
  checkRinging(200.0);
}

TEST_CASE("a hinge drive rings at sqrt(k over I) within its accurate range", "[dynamics][joints]") {
  checkRinging(10.0);
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

namespace {

double checkLimitAtCap(double dt, uint32_t substeps, const SoftBlock* beside = nullptr) {
  const talus::field::SampledSdf crate = crateField();
  const double bound = 0.5;
  double widest = 0.0;

  for (const double side : {1.0, -1.0}) {
    CAPTURE(side);

    World world;
    world.parameters.sleep = false;
    world.parameters.dt = dt;
    world.parameters.jointSubsteps = substeps;
    const uint32_t index = addHingedCrate(world, crate);

    if (beside != nullptr) {
      (void)addStatic(world.bodies, floor(), 0.5, 0.0, hingePivot + glm::dvec3(0.0, -0.4, 0.0));
      addSoftBlock(world, *beside, hingePivot + glm::dvec3(0.16, -0.1, 0.0));
    }

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
    uint32_t limitPassRows = 0u;
    const uint32_t steps = static_cast<uint32_t>(std::lround(2.0 / dt));

    for (uint32_t frame = 0u; frame != steps; ++frame) {
      const StepReport stepped = talus::dynamics::step(world);

      REQUIRE_FALSE(stepped.nonFinite);
      furthest = std::max(furthest, side * hingeAngle(world, def));
      limitPassRows = stepped.limitPassRows;

      if (beside != nullptr) {
        requireStrainstep(world, index);
      }
    }

    const JointReport report = *jointReport(world, joint);

    CHECK(limitPassRows != 0u);
    CHECK(furthest <= 0.53);
    CHECK(side * hingeAngle(world, def) >= bound - 0.02);
    CHECK(report.limitActive);
    CHECK(report.driveSaturated);
    widest = std::max(widest, furthest);
  }

  return widest;
}

}

TEST_CASE("a hinge limit holds at the drive cap", "[dynamics][joints]") {
  checkLimitAtCap(1.0 / 60.0, 16u);
}

TEST_CASE("a hinge limit holds at the drive cap with a 30 Hz step", "[dynamics][joints]") {
  checkLimitAtCap(1.0 / 30.0, 32u);
}

TEST_CASE("a hinge limit in a strainstep island holds at the drive cap the same at 60 and 30 Hz", "[dynamics][joints]") {
  const SoftBlock wall = softBlock(glm::dvec3(0.1, 0.2, 0.2), 0.1);
  const double sixty = checkLimitAtCap(1.0 / 60.0, 16u, &wall);
  const double thirty = checkLimitAtCap(1.0 / 30.0, 32u, &wall);

  CAPTURE(sixty, thirty);
  CHECK(std::abs(sixty - thirty) <= 1.0e-6);
}

TEST_CASE("hinge limits and drive targets beyond a half turn are rejected", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t index = addHingedCrate(world, crate);
  const double pi = std::numbers::pi;

  const auto accepts = [&](double lower, double upper, double target) {
    JointDef def = worldHinge(world, index, hingePivot);
    def.hinge.enabled = true;
    def.hinge.lower = lower;
    def.hinge.upper = upper;
    def.drive.axes[0].targetAngle = target;
    def.drive.axes[0].stiffness = 20.0;
    const uint32_t joint = addJoint(world, def);

    if (joint != 0u) {
      REQUIRE(removeJoint(world, joint));
    }

    return joint != 0u;
  };

  CHECK(accepts(-0.99 * pi, 0.99 * pi, pi));
  CHECK(accepts(-0.5, 0.5, -pi));
  CHECK_FALSE(accepts(10.0, 45.0, 0.0));
  CHECK_FALSE(accepts(-3.12, 0.5, 0.0));
  CHECK_FALSE(accepts(-0.5, 3.12, 0.0));
  CHECK_FALSE(accepts(-0.5, 0.5, 4.0));
  CHECK_FALSE(accepts(-0.5, 0.5, -3.2));
  CHECK(world.joints.empty());

  const uint32_t joint = addJoint(world, worldHinge(world, index, hingePivot));
  REQUIRE(joint != 0u);

  JointDrive drive;
  drive.axes[0].targetAngle = 4.0;
  CHECK_FALSE(setJointDrive(world, joint, drive));
  drive.axes[0].targetAngle = 1.0;
  CHECK(setJointDrive(world, joint, drive));
}

namespace {

struct SwingTwistAngles {
  double twist = 0.0;
  glm::dvec2 swing{0.0};
};

SwingTwistAngles reflexSwingTwist(const glm::dquat& rotation) {
  const glm::dquat q = rotation.w < 0.0 ? -rotation : rotation;
  const double twistNorm = std::sqrt(q.w * q.w + q.x * q.x);

  if (twistNorm < 1.0e-6) {
    return {0.0, glm::normalize(glm::dvec2(q.y, q.z)) * std::numbers::pi};
  }

  const double cosine = q.w / twistNorm;
  const double sine = q.x / twistNorm;
  const glm::dvec2 swing(q.y * cosine - q.z * sine, q.y * sine + q.z * cosine);
  const double swingSine = glm::length(swing);
  const double twist = 2.0 * std::atan2(sine, cosine);

  if (swingSine < 1.0e-6) {
    return {twist, swing * (2.0 / twistNorm)};
  }

  return {twist, swing * (2.0 * std::atan2(swingSine, twistNorm) / swingSine)};
}

SwingTwistAngles jointAngles(const World& world, const JointDef& def) {
  return reflexSwingTwist(
    glm::conjugate(jointFrameRotation(world, def.bodyA, def.frameA)) * jointFrameRotation(world, def.bodyB, def.frameB)
  );
}

const glm::dquat limbDown = glm::angleAxis(-0.5 * std::numbers::pi, glm::dvec3(0.0, 0.0, 1.0));

JointDef worldSwingTwist(const World& world, uint32_t index) {
  const RigidBody& body = world.bodies.rigids[index];
  JointDef def = between(noBody, body.id);
  def.kind = JointKind::SwingTwist;
  def.frameA.position = hingePivot;
  def.frameA.rotation = limbDown;
  def.frameB.position = glm::conjugate(body.rotation) * (hingePivot - body.position);
  def.frameB.rotation = glm::conjugate(body.rotation) * limbDown;

  return def;
}

ConeLimit ellipticalCone() {
  ConeLimit cone;
  cone.enabled = true;
  cone.twistLower = -0.4;
  cone.twistUpper = 0.2;
  cone.swing = glm::dvec2(0.6, 0.3);
  cone.centre = glm::dvec2(0.1, 0.0);

  return cone;
}

JointDef shoulder(const World& world, uint32_t index) {
  JointDef def = worldSwingTwist(world, index);
  def.cone = ellipticalCone();

  return def;
}

void pushAtCap(JointDef& def) {
  def.drive.maxTorque = 100.0;

  for (AxisDrive& axis : def.drive.axes) {
    axis.stiffness = 1.0e5;
  }
}

double ellipse(const ConeLimit& cone, const glm::dvec2& swing) {
  const glm::dvec2 scaled = (swing - cone.centre) / cone.swing;

  return glm::dot(scaled, scaled);
}

void checkConeAtCap(const ConeLimit& cone, double reach, double cap) {
  const talus::field::SampledSdf crate = crateField();
  const double margin = 0.03 / std::min(cone.swing.x, cone.swing.y);
  const double inner = std::max(0.0, 1.0 - margin);

  for (uint32_t direction = 0u; direction != 8u; ++direction) {
    const double heading = 0.25 * std::numbers::pi * static_cast<double>(direction);
    CAPTURE(direction);

    World world;
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldSwingTwist(world, index);
    def.cone = cone;
    pushAtCap(def);
    def.drive.maxTorque = cap;
    def.drive.axes[1].targetAngle = reach * std::cos(heading);
    def.drive.axes[2].targetAngle = reach * std::sin(heading);
    const uint32_t joint = addJoint(world, def);
    REQUIRE(joint != 0u);

    double furthest = 0.0;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
      furthest = std::max(furthest, ellipse(def.cone, jointAngles(world, def).swing));
    }

    const JointReport report = *jointReport(world, joint);

    CAPTURE(furthest);
    CHECK(furthest <= (1.0 + margin) * (1.0 + margin));
    CHECK(ellipse(def.cone, jointAngles(world, def).swing) >= inner * inner);
    CHECK(report.limitActive);
    CHECK(report.driveSaturated);
  }
}

ConeLimit reflexShoulder() {
  ConeLimit cone;
  cone.enabled = true;
  cone.twistLower = -2.3;
  cone.twistUpper = 1.1;
  cone.swing = glm::dvec2(1.85, 1.86);

  return cone;
}

}

TEST_CASE("a swing-twist cone holds an ellipse", "[dynamics][joints]") {
  checkConeAtCap(ellipticalCone(), 2.0, 100.0);
}

TEST_CASE("a swing-twist cone holds Reflex's shoulder", "[dynamics][joints]") {
  checkConeAtCap(reflexShoulder(), 2.5, 100.0);
}

TEST_CASE("a swing-twist cone holds a light limb pushed hard at a hip-like cone", "[dynamics][joints]") {
  ConeLimit hip = ellipticalCone();
  hip.swing = glm::dvec2(1.3, 0.65);
  hip.centre = glm::dvec2(-0.8, 0.15);

  checkConeAtCap(hip, 2.79, 250.0);
}

TEST_CASE("a swing-twist cone at the smallest half-angle still holds", "[dynamics][joints]") {
  ConeLimit narrow = ellipticalCone();
  narrow.swing = glm::dvec2(1.0e-3);
  narrow.centre = glm::dvec2(0.0);

  checkConeAtCap(narrow, 0.5, 10.0);
}

TEST_CASE("a swing-twist drive holds a swing at the edge of its reach", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double reach = 0.89 * std::numbers::pi;
  const double stiffness = 1.0e4;

  for (uint32_t direction = 0u; direction != 8u; ++direction) {
    const double heading = 0.25 * std::numbers::pi * static_cast<double>(direction) + (direction % 2u == 0u ? 0.0 : 0.1);
    const glm::dvec2 target = reach * glm::dvec2(std::cos(heading), std::sin(heading));
    CAPTURE(direction);

    World world;
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldSwingTwist(world, index);
    def.drive.maxTorque = 100.0;
    def.drive.axes[1].targetAngle = target.x;
    def.drive.axes[2].targetAngle = target.y;

    for (AxisDrive& axis : def.drive.axes) {
      axis.stiffness = stiffness;
      axis.damping = 0.05 * stiffness;
    }

    REQUIRE(addJoint(world, def) != 0u);

    double worst = 0.0;

    for (uint32_t frame = 0u; frame != 600u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      if (frame >= 120u) {
        worst = std::max(worst, glm::length(jointAngles(world, def).swing - target));
      }
    }

    CAPTURE(worst);
    CHECK(worst <= 0.03);
  }
}

TEST_CASE("a swing-twist drive swings a wide cone through its inside", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double start = -1.7;
  const double target = 1.7;
  World world;
  world.parameters.sleep = false;
  world.parameters.gravity = glm::dvec3(0.0);
  const uint32_t index = addHingedCrate(world, crate);
  JointDef def = worldSwingTwist(world, index);
  def.cone = reflexShoulder();
  def.drive.maxTorque = 100.0;
  def.drive.axes[1].targetAngle = target;

  for (AxisDrive& axis : def.drive.axes) {
    axis.stiffness = 100.0;
    axis.damping = 5.0;
  }

  RigidBody& body = world.bodies.rigids[index];
  const glm::dquat raised = glm::angleAxis(start, limbDown * glm::dvec3(0.0, 1.0, 0.0));
  body.position = hingePivot + raised * (body.position - hingePivot);
  body.rotation = raised * body.rotation;
  REQUIRE(addJoint(world, def) != 0u);
  REQUIRE(glm::length(jointAngles(world, def).swing - glm::dvec2(start, 0.0)) < 1.0e-9);

  const double margin = 0.03 / std::min(def.cone.swing.x, def.cone.swing.y);
  double previous = start;
  double furthest = 0.0;
  bool crossed = false;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec2 swing = jointAngles(world, def).swing;

    furthest = std::max(furthest, ellipse(def.cone, swing));
    crossed = crossed || (previous < 0.0 && swing.x >= 0.0 && swing.x - previous < 1.0);
    previous = swing.x;
  }

  const glm::dvec2 settled = jointAngles(world, def).swing;

  CAPTURE(furthest, settled.x, settled.y);
  CHECK(furthest <= (1.0 + margin) * (1.0 + margin));
  CHECK(crossed);
  CHECK(glm::length(settled - glm::dvec2(target, 0.0)) < 0.02);
}

TEST_CASE("a swing-twist twist range holds", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();

  for (const double side : {1.0, -1.0}) {
    CAPTURE(side);

    World world;
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = shoulder(world, index);
    pushAtCap(def);
    def.drive.axes[0].targetAngle = 2.0 * side;
    const double end = side > 0.0 ? def.cone.twistUpper : def.cone.twistLower;
    const uint32_t joint = addJoint(world, def);
    REQUIRE(joint != 0u);

    double overshoot = -1.0;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
      overshoot = std::max(overshoot, side * (jointAngles(world, def).twist - end));
    }

    const JointReport report = *jointReport(world, joint);

    CAPTURE(overshoot);
    CHECK(overshoot <= 0.03);
    CHECK(side * (jointAngles(world, def).twist - end) >= -0.02);
    CHECK(report.limitActive);
    CHECK(report.driveSaturated);
  }
}

TEST_CASE("a swing-twist drive turns a wide twist range through its inside", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double start = 1.05;

  for (const double target : {-2.0, -2.25}) {
    CAPTURE(target);

    World world;
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldSwingTwist(world, index);
    def.cone = reflexShoulder();
    def.drive.maxTorque = 100.0;
    def.drive.axes[0].targetAngle = target;

    for (AxisDrive& axis : def.drive.axes) {
      axis.stiffness = 100.0;
      axis.damping = 5.0;
    }

    world.bodies.rigids[index].rotation = glm::angleAxis(start, limbDown * glm::dvec3(1.0, 0.0, 0.0));
    REQUIRE(addJoint(world, def) != 0u);
    REQUIRE(std::abs(jointAngles(world, def).twist - start) < 1.0e-9);

    double previous = start;
    double highest = start;
    bool crossed = false;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      const double twist = jointAngles(world, def).twist;

      highest = std::max(highest, twist);
      crossed = crossed || (previous > 0.0 && twist <= 0.0 && previous - twist < 1.0);
      previous = twist;
    }

    CAPTURE(highest, previous);
    CHECK(highest <= def.cone.twistUpper + 0.03);
    CHECK(crossed);
    CHECK(std::abs(previous - target) < 0.02);
  }
}

TEST_CASE("a swing-twist drive deflects per axis", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const glm::dvec3 stiffness(100.0, 200.0, 400.0);
  const double torque = 10.0;

  for (int axis = 0; axis != 3; ++axis) {
    CAPTURE(axis);

    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldSwingTwist(world, index);
    glm::dvec3 local(0.0);
    local[axis] = torque;
    def.drive.feedForward = limbDown * local;

    for (int other = 0; other != 3; ++other) {
      def.drive.axes[other].stiffness = stiffness[other];
      def.drive.axes[other].damping = 20.0;
    }

    const uint32_t joint = addJoint(world, def);
    REQUIRE(joint != 0u);

    for (uint32_t frame = 0u; frame != 180u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    }

    const SwingTwistAngles angles = jointAngles(world, def);
    const glm::dvec3 deflection(angles.twist, angles.swing.x, angles.swing.y);
    const double expected = torque / stiffness[axis];
    const JointReport report = *jointReport(world, joint);

    CAPTURE(deflection.x, deflection.y, deflection.z);
    CHECK(std::abs(deflection[axis] - expected) <= 0.02 * expected);
    CHECK(std::abs(report.driveTorque[axis] + torque) <= 0.02 * torque);

    for (int other = 0; other != 3; ++other) {
      if (other != axis) {
        CHECK(std::abs(deflection[other]) < 0.002);
      }
    }
  }
}

TEST_CASE("a swing-twist drive reaches its target in Reflex's angles", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;
  const glm::dvec3 target(0.3, 0.4, -0.2);
  const uint32_t index = addHingedCrate(world, crate);
  JointDef def = worldSwingTwist(world, index);

  for (int axis = 0; axis != 3; ++axis) {
    def.drive.axes[axis].targetAngle = target[axis];
    def.drive.axes[axis].stiffness = 50.0;
    def.drive.axes[axis].damping = 5.0;
  }

  REQUIRE(addJoint(world, def) != 0u);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const SwingTwistAngles angles = jointAngles(world, def);

  CAPTURE(angles.twist, angles.swing.x, angles.swing.y);
  CHECK(std::abs(angles.twist - target.x) < 1.0e-3);
  CHECK(std::abs(angles.swing.x - target.y) < 1.0e-3);
  CHECK(std::abs(angles.swing.y - target.z) < 1.0e-3);
}

TEST_CASE("extra limit passes visit only joints at or near a limit", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();

  {
    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    const uint32_t hinged = addHingedCrate(world, crate);
    const glm::dvec3 pivot = hingePivot + glm::dvec3(1.0, 0.0, 0.0);
    const uint32_t twisted =
      addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, pivot - hingeLever, glm::dvec3(0.0), 0.5, 3u);
    JointDef hinge = worldHinge(world, hinged, hingePivot);
    hinge.hinge = {true, -0.5, 0.5};
    JointDef cone = worldHinge(world, twisted, pivot);
    cone.kind = JointKind::SwingTwist;
    cone.cone = {true, -0.5, 0.5, glm::dvec2(0.5), glm::dvec2(0.0)};
    REQUIRE(addJoint(world, hinge) != 0u);
    REQUIRE(addJoint(world, cone) != 0u);
    const auto spin = [&](uint32_t index, const glm::dvec3& about, const glm::dvec3& rate) {
      RigidBody& body = world.bodies.rigids[index];
      body.angularVelocity = rate;
      body.velocity = glm::cross(rate, body.position - about);
    };

    spin(hinged, hingePivot, glm::dvec3(0.3, 0.0, 0.0));
    spin(twisted, pivot, glm::dvec3(0.3, 0.2, 0.0));

    for (uint32_t frame = 0u; frame != 30u; ++frame) {
      const StepReport report = talus::dynamics::step(world);

      REQUIRE_FALSE(report.nonFinite);
      CHECK(report.jointRows == 2u);
      CHECK(report.limitPassRows == 0u);
    }

    CHECK(hingeAngle(world, hinge) > 0.1);
    CHECK(jointAngles(world, cone).twist > 0.1);
  }

  const auto restingRows = [&](double lower) {
    World world;
    world.parameters.gravity = glm::dvec3(0.0);
    world.parameters.sleep = false;
    const uint32_t index = addHingedCrate(world, crate);
    JointDef def = worldHinge(world, index, hingePivot);
    def.hinge = {true, lower, 0.5};
    REQUIRE(addJoint(world, def) != 0u);
    std::vector<uint32_t> rows;

    for (uint32_t frame = 0u; frame != 10u; ++frame) {
      const StepReport report = talus::dynamics::step(world);

      REQUIRE_FALSE(report.nonFinite);
      rows.push_back(report.limitPassRows);
    }

    return rows;
  };

  CHECK(restingRows(-0.04) == std::vector<uint32_t>(10u, 4u * IntegrationParameters{}.jointSubsteps));
  CHECK(restingRows(-0.06) == std::vector<uint32_t>(10u, 0u));
}

TEST_CASE("swing-twist ranges, cones and drive targets beyond their bounds are rejected", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  const uint32_t index = addHingedCrate(world, crate);
  const double pi = std::numbers::pi;

  const glm::dvec2 cone(0.6, 0.3);
  const glm::dvec2 offset(0.1, 0.0);
  const glm::dvec3 still(0.0);

  const auto accepts = [&](double lower, double upper, const glm::dvec2& swing, const glm::dvec2& centre, const glm::dvec3& target) {
    JointDef def = shoulder(world, index);
    def.cone.twistLower = lower;
    def.cone.twistUpper = upper;
    def.cone.swing = swing;
    def.cone.centre = centre;

    for (int axis = 0; axis != 3; ++axis) {
      def.drive.axes[axis].targetAngle = target[axis];
      def.drive.axes[axis].stiffness = 20.0;
    }

    const uint32_t joint = addJoint(world, def);

    if (joint != 0u) {
      REQUIRE(removeJoint(world, joint));
    }

    return joint != 0u;
  };

  const glm::dvec2 widest(0.89 * pi);

  CHECK_FALSE(accepts(-0.99 * pi, 0.99 * pi, widest, glm::dvec2(0.0), glm::dvec3(pi, -pi, pi)));
  CHECK(accepts(-0.99 * pi, 0.99 * pi, widest, glm::dvec2(0.0), glm::dvec3(pi, 0.62 * pi, -0.63 * pi)));
  CHECK_FALSE(accepts(-0.99 * pi, 0.99 * pi, widest, glm::dvec2(0.0), glm::dvec3(pi, 0.63 * pi, -0.63 * pi)));
  CHECK(accepts(-0.4, 0.2, glm::dvec2(0.889 * pi, 0.3), glm::dvec2(0.0), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.891 * pi, 0.3), glm::dvec2(0.0), still));
  CHECK(accepts(-0.4, 0.2, glm::dvec2(1.85, 1.86), glm::dvec2(0.93, 0.0), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(1.85, 1.86), glm::dvec2(0.94, 0.0), still));
  CHECK(accepts(-0.4, 0.2, cone, offset, still));
  CHECK(accepts(-2.3, 1.1, glm::dvec2(1.85, 1.86), glm::dvec2(0.0), still));
  CHECK(accepts(-0.6, 0.6, glm::dvec2(1.3, 0.65), glm::dvec2(-0.8, 0.15), still));
  CHECK(accepts(-0.4, 0.2, glm::dvec2(1.0e-3), glm::dvec2(0.0), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.999e-3, 0.3), offset, still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.6, 0.999e-3), offset, still));
  CHECK_FALSE(accepts(-3.12, 0.2, cone, offset, still));
  CHECK_FALSE(accepts(-0.4, 3.12, cone, offset, still));
  CHECK_FALSE(accepts(0.2, -0.4, cone, offset, still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.0, 0.3), offset, still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.6, -0.3), offset, still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(3.12, 0.3), glm::dvec2(0.0), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.6, 3.12), glm::dvec2(0.0), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(1.85, 1.86), glm::dvec2(1.0, 0.8), still));
  CHECK_FALSE(accepts(-0.4, 0.2, glm::dvec2(0.99 * pi, 0.3), offset, still));
  CHECK_FALSE(accepts(-0.4, 0.2, cone, offset, glm::dvec3(3.2, 0.0, 0.0)));
  CHECK_FALSE(accepts(-0.4, 0.2, cone, offset, glm::dvec3(0.0, -3.2, 0.0)));
  CHECK_FALSE(accepts(-0.4, 0.2, cone, offset, glm::dvec3(0.0, 0.0, 3.2)));
  CHECK(world.joints.empty());

  JointDef unlimited = shoulder(world, index);
  unlimited.cone.enabled = false;
  unlimited.cone.swing = glm::dvec2(0.0);
  unlimited.cone.twistLower = -4.0;
  const uint32_t joint = addJoint(world, unlimited);
  REQUIRE(joint != 0u);

  JointDrive drive = unlimited.drive;
  drive.axes[1].targetAngle = 3.0;
  drive.axes[2].targetAngle = 3.0;
  CHECK_FALSE(setJointDrive(world, joint, drive));
  drive.axes[1].targetAngle = 2.0;
  drive.axes[2].targetAngle = 2.0;
  CHECK_FALSE(setJointDrive(world, joint, drive));
  drive.axes[1].targetAngle = 1.9;
  drive.axes[2].targetAngle = 1.9;
  CHECK(setJointDrive(world, joint, drive));
}

namespace {

JointDef worldSpring(uint32_t body, const glm::dvec3& target) {
  JointDef def = between(noBody, body);
  def.kind = JointKind::Spring;
  def.spring.target.position = target;
  def.spring.linearStiffness = 5000.0;
  def.spring.linearDamping = 200.0;
  def.spring.maxForce = 50.0;

  return def;
}

struct SpringHold {
  glm::dvec3 offset{0.0};
  double furthest = 0.0;
  double strongest = 0.0;
  bool everSaturated = false;
  JointReport report;
};

SpringHold holdBox(double mass) {
  const talus::field::SampledSdf crate = crateField();
  const glm::dvec3 target(0.0, 2.0, 0.0);
  World world;
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), mass, target, glm::dvec3(0.0), 0.5, 3u);
  const uint32_t joint = addJoint(world, worldSpring(world.bodies.rigids[index].id, target));
  REQUIRE(joint != 0u);

  SpringHold hold;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    hold.report = *jointReport(world, joint);
    hold.offset = world.bodies.rigids[index].position - target;
    hold.furthest = std::max(hold.furthest, glm::length(hold.offset));
    hold.strongest = std::max(hold.strongest, glm::length(hold.report.force));
    hold.everSaturated = hold.everSaturated || hold.report.forceSaturated;
  }

  return hold;
}

}

TEST_CASE("a capped spring holds a light box and lets a heavy one slip", "[dynamics][joints]") {
  const double cap = 50.0;
  const double gravity = std::abs(IntegrationParameters{}.gravity.y);
  const SpringHold light = holdBox(3.0);
  const SpringHold heavy = holdBox(10.0);
  const double sag = 3.0 * gravity / 5000.0;

  CAPTURE(light.furthest, light.offset.y, light.strongest, heavy.offset.y, heavy.strongest);
  CHECK(light.furthest < 0.02);
  CHECK_FALSE(light.everSaturated);
  CHECK(std::abs(-light.offset.y - sag) <= 0.02 * sag);
  CHECK(light.strongest <= 1.01 * cap);
  CHECK(heavy.offset.y < -1.0);
  CHECK(heavy.report.forceSaturated);
  CHECK(std::abs(glm::length(heavy.report.force) - cap) <= 0.01 * cap);
  CHECK(heavy.strongest <= 1.01 * cap);
}

TEST_CASE("a spring target moved every step is followed", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const glm::dvec3 start(0.0, 2.0, 0.0);
  const glm::dvec3 speed(1.0, 0.0, 0.0);
  World world;
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 3.0, start, glm::dvec3(0.0), 0.5, 3u);
  const uint32_t joint = addJoint(world, worldSpring(world.bodies.rigids[index].id, start));
  REQUIRE(joint != 0u);

  const double dt = world.parameters.dt;
  const uint32_t steps = static_cast<uint32_t>(std::lround(2.0 / dt));
  JointFrame target;

  for (uint32_t frame = 1u; frame <= steps; ++frame) {
    target.position = start + dt * static_cast<double>(frame) * speed;
    REQUIRE(setSpringTarget(world, joint, target));
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 lag = target.position - world.bodies.rigids[index].position;

  CAPTURE(lag.x, lag.y, lag.z);
  CHECK(glm::length(lag) < 0.05);
}

TEST_CASE("a capped spring's force stays within its cap in any direction", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double cap = 50.0;
  const glm::dvec3 start(0.0, 1.0, 0.0);
  World world;
  addStatic(world.bodies, floor(), 0.5, 0.0, glm::dvec3(0.2, 1.0, 0.0), glm::angleAxis(0.5 * std::numbers::pi, glm::dvec3(0.0, 0.0, 1.0)));
  const uint32_t index = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 3.0, start, glm::dvec3(0.0), 0.5, 3u);
  const uint32_t joint = addJoint(world, worldSpring(world.bodies.rigids[index].id, start + glm::dvec3(0.6, 0.0, 0.0)));
  REQUIRE(joint != 0u);

  double strongest = 0.0;
  JointReport report;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    report = *jointReport(world, joint);
    strongest = std::max(strongest, glm::length(report.force));
  }

  const glm::dvec3 position = world.bodies.rigids[index].position;

  CAPTURE(strongest, report.force.x, report.force.y, report.force.z, position.x, position.y);
  CHECK(strongest <= 1.01 * cap);
  CHECK(report.forceSaturated);
  CHECK(std::abs(report.force.y) > 0.1 * cap);
  CHECK(std::abs(position.x - 0.05) < 0.01);
}

TEST_CASE("a spring turns its body to the target rotation within its torque cap", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double cap = 0.5;
  const glm::dvec3 centre(0.0, 2.0, 0.0);
  const glm::dquat turn = glm::angleAxis(0.8, glm::normalize(glm::dvec3(1.0, 1.0, 0.0)));
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;
  const uint32_t box = addCrate(world, crate, centre);
  JointDef def = worldSpring(box, centre);
  def.spring.target.rotation = turn;
  def.spring.angularStiffness = 2.0;
  def.spring.angularDamping = 0.2;
  def.spring.maxTorque = cap;
  const uint32_t joint = addJoint(world, def);
  REQUIRE(joint != 0u);

  double strongest = 0.0;
  bool startSaturated = false;
  JointReport report;

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    report = *jointReport(world, joint);
    strongest = std::max(strongest, glm::length(report.torque));
    startSaturated = startSaturated || (frame == 0u && report.torqueSaturated);
  }

  const RigidBody& body = world.bodies.rigids[findBody(world.bodies, box)->index];
  const glm::dquat rest = glm::conjugate(turn) * body.rotation;
  const double error = 2.0 * std::asin(std::min(1.0, glm::length(glm::dvec3(rest.x, rest.y, rest.z))));

  CAPTURE(strongest, error);
  CHECK(startSaturated);
  CHECK(strongest <= 1.01 * cap);
  CHECK(error < 1.0e-3);
  CHECK(glm::length(body.position - centre) < 1.0e-3);
  CHECK_FALSE(report.torqueSaturated);
  CHECK_FALSE(report.forceSaturated);
}

TEST_CASE("an off-centre spring grip deflects by its load over its stiffness", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double mass = 3.0;
  const double lever = 0.05;
  const double stiffness = 20.0;
  const glm::dvec3 grip(0.0, 2.0, 0.0);
  World world;
  const uint32_t index =
    addRigidBox(world.bodies, crate, glm::dvec3(0.05), mass, grip + glm::dvec3(lever, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);
  JointDef def = worldSpring(world.bodies.rigids[index].id, grip);
  def.frameB.position = glm::dvec3(-lever, 0.0, 0.0);
  def.spring.angularStiffness = stiffness;
  def.spring.angularDamping = 1.0;
  const uint32_t joint = addJoint(world, def);
  REQUIRE(joint != 0u);

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  const RigidBody& body = world.bodies.rigids[index];
  const glm::dvec3 anchor = body.position + body.rotation * def.frameB.position;
  const double weight = mass * std::abs(world.parameters.gravity.y);
  const double angle = -2.0 * std::asin(body.rotation.z);
  double expected = 0.0;

  for (int pass = 0; pass != 8; ++pass) {
    expected = weight * lever * std::cos(expected) / stiffness;
  }

  CAPTURE(angle, expected, anchor.x, anchor.y, anchor.z);
  CHECK(std::abs(angle - expected) <= 0.02 * expected);
  CHECK(std::abs(grip.y - anchor.y - weight / 5000.0) <= 0.02 * weight / 5000.0);
  CHECK(std::hypot(anchor.x - grip.x, anchor.z - grip.z) < 1.0e-3);
}

namespace {

const glm::dvec3 gripPoint(0.0, 2.0, 0.0);
const glm::dvec3 handReach(0.0, 0.3, 0.0);

struct HandGrip {
  uint32_t hand = 0u;
  uint32_t box = 0u;
  uint32_t joint = 0u;
};

HandGrip gripBox(World& world, const talus::field::SampledSdf& crate) {
  HandGrip grip;
  grip.hand = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 0.0, gripPoint + handReach, glm::dvec3(0.0), 0.5, 3u);
  grip.box = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 3.0, gripPoint, glm::dvec3(0.0), 0.5, 3u);
  JointDef def = worldSpring(world.bodies.rigids[grip.box].id, glm::dvec3(0.0));
  def.bodyA = world.bodies.rigids[grip.hand].id;
  def.frameA.position = -handReach;
  grip.joint = addJoint(world, def);
  REQUIRE(grip.joint != 0u);

  return grip;
}

void moveHand(World& world, const HandGrip& grip, const glm::dvec3& position, const glm::dvec3& velocity) {
  const uint32_t id = world.bodies.rigids[grip.hand].id;

  REQUIRE(setRigidPose(world, id, position, glm::dquat(1.0, 0.0, 0.0, 0.0)));
  REQUIRE(setVelocity(world, id, velocity));
}

glm::dvec3 gripSlack(const World& world, const HandGrip& grip) {
  const RigidBody& hand = world.bodies.rigids[grip.hand];

  return hand.position + hand.rotation * -handReach - world.bodies.rigids[grip.box].position;
}

}

TEST_CASE("a spring grip follows a moving kinematic hand without lag", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double weight = 3.0 * std::abs(IntegrationParameters{}.gravity.y);
  const double bound = 1.0e-3;

  for (const double speed : {1.0, 3.0}) {
    CAPTURE(speed);

    World world;
    const HandGrip grip = gripBox(world, crate);
    const glm::dvec3 velocity(speed, 0.0, 0.0);
    const double dt = world.parameters.dt;
    double furthest = 0.0;
    double weakest = jointInfinity;
    double strongest = 0.0;

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      moveHand(world, grip, gripPoint + handReach + dt * static_cast<double>(frame) * velocity, velocity);
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      if (frame >= 60u) {
        const double force = glm::length(jointReport(world, grip.joint)->force);

        furthest = std::max(furthest, std::abs(gripSlack(world, grip).x));
        weakest = std::min(weakest, force);
        strongest = std::max(strongest, force);
      }
    }

    CAPTURE(furthest, weakest, strongest);
    CHECK(furthest < bound);
    CHECK(std::abs(weakest - weight) <= 0.01 * weight);
    CHECK(std::abs(strongest - weight) <= 0.01 * weight);
  }
}

TEST_CASE("a spring grip slips at its cap under a fast-rising hand and recovers below it", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double cap = 50.0;
  const double weight = 3.0 * std::abs(IntegrationParameters{}.gravity.y);
  World world;
  const HandGrip grip = gripBox(world, crate);
  const glm::dvec3 velocity(0.0, 3.0, 0.0);
  const double dt = world.parameters.dt;
  double strongest = 0.0;
  bool slipped = false;
  JointReport report;

  for (uint32_t frame = 0u; frame != 180u; ++frame) {
    const double moved = dt * static_cast<double>(std::min(frame, 60u));

    moveHand(world, grip, gripPoint + handReach + moved * velocity, frame < 60u ? velocity : glm::dvec3(0.0));
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    report = *jointReport(world, grip.joint);
    strongest = std::max(strongest, glm::length(report.force));
    slipped = slipped || report.forceSaturated;
  }

  const glm::dvec3 slack = gripSlack(world, grip);

  CAPTURE(strongest, report.force.y, slack.x, slack.y, slack.z);
  CHECK(slipped);
  CHECK(strongest <= 1.01 * cap);
  CHECK_FALSE(report.forceSaturated);
  CHECK(std::abs(glm::length(report.force) - weight) <= 0.01 * weight);
  CHECK(glm::length(slack - glm::dvec3(0.0, weight / 5000.0, 0.0)) < 1.0e-3);
}

namespace {

const double humanoidTargetReach = 0.88 * std::numbers::pi;

glm::dvec3 jointAnchor(const World& world, uint32_t id, const glm::dvec3& local) {
  if (id == noBody) {
    return local;
  }

  const RigidBody& body = world.bodies.rigids[findBody(world.bodies, id)->index];

  return body.position + body.rotation * local;
}

double anchorGap(const World& world, const JointDef& def) {
  const glm::dvec3 a = jointAnchor(world, def.bodyA, def.frameA.position);
  const glm::dvec3 b = jointAnchor(world, def.bodyB, def.frameB.position);

  return glm::length(b - a);
}

double outsideEllipse(const glm::dvec2& offset, const glm::dvec2& half) {
  const glm::dvec2 point = glm::abs(offset);
  const glm::dvec2 squared = half * half;
  const auto closest = [&](double t) { return squared * point / (glm::dvec2(t) + squared); };
  const auto extent = [&](double t) {
    const glm::dvec2 scaled = closest(t) / half;

    return glm::dot(scaled, scaled);
  };

  if (extent(0.0) <= 1.0) {
    return 0.0;
  }

  double low = 0.0;
  double high = glm::length(half * point);

  for (int iteration = 0; iteration != 100; ++iteration) {
    const double middle = 0.5 * (low + high);

    if (extent(middle) > 1.0) {
      low = middle;
    } else {
      high = middle;
    }
  }

  return glm::length(point - closest(high));
}

glm::dvec2 swingBeyondCone(const ConeLimit& cone, double heading) {
  const glm::dvec2 direction(std::cos(heading), std::sin(heading));
  const glm::dvec2 edge = cone.centre + direction / glm::length(direction / cone.swing);
  const double along = glm::dot(edge, direction);
  const double room =
    std::sqrt(along * along - glm::dot(edge, edge) + humanoidTargetReach * humanoidTargetReach) - along;

  return edge + std::min(1.0, room) * direction;
}

struct Overshoot {
  double angle = 0.0;
  double side = 0.0;
  uint32_t direction = 0u;
};

struct LimitOvershoot {
  Overshoot range;
  Overshoot cone;
  bool engaged = false;
};

struct HumanoidPush {
  std::vector<LimitOvershoot> joints;
  double gap = 0.0;
  uint32_t contacts = 0u;
};

struct HumanoidLimits {
  std::vector<LimitOvershoot> worst;
  std::vector<uint32_t> idle;
  double gap = 0.0;
  uint32_t contacts = 0u;
};

struct HumanoidGains {
  const char* label = "";
  double stiffness = 0.0;
  double damping = 0.0;
  double hertz = 0.0;
};

const HumanoidGains stiffHumanoidGains{"k 1e5, c 1e3", 1.0e5, 1.0e3, 0.0};
const HumanoidGains reflexHumanoidGains{"Reflex 10 Hz, damping ratio 1", 0.0, 0.0, 10.0};

void keepWorse(Overshoot& worst, const Overshoot& candidate) {
  if (candidate.angle > worst.angle) {
    worst = candidate;
  }
}

double inertiaBeyond(const World& world, const fixture::Humanoid& humanoid, const JointDef& def) {
  const glm::dvec3 anchor = jointAnchor(world, def.bodyB, def.frameB.position);
  std::vector<uint32_t> members{def.bodyB};
  double inertia = 0.0;

  for (size_t grown = 0u; grown != members.size(); ++grown) {
    for (const uint32_t id : humanoid.joints) {
      const JointDef& other = world.joints.find(id)->def;

      if (other.bodyA == members[grown]) {
        members.push_back(other.bodyB);
      }
    }
  }

  for (const uint32_t member : members) {
    const RigidBody& body = world.bodies.rigids[findBody(world.bodies, member)->index];
    const glm::dvec3 offset = body.position - anchor;

    inertia += body.mass * glm::dot(offset, offset) + std::max({body.moment.x, body.moment.y, body.moment.z});
  }

  return inertia;
}

HumanoidPush pushHumanoidLimits(const HumanoidGains& gains, bool selfContact, double side, uint32_t direction) {
  World world;
  world.parameters.sleep = false;
  const fixture::Humanoid humanoid = fixture::addHumanoid(world, glm::dvec3(0.0, 2.0, 0.0), true);
  const double heading = 0.25 * std::numbers::pi * static_cast<double>(direction);
  const double omega = 2.0 * std::numbers::pi * gains.hertz;
  std::vector<JointDef> defs;

  if (!selfContact) {
    fixture::excludeSelfContact(world, humanoid);
  }

  for (const uint32_t id : humanoid.joints) {
    JointDef def = world.joints.find(id)->def;
    const bool hinge = def.kind == JointKind::Hinge;
    const double end = side > 0.0 ? (hinge ? def.hinge.upper : def.cone.twistUpper)
                                  : (hinge ? def.hinge.lower : def.cone.twistLower);
    const double inertia = gains.hertz > 0.0 ? inertiaBeyond(world, humanoid, def) : 0.0;
    def.drive.axes[0].targetAngle = std::clamp(end + side, -humanoidTargetReach, humanoidTargetReach);

    if (!hinge) {
      const glm::dvec2 swing = swingBeyondCone(def.cone, heading);
      def.drive.axes[1].targetAngle = swing.x;
      def.drive.axes[2].targetAngle = swing.y;
    }

    for (size_t axis = 0u; axis != (hinge ? 1u : 3u); ++axis) {
      def.drive.axes[axis].stiffness = gains.hertz > 0.0 ? omega * omega * inertia : gains.stiffness;
      def.drive.axes[axis].damping = gains.hertz > 0.0 ? 2.0 * omega * inertia : gains.damping;
    }

    REQUIRE(setJointDrive(world, id, def.drive));
    defs.push_back(def);
  }

  HumanoidPush push;
  push.joints.resize(defs.size());

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const StepReport report = talus::dynamics::step(world);

    REQUIRE_FALSE(report.nonFinite);
    push.contacts = std::max(push.contacts, report.contacts);

    for (size_t joint = 0u; joint != defs.size(); ++joint) {
      const JointDef& def = defs[joint];
      LimitOvershoot& worst = push.joints[joint];
      double range = 0.0;
      double cone = 0.0;

      if (def.kind == JointKind::Hinge) {
        const double angle = hingeAngle(world, def);
        range = std::max(angle - def.hinge.upper, def.hinge.lower - angle);
      } else {
        const SwingTwistAngles angles = jointAngles(world, def);
        range = std::max(angles.twist - def.cone.twistUpper, def.cone.twistLower - angles.twist);
        cone = outsideEllipse(angles.swing - def.cone.centre, def.cone.swing);
      }

      keepWorse(worst.range, {range, side, direction});
      keepWorse(worst.cone, {cone, side, direction});
      worst.engaged = worst.engaged || jointReport(world, humanoid.joints[joint])->limitActive;
      push.gap = std::max(push.gap, anchorGap(world, def));
    }
  }

  return push;
}

void printOvershoot(const char* label, const Overshoot& overshoot) {
  if (!(overshoot.angle > 0.0)) {
    std::printf(" %s within its limit", label);

    return;
  }

  std::printf(
    " %s %.4f rad (%s, heading %3.0f deg)",
    label,
    overshoot.angle,
    overshoot.side > 0.0 ? "upper" : "lower",
    45.0 * static_cast<double>(overshoot.direction)
  );
}

HumanoidLimits pushEveryHumanoidLimit(const HumanoidGains& gains, bool selfContact) {
  const size_t count = fixture::humanoidJointNames.size();
  HumanoidLimits limits;
  limits.worst.resize(count);
  limits.idle.resize(count, 0u);

  for (const double side : {1.0, -1.0}) {
    for (uint32_t direction = 0u; direction != 8u; ++direction) {
      const HumanoidPush push = pushHumanoidLimits(gains, selfContact, side, direction);

      for (size_t joint = 0u; joint != count; ++joint) {
        keepWorse(limits.worst[joint].range, push.joints[joint].range);
        keepWorse(limits.worst[joint].cone, push.joints[joint].cone);
        limits.idle[joint] += push.joints[joint].engaged ? 0u : 1u;
      }

      limits.gap = std::max(limits.gap, push.gap);
      limits.contacts = std::max(limits.contacts, push.contacts);
    }
  }

  return limits;
}

void checkHumanoidLimits(const HumanoidGains& gains, const HumanoidLimits& limits) {
  for (size_t joint = 0u; joint != limits.worst.size(); ++joint) {
    const char* name = fixture::humanoidJointNames[joint];
    const LimitOvershoot& overshoot = limits.worst[joint];

    CAPTURE(gains.label, name, overshoot.range.angle, overshoot.range.side, overshoot.range.direction);
    CAPTURE(overshoot.cone.angle, overshoot.cone.side, overshoot.cone.direction);
    CHECK(overshoot.range.angle <= 0.03);
    CHECK(overshoot.cone.angle <= 0.03);
    CHECK(limits.idle[joint] == 0u);
  }
}

}

TEST_CASE("every humanoid limit holds with its drive at the cap", "[dynamics][joints][!mayfail]") {
  const HumanoidLimits limits = pushEveryHumanoidLimit(stiffHumanoidGains, true);

  for (size_t joint = 0u; joint != limits.worst.size(); ++joint) {
    std::printf("humanoid limit %-13s", fixture::humanoidJointNames[joint]);
    printOvershoot("hinge or twist", limits.worst[joint].range);
    printOvershoot("cone", limits.worst[joint].cone);
    std::printf("\n");
  }

  std::printf("humanoid limit largest anchor gap %.4f m\n", limits.gap);
  checkHumanoidLimits(stiffHumanoidGains, limits);
}

TEST_CASE("every humanoid limit holds with all joints at the cap, without self-contact", "[dynamics][joints]") {
  for (const HumanoidGains& gains : {stiffHumanoidGains, reflexHumanoidGains}) {
    const HumanoidLimits limits = pushEveryHumanoidLimit(gains, false);
    size_t worst = 0u;

    for (size_t joint = 0u; joint != limits.worst.size(); ++joint) {
      const LimitOvershoot& overshoot = limits.worst[joint];
      const LimitOvershoot& before = limits.worst[worst];

      if (std::max(overshoot.range.angle, overshoot.cone.angle) > std::max(before.range.angle, before.cone.angle)) {
        worst = joint;
      }
    }

    std::printf(
      "humanoid limit without self-contact, %s: worst %-13s range %.4f cone %.4f rad, largest anchor gap %.4f m\n",
      gains.label,
      fixture::humanoidJointNames[worst],
      limits.worst[worst].range.angle,
      limits.worst[worst].cone.angle,
      limits.gap
    );

    CHECK(limits.contacts == 0u);
    checkHumanoidLimits(gains, limits);
  }
}

namespace {

struct HangingChain {
  std::vector<uint32_t> links;
  std::vector<JointDef> defs;
};

const glm::dvec3 chainTop(0.0, 5.0, 0.0);

glm::dvec3 chainLinkAt(const glm::dvec3& half, uint32_t link) {
  return chainTop + (2.0 * static_cast<double>(link) + 1.0) * half;
}

HangingChain addHangingChain(World& world, const talus::field::SampledSdf& crate, double release) {
  const glm::dvec3 half = 0.1 * glm::dvec3(std::sin(release), -std::cos(release), 0.0);
  HangingChain chain;

  for (uint32_t link = 0u; link != 20u; ++link) {
    const double mass = link % 2u == 0u ? 40.0 : 1.0;
    const uint32_t index =
      addRigidBox(world.bodies, crate, glm::dvec3(0.05), mass, chainLinkAt(half, link), glm::dvec3(0.0), 0.5, 3u);
    JointDef def = between(chain.links.empty() ? noBody : world.bodies.rigids[chain.links.back()].id, world.bodies.rigids[index].id);
    def.frameA.position = chain.links.empty() ? chainTop : half;
    def.frameB.position = -half;
    REQUIRE(addJoint(world, def) != 0u);
    chain.links.push_back(index);
    chain.defs.push_back(def);
  }

  return chain;
}

double chainPotential(const World& world, const RigidBody& body, const glm::dvec3& position) {
  return -body.mass * glm::dot(world.parameters.gravity, position);
}

double chainEnergy(const World& world, const HangingChain& chain) {
  double energy = 0.0;

  for (const uint32_t index : chain.links) {
    const RigidBody& body = world.bodies.rigids[index];
    const glm::dvec3 spin = glm::conjugate(body.rotation) * body.angularVelocity;

    energy += 0.5 * body.mass * glm::dot(body.velocity, body.velocity) + 0.5 * glm::dot(spin, body.moment * spin)
      + chainPotential(world, body, body.position);
  }

  return energy;
}

double largestGap(const World& world, const HangingChain& chain) {
  double gap = 0.0;

  for (const JointDef& def : chain.defs) {
    gap = std::max(gap, anchorGap(world, def));
  }

  return gap;
}

}

TEST_CASE("a humanoid-mass chain hangs at rest without creep", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  world.parameters.sleep = false;
  const HangingChain chain = addHangingChain(world, crate, 0.0);
  const uint32_t frames = static_cast<uint32_t>(std::lround(60.0 / world.parameters.dt));
  const uint32_t lastStretch = frames - static_cast<uint32_t>(std::lround(10.0 / world.parameters.dt));
  const uint32_t lowest = chain.links.back();
  glm::dvec3 from(0.0);
  double creep = 0.0;
  double gap = 0.0;

  for (uint32_t frame = 1u; frame <= frames; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec3 bottom = world.bodies.rigids[lowest].position;

    if (frame == lastStretch) {
      from = bottom;
    } else if (frame > lastStretch) {
      creep = std::max(creep, glm::length(bottom - from));
    }

    gap = std::max(gap, largestGap(world, chain));
  }

  std::printf("humanoid-mass chain at rest largest anchor gap %.4f m\n", gap);

  CAPTURE(creep);
  CHECK(creep < 0.005);
}

TEST_CASE("a humanoid-mass chain swings without gaining energy", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  World world;
  world.parameters.sleep = false;
  const HangingChain chain = addHangingChain(world, crate, 0.3);
  double rest = 0.0;

  for (uint32_t link = 0u; link != chain.links.size(); ++link) {
    const RigidBody& body = world.bodies.rigids[chain.links[link]];
    rest += chainPotential(world, body, chainLinkAt(glm::dvec3(0.0, -0.1, 0.0), link));
  }

  const double start = chainEnergy(world, chain);
  const double swing = start - rest;
  const uint32_t frames = static_cast<uint32_t>(std::lround(60.0 / world.parameters.dt));
  double gain = -std::numeric_limits<double>::infinity();
  double fastest = 0.0;
  double gap = 0.0;

  for (uint32_t frame = 1u; frame <= frames; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    for (const uint32_t index : chain.links) {
      fastest = std::max(fastest, glm::length(world.bodies.rigids[index].velocity));
    }

    gain = std::max(gain, chainEnergy(world, chain) - start);
    gap = std::max(gap, largestGap(world, chain));
  }

  std::printf("humanoid-mass chain swinging largest anchor gap %.4f m\n", gap);

  CAPTURE(swing, gain, fastest);
  REQUIRE(swing > 0.0);
  CHECK(fastest <= 5.0);
  CHECK(gain <= 0.01 * swing);
}

TEST_CASE("a jointed chain touching a lattice keeps its joints", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const SoftBlock slab = chainSlab();
  World world;
  const LatticeChain chain = addLatticeChain(world, crate, slab);
  const uint32_t end = findBody(world.bodies, chain.links.front())->index;
  const uint32_t tip = findBody(world.bodies, chain.links.back())->index;
  double widest = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    const StepReport report = talus::dynamics::step(world);
    REQUIRE_FALSE(report.nonFinite);
    requireStrainstep(world, end);
    CHECK(report.jointRows == 4u);
    CHECK(report.substeps >= world.parameters.jointSubsteps);

    for (const JointDef& def : chain.defs) {
      widest = std::max(widest, anchorGap(world, def));
    }
  }

  CAPTURE(world.bodies.rigids[tip].position.y);
  CHECK(widest <= 0.005);
  CHECK(world.bodies.rigids[tip].position.y < 0.2);
}

namespace {

talus::PhysicalMaterial softRubber() {
  talus::PhysicalMaterial material;
  material.density = 1000.0f;
  material.youngsModulus = 4.0e4f;
  material.poissonsRatio = 0.45f;

  return material;
}

int firstPromotion(bool jointed) {
  const std::vector<talus::PhysicalMaterial> lanes{softRubber()};
  const talus::field::SampledSdf blockField = boxOf(glm::dvec3(0.1, 0.025, 0.1), 0.025);
  const talus::field::SampledSdf crate = boxOf(glm::dvec3(0.025), 0.005);
  World world;
  addFloor(world);
  const uint32_t block = addSwitchable(
    world.bodies,
    talus::deformable::buildMesh(
      blockField,
      lanes,
      glm::mat4(1.0f),
      std::span<const talus::deformable::SupportVolume>{},
      talus::deformable::VolumeElementType::Hex
    ),
    lanes,
    blockField,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  (void)addRigidBox(world.bodies, crate, glm::dvec3(0.025), 2.0, glm::dvec3(0.0, 0.175, 0.0), glm::dvec3(0.0), 0.5, 3u);

  if (jointed) {
    JointDef def = between(noBody, block);
    def.frameA.position = world.bodies.rigids[findBody(world.bodies, block)->index].position;
    REQUIRE(addJoint(world, def) != 0u);
  }

  for (int frame = 0; frame != 30; ++frame) {
    const StepReport report = talus::dynamics::step(world);
    REQUIRE_FALSE(report.nonFinite);

    if (report.promotions != 0u || world.bodies.switchables[0].tier != Tier::Rigid) {
      return frame;
    }
  }

  return -1;
}

}

TEST_CASE("a jointed switchable body stays rigid when struck", "[dynamics][joints]") {
  REQUIRE(firstPromotion(false) >= 0);
  CHECK(firstPromotion(true) < 0);
}

TEST_CASE("a sleeping body wakes and follows the mass-0 body it hangs from", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const double mass = 3.0;
  const glm::dvec3 velocity(1.0, 0.0, 0.0);
  World world;
  HandGrip grip;
  grip.hand = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 0.0, gripPoint + handReach, glm::dvec3(0.0), 0.5, 3u);
  grip.box = addRigidBox(world.bodies, crate, glm::dvec3(0.05), mass, gripPoint, glm::dvec3(0.0), 0.5, 3u);
  JointDef def = worldSpring(world.bodies.rigids[grip.box].id, glm::dvec3(0.0));
  def.bodyA = world.bodies.rigids[grip.hand].id;
  def.frameA.position = -handReach;
  def.spring.maxForce = jointInfinity;
  grip.joint = addJoint(world, def);
  REQUIRE(grip.joint != 0u);
  REQUIRE(sleepsWithin(world, 300u));

  const double dt = world.parameters.dt;
  const double bound = glm::length(velocity) * std::sqrt(mass / def.spring.linearStiffness);
  double furthest = 0.0;
  double late = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    moveHand(world, grip, gripPoint + handReach + dt * static_cast<double>(frame) * velocity, velocity);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (frame == 0u) {
      REQUIRE_FALSE(world.bodies.rigids[grip.box].asleep);
      REQUIRE(world.bodies.rigids[grip.box].velocity.x > 0.0);
    }

    const double lag = std::abs(gripSlack(world, grip).x);

    furthest = std::max(furthest, lag);

    if (frame >= 60u) {
      late = std::max(late, lag);
    }
  }

  const double weight = mass * std::abs(world.parameters.gravity.y);

  CAPTURE(furthest, late, bound);
  CHECK(furthest <= bound);
  CHECK(late < 1.0e-3);
  CHECK(std::abs(glm::length(jointReport(world, grip.joint)->force) - weight) <= 0.01 * weight);
}

TEST_CASE("a body held by friction stays awake while the mass-0 body it is sprung to moves", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const glm::dvec3 lift(0.0, 1.0, 0.0);
  World world;
  addFloor(world);
  const uint32_t box = findBody(world.bodies, addResting(world, crate, 0.0))->index;
  const uint32_t hand =
    addRigidBox(world.bodies, crate, glm::dvec3(0.05), 0.0, world.bodies.rigids[box].position + lift, glm::dvec3(0.2, 0.0, 0.0), 0.5, 3u);
  JointDef def = between(world.bodies.rigids[hand].id, world.bodies.rigids[box].id);
  def.kind = JointKind::Spring;
  def.frameA.position = -lift;
  def.spring.linearStiffness = 20.0;
  def.spring.linearDamping = 5.0;
  REQUIRE(addJoint(world, def) != 0u);

  const double start = world.bodies.rigids[box].position.x;

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  CAPTURE(world.bodies.rigids[box].position.x - start);
  CHECK_FALSE(world.bodies.rigids[box].asleep);
  CHECK(world.bodies.rigids[box].position.x - start > 0.1);
}

TEST_CASE("a joint rides a moving mass-0 body in a strainstep island", "[dynamics][joints]") {
  const talus::field::SampledSdf crate = crateField();
  const glm::dvec3 trayHalf(0.15, 0.025, 0.15);
  const talus::field::SampledSdf trayField = boxOf(trayHalf, 0.025);
  const SoftBlock load = softBlock(glm::dvec3(0.05), 0.05);
  const glm::dvec3 velocity(1.0, 0.0, 0.0);
  World world;
  world.parameters.sleep = false;
  HandGrip grip;
  grip.hand = addRigidBox(world.bodies, crate, glm::dvec3(0.05), 0.0, gripPoint + handReach, velocity, 0.5, 3u);
  grip.box = addRigidBox(world.bodies, trayField, trayHalf, 2.0, gripPoint, velocity, 0.5, 3u);
  addSoftBlock(world, load, gripPoint + glm::dvec3(0.0, trayHalf.y + 0.05, 0.0), velocity);
  JointDef def = worldSpring(world.bodies.rigids[grip.box].id, glm::dvec3(0.0));
  def.bodyA = world.bodies.rigids[grip.hand].id;
  def.frameA.position = -handReach;
  def.spring.maxForce = jointInfinity;
  def.spring.angularStiffness = 50.0;
  def.spring.angularDamping = 5.0;
  grip.joint = addJoint(world, def);
  REQUIRE(grip.joint != 0u);

  const double dt = world.parameters.dt;
  double furthest = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    moveHand(world, grip, gripPoint + handReach + dt * static_cast<double>(frame) * velocity, velocity);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    requireStrainstep(world, grip.box);

    if (frame >= 60u) {
      furthest = std::max(furthest, std::abs(gripSlack(world, grip).x));
    }
  }

  double carried = world.bodies.rigids[grip.box].mass;

  for (const float node : world.bodies.lattices[0].mesh->nodeMass) {
    carried += static_cast<double>(node);
  }

  const double sag = carried * std::abs(world.parameters.gravity.y) / def.spring.linearStiffness;
  const glm::dvec3 slack = gripSlack(world, grip);

  CAPTURE(furthest, slack.x, slack.y, sag);
  CHECK(furthest < 1.0e-3);
  CHECK(std::abs(slack.y - sag) <= 0.02 * sag);
}
