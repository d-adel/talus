#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <fixtures/mover.h>

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

namespace {

using namespace talus::dynamics;

talus::field::SampledSdf boxField(const glm::dvec3& half, double voxel) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

const RigidBody& rigidOf(const World& world, uint32_t id) {
  return world.bodies.rigids[findBody(world.bodies, id)->index];
}

uint32_t addBox(
  World& world,
  const talus::field::SampledSdf& field,
  const glm::dvec3& half,
  double mass,
  const glm::dvec3& at,
  const glm::dvec3& velocity = glm::dvec3(0.0),
  double friction = 0.5
) {
  const uint32_t index = addRigidBox(
    world.bodies,
    field,
    half,
    mass,
    at,
    velocity,
    friction,
    3u
  );

  return world.bodies.rigids[index].id;
}

void addGround(World& world) {
  (void)addStatic(
    world.bodies,
    fixture::groundField(),
    0.5,
    0.0,
    glm::dvec3(0.0, -0.5, 0.0)
  );
}

void run(World& world, uint32_t frames) {
  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }
}

JointDef planarJoint(uint32_t a, uint32_t b, const PlanarVelocitySettings& planar) {
  JointDef def;
  def.kind = JointKind::PlanarVelocity;
  def.bodyA = a;
  def.bodyB = b;
  def.collideConnected = true;
  def.planar = planar;

  return def;
}

bool samePlanar(const PlanarVelocitySettings& a, const PlanarVelocitySettings& b) {
  return a.targetVelocity == b.targetVelocity && a.maxForce == b.maxForce;
}

double reachAlongX(const RigidBody& body, const glm::dvec3& half) {
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);

  return std::abs(turn[0].x) * half.x + std::abs(turn[1].x) * half.y
         + std::abs(turn[2].x) * half.z;
}

}

TEST_CASE(
  "a planar velocity joint reaches its target in its plane and never turns the body",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0));

  REQUIRE(setPlanarVelocity(
    world,
    character.planar,
    {glm::dvec3(1.0, 2.0, -3.0), 1000.0}
  ));
  run(world, 60u);

  const RigidBody& body = rigidOf(world, character.body);
  const JointReport report = *jointReport(world, character.planar);

  CHECK(std::abs(body.velocity.x - 1.0) < 1.0e-4);
  CHECK(std::abs(body.velocity.z + 3.0) < 1.0e-4);
  CHECK(body.velocity.y == 0.0);
  CHECK(body.angularVelocity == glm::dvec3(0.0));
  CHECK(body.rotation == glm::dquat(1.0, 0.0, 0.0, 0.0));
  CHECK(report.force.y == 0.0);
}

TEST_CASE(
  "a planar velocity joint's plane is normal to frame B's y in world axes",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  const glm::dquat half =
    glm::angleAxis(0.25 * std::numbers::pi, glm::dvec3(1.0, 0.0, 0.0));
  const uint32_t body = addBox(
    world,
    fixture::characterField(),
    fixture::characterHalf,
    fixture::characterMass,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.0
  );

  REQUIRE(setRigidPose(world, body, glm::dvec3(0.0), half));

  JointDef def =
    planarJoint(noBody, body, {glm::dvec3(1.0, 2.0, -3.0), 1000.0});
  def.frameA.rotation =
    glm::angleAxis(0.5 * std::numbers::pi, glm::dvec3(0.0, 0.0, 1.0));
  def.frameB.rotation = half;

  REQUIRE(addJoint(world, def) != 0u);
  run(world, 60u);

  CHECK(
    glm::length(rigidOf(world, body).velocity - glm::dvec3(1.0, 2.0, 0.0))
    < 1.0e-4
  );
  CHECK(rigidOf(world, body).angularVelocity == glm::dvec3(0.0));
}

TEST_CASE(
  "a planar velocity joint accelerates at exactly its force cap",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0));

  REQUIRE(setPlanarVelocity(
    world,
    character.planar,
    {glm::dvec3(5.0, 0.0, 0.0), 400.0}
  ));
  run(world, 30u);

  JointReport report = *jointReport(world, character.planar);

  CHECK(std::abs(rigidOf(world, character.body).velocity.x - 2.5) < 2.5e-4);
  CHECK(std::abs(report.force.x - 400.0) < 0.04);
  CHECK(report.forceSaturated);

  run(world, 120u);
  report = *jointReport(world, character.planar);

  CHECK(std::abs(rigidOf(world, character.body).velocity.x - 5.0) < 1.0e-4);
  CHECK_FALSE(report.forceSaturated);
  CHECK(glm::length(report.force) < 1.0);
}

TEST_CASE(
  "a planar velocity joint holds a body at a moving mass-0 deck's velocity",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  const glm::dvec3 deckHalf(1.0, 0.1, 1.0);
  const glm::dvec3 deckVelocity(1.0, 0.0, 0.5);
  const talus::field::SampledSdf deckField = boxField(deckHalf, 0.05);
  const uint32_t deck = addBox(
    world,
    deckField,
    deckHalf,
    0.0,
    glm::dvec3(0.0),
    deckVelocity
  );
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, 2.0, 0.0), deck);

  REQUIRE(setPlanarVelocity(world, character.planar, {glm::dvec3(0.0), 2000.0}));
  run(world, 60u);

  CHECK(glm::length(rigidOf(world, character.body).velocity - deckVelocity) < 1.0e-4);
  CHECK_FALSE(fixture::touching(world, character.body, deck));

  REQUIRE(setPlanarVelocity(
    world,
    character.planar,
    {glm::dvec3(0.5, 0.0, 0.0), 2000.0}
  ));
  run(world, 60u);

  CHECK(
    glm::length(
      rigidOf(world, character.body).velocity - glm::dvec3(1.5, 0.0, 0.5)
    )
    < 1.0e-4
  );
}

TEST_CASE(
  "a planar velocity joint presses a wall with its force cap and stays still",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;

  const glm::dvec3 wallHalf(0.1, 2.0, 2.0);
  const talus::field::SampledSdf wallField = boxField(wallHalf, 0.05);
  const double wallFace = 1.0 - wallHalf.x;

  (void)addStatic(world.bodies, wallField, 0.5, 0.0, glm::dvec3(1.0, 0.0, 0.0));

  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0));
  double deepest = -std::numeric_limits<double>::infinity();

  REQUIRE(setPlanarVelocity(
    world,
    character.planar,
    {glm::dvec3(2.0, 0.0, 0.0), 300.0}
  ));

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const RigidBody& body = rigidOf(world, character.body);

    deepest = std::max(
      deepest,
      body.position.x + reachAlongX(body, fixture::characterHalf)
    );
  }

  const RigidBody& body = rigidOf(world, character.body);

  CHECK(glm::length(body.velocity) < 1.0e-3);
  CHECK(
    std::abs(normalForceOn(world, *findBody(world.bodies, character.body)) - 300.0)
    < 3.0
  );
  CHECK(glm::length(body.angularVelocity) < 1.0e-3);
  CHECK(deepest <= wallFace + world.parameters.speculativeDistance);
}

TEST_CASE(
  "a planar velocity joint reaches its target in strainstep",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 crateHalf(0.05);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.01);
  const fixture::SoftBlock slab =
    fixture::softBlock(glm::dvec3(0.2, 0.05, 0.15), 0.05);

  addGround(world);
  fixture::addSoftBlock(world, slab, glm::dvec3(0.0, 0.05, 0.0));

  const uint32_t crate = addBox(
    world,
    crateField,
    crateHalf,
    1.5,
    glm::dvec3(-0.15, 0.15, 0.0),
    glm::dvec3(0.0),
    0.0
  );
  const uint32_t crateIndex = findBody(world.bodies, crate)->index;

  REQUIRE(
    addJoint(
      world,
      planarJoint(noBody, crate, {glm::dvec3(0.5, 0.0, 0.0), 20.0})
    )
    != 0u
  );

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    fixture::requireStrainstep(world, crateIndex);
  }

  CHECK(std::abs(rigidOf(world, crate).velocity.x - 0.5) < 1.0e-3);
}

TEST_CASE(
  "invalid planar velocity joints and edits are rejected",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.gravity = glm::dvec3(0.0);
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0));
  const glm::dvec3 otherHalf(0.1);
  const talus::field::SampledSdf otherField = boxField(otherHalf, 0.05);
  const uint32_t other =
    addBox(world, otherField, otherHalf, 1.0, glm::dvec3(2.0, 0.0, 0.0));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const PlanarVelocitySettings valid{glm::dvec3(1.0, 0.0, 0.0), 100.0};
  const size_t count = world.joints.all().size();

  const auto rejects = [&](const JointDef& def) {
    CHECK(addJoint(world, def) == 0u);
    CHECK(world.joints.all().size() == count);
  };

  JointDef def = planarJoint(noBody, other, valid);
  def.planar.targetVelocity.x = nan;
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.planar.targetVelocity.x = inf;
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.planar.maxForce = -1.0;
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.planar.maxForce = nan;
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.collideConnected = false;
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.kind = static_cast<JointKind>(6);
  rejects(def);

  def = planarJoint(noBody, other, valid);
  def.planar.maxForce = jointInfinity;
  CHECK(addJoint(world, def) != 0u);

  JointDef hingeDef;
  hingeDef.kind = JointKind::Hinge;
  hingeDef.bodyB = other;
  const uint32_t hinge = addJoint(world, hingeDef);
  REQUIRE(hinge != 0u);

  const PlanarVelocitySettings before =
    jointDef(world, character.planar)->planar;
  const PlanarVelocitySettings hingeBefore = jointDef(world, hinge)->planar;

  CHECK_FALSE(setPlanarVelocity(world, 999u, valid));
  CHECK_FALSE(setPlanarVelocity(world, hinge, valid));
  CHECK(samePlanar(jointDef(world, hinge)->planar, hingeBefore));

  for (const PlanarVelocitySettings& bad :
       {PlanarVelocitySettings{glm::dvec3(nan, 0.0, 0.0), 100.0},
        PlanarVelocitySettings{glm::dvec3(0.0, 0.0, inf), 100.0},
        PlanarVelocitySettings{glm::dvec3(0.0), -1.0},
        PlanarVelocitySettings{glm::dvec3(0.0), nan}}) {
    CHECK_FALSE(setPlanarVelocity(world, character.planar, bad));
    CHECK(samePlanar(jointDef(world, character.planar)->planar, before));
  }

  CHECK(setPlanarVelocity(world, character.planar, valid));
  CHECK(samePlanar(jointDef(world, character.planar)->planar, valid));
}

TEST_CASE(
  "editing a planar velocity joint wakes both bodies and keeps its id",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 crateHalf(0.05);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.01);

  addGround(world);

  const uint32_t crate =
    addBox(world, crateField, crateHalf, 1.5, glm::dvec3(0.0, 0.05, 0.0));
  const uint32_t base =
    addBox(world, crateField, crateHalf, 0.0, glm::dvec3(1.0, 0.5, 0.0));
  const uint32_t joint = addJoint(
    world,
    planarJoint(base, crate, {glm::dvec3(0.0), 50.0})
  );

  REQUIRE(joint != 0u);

  for (uint32_t frame = 0u;
       frame != 300u && !(rigidOf(world, crate).asleep && rigidOf(world, base).asleep);
       ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(rigidOf(world, crate).asleep);
  REQUIRE(rigidOf(world, base).asleep);

  const size_t count = world.joints.all().size();

  CHECK(setPlanarVelocity(world, joint, {glm::dvec3(0.5, 0.0, 0.0), 50.0}));
  CHECK_FALSE(rigidOf(world, crate).asleep);
  CHECK_FALSE(rigidOf(world, base).asleep);
  CHECK(world.joints.all().size() == count);
  CHECK(world.joints.all().front().id == joint);
  CHECK(jointDef(world, joint)->bodyB == crate);
}
