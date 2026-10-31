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
#include <array>
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

constexpr glm::dvec3 up{0.0, 1.0, 0.0};

JointDef pogoJoint(
  uint32_t a,
  uint32_t b,
  const glm::dvec3& ground,
  const PogoSettings& pogo
) {
  JointDef def;
  def.kind = JointKind::Pogo;
  def.bodyA = a;
  def.bodyB = b;
  def.frameA.position = ground;
  def.frameB.position = fixture::footAnchor;
  def.collideConnected = true;
  def.pogo = pogo;

  return def;
}

glm::dvec3 anchorOf(const World& world, uint32_t body) {
  const RigidBody& rigid = rigidOf(world, body);

  return rigid.position + rigid.rotation * fixture::footAnchor;
}

double centreAbove(double ground, double stretch) {
  return ground + fixture::pogoRest + stretch - fixture::footAnchor.y;
}

fixture::Character addFloating(
  World& world,
  double stretch,
  const PogoSettings& pogo
) {
  fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, stretch), 0.0));

  character.pogo = addJoint(
    world,
    pogoJoint(noBody, character.body, glm::dvec3(0.0), pogo)
  );
  REQUIRE(character.pogo != 0u);

  return character;
}

double worstSpringError(
  World& world,
  uint32_t body,
  double displacement,
  bool strainstep
) {
  const uint32_t index = findBody(world.bodies, body)->index;
  double worst = 0.0;

  for (uint32_t frame = 1u; frame <= 60u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (strainstep) {
      fixture::requireStrainstep(world, index);
    }

    const double expected = fixture::springResponse(
      fixture::pogoHertz,
      fixture::pogoDamping,
      displacement,
      static_cast<double>(frame) * world.parameters.dt
    );

    worst = std::max(
      worst,
      std::abs(anchorOf(world, body).y - fixture::pogoRest - expected)
    );
  }

  return worst;
}

bool bothAsleep(const World& world, const std::array<uint32_t, 2>& bodies) {
  return rigidOf(world, bodies[0]).asleep && rigidOf(world, bodies[1]).asleep;
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

TEST_CASE(
  "a pogo floats a body at its rest length and carries exactly its weight",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.sleep = false;
  const fixture::Character character =
    addFloating(world, 0.2, fixture::pogoFor(world, up));
  const double weight = fixture::weightOf(world);
  double height = anchorOf(world, character.body).y;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 300u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const double next = anchorOf(world, character.body).y;

    if (frame > 240u) {
      drift = std::max(drift, std::abs(next - height));
    }

    height = next;
  }

  const JointReport report = *jointReport(world, character.pogo);

  CHECK(std::abs(height - fixture::pogoRest) < 1.0e-4);
  CHECK(std::abs(report.pogoForce - weight) < 1.0e-3 * weight);
  CHECK(report.force == glm::dvec3(0.0, report.pogoForce, 0.0));
  CHECK(drift < 1.0e-5);
  CHECK_FALSE(report.forceSaturated);
}

TEST_CASE(
  "a pogo moves a body along its implicit spring",
  "[dynamics][joints][mover]"
) {
  World capped;
  World free;

  capped.parameters.sleep = false;
  free.parameters.sleep = false;

  PogoSettings unbounded = fixture::pogoFor(free, up);

  unbounded.maxTension = jointInfinity;
  unbounded.maxCompression = jointInfinity;

  const fixture::Character character =
    addFloating(capped, 0.2, fixture::pogoFor(capped, up));
  const fixture::Character twin = addFloating(free, 0.2, unbounded);
  double worst = 0.0;
  bool same = true;
  bool saturated = false;

  for (uint32_t frame = 1u; frame <= 60u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(capped).nonFinite);
    REQUIRE_FALSE(talus::dynamics::step(free).nonFinite);

    const double expected = fixture::springResponse(
      fixture::pogoHertz,
      fixture::pogoDamping,
      0.2,
      static_cast<double>(frame) * capped.parameters.dt
    );

    worst = std::max(
      worst,
      std::abs(anchorOf(capped, character.body).y - fixture::pogoRest - expected)
    );
    same = same
           && rigidOf(capped, character.body).position
                == rigidOf(free, twin.body).position;
    saturated = saturated || jointReport(capped, character.pogo)->forceSaturated;
  }

  CHECK(worst <= 0.02 * 0.2);
  CHECK(same);
  CHECK_FALSE(saturated);
}

TEST_CASE(
  "a pogo moves a body along the same spring in strainstep",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.sleep = false;
  const double release = 0.02;
  const fixture::Character character =
    addFloating(world, release, fixture::pogoFor(world, up));
  const fixture::SoftBlock block =
    fixture::softBlock(glm::dvec3(0.25, 0.05, 0.25), 0.05);
  const glm::dvec3 top = rigidOf(world, character.body).position
                         + glm::dvec3(0.0, fixture::characterHalf.y, 0.0);

  fixture::addSoftBlock(world, block, top + glm::dvec3(0.0, 0.05, 0.0));

  CHECK(
    worstSpringError(world, character.body, release, true) <= 0.02 * release
  );
}

TEST_CASE(
  "a pogo carries a body up a 0.15 m step with no upward speed spike",
  "[dynamics][joints][mover]"
) {
  World world;
  const double rise = 0.15;

  (void)addStatic(world.bodies, fixture::stepField(), 0.5);

  fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(-0.8, 1.4, 0.0));
  glm::dvec3 anchor = anchorOf(world, character.body);
  double highest = anchor.y;
  double fastest = 0.0;
  double sideways = 0.0;
  bool touched = false;

  for (uint32_t frame = 1u; frame <= 210u; ++frame) {
    fixture::MoverStep move;
    move.velocity = frame <= 90u ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0);

    (void)fixture::moveCharacter(world, character, move);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const glm::dvec3 next = anchorOf(world, character.body);
    const glm::dvec3 rate = (next - anchor) / world.parameters.dt;

    fastest = std::max(fastest, rate.y);
    highest = std::max(highest, next.y);
    anchor = next;

    if (frame >= 5u && frame <= 88u) {
      sideways = std::max(sideways, std::abs(rate.x - 1.0));
    }

    touched = touched
              || !contactsOf(world, *findBody(world.bodies, character.body)).empty();
  }

  CHECK(std::abs(anchor.y - (fixture::pogoRest + rise)) < 1.0e-3);
  CHECK(
    fastest
    <= fixture::springPeakSpeed(fixture::pogoHertz, fixture::pogoDamping, rise)
  );
  CHECK(
    highest <= fixture::pogoRest + rise
                 + fixture::springOvershoot(fixture::pogoDamping) * rise + 1.0e-3
  );
  CHECK(sideways <= 0.1);
  CHECK_FALSE(touched);
}

TEST_CASE(
  "a pogo's compression cap lets a heavy body down onto its ground body",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);

  addGround(world);

  const uint32_t crate = addBox(
    world,
    crateField,
    crateHalf,
    20.0,
    glm::dvec3(0.0, crateHalf.y, 0.0)
  );
  const double weight = fixture::weightOf(world);
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
  );
  PogoSettings pogo = fixture::pogoFor(world, up);

  pogo.maxCompression = 0.5 * weight;
  character.pogo = addJoint(
    world,
    pogoJoint(crate, character.body, glm::dvec3(0.0, crateHalf.y, 0.0), pogo)
  );
  REQUIRE(character.pogo != 0u);
  run(world, 180u);

  const JointReport report = *jointReport(world, character.pogo);

  CHECK(fixture::touching(world, character.body, crate));
  CHECK(report.forceSaturated);
  CHECK(std::abs(report.pogoForce - 0.5 * weight) < 1.0e-3 * weight);
  CHECK(
    std::abs(
      normalForceOn(world, *findBody(world.bodies, character.body)) - 0.5 * weight
    )
    < 0.01 * weight
  );
}

TEST_CASE(
  "a pogo's tension cap lets a body leave the ground",
  "[dynamics][joints][mover]"
) {
  World world;
  const double weight = fixture::weightOf(world);
  PogoSettings pogo = fixture::pogoFor(world, up);

  pogo.maxTension = 0.5 * weight;

  const fixture::Character character = addFloating(world, 0.0, pogo);
  const double launch = 3.0;
  const double expected =
    launch * launch / (2.0 * 1.5 * glm::length(world.parameters.gravity));
  double rise = 0.0;
  double pull = 0.0;
  bool saturated = false;

  for (uint32_t frame = 1u; frame <= 180u; ++frame) {
    if (frame == 61u) {
      REQUIRE(setVelocity(world, character.body, glm::dvec3(0.0, launch, 0.0)));
    }

    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const JointReport report = *jointReport(world, character.pogo);

    rise = std::max(rise, anchorOf(world, character.body).y - fixture::pogoRest);
    pull = std::min(pull, report.pogoForce);

    if (frame == 61u) {
      saturated = report.forceSaturated;
    }
  }

  CHECK(std::abs(rise - expected) <= 0.01 * expected);
  CHECK(pull >= -0.5 * weight * (1.0 + 1.0e-6));
  CHECK(saturated);
}

TEST_CASE(
  "a pogo re-created every step from its read-back moves exactly like one kept",
  "[dynamics][joints][mover]"
) {
  const auto build = [](World& world) {
    addGround(world);

    fixture::Character character =
      fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, 0.1), 0.0));

    REQUIRE(setPlanarVelocity(
      world,
      character.planar,
      {glm::dvec3(0.5, 0.0, 0.0), fixture::groundForce}
    ));
    character.pogo = addJoint(
      world,
      pogoJoint(noBody, character.body, glm::dvec3(0.0), fixture::pogoFor(world, up))
    );
    REQUIRE(character.pogo != 0u);

    return character;
  };
  World kept;
  World remade;
  const fixture::Character keptCharacter = build(kept);
  fixture::Character remadeCharacter = build(remade);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    INFO("frame " << frame);

    const JointReport old = *jointReport(remade, remadeCharacter.pogo);
    JointDef def = *jointDef(remade, remadeCharacter.pogo);

    def.pogo.force = old.pogoForce;
    def.pogo.speed = old.pogoSpeed;
    REQUIRE(removeJoint(remade, remadeCharacter.pogo));
    remadeCharacter.pogo = addJoint(remade, def);
    REQUIRE(remadeCharacter.pogo != 0u);

    const JointReport fresh = *jointReport(remade, remadeCharacter.pogo);
    const JointReport same = *jointReport(kept, keptCharacter.pogo);
    const uint32_t keptIsland = kept.islandStore.islandOf(keptCharacter.body);
    const uint32_t remadeIsland =
      remade.islandStore.islandOf(remadeCharacter.body);

    REQUIRE(fresh.pogoForce == old.pogoForce);
    REQUIRE(fresh.pogoSpeed == old.pogoSpeed);
    REQUIRE(fresh.pogoForce == same.pogoForce);
    REQUIRE(fresh.pogoSpeed == same.pogoSpeed);
    REQUIRE(remade.joints.all().size() == kept.joints.all().size());
    REQUIRE(remade.islandStore.pendingSplit() == kept.islandStore.pendingSplit());
    REQUIRE(remade.islandStore.splits() == kept.islandStore.splits());
    REQUIRE(remadeIsland == keptIsland);

    if (keptIsland != noIsland) {
      REQUIRE(
        remade.islandStore.removed(remadeIsland)
        == kept.islandStore.removed(keptIsland)
      );
    }

    REQUIRE_FALSE(talus::dynamics::step(kept).nonFinite);
    REQUIRE_FALSE(talus::dynamics::step(remade).nonFinite);

    const RigidBody& a = rigidOf(kept, keptCharacter.body);
    const RigidBody& b = rigidOf(remade, remadeCharacter.body);

    REQUIRE(b.position == a.position);
    REQUIRE(b.velocity == a.velocity);
    REQUIRE(b.rotation == a.rotation);
    REQUIRE(b.angularVelocity == a.angularVelocity);
  }
}

TEST_CASE(
  "a pogo re-created on a different body every step holds steady",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);

  addGround(world);

  const std::array<uint32_t, 2> crates{
    addBox(world, crateField, crateHalf, 20.0, glm::dvec3(-crateHalf.x, crateHalf.y, 0.0)),
    addBox(world, crateField, crateHalf, 20.0, glm::dvec3(crateHalf.x, crateHalf.y, 0.0))
  };
  const std::array<glm::dvec3, 2> seams{
    glm::dvec3(crateHalf.x, crateHalf.y, 0.0),
    glm::dvec3(-crateHalf.x, crateHalf.y, 0.0)
  };
  const double weight = fixture::weightOf(world);
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
  );
  size_t count = 0u;
  bool steadyCount = true;
  double sag = 0.0;
  double wobble = 0.0;
  double crateSpeed = 0.0;

  for (uint32_t frame = 0u; frame != 240u; ++frame) {
    const uint32_t side = frame % 2u;
    PogoSettings pogo = fixture::pogoFor(world, up);

    if (character.pogo != 0u) {
      const JointReport report = *jointReport(world, character.pogo);

      pogo.force = report.pogoForce;
      pogo.speed = report.pogoSpeed;
      REQUIRE(removeJoint(world, character.pogo));
    }

    character.pogo = addJoint(
      world,
      pogoJoint(crates[side], character.body, seams[side], pogo)
    );
    REQUIRE(character.pogo != 0u);
    character.ground = crates[side];
    character.onGround = true;

    if (frame == 60u) {
      count = world.joints.all().size();
    }

    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (frame < 60u) {
      continue;
    }

    const RigidBody& ground = rigidOf(world, crates[side]);
    const double loaded = (ground.position + ground.rotation * seams[side]).y;

    sag = std::max(
      sag,
      std::abs(anchorOf(world, character.body).y - loaded - fixture::pogoRest)
    );
    wobble = std::max(
      wobble,
      std::abs(jointReport(world, character.pogo)->pogoForce - weight)
    );

    for (const uint32_t crate : crates) {
      crateSpeed = std::max(crateSpeed, glm::length(rigidOf(world, crate).velocity));
    }

    steadyCount = steadyCount && world.joints.all().size() == count;
  }

  CHECK(sag < 2.0e-3);
  CHECK(wobble < 0.05 * weight);
  CHECK(crateSpeed < 0.05);
  CHECK(steadyCount);

  std::optional<uint32_t> left;
  std::optional<uint32_t> slept;

  for (uint32_t frame = 0u; frame != 300u && !slept; ++frame) {
    fixture::MoverStep move;

    if (anchorOf(world, character.body).x < 1.5) {
      move.velocity = glm::dvec3(1.0, 0.0, 0.0);
    }

    REQUIRE(fixture::moveCharacter(world, character, move).has_value());

    if (!left && character.ground == noBody) {
      left = frame;
    }

    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (left && bothAsleep(world, crates)) {
      slept = frame;
    }
  }

  REQUIRE(left.has_value());
  REQUIRE(slept.has_value());
  CHECK(*slept - *left <= 90u);
}

TEST_CASE(
  "a pogo on steep ground holds still with the mover on and slides without launching with it off",
  "[dynamics][joints][mover]"
) {
  World world;
  const double slope = std::numbers::pi / 3.0;
  const glm::dvec3 slopeHalf(3.0, 0.2, 1.0);
  const talus::field::SampledSdf slopeField = boxField(slopeHalf, 0.05);
  const glm::dquat turn = glm::angleAxis(slope, glm::dvec3(0.0, 0.0, 1.0));
  const glm::dvec3 normal = turn * up;
  const double gravity = glm::length(world.parameters.gravity);
  const double weight = fixture::weightOf(world);
  const auto surfaceAt = [&](double x) {
    return (slopeHalf.y - normal.x * x) / normal.y;
  };

  (void)addStatic(world.bodies, slopeField, 0.5, 0.0, glm::dvec3(0.0), turn);

  const glm::dvec3 foot = turn * glm::dvec3(2.0, slopeHalf.y, 0.0);
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(foot.x, centreAbove(foot.y, 0.0), foot.z)
  );

  character.onGround = true;

  const glm::dvec3 start = rigidOf(world, character.body).position;
  double moved = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(fixture::moveCharacter(world, character, {}).has_value());
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    moved = std::max(
      moved,
      glm::length(rigidOf(world, character.body).position - start)
    );
  }

  const JointReport held = *jointReport(world, character.pogo);
  const double planarForce =
    glm::length(jointReport(world, character.planar)->force);
  const double support = weight / normal.y;
  const double sideways = weight * std::tan(slope);

  CHECK(moved < 1.0e-3);
  CHECK(std::abs(held.pogoForce - support) < 0.01 * support);
  CHECK(std::abs(planarForce - sideways) < 0.01 * sideways);

  const PogoSettings caps = fixture::pogoFor(world, normal);
  const double top = rigidOf(world, character.body).position.y;
  glm::dvec3 previous = rigidOf(world, character.body).position;
  double excess = -std::numeric_limits<double>::infinity();
  double gap = 0.0;
  bool finite = true;
  bool capped = true;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    fixture::MoverStep slide;
    slide.maxForce = 0.0;

    REQUIRE(fixture::moveCharacter(world, character, slide).has_value());
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const RigidBody& body = rigidOf(world, character.body);
    const glm::dvec3 anchor = anchorOf(world, character.body);
    const double force = jointReport(world, character.pogo)->pogoForce;
    const double speed =
      glm::length(body.position - previous) / world.parameters.dt;

    excess = std::max(
      excess,
      speed - std::sqrt(2.0 * gravity * std::abs(body.position.y - top)) - 0.05
    );
    gap = std::max(
      gap,
      std::abs(anchor.y - surfaceAt(anchor.x) - fixture::pogoRest)
    );
    finite = finite && std::isfinite(body.position.x)
             && std::isfinite(body.position.y) && std::isfinite(body.position.z)
             && std::isfinite(force);
    capped = capped && force >= -caps.maxTension * (1.0 + 1.0e-6)
             && force <= caps.maxCompression * (1.0 + 1.0e-6);
    previous = body.position;
  }

  CHECK(excess <= 0.0);
  CHECK(gap < 0.05);
  CHECK(finite);
  CHECK(capped);
}

TEST_CASE(
  "a pogo rides a rising mass-0 platform at its rest length",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 platformHalf(1.0, 0.1, 1.0);
  const talus::field::SampledSdf platformField = boxField(platformHalf, 0.05);
  const glm::dvec3 lift(0.0, 0.5, 0.0);
  const uint32_t platform =
    addBox(world, platformField, platformHalf, 0.0, glm::dvec3(0.0), lift);
  const double weight = fixture::weightOf(world);
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(platformHalf.y, 0.0), 0.0)
  );

  character.pogo = addJoint(
    world,
    pogoJoint(
      platform,
      character.body,
      glm::dvec3(0.0, platformHalf.y, 0.0),
      fixture::pogoFor(world, up)
    )
  );
  REQUIRE(character.pogo != 0u);
  run(world, 120u);

  const double top = rigidOf(world, platform).position.y + platformHalf.y;

  CHECK(
    std::abs(anchorOf(world, character.body).y - (top + fixture::pogoRest))
    < 1.0e-3
  );
  CHECK(std::abs(rigidOf(world, character.body).velocity.y - lift.y) < 1.0e-3);
  CHECK(
    std::abs(jointReport(world, character.pogo)->pogoForce - weight)
    < 0.01 * weight
  );
}

TEST_CASE(
  "invalid pogo joints are rejected and a pogo's normal is stored unit length",
  "[dynamics][joints][mover]"
) {
  World world;
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const JointDef valid = pogoJoint(
    noBody,
    character.body,
    glm::dvec3(0.0),
    fixture::pogoFor(world, up)
  );
  const size_t count = world.joints.all().size();

  const auto rejects = [&](const JointDef& def) {
    CHECK(addJoint(world, def) == 0u);
    CHECK(world.joints.all().size() == count);
  };

  JointDef def = valid;
  def.pogo.normal = glm::dvec3(0.0);
  rejects(def);

  def = valid;
  def.pogo.normal.y = nan;
  rejects(def);

  def = valid;
  def.pogo.restLength = -0.1;
  rejects(def);

  def = valid;
  def.pogo.hertz = -1.0;
  rejects(def);

  def = valid;
  def.pogo.hertz = nan;
  rejects(def);

  def = valid;
  def.pogo.dampingRatio = -1.0;
  rejects(def);

  def = valid;
  def.pogo.maxTension = -1.0;
  rejects(def);

  def = valid;
  def.pogo.maxTension = nan;
  rejects(def);

  def = valid;
  def.pogo.maxCompression = -1.0;
  rejects(def);

  for (const double bad : {nan, inf}) {
    def = valid;
    def.pogo.force = bad;
    rejects(def);

    def = valid;
    def.pogo.speed = bad;
    rejects(def);
  }

  def = valid;
  def.collideConnected = false;
  rejects(def);

  def = valid;
  def.pogo.normal = glm::dvec3(0.0, 2.0, 0.0);
  def.pogo.maxTension = jointInfinity;
  def.pogo.maxCompression = jointInfinity;
  def.pogo.force = 412.5;
  def.pogo.speed = -0.25;

  const uint32_t joint = addJoint(world, def);

  REQUIRE(joint != 0u);
  CHECK(jointDef(world, joint)->pogo.normal == up);

  const JointReport report = *jointReport(world, joint);

  CHECK(report.pogoForce == def.pogo.force);
  CHECK(report.pogoSpeed == def.pogo.speed);
}

TEST_CASE(
  "a pogo with a low cap still follows its spring, accelerating at the cap",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.sleep = false;
  const double weight = fixture::weightOf(world);
  const double gravity = glm::length(world.parameters.gravity);
  const double h =
    world.parameters.dt / static_cast<double>(world.parameters.jointSubsteps);
  PogoSettings pogo = fixture::pogoFor(world, up);

  pogo.maxTension = 1.5 * weight;
  pogo.maxCompression = 1.5 * weight;

  const fixture::Character character = addFloating(world, -0.2, pogo);
  const double lift = pogo.maxCompression / fixture::characterMass - gravity;
  const double start = anchorOf(world, character.body).y;
  double drift = 0.0;
  bool saturated = true;
  bool carried = true;

  for (uint32_t frame = 1u; frame <= 9u; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const double time = static_cast<double>(frame) * world.parameters.dt;
    const double risen = anchorOf(world, character.body).y - start;
    const JointReport report = *jointReport(world, character.pogo);

    drift = std::max(
      drift,
      std::abs(risen - 0.5 * lift * time * time) / (lift * h * time)
    );
    saturated = saturated && report.forceSaturated;
    carried = carried && std::abs(report.pogoForce - weight) < 1.0e-3 * weight;
  }

  CHECK(drift <= 1.0);
  CHECK(saturated);
  CHECK(carried);

  run(world, 171u);

  const JointReport report = *jointReport(world, character.pogo);

  CHECK(std::abs(anchorOf(world, character.body).y - fixture::pogoRest) < 1.0e-4);
  CHECK(std::abs(report.pogoForce - weight) < 1.0e-3 * weight);
  CHECK_FALSE(report.forceSaturated);
}

TEST_CASE(
  "a pogo refuses ground too steep for its axis and lets go when its body turns past the guard",
  "[dynamics][joints][mover]"
) {
  const double guard = std::acos(pogoSteepestCosine);
  const auto tilted = [](double angle) {
    return glm::dvec3(-std::sin(angle), std::cos(angle), 0.0);
  };
  const auto turn = [](double angle) {
    return glm::angleAxis(angle, glm::dvec3(0.0, 0.0, 1.0));
  };
  const auto addBody = [](World& world) {
    return addBox(
      world,
      fixture::characterField(),
      fixture::characterHalf,
      fixture::characterMass,
      glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0),
      glm::dvec3(0.0),
      0.0
    );
  };

  {
    World world;
    const uint32_t body = addBody(world);

    CHECK(
      addJoint(
        world,
        pogoJoint(noBody, body, glm::dvec3(0.0), fixture::pogoFor(world, tilted(guard + 0.01)))
      )
      == 0u
    );
    CHECK(
      addJoint(
        world,
        pogoJoint(noBody, body, glm::dvec3(0.0), fixture::pogoFor(world, tilted(guard - 0.01)))
      )
      != 0u
    );

    JointDef framed =
      pogoJoint(noBody, body, glm::dvec3(0.0), fixture::pogoFor(world, up));
    framed.frameB.rotation = turn(guard + 0.01);

    CHECK(addJoint(world, framed) == 0u);
    REQUIRE(setRigidPose(
      world,
      body,
      rigidOf(world, body).position,
      turn(guard + 0.01)
    ));
    CHECK(
      addJoint(
        world,
        pogoJoint(noBody, body, glm::dvec3(0.0), fixture::pogoFor(world, up))
      )
      == 0u
    );
  }

  for (const double angle :
       {guard - 0.05, guard + 0.05, 0.5 * std::numbers::pi, std::numbers::pi}) {
    INFO("turned by " << angle << " rad");

    World world;
    world.parameters.sleep = false;
    const uint32_t body = addBody(world);
    const PogoSettings pogo = fixture::pogoFor(world, up);
    const uint32_t joint =
      addJoint(world, pogoJoint(noBody, body, glm::dvec3(0.0), pogo));

    REQUIRE(joint != 0u);
    run(world, 30u);
    REQUIRE(setRigidPose(world, body, rigidOf(world, body).position, turn(angle)));
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const JointReport report = *jointReport(world, joint);
    const RigidBody& rigid = rigidOf(world, body);

    CHECK(std::isfinite(report.pogoForce));
    CHECK(std::isfinite(report.pogoSpeed));
    CHECK(std::isfinite(rigid.position.y));

    if (angle < guard) {
      CHECK(report.pogoForce >= -pogo.maxTension * (1.0 + 1.0e-6));
      CHECK(report.pogoForce <= pogo.maxCompression * (1.0 + 1.0e-6));
    } else {
      CHECK(report.pogoForce == 0.0);
      CHECK(report.pogoSpeed == 0.0);
      CHECK_FALSE(report.forceSaturated);
      CHECK(
        std::abs(
          rigid.velocity.y
          + glm::length(world.parameters.gravity) * world.parameters.dt
        )
        < 1.0e-5
      );
    }
  }
}
