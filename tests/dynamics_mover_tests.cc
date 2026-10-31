#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <fixtures/mover.h>

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <numbers>
#include <optional>
#include <tuple>
#include <utility>
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
  const uint32_t index =
    addRigidBox(world.bodies, field, half, mass, at, velocity, friction, 3u);

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

JointDef planarJoint(
  uint32_t a,
  uint32_t b,
  const PlanarVelocitySettings& planar
) {
  JointDef def;
  def.kind = JointKind::PlanarVelocity;
  def.bodyA = a;
  def.bodyB = b;
  def.collideConnected = true;
  def.planar = planar;

  return def;
}

bool samePlanar(
  const PlanarVelocitySettings& a,
  const PlanarVelocitySettings& b
) {
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
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(0.0, stretch), 0.0)
  );

  character.pogo =
    addJoint(world, pogoJoint(noBody, character.body, glm::dvec3(0.0), pogo));
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

double tiltOf(const RigidBody& body) {
  const glm::dvec3 axis = body.rotation * up;

  return std::atan2(glm::length(glm::cross(axis, up)), glm::dot(axis, up));
}

}

TEST_CASE(
  "a planar velocity joint reaches its target in its plane and never turns the "
  "body",
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
  const uint32_t deck =
    addBox(world, deckField, deckHalf, 0.0, glm::dvec3(0.0), deckVelocity);
  const fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, 2.0, 0.0), deck);

  REQUIRE(
    setPlanarVelocity(world, character.planar, {glm::dvec3(0.0), 2000.0})
  );
  run(world, 60u);

  CHECK(
    glm::length(rigidOf(world, character.body).velocity - deckVelocity) < 1.0e-4
  );
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
    std::abs(
      normalForceOn(world, *findBody(world.bodies, character.body)) - 300.0
    )
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
  const uint32_t joint =
    addJoint(world, planarJoint(base, crate, {glm::dvec3(0.0), 50.0}));

  REQUIRE(joint != 0u);

  for (uint32_t frame = 0u;
       frame != 300u
       && !(rigidOf(world, crate).asleep && rigidOf(world, base).asleep);
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
      std::abs(
        anchorOf(capped, character.body).y - fixture::pogoRest - expected
      )
    );
    same = same
           && rigidOf(capped, character.body).position
                == rigidOf(free, twin.body).position;
    saturated =
      saturated || jointReport(capped, character.pogo)->forceSaturated;
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

    touched =
      touched
      || !contactsOf(world, *findBody(world.bodies, character.body)).empty();
  }

  CHECK(std::abs(anchor.y - (fixture::pogoRest + rise)) < 1.0e-3);
  CHECK(
    fastest
    <= fixture::springPeakSpeed(fixture::pogoHertz, fixture::pogoDamping, rise)
  );
  CHECK(
    highest <= fixture::pogoRest + rise
                 + fixture::springOvershoot(fixture::pogoDamping) * rise
                 + 1.0e-3
  );
  CHECK(sideways <= 0.1);
  CHECK_FALSE(touched);
}

TEST_CASE(
  "a pogo's compression cap lets a heavy body down onto its ground body",
  "[dynamics][joints][mover]"
) {
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);

  for (const bool capped : {true, false}) {
    INFO((capped ? "compression cap 0.5 W" : "the host's caps"));

    World world;
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

    if (capped) {
      pogo.maxCompression = 0.5 * weight;
    }

    character.pogo = addJoint(
      world,
      pogoJoint(crate, character.body, glm::dvec3(0.0, crateHalf.y, 0.0), pogo)
    );
    REQUIRE(character.pogo != 0u);

    std::optional<uint32_t> slept;
    double drift = 0.0;

    for (uint32_t frame = 1u; frame <= 180u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      const RigidBody& body = rigidOf(world, character.body);

      if (!slept && body.asleep && rigidOf(world, crate).asleep) {
        slept = frame;
      }

      if (frame >= 60u) {
        drift = std::max(drift, std::abs(body.velocity.y));
      }
    }

    if (capped) {
      const JointReport report = *jointReport(world, character.pogo);

      CHECK(fixture::touching(world, character.body, crate));
      CHECK(report.forceSaturated);
      CHECK(std::abs(report.pogoForce - 0.5 * weight) < 1.0e-3 * weight);
      CHECK(
        std::abs(
          normalForceOn(world, *findBody(world.bodies, character.body))
          - 0.5 * weight
        )
        < 0.01 * weight
      );
    }

    CHECK(drift < 1.0e-3);
    REQUIRE(slept.has_value());
    CHECK(*slept <= 90u);
  }
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

    rise =
      std::max(rise, anchorOf(world, character.body).y - fixture::pogoRest);
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
      pogoJoint(
        noBody,
        character.body,
        glm::dvec3(0.0),
        fixture::pogoFor(world, up)
      )
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
    REQUIRE(
      remade.islandStore.pendingSplit() == kept.islandStore.pendingSplit()
    );
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
    addBox(
      world,
      crateField,
      crateHalf,
      20.0,
      glm::dvec3(-crateHalf.x, crateHalf.y, 0.0)
    ),
    addBox(
      world,
      crateField,
      crateHalf,
      20.0,
      glm::dvec3(crateHalf.x, crateHalf.y, 0.0)
    )
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
      crateSpeed =
        std::max(crateSpeed, glm::length(rigidOf(world, crate).velocity));
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
  "a pogo on steep ground holds still with the mover on and slides without "
  "launching with it off",
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
  "a pogo refuses ground too steep for its axis and lets go when its body "
  "turns past the guard",
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
        pogoJoint(
          noBody,
          body,
          glm::dvec3(0.0),
          fixture::pogoFor(world, tilted(guard + 0.01))
        )
      )
      == 0u
    );
    CHECK(
      addJoint(
        world,
        pogoJoint(
          noBody,
          body,
          glm::dvec3(0.0),
          fixture::pogoFor(world, tilted(guard - 0.01))
        )
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
    REQUIRE(
      setRigidPose(world, body, rigidOf(world, body).position, turn(angle))
    );
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

TEST_CASE(
  "a pogo created with a stored speed comes to rest on its crate and does not "
  "hop",
  "[dynamics][joints][mover]"
) {
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);

  for (const auto& [capped, seed] :
       {std::pair{true, 0.05},
        std::pair{true, 0.5},
        std::pair{false, 0.05},
        std::pair{false, 0.5}}) {
    INFO(
      (capped ? "compression cap 0.5 W" : "the host's caps")
      << ", created with speed " << seed
    );

    World world;
    world.parameters.sleep = false;
    addGround(world);

    const uint32_t crate = addBox(
      world,
      crateField,
      crateHalf,
      20.0,
      glm::dvec3(0.0, crateHalf.y, 0.0)
    );
    fixture::Character character = fixture::addCharacter(
      world,
      glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
    );
    PogoSettings pogo = fixture::pogoFor(world, up);

    if (capped) {
      pogo.maxCompression = 0.5 * fixture::weightOf(world);
    }

    pogo.speed = seed;
    character.pogo = addJoint(
      world,
      pogoJoint(crate, character.body, glm::dvec3(0.0, crateHalf.y, 0.0), pogo)
    );
    REQUIRE(character.pogo != 0u);
    run(world, 300u);

    double lowest = rigidOf(world, character.body).position.y;
    double highest = lowest;
    double drift = 0.0;

    for (uint32_t frame = 0u; frame != 300u; ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      const RigidBody& body = rigidOf(world, character.body);

      lowest = std::min(lowest, body.position.y);
      highest = std::max(highest, body.position.y);
      drift = std::max(drift, std::abs(body.velocity.y));
    }

    CHECK(highest - lowest < 1.0e-4);
    CHECK(drift < 1.0e-3);
  }
}

TEST_CASE(
  "a pogo with both caps 0 lets its body fall freely",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.sleep = false;
  const double gravity = glm::length(world.parameters.gravity);
  const double h =
    world.parameters.dt / static_cast<double>(world.parameters.jointSubsteps);
  const uint32_t body = addBox(
    world,
    fixture::characterField(),
    fixture::characterHalf,
    fixture::characterMass,
    glm::dvec3(0.0, centreAbove(0.0, 0.5), 0.0),
    glm::dvec3(0.0),
    0.0
  );
  PogoSettings pogo = fixture::pogoFor(world, up);

  pogo.maxTension = 0.0;
  pogo.maxCompression = 0.0;
  pogo.speed = 0.5;

  uint32_t joint =
    addJoint(world, pogoJoint(noBody, body, glm::dvec3(0.0), pogo));
  const double start = rigidOf(world, body).position.y;
  double worst = 0.0;
  bool forceless = true;

  REQUIRE(joint != 0u);

  for (uint32_t frame = 1u; frame <= 30u; ++frame) {
    const JointReport old = *jointReport(world, joint);
    JointDef def = *jointDef(world, joint);

    def.pogo.force = old.pogoForce;
    def.pogo.speed = old.pogoSpeed;
    REQUIRE(removeJoint(world, joint));
    joint = addJoint(world, def);
    REQUIRE(joint != 0u);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    const double time = static_cast<double>(frame) * world.parameters.dt;
    const JointReport report = *jointReport(world, joint);

    worst = std::max(
      worst,
      std::abs(
        rigidOf(world, body).position.y
        - (start - 0.5 * gravity * time * (time + h))
      )
    );
    forceless = forceless && report.pogoForce == 0.0;
  }

  CHECK(worst < 1.0e-4);
  CHECK(forceless);
}

TEST_CASE(
  "lowering a pogo's cap mid-motion leaves no velocity once the body rests",
  "[dynamics][joints][mover]"
) {
  World world;
  world.parameters.sleep = false;
  addGround(world);

  const double weight = fixture::weightOf(world);
  fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, -0.2), 0.0));

  character.pogo = addJoint(
    world,
    pogoJoint(
      noBody,
      character.body,
      glm::dvec3(0.0),
      fixture::pogoFor(world, up)
    )
  );
  REQUIRE(character.pogo != 0u);

  double moving = 0.0;
  double lowest = jointInfinity;
  double highest = -jointInfinity;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 240u; ++frame) {
    const JointReport old = *jointReport(world, character.pogo);
    PogoSettings pogo = fixture::pogoFor(world, up);

    if (frame >= 4u) {
      pogo.maxCompression = 0.5 * weight;
    }

    if (frame == 4u) {
      moving = old.pogoSpeed;
    }

    pogo.force = old.pogoForce;
    pogo.speed = old.pogoSpeed;
    REQUIRE(removeJoint(world, character.pogo));
    character.pogo =
      addJoint(world, pogoJoint(noBody, character.body, glm::dvec3(0.0), pogo));
    REQUIRE(character.pogo != 0u);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (frame > 120u) {
      const RigidBody& body = rigidOf(world, character.body);

      lowest = std::min(lowest, body.position.y);
      highest = std::max(highest, body.position.y);
      drift = std::max(drift, std::abs(body.velocity.y));
    }
  }

  REQUIRE(moving > 0.5);
  CHECK(highest - lowest < 1.0e-4);
  CHECK(drift < 1.0e-3);
}

TEST_CASE(
  "a jump's height does not depend on whether the pogo's spring is moving",
  "[dynamics][joints][mover]"
) {
  const double launch = 3.0;

  for (const auto& [tension, release, jump] :
       {std::tuple{0.0, 0.0, 60u},
        std::tuple{0.0, -0.1, 5u},
        std::tuple{0.5, 0.0, 60u},
        std::tuple{0.5, -0.1, 5u}}) {
    INFO(
      "tension cap " << tension << " W, released from " << release
                     << ", jump at frame " << jump
    );

    World world;
    world.parameters.sleep = false;
    PogoSettings pogo = fixture::pogoFor(world, up);
    const double gravity = glm::length(world.parameters.gravity);

    pogo.maxTension = tension * fixture::weightOf(world);

    const fixture::Character character = addFloating(world, release, pogo);
    const double expected =
      launch * launch
      / (2.0 * (gravity + pogo.maxTension / fixture::characterMass));
    double from = 0.0;
    double spring = 0.0;
    double rise = 0.0;

    for (uint32_t frame = 1u; frame <= 120u; ++frame) {
      if (frame == jump) {
        from = anchorOf(world, character.body).y;
        spring = jointReport(world, character.pogo)->pogoSpeed;
        REQUIRE(
          setVelocity(world, character.body, glm::dvec3(0.0, launch, 0.0))
        );
      }

      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      if (frame >= jump) {
        rise = std::max(rise, anchorOf(world, character.body).y - from);
      }
    }

    if (release != 0.0) {
      REQUIRE(spring > 0.5);
    }

    CHECK(std::abs(rise - expected) <= 0.01 * expected);
  }
}

TEST_CASE(
  "a pogo on ground that shakes in impulses neither climbs nor sinks",
  "[dynamics][joints][mover]"
) {
  struct Drive {
    double amplitude;
    double compression;
    double tension;
  };

  const glm::dvec3 platformHalf(1.0, 0.1, 1.0);
  const talus::field::SampledSdf platformField = boxField(platformHalf, 0.05);
  const double omega = 2.0 * std::numbers::pi * 2.0;
  const double host =
    fixture::capScale
    * static_cast<double>(IntegrationParameters{}.jointSubsteps)
    / static_cast<double>(fixture::capSubsteps);

  for (const Drive drive : {Drive{0.10, host, host}, Drive{0.05, host, host}}) {
    INFO(
      "amplitude " << drive.amplitude << " m, compression " << drive.compression
                   << " W, tension " << drive.tension << " W"
    );

    World world;
    world.parameters.sleep = false;

    const uint32_t platform =
      addBox(world, platformField, platformHalf, 0.0, glm::dvec3(0.0));
    const double weight = fixture::weightOf(world);
    fixture::Character character = fixture::addCharacter(
      world,
      glm::dvec3(0.0, centreAbove(platformHalf.y, 0.0), 0.0)
    );
    PogoSettings pogo = fixture::pogoFor(world, up);

    (void)fixture::addUprightSpring(world, character.body);
    pogo.maxCompression = drive.compression * weight;
    pogo.maxTension = drive.tension * weight;
    character.pogo = addJoint(
      world,
      pogoJoint(
        platform,
        character.body,
        glm::dvec3(0.0, platformHalf.y, 0.0),
        pogo
      )
    );
    REQUIRE(character.pogo != 0u);

    std::array<double, 8> means{};

    for (uint32_t frame = 1u; frame <= 240u; ++frame) {
      const double time = static_cast<double>(frame) * world.parameters.dt;

      REQUIRE(setVelocity(
        world,
        platform,
        glm::dvec3(0.0, drive.amplitude * omega * std::sin(omega * time), 0.0)
      ));

      const JointReport old = *jointReport(world, character.pogo);
      JointDef def = *jointDef(world, character.pogo);

      def.pogo.force = old.pogoForce;
      def.pogo.speed = old.pogoSpeed;
      REQUIRE(removeJoint(world, character.pogo));
      character.pogo = addJoint(world, def);
      REQUIRE(character.pogo != 0u);
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      const double gap =
        anchorOf(world, character.body).y
        - (rigidOf(world, platform).position.y + platformHalf.y)
        - fixture::pogoRest;

      means[(frame - 1u) / 30u] += gap / 30.0;
    }

    CHECK(std::abs(means[7] - means[3]) < 5.0e-3);
  }
}

TEST_CASE(
  "a pogo with both caps 0 moves nothing in zero gravity",
  "[dynamics][joints][mover]"
) {
  for (const bool remade : {false, true}) {
    INFO("re-created every frame: " << remade);

    World world;
    world.parameters.sleep = false;
    world.parameters.gravity = glm::dvec3(0.0);

    const uint32_t body = addBox(
      world,
      fixture::characterField(),
      fixture::characterHalf,
      fixture::characterMass,
      glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0),
      glm::dvec3(0.0),
      0.0
    );
    PogoSettings pogo = fixture::pogoFor(world, up);

    pogo.maxTension = 0.0;
    pogo.maxCompression = 0.0;
    pogo.speed = 0.5;

    uint32_t joint =
      addJoint(world, pogoJoint(noBody, body, glm::dvec3(0.0), pogo));
    const glm::dvec3 start = rigidOf(world, body).position;

    REQUIRE(joint != 0u);

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      if (remade) {
        const JointReport old = *jointReport(world, joint);
        JointDef def = *jointDef(world, joint);

        def.pogo.force = old.pogoForce;
        def.pogo.speed = old.pogoSpeed;
        REQUIRE(removeJoint(world, joint));
        joint = addJoint(world, def);
        REQUIRE(joint != 0u);
      }

      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    }

    const JointReport report = *jointReport(world, joint);

    CHECK(rigidOf(world, body).position == start);
    CHECK(report.pogoForce == 0.0);
  }
}

TEST_CASE(
  "editing a pogo in place moves exactly like re-creating it",
  "[dynamics][joints][mover]"
) {
  const auto build = [](World& world) {
    addGround(world);

    fixture::Character character =
      fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0));

    (void)fixture::addUprightSpring(world, character.body);

    return character;
  };
  World remade;
  World edited;
  fixture::Character remadeCharacter = build(remade);
  fixture::Character editedCharacter = build(edited);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    INFO("frame " << frame);

    fixture::MoverStep move;

    move.velocity = glm::dvec3(1.0, 0.0, 0.0);

    const auto remadeHit =
      fixture::moveCharacter(remade, remadeCharacter, move);

    move.update = fixture::PogoUpdate::InPlace;

    const auto editedHit =
      fixture::moveCharacter(edited, editedCharacter, move);

    REQUIRE(remadeHit.has_value() == editedHit.has_value());
    REQUIRE(remadeCharacter.onGround == editedCharacter.onGround);
    REQUIRE(remade.joints.all().size() == edited.joints.all().size());
    REQUIRE_FALSE(talus::dynamics::step(remade).nonFinite);
    REQUIRE_FALSE(talus::dynamics::step(edited).nonFinite);

    const RigidBody& a = rigidOf(remade, remadeCharacter.body);
    const RigidBody& b = rigidOf(edited, editedCharacter.body);

    REQUIRE(b.position == a.position);
    REQUIRE(b.velocity == a.velocity);
    REQUIRE(b.rotation == a.rotation);
    REQUIRE(b.angularVelocity == a.angularVelocity);
  }

  CHECK(rigidOf(edited, editedCharacter.body).position.x > 1.5);
  CHECK(editedCharacter.pogo != 0u);
}

TEST_CASE(
  "a pogo edited in place on a crate never splits the character's island",
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
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
  );
  uint32_t splits = 0u;
  uint32_t onCrate = 0u;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    INFO("frame " << frame);

    fixture::MoverStep move;

    move.update = fixture::PogoUpdate::InPlace;

    (void)fixture::moveCharacter(world, character, move);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (character.ground == crate) {
      ++onCrate;
    }

    if (frame == 2u) {
      splits = world.islandStore.splits();
    }

    if (frame > 2u) {
      REQUIRE(world.islandStore.splits() == splits);
    }
  }

  CHECK(onCrate >= 100u);
  CHECK(character.ground == crate);
}

TEST_CASE(
  "setPogo rejects unknown joints, other kinds and invalid values, and wakes "
  "both bodies",
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
  fixture::Character character = fixture::addCharacter(
    world,
    glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
  );
  const uint32_t pogo = addJoint(
    world,
    pogoJoint(
      crate,
      character.body,
      glm::dvec3(0.0, crateHalf.y, 0.0),
      fixture::pogoFor(world, up)
    )
  );

  REQUIRE(pogo != 0u);

  for (uint32_t frame = 0u;
       frame != 600u && !bothAsleep(world, {crate, character.body});
       ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(bothAsleep(world, {crate, character.body}));

  const JointDef before = *jointDef(world, pogo);
  const JointDef planarBefore = *jointDef(world, character.planar);
  const PogoSettings good = fixture::pogoFor(world, up);
  const auto unchanged = [&](uint32_t joint, const JointDef& expected) {
    const JointDef now = *jointDef(world, joint);

    return now.frameA.position == expected.frameA.position
           && now.frameB.position == expected.frameB.position
           && now.frameB.rotation == expected.frameB.rotation
           && now.pogo.normal == expected.pogo.normal
           && now.pogo.restLength == expected.pogo.restLength
           && now.pogo.force == expected.pogo.force
           && now.pogo.speed == expected.pogo.speed;
  };

  SECTION("unknown joint") {
    CHECK_FALSE(setPogo(world, 9999u, before.frameA, before.frameB, good));
  }

  SECTION("a planar velocity joint") {
    CHECK_FALSE(
      setPogo(world, character.planar, before.frameA, before.frameB, good)
    );
    CHECK(unchanged(character.planar, planarBefore));
  }

  SECTION("a zero-length frame quaternion") {
    JointFrame frame = before.frameB;

    frame.rotation = glm::dquat(0.0, 0.0, 0.0, 0.0);

    CHECK_FALSE(setPogo(world, pogo, before.frameA, frame, good));
    CHECK_FALSE(setPogo(world, pogo, frame, before.frameB, good));
    CHECK(unchanged(pogo, before));
  }

  SECTION("a zero normal") {
    PogoSettings bad = good;

    bad.normal = glm::dvec3(0.0);

    CHECK_FALSE(setPogo(world, pogo, before.frameA, before.frameB, bad));
    CHECK(unchanged(pogo, before));
  }

  SECTION("a negative rest length") {
    PogoSettings bad = good;

    bad.restLength = -1.0;

    CHECK_FALSE(setPogo(world, pogo, before.frameA, before.frameB, bad));
    CHECK(unchanged(pogo, before));
  }

  SECTION("a normal too steep for the foot axis") {
    PogoSettings bad = good;

    bad.normal = glm::dvec3(1.0, 0.0, 0.0);

    CHECK_FALSE(setPogo(world, pogo, before.frameA, before.frameB, bad));
    CHECK(unchanged(pogo, before));
    CHECK(bothAsleep(world, {crate, character.body}));
  }

  SECTION(
    "a valid edit keeps the id, stores normalised values and wakes both bodies"
  ) {
    PogoSettings edit = good;
    JointFrame frameA = before.frameA;
    JointFrame frameB = before.frameB;

    edit.normal = glm::dvec3(0.0, 2.0, 0.0);
    edit.force = 12.0;
    edit.speed = -0.5;
    frameA.position = glm::dvec3(0.1, crateHalf.y, 0.0);
    frameB.rotation = glm::dquat(2.0, 0.0, 0.0, 0.0);

    REQUIRE(setPogo(world, pogo, frameA, frameB, edit));

    const JointDef after = *jointDef(world, pogo);
    const JointReport report = *jointReport(world, pogo);

    CHECK(after.bodyA == crate);
    CHECK(after.bodyB == character.body);
    CHECK(after.frameA.position == frameA.position);
    CHECK(after.frameB.rotation == glm::dquat(1.0, 0.0, 0.0, 0.0));
    CHECK(after.pogo.normal == up);
    CHECK(after.pogo.force == 12.0);
    CHECK(after.pogo.speed == -0.5);
    CHECK(report.pogoForce == 12.0);
    CHECK(report.pogoSpeed == -0.5);
    CHECK_FALSE(rigidOf(world, crate).asleep);
    CHECK_FALSE(rigidOf(world, character.body).asleep);
  }
}

namespace {

JointDef ballBetween(uint32_t a, uint32_t b, const glm::dvec3& halfway) {
  JointDef def;
  def.kind = JointKind::Ball;
  def.bodyA = a;
  def.bodyB = b;
  def.frameA.position = halfway;
  def.frameB.position = -halfway;

  return def;
}

uint32_t splitsOver(World& world, uint32_t frames) {
  uint32_t total = 0u;

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    total += world.islandStore.splits();
  }

  return total;
}

}

TEST_CASE(
  "removing a joint to the world or a mass-0 body never splits an island",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 half(0.2);
  const talus::field::SampledSdf field = boxField(half, 0.05);

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t b = addBox(world, field, half, 10.0, glm::dvec3(0.0));
  const uint32_t c =
    addBox(world, field, half, 10.0, glm::dvec3(1.0, 0.0, 0.0));
  const uint32_t hand =
    addBox(world, field, half, 0.0, glm::dvec3(0.0, 1.0, 0.0));
  JointDef spring;

  spring.kind = JointKind::Spring;
  spring.bodyA = hand;
  spring.bodyB = b;
  spring.frameA.position = glm::dvec3(0.0, -1.0, 0.0);
  spring.spring.linearStiffness = 2000.0;
  spring.spring.linearDamping = 100.0;

  const uint32_t pair =
    addJoint(world, ballBetween(b, c, glm::dvec3(0.5, 0.0, 0.0)));
  const uint32_t held = addJoint(world, spring);
  const uint32_t upright = fixture::addUprightSpring(world, b);

  REQUIRE(pair != 0u);
  REQUIRE(held != 0u);
  run(world, 3u);
  REQUIRE(world.islandStore.islandOf(b) == world.islandStore.islandOf(c));

  REQUIRE(removeJoint(world, held));
  CHECK(splitsOver(world, 5u) == 0u);

  REQUIRE(removeJoint(world, upright));
  CHECK(splitsOver(world, 5u) == 0u);
  CHECK(world.islandStore.islandOf(b) == world.islandStore.islandOf(c));

  REQUIRE(removeJoint(world, pair));
  CHECK(splitsOver(world, 5u) == 1u);
  CHECK(world.islandStore.islandOf(b) != world.islandStore.islandOf(c));
}

TEST_CASE(
  "a joint removed every step does not keep other islands awake",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);
  const glm::dvec3 pairHalf(0.2);
  const talus::field::SampledSdf pairField = boxField(pairHalf, 0.05);

  addGround(world);

  const std::array<double, 2> sides{-1.2, 1.2};
  std::array<uint32_t, 2> crates{};
  std::array<fixture::Character, 2> characters{};

  for (size_t index = 0u; index != sides.size(); ++index) {
    crates[index] = addBox(
      world,
      crateField,
      crateHalf,
      20.0,
      glm::dvec3(-1.5, crateHalf.y, sides[index])
    );
  }

  const std::array<uint32_t, 2> pair{
    addBox(world, pairField, pairHalf, 10.0, glm::dvec3(1.0, pairHalf.y, 0.0)),
    addBox(world, pairField, pairHalf, 10.0, glm::dvec3(2.0, pairHalf.y, 0.0))
  };
  const uint32_t joint =
    addJoint(world, ballBetween(pair[0], pair[1], glm::dvec3(0.5, 0.0, 0.0)));

  REQUIRE(joint != 0u);

  for (size_t index = 0u; index != sides.size(); ++index) {
    characters[index] = fixture::addCharacter(
      world,
      glm::dvec3(-1.5, centreAbove(2.0 * crateHalf.y, 0.0), sides[index])
    );
    characters[index].onGround = true;
  }

  const uint32_t removedAt = 10u;
  const uint32_t sleepFrames = static_cast<uint32_t>(
    std::lround(world.parameters.timeToSleep / world.parameters.dt)
  );
  const uint32_t deadline =
    removedAt
    + static_cast<uint32_t>(
      std::lround((world.parameters.timeToSleep + 1.0) / world.parameters.dt)
    );
  uint32_t onCrate = 0u;
  uint32_t asleepAt = deadline;

  for (uint32_t frame = 0u; frame != deadline; ++frame) {
    INFO("frame " << frame);

    (void)fixture::moveCharacter(world, characters[0], fixture::MoverStep{});

    if (frame == removedAt) {
      REQUIRE(removeJoint(world, joint));
    }

    (void)fixture::moveCharacter(world, characters[1], fixture::MoverStep{});
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    for (size_t index = 0u; index != characters.size(); ++index) {
      if (frame >= removedAt && characters[index].ground == crates[index]) {
        ++onCrate;
      }
    }

    if (asleepAt == deadline && bothAsleep(world, pair)) {
      asleepAt = frame;
    }
  }

  CAPTURE(asleepAt, onCrate, sleepFrames);
  CHECK(onCrate == 2u * (deadline - removedAt));
  CHECK(asleepAt > removedAt + sleepFrames);
  CHECK(bothAsleep(world, pair));
  CHECK(
    world.islandStore.islandOf(pair[0]) != world.islandStore.islandOf(pair[1])
  );
}

TEST_CASE(
  "a character does not lift itself on the crate it carries",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 hand(0.0, -1.0, 0.0);
  const glm::dvec3 crateHalf(0.15);

  addGround(world);

  fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0));

  character.onGround = true;
  (void)fixture::addUprightSpring(world, character.body);

  const fixture::Grip grip =
    fixture::addGrip(world, character.body, hand, crateHalf, 10.0);
  fixture::MoverStep move;

  move.skips = {RaySkip{grip.held, noChild}, RaySkip{grip.hand, noChild}};

  bool onFloor = true;
  double drift = 0.0;

  for (uint32_t frame = 1u; frame <= 180u; ++frame) {
    const std::optional<WorldRayHit> hit =
      fixture::moveCharacter(world, character, move);

    REQUIRE(fixture::driveGrip(world, grip, character.body, hand));
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    onFloor =
      onFloor && character.ground == noBody && hit && hit->side == Side::Static;

    if (frame > 120u) {
      drift = std::max(
        drift,
        std::abs(anchorOf(world, character.body).y - fixture::pogoRest)
      );
    }
  }

  const glm::dvec3 foot = anchorOf(world, character.body);
  const std::array<RaySkip, 2> bodyAndHand{
    RaySkip{character.body, noChild},
    RaySkip{grip.hand, noChild}
  };
  const std::optional<WorldRayHit> unskipped = castRay(
    world,
    foot,
    foot - glm::dvec3(0.0, 2.0 * fixture::pogoRest, 0.0),
    bodyAndHand
  );

  CHECK(onFloor);
  CHECK(drift < 2.0e-3);
  REQUIRE(unskipped.has_value());
  CHECK(unskipped->body == grip.held);
}

TEST_CASE(
  "a character pressing a wall at its force cap stays upright, level and still",
  "[dynamics][joints][mover]"
) {
  World world;
  const glm::dvec3 wallHalf(0.1, 2.0, 2.0);
  const talus::field::SampledSdf wallField = boxField(wallHalf, 0.05);
  const double wallFace = 0.9;
  const double cap = 600.0;

  addGround(world);

  const uint32_t wall = addStatic(
    world.bodies,
    wallField,
    0.5,
    0.0,
    glm::dvec3(wallFace + wallHalf.x, wallHalf.y, 0.0)
  );
  fixture::Character character =
    fixture::addCharacter(world, glm::dvec3(0.0, centreAbove(0.0, 0.0), 0.0));

  (void)fixture::addUprightSpring(world, character.body);

  const double weight = fixture::weightOf(world);
  fixture::MoverStep push;

  push.velocity = glm::dvec3(3.0, 0.0, 0.0);
  push.maxForce = cap;

  double speed = 0.0;
  double press = 0.0;
  double climb = 0.0;
  double tilt = 0.0;
  double carry = 0.0;
  bool wallOnly = true;

  for (uint32_t frame = 1u; frame <= 180u; ++frame) {
    (void)fixture::moveCharacter(world, character, push);
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

    if (frame <= 120u) {
      continue;
    }

    const RigidBody& body = rigidOf(world, character.body);
    const BodyRef ref = *findBody(world.bodies, character.body);
    const std::vector<Contact> contacts = contactsOf(world, ref);

    wallOnly = wallOnly && !contacts.empty();

    for (const Contact& contact : contacts) {
      const bool first =
        contact.a.side == Side::Rigid && contact.a.body == ref.index;
      const ContactSide& other = first ? contact.b : contact.a;

      wallOnly = wallOnly && other.side == Side::Static && other.body == wall;
    }

    speed = std::max(speed, glm::length(body.velocity));
    press = std::max(press, std::abs(normalForceOn(world, ref) - cap));
    climb = std::max(
      climb,
      std::abs(anchorOf(world, character.body).y - fixture::pogoRest)
    );
    tilt = std::max(tilt, tiltOf(body));
    carry = std::max(
      carry,
      std::abs(jointReport(world, character.pogo)->pogoForce - weight)
    );
  }

  CHECK(wallOnly);
  CHECK(speed < 1.0e-3);
  CHECK(press < 0.01 * cap);
  CHECK(climb < 1.0e-3);
  CHECK(tilt < 0.01);
  CHECK(carry < 0.01 * weight);
}

TEST_CASE(
  "a character standing on a sleeping crate wakes it, and the crate sleeps "
  "again after the character leaves",
  "[dynamics][joints][mover]"
) {
  const glm::dvec3 crateHalf(0.3, 0.1, 0.3);
  const talus::field::SampledSdf crateField = boxField(crateHalf, 0.05);

  for (const fixture::PogoUpdate update :
       {fixture::PogoUpdate::Recreate, fixture::PogoUpdate::InPlace}) {
    INFO((
      update == fixture::PogoUpdate::Recreate ? "re-created" : "edited in place"
    ));

    World world;
    addGround(world);

    const uint32_t crate = addBox(
      world,
      crateField,
      crateHalf,
      20.0,
      glm::dvec3(0.0, crateHalf.y, 0.0)
    );

    for (uint32_t frame = 0u; frame != 300u && !rigidOf(world, crate).asleep;
         ++frame) {
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
    }

    REQUIRE(rigidOf(world, crate).asleep);

    const double rest = rigidOf(world, crate).position.y;
    fixture::Character character = fixture::addCharacter(
      world,
      glm::dvec3(0.0, centreAbove(2.0 * crateHalf.y, 0.0), 0.0)
    );

    character.onGround = true;
    (void)fixture::addUprightSpring(world, character.body);

    fixture::MoverStep move;

    move.update = update;

    double sink = 0.0;
    bool onCrate = true;

    for (uint32_t frame = 1u; frame <= 120u; ++frame) {
      (void)fixture::moveCharacter(world, character, move);
      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      if (frame == 1u) {
        CHECK_FALSE(rigidOf(world, crate).asleep);
      }

      onCrate = onCrate && character.ground == crate;
      sink = std::max(sink, std::abs(rigidOf(world, crate).position.y - rest));
    }

    CHECK(onCrate);
    CHECK(sink < 1.0e-3);

    std::optional<uint32_t> left;
    std::optional<uint32_t> slept;

    for (uint32_t frame = 0u; frame != 300u && !slept; ++frame) {
      move.velocity = anchorOf(world, character.body).x < 1.5
                        ? glm::dvec3(1.0, 0.0, 0.0)
                        : glm::dvec3(0.0);

      REQUIRE(fixture::moveCharacter(world, character, move).has_value());

      if (!left && character.ground == noBody) {
        left = frame;
      }

      REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);

      if (left && rigidOf(world, crate).asleep) {
        slept = frame;
      }
    }

    REQUIRE(left.has_value());
    REQUIRE(slept.has_value());
    CAPTURE(*left, *slept);
    CHECK(*slept - *left <= 90u);
  }
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

uint64_t worldHash(uint64_t hash, const World& world) {
  for (const RigidBody& body : world.bodies.rigids) {
    hash = bodyHash(hash, body);
  }

  for (const LatticeBody& lattice : world.bodies.lattices) {
    for (const glm::dvec3& node : lattice.position) {
      for (int axis = 0; axis != 3; ++axis) {
        hash = mixBits(hash, node[axis]);
      }
    }
  }

  for (const Joint& joint : world.joints.all()) {
    for (int axis = 0; axis != 3; ++axis) {
      hash = mixBits(hash, joint.report.force[axis]);
      hash = mixBits(hash, joint.report.torque[axis]);
    }

    hash = mixBits(hash, joint.report.pogoForce);
    hash = mixBits(hash, joint.report.pogoSpeed);
  }

  return hash;
}

bool ownedByStrainstep(const World& world, uint32_t id) {
  const uint32_t index = findBody(world.bodies, id)->index;

  return std::ranges::any_of(world.islands, [&](const Island& island) {
    return island.owner == Owner::Strainstep
           && std::ranges::any_of(island.bodies, [&](const BodyRef& ref) {
                return ref.side == Side::Rigid && ref.index == index;
              });
  });
}

bool touchesLattice(const World& world, uint32_t id) {
  const uint32_t index = findBody(world.bodies, id)->index;

  return std::ranges::any_of(
    contactRecords(world),
    [&](const Contact& contact) {
      const auto rigid = [&](const ContactSide& side) {
        return side.side == Side::Rigid && side.body == index;
      };

      return (rigid(contact.a) && contact.b.side == Side::Lattice)
             || (rigid(contact.b) && contact.a.side == Side::Lattice);
    }
  );
}

struct MoverRun {
  uint64_t hash = 0u;
  bool saturated = false;
  bool onStand = true;
  std::vector<uint32_t> grounds;
  bool strainstep = false;
  bool touched = false;
};

MoverRun moverScene(uint32_t workers, uint32_t width) {
  static const glm::dvec3 pileHalf(0.1);
  static const talus::field::SampledSdf pileField = boxField(pileHalf, 0.05);
  static const glm::dvec3 standHalf(0.3, 0.1, 0.3);
  static const talus::field::SampledSdf standField = boxField(standHalf, 0.05);
  static const glm::dvec3 blockHalf(0.4);
  static const fixture::SoftBlock block = fixture::softBlock(blockHalf, 0.2);
  static const glm::dvec3 gripOffset(0.5, 0.0, 0.0);
  World world;

  world.workerCount = workers;
  world.simdWidth = width;
  addGround(world);

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      for (int level = 0; level != 2; ++level) {
        (void)addBox(
          world,
          pileField,
          pileHalf,
          10.0,
          glm::dvec3(
            -0.86 + 0.21 * column,
            pileHalf.y + 2.01 * pileHalf.y * level,
            0.6 + 0.21 * row
          )
        );
      }
    }
  }

  fixture::addSoftBlock(world, block, glm::dvec3(0.0, blockHalf.y, 0.0));

  const glm::dvec3 standAt(1.5, standHalf.y, -1.5);
  const uint32_t stand = addBox(world, standField, standHalf, 20.0, standAt);
  fixture::Character walker =
    fixture::addCharacter(world, glm::dvec3(-1.0, centreAbove(0.0, 0.0), 0.0));
  fixture::Character stander = fixture::addCharacter(
    world,
    glm::dvec3(standAt.x, centreAbove(2.0 * standHalf.y, 0.0), standAt.z)
  );

  walker.onGround = true;
  stander.onGround = true;
  (void)fixture::addUprightSpring(world, walker.body);
  (void)fixture::addUprightSpring(world, stander.body);

  const fixture::Grip grip =
    fixture::addGrip(world, stander.body, gripOffset, glm::dvec3(0.1), 3.0);
  MoverRun run;

  run.hash = 1469598103934665603ull;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    fixture::MoverStep walk;
    fixture::MoverStep still;

    walk.velocity =
      frame < 60u ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 0.0, 1.5);
    walk.maxForce = 600.0;
    still.update = fixture::PogoUpdate::InPlace;
    still.skips = {RaySkip{grip.hand, noChild}, RaySkip{grip.held, noChild}};

    if (!fixture::tryMoveCharacter(world, walker, walk).applied
        || !fixture::tryMoveCharacter(world, stander, still).applied
        || !fixture::driveGrip(world, grip, stander.body, gripOffset)
        || talus::dynamics::step(world).nonFinite) {
      return {};
    }

    run.saturated =
      run.saturated || jointReport(world, walker.planar)->forceSaturated;
    run.onStand = run.onStand && stander.pogo != 0u
                  && jointDef(world, stander.pogo)->bodyA == stand;

    if (walker.pogo != 0u) {
      const uint32_t ground = jointDef(world, walker.pogo)->bodyA;

      if (std::ranges::find(run.grounds, ground) == run.grounds.end()) {
        run.grounds.push_back(ground);
      }
    }

    run.strainstep = run.strainstep || ownedByStrainstep(world, walker.body);
    run.touched = run.touched || touchesLattice(world, walker.body);
    run.hash = worldHash(run.hash, world);
  }

  return run;
}

}

TEST_CASE(
  "mover worlds step the same at widths 1, 4 and 8 and workers 1, 4 and 8",
  "[dynamics][joints][mover]"
) {
  std::vector<uint32_t> widths{1u, 4u};

  if (hasAvx2()) {
    widths.push_back(8u);
  }

  const MoverRun serial = moverScene(1u, 1u);

  REQUIRE(serial.hash != 0u);
  REQUIRE(serial.saturated);
  REQUIRE(serial.onStand);
  REQUIRE(serial.grounds.size() >= 2u);
  REQUIRE(serial.strainstep);
  REQUIRE(serial.touched);

  for (const uint32_t width : widths) {
    for (const uint32_t workers : {1u, 4u, 8u}) {
      CAPTURE(width, workers);
      CHECK(moverScene(workers, width).hash == serial.hash);
    }
  }
}

namespace {

struct CostRun {
  double hostMs = 0.0;
  double warmMs = 0.0;
  double stepMs = 0.0;
  double jointsMs = 0.0;
  double splits = 0.0;
  double awake = 0.0;
  double planarSaturated = 0.0;
  double scanMs = 0.0;
  size_t jointedBodies = 0u;
};

struct CostCase {
  uint32_t background = 0u;
  fixture::PogoUpdate update = fixture::PogoUpdate::Recreate;
  bool crateGround = false;
  bool pile = false;
};

constexpr uint32_t costSteps = 600u;
constexpr double costRadius = 2.0;
constexpr double costSpeed = 2.0;
constexpr uint32_t costSmall = 5000u;
constexpr uint32_t costLarge = 20000u;
constexpr uint32_t costScanBodies = costLarge - costSmall;
constexpr size_t costScans = 9u;
constexpr double recreateEdits = 3.0;
constexpr double inPlaceEdits = 2.0;
constexpr double gripEdits = 4.0;

uint32_t costCharacters(const CostCase& scenario) {
  return scenario.pile ? 1u : 4u;
}

double costEditsPerStep(const CostCase& scenario) {
  const double mover = scenario.update == fixture::PogoUpdate::Recreate
                         ? recreateEdits
                         : inPlaceEdits;

  return (mover + gripEdits) * static_cast<double>(costCharacters(scenario));
}

CostRun runCost(const CostCase& scenario) {
  using Clock = std::chrono::steady_clock;

  static const talus::field::SampledSdf floorField =
    boxField(glm::dvec3(60.0, 1.0, 60.0), 0.25);
  static const talus::field::SampledSdf backgroundField =
    boxField(glm::dvec3(0.05), 0.05);
  static const glm::dvec3 standHalf(0.3, 0.1, 0.3);
  static const talus::field::SampledSdf standField = boxField(standHalf, 0.05);
  static const glm::dvec3 pileHalf(0.15);
  static const talus::field::SampledSdf pileField = boxField(pileHalf, 0.05);

  World world;

  world.workerCount = 1u;
  (
    void
  )addStatic(world.bodies, floorField, 0.5, 0.0, glm::dvec3(0.0, -1.0, 0.0));

  const uint32_t side = static_cast<uint32_t>(
    std::ceil(std::sqrt(static_cast<double>(scenario.background)))
  );

  for (uint32_t index = 0u; index != scenario.background; ++index) {
    (void)addBox(
      world,
      backgroundField,
      glm::dvec3(0.05),
      1.5,
      glm::dvec3(
        8.0 + 0.3 * static_cast<double>(index % side),
        0.05,
        0.3 * (static_cast<double>(index / side) - 0.5 * side)
      )
    );
  }

  const uint32_t count = costCharacters(scenario);
  std::vector<glm::dvec3> centres;

  for (uint32_t index = 0u; index != count; ++index) {
    centres.emplace_back(
      scenario.pile ? -3.0 : 0.0,
      0.0,
      scenario.pile ? 0.0 : -9.0 + 6.0 * static_cast<double>(index)
    );
  }

  if (scenario.pile) {
    for (uint32_t x = 0u; x != 5u; ++x) {
      for (uint32_t z = 0u; z != 5u; ++z) {
        for (uint32_t y = 0u; y != 4u; ++y) {
          (void)addBox(
            world,
            pileField,
            pileHalf,
            40.0,
            glm::dvec3(
              0.3 * static_cast<double>(x),
              pileHalf.y + 2.0 * pileHalf.y * static_cast<double>(y),
              0.3 * (static_cast<double>(z) - 2.0)
            )
          );
        }
      }
    }
  } else if (scenario.crateGround) {
    for (const glm::dvec3& centre : centres) {
      (void)addBox(
        world,
        standField,
        standHalf,
        20.0,
        glm::dvec3(centre.x + costRadius, standHalf.y, centre.z)
      );
    }
  }

  const auto allAsleep = [&] {
    return std::ranges::all_of(world.bodies.rigids, [](const RigidBody& body) {
      return body.asleep;
    });
  };

  for (uint32_t settle = 0u; settle != 900u && !allAsleep(); ++settle) {
    REQUIRE_FALSE(talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(allAsleep());

  const double top = scenario.crateGround ? 2.0 * standHalf.y : 0.0;
  const std::array<glm::dvec3, 2> offsets{
    glm::dvec3(-0.4, 0.2, 0.3),
    glm::dvec3(0.4, 0.2, 0.3)
  };
  std::vector<fixture::Character> characters;
  std::vector<std::array<fixture::Grip, 2>> grips;

  for (const glm::dvec3& centre : centres) {
    characters.push_back(
      fixture::addCharacter(
        world,
        glm::dvec3(
          centre.x + (scenario.pile ? 0.0 : costRadius),
          centreAbove(top, 0.0),
          centre.z
        )
      )
    );
    (void)fixture::addUprightSpring(world, characters.back().body);
    grips.push_back(
      {fixture::addGrip(
         world,
         characters.back().body,
         offsets[0],
         glm::dvec3(0.1),
         3.0
       ),
       fixture::addGrip(
         world,
         characters.back().body,
         offsets[1],
         glm::dvec3(0.1),
         3.0
       )}
    );
  }

  const auto millisecondsSince = [](const Clock::time_point& from) {
    return std::chrono::duration<double, std::milli>(Clock::now() - from)
      .count();
  };
  double hostMs = 0.0;
  double warmMs = 0.0;
  double stepMs = 0.0;
  double jointsMs = 0.0;
  double awake = 0.0;
  uint32_t splits = 0u;
  uint32_t saturated = 0u;

  for (uint32_t frame = 0u; frame != costSteps; ++frame) {
    const double time = static_cast<double>(frame) / 60.0;
    bool applied = true;
    const auto edit = [&] {
      for (size_t index = 0u; index != characters.size(); ++index) {
        fixture::MoverStep move;

        move.update = scenario.update;

        if (scenario.pile) {
          move.velocity = glm::dvec3(costSpeed, 0.0, 0.0);
        } else if (!scenario.crateGround) {
          move.velocity = costSpeed
                          * glm::dvec3(
                            -std::sin(time * costSpeed / costRadius),
                            0.0,
                            std::cos(time * costSpeed / costRadius)
                          );
        }

        applied =
          fixture::tryMoveCharacter(world, characters[index], move).applied
          && applied;

        for (size_t hand = 0u; hand != 2u; ++hand) {
          applied = fixture::driveGrip(
                      world,
                      grips[index][hand],
                      characters[index].body,
                      offsets[hand]
                    )
                    && applied;
        }
      }
    };
    const Clock::time_point cold = Clock::now();

    edit();
    hostMs += millisecondsSince(cold);

    const Clock::time_point warm = Clock::now();

    edit();
    warmMs += millisecondsSince(warm);
    REQUIRE(applied);

    const StepReport report = talus::dynamics::step(world);

    REQUIRE_FALSE(report.nonFinite);
    stepMs += report.profile.total;
    jointsMs += report.profile.joints;
    splits += world.islandStore.splits();
    saturated +=
      jointReport(world, characters.front().planar)->forceSaturated ? 1u : 0u;
    awake += static_cast<double>(
      std::ranges::count_if(world.bodies.rigids, [](const RigidBody& body) {
        return !body.asleep;
      })
    );
  }

  CostRun result;
  const double steps = static_cast<double>(costSteps);

  if (scenario.background == costLarge) {
    const uint32_t far = world.bodies.rigids[costScanBodies - 1u].id;
    std::array<double, costScans> scans{};

    for (double& scan : scans) {
      const Clock::time_point start = Clock::now();
      const std::optional<BodyRef> found = findBody(world.bodies, far);

      scan = millisecondsSince(start);
      REQUIRE(found);
      REQUIRE(found->index == costScanBodies - 1u);
    }

    std::ranges::sort(scans);
    result.scanMs = scans[costScans / 2u];
  }

  result.hostMs = hostMs / steps;
  result.warmMs = warmMs / steps;
  result.stepMs = stepMs / steps;
  result.jointsMs = jointsMs / steps;
  result.splits = static_cast<double>(splits) / steps;
  result.awake = awake / steps;
  result.planarSaturated = static_cast<double>(saturated) / steps;

  for (const Island& island : world.islands) {
    for (const BodyRef& ref : island.bodies) {
      if (ref.side == Side::Rigid
          && world.bodies.rigids[ref.index].id == characters.front().body) {
        result.jointedBodies = island.bodies.size();
      }
    }
  }

  return result;
}

double median(std::array<double, 3> values) {
  std::ranges::sort(values);

  return values[1];
}

}

TEST_CASE(
  "mover cost with four characters among many sleeping bodies",
  "[.][movercost]"
) {
  for (const bool pile : {false, true}) {
    for (const bool crateGround : {false, true}) {
      if (pile && crateGround) {
        continue;
      }

      std::array<double, 2> smallHost{};
      std::array<double, 2> smallWarm{};

      for (const uint32_t background : {0u, costSmall, costLarge}) {
        for (const fixture::PogoUpdate update :
             {fixture::PogoUpdate::Recreate, fixture::PogoUpdate::InPlace}) {
          CostCase scenario;

          scenario.background = background;
          scenario.update = update;
          scenario.crateGround = crateGround;
          scenario.pile = pile;

          std::array<CostRun, 3> runs;

          for (CostRun& run : runs) {
            run = runCost(scenario);
          }

          const auto pick = [&](double CostRun::* field) {
            return std::array<double, 3>{
              runs[0].*field,
              runs[1].*field,
              runs[2].*field
            };
          };
          const auto spread = [&](double CostRun::* field) {
            const std::array<double, 3> values = pick(field);

            return std::make_pair(
              *std::ranges::min_element(values),
              *std::ranges::max_element(values)
            );
          };
          const auto hostSpread = spread(&CostRun::hostMs);
          const auto stepSpread = spread(&CostRun::stepMs);
          const char* updateName =
            update == fixture::PogoUpdate::Recreate ? "recreate" : "inplace";
          const char* groundName =
            pile ? "pile" : (crateGround ? "crate" : "floor");

          std::printf(
            "movercost bodies %u update %s ground %s host_ms %.4f warm_ms "
            "%.4f step_ms %.4f joints_ms %.4f splits_per_step %.4f awake "
            "%.1f\n",
            background,
            updateName,
            groundName,
            median(pick(&CostRun::hostMs)),
            median(pick(&CostRun::warmMs)),
            median(pick(&CostRun::stepMs)),
            median(pick(&CostRun::jointsMs)),
            median(pick(&CostRun::splits)),
            median(pick(&CostRun::awake))
          );
          std::printf(
            "movercost spread bodies %u update %s ground %s host_ms %.4f..%.4f "
            "step_ms %.4f..%.4f\n",
            background,
            updateName,
            groundName,
            hostSpread.first,
            hostSpread.second,
            stepSpread.first,
            stepSpread.second
          );

          if (pile) {
            std::printf(
              "movercost pile bodies %u update %s jointed_island_bodies %zu "
              "planar_saturated_share %.3f\n",
              background,
              updateName,
              runs[0].jointedBodies,
              median(pick(&CostRun::planarSaturated))
            );
          }

          INFO(
            "bodies " << background << " update " << updateName << " ground "
                      << groundName
          );

          const double host = median(pick(&CostRun::hostMs));
          const double warm = median(pick(&CostRun::warmMs));
          const size_t slot = update == fixture::PogoUpdate::Recreate ? 0u : 1u;

          if (background == costSmall) {
            smallHost[slot] = host;
            smallWarm[slot] = warm;
          } else if (background == costLarge) {
            const double growth = host - smallHost[slot];
            const double edits = costEditsPerStep(scenario);
            const double perEdit = growth / edits;
            const double scan = median(pick(&CostRun::scanMs));

            std::printf(
              "movercost growth update %s ground %s cold_ratio %.2f warm_ratio "
              "%.2f growth_ms %.4f edits_per_step %.0f growth_per_edit_us "
              "%.4f scan_%u_bodies_us %.3f margin %.1f\n",
              updateName,
              groundName,
              host / smallHost[slot],
              warm / smallWarm[slot],
              growth,
              edits,
              1000.0 * perEdit,
              costScanBodies,
              1000.0 * scan,
              scan / perEdit
            );
            CHECK(perEdit < scan);
          }

          if (!pile
              && (update == fixture::PogoUpdate::InPlace || !crateGround)) {
            for (const CostRun& run : runs) {
              CHECK(run.splits == 0.0);
            }
          }
        }
      }
    }
  }
}
