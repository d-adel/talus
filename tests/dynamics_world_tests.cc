#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace {

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

talus::field::SampledSdf floorField() {
  const glm::vec3 low(-1.04f, -0.14f, -1.04f);
  const glm::vec3 high(1.04f, 0.34f, 1.04f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(1.0f, 0.1f, 1.0f))
    },
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );
}

talus::field::SampledSdf sphereField(double radius, double voxel) {
  const float reach = static_cast<float>(radius + 2.0 * voxel);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius))
    },
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(
      static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel)))
      + 1u
    )
  );
}

glm::dquat turnAboutZ(double degrees) {
  return glm::angleAxis(glm::radians(degrees), glm::dvec3(0.0, 0.0, 1.0));
}

void run(talus::dynamics::World& world, uint32_t frames) {
  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }
}

talus::PhysicalMaterial rubber() {
  talus::PhysicalMaterial material;

  material.density = 1000.0f;
  material.youngsModulus = 1.2854547e6f;
  material.poissonsRatio = 0.45f;

  return material;
}

std::shared_ptr<const talus::PreparedMesh> latticeBlock(
  const glm::dvec3& half,
  double voxel,
  std::span<const talus::PhysicalMaterial> lanes,
  talus::field::SampledSdf& field
) {
  field = boxField(half, voxel);

  return talus::deformable::buildMesh(
    field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

uint32_t addCrate(
  talus::dynamics::World& world,
  const talus::field::SampledSdf& crate,
  const glm::dvec3& position
) {
  const uint32_t index = talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    glm::dvec3(0.05),
    1.5,
    position,
    glm::dvec3(0.0),
    0.5,
    3u
  );

  return world.bodies.rigids[index].id;
}

const talus::dynamics::RigidBody& rigidOf(
  const talus::dynamics::World& world,
  uint32_t id
) {
  const std::optional<talus::dynamics::BodyRef> ref =
    talus::dynamics::findBody(world.bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == talus::dynamics::Side::Rigid);

  return world.bodies.rigids[ref->index];
}

std::vector<talus::dynamics::ContactHitEvent> collectHits(
  talus::dynamics::World& world,
  uint32_t frames
) {
  std::vector<talus::dynamics::ContactHitEvent> hits;

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    hits.insert(hits.end(), report.hits.begin(), report.hits.end());
  }

  return hits;
}

glm::dvec3 latticeCentre(const talus::dynamics::LatticeBody& body) {
  glm::dvec3 centre(0.0);

  for (const glm::dvec3& position : body.position) {
    centre += position;
  }

  return centre / static_cast<double>(body.position.size());
}

uint32_t placeOnRamp(
  talus::dynamics::World& world,
  const talus::field::SampledSdf& crate,
  const glm::dquat& ramp,
  double friction
) {
  const uint32_t index = talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    glm::dvec3(0.05),
    1.5,
    ramp * glm::dvec3(0.0, 0.151, 0.0),
    glm::dvec3(0.0),
    friction,
    3u
  );
  talus::dynamics::RigidBody& box = world.bodies.rigids[index];

  box.rotation = ramp;
  box.initialRotation = ramp;
  box.inertialRotation = ramp;

  return index;
}

}

TEST_CASE(
  "a box rests on a static floor raised by its pose",
  "[dynamics][world][static]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(
    world.bodies,
    floor,
    0.5,
    0.0,
    glm::dvec3(0.0, 1.0, 0.0)
  );
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 1.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  run(world, 120u);

  const talus::dynamics::RigidBody& box = world.bodies.rigids.front();

  CHECK(std::abs(box.position.y - 1.15) <= 0.01);
  CHECK(glm::length(box.velocity) <= 0.05);
}

TEST_CASE(
  "a static field turned by its pose has turned bounds",
  "[dynamics][world][static]"
) {
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(
    bodies,
    floor,
    0.5,
    0.0,
    glm::dvec3(1.0, 0.0, 0.0),
    turnAboutZ(90.0)
  );

  const talus::dynamics::Bounds bounds = talus::dynamics::boundsOf(
    bodies,
    {talus::dynamics::Side::Static, 0u},
    0.0
  );

  CHECK(std::abs(bounds.low.x - 0.66) <= 1.0e-4);
  CHECK(std::abs(bounds.high.x - 1.14) <= 1.0e-4);
  CHECK(std::abs(bounds.low.y + 1.04) <= 1.0e-4);
  CHECK(std::abs(bounds.high.y - 1.04) <= 1.0e-4);
}

TEST_CASE(
  "a static pose with a non-unit rotation is normalised",
  "[dynamics][world][static]"
) {
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(
    bodies,
    floor,
    0.5,
    0.0,
    glm::dvec3(0.0),
    2.0 * turnAboutZ(90.0)
  );

  CHECK(std::abs(glm::length(bodies.statics.front().rotation) - 1.0) <= 1.0e-12);
}

TEST_CASE(
  "a box holds still on a static ramp turned by its pose",
  "[dynamics][world][static]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  const glm::dquat ramp = turnAboutZ(20.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0, glm::dvec3(0.0), ramp);

  const uint32_t index = placeOnRamp(world, crate, ramp, 0.5);

  run(world, 60u);

  const glm::dvec3 settled = world.bodies.rigids[index].position;
  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    fastest = std::max(fastest, glm::length(world.bodies.rigids[index].velocity));
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[index];
  const glm::dvec3 up = ramp * glm::dvec3(0.0, 1.0, 0.0);

  CHECK(glm::length(box.position - settled) < 0.01);
  CHECK(fastest < 0.05);
  CHECK(std::abs(glm::dot(box.position, up) - 0.15) <= 0.01);
  CHECK(
    glm::dot(box.rotation * glm::dvec3(0.0, 1.0, 0.0), up)
    >= std::cos(glm::radians(1.0))
  );
}

TEST_CASE(
  "a box slides down a steep static ramp turned by its pose",
  "[dynamics][world][static]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  const glm::dquat ramp = turnAboutZ(30.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.1, 0.0, glm::dvec3(0.0), ramp);

  const uint32_t index = placeOnRamp(world, crate, ramp, 0.1);

  run(world, 30u);

  const glm::dvec3 downhill = ramp * glm::dvec3(-1.0, 0.0, 0.0);
  const glm::dvec3 velocity = world.bodies.rigids[index].velocity;
  const double along = glm::dot(velocity, downhill);

  CHECK(along > 0.0);
  CHECK(glm::length(velocity - along * downhill) <= 0.1 * glm::length(velocity));
}

TEST_CASE(
  "a fast ball does not pass through a static wall turned by its pose",
  "[dynamics][world][static]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf wall = floorField();
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addStatic(
    world.bodies,
    wall,
    0.5,
    0.0,
    glm::dvec3(1.0, 0.0, 0.0),
    turnAboutZ(90.0)
  );
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0),
    glm::dvec3(60.0, 0.0, 0.0),
    0.5,
    0.0
  );

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    CHECK(world.bodies.rigids.front().position.x < 0.9);
  }
}

TEST_CASE(
  "removing a rigid body leaves the other bodies stepping",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t first = addCrate(world, crate, glm::dvec3(-0.3, 0.15, 0.0));
  const uint32_t second = addCrate(world, crate, glm::dvec3(0.3, 0.15, 0.0));

  run(world, 60u);
  REQUIRE(talus::dynamics::removeBody(world, first));
  run(world, 60u);

  REQUIRE(world.bodies.rigids.size() == 1u);
  CHECK(!talus::dynamics::findBody(world.bodies, first).has_value());
  CHECK(std::abs(rigidOf(world, second).position.y - 0.15) <= 0.01);
  CHECK(std::abs(rigidOf(world, second).position.x - 0.3) <= 0.01);
}

TEST_CASE(
  "removing the lower of two stacked boxes lets the upper one fall to the floor",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t lower = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));
  const uint32_t upper = addCrate(world, crate, glm::dvec3(0.0, 0.251, 0.0));

  run(world, 90u);
  REQUIRE(talus::dynamics::removeBody(world, lower));
  run(world, 90u);

  CHECK(std::abs(rigidOf(world, upper).position.y - 0.15) <= 0.01);
}

TEST_CASE(
  "removing the floor lets a resting box fall",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 60u);
  REQUIRE(talus::dynamics::removeStatic(world, world.bodies.statics.front().id));
  run(world, 30u);

  CHECK(world.bodies.statics.empty());
  CHECK(rigidOf(world, box).position.y < 0.0);
}

TEST_CASE(
  "removing a switchable body removes its record and its body",
  "[dynamics][world][edits]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  run(world, 10u);
  REQUIRE(talus::dynamics::removeBody(world, id));

  CHECK(world.bodies.switchables.empty());
  CHECK(world.bodies.rigids.empty());
  CHECK(!talus::dynamics::findBody(world.bodies, id).has_value());

  run(world, 10u);
}

TEST_CASE(
  "removing a switchable body in its lattice tier removes the lattice",
  "[dynamics][world][edits]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  talus::dynamics::promote(world, id);
  REQUIRE(world.bodies.lattices.size() == 1u);
  REQUIRE(talus::dynamics::removeBody(world, id));

  CHECK(world.bodies.lattices.empty());
  CHECK(world.bodies.switchables.empty());

  run(world, 10u);
}

TEST_CASE(
  "removing an unknown body or static changes nothing",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  (void)addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  CHECK(!talus::dynamics::removeBody(world, 999u));
  CHECK(!talus::dynamics::removeStatic(world, 5u));
  CHECK(world.bodies.rigids.size() == 1u);
  CHECK(world.bodies.statics.size() == 1u);
}

TEST_CASE(
  "a pose edit moves a resting box and it lands again",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 60u);
  REQUIRE(talus::dynamics::setRigidPose(
    world,
    box,
    glm::dvec3(0.5, 0.6, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  ));

  CHECK(rigidOf(world, box).position.x == 0.5);

  run(world, 90u);

  CHECK(std::abs(rigidOf(world, box).position.y - 0.15) <= 0.01);
  CHECK(std::abs(rigidOf(world, box).position.x - 0.5) <= 0.01);
}

TEST_CASE(
  "a velocity edit gives a free box that velocity",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0));

  REQUIRE(talus::dynamics::setVelocity(world, box, glm::dvec3(1.0, 0.0, 0.0)));
  run(world, 1u);

  CHECK(glm::length(rigidOf(world, box).velocity - glm::dvec3(1.0, 0.0, 0.0)) <= 1.0e-6);
  CHECK(std::abs(rigidOf(world, box).position.x - world.parameters.dt) <= 1.0e-6);
}

TEST_CASE(
  "a velocity edit moves a free lattice's centre of mass at that speed",
  "[dynamics][world][edits]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t index = talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5
  );
  const uint32_t id = world.bodies.lattices[index].id;

  REQUIRE(talus::dynamics::setVelocity(world, id, glm::dvec3(0.0, 0.0, 2.0)));
  run(world, 10u);

  const talus::dynamics::LatticeBody& body = world.bodies.lattices.front();
  double mass = 0.0;
  glm::dvec3 momentum(0.0);

  for (size_t node = 0u; node != body.velocity.size(); ++node) {
    mass += static_cast<double>(mesh->nodeMass[node]);
    momentum += static_cast<double>(mesh->nodeMass[node]) * body.velocity[node];
  }

  CHECK(glm::length(momentum / mass - glm::dvec3(0.0, 0.0, 2.0)) <= 1.0e-6);
}

TEST_CASE(
  "a mass edit changes the weight a box puts on the floor",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 60u);
  REQUIRE(talus::dynamics::setMass(world, box, 3.0));
  run(world, 60u);

  const double load = talus::dynamics::normalForceOn(
    world,
    {talus::dynamics::Side::Rigid, 0u}
  );

  CHECK(std::abs(load - 3.0 * 9.81) <= 0.10 * 3.0 * 9.81);
}

TEST_CASE(
  "a mass edit is refused for a switchable body and a mass that is not positive",
  "[dynamics][world][edits]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  talus::dynamics::World world;

  const uint32_t block = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  const uint32_t box = addCrate(world, crate, glm::dvec3(1.0, 0.15, 0.0));

  CHECK(!talus::dynamics::setMass(world, block, 2.0));
  CHECK(!talus::dynamics::setMass(world, box, 0.0));
  CHECK(rigidOf(world, box).mass == 1.5);
}

TEST_CASE(
  "a static pose edit lowers the floor and the box comes to rest on it",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 60u);
  REQUIRE(talus::dynamics::setStaticPose(
    world,
    world.bodies.statics.front().id,
    glm::dvec3(0.0, -0.2, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  ));
  run(world, 90u);

  CHECK(std::abs(rigidOf(world, box).position.y + 0.05) <= 0.01);
}

TEST_CASE(
  "a shape replacement makes a box rest at its new size",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf large = boxField(glm::dvec3(0.1), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;
  talus::dynamics::BodySet scratch;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  talus::dynamics::addRigidBox(
    scratch,
    large,
    glm::dvec3(0.1),
    1.5,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  run(world, 60u);
  REQUIRE(talus::dynamics::replaceRigidShape(world, box, scratch.rigids.front()));
  run(world, 120u);

  CHECK(std::abs(rigidOf(world, box).position.y - 0.2) <= 0.01);
}

TEST_CASE(
  "a box at rest on the floor falls asleep after half a second",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, box).asleep);

  const glm::dvec3 held = rigidOf(world, box).position;

  run(world, 60u);

  CHECK(rigidOf(world, box).position == held);
}

TEST_CASE(
  "sleep is on by default",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);

  CHECK(rigidOf(world, box).asleep);
}

TEST_CASE(
  "turning sleep off wakes a sleeping box",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, box).asleep);

  world.parameters.sleep = false;
  run(world, 1u);

  CHECK(!rigidOf(world, box).asleep);
}

TEST_CASE(
  "a box dropped onto a sleeping box wakes it and both sleep again",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t lower = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, lower).asleep);

  const uint32_t upper = addCrate(world, crate, glm::dvec3(0.0, 0.6, 0.0));
  bool woke = false;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    woke = woke || !rigidOf(world, lower).asleep;
  }

  run(world, 240u);

  CHECK(woke);
  CHECK(rigidOf(world, lower).asleep);
  CHECK(rigidOf(world, upper).asleep);
  CHECK(std::abs(rigidOf(world, upper).position.y - 0.25) <= 0.01);
}

TEST_CASE(
  "a slow box that slides into a sleeping box wakes it and keeps its own sleep timer",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t sleeper = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, sleeper).asleep);

  const uint32_t index = talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    glm::dvec3(0.05),
    1.5,
    glm::dvec3(-0.125, 0.15, 0.0),
    glm::dvec3(0.045, 0.0, 0.0),
    0.0,
    3u
  );
  const uint32_t slider = world.bodies.rigids[index].id;
  uint32_t frames = 0u;

  while (frames != 60u && rigidOf(world, sleeper).asleep) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    ++frames;
  }

  REQUIRE(!rigidOf(world, sleeper).asleep);
  REQUIRE(frames > 2u);
  CHECK(!rigidOf(world, slider).asleep);
  CHECK(rigidOf(world, sleeper).sleepTime <= world.parameters.dt);
  CHECK(rigidOf(world, slider).sleepTime >= static_cast<double>(frames - 1u) * world.parameters.dt);
}

TEST_CASE(
  "a sleeping box touched by a moving mass-0 body wakes and is pushed",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  uint32_t settle = 0u;

  while (settle != 300u && !rigidOf(world, box).asleep) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    ++settle;
  }

  REQUIRE(rigidOf(world, box).asleep);

  const uint32_t index = talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    glm::dvec3(0.05),
    0.0,
    glm::dvec3(-0.3, 0.15, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    0.5,
    3u
  );
  const uint32_t pusher = world.bodies.rigids[index].id;
  bool awakeByFrame30 = false;
  double deepestOverlap = 0.0;

  for (uint32_t frame = 1u; frame <= 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame == 30u) {
      awakeByFrame30 = !rigidOf(world, box).asleep;
    }

    const double gap = rigidOf(world, box).position.x - rigidOf(world, pusher).position.x - 0.1;

    deepestOverlap = std::max(deepestOverlap, -gap);
  }

  CHECK(awakeByFrame30);
  CHECK(rigidOf(world, box).position.x >= 0.2);
  CHECK(deepestOverlap <= world.parameters.speculativeDistance);
}

TEST_CASE(
  "a glass pane box under a steel block box on a sloped zero-mass box ledge does not turn under it",
  "[dynamics][world][sleep]"
) {
  const glm::dquat tilt = turnAboutZ(25.0);
  const glm::dmat3 slope = glm::mat3_cast(tilt);
  const glm::dvec3 normal = slope * glm::dvec3(0.0, 1.0, 0.0);
  const glm::dvec3 ledgeHalf(0.6, 0.05, 0.15);
  const glm::dvec3 paneHalf(0.35, 0.005, 0.3);
  const glm::dvec3 blockHalf(0.25, 0.25, 0.2);
  const talus::field::SampledSdf ledgeShape = boxField(ledgeHalf, 0.02);
  const talus::field::SampledSdf paneShape = boxField(paneHalf, 0.01);
  const talus::field::SampledSdf blockShape = boxField(blockHalf, 0.02);
  talus::dynamics::World world;
  const auto place = [&](const talus::field::SampledSdf& shape, const glm::dvec3& half, double mass, const glm::dvec3& position) {
    const uint32_t index = talus::dynamics::addRigidBox(world.bodies, shape, half, mass, position, glm::dvec3(0.0), 0.6, 2u);
    talus::dynamics::RigidBody& body = world.bodies.rigids[index];

    body.rotation = tilt;
    body.initialRotation = tilt;
    body.inertialRotation = tilt;

    return body.id;
  };

  (void)place(ledgeShape, ledgeHalf, 0.0, glm::dvec3(0.0));

  const uint32_t glass = place(paneShape, paneHalf, 2500.0 * 8.0 * paneHalf.x * paneHalf.y * paneHalf.z, slope * glm::dvec3(0.0, ledgeHalf.y + paneHalf.y, 0.0));
  const uint32_t steel = place(
    blockShape,
    blockHalf,
    7850.0 * 8.0 * blockHalf.x * blockHalf.y * blockHalf.z,
    slope * glm::dvec3(0.0, ledgeHalf.y + 2.0 * paneHalf.y + blockHalf.y, 0.08)
  );
  const double reach = glm::length(paneHalf);
  double turned = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const glm::dquat turn = rigidOf(world, glass).rotation * glm::conjugate(tilt);
    const double twist = 2.0 * std::asin(std::clamp(glm::dot(glm::dvec3(turn.x, turn.y, turn.z), normal), -1.0, 1.0));

    turned = std::max(turned, std::abs(twist) * reach);
  }

  CHECK(turned < 0.01);
  CHECK(rigidOf(world, glass).asleep);
  CHECK(rigidOf(world, steel).asleep);
}

TEST_CASE(
  "a glass pane box under a steel block box that carries a soft block, on a sloped zero-mass box ledge, does not turn under it while it stays awake",
  "[dynamics][world][sleep]"
) {
  const glm::dquat tilt = turnAboutZ(25.0);
  const glm::dmat3 slope = glm::mat3_cast(tilt);
  const glm::dvec3 normal = slope * glm::dvec3(0.0, 1.0, 0.0);
  const glm::dvec3 ledgeHalf(0.6, 0.05, 0.15);
  const glm::dvec3 paneHalf(0.35, 0.005, 0.3);
  const glm::dvec3 blockHalf(0.25, 0.25, 0.2);
  const talus::field::SampledSdf ledgeShape = boxField(ledgeHalf, 0.02);
  const talus::field::SampledSdf paneShape = boxField(paneHalf, 0.01);
  const talus::field::SampledSdf blockShape = boxField(blockHalf, 0.02);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf softField;
  const auto mesh = latticeBlock(glm::dvec3(0.05), 0.025, lanes, softField);
  talus::dynamics::World world;
  const auto place = [&](const talus::field::SampledSdf& shape, const glm::dvec3& half, double mass, const glm::dvec3& position) {
    const uint32_t index = talus::dynamics::addRigidBox(world.bodies, shape, half, mass, position, glm::dvec3(0.0), 0.6, 2u);
    talus::dynamics::RigidBody& body = world.bodies.rigids[index];

    body.rotation = tilt;
    body.initialRotation = tilt;
    body.inertialRotation = tilt;

    return body.id;
  };

  (void)place(ledgeShape, ledgeHalf, 0.0, glm::dvec3(0.0));

  const uint32_t glass = place(paneShape, paneHalf, 2500.0 * 8.0 * paneHalf.x * paneHalf.y * paneHalf.z, slope * glm::dvec3(0.0, ledgeHalf.y + paneHalf.y, 0.0));
  const glm::dvec3 blockAt = slope * glm::dvec3(0.0, ledgeHalf.y + 2.0 * paneHalf.y + blockHalf.y, 0.08);

  (void)place(blockShape, blockHalf, 7850.0 * 8.0 * blockHalf.x * blockHalf.y * blockHalf.z, blockAt);

  const glm::dvec3 softAt = blockAt + slope * glm::dvec3(0.0, blockHalf.y + 0.052, 0.0);
  talus::dynamics::LatticeBody& soft = world.bodies.lattices[talus::dynamics::addLattice(world.bodies, mesh, lanes, softAt, glm::dvec3(0.0), 0.9, &softField)];

  for (size_t node = 0u; node != soft.position.size(); ++node) {
    soft.position[node] = softAt + slope * (soft.position[node] - softAt);
    soft.initial[node] = soft.position[node];
    soft.inertial[node] = soft.position[node];
  }

  world.parameters.sleep = false;

  const double reach = glm::length(paneHalf);
  double turned = 0.0;
  uint32_t latticeIslands = 0u;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    latticeIslands = std::max(latticeIslands, report.strainstepIslands);

    const glm::dquat turn = rigidOf(world, glass).rotation * glm::conjugate(tilt);
    const double twist = 2.0 * std::asin(std::clamp(glm::dot(glm::dvec3(turn.x, turn.y, turn.z), normal), -1.0, 1.0));

    turned = std::max(turned, std::abs(twist) * reach);
  }

  REQUIRE(latticeIslands != 0u);
  CHECK(turned < 0.01);
}

TEST_CASE(
  "a rolling ball stays awake",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t index = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(-0.8, 0.15, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    0.5,
    0.0
  );

  world.bodies.rigids[index].angularVelocity = glm::dvec3(0.0, 0.0, -20.0);
  run(world, 60u);

  CHECK(!world.bodies.rigids[index].asleep);
}

TEST_CASE(
  "a velocity edit wakes a sleeping box",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, box).asleep);

  const double before = rigidOf(world, box).position.y;

  REQUIRE(talus::dynamics::setVelocity(world, box, glm::dvec3(0.0, 2.0, 0.0)));
  CHECK(!rigidOf(world, box).asleep);

  run(world, 1u);

  CHECK(rigidOf(world, box).position.y > before);
}

TEST_CASE(
  "moving a box out from under a sleeping box drops the box on top",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t lower = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));
  const uint32_t upper = addCrate(world, crate, glm::dvec3(0.0, 0.251, 0.0));

  run(world, 180u);
  REQUIRE(rigidOf(world, upper).asleep);
  REQUIRE(talus::dynamics::setRigidPose(
    world,
    lower,
    glm::dvec3(0.6, 0.15, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  ));
  run(world, 90u);

  CHECK(std::abs(rigidOf(world, upper).position.y - 0.15) <= 0.01);
}

TEST_CASE(
  "removing the floor under a sleeping box drops it",
  "[dynamics][world][sleep]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t box = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 120u);
  REQUIRE(rigidOf(world, box).asleep);
  REQUIRE(talus::dynamics::removeStatic(world, world.bodies.statics.front().id));
  run(world, 30u);

  CHECK(rigidOf(world, box).position.y < 0.0);
}

TEST_CASE(
  "a resting lattice block falls asleep",
  "[dynamics][world][sleep]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.151, 0.0),
    glm::dvec3(0.0),
    0.5
  );

  run(world, 180u);

  CHECK(world.bodies.lattices.front().asleep);
}

TEST_CASE(
  "a ball dropped on the floor reports a hit at its approach speed",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  const std::vector<talus::dynamics::ContactHitEvent> hits = collectHits(world, 60u);
  const double impact = std::sqrt(2.0 * 9.81 * 0.85);

  REQUIRE(!hits.empty());
  CHECK(hits.size() <= 3u);

  const talus::dynamics::ContactHitEvent& first = hits.front();
  double impulse = 0.0;

  for (const talus::dynamics::ContactHitEvent& hit : hits) {
    impulse += hit.impulse;
  }

  const double gravity =
    9.81 * world.parameters.dt * static_cast<double>(hits.size());

  CHECK(first.firstSide == talus::dynamics::Side::Rigid);
  CHECK(first.secondSide == talus::dynamics::Side::Static);
  CHECK(std::abs(first.approachSpeed - impact) <= 0.10 * impact);
  CHECK(std::abs(impulse - (first.approachSpeed + gravity)) <= 0.10 * impact);
  CHECK(glm::dot(first.normal, glm::dvec3(0.0, -1.0, 0.0)) >= 0.99);
  CHECK(std::abs(first.point.y - 0.1) <= 0.02);
}

TEST_CASE(
  "a box resting on the floor reports no hits",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  (void)addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  run(world, 60u);

  CHECK(collectHits(world, 60u).empty());
}

TEST_CASE(
  "a touch slower than the threshold reports no hit",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.18, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  CHECK(collectHits(world, 60u).empty());
}

TEST_CASE(
  "a higher threshold hides a hit",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  world.parameters.hitEventThreshold = 5.0;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  CHECK(collectHits(world, 60u).empty());
}

TEST_CASE(
  "a sleeping pair reports no hits",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t index = talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  REQUIRE(!collectHits(world, 60u).empty());
  run(world, 120u);
  REQUIRE(world.bodies.rigids[index].asleep);

  CHECK(collectHits(world, 60u).empty());
}

TEST_CASE(
  "a lattice block dropped on the floor reports a hit at its approach speed",
  "[dynamics][world][hits]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.6, 0.0),
    glm::dvec3(0.0),
    0.5
  );

  const std::vector<talus::dynamics::ContactHitEvent> hits = collectHits(world, 60u);
  const double impact = std::sqrt(2.0 * 9.81 * 0.45);

  REQUIRE(!hits.empty());
  CHECK(hits.front().firstSide == talus::dynamics::Side::Lattice);
  CHECK(std::abs(hits.front().approachSpeed - impact) <= 0.10 * impact);
  CHECK(hits.front().impulse > 0.0);
}

TEST_CASE(
  "removing a sleeping crate from under a sleeping lattice block drops the block",
  "[dynamics][world][sleep]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t under = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.251, 0.0),
    glm::dvec3(0.0),
    0.5
  );

  run(world, 240u);
  REQUIRE(rigidOf(world, under).asleep);
  REQUIRE(world.bodies.lattices.front().asleep);
  REQUIRE(talus::dynamics::removeBody(world, under));
  run(world, 90u);

  CHECK(std::abs(latticeCentre(world.bodies.lattices.front()).y - 0.15) <= 0.01);
}

TEST_CASE(
  "moving a sleeping crate out from under a sleeping lattice block drops the block",
  "[dynamics][world][sleep]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t under = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.251, 0.0),
    glm::dvec3(0.0),
    0.5
  );

  run(world, 240u);
  REQUIRE(rigidOf(world, under).asleep);
  REQUIRE(world.bodies.lattices.front().asleep);
  REQUIRE(talus::dynamics::setRigidPose(
    world,
    under,
    glm::dvec3(0.6, 0.15, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  ));
  run(world, 90u);

  CHECK(std::abs(latticeCentre(world.bodies.lattices.front()).y - 0.15) <= 0.01);
}

TEST_CASE(
  "a crate sleeping on a lattice block falls when the block is removed",
  "[dynamics][world][sleep]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t index = talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.151, 0.0),
    glm::dvec3(0.0),
    0.5
  );
  const uint32_t block = world.bodies.lattices[index].id;
  const uint32_t above = addCrate(world, crate, glm::dvec3(0.0, 0.251, 0.0));

  run(world, 240u);
  REQUIRE(rigidOf(world, above).asleep);
  REQUIRE(world.bodies.lattices.front().asleep);
  REQUIRE(talus::dynamics::removeBody(world, block));
  run(world, 90u);

  CHECK(std::abs(rigidOf(world, above).position.y - 0.15) <= 0.01);
}

TEST_CASE(
  "a ball hitting a sleeping lattice block wakes the crate under it in the same step",
  "[dynamics][world][sleep]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t under = addCrate(world, crate, glm::dvec3(0.0, 0.15, 0.0));

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.251, 0.0),
    glm::dvec3(0.0),
    0.5
  );

  run(world, 240u);
  REQUIRE(rigidOf(world, under).asleep);
  REQUIRE(world.bodies.lattices.front().asleep);

  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.6, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  bool woke = false;
  bool together = true;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const bool blockAwake = !world.bodies.lattices.front().asleep;

    woke = woke || blockAwake;
    together = together && (!blockAwake || !rigidOf(world, under).asleep);
  }

  CHECK(woke);
  CHECK(together);
}

TEST_CASE(
  "a static keeps its id when an earlier static is removed",
  "[dynamics][world][edits]"
) {
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0, glm::dvec3(5.0, 0.0, 0.0));
  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t first = world.bodies.statics[0].id;
  const uint32_t second = world.bodies.statics[1].id;

  REQUIRE(first != second);
  REQUIRE(talus::dynamics::removeStatic(world, first));
  REQUIRE(talus::dynamics::setStaticPose(
    world,
    second,
    glm::dvec3(0.0, -0.2, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  ));

  CHECK(world.bodies.statics.front().position.y == -0.2);
  CHECK(!talus::dynamics::removeStatic(world, first));
}

TEST_CASE(
  "a hit names a static by its id",
  "[dynamics][world][hits]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  const talus::field::SampledSdf floor = floorField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0, glm::dvec3(5.0, 0.0, 0.0));
  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t ground = world.bodies.statics[1].id;

  REQUIRE(talus::dynamics::removeStatic(world, world.bodies.statics[0].id));
  talus::dynamics::addRigidSphere(
    world.bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  const std::vector<talus::dynamics::ContactHitEvent> hits = collectHits(world, 60u);

  REQUIRE(!hits.empty());
  CHECK(hits.front().secondSide == talus::dynamics::Side::Static);
  CHECK(hits.front().second == ground);
}
