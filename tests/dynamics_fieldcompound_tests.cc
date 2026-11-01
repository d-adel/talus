#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/fieldbody/fieldbody.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

namespace {

using namespace talus::dynamics;

constexpr double ballRadius = 0.1;
constexpr double ballMass = 2.0;
constexpr uint32_t sampleBudget = 160u;
const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);

talus::field::SampledSdf baked(
  const std::vector<talus::field::Primitive>& primitives,
  const glm::vec3& low,
  const glm::vec3& high,
  float voxel
) {
  return talus::field::bake(
    primitives,
    low,
    high,
    glm::uvec3(glm::round((high - low) / voxel)) + glm::uvec3(1u)
  );
}

const talus::field::SampledSdf& ballField() {
  static const talus::field::SampledSdf field = baked(
    {fixture::sphere(glm::vec3(0.0f), static_cast<float>(ballRadius))},
    glm::vec3(-0.14f),
    glm::vec3(0.14f),
    0.01f
  );

  return field;
}

const talus::field::SampledSdf& peanutField() {
  static const talus::field::SampledSdf field = baked(
    {fixture::sphere(glm::vec3(-0.04f, 0.0f, 0.0f), 0.06f),
     fixture::sphere(glm::vec3(0.04f, 0.0f, 0.0f), 0.06f)},
    glm::vec3(-0.13f, -0.09f, -0.09f),
    glm::vec3(0.13f, 0.09f, 0.09f),
    0.01f
  );

  return field;
}

uint32_t addFieldBody(
  World& world,
  const talus::field::SampledSdf& field,
  double mass,
  const glm::dvec3& position,
  const glm::dquat& rotation,
  double friction
) {
  RigidBody body = rigidFromField(
    field,
    {},
    std::vector<talus::PhysicalMaterial>(1),
    sampleBudget
  );

  REQUIRE(body.shape == RigidShape::Sampled);
  REQUIRE(body.samples.size() > sampleBudget / 2u);
  REQUIRE(body.mass > 0.0);

  body.moment *= mass / body.mass;
  body.mass = mass;
  body.position = position;
  body.initialPosition = position;
  body.inertialPosition = position;
  body.rotation = rotation;
  body.initialRotation = rotation;
  body.inertialRotation = rotation;
  body.friction = friction;
  body.id = world.bodies.nextId++;
  world.bodies.rigids.push_back(std::move(body));

  return static_cast<uint32_t>(world.bodies.rigids.size() - 1u);
}

uint32_t addBall(World& world, const glm::dvec3& position, double friction) {
  return addFieldBody(world, ballField(), ballMass, position, noTurn, friction);
}

CompoundChild part(
  const glm::dvec3& centre,
  const glm::dvec3& half,
  uint32_t key
) {
  CompoundChild child;

  child.centre = centre;
  child.half = half;
  child.density = 1000.0;
  child.key = key;

  return child;
}

uint32_t addAnchoredBox(
  World& world,
  const glm::dvec3& centre,
  const glm::dvec3& half
) {
  return addCompoundBody(
    world.bodies,
    std::vector<CompoundChild>{part(glm::dvec3(0.0), half, 0u)},
    centre,
    noTurn,
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );
}

uint32_t addFloor(World& world) {
  return addAnchoredBox(
    world,
    glm::dvec3(0.0, -0.5, 0.0),
    glm::dvec3(2.0, 0.5, 2.0)
  );
}

std::vector<glm::dvec3> worldSamples(const RigidBody& body) {
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  std::vector<glm::dvec3> points;

  for (const glm::dvec3& sample : body.samples) {
    points.push_back(body.position + turn * sample);
  }

  return points;
}

double lowestPoint(const RigidBody& body) {
  double lowest = std::numeric_limits<double>::infinity();

  for (const glm::dvec3& point : worldSamples(body)) {
    lowest = std::min(lowest, point.y);
  }

  return lowest;
}

double depthInside(
  const glm::dvec3& point,
  const glm::dvec3& centre,
  const glm::dvec3& half
) {
  const glm::dvec3 room = half - glm::abs(point - centre);

  return std::max(0.0, std::min({room.x, room.y, room.z}));
}

void run(World& world, uint32_t steps) {
  for (uint32_t frame = 0u; frame != steps; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);
  }
}

bool sameBits(double a, double b) {
  return std::bit_cast<uint64_t>(a) == std::bit_cast<uint64_t>(b);
}

bool sameBits(const glm::dvec3& a, const glm::dvec3& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z);
}

bool sameBits(const glm::dquat& a, const glm::dquat& b) {
  return sameBits(a.x, b.x) && sameBits(a.y, b.y) && sameBits(a.z, b.z)
         && sameBits(a.w, b.w);
}

std::vector<CompoundChild> bench(uint32_t firstKey) {
  return {
    part(glm::dvec3(-0.06, 0.0, 0.0), glm::dvec3(0.06, 0.03, 0.05), firstKey),
    part(
      glm::dvec3(0.06, 0.0, 0.0),
      glm::dvec3(0.06, 0.03, 0.05),
      firstKey + 1u
    ),
    part(
      glm::dvec3(0.06, 0.06, 0.0),
      glm::dvec3(0.04, 0.03, 0.04),
      firstKey + 2u
    )
  };
}

struct Pile {
  World world;
  uint32_t fieldToCompound = 0u;
  uint32_t compoundToField = 0u;
};

void pileUp(Pile& pile, uint32_t workers, uint32_t width) {
  World& world = pile.world;

  world.workerCount = workers;
  world.simdWidth = width;
  (void)addFloor(world);

  for (uint32_t index = 0u; index != 40u; ++index) {
    const uint32_t layer = index / 20u;
    const uint32_t slot = index % 20u;
    const glm::dvec3 position(
      -0.45 + 0.3 * static_cast<double>(slot % 4u)
        + 0.02 * static_cast<double>(layer),
      0.15 + 0.7 * static_cast<double>(layer)
        + 0.01 * static_cast<double>(slot),
      -0.6 + 0.3 * static_cast<double>(slot / 4u)
    );
    const glm::dquat turn = glm::angleAxis(
      0.37 * static_cast<double>(index),
      glm::normalize(glm::dvec3(0.3, 1.0, 0.2 * static_cast<double>(slot % 3u)))
    );

    if (index % 2u == 0u) {
      (void)addFieldBody(world, ballField(), ballMass, position, turn, 0.5);
    } else {
      (void)addFieldBody(world, peanutField(), 1.0, position, turn, 0.5);
    }
  }

  for (uint32_t index = 0u; index != 10u; ++index) {
    (void)addCompoundBody(
      world.bodies,
      bench(3u * index),
      glm::dvec3(
        -0.4 + 0.27 * static_cast<double>(index % 4u),
        0.5 + 0.02 * static_cast<double>(index),
        -0.45 + 0.3 * static_cast<double>(index / 4u)
      ),
      glm::angleAxis(
        0.5 * static_cast<double>(index),
        glm::dvec3(0.0, 1.0, 0.0)
      ),
      glm::dvec3(0.0),
      0.5,
      0.0,
      false
    );
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);

    for (const Contact& contact : contactRecords(world)) {
      if (contact.a.side != Side::Rigid || contact.b.side != Side::Rigid) {
        continue;
      }

      const RigidBody& a = world.bodies.rigids[contact.a.body];
      const RigidBody& b = world.bodies.rigids[contact.b.body];

      if (!(a.mass > 0.0 && b.mass > 0.0)) {
        continue;
      }

      if (a.shape == RigidShape::Sampled && b.shape == RigidShape::Compound) {
        pile.fieldToCompound += 1u;
      }

      if (a.shape == RigidShape::Compound && b.shape == RigidShape::Sampled) {
        pile.compoundToField += 1u;
      }
    }
  }
}

}

TEST_CASE(
  "a field body rests on the middle of a large box face",
  "[dynamics][fieldcompound]"
) {
  World world;

  (void)addFloor(world);

  const uint32_t ball = addBall(world, glm::dvec3(0.7, 0.3, 0.4), 0.5);

  run(world, 120u);

  const RigidBody& body = world.bodies.rigids[ball];
  const double lowest = lowestPoint(body);
  const double speed = glm::length(body.velocity);

  CAPTURE(lowest, speed);
  CHECK(std::abs(lowest) <= 0.002);
  CHECK(speed < 0.01);
}

TEST_CASE(
  "a field body rests across a stair edge",
  "[dynamics][fieldcompound]"
) {
  const glm::dvec3 floorCentre(0.0, -0.5, 0.0);
  const glm::dvec3 floorHalf(2.0, 0.5, 2.0);
  const glm::dvec3 stepCentre(0.5, 0.075, 0.0);
  const glm::dvec3 stepHalf(0.5, 0.075, 0.5);
  World world;

  (void)addAnchoredBox(world, floorCentre, floorHalf);
  (void)addAnchoredBox(world, stepCentre, stepHalf);

  const uint32_t ball = addBall(world, glm::dvec3(0.03, 0.35, 0.0), 0.5);

  run(world, 120u);

  const RigidBody& body = world.bodies.rigids[ball];
  double deepest = 0.0;

  for (const glm::dvec3& point : worldSamples(body)) {
    deepest = std::max(
      {deepest,
       depthInside(point, floorCentre, floorHalf),
       depthInside(point, stepCentre, stepHalf)}
    );
  }

  CAPTURE(body.position.x, body.position.y, deepest);
  CHECK(deepest <= 0.002);
  CHECK(std::abs(body.position.y - (0.15 + ballRadius)) <= 0.003);
  CHECK(body.position.x > 0.0);
  CHECK(glm::length(body.velocity) < 0.01);
}

TEST_CASE(
  "a field body slides along a box face without bumps",
  "[dynamics][fieldcompound]"
) {
  World world;

  (void)addFloor(world);

  const uint32_t ball =
    addBall(world, glm::dvec3(-0.5, ballRadius + 0.01, 0.0), 0.0);

  run(world, 60u);
  REQUIRE(lowestPoint(world.bodies.rigids[ball]) > -0.002);
  REQUIRE(
    setVelocity(world, world.bodies.rigids[ball].id, glm::dvec3(1.0, 0.0, 0.0))
  );

  const double start = world.bodies.rigids[ball].position.x;
  double low = std::numeric_limits<double>::infinity();
  double high = -std::numeric_limits<double>::infinity();

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);

    const double height = world.bodies.rigids[ball].position.y;

    low = std::min(low, height);
    high = std::max(high, height);
  }

  const double travelled = world.bodies.rigids[ball].position.x - start;

  CAPTURE(low, high, travelled);
  CHECK(high - low < 0.001);
  CHECK(travelled > 0.9);
}

TEST_CASE(
  "a field body rolling along a box face stays on it",
  "[dynamics][fieldcompound]"
) {
  World world;

  (void)addFloor(world);

  const uint32_t ball =
    addBall(world, glm::dvec3(-0.5, ballRadius + 0.01, 0.0), 0.5);

  run(world, 60u);
  REQUIRE(
    setVelocity(world, world.bodies.rigids[ball].id, glm::dvec3(1.0, 0.0, 0.0))
  );

  double deepest = 0.0;
  double lowest = std::numeric_limits<double>::infinity();
  double highest = -std::numeric_limits<double>::infinity();
  double speed = 0.0;
  double spin = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE_FALSE(step(world).nonFinite);

    const RigidBody& body = world.bodies.rigids[ball];

    deepest = std::min(deepest, lowestPoint(body));
    lowest = std::min(lowest, body.position.y);
    highest = std::max(highest, body.position.y);

    if (frame == 9u) {
      speed = glm::length(body.velocity);
      spin = glm::length(body.angularVelocity) * ballRadius;
    }
  }

  CAPTURE(deepest, lowest, highest, speed, spin);
  CHECK(deepest >= -0.002);
  CHECK(lowest >= ballRadius - 0.004);
  CHECK(highest <= ballRadius + 0.002);
  CHECK(speed > 0.3);
  CHECK(std::abs(spin - speed) <= 0.2 * speed);
}

TEST_CASE(
  "a box edge resting on a field body is held at its edge points",
  "[dynamics][fieldcompound]"
) {
  const glm::vec3 slabHalf(0.5f, 0.05f, 0.5f);
  const talus::field::SampledSdf slab = baked(
    {fixture::box(glm::vec3(0.0f), slabHalf)},
    -slabHalf - glm::vec3(0.03f),
    slabHalf + glm::vec3(0.03f),
    0.01f
  );
  const glm::dvec3 barHalf(0.15, 0.02, 0.02);
  World world;

  (void)addFloor(world);

  const RigidBody upright = rigidFromField(
    slab,
    {},
    std::vector<talus::PhysicalMaterial>(1),
    sampleBudget
  );
  const uint32_t slabIndex = addFieldBody(
    world,
    slab,
    500.0,
    glm::dvec3(0.0, 0.05, 0.0) + upright.fieldOrigin,
    upright.fieldRotation,
    0.5
  );
  const uint32_t barIndex = addCompoundBody(
    world.bodies,
    std::vector<CompoundChild>{part(glm::dvec3(0.0), barHalf, 0u)},
    glm::dvec3(0.03, 0.1 + barHalf.y * std::numbers::sqrt2 + 0.002, 0.012),
    glm::angleAxis(std::numbers::pi / 4.0, glm::dvec3(1.0, 0.0, 0.0)),
    glm::dvec3(0.0),
    0.5,
    0.0,
    false
  );

  run(world, 120u);

  const RigidBody& support = world.bodies.rigids[slabIndex];
  const glm::dquat turn =
    support.rotation * glm::conjugate(support.fieldRotation);
  const glm::dvec3 up = turn * glm::dvec3(0.0, 1.0, 0.0);
  const glm::dvec3 top =
    support.position
    + turn * (glm::dvec3(0.0, slabHalf.y, 0.0) - support.fieldOrigin);
  const RigidBody& bar = world.bodies.rigids[barIndex];
  double depth = -std::numeric_limits<double>::infinity();

  for (const glm::dvec3& sample :
       childSamples(bar.compound->children.front())) {
    depth = std::max(
      depth,
      glm::dot(top - (bar.position + bar.rotation * sample), up)
    );
  }

  const glm::dquat barTurn =
    bar.rotation * bar.compound->children.front().rotation;
  const double sideY =
    std::abs(glm::dot(barTurn * glm::dvec3(0.0, 1.0, 0.0), up));
  const double sideZ =
    std::abs(glm::dot(barTurn * glm::dvec3(0.0, 0.0, 1.0), up));

  CAPTURE(depth, sideY, sideZ, glm::length(bar.velocity));
  CHECK(depth <= 0.002);
  CHECK(sideY > 0.6);
  CHECK(sideZ > 0.6);
  CHECK(glm::length(bar.velocity) < 0.01);
}

TEST_CASE(
  "field-compound contacts are identical across workers and widths",
  "[dynamics][fieldcompound]"
) {
  Pile serial;

  pileUp(serial, 1u, 1u);
  CAPTURE(serial.fieldToCompound, serial.compoundToField);
  REQUIRE(serial.fieldToCompound > 0u);
  REQUIRE(serial.compoundToField > 0u);

  std::vector<uint32_t> widths{1u, 4u};

  if (hasAvx2()) {
    widths.push_back(8u);
  }

  for (const uint32_t workers : {1u, 4u, 8u}) {
    for (const uint32_t width : widths) {
      CAPTURE(workers, width);
      Pile pile;

      pileUp(pile, workers, width);
      CHECK(pile.fieldToCompound == serial.fieldToCompound);
      CHECK(pile.compoundToField == serial.compoundToField);
      REQUIRE(
        pile.world.bodies.rigids.size() == serial.world.bodies.rigids.size()
      );

      for (size_t index = 0u; index != serial.world.bodies.rigids.size();
           ++index) {
        const RigidBody& a = serial.world.bodies.rigids[index];
        const RigidBody& b = pile.world.bodies.rigids[index];

        CAPTURE(index);
        REQUIRE(a.asleep == b.asleep);
        REQUIRE(sameBits(a.sleepTime, b.sleepTime));
        REQUIRE(sameBits(a.position, b.position));
        REQUIRE(sameBits(a.rotation, b.rotation));
        REQUIRE(sameBits(a.velocity, b.velocity));
        REQUIRE(sameBits(a.angularVelocity, b.angularVelocity));
      }
    }
  }
}
