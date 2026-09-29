#include <catch2/catch_test_macros.hpp>

#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/impact/impact.h>
#include <talus/physics/impact/impact.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/physics/layers/layers.h>
#include <talus/physics/physicsworld/physicsworld.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using talus::objectmodel::Entity;
using talus::objectmodel::Impact;
using talus::physics::JoltLibrary;
using talus::physics::PhysicsWorld;

namespace {

constexpr Entity FLOOR_ENTITY{3u};
constexpr Entity SPHERE_ENTITY{7u};

struct Drop {
  JPH::BodyID floor;
  JPH::BodyID sphere;
  std::vector<Impact> impacts;
};

void addFloor(PhysicsWorld& world, Drop& out, JPH::uint64 material = 0u) {
  JPH::BoxShapeSettings settings(JPH::Vec3(10.0f, 0.5f, 10.0f));
  settings.SetEmbedded();
  settings.mUserData = material;

  const JPH::ShapeSettings::ShapeResult shape = settings.Create();
  REQUIRE_FALSE(shape.HasError());

  JPH::BodyCreationSettings body(
    shape.Get(),
    JPH::RVec3(0.0f, -0.5f, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Static,
    talus::physics::ObjectLayers::NON_MOVING
  );

  body.mUserData = static_cast<uint32_t>(FLOOR_ENTITY);

  out.floor =
    world.bodies().CreateAndAddBody(body, JPH::EActivation::DontActivate);

  REQUIRE_FALSE(out.floor.IsInvalid());
}

void addSphere(
  PhysicsWorld& world,
  Drop& out,
  float height,
  float speed,
  JPH::uint64 material
) {
  JPH::SphereShape* ball = new JPH::SphereShape(0.5f);
  ball->SetUserData(material);

  JPH::BodyCreationSettings settings(
    ball,
    JPH::RVec3(0.0f, height, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  settings.mLinearVelocity = JPH::Vec3(0.0f, -speed, 0.0f);
  settings.mUserData = static_cast<uint32_t>(SPHERE_ENTITY);

  out.sphere =
    world.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);

  REQUIRE_FALSE(out.sphere.IsInvalid());
}

void run(
  PhysicsWorld& world,
  Drop& out,
  float height,
  float speed,
  int steps,
  JPH::uint64 material = 0u
) {
  addSphere(world, out, height, speed, material);

  world.optimizeBroadPhase();

  std::vector<Impact> batch;

  for (int i = 0; i != steps; ++i) {
    world.step();

    world.impacts().drain(batch);
    out.impacts.insert(out.impacts.end(), batch.begin(), batch.end());
  }
}

float highestAfterFirstImpact(
  PhysicsWorld& world,
  Drop& out,
  JPH::uint64 material,
  int steps
) {
  addSphere(world, out, 3.0f, 0.0f, material);

  world.optimizeBroadPhase();

  std::vector<Impact> batch;

  float highest = std::numeric_limits<float>::lowest();
  bool landed = false;

  for (int i = 0; i != steps; ++i) {
    world.step();

    world.impacts().drain(batch);
    out.impacts.insert(out.impacts.end(), batch.begin(), batch.end());

    landed = landed || !batch.empty();

    if (landed) {
      highest = std::max(
        highest,
        world.bodies().GetCenterOfMassPosition(out.sphere).GetY()
      );
    }
  }

  return highest;
}

float slide(PhysicsWorld& world, Drop& out, JPH::uint64 material, int steps) {
  JPH::BoxShapeSettings shape(JPH::Vec3(0.25f, 0.25f, 0.25f));
  shape.SetEmbedded();
  shape.mUserData = material;

  const JPH::ShapeSettings::ShapeResult created = shape.Create();
  REQUIRE_FALSE(created.HasError());

  JPH::BodyCreationSettings settings(
    created.Get(),
    JPH::RVec3(0.0f, 0.25f, 0.0f),
    JPH::Quat::sIdentity(),
    JPH::EMotionType::Dynamic,
    talus::physics::ObjectLayers::MOVING
  );

  settings.mLinearVelocity = JPH::Vec3(5.0f, 0.0f, 0.0f);
  settings.mUserData = static_cast<uint32_t>(SPHERE_ENTITY);

  out.sphere =
    world.bodies().CreateAndAddBody(settings, JPH::EActivation::Activate);

  REQUIRE_FALSE(out.sphere.IsInvalid());

  world.optimizeBroadPhase();

  for (int i = 0; i != steps; ++i) {
    world.step();
  }

  return world.bodies().GetCenterOfMassPosition(out.sphere).GetX();
}

void destroy(PhysicsWorld& world, const Drop& drop) {
  JPH::BodyInterface& bodies = world.bodies();

  bodies.RemoveBody(drop.sphere);
  bodies.DestroyBody(drop.sphere);
  bodies.RemoveBody(drop.floor);
  bodies.DestroyBody(drop.floor);
}

}

TEST_CASE("a hard landing is recorded for both bodies", "[impact]") {
  JoltLibrary library;
  PhysicsWorld world;

  Drop drop;
  addFloor(world, drop);
  run(world, drop, 3.0f, 15.0f, 60);

  REQUIRE_FALSE(drop.impacts.empty());

  REQUIRE(drop.impacts.size() % 2u == 0u);

  const Impact& first = drop.impacts[0];
  const Impact& second = drop.impacts[1];

  REQUIRE(first.speed > 10.0f);
  REQUIRE(second.speed == first.speed);
  REQUIRE(first.impulse > 0.0f);

  REQUIRE(std::abs(first.outward.y + second.outward.y) < 1e-5f);
  REQUIRE(std::abs(first.outward.y) > 0.9f);

  REQUIRE(std::abs(first.point.y) < 0.1f);

  destroy(world, drop);
}

TEST_CASE("each record names the body it hit", "[impact]") {
  JoltLibrary library;
  PhysicsWorld world;

  Drop drop;
  addFloor(world, drop);
  run(world, drop, 3.0f, 15.0f, 60);

  REQUIRE(drop.impacts.size() >= 2u);

  const Impact& first = drop.impacts[0];
  const Impact& second = drop.impacts[1];

  REQUIRE(first.entity != second.entity);
  REQUIRE(first.other == second.entity);
  REQUIRE(second.other == first.entity);

  const bool floorFirst =
    first.entity == FLOOR_ENTITY && second.entity == SPHERE_ENTITY;
  const bool sphereFirst =
    first.entity == SPHERE_ENTITY && second.entity == FLOOR_ENTITY;

  REQUIRE((floorFirst || sphereFirst));

  destroy(world, drop);
}

TEST_CASE("a body settling under gravity records nothing", "[impact]") {
  JoltLibrary library;
  PhysicsWorld world;

  Drop drop;
  addFloor(world, drop);

  run(world, drop, 0.505f, 0.0f, 120);

  REQUIRE(drop.impacts.empty());

  const float resting =
    world.bodies().GetCenterOfMassPosition(drop.sphere).GetY();

  REQUIRE(resting > 0.4f);
  REQUIRE(resting < 0.6f);

  destroy(world, drop);
}

TEST_CASE("the threshold is what decides, and it moves", "[impact]") {
  JoltLibrary library;
  PhysicsWorld world;

  world.impacts().setThreshold(500.0f);

  REQUIRE(world.impacts().threshold() == 500.0f);

  Drop drop;
  addFloor(world, drop);
  run(world, drop, 3.0f, 15.0f, 60);

  REQUIRE(drop.impacts.empty());

  destroy(world, drop);
}

TEST_CASE(
  "impacts sort strongest first, then by entity, then by the other body",
  "[impact]"
) {
  Impact a{};
  a.entity = Entity{5u};
  a.other = Entity{9u};
  a.impulse = 2.0f;

  Impact b{};
  b.entity = Entity{5u};
  b.other = Entity{4u};
  b.impulse = 2.0f;

  Impact c{};
  c.entity = Entity{1u};
  c.other = Entity{9u};
  c.impulse = 2.0f;

  Impact d{};
  d.entity = Entity{7u};
  d.other = Entity{0u};
  d.impulse = 9.0f;

  std::vector<Impact> impacts{a, b, c, d};

  talus::objectmodel::sortStrongestFirst(impacts);

  REQUIRE(impacts[0].impulse == 9.0f);
  REQUIRE(impacts[1].entity == Entity{1u});
  REQUIRE(impacts[2].other == Entity{4u});
  REQUIRE(impacts[3].other == Entity{9u});
}

TEST_CASE(
  "each side of a contact names its material on each side and the impulse",
  "[impact]"
) {
  JoltLibrary library;

  const std::vector<talus::objectmodel::PhysicalMaterial> table(6);

  PhysicsWorld world;

  world.impacts().setMaterials(table);

  Drop drop;
  addFloor(world, drop, 2u);
  run(world, drop, 3.0f, 15.0f, 60, 5u);

  REQUIRE(drop.impacts.size() >= 2u);

  const bool floorFirst = drop.impacts[0].entity == FLOOR_ENTITY;

  const Impact& floorSide = floorFirst ? drop.impacts[0] : drop.impacts[1];
  const Impact& sphereSide = floorFirst ? drop.impacts[1] : drop.impacts[0];

  REQUIRE(floorSide.entity == FLOOR_ENTITY);
  REQUIRE(sphereSide.entity == SPHERE_ENTITY);

  REQUIRE(floorSide.material == 2u);
  REQUIRE(floorSide.otherMaterial == 5u);

  REQUIRE(sphereSide.material == 5u);
  REQUIRE(sphereSide.otherMaterial == 2u);

  REQUIRE(floorSide.impulse > 0.0f);
  REQUIRE(sphereSide.impulse == floorSide.impulse);

  destroy(world, drop);
}

TEST_CASE(
  "a restitution of one bounces and a restitution of zero does not",
  "[impact]"
) {
  JoltLibrary library;

  std::vector<talus::objectmodel::PhysicalMaterial> table(2);
  table[1].restitution = 1.0f;

  float bouncy = 0.0f;
  float dead = 0.0f;

  float bouncyImpulse = 0.0f;
  float deadImpulse = 0.0f;

  {
    PhysicsWorld world;
    world.impacts().setMaterials(table);

    Drop drop;
    addFloor(world, drop, 0u);

    bouncy = highestAfterFirstImpact(world, drop, 1u, 180);

    REQUIRE_FALSE(drop.impacts.empty());

    bouncyImpulse = drop.impacts[0].impulse;

    destroy(world, drop);
  }

  {
    PhysicsWorld world;
    world.impacts().setMaterials(table);

    Drop drop;
    addFloor(world, drop, 0u);

    dead = highestAfterFirstImpact(world, drop, 0u, 180);

    REQUIRE_FALSE(drop.impacts.empty());

    deadImpulse = drop.impacts[0].impulse;

    destroy(world, drop);
  }

  REQUIRE(bouncy > 1.5f);
  REQUIRE(dead < 0.8f);

  REQUIRE(deadImpulse > 0.0f);
  REQUIRE(bouncyImpulse > 1.5f * deadImpulse);
}

TEST_CASE(
  "material friction holds a sliding contact after its first frame",
  "[impact]"
) {
  JoltLibrary library;

  std::vector<talus::objectmodel::PhysicalMaterial> table(2);
  table[1].friction = 0.0f;

  float gripped = 0.0f;
  float slippery = 0.0f;

  {
    PhysicsWorld world;
    world.impacts().setMaterials(table);

    Drop drop;
    addFloor(world, drop, 0u);

    gripped = slide(world, drop, 0u, 120);

    destroy(world, drop);
  }

  {
    PhysicsWorld world;
    world.impacts().setMaterials(table);

    Drop drop;
    addFloor(world, drop, 0u);

    slippery = slide(world, drop, 1u, 120);

    destroy(world, drop);
  }

  REQUIRE(gripped < 4.0f);
  REQUIRE(slippery > 8.0f);
}

TEST_CASE("a drained listener does not repeat itself", "[impact]") {
  JoltLibrary library;
  PhysicsWorld world;

  Drop drop;
  addFloor(world, drop);
  run(world, drop, 3.0f, 15.0f, 60);

  REQUIRE_FALSE(drop.impacts.empty());

  std::vector<Impact> again;
  world.impacts().drain(again);

  REQUIRE(again.empty());

  destroy(world, drop);
}
