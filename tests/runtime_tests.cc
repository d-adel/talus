#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "fieldprimitives.h"

#include <talus/core/config/config.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/massscale/massscale.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/staticbody/staticbody.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/runtime/runtime/runtime.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <limits>
#include <vector>

using Catch::Matchers::WithinRel;
using talus::objectmodel::Entity;
using talus::objectmodel::World;
using talus::physics::JoltLibrary;
using talus::runtime::Runtime;

namespace {

constexpr uint32_t RESOLUTION = 33u;

void rebake(World& world, Entity entity) {
  const talus::objectmodel::SdfGrid& grid =
    world.registry().get<talus::objectmodel::SdfGrid>(entity);

  const talus::field::Aabb box =
    talus::objectmodel::gridBox(world.primitives(entity));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      box.minimum,
      box.maximum,
      grid.resolution
    )
  );
}

Entity sphereAt(World& world, const glm::vec3& at, float radius) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(RESOLUTION);

  talus::objectmodel::Pose pose{};
  pose.position = at;

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(entity, fixture::sphere(glm::vec3(0.0f), radius)));
  rebake(world, entity);

  world.addRigidBody(entity);

  return entity;
}

Entity sphereEntity(World& world, float radius) {
  return sphereAt(world, glm::vec3(0.0f), radius);
}

float bodyMass(Runtime& runtime, Entity entity) {
  const auto& body =
    runtime.world().registry().get<talus::objectmodel::BodyId>(entity);

  JPH::BodyLockRead lock(
    runtime.physics().system().GetBodyLockInterface(),
    JPH::BodyID(body.id)
  );

  REQUIRE(lock.Succeeded());

  return 1.0f / lock.GetBody().GetMotionProperties()->GetInverseMass();
}

}

TEST_CASE("a mass override survives a rebake as a density", "[runtime]") {
  JoltLibrary library;

  World source;
  const Entity entity = sphereEntity(source, 1.0f);

  Runtime runtime(source, library);
  World& live = runtime.world();

  runtime.refreshBody(entity);

  const float derived = bodyMass(runtime, entity);

  REQUIRE(derived > 0.0f);

  runtime.setMass(entity, 150.0f);

  REQUIRE_THAT(bodyMass(runtime, entity), WithinRel(150.0f, 0.001f));

  REQUIRE(
    live.setPrimitive(entity, 0u, fixture::sphere(glm::vec3(0.0f), 0.5f))
  );
  rebake(live, entity);
  runtime.refreshBody(entity);

  REQUIRE_THAT(bodyMass(runtime, entity), WithinRel(150.0f / 8.0f, 0.1f));
}

TEST_CASE("a body nobody weighed follows its geometry", "[runtime]") {
  JoltLibrary library;

  World source;
  const Entity entity = sphereEntity(source, 1.0f);

  Runtime runtime(source, library);
  World& live = runtime.world();

  runtime.refreshBody(entity);

  const float derived = bodyMass(runtime, entity);

  REQUIRE_FALSE(live.registry().all_of<talus::objectmodel::MassScale>(entity));

  REQUIRE(
    live.setPrimitive(entity, 0u, fixture::sphere(glm::vec3(0.0f), 0.5f))
  );
  rebake(live, entity);
  runtime.refreshBody(entity);

  REQUIRE_THAT(bodyMass(runtime, entity), WithinRel(derived / 8.0f, 0.1f));
}

namespace {

talus::field::Primitive lane(talus::field::Primitive primitive, uint32_t of) {
  primitive.shapeConfig.y = of;

  return primitive;
}

void bakeEveryGrid(talus::objectmodel::World& world) {
  for (const talus::objectmodel::Entity entity : world.sdfGrids()) {
    const talus::objectmodel::SdfGrid& fitted =
      world.registry().get<talus::objectmodel::SdfGrid>(entity);
    const talus::field::Aabb box =
      talus::objectmodel::gridBox(world.primitives(entity));

    world.setSampledSdf(
      entity,
      talus::field::bake(
        world.primitives(entity),
        box.minimum,
        box.maximum,
        fitted.resolution
      )
    );
  }
}

void fillFieldWorld(talus::objectmodel::World& world) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(33u);

  talus::objectmodel::Pose floorPose{};
  floorPose.position = glm::vec3(0.0f, -1.0f, 0.0f);

  const talus::objectmodel::Entity floor = world.createSdfGrid(floorPose, grid);

  REQUIRE(world.addPrimitive(
    floor,
    fixture::box(glm::vec3(0.0f), glm::vec3(4.0f, 0.5f, 4.0f))
  ));
  world.addStaticBody(floor);

  talus::objectmodel::Pose ballPose{};
  ballPose.position = glm::vec3(0.13f, 1.7f, 0.07f);

  const talus::objectmodel::Entity ball = world.createSdfGrid(ballPose, grid);

  REQUIRE(
    world.addPrimitive(ball, lane(fixture::sphere(glm::vec3(0.0f), 0.4f), 2u))
  );

  world.addRigidBody(ball);

  bakeEveryGrid(world);
}

void fillTwoLaneWorld(talus::objectmodel::World& world) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(33u);

  talus::objectmodel::Pose floorPose{};
  floorPose.position = glm::vec3(3.0f, -1.0f, 0.0f);

  const talus::objectmodel::Entity floor = world.createSdfGrid(floorPose, grid);

  REQUIRE(world.addPrimitive(
    floor,
    lane(
      fixture::box(glm::vec3(-2.0f, 0.0f, 0.0f), glm::vec3(2.0f, 0.5f, 4.0f)),
      1u
    )
  ));

  REQUIRE(world.addPrimitive(
    floor,
    lane(
      fixture::box(glm::vec3(2.0f, 0.0f, 0.0f), glm::vec3(2.0f, 0.5f, 1.0f)),
      3u
    )
  ));

  world.addStaticBody(floor);

  talus::objectmodel::Pose blockPose{};
  blockPose.position = glm::vec3(2.4f, 1.7f, 0.0f);
  blockPose.rotation =
    glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f));

  const talus::objectmodel::Entity block = world.createSdfGrid(blockPose, grid);

  REQUIRE(world.addPrimitive(
    block,
    lane(
      fixture::box(glm::vec3(0.0f, -0.25f, 0.0f), glm::vec3(0.3f, 0.25f, 0.3f)),
      2u
    )
  ));

  REQUIRE(world.addPrimitive(
    block,
    lane(
      fixture::box(glm::vec3(0.0f, 0.15f, 0.0f), glm::vec3(0.3f, 0.15f, 0.3f)),
      4u
    )
  ));

  world.addRigidBody(block);

  bakeEveryGrid(world);
}

std::vector<float> fieldDrop(
  const talus::objectmodel::World& source,
  JoltLibrary& library,
  uint32_t steps
) {
  talus::runtime::Runtime runtime(source, library);

  for (const talus::objectmodel::Entity entity : runtime.world().sdfGrids()) {
    runtime.refreshBody(entity);
  }

  std::vector<float> heights;

  for (uint32_t step = 0u; step != steps; ++step) {
    runtime.beginFrame();
    runtime.step();
    runtime.syncPoses(1.0f);

    for (const talus::objectmodel::Entity entity : runtime.world().sdfGrids()) {
      heights.push_back(runtime.world()
                          .registry()
                          .get<talus::objectmodel::Pose>(entity)
                          .position.y);
    }
  }

  return heights;
}

}

TEST_CASE(
  "the same initial conditions reproduce the same trajectory over a field",
  "[physics]"
) {
  JoltLibrary library;

  talus::objectmodel::World source;
  fillFieldWorld(source);

  const std::vector<float> first = fieldDrop(source, library, 120u);
  const std::vector<float> second = fieldDrop(source, library, 120u);

  REQUIRE(first.size() == second.size());
  REQUIRE(first.size() == 240u);

  for (size_t i = 0; i != first.size(); ++i) {
    INFO("sample " << i);
    REQUIRE(first[i] == second[i]);
  }

  REQUIRE(first.back() < 1.0f);
  REQUIRE(first.back() > -1.0f);
}

TEST_CASE("a field with nothing solid in it gets no body", "[runtime]") {
  JoltLibrary library;

  World source;

  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(17u);

  const Entity entity = source.createSdfGrid(talus::objectmodel::Pose{}, grid);

  source.setSampledSdf(
    entity,
    talus::field::bake(
      std::vector<talus::field::Primitive>{
        fixture::sphere(glm::vec3(10.0f), 0.5f)
      },
      glm::vec3(-1.0f),
      glm::vec3(1.0f),
      grid.resolution
    )
  );

  source.addRigidBody(entity);

  Runtime runtime(source, library);

  runtime.refreshBody(entity);

  REQUIRE_FALSE(
    runtime.world().registry().all_of<talus::objectmodel::BodyId>(entity)
  );
}

namespace {

const JPH::Shape* bodyShape(Runtime& runtime, Entity entity) {
  const auto& body =
    runtime.world().registry().get<talus::objectmodel::BodyId>(entity);

  return runtime.physics().bodies().GetShape(JPH::BodyID(body.id));
}

}

TEST_CASE("objects on one lattice get one collision shape", "[runtime]") {
  JoltLibrary library;

  World source;

  const Entity first = sphereEntity(source, 1.0f);
  const Entity second = sphereEntity(source, 1.0f);

  source.shareSampledSdf(first, second);

  Runtime runtime(source, library);
  World& live = runtime.world();

  REQUIRE(live.sampledSdf(first) == live.sampledSdf(second));

  runtime.refreshBody(first);
  runtime.refreshBody(second);

  REQUIRE(bodyShape(runtime, first) == bodyShape(runtime, second));
  REQUIRE(runtime.shapeCount() == 1u);
}

TEST_CASE("a dent takes the object off the shared shape", "[runtime]") {
  JoltLibrary library;

  World source;

  const Entity first = sphereEntity(source, 1.0f);
  const Entity second = sphereEntity(source, 1.0f);

  source.shareSampledSdf(first, second);

  Runtime runtime(source, library);
  World& live = runtime.world();

  runtime.refreshBody(first);
  runtime.refreshBody(second);

  REQUIRE(bodyShape(runtime, first) == bodyShape(runtime, second));

  live.patchSampledSdf(second, [](talus::field::SampledSdf& lattice) {
    lattice.distances[0] = -1.0f;
  });

  REQUIRE(live.sampledSdf(first) != live.sampledSdf(second));

  runtime.refreshBody(second);

  REQUIRE(bodyShape(runtime, first) != bodyShape(runtime, second));
  REQUIRE(runtime.shapeCount() == 2u);
}

TEST_CASE("a rebuilt shape replaces the one the cache hands out", "[runtime]") {
  JoltLibrary library;

  World source;
  const Entity entity = sphereEntity(source, 1.0f);

  Runtime runtime(source, library);
  World& live = runtime.world();

  runtime.refreshBody(entity);

  REQUIRE(live.sampledSdfUsers(entity) == 2);

  live.patchSampledSdf(entity, [](talus::field::SampledSdf& grid) {
    grid.distances[0] = -1.0f;
  });

  runtime.refreshBody(entity);

  const talus::field::SampledSdf* lattice = live.sampledSdf(entity);

  REQUIRE(live.sampledSdfUsers(entity) == 1);

  live.patchSampledSdf(entity, [](talus::field::SampledSdf& grid) {
    grid.distances[1] = -1.0f;
  });

  REQUIRE(live.sampledSdf(entity) == lattice);

  const glm::uvec3 bricks = talus::field::brickCounts(*lattice);

  runtime.reshapeAfterDeform(entity, glm::uvec3(0u), bricks);

  const JPH::Shape* dented = bodyShape(runtime, entity);

  runtime.refreshBody(entity);

  REQUIRE(bodyShape(runtime, entity) == dented);
}

TEST_CASE(
  "impacts collected after a step wait to be read until the next collection",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : runtime.world().sdfGrids()) {
    runtime.refreshBody(entity);
  }

  REQUIRE(runtime.impacts().empty());

  bool landed = false;

  for (uint32_t step = 0u; step != 120u && !landed; ++step) {
    runtime.beginFrame();
    runtime.step();
    runtime.collectImpacts();

    landed = !runtime.impacts().empty();
  }

  REQUIRE(landed);

  const std::span<const talus::objectmodel::Impact> hits = runtime.impacts();

  REQUIRE(hits.size() % 2u == 0u);
  REQUIRE(hits[0].entity != hits[1].entity);
  REQUIRE(hits[0].other == hits[1].entity);
  REQUIRE(hits[1].other == hits[0].entity);
  REQUIRE(hits[0].speed > 3.0f);

  const auto grids = runtime.world().sdfGrids();

  REQUIRE(std::ranges::find(grids, hits[0].entity) != grids.end());
  REQUIRE(std::ranges::find(grids, hits[1].entity) != grids.end());

  runtime.collectImpacts();

  REQUIRE(runtime.impacts().empty());
}

namespace {

Entity fallingGrid(const World& world) {
  for (const Entity entity : world.sdfGrids()) {
    if (!world.registry().all_of<talus::objectmodel::StaticBody>(entity)) {
      return entity;
    }
  }

  return talus::objectmodel::INVALID_ENTITY;
}

float highestAfterFirstImpact(
  const World& source,
  JoltLibrary& library,
  uint32_t steps
) {
  Runtime runtime(source, library);

  for (const Entity entity : runtime.world().sdfGrids()) {
    runtime.refreshBody(entity);
  }

  const Entity ball = fallingGrid(runtime.world());

  REQUIRE(ball != talus::objectmodel::INVALID_ENTITY);

  float highest = std::numeric_limits<float>::lowest();
  bool landed = false;

  for (uint32_t step = 0u; step != steps; ++step) {
    runtime.beginFrame();
    runtime.step();
    runtime.syncPoses(1.0f);
    runtime.collectImpacts();

    landed = landed || !runtime.impacts().empty();

    if (landed) {
      highest = std::max(
        highest,
        runtime.world()
          .registry()
          .get<talus::objectmodel::Pose>(ball)
          .position.y
      );
    }
  }

  return highest;
}

}

TEST_CASE(
  "impacts over fields name the voxel material on each side",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillTwoLaneWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : runtime.world().sdfGrids()) {
    runtime.refreshBody(entity);
  }

  bool landed = false;

  for (uint32_t step = 0u; step != 120u && !landed; ++step) {
    runtime.beginFrame();
    runtime.step();
    runtime.collectImpacts();

    landed = !runtime.impacts().empty();
  }

  REQUIRE(landed);

  const auto hits = runtime.impacts();

  REQUIRE(hits.size() >= 2u);

  const bool floorFirst =
    runtime.world().registry().all_of<talus::objectmodel::StaticBody>(
      hits[0].entity
    );

  const talus::objectmodel::Impact& floorSide = floorFirst ? hits[0] : hits[1];
  const talus::objectmodel::Impact& blockSide = floorFirst ? hits[1] : hits[0];

  REQUIRE(floorSide.point.x > 2.0f);
  REQUIRE(floorSide.point.x < 2.8f);

  REQUIRE(floorSide.material == 1u);
  REQUIRE(floorSide.otherMaterial == 4u);

  REQUIRE(blockSide.material == 4u);
  REQUIRE(blockSide.otherMaterial == 1u);
}

TEST_CASE(
  "a restitution of one bounces through the runtime and zero does not",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  std::vector<talus::objectmodel::PhysicalMaterial> bouncy(3);
  bouncy[2].restitution = 1.0f;

  source.setMaterials(bouncy);

  const float rebound = highestAfterFirstImpact(source, library, 150u);

  source.setMaterials(std::vector<talus::objectmodel::PhysicalMaterial>(3));

  const float settled = highestAfterFirstImpact(source, library, 150u);

  REQUIRE(rebound > 0.6f);
  REQUIRE(settled < 0.2f);
}

TEST_CASE(
  "an edited pose teleports its body before the next step",
  "[runtime]"
) {
  JoltLibrary library;
  World source;
  const Entity entity = sphereEntity(source, 1.0f);
  Runtime runtime(source, library);
  runtime.refreshBody(entity);
  runtime.beginFrame();
  runtime.step();
  runtime.syncPoses(1.0f);
  const float fallen =
    runtime.world().registry().get<talus::objectmodel::Pose>(entity).position.y;
  REQUIRE(fallen <= 0.0f);
  runtime.world().patch<talus::objectmodel::Pose>(
    entity,
    [](talus::objectmodel::Pose& pose) {
      pose.position = glm::vec3(0.0f, 10.0f, 0.0f);
    }
  );
  runtime.applyPoseEdits();
  runtime.beginFrame();
  runtime.step();
  runtime.syncPoses(1.0f);
  const float after =
    runtime.world().registry().get<talus::objectmodel::Pose>(entity).position.y;
  REQUIRE(after > 9.0f);
  REQUIRE(after <= 10.0f);
  REQUIRE(runtime.world().takePoseEdits().empty());
}

namespace {

std::vector<Entity> gridsOf(const World& world) {
  return std::vector<Entity>(world.sdfGrids().begin(), world.sdfGrids().end());
}

JPH::BodyID bodyOf(Runtime& runtime, Entity entity) {
  return JPH::BodyID(
    runtime.world().registry().get<talus::objectmodel::BodyId>(entity).id
  );
}

std::vector<JPH::BodyID> bodiesIn(
  Runtime& runtime,
  const glm::vec3& minimum,
  const glm::vec3& maximum
) {
  JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> found;

  runtime.physics().system().GetBroadPhaseQuery().CollideAABox(
    JPH::AABox(
      JPH::Vec3(minimum.x, minimum.y, minimum.z),
      JPH::Vec3(maximum.x, maximum.y, maximum.z)
    ),
    found
  );

  return std::vector<JPH::BodyID>(found.mHits.begin(), found.mHits.end());
}

}

TEST_CASE(
  "a loaded world runs one broadphase optimize and its first step none",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : gridsOf(runtime.world())) {
    runtime.refreshBody(entity);
  }

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 0u);

  runtime.optimizeBroadPhase();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);
}

TEST_CASE(
  "a world loaded without an optimize runs one at its first step",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 0u);

  for (const Entity entity : gridsOf(runtime.world())) {
    runtime.refreshBody(entity);
  }

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);
}

TEST_CASE(
  "a body spawned during play adds no broadphase optimize",
  "[runtime]"
) {
  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : gridsOf(runtime.world())) {
    runtime.refreshBody(entity);
  }

  runtime.optimizeBroadPhase();

  runtime.beginFrame();
  runtime.step();

  const float height = 2.0f;

  const Entity spawned =
    sphereAt(runtime.world(), glm::vec3(-2.0f, height, 0.0f), 0.3f);

  runtime.refreshBody(spawned);

  const JPH::BodyID id = bodyOf(runtime, spawned);

  REQUIRE(runtime.physics().bodies().IsAdded(id));
  REQUIRE(runtime.physics().bodies().IsActive(id));

  for (uint32_t step = 0u; step != 30u; ++step) {
    runtime.beginFrame();
    runtime.step();
  }

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);
  REQUIRE(runtime.physics().bodies().GetPosition(id).GetY() < height);
}

TEST_CASE(
  "bodies spawned together enter the broadphase without an optimize",
  "[runtime]"
) {
  constexpr uint32_t SPAWNED = 4u;

  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : gridsOf(runtime.world())) {
    runtime.refreshBody(entity);
  }

  runtime.optimizeBroadPhase();

  runtime.beginFrame();
  runtime.step();

  const float height = 2.0f;
  const float row = 1.5f;

  std::vector<Entity> spawned;

  for (uint32_t index = 0u; index != SPAWNED; ++index) {
    spawned.push_back(sphereAt(
      runtime.world(),
      glm::vec3(-3.0f + 1.5f * static_cast<float>(index), height, row),
      0.3f
    ));
  }

  runtime.refreshBodies(spawned);

  const std::vector<JPH::BodyID> found = bodiesIn(
    runtime,
    glm::vec3(-4.0f, height - 0.5f, row - 0.5f),
    glm::vec3(4.0f, height + 0.5f, row + 0.5f)
  );

  REQUIRE(found.size() == SPAWNED);

  for (const Entity entity : spawned) {
    const JPH::BodyID id = bodyOf(runtime, entity);

    REQUIRE(runtime.physics().bodies().IsAdded(id));
    REQUIRE(runtime.physics().bodies().IsActive(id));
    REQUIRE(std::ranges::find(found, id) != found.end());
  }

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);

  for (const Entity entity : spawned) {
    REQUIRE(
      runtime.physics().bodies().GetPosition(bodyOf(runtime, entity)).GetY()
      < height
    );
  }
}

TEST_CASE(
  "bodies added past the reoptimize share run one optimize at the next step",
  "[runtime]"
) {
  constexpr uint32_t SHARE = 64u;
  constexpr uint32_t ROW = 8u;
  constexpr float PITCH = 0.9f;

  JoltLibrary library;

  World source;
  fillFieldWorld(source);

  Runtime runtime(source, library);

  for (const Entity entity : gridsOf(runtime.world())) {
    runtime.refreshBody(entity);
  }

  runtime.optimizeBroadPhase();

  runtime.beginFrame();
  runtime.step();

  World& live = runtime.world();

  const auto spot = [](uint32_t index) {
    return glm::vec3(
      -3.5f + PITCH * static_cast<float>(index % ROW),
      3.0f,
      -3.5f + PITCH * static_cast<float>(index / ROW)
    );
  };

  const Entity donor = sphereAt(live, spot(0u), 0.2f);

  std::vector<Entity> spawned{donor};

  for (uint32_t index = 1u; index != SHARE + 1u; ++index) {
    talus::objectmodel::SdfGrid grid{};
    grid.resolution = glm::uvec3(RESOLUTION);

    talus::objectmodel::Pose pose{};
    pose.position = spot(index);

    const Entity entity = live.createSdfGrid(pose, grid);

    live.shareSampledSdf(donor, entity);
    live.addRigidBody(entity);

    spawned.push_back(entity);
  }

  runtime.refreshBodies(std::span<const Entity>(spawned.data(), SHARE - 1u));

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 1u);

  runtime.refreshBody(spawned[SHARE - 1u]);

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 2u);

  runtime.refreshBody(spawned[SHARE]);

  runtime.beginFrame();
  runtime.step();

  REQUIRE(runtime.physics().broadPhaseOptimizations() == 2u);
}
