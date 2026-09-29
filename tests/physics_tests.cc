#include <catch2/catch_test_macros.hpp>

#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/physics/layers/layers.h>
#include <talus/physics/physicsworld/physicsworld.h>
#include <talus/objectmodel/world/world.h>
#include <talus/runtime/runtime/runtime.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <cstdint>
#include <cstring>
#include <vector>

using talus::physics::JoltLibrary;
using talus::physics::PhysicsWorld;

TEST_CASE("the jolt library registers and unregisters", "[physics]") {
  JoltLibrary library;

  SUCCEED();
}

TEST_CASE("a box shape can be created after registration", "[physics]") {
  JoltLibrary library;

  JPH::BoxShapeSettings settings(JPH::Vec3(10.0f, 0.5f, 10.0f));
  settings.SetEmbedded();

  const JPH::ShapeSettings::ShapeResult result = settings.Create();

  REQUIRE_FALSE(result.HasError());
}

TEST_CASE("a physics world initialises", "[physics]") {
  JoltLibrary library;
  PhysicsWorld world;

  SUCCEED();
}

TEST_CASE(
  "jolt memory is counted for a physics world's lifetime",
  "[physics]"
) {
  JoltLibrary library;

  const talus::physics::JoltMemory before = talus::physics::joltMemory();

  const int64_t held = [] {
    PhysicsWorld world;

    return talus::physics::joltMemory().liveBytes;
  }();

  const talus::physics::JoltMemory after = talus::physics::joltMemory();

  REQUIRE(held > before.liveBytes);
  REQUIRE(after.peakBytes >= held);
  REQUIRE(after.liveBytes == before.liveBytes);
}

TEST_CASE("jolt memory counts aligned and reallocated blocks", "[physics]") {
  constexpr size_t SIZE = 4096u;
  constexpr size_t GROWN = 3u * SIZE;
  constexpr size_t ALIGNMENT = 64u;
  constexpr unsigned char MARK = 7u;

  JoltLibrary library;

  const int64_t before = talus::physics::joltMemory().liveBytes;

  void* aligned = JPH::AlignedAllocate(SIZE, ALIGNMENT);

  REQUIRE(aligned != nullptr);
  REQUIRE(reinterpret_cast<uintptr_t>(aligned) % ALIGNMENT == 0u);
  REQUIRE(
    talus::physics::joltMemory().liveBytes
    == before + static_cast<int64_t>(SIZE)
  );

  void* plain = JPH::Allocate(SIZE);

  REQUIRE(plain != nullptr);

  std::memset(plain, MARK, SIZE);

  void* grown = JPH::Reallocate(plain, SIZE, GROWN);

  REQUIRE(grown != nullptr);
  REQUIRE(static_cast<unsigned char*>(grown)[SIZE - 1u] == MARK);
  REQUIRE(
    talus::physics::joltMemory().liveBytes
    == before + static_cast<int64_t>(SIZE + GROWN)
  );

  JPH::Free(grown);
  JPH::AlignedFree(aligned);

  REQUIRE(talus::physics::joltMemory().liveBytes == before);
}

TEST_CASE("a sphere falls onto a floor and comes to rest", "[physics]") {
  JoltLibrary library;
  PhysicsWorld world;

  JPH::BodyInterface& bodies = world.bodies();

  JPH::BoxShapeSettings floorSettings(JPH::Vec3(10.0f, 0.5f, 10.0f));
  floorSettings.SetEmbedded();

  const JPH::ShapeSettings::ShapeResult floorShape = floorSettings.Create();
  REQUIRE_FALSE(floorShape.HasError());

  const JPH::BodyID floorId = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      floorShape.Get(),
      JPH::RVec3(0.0f, -0.5f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );
  REQUIRE_FALSE(floorId.IsInvalid());

  const JPH::BodyID sphereId = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      new JPH::SphereShape(0.5f),
      JPH::RVec3(0.0f, 5.0f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Dynamic,
      talus::physics::ObjectLayers::MOVING
    ),
    JPH::EActivation::Activate
  );
  REQUIRE_FALSE(sphereId.IsInvalid());

  world.optimizeBroadPhase();

  for (int i = 0; i != 120; ++i) {
    world.step();
  }

  const float restingHeight = bodies.GetCenterOfMassPosition(sphereId).GetY();

  REQUIRE(restingHeight > 0.4f);
  REQUIRE(restingHeight < 0.6f);

  bodies.RemoveBody(sphereId);
  bodies.DestroyBody(sphereId);
  bodies.RemoveBody(floorId);
  bodies.DestroyBody(floorId);
}

namespace {

std::vector<float> dropTrajectory(int steps) {
  JoltLibrary library;
  PhysicsWorld world;

  JPH::BodyInterface& bodies = world.bodies();

  JPH::BoxShapeSettings floorSettings(JPH::Vec3(10.0f, 0.5f, 10.0f));
  floorSettings.SetEmbedded();

  const JPH::BodyID floorId = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      floorSettings.Create().Get(),
      JPH::RVec3(0.0f, -0.5f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Static,
      talus::physics::ObjectLayers::NON_MOVING
    ),
    JPH::EActivation::DontActivate
  );

  const JPH::BodyID sphereId = bodies.CreateAndAddBody(
    JPH::BodyCreationSettings(
      new JPH::SphereShape(0.5f),
      JPH::RVec3(0.0f, 5.0f, 0.0f),
      JPH::Quat::sIdentity(),
      JPH::EMotionType::Dynamic,
      talus::physics::ObjectLayers::MOVING
    ),
    JPH::EActivation::Activate
  );

  world.optimizeBroadPhase();

  std::vector<float> heights;
  heights.reserve(static_cast<size_t>(steps));

  for (int i = 0; i != steps; ++i) {
    world.step();
    heights.push_back(bodies.GetCenterOfMassPosition(sphereId).GetY());
  }

  bodies.RemoveBody(sphereId);
  bodies.DestroyBody(sphereId);
  bodies.RemoveBody(floorId);
  bodies.DestroyBody(floorId);

  return heights;
}

}

TEST_CASE(
  "the same initial conditions reproduce the same trajectory",
  "[physics]"
) {
  const std::vector<float> first = dropTrajectory(120);
  const std::vector<float> second = dropTrajectory(120);

  REQUIRE(first.size() == second.size());

  for (size_t i = 0; i != first.size(); ++i) {
    INFO("step " << i);
    REQUIRE(first[i] == second[i]);
  }
}

TEST_CASE("a runtime can be constructed", "[runtime]") {
  JoltLibrary library;
  talus::objectmodel::World source;

  talus::runtime::Runtime runtime(source, library);

  SUCCEED();
}
