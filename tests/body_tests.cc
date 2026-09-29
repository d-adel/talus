#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "fieldprimitives.h"

#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/name/name.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/support/support.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/query/query/query.h>
#include <talus/runtime/runtime/runtime.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/MotionType.h>

#include <glm/glm.hpp>

#include <vector>

using Catch::Matchers::ContainsSubstring;
using talus::objectmodel::Entity;
using talus::objectmodel::PhysicalMaterial;
using talus::objectmodel::Pose;
using talus::objectmodel::World;
using talus::physics::JoltLibrary;
using talus::runtime::Runtime;

namespace {

constexpr uint32_t RESOLUTION = 33u;

constexpr uint32_t ELASTIC_LANE = 1u;

void rebake(World& world, Entity entity) {
  const talus::field::Aabb box =
    talus::objectmodel::gridBox(world.primitives(entity));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      box.minimum,
      box.maximum,
      world.registry().get<talus::objectmodel::SdfGrid>(entity).resolution
    )
  );
}

Entity sphereEntity(World& world, const Pose& pose, float radius) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(RESOLUTION);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(entity, fixture::sphere(glm::vec3(0.0f), radius)));

  rebake(world, entity);

  return entity;
}

talus::field::Primitive lane(talus::field::Primitive primitive, uint32_t slot) {
  primitive.shapeConfig.y = slot;

  return primitive;
}

std::vector<PhysicalMaterial> elasticMaterials() {
  std::vector<PhysicalMaterial> materials(2);

  materials[ELASTIC_LANE].density = 500.0f;
  materials[ELASTIC_LANE].youngsModulus = 1.0e7f;
  materials[ELASTIC_LANE].poissonsRatio = 0.3f;

  return materials;
}

Entity barEntity(World& world, const Pose& pose, uint32_t slot) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(21u, 7u, 7u);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    lane(fixture::box(glm::vec3(0.0f), glm::vec3(0.5f, 0.15f, 0.15f)), slot)
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      glm::vec3(-0.6f, -0.3f, -0.3f),
      glm::vec3(0.6f, 0.3f, 0.3f),
      grid.resolution
    )
  );

  return entity;
}

Entity supportEntity(
  World& world,
  const glm::vec3& centre,
  const glm::vec3& halfExtents
) {
  const Entity entity = world.createAuthored();

  Pose pose{};
  pose.position = centre;

  world.emplaceAuthored<Pose>(entity, pose);

  world.emplaceAuthored<talus::objectmodel::Support>(
    entity,
    talus::objectmodel::Support{halfExtents}
  );

  return entity;
}

void name(World& world, Entity entity, std::string_view text) {
  world.emplaceAuthored<talus::objectmodel::Name>(
    entity,
    talus::objectmodel::Name{talus::objectmodel::Text::of(text)}
  );
}

JPH::EMotionType motionOf(Runtime& runtime, Entity entity) {
  const auto& body =
    runtime.world().registry().get<talus::objectmodel::BodyId>(entity);

  return runtime.physics().bodies().GetMotionType(JPH::BodyID(body.id));
}

float heightOf(Runtime& runtime, Entity entity) {
  return runtime.world().registry().get<Pose>(entity).position.y;
}

}

TEST_CASE("a body component is what admits an entity to physics", "[body]") {
  JoltLibrary library;

  World source;

  const Entity decoration =
    sphereEntity(source, Pose{glm::vec3(0.0f, 4.0f, 0.0f)}, 1.0f);

  const Entity mover = sphereEntity(source, Pose{}, 1.0f);

  source.addRigidBody(mover);

  Runtime runtime(source, library);

  for (const Entity entity : runtime.world().sdfGrids()) {
    runtime.refreshBody(entity);
  }

  const entt::registry& registry = runtime.world().registry();

  REQUIRE_FALSE(registry.all_of<talus::objectmodel::BodyId>(decoration));
  REQUIRE(runtime.world().deformablePrepared(decoration) == nullptr);
  REQUIRE(registry.all_of<talus::objectmodel::BodyId>(mover));

  const talus::query::Context context{runtime.world(), &runtime.physics()};

  REQUIRE_FALSE(
    talus::query::sweep(context, mover, glm::vec3(0.0f, 8.0f, 0.0f)).has_value()
  );

  REQUIRE(talus::query::overlap(context, mover).empty());
}

TEST_CASE("a body marked rigid falls under gravity", "[body]") {
  JoltLibrary library;

  World source;

  const Entity ball =
    sphereEntity(source, Pose{glm::vec3(0.0f, 5.0f, 0.0f)}, 1.0f);

  source.addRigidBody(ball);

  Runtime runtime(source, library);

  runtime.refreshBody(ball);

  REQUIRE(motionOf(runtime, ball) == JPH::EMotionType::Dynamic);

  const float began = heightOf(runtime, ball);

  for (uint32_t step = 0u; step != 30u; ++step) {
    runtime.beginFrame();
    runtime.step();
  }

  runtime.syncPoses(1.0f);

  REQUIRE(heightOf(runtime, ball) < began);
}

TEST_CASE("a body marked static holds its place", "[body]") {
  JoltLibrary library;

  World source;

  const Entity ground =
    sphereEntity(source, Pose{glm::vec3(0.0f, 5.0f, 0.0f)}, 1.0f);

  source.addStaticBody(ground);

  Runtime runtime(source, library);

  runtime.refreshBody(ground);

  REQUIRE(motionOf(runtime, ground) == JPH::EMotionType::Static);

  const float began = heightOf(runtime, ground);

  for (uint32_t step = 0u; step != 30u; ++step) {
    runtime.beginFrame();
    runtime.step();
  }

  runtime.syncPoses(1.0f);

  REQUIRE(heightOf(runtime, ground) == began);
}

TEST_CASE("a body marked deformable gets a prepared mesh alone", "[body]") {
  JoltLibrary library;

  World source;

  source.setMaterials(elasticMaterials());

  const Entity bar = barEntity(source, Pose{}, ELASTIC_LANE);

  source.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    bar,
    talus::objectmodel::DeformableBody{}
  );

  Runtime runtime(source, library);

  runtime.refreshBody(bar);

  REQUIRE(runtime.world().deformablePrepared(bar) != nullptr);
  REQUIRE(runtime.world().deformableState(bar) != nullptr);
  REQUIRE_FALSE(
    runtime.world().registry().all_of<talus::objectmodel::BodyId>(bar)
  );
}

TEST_CASE("a body is one kind or another, never two", "[body]") {
  JoltLibrary library;

  World source;

  const Entity twice = sphereEntity(source, Pose{}, 1.0f);

  name(source, twice, "confused crate");

  source.addRigidBody(twice);
  source.addStaticBody(twice);

  REQUIRE_THROWS_WITH(
    (Runtime{source, library}),
    ContainsSubstring("confused crate")
  );
}

TEST_CASE("a body of elastic material needs a Young's modulus", "[body]") {
  JoltLibrary library;

  World source;

  source.setMaterials(elasticMaterials());

  const Entity bar = barEntity(source, Pose{}, 0u);

  name(source, bar, "limp bar");

  source.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    bar,
    talus::objectmodel::DeformableBody{}
  );

  REQUIRE_THROWS_WITH(
    (Runtime{source, library}),
    ContainsSubstring("limp bar") && ContainsSubstring("lane 0")
  );
}

TEST_CASE("a support anchors the nodes its box covers", "[body]") {
  JoltLibrary library;

  World source;

  source.setMaterials(elasticMaterials());

  const glm::vec3 at(2.0f, 0.0f, 0.0f);

  const Entity bar = barEntity(source, Pose{at}, ELASTIC_LANE);

  source.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    bar,
    talus::objectmodel::DeformableBody{}
  );

  const glm::vec3 centre(1.6f, 0.0f, 0.0f);
  const glm::vec3 halfExtents(0.2f, 1.0f, 1.0f);

  REQUIRE(
    supportEntity(source, centre, halfExtents)
    != talus::objectmodel::INVALID_ENTITY
  );

  Runtime runtime(source, library);

  runtime.refreshBody(bar);

  const talus::objectmodel::PreparedMesh* mesh =
    runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  uint32_t anchored = 0u;
  uint32_t loose = 0u;

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    const glm::vec3 world = mesh->restPositions[node] + at;

    const bool inside =
      glm::all(glm::lessThanEqual(glm::abs(world - centre), halfExtents));

    INFO("node " << node << " rests at " << world.x);
    REQUIRE((mesh->anchored[node] != 0u) == inside);

    if (inside) {
      ++anchored;
    } else {
      ++loose;
    }
  }

  REQUIRE(anchored > 0u);
  REQUIRE(loose > 0u);
}
