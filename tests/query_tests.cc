#include <talus/script/api/api.h>
#include <talus/systems/schedule/schedule.h>
#include <talus/query/query/query.h>

#include <talus/field/deform/deform.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/world/world.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/runtime/runtime/runtime.h>

#include "fieldprimitives.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
#include <stdexcept>

using Catch::Matchers::WithinAbs;
using talus::objectmodel::Entity;
using talus::objectmodel::Pose;
using talus::objectmodel::World;
using talus::query::raycast;

namespace {

Entity sphere(
  World& world,
  const Pose& pose = {},
  const glm::vec3& centre = glm::vec3(0.0f)
) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(33u);
  const Entity entity = world.createSdfGrid(pose, grid);
  REQUIRE(world.addPrimitive(entity, fixture::sphere(centre, 1.0f)));
  return entity;
}

void bake(World& world, Entity entity) {
  const auto bounds = talus::objectmodel::gridBox(world.primitives(entity));
  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      bounds.minimum,
      bounds.maximum,
      glm::uvec3(33u)
    )
  );
}

}

TEST_CASE("world rays return the nearest surface in world units", "[query]") {
  World world;
  const Entity near = sphere(world, Pose{glm::vec3(0.0f, 0.0f, 4.0f)});
  const Entity far = sphere(world, Pose{glm::vec3(0.0f, 0.0f, 8.0f)});
  REQUIRE(near != far);
  const talus::field::Ray ray{glm::vec3(0.0f), glm::vec3(0.0f, 0.0f, 3.0f)};
  const auto hit = raycast({world}, ray);
  REQUIRE(hit);
  REQUIRE(hit->entity == near);
  REQUIRE_THAT(hit->distance, WithinAbs(3.0f, 0.002f));
  REQUIRE_THAT(hit->position.z, WithinAbs(3.0f, 0.002f));
  REQUIRE_THAT(hit->normal.z, WithinAbs(-1.0f, 0.002f));
  REQUIRE(hit->material == 1u);
  REQUIRE_FALSE(raycast({world}, ray, 2.9f));
  REQUIRE_FALSE(raycast({world}, {glm::vec3(4.0f, 0.0f, 0.0f), ray.direction}));
}

TEST_CASE("world rays rotate normals and exit an enclosing solid", "[query]") {
  World world;
  Pose pose{glm::vec3(3.0f, 2.0f, 0.0f)};
  pose.rotation =
    glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  const Entity entity = sphere(world, pose, glm::vec3(0.0f, 2.0f, 0.0f));
  const auto hit = raycast(
    {world},
    {glm::vec3(1.0f, 2.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f)}
  );
  REQUIRE(hit);
  REQUIRE(hit->entity == entity);
  REQUIRE_THAT(hit->position.x, WithinAbs(2.0f, 0.003f));
  REQUIRE_THAT(hit->normal.x, WithinAbs(1.0f, 0.003f));
  REQUIRE_THAT(hit->distance, WithinAbs(1.0f, 0.003f));
}

TEST_CASE(
  "world rays follow the deformed lattice instead of its source primitives",
  "[query]"
) {
  World world;
  const Entity entity = sphere(world);
  bake(world, entity);
  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(entity, {});
  const talus::field::Ray ray{
    glm::vec3(0.0f, 0.0f, -3.0f),
    glm::vec3(0.0f, 0.0f, 1.0f)
  };
  REQUIRE(raycast({world}, ray));
  world.patchSampledSdf(entity, [](talus::field::SampledSdf& field) {
    talus::field::deformAndRepair(
      field,
      fixture::box(
        glm::vec3(0.0f),
        glm::vec3(0.4f, 0.4f, 2.0f),
        talus::core::FIELD_OP_SUBTRACTION
      )
    );
  });
  REQUIRE_FALSE(raycast({world}, ray));
  const auto edge =
    raycast({world}, {glm::vec3(0.8f, 0.0f, -3.0f), ray.direction});
  REQUIRE(edge);
  REQUIRE(edge->entity == entity);
  REQUIRE(edge->normal.z < -0.4f);
}

TEST_CASE(
  "world rays use stepped body poses before ECS synchronization",
  "[query]"
) {
  talus::physics::JoltLibrary library;
  World source;
  const Entity entity = sphere(source, {}, glm::vec3(0.0f, 2.0f, 0.0f));
  bake(source, entity);
  source.addRigidBody(entity);
  talus::runtime::Runtime runtime(source, library);
  runtime.refreshBody(entity);
  const talus::field::Ray ray{
    glm::vec3(5.0f, 2.0f, -3.0f),
    glm::vec3(0.0f, 0.0f, 1.0f)
  };
  runtime.setLinearVelocity(entity, glm::vec3(300.0f, 0.0f, 0.0f));
  runtime.step();
  const auto hit = raycast({runtime.world(), &runtime.physics()}, ray);
  REQUIRE(hit);
  REQUIRE(hit->entity == entity);
  REQUIRE_THAT(hit->distance, WithinAbs(2.0f, 0.03f));
  REQUIRE_FALSE(raycast({runtime.world()}, ray));
}

TEST_CASE(
  "world rays reject invalid inputs and break equal-distance ties by entity",
  "[query]"
) {
  World world;
  const Entity first = sphere(world);
  const Entity second = sphere(world);
  const talus::field::Ray ray{
    glm::vec3(0.0f, 0.0f, -3.0f),
    glm::vec3(0.0f, 0.0f, 1.0f)
  };
  const auto hit = raycast({world}, ray);
  REQUIRE(hit);
  REQUIRE(hit->entity == first);
  REQUIRE(first != second);
  REQUIRE_THROWS_AS(
    raycast({world}, {ray.origin, glm::vec3(0.0f)}),
    std::invalid_argument
  );
  REQUIRE_THROWS_AS(raycast({world}, ray, -1.0f), std::invalid_argument);
  REQUIRE_THROWS_AS(
    raycast({world}, ray, std::numeric_limits<float>::quiet_NaN()),
    std::invalid_argument
  );
  const auto unbounded =
    raycast({world}, ray, std::numeric_limits<float>::infinity());
  REQUIRE(unbounded);
  REQUIRE(unbounded->entity == first);
}

TEST_CASE(
  "a body sweep reports the first field contact in world space",
  "[query][sweep]"
) {
  talus::physics::JoltLibrary library;
  World source;
  Pose movingPose{glm::vec3(3.0f, 0.0f, 0.0f)};
  Pose fixedPose{glm::vec3(3.0f, 5.0f, 0.0f)};
  fixedPose.rotation =
    glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  const Entity moving = sphere(source, movingPose, glm::vec3(0.0f, 2.0f, 0.0f));
  const Entity fixed = sphere(source, fixedPose);
  REQUIRE(source.setPrimitive(
    moving,
    0u,
    fixture::box(glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(1.0f))
  ));
  REQUIRE(source.setPrimitive(
    fixed,
    0u,
    fixture::box(glm::vec3(0.0f), glm::vec3(1.0f))
  ));
  bake(source, moving);
  bake(source, fixed);
  source.addRigidBody(moving);
  source.addStaticBody(fixed);
  talus::runtime::Runtime runtime(source, library);
  runtime.refreshBody(moving);
  runtime.refreshBody(fixed);
  const talus::query::Context context{runtime.world(), &runtime.physics()};
  const auto hit =
    talus::query::sweep(context, moving, glm::vec3(0.0f, 4.0f, 0.0f));
  REQUIRE(hit);
  REQUIRE(hit->entity == fixed);
  REQUIRE_THAT(hit->distance, WithinAbs(1.0f, 0.04f));
  REQUIRE(hit->position.x >= 2.0f);
  REQUIRE(hit->position.x <= 4.0f);
  REQUIRE_THAT(hit->position.y, WithinAbs(4.0f, 0.08f));
  REQUIRE(hit->normal.y < -0.95f);
  REQUIRE(hit->material == 1u);
  REQUIRE_FALSE(
    talus::query::sweep(context, moving, glm::vec3(0.0f, -4.0f, 0.0f))
  );
  REQUIRE_FALSE(
    talus::query::sweep(context, moving, glm::vec3(0.0f, 0.5f, 0.0f))
  );
  REQUIRE_THROWS_AS(
    talus::query::sweep({runtime.world()}, moving, glm::vec3(0.0f, 4.0f, 0.0f)),
    std::invalid_argument
  );
}

TEST_CASE(
  "body overlaps return unique other entities and stop after separation",
  "[query][overlap]"
) {
  talus::physics::JoltLibrary library;
  World source;
  const Entity moving = sphere(
    source,
    Pose{glm::vec3(3.0f, 0.0f, 0.0f)},
    glm::vec3(0.0f, 2.0f, 0.0f)
  );
  const Entity fixed = sphere(source, Pose{glm::vec3(3.0f, 3.5f, 0.0f)});
  bake(source, moving);
  bake(source, fixed);
  source.addRigidBody(moving);
  source.addStaticBody(fixed);
  talus::runtime::Runtime runtime(source, library);
  runtime.refreshBody(moving);
  runtime.refreshBody(fixed);
  const talus::query::Context context{runtime.world(), &runtime.physics()};
  const auto hits = talus::query::overlap(context, moving);
  REQUIRE(hits.size() == 1u);
  REQUIRE(hits.front() == fixed);
  const auto& body =
    runtime.world().registry().get<talus::objectmodel::BodyId>(moving);
  runtime.physics().bodies().SetPosition(
    JPH::BodyID(body.id),
    JPH::RVec3(20.0f, 0.0f, 0.0f),
    JPH::EActivation::Activate
  );
  REQUIRE(talus::query::overlap(context, moving).empty());
  REQUIRE_THROWS_AS(
    talus::query::overlap({runtime.world()}, moving),
    std::invalid_argument
  );
  REQUIRE_THROWS_AS(
    talus::query::overlap(context, talus::objectmodel::INVALID_ENTITY),
    std::invalid_argument
  );
}

TEST_CASE(
  "the script raycast ABI returns complete hits and clears misses",
  "[query][scriptquery]"
) {
  World world;
  const Entity entity = sphere(world, Pose{glm::vec3(0.0f, 0.0f, 5.0f)});
  const talus::query::Context queries{world};
  const float ray[6]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.0f};
  const auto& api = talus::script::api();
  talus::script::RaycastHit hit{};
  REQUIRE(api.raycast(&queries, ray, 10.0f, &hit) == 1);
  REQUIRE(hit.entity == static_cast<uint32_t>(entity));
  REQUIRE_THAT(hit.distance, WithinAbs(4.0f, 0.002f));
  REQUIRE_THAT(hit.position[2], WithinAbs(4.0f, 0.002f));
  REQUIRE_THAT(hit.normal[2], WithinAbs(-1.0f, 0.002f));
  REQUIRE(hit.material == 1u);
  REQUIRE(api.raycast(&queries, ray, 3.0f, &hit) == 0);
  REQUIRE(hit.entity == UINT32_MAX);
  REQUIRE(api.raycast(nullptr, ray, 10.0f, &hit) == -1);
  REQUIRE(api.raycast(&queries, nullptr, 10.0f, &hit) == -1);
  REQUIRE(api.raycast(&queries, ray, 10.0f, nullptr) == -1);
  REQUIRE(api.raycast(&queries, ray, -1.0f, &hit) == -1);
}

TEST_CASE(
  "script systems raycast against the current physics context",
  "[query][scriptquery]"
) {
  talus::physics::JoltLibrary library;
  World source;
  const Entity entity = sphere(source);
  bake(source, entity);
  source.addRigidBody(entity);
  talus::runtime::Runtime runtime(source, library);
  runtime.refreshBody(entity);
  runtime.setLinearVelocity(entity, glm::vec3(300.0f, 0.0f, 0.0f));
  runtime.step();
  talus::systems::Schedule schedule;

  struct Seen {
    int32_t result = -1;
    talus::script::RaycastHit hit{};
  } seen;

  const auto callback =
    +[](void* user, const talus::script::SystemContext* context) {
      auto& output = *static_cast<Seen*>(user);
      const float ray[6]{5.0f, 0.0f, -3.0f, 0.0f, 0.0f, 1.0f};
      output.result =
        talus::script::api().raycast(context->queries, ray, 10.0f, &output.hit);
    };
  REQUIRE(
    talus::script::api().addSystem(&schedule, 1u, 0, "query", callback, &seen)
    == 1
  );
  talus::systems::InputState input;
  const talus::query::Context queries{runtime.world(), &runtime.physics()};
  talus::systems::Context context{runtime.world(), input};
  context.queries = &queries;
  schedule.run(talus::systems::Phase::PostPhysics, context);
  REQUIRE(seen.result == 1);
  REQUIRE(seen.hit.entity == static_cast<uint32_t>(entity));
}

TEST_CASE(
  "world rays ignore a stale lattice while authored geometry awaits rebaking",
  "[query]"
) {
  World world;
  const Entity entity = sphere(world);
  bake(world, entity);
  REQUIRE(world.setPrimitive(
    entity,
    0u,
    fixture::sphere(glm::vec3(4.0f, 0.0f, 0.0f), 1.0f)
  ));
  REQUIRE(world.needsResample(entity));
  const auto hit = raycast(
    {world},
    {glm::vec3(4.0f, 0.0f, -3.0f), glm::vec3(0.0f, 0.0f, 1.0f)}
  );
  REQUIRE(hit);
  REQUIRE(hit->entity == entity);
  REQUIRE_FALSE(raycast(
    {world},
    {glm::vec3(0.0f, 0.0f, -3.0f), glm::vec3(0.0f, 0.0f, 1.0f)}
  ));
}
