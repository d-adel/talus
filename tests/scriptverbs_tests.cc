#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/impact/impact.h>
#include <talus/objectmodel/world/world.h>
#include <talus/script/api/api.h>
#include <talus/serialize/materialfile/materialfile.h>
#include <talus/systems/schedule/schedule.h>

#include <cmath>
#include <limits>
#include <vector>

namespace {

struct Recorder {
  uint32_t entity = UINT32_MAX;
  float mass = 0.0f;
  float velocity[3]{};
  uint32_t destroyed = UINT32_MAX;
};

int32_t recordBody(void* host, void*, uint32_t entity, float mass) {
  auto* seen = static_cast<Recorder*>(host);

  seen->entity = entity;
  seen->mass = mass;

  return 1;
}

int32_t recordVelocity(void* host, void*, uint32_t entity, const float* value) {
  auto* seen = static_cast<Recorder*>(host);

  seen->entity = entity;
  seen->velocity[0] = value[0];
  seen->velocity[1] = value[1];
  seen->velocity[2] = value[2];

  return 1;
}

int32_t recordDestroy(void* host, void*, uint32_t entity) {
  static_cast<Recorder*>(host)->destroyed = entity;

  return 1;
}

constexpr talus::script::PhysicsVerbs RECORDING{
  &recordBody,
  &recordVelocity,
  &recordDestroy
};

}

TEST_CASE("a script builds an object out of general verbs", "[scriptverbs]") {
  talus::objectmodel::World world;

  const talus::script::Api& api = talus::script::api();

  const float pose[7]{1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const uint32_t resolution[3]{33u, 33u, 33u};

  const uint32_t made = api.createSdfGrid(&world, pose, resolution, 0.0f);

  REQUIRE(made != UINT32_MAX);

  const auto entity =
    static_cast<talus::objectmodel::Entity>(entt::entity{made});

  REQUIRE(world.registry().all_of<talus::objectmodel::SdfGrid>(entity));
  REQUIRE(
    world.registry().get<talus::objectmodel::Pose>(entity).position
    == glm::vec3(1.0f, 2.0f, 3.0f)
  );
  REQUIRE(
    world.registry().get<talus::objectmodel::SdfGrid>(entity).resolution
    == glm::uvec3(33u)
  );

  talus::script::SdfEditDescriptor shape{};
  shape.kind = 0u;
  shape.material = 1u;
  shape.size[0] = 0.35f;
  shape.position[0] = 1.0f;
  shape.position[1] = 2.0f;
  shape.position[2] = 3.0f;
  shape.rotation[3] = 1.0f;

  REQUIRE(api.addPrimitive(&world, made, &shape) == 1);
  REQUIRE(world.primitiveCount(entity) == 1u);

  const glm::vec3 centre =
    glm::vec3(glm::inverse(world.primitives(entity)[0].inverseModel)[3]);

  REQUIRE(glm::length(centre) < 1.0e-4f);
}

TEST_CASE(
  "the physics verbs reach whichever host installed them",
  "[scriptverbs]"
) {
  talus::objectmodel::World world;

  const talus::script::Api& api = talus::script::api();

  Recorder seen;

  {
    const talus::script::PhysicsScope scope(&RECORDING, &seen);

    const float velocity[3]{0.0f, 0.0f, -22.0f};

    REQUIRE(api.setRigidBody(&world, 7u, 150.0f) == 1);
    REQUIRE(api.setVelocity(&world, 7u, velocity) == 1);
    REQUIRE(api.destroy(&world, 7u) == 1);
  }

  REQUIRE(seen.entity == 7u);
  REQUIRE(seen.mass == 150.0f);
  REQUIRE(seen.velocity[2] == -22.0f);
  REQUIRE(seen.destroyed == 7u);

  REQUIRE(api.setRigidBody(&world, 7u, 1.0f) == 0);
  REQUIRE(api.setVelocity(&world, 7u, nullptr) == 0);
}

TEST_CASE("a shared lattice is one lattice", "[scriptverbs]") {
  talus::objectmodel::World world;

  const talus::script::Api& api = talus::script::api();

  const float pose[7]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const uint32_t resolution[3]{17u, 17u, 17u};

  const uint32_t donor = api.createSdfGrid(&world, pose, resolution, 0.0f);
  const uint32_t taker = api.createSdfGrid(&world, pose, resolution, 0.0f);

  REQUIRE(api.shareSdf(&world, donor, taker) == 0);

  const auto donorEntity =
    static_cast<talus::objectmodel::Entity>(entt::entity{donor});
  const auto takerEntity =
    static_cast<talus::objectmodel::Entity>(entt::entity{taker});

  talus::field::SampledSdf lattice{};
  lattice.resolution = glm::uvec3(4u);
  lattice.distances.assign(64u, 1.0f);
  lattice.materials.assign(64u, 0u);

  world.setSampledSdf(donorEntity, std::move(lattice));

  REQUIRE(api.shareSdf(&world, donor, taker) == 1);
  REQUIRE(world.sampledSdf(donorEntity) == world.sampledSdf(takerEntity));
}

TEST_CASE(
  "a script reads back the physical material the file says",
  "[scriptverbs]"
) {
  talus::objectmodel::World world;

  const talus::script::Api& api = talus::script::api();

  talus::serialize::StoredMaterial stored{};
  REQUIRE(talus::serialize::readMaterial(
    R"({"friction":0.9,"restitution":0.6,"density":2500,"strength":250})",
    stored
  ));

  const talus::objectmodel::PhysicalMaterial authored =
    talus::serialize::physicalMaterial(stored);

  world.setMaterials({talus::objectmodel::PhysicalMaterial{}, authored});

  talus::script::PhysicalMaterial out{};

  REQUIRE(api.material(&world, 1u, &out) == 1);
  REQUIRE(out.friction == 0.9f);
  REQUIRE(out.restitution == 0.6f);
  REQUIRE(out.density == 2500.0f);
  REQUIRE(out.strength == 250.0f);

  REQUIRE(api.material(&world, 7u, &out) == 0);
  REQUIRE(out.friction == 0.5f);
  REQUIRE(out.restitution == 0.0f);
  REQUIRE(out.density == 1000.0f);
  REQUIRE(out.strength == std::numeric_limits<float>::infinity());

  REQUIRE(api.material(&world, 1u, nullptr) == 0);
  REQUIRE(api.material(nullptr, 1u, &out) == 0);
}

TEST_CASE(
  "a script system reads back the material each side of an impact struck",
  "[scriptverbs]"
) {
  talus::objectmodel::World world;

  const talus::script::Api& api = talus::script::api();

  talus::systems::Schedule schedule;

  struct Seen {
    uint32_t count = 0u;
    uint32_t entity = 0u;
    uint32_t other = 0u;
    uint32_t material = 0u;
    uint32_t otherMaterial = 0u;
  } seen;

  const auto callback =
    +[](void* user, const talus::script::SystemContext* context) {
      auto& output = *static_cast<Seen*>(user);
      output.count = context->impactCount;
      if (output.count > 0u) {
        output.entity = context->impacts[0].entity;
        output.other = context->impacts[0].other;
        output.material = context->impacts[0].material;
        output.otherMaterial = context->impacts[0].otherMaterial;
      }
    };

  REQUIRE(api.addSystem(&schedule, 1u, 0, "impacts", callback, &seen) == 1);

  talus::objectmodel::Impact impact{};
  impact.entity = static_cast<talus::objectmodel::Entity>(entt::entity{3u});
  impact.other = static_cast<talus::objectmodel::Entity>(entt::entity{9u});
  impact.material = 2u;
  impact.otherMaterial = 5u;

  const std::vector<talus::objectmodel::Impact> impacts{impact};

  talus::systems::InputState input;
  talus::systems::Context context{world, input};
  context.impacts = impacts;

  schedule.run(talus::systems::Phase::PostPhysics, context);

  REQUIRE(seen.count == 1u);
  REQUIRE(seen.entity == 3u);
  REQUIRE(seen.other == 9u);
  REQUIRE(seen.material == 2u);
  REQUIRE(seen.otherMaterial == 5u);
}
