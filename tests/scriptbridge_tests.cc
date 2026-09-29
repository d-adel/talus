#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/rigidbody/rigidbody.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/instantiate/instantiate.h>
#include <talus/objectmodel/world/world.h>
#include <talus/script/api/api.h>
#include <talus/script/runner/runner.h>
#include <talus/systems/input/input.h>
#include <talus/systems/schedule/schedule.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <climits>
#include <filesystem>

using talus::objectmodel::ComponentType;
using talus::objectmodel::Entity;
using talus::objectmodel::INVALID_COMPONENT_TYPE;
using talus::objectmodel::World;
using talus::script::Runner;
using talus::systems::Context;

namespace {

const talus::systems::InputState& noInput() {
  static const talus::systems::InputState idle;

  return idle;
}

}

using talus::systems::Schedule;

namespace {

std::filesystem::path managedDirectory() {
  return std::filesystem::path(TALUS_MANAGED_DIR);
}

std::filesystem::path scriptsDirectory() {
  return std::filesystem::path(TALUS_SCRIPTS_DIR);
}

Entity rigidSphere(World& world, float radius) {
  talus::objectmodel::SdfGrid grid{};
  grid.resolution = glm::uvec3(33u);

  const Entity entity = world.createSdfGrid({}, grid);

  REQUIRE(world.addPrimitive(entity, fixture::sphere(glm::vec3(0.0f), radius)));

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

  world.addRigidBody(entity);

  return entity;
}

std::array<std::byte, 12> nudge(const std::array<float, 3>& offset) {
  std::array<std::byte, 12> value{};

  std::memcpy(value.data(), offset.data(), 12u);

  return value;
}

glm::vec3 positionOf(const World& world, Entity entity) {
  return world.registry().get<talus::objectmodel::Pose>(entity).position;
}

uint32_t idOf(Entity entity) {
  return static_cast<uint32_t>(entt::to_integral(entity));
}

}

TEST_CASE(
  "a project's scripts boot and register their systems",
  "[scriptbridge]"
) {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, scriptsDirectory()));
  REQUIRE(runner.running());

  REQUIRE(schedule.size() >= 1u);

  bool sawNudge = false;
  int32_t previous = INT32_MIN;

  for (const talus::systems::Entry& entry : schedule.systems()) {
    REQUIRE(entry.order >= previous);

    previous = entry.order;

    if (entry.name == "nudge") {
      sawNudge = true;

      REQUIRE(entry.order == 100);
    }
  }

  REQUIRE(sawNudge);
}

TEST_CASE(
  "the engine derives a component's layout from the script's struct",
  "[scriptbridge]"
) {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, scriptsDirectory()));

  const ComponentType type = world.dynamic().find("Nudge");

  REQUIRE(type != INVALID_COMPONENT_TYPE);

  const talus::objectmodel::ComponentSpec* spec = world.dynamic().spec(type);

  REQUIRE(spec->size == 12u);
  REQUIRE(spec->fields.size() == 1u);

  REQUIRE(spec->fields[0].name == "offset");
  REQUIRE(spec->fields[0].offset == 0u);
  REQUIRE(spec->fields[0].kind == talus::objectmodel::FieldKind::Vec3);

  REQUIRE(spec->requirements.size() == 1u);
  REQUIRE(spec->requirements[0] == "RigidBody");
}

TEST_CASE("a script system moves the engine's entity", "[scriptbridge]") {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, scriptsDirectory()));

  const Entity entity = rigidSphere(world, 1.0f);

  REQUIRE(positionOf(world, entity) == glm::vec3(0.0f));

  const ComponentType request = world.dynamic().find("Nudge");

  REQUIRE(world.dynamic().emplace(request, entity, nudge({1.0f, 0.0f, 0.0f})));

  Context context{world, noInput(), 0.016f, 1u};
  schedule.run(context);

  REQUIRE(positionOf(world, entity) == glm::vec3(1.0f, 0.0f, 0.0f));
  REQUIRE_FALSE(world.dynamic().contains(request, entity));
}

TEST_CASE(
  "a script system leaves entities it did not match alone",
  "[scriptbridge]"
) {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, scriptsDirectory()));

  const Entity moved = rigidSphere(world, 1.0f);
  const Entity spared = rigidSphere(world, 1.0f);

  const ComponentType request = world.dynamic().find("Nudge");

  REQUIRE(world.dynamic().emplace(request, moved, nudge({0.0f, 2.0f, 0.0f})));

  Context context{world, noInput(), 0.016f, 1u};
  schedule.run(context);

  REQUIRE(positionOf(world, moved) == glm::vec3(0.0f, 2.0f, 0.0f));
  REQUIRE(positionOf(world, spared) == glm::vec3(0.0f));
  REQUIRE(world.dynamic().count(request) == 0u);
}

TEST_CASE("every nudge moves its entity by its own offset", "[scriptbridge]") {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, scriptsDirectory()));

  const ComponentType request = world.dynamic().find("Nudge");

  for (const float along : {0.5f, 1.5f, 3.0f}) {
    const Entity entity = rigidSphere(world, 1.0f);

    REQUIRE(
      world.dynamic().emplace(request, entity, nudge({0.0f, 0.0f, along}))
    );

    Context context{world, noInput(), 0.016f, 1u};
    schedule.run(context);

    REQUIRE(positionOf(world, entity) == glm::vec3(0.0f, 0.0f, along));
  }
}

TEST_CASE(
  "a runner pointed at nothing reports rather than crashing",
  "[scriptbridge]"
) {
  Runner runner;

  REQUIRE_FALSE(runner.load(managedDirectory() / "absent"));
  REQUIRE_FALSE(runner.running());
  REQUIRE_FALSE(runner.lastError().empty());
}

TEST_CASE("a project with no scripts runs with no systems", "[scriptbridge]") {
  Runner runner;
  Schedule schedule;
  World world;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, world, managedDirectory() / "absent"));
  REQUIRE(runner.running());

  REQUIRE(schedule.size() == 0u);
  REQUIRE(world.dynamic().types() == 0u);
}

TEST_CASE(
  "declared component types survive instantiation with their indices",
  "[scriptbridge]"
) {
  Runner runner;
  Schedule schedule;
  World authored;

  REQUIRE(runner.load(managedDirectory()));
  REQUIRE(runner.initialize(schedule, authored, scriptsDirectory()));

  const ComponentType type = authored.dynamic().find("Nudge");

  const Entity entity = authored.createSdfGrid({}, {});

  authored.addRigidBody(entity);

  REQUIRE(authored.dynamic().emplace(type, entity, nudge({1.0f, 2.0f, 3.0f})));

  World runtime;
  talus::objectmodel::instantiateWorld(authored, runtime);

  REQUIRE(runtime.dynamic().find("Nudge") == type);
  REQUIRE(runtime.dynamic().contains(type, entity));

  float along = 0.0f;
  std::memcpy(&along, runtime.dynamic().get(type, entity) + 8, 4u);

  REQUIRE(along == 3.0f);
}

TEST_CASE(
  "the material query reports what a cut would reach",
  "[scriptbridge]"
) {
  World world;

  const Entity entity = rigidSphere(world, 1.0f);

  talus::script::SdfEditDescriptor inside{};
  inside.kind = 0u;
  inside.size[0] = 0.3f;
  inside.rotation[3] = 1.0f;

  std::array<uint32_t, 8> found{};

  const uint32_t count = talus::script::api().materialsUnderSdf(
    &world,
    idOf(entity),
    &inside,
    found.data(),
    static_cast<uint32_t>(found.size())
  );

  REQUIRE(count >= 1u);
  REQUIRE(std::ranges::find(found, 1u) != found.end());

  talus::script::SdfEditDescriptor outside{};
  outside.kind = 0u;
  outside.size[0] = 0.1f;
  outside.position[0] = 5.0f;
  outside.rotation[3] = 1.0f;

  REQUIRE(
    talus::script::api().materialsUnderSdf(
      &world,
      idOf(entity),
      &outside,
      found.data(),
      static_cast<uint32_t>(found.size())
    )
    == 0u
  );
}

TEST_CASE(
  "a deform records the samples it touched on the world",
  "[scriptbridge]"
) {
  World world;

  const Entity entity = rigidSphere(world, 1.0f);

  REQUIRE(world.sdfDirty().empty());

  talus::script::SdfEditDescriptor cut{};
  cut.kind = 0u;
  cut.operation = talus::core::FIELD_OP_SUBTRACTION;
  cut.size[0] = 0.3f;
  cut.rotation[3] = 1.0f;

  REQUIRE(
    talus::script::api().deformSdf(&world, idOf(entity), &cut, nullptr) == 1
  );

  REQUIRE(world.sdfDirty().size() == 1u);
  REQUIRE(world.sdfDirty()[0].first == entity);

  const talus::field::SampleBox& touched = world.sdfDirty()[0].second;

  REQUIRE(touched.extent.x > 0u);
  REQUIRE(touched.extent.y > 0u);
  REQUIRE(touched.extent.z > 0u);
}

TEST_CASE(
  "the bricks a deform reports are the bricks its samples reach",
  "[scriptbridge]"
) {
  World world;

  const Entity entity = rigidSphere(world, 1.0f);

  talus::script::SdfEditDescriptor cut{};
  cut.kind = 0u;
  cut.operation = talus::core::FIELD_OP_SUBTRACTION;
  cut.size[0] = 0.3f;
  cut.rotation[3] = 1.0f;

  talus::script::SdfDeformSummary summary{};

  REQUIRE(
    talus::script::api().deformSdf(&world, idOf(entity), &cut, &summary) == 1
  );

  const talus::field::SampledSdf* lattice = world.sampledSdf(entity);

  REQUIRE(lattice != nullptr);

  const talus::field::BrickRange derived =
    talus::field::brickRange(*lattice, world.sdfDirty()[0].second);

  REQUIRE(derived.begin.x == summary.brickBegin[0]);
  REQUIRE(derived.begin.y == summary.brickBegin[1]);
  REQUIRE(derived.begin.z == summary.brickBegin[2]);
  REQUIRE(derived.end.x == summary.brickEnd[0]);
  REQUIRE(derived.end.y == summary.brickEnd[1]);
  REQUIRE(derived.end.z == summary.brickEnd[2]);
}
