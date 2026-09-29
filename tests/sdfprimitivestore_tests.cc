#include <catch2/catch_test_macros.hpp>

#include <talus/core/config/config.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/sdfprimitivestore/sdfprimitivestore.h>

#include <entt/core/hashed_string.hpp>
#include <entt/entity/registry.hpp>

#include <cstdint>

namespace {

using talus::objectmodel::Entity;
using talus::objectmodel::SdfGrid;
using talus::objectmodel::SdfPrimitiveRange;
using talus::objectmodel::SdfPrimitiveStore;

auto& bakeQueue(entt::registry& registry) {
  auto& queue = registry.storage<entt::reactive>(entt::hashed_string{"bake"});
  queue.on_update<SdfGrid>();
  return queue;
}

talus::field::Primitive marker(uint32_t material) {
  return talus::field::makeSphere(glm::vec3(0.0f), 1.0f, material);
}

Entity makeObject(entt::registry& registry) {
  const Entity entity = registry.create();

  SdfGrid grid{};
  grid.resolution = glm::uvec3(talus::core::FIELD_GRID_RESOLUTION);

  registry.emplace<SdfGrid>(entity, grid);

  return entity;
}

uint32_t materialAt(
  const SdfPrimitiveStore& store,
  const entt::registry& registry,
  Entity entity,
  uint32_t index
) {
  return store.primitives(registry, entity)[index].shapeConfig.y;
}

}

TEST_CASE("primitives need a grid to be sampled onto", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity bare = registry.create();

  REQUIRE_FALSE(store.append(registry, bare, marker(1)));
  REQUIRE(store.count(registry, bare) == 0);
}

TEST_CASE("appending keeps order and grows the count", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity entity = makeObject(registry);

  REQUIRE(store.append(registry, entity, marker(1)));
  REQUIRE(store.append(registry, entity, marker(2)));

  REQUIRE(store.count(registry, entity) == 2);
  REQUIRE(materialAt(store, registry, entity, 0) == 1);
  REQUIRE(materialAt(store, registry, entity, 1) == 2);
}

TEST_CASE("inserting shifts the primitives after it", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity entity = makeObject(registry);

  REQUIRE(store.append(registry, entity, marker(1)));
  REQUIRE(store.append(registry, entity, marker(3)));

  REQUIRE(store.insert(registry, entity, 1, marker(2)));

  REQUIRE(store.count(registry, entity) == 3);
  REQUIRE(materialAt(store, registry, entity, 0) == 1);
  REQUIRE(materialAt(store, registry, entity, 1) == 2);
  REQUIRE(materialAt(store, registry, entity, 2) == 3);
}

TEST_CASE("removing shifts the primitives down", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity entity = makeObject(registry);

  REQUIRE(store.append(registry, entity, marker(1)));
  REQUIRE(store.append(registry, entity, marker(2)));
  REQUIRE(store.append(registry, entity, marker(3)));

  REQUIRE(store.remove(registry, entity, 1));

  REQUIRE(store.count(registry, entity) == 2);
  REQUIRE(materialAt(store, registry, entity, 0) == 1);
  REQUIRE(materialAt(store, registry, entity, 1) == 3);
}

TEST_CASE("setting replaces in place", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity entity = makeObject(registry);

  REQUIRE(store.append(registry, entity, marker(1)));
  REQUIRE(store.append(registry, entity, marker(2)));

  REQUIRE(store.set(registry, entity, 0, marker(9)));

  REQUIRE(store.count(registry, entity) == 2);
  REQUIRE(materialAt(store, registry, entity, 0) == 9);
  REQUIRE(materialAt(store, registry, entity, 1) == 2);
}

TEST_CASE(
  "crossing the capacity boundary preserves the primitives",
  "[sdfstore]"
) {
  entt::registry registry;
  SdfPrimitiveStore store;

  const Entity entity = makeObject(registry);

  for (uint32_t i = 0; i != 5; ++i) {
    REQUIRE(store.append(registry, entity, marker(i + 1)));
  }

  REQUIRE(store.count(registry, entity) == 5);
  REQUIRE(registry.get<SdfPrimitiveRange>(entity).capacity == 8);

  for (uint32_t i = 0; i != 5; ++i) {
    REQUIRE(materialAt(store, registry, entity, i) == i + 1);
  }
}

TEST_CASE(
  "an edit queues the grid for bake and re-fits its box",
  "[sdfstore]"
) {
  entt::registry registry;
  SdfPrimitiveStore store;

  auto& queue = bakeQueue(registry);

  const Entity entity = makeObject(registry);

  REQUIRE_FALSE(queue.contains(entity));

  REQUIRE(store.append(registry, entity, marker(1)));

  REQUIRE(queue.contains(entity));
  REQUIRE(registry.get<SdfGrid>(entity).voxelSize.x > 0.0f);
}

TEST_CASE("destroying an entity returns its range to the arena", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  store.connect(registry);

  const Entity entity = makeObject(registry);
  REQUIRE(store.append(registry, entity, marker(1)));

  REQUIRE(store.pool().size() == 4);

  registry.destroy(entity);

  REQUIRE(store.pool().size() == 0);
}

TEST_CASE("removing the range component alone returns it too", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  store.connect(registry);

  const Entity entity = makeObject(registry);
  REQUIRE(store.append(registry, entity, marker(1)));

  registry.remove<SdfPrimitiveRange>(entity);

  REQUIRE(store.pool().size() == 0);
  REQUIRE(registry.valid(entity));
}

TEST_CASE("clearing the registry returns every range", "[sdfstore]") {
  entt::registry registry;
  SdfPrimitiveStore store;

  store.connect(registry);

  const Entity first = makeObject(registry);
  const Entity second = makeObject(registry);

  REQUIRE(store.append(registry, first, marker(1)));
  REQUIRE(store.append(registry, second, marker(2)));

  REQUIRE(store.pool().size() == 8);

  registry.clear();

  REQUIRE(store.pool().size() == 0);
}

TEST_CASE(
  "clearing empties the primitives and keeps the allocation",
  "[sdfstore]"
) {
  entt::registry registry;
  SdfPrimitiveStore store;

  store.connect(registry);

  auto& queue = bakeQueue(registry);

  const Entity entity = makeObject(registry);

  REQUIRE(store.append(registry, entity, marker(1)));
  REQUIRE(store.append(registry, entity, marker(2)));
  REQUIRE(store.append(registry, entity, marker(3)));

  const size_t allocated = store.pool().size();

  queue.remove(entity);

  REQUIRE(store.clear(registry, entity));

  REQUIRE(store.count(registry, entity) == 0);
  REQUIRE(store.primitives(registry, entity).empty());
  REQUIRE(store.pool().size() == allocated);
  REQUIRE(queue.contains(entity));
}

TEST_CASE(
  "clearing an entity that holds no primitives is refused",
  "[sdfstore]"
) {
  entt::registry registry;
  SdfPrimitiveStore store;

  store.connect(registry);

  const Entity entity = makeObject(registry);

  REQUIRE_FALSE(store.clear(registry, entity));
}
