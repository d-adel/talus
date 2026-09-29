#include <catch2/catch_test_macros.hpp>

#include <entt/entity/registry.hpp>

#include <cstdint>

namespace {

struct Marker {
  float value;
};

}

TEST_CASE("a component round trips through a registry", "[entt]") {
  entt::registry registry;

  const entt::entity entity = registry.create();
  registry.emplace<Marker>(entity, 42.0f);

  REQUIRE(registry.valid(entity));
  REQUIRE(registry.get<Marker>(entity).value == 42.0f);
}

TEST_CASE("a handle to a destroyed entity is detectable", "[entt]") {
  entt::registry registry;

  const entt::entity entity = registry.create();
  registry.destroy(entity);

  REQUIRE_FALSE(registry.valid(entity));
}

TEST_CASE("a recycled entity is not the same number", "[entt]") {
  entt::registry registry;

  const entt::entity first = registry.create();
  registry.destroy(first);

  const entt::entity second = registry.create();

  REQUIRE(registry.valid(second));
  REQUIRE_FALSE(registry.valid(first));
  REQUIRE(
    static_cast<std::uint32_t>(second) != static_cast<std::uint32_t>(first)
  );
}

namespace {

struct UpdateCounter {
  uint32_t count = 0;

  void bump(entt::registry&, entt::entity) {
    ++count;
  }
};

}

TEST_CASE(
  "patch and replace publish on_update, a write through get does not",
  "[entt]"
) {
  entt::registry registry;

  UpdateCounter counter;
  registry.on_update<Marker>().connect<&UpdateCounter::bump>(counter);

  const entt::entity entity = registry.create();
  registry.emplace<Marker>(entity, 1.0f);

  REQUIRE(counter.count == 0);

  registry.patch<Marker>(entity, [](Marker& marker) { marker.value = 2.0f; });

  REQUIRE(counter.count == 1);

  registry.replace<Marker>(entity, Marker{3.0f});

  REQUIRE(counter.count == 2);

  registry.get<Marker>(entity).value = 4.0f;

  REQUIRE(counter.count == 2);
  REQUIRE(registry.get<Marker>(entity).value == 4.0f);
}
