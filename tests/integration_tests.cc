#include <catch2/catch_test_macros.hpp>
#include <talus/talus.h>

#include <array>
#include <cmath>

namespace {

uint32_t box(talus::World& world, const glm::dvec3& at, const glm::dvec3& half, bool fixed) {
  const std::array<talus::dynamics::CompoundChild, 1> shape{
    talus::dynamics::CompoundChild{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, half, 500.0, 0u}
  };
  return talus::dynamics::addCompoundBody(world.dynamics.bodies, shape, at,
    {1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0.6, 0.0, fixed);
}

}

TEST_CASE("the engine-neutral world advances fixed ticks without a scene or renderer", "[integration]") {
  talus::World world;
  // Dyadic inputs make this a tick/force contract check, independent of float accumulation error.
  world.dynamics.parameters.dt = 1.0 / 64.0;
  world.dynamics.parameters.gravity = {0.0, -8.0, 0.0};
  const uint32_t body = box(world, {0.0, 10.0, 0.0}, {0.5, 0.5, 0.5}, false);
  for (uint32_t tick = 0u; tick < 64u; ++tick) {
    REQUIRE(!world.step().dynamics.nonFinite);
  }
  const auto& state = world.dynamics.bodies.rigids.at(body);
  CHECK(std::abs(world.dynamics.time - 1.0) < 1.0e-12);
  CHECK(std::abs(state.velocity.y + 8.0) < 1.0e-10);
  CHECK(state.position.y < 10.0);
}

TEST_CASE("an embedded world lands a box on a static shape", "[integration]") {
  talus::World world;
  world.dynamics.workerCount = 8u;
  box(world, {0.0, -0.5, 0.0}, {8.0, 0.5, 8.0}, true);
  const uint32_t body = box(world, {0.0, 3.0, 0.0}, {0.5, 0.5, 0.5}, false);
  for (uint32_t tick = 0u; tick < 360u; ++tick) {
    REQUIRE(!world.step().dynamics.nonFinite);
  }
  const auto& state = world.dynamics.bodies.rigids.at(body);
  CHECK(state.position.y >= 0.48);
  CHECK(state.position.y <= 0.53);
  CHECK(glm::length(state.velocity) < 0.05);
}

TEST_CASE("an embedded world processes scheduled destruction before solving motion", "[integration]") {
  talus::World world;
  using talus::destruction::Role;
  talus::destruction::Building beam;
  beam.members = {
    {{0.0, -0.25, 0.0}, {2.0, 0.25, 2.0}, 0u, Role::Foundation},
    {{0.0, 0.5, 0.0}, {0.25, 0.5, 0.25}, 0u, Role::Column},
    {{0.0, 1.25, 0.0}, {1.0, 0.25, 0.25}, 0u, Role::Beam}
  };
  auto steel = talus::destruction::steel();
  steel.chunkSize = 4.0;
  const std::array<talus::destruction::StructuralMaterial, 1> materials{steel};
  const auto added = world.destruction.addBuilding(world.dynamics, beam, materials);
  REQUIRE(added.settled);
  REQUIRE(added.broken.empty());
  std::vector<uint32_t> support;
  const auto& chunks = world.destruction.chunked(added.building).chunks;
  for (uint32_t chunk = 0; chunk < chunks.size(); ++chunk) {
    if (beam.members[chunks[chunk].member].role == Role::Column) {
      support.push_back(chunk);
    }
  }
  REQUIRE(support.size() == 1u);
  world.destruction.removeAt(3u, added.building, support);
  CHECK(world.step().destruction.removed == 0u);
  CHECK(world.step().destruction.removed == 0u);
  const auto removed = world.step();
  CHECK(removed.destruction.removed == 1u);
  REQUIRE(!removed.destruction.created.empty());
  for (uint32_t tick = 0u; tick < 120u; ++tick) {
    REQUIRE(!world.step().dynamics.nonFinite);
  }
  bool fell = false;
  for (const auto& body : world.dynamics.bodies.rigids) {
    fell |= body.mass + body.frozenMass > 0.0 && body.position.y < 1.0;
  }
  CHECK(fell);
}
