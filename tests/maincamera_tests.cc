#include <catch2/catch_test_macros.hpp>

#include <talus/objectmodel/component/lens/lens.h>
#include <talus/objectmodel/component/maincamera/maincamera.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/world/world.h>
#include <talus/objectmodel/worldquery/worldquery.h>

#include <entt/entity/registry.hpp>

using talus::objectmodel::Entity;
using talus::objectmodel::Lens;
using talus::objectmodel::MainCamera;
using talus::objectmodel::Pose;
using talus::objectmodel::World;

namespace {

Entity camera(World& world) {
  const Entity eye = world.createAuthored();

  world.emplaceAuthored<Pose>(eye, Pose{});
  world.emplaceAuthored<Lens>(eye, Lens{});

  return eye;
}

uint32_t tagged(const World& world) {
  return uint32_t(world.registry().view<const MainCamera>().size());
}

}

TEST_CASE("an untagged camera is not the main one", "[maincamera]") {
  World world;

  const Entity eye = camera(world);

  REQUIRE(tagged(world) == 0u);
  REQUIRE(
    talus::objectmodel::mainCamera(world) == talus::objectmodel::INVALID_ENTITY
  );

  REQUIRE(world.setMainCamera(eye));
  REQUIRE(talus::objectmodel::mainCamera(world) == eye);
}

TEST_CASE("the tagged camera wins over the first one", "[maincamera]") {
  World world;

  const Entity first = camera(world);
  const Entity second = camera(world);

  REQUIRE(world.setMainCamera(first));
  REQUIRE(talus::objectmodel::mainCamera(world) == first);
  REQUIRE(world.setMainCamera(second));
  REQUIRE(talus::objectmodel::mainCamera(world) == second);
}

TEST_CASE("only one camera is ever the main one", "[maincamera]") {
  World world;

  const Entity first = camera(world);
  const Entity second = camera(world);

  REQUIRE(world.setMainCamera(first));
  REQUIRE(world.setMainCamera(second));

  REQUIRE(tagged(world) == 1u);
  REQUIRE(talus::objectmodel::mainCamera(world) == second);
}

TEST_CASE(
  "an entity without a lens cannot be the main camera",
  "[maincamera]"
) {
  World world;

  const Entity bare = world.createAuthored();

  world.emplaceAuthored<Pose>(bare, Pose{});

  REQUIRE_FALSE(world.setMainCamera(bare));
  REQUIRE(tagged(world) == 0u);
}

TEST_CASE("a world with no camera at all reports none", "[maincamera]") {
  World world;

  REQUIRE(
    talus::objectmodel::mainCamera(world) == talus::objectmodel::INVALID_ENTITY
  );
}
