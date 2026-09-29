#include <talus/view/lookthrough/lookthrough.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using Catch::Matchers::WithinAbs;
using talus::objectmodel::Entity;
using talus::objectmodel::Lens;
using talus::objectmodel::Pose;
using talus::objectmodel::World;
using talus::view::Viewport;

namespace {

constexpr float TOLERANCE = 1e-4f;

Entity cameraAt(World& world, const glm::vec3& where, float fov) {
  const Entity eye = world.createAuthored();

  world.emplaceAuthored<Pose>(
    eye,
    Pose{where, glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}
  );
  world.emplaceAuthored<Lens>(eye, Lens{fov, 0.1f, 1000.0f});

  return eye;
}

glm::vec3 eyeOf(const talus::objectmodel::CameraView& seen) {
  return seen.position;
}

}

TEST_CASE("an unbound viewport looks through its own camera", "[lookthrough]") {
  World world;

  Viewport port{};
  port.pose.position = glm::vec3(1.0f, 2.0f, 3.0f);

  REQUIRE_FALSE(bindingIsLive(port, world));

  const auto seen = lookThrough(port, world, 1.5f);

  REQUIRE_THAT(eyeOf(seen).x, WithinAbs(1.0f, TOLERANCE));
  REQUIRE_THAT(eyeOf(seen).y, WithinAbs(2.0f, TOLERANCE));
  REQUIRE_THAT(eyeOf(seen).z, WithinAbs(3.0f, TOLERANCE));
}

TEST_CASE(
  "a bound viewport looks through the world's camera",
  "[lookthrough]"
) {
  World world;

  const Entity eye = cameraAt(world, glm::vec3(10.0f, 0.0f, 0.0f), 70.0f);

  Viewport port{};
  port.pose.position = glm::vec3(1.0f, 2.0f, 3.0f);
  port.camera = eye;

  REQUIRE(bindingIsLive(port, world));

  const auto seen = lookThrough(port, world, 1.5f);

  REQUIRE_THAT(eyeOf(seen).x, WithinAbs(10.0f, TOLERANCE));
  REQUIRE_THAT(eyeOf(seen).y, WithinAbs(0.0f, TOLERANCE));
}

TEST_CASE("a binding to a destroyed camera falls back", "[lookthrough]") {
  World world;

  const Entity eye = cameraAt(world, glm::vec3(10.0f, 0.0f, 0.0f), 70.0f);

  Viewport port{};
  port.pose.position = glm::vec3(1.0f, 2.0f, 3.0f);
  port.camera = eye;

  REQUIRE(world.destroy(eye));
  REQUIRE_FALSE(bindingIsLive(port, world));

  const auto seen = lookThrough(port, world, 1.5f);

  REQUIRE_THAT(eyeOf(seen).x, WithinAbs(1.0f, TOLERANCE));
}

TEST_CASE(
  "a binding to something posed but lensless falls back",
  "[lookthrough]"
) {
  World world;

  const Entity posed = world.createAuthored();

  world.emplaceAuthored<Pose>(
    posed,
    Pose{glm::vec3(99.0f, 0.0f, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f)}
  );

  Viewport port{};
  port.pose.position = glm::vec3(4.0f, 5.0f, 6.0f);
  port.camera = posed;

  REQUIRE_FALSE(bindingIsLive(port, world));

  const auto seen = lookThrough(port, world, 1.5f);

  REQUIRE_THAT(eyeOf(seen).x, WithinAbs(4.0f, TOLERANCE));
  REQUIRE_THAT(eyeOf(seen).z, WithinAbs(6.0f, TOLERANCE));
}

TEST_CASE("a binding to a bare entity falls back", "[lookthrough]") {
  World world;

  const Entity bare = world.createAuthored();

  Viewport port{};
  port.pose.position = glm::vec3(4.0f, 5.0f, 6.0f);
  port.camera = bare;

  REQUIRE_FALSE(bindingIsLive(port, world));

  const auto seen = lookThrough(port, world, 1.5f);

  REQUIRE_THAT(eyeOf(seen).z, WithinAbs(6.0f, TOLERANCE));
}

TEST_CASE("the viewport's own lens shapes its projection", "[lookthrough]") {
  World world;

  Viewport narrow{};
  narrow.lens.verticalFov = 30.0f;

  Viewport wide{};
  wide.lens.verticalFov = 90.0f;

  const auto tight = lookThrough(narrow, world, 1.0f);
  const auto open = lookThrough(wide, world, 1.0f);

  REQUIRE(tight.projection[1][1] < open.projection[1][1]);
}

TEST_CASE("the bound camera's lens wins over the viewport's", "[lookthrough]") {
  World world;

  const Entity eye = cameraAt(world, glm::vec3(0.0f), 30.0f);

  Viewport port{};
  port.lens.verticalFov = 90.0f;
  port.camera = eye;

  const auto seen = lookThrough(port, world, 1.0f);

  Viewport onlyNarrow{};
  onlyNarrow.lens.verticalFov = 30.0f;

  const auto expected = lookThrough(onlyNarrow, world, 1.0f);

  REQUIRE_THAT(
    seen.projection[1][1],
    WithinAbs(expected.projection[1][1], TOLERANCE)
  );
}
