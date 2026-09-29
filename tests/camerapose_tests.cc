#include <talus/view/camera/camera.h>
#include <talus/view/lookthrough/lookthrough.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <glm/gtc/quaternion.hpp>

using Catch::Matchers::WithinAbs;
using talus::view::Camera;

namespace {

constexpr float TOLERANCE = 1e-3f;

glm::vec3 aim(const Camera& camera) {
  return camera.pose().rotation * glm::vec3(0.0f, 0.0f, -1.0f);
}

void sameDirection(const glm::vec3& left, const glm::vec3& right) {
  const glm::vec3 a = glm::normalize(left);
  const glm::vec3 b = glm::normalize(right);

  REQUIRE_THAT(a.x, WithinAbs(b.x, TOLERANCE));
  REQUIRE_THAT(a.y, WithinAbs(b.y, TOLERANCE));
  REQUIRE_THAT(a.z, WithinAbs(b.z, TOLERANCE));
}

}

TEST_CASE("framing points the camera at what it framed", "[camera]") {
  Camera camera;

  const glm::vec3 centre(4.0f, 1.0f, -2.0f);

  camera.frame(centre, 3.0f);

  sameDirection(centre - camera.eye(), aim(camera));
  REQUIRE(glm::length(centre - camera.eye()) > 3.0f);
  REQUIRE(camera.placed());
}

TEST_CASE("framing something larger stands further back", "[camera]") {
  Camera near;
  Camera far;

  const glm::vec3 centre(0.0f);

  near.frame(centre, 1.0f);
  far.frame(centre, 8.0f);

  REQUIRE(glm::length(centre - far.eye()) > glm::length(centre - near.eye()));
}

TEST_CASE("orbiting keeps what was framed in the middle", "[camera]") {
  Camera camera;

  const glm::vec3 centre(2.0f, 0.5f, 1.0f);

  camera.frame(centre, 2.0f);

  const glm::vec3 before = camera.eye();
  const float reach = glm::length(centre - before);

  camera.orbit(0.6f, 0.2f);

  REQUIRE(glm::length(camera.eye() - before) > TOLERANCE);
  REQUIRE_THAT(glm::length(centre - camera.eye()), WithinAbs(reach, TOLERANCE));
  sameDirection(centre - camera.eye(), aim(camera));
}

TEST_CASE("zooming moves along the aim without turning", "[camera]") {
  Camera camera;

  const glm::vec3 centre(0.0f, 0.0f, 0.0f);

  camera.frame(centre, 2.0f);

  const glm::vec3 heading = aim(camera);
  const float reach = glm::length(centre - camera.eye());

  camera.zoom(1.0f);

  REQUIRE(glm::length(centre - camera.eye()) < reach);
  sameDirection(aim(camera), heading);
  sameDirection(centre - camera.eye(), heading);
}

TEST_CASE("panning slides the camera without turning it", "[camera]") {
  Camera camera;

  camera.frame(glm::vec3(0.0f), 2.0f);

  const glm::vec3 heading = aim(camera);
  const glm::vec3 before = camera.eye();

  camera.pan(1.0f, 0.0f);

  REQUIRE(glm::length(camera.eye() - before) > TOLERANCE);
  sameDirection(aim(camera), heading);
}

TEST_CASE("placeFrom looks the way the seat looks", "[camera]") {
  Camera camera;

  const glm::quat turned =
    glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

  const talus::objectmodel::Pose seat{glm::vec3(5.0f, 2.0f, 0.0f), turned};

  camera.placeFrom(seat, 6.0f);

  sameDirection(aim(camera), seat.rotation * glm::vec3(0.0f, 0.0f, -1.0f));

  REQUIRE_THAT(camera.eye().x, WithinAbs(seat.position.x, TOLERANCE));
  REQUIRE_THAT(camera.eye().y, WithinAbs(seat.position.y, TOLERANCE));
  REQUIRE_THAT(camera.eye().z, WithinAbs(seat.position.z, TOLERANCE));
}

TEST_CASE("a reset camera has forgotten where it was put", "[camera]") {
  Camera camera;

  REQUIRE_FALSE(camera.placed());

  camera.frame(glm::vec3(9.0f), 2.0f);

  REQUIRE(camera.placed());

  camera.reset();

  REQUIRE_FALSE(camera.placed());
  REQUIRE_THAT(camera.eye().x, WithinAbs(0.0f, TOLERANCE));
}

TEST_CASE("a viewport driven by the camera sees from its eye", "[camera]") {
  talus::objectmodel::World world;

  Camera camera;
  camera.frame(glm::vec3(1.0f, 2.0f, 3.0f), 4.0f);

  talus::view::Viewport port{};
  port.pose = camera.pose();
  port.lens = camera.lens();

  const auto seen = talus::view::lookThrough(port, world, 1.6f);

  REQUIRE_THAT(seen.position.x, WithinAbs(camera.eye().x, TOLERANCE));
  REQUIRE_THAT(seen.position.y, WithinAbs(camera.eye().y, TOLERANCE));
  REQUIRE_THAT(seen.position.z, WithinAbs(camera.eye().z, TOLERANCE));

  REQUIRE_THAT(seen.nearPlane, WithinAbs(camera.lens().nearPlane, TOLERANCE));
}
