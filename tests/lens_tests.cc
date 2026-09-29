#include <catch2/catch_test_macros.hpp>

#include <talus/objectmodel/component/lens/lens.h>

using talus::objectmodel::Lens;
using talus::objectmodel::Pose;

TEST_CASE("a view matrix undoes the camera's pose", "[lens]") {
  Pose pose{};
  pose.position = glm::vec3(3.0f, 4.0f, 5.0f);
  pose.rotation =
    glm::angleAxis(glm::radians(35.0f), glm::vec3(0.0f, 1.0f, 0.0f));

  const glm::vec4 atCamera =
    talus::objectmodel::view(pose) * glm::vec4(pose.position, 1.0f);

  REQUIRE(std::abs(atCamera.x) < 1.0e-4f);
  REQUIRE(std::abs(atCamera.y) < 1.0e-4f);
  REQUIRE(std::abs(atCamera.z) < 1.0e-4f);
}

TEST_CASE("a point in front of the camera lands down the view", "[lens]") {
  Pose pose{};
  pose.position = glm::vec3(0.0f, 0.0f, 10.0f);

  const glm::vec4 ahead =
    talus::objectmodel::view(pose) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

  REQUIRE(ahead.z < 0.0f);
}

TEST_CASE("the projection flips Y for Vulkan", "[lens]") {
  const Lens lens{};

  const glm::mat4 flipped = talus::objectmodel::projection(lens, 1.5f);

  REQUIRE(flipped[1][1] < 0.0f);
}

TEST_CASE("the lens drives the projection it is given", "[lens]") {
  const glm::mat4 wide =
    talus::objectmodel::projection(Lens{90.0f, 0.1f, 100.0f}, 1.0f);

  const glm::mat4 narrow =
    talus::objectmodel::projection(Lens{30.0f, 0.1f, 100.0f}, 1.0f);

  REQUIRE(wide[0][0] < narrow[0][0]);
}

TEST_CASE("a default lens matches what the camera used to hardcode", "[lens]") {
  const Lens lens{};

  REQUIRE(lens.verticalFov == 70.0f);
  REQUIRE(lens.nearPlane == 0.1f);
  REQUIRE(lens.farPlane == 10000.0f);
}
