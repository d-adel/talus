#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <talus/field/field.h>
#include <talus/field/raycast/raycast.h>
#include <talus/objectmodel/component/pose/pose.h>

#include "tolerances.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::Primitive;
using talus::field::Ray;
using talus::field::RayHit;
using talus::objectmodel::model;
using talus::objectmodel::Pose;

namespace {

Primitive makeSphere(const glm::vec3& centre, float radius, uint32_t material) {
  Primitive primitive{};
  primitive.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), centre));
  primitive.shape = glm::vec4(radius, 0.0f, 0.0f, 0.0f);
  primitive.shapeConfig = glm::uvec4(0, material, 0, 0);

  return primitive;
}

Ray toLocal(const Pose& pose, const Ray& world) {
  const glm::mat4 inverseModel = glm::inverse(model(pose));

  return Ray{
    glm::vec3(inverseModel * glm::vec4(world.origin, 1.0f)),
    glm::vec3(inverseModel * glm::vec4(world.direction, 0.0f))
  };
}

}

TEST_CASE("a pose puts an off-centre primitive where the turn says", "[pose]") {
  Pose pose{};
  pose.position = glm::vec3(3.0f, 0.0f, 0.0f);
  pose.rotation =
    glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

  const std::vector<Primitive> primitives{
    makeSphere(glm::vec3(2.0f, 0.0f, 0.0f), 1.0f, 3)
  };

  const Ray at{glm::vec3(3.0f, 0.0f, -6.0f), glm::vec3(0.0f, 0.0f, 1.0f)};

  const std::optional<RayHit> hit =
    talus::field::raymarch(primitives, toLocal(pose, at));

  REQUIRE(hit.has_value());
  REQUIRE(hit->material == 3);

  const glm::vec3 world =
    glm::vec3(model(pose) * glm::vec4(hit->position, 1.0f));

  REQUIRE_THAT(world.x, WithinAbs(3.0f, MARCH_TOLERANCE));
  REQUIRE_THAT(world.y, WithinAbs(0.0f, MARCH_TOLERANCE));
  REQUIRE_THAT(world.z, WithinAbs(-3.0f, MARCH_TOLERANCE));

  const Ray beside{glm::vec3(10.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f)};

  REQUIRE_FALSE(
    talus::field::raymarch(primitives, toLocal(pose, beside)).has_value()
  );
}

TEST_CASE("a point crossed into a pose and back comes home", "[pose]") {
  Pose pose{};
  pose.position = glm::vec3(-2.0f, 0.7f, 4.0f);
  pose.rotation = glm::angleAxis(
    glm::radians(37.0f),
    glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f))
  );

  const glm::mat4 forward = model(pose);
  const glm::mat4 inverseModel = glm::inverse(forward);

  for (const glm::vec3 point :
       {glm::vec3(0.0f),
        glm::vec3(1.0f, -2.0f, 3.0f),
        glm::vec3(-5.0f, 0.25f, 0.0f)}) {
    const glm::vec3 local = glm::vec3(inverseModel * glm::vec4(point, 1.0f));
    const glm::vec3 home = glm::vec3(forward * glm::vec4(local, 1.0f));

    REQUIRE_THAT(home.x, WithinAbs(point.x, ANALYTIC_TOLERANCE));
    REQUIRE_THAT(home.y, WithinAbs(point.y, ANALYTIC_TOLERANCE));
    REQUIRE_THAT(home.z, WithinAbs(point.z, ANALYTIC_TOLERANCE));
  }
}

TEST_CASE("a default pose is the identity", "[pose]") {
  REQUIRE(model(Pose{}) == glm::mat4(1.0f));
}

TEST_CASE(
  "a pose reproduces the matrix it replaced, to a float ulp",
  "[pose]"
) {
  const float angle = glm::radians(-90.0f);
  const glm::vec3 axis(1.0f, 0.0f, 0.0f);
  const glm::vec3 position(0.0f, 0.0f, -2.0f);

  const glm::mat4 stored = glm::translate(glm::mat4(1.0f), position)
                           * glm::rotate(glm::mat4(1.0f), angle, axis);

  const glm::mat4 posed = model(Pose{position, glm::angleAxis(angle, axis)});

  float worst = 0.0f;

  for (int column = 0; column != 4; ++column) {
    for (int row = 0; row != 4; ++row) {
      worst =
        std::max(worst, std::abs(stored[column][row] - posed[column][row]));
    }
  }

  REQUIRE(worst < 2.0e-07f);
}

TEST_CASE("a pose is valid only with a unit rotation", "[pose]") {
  REQUIRE(talus::objectmodel::valid(talus::objectmodel::Pose{}));
  talus::objectmodel::Pose turned{};
  turned.rotation =
    glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  REQUIRE(talus::objectmodel::valid(turned));
  talus::objectmodel::Pose stretched{};
  stretched.rotation = glm::quat(2.0f, 0.0f, 0.0f, 0.0f);
  REQUIRE_FALSE(talus::objectmodel::valid(stretched));
  talus::objectmodel::Pose collapsed{};
  collapsed.rotation = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
  REQUIRE_FALSE(talus::objectmodel::valid(collapsed));
}
