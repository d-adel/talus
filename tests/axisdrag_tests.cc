#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/field/raycast/raycast.h>

#include <glm/glm.hpp>

#include <optional>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::alongAxis;
using talus::field::Ray;

namespace {

constexpr float TOLERANCE = 1e-4f;

const glm::vec3 ANCHOR(0.0f, 0.0f, 10.0f);
const glm::vec3 X_AXIS(1.0f, 0.0f, 0.0f);

Ray fromOriginTowards(float x, float y) {
  return {glm::vec3(0.0f), glm::normalize(glm::vec3(x, y, 10.0f))};
}

}

TEST_CASE("a ray aimed right of the anchor lands at positive x", "[axisdrag]") {
  const std::optional<float> at =
    alongAxis(ANCHOR, X_AXIS, fromOriginTowards(1.0f, 0.0f));

  REQUIRE(at.has_value());
  REQUIRE_THAT(*at, WithinAbs(1.0f, TOLERANCE));
}

TEST_CASE("a ray through the anchor lands at zero", "[axisdrag]") {
  const std::optional<float> at =
    alongAxis(ANCHOR, X_AXIS, fromOriginTowards(0.0f, 0.0f));

  REQUIRE(at.has_value());
  REQUIRE_THAT(*at, WithinAbs(0.0f, TOLERANCE));
}

TEST_CASE("a ray parallel to the axis has no answer", "[axisdrag]") {
  const Ray parallel{glm::vec3(0.0f, 1.0f, 10.0f), X_AXIS};

  REQUIRE_FALSE(alongAxis(ANCHOR, X_AXIS, parallel).has_value());
}

TEST_CASE(
  "sweeping the mouse right moves along the axis monotonically",
  "[axisdrag]"
) {
  float previous = -1.0e9f;

  for (int step = -5; step <= 5; ++step) {
    const std::optional<float> at =
      alongAxis(ANCHOR, X_AXIS, fromOriginTowards(0.5f * step, 0.0f));

    REQUIRE(at.has_value());
    REQUIRE(*at > previous);

    previous = *at;
  }
}

TEST_CASE("a drag is a pure function of the mouse", "[axisdrag]") {
  const glm::vec3 origin(3.0f, 2.0f, 10.0f);

  const std::optional<float> grabbed =
    alongAxis(ANCHOR, X_AXIS, fromOriginTowards(0.5f, 0.0f));

  REQUIRE(grabbed.has_value());

  std::vector<glm::vec3> seen;

  for (int repeat = 0; repeat != 3; ++repeat) {
    const std::optional<float> at =
      alongAxis(ANCHOR, X_AXIS, fromOriginTowards(1.5f, 0.0f));

    REQUIRE(at.has_value());

    seen.push_back(origin + X_AXIS * (*at - *grabbed));
  }

  REQUIRE(seen[0] == seen[1]);
  REQUIRE(seen[1] == seen[2]);
  REQUIRE_THAT(seen[0].x, WithinAbs(4.0f, TOLERANCE));
  REQUIRE_THAT(seen[0].y, WithinAbs(2.0f, TOLERANCE));
  REQUIRE_THAT(seen[0].z, WithinAbs(10.0f, TOLERANCE));
}
