#include <talus/view/viewport/viewport.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <type_traits>

using Catch::Matchers::WithinAbs;

namespace {

constexpr float TOLERANCE = 1e-6f;

}

TEST_CASE("a viewport crosses the ABI as plain bytes", "[viewport]") {
  static_assert(
    std::is_trivially_copyable_v<talus::view::Viewport>,
    "a viewport is marshalled to C# by copying its bytes"
  );

  static_assert(
    std::is_standard_layout_v<talus::view::Viewport>,
    "a viewport is marshalled to C# by copying its bytes"
  );

  REQUIRE(std::is_aggregate_v<talus::view::Viewport>);
}

TEST_CASE(
  "an unconfigured viewport names no target and no camera",
  "[viewport]"
) {
  const talus::view::Viewport port{};

  REQUIRE(port.target == talus::view::INVALID_TARGET);
  REQUIRE(port.camera == talus::objectmodel::INVALID_ENTITY);
}

TEST_CASE("a viewport carries a camera of its own", "[viewport]") {
  const talus::view::Viewport port{};

  REQUIRE_THAT(port.pose.position.x, WithinAbs(0.0f, TOLERANCE));
  REQUIRE_THAT(port.pose.position.y, WithinAbs(0.0f, TOLERANCE));
  REQUIRE_THAT(port.pose.position.z, WithinAbs(0.0f, TOLERANCE));

  REQUIRE_THAT(port.lens.verticalFov, WithinAbs(70.0f, TOLERANCE));
  REQUIRE_THAT(port.lens.nearPlane, WithinAbs(0.1f, TOLERANCE));
  REQUIRE_THAT(port.lens.farPlane, WithinAbs(10000.0f, TOLERANCE));
}

TEST_CASE("an unconfigured viewport draws the whole world", "[viewport]") {
  const talus::view::Viewport port{};

  REQUIRE(port.mode == talus::view::DrawMode::Both);
  REQUIRE(drawsMeshes(port.mode));
  REQUIRE(drawsSdf(port.mode));

  REQUIRE(port.fieldRepresentation == talus::core::FIELD_SAMPLED);
  REQUIRE_THAT(port.supersample, WithinAbs(1.0f, TOLERANCE));
  REQUIRE_FALSE(port.gridVisible);
}

TEST_CASE("a viewport's march settings default to the build's", "[viewport]") {
  const talus::view::MarchSettings march{};

  REQUIRE_THAT(march.epsilon, WithinAbs(TALUS_MARCH_EPSILON, TOLERANCE));
  REQUIRE_THAT(
    march.maxDistance,
    WithinAbs(TALUS_MARCH_MAX_DISTANCE, TOLERANCE)
  );
  REQUIRE_THAT(march.omega, WithinAbs(TALUS_MARCH_OMEGA, TOLERANCE));
  REQUIRE_THAT(
    march.gradientEpsilon,
    WithinAbs(TALUS_GRADIENT_EPSILON, TOLERANCE)
  );
  REQUIRE_THAT(
    march.shadowMaxDistance,
    WithinAbs(TALUS_SHADOW_MAX_DISTANCE, TOLERANCE)
  );
  REQUIRE_THAT(
    march.shadowSharpness,
    WithinAbs(TALUS_SHADOW_SHARPNESS, TOLERANCE)
  );

  REQUIRE(march.maxIterations == TALUS_MARCH_MAX_ITERATIONS);
}
