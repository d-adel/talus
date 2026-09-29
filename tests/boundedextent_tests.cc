#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/field.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>

#include "tolerances.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <optional>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::Aabb;
using talus::field::Primitive;

namespace {

constexpr uint32_t SMOOTH_UNION = 1;
constexpr uint32_t INTERSECTION = 2;
constexpr uint32_t SUBTRACTION = 3;
constexpr uint32_t SMOOTH_SUBTRACTION = 4;

void requireExtent(
  const std::optional<Aabb>& extent,
  const glm::vec3& minimum,
  const glm::vec3& maximum
) {
  REQUIRE(extent.has_value());

  REQUIRE_THAT(extent->minimum.x, WithinAbs(minimum.x, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(extent->minimum.y, WithinAbs(minimum.y, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(extent->minimum.z, WithinAbs(minimum.z, ANALYTIC_TOLERANCE));

  REQUIRE_THAT(extent->maximum.x, WithinAbs(maximum.x, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(extent->maximum.y, WithinAbs(maximum.y, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(extent->maximum.z, WithinAbs(maximum.z, ANALYTIC_TOLERANCE));
}

}

TEST_CASE("an axis-aligned box bounds its half extents", "[bounds]") {
  const std::vector<Primitive> primitives{talus::field::makeBox(
    glm::vec3(0.0f, -0.5f, 0.0f),
    glm::vec3(10.0f, 0.5f, 10.0f)
  )};

  requireExtent(
    talus::field::boundedExtent(primitives),
    glm::vec3(-10.0f, -1.0f, -10.0f),
    glm::vec3(10.0f, 0.0f, 10.0f)
  );
}

TEST_CASE(
  "a rotated box bounds the turned extents, not the diagonal",
  "[bounds]"
) {
  const float diagonal = 0.5f * (std::sqrt(2.0f) / 2.0f) * 2.0f;

  const std::vector<Primitive> primitives{talus::field::makeBox(
    glm::vec3(0.0f),
    glm::vec3(0.5f),
    glm::radians(45.0f),
    glm::vec3(0.0f, 1.0f, 0.0f)
  )};

  requireExtent(
    talus::field::boundedExtent(primitives),
    glm::vec3(-diagonal, -0.5f, -diagonal),
    glm::vec3(diagonal, 0.5f, diagonal)
  );
}

TEST_CASE("a sphere bounds its radius exactly", "[bounds]") {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(1.0f, 2.0f, 3.0f), 0.5f)
  };

  requireExtent(
    talus::field::boundedExtent(primitives),
    glm::vec3(0.5f, 1.5f, 2.5f),
    glm::vec3(1.5f, 2.5f, 3.5f)
  );
}

TEST_CASE("the extent contains every corner of a turned box", "[bounds]") {
  const glm::vec3 halfExtents(0.4f, 1.2f, 0.7f);
  const glm::mat4 rotation = glm::rotate(
    glm::mat4(1.0f),
    glm::radians(37.0f),
    glm::normalize(glm::vec3(0.3f, 1.0f, -0.6f))
  );

  const std::vector<Primitive> primitives{talus::field::makeBox(
    glm::vec3(2.0f, -1.0f, 0.5f),
    halfExtents,
    glm::radians(37.0f),
    glm::normalize(glm::vec3(0.3f, 1.0f, -0.6f))
  )};

  const std::optional<Aabb> extent = talus::field::boundedExtent(primitives);
  REQUIRE(extent.has_value());

  for (int corner = 0; corner != 8; ++corner) {
    const glm::vec3 sign(
      (corner & 1) ? 1.0f : -1.0f,
      (corner & 2) ? 1.0f : -1.0f,
      (corner & 4) ? 1.0f : -1.0f
    );

    const glm::vec3 point =
      glm::vec3(2.0f, -1.0f, 0.5f)
      + glm::vec3(rotation * glm::vec4(sign * halfExtents, 0.0f));

    REQUIRE(point.x >= extent->minimum.x - ANALYTIC_TOLERANCE);
    REQUIRE(point.y >= extent->minimum.y - ANALYTIC_TOLERANCE);
    REQUIRE(point.z >= extent->minimum.z - ANALYTIC_TOLERANCE);

    REQUIRE(point.x <= extent->maximum.x + ANALYTIC_TOLERANCE);
    REQUIRE(point.y <= extent->maximum.y + ANALYTIC_TOLERANCE);
    REQUIRE(point.z <= extent->maximum.z + ANALYTIC_TOLERANCE);
  }
}

TEST_CASE("a blend reaches past the shape on every axis", "[bounds]") {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f, 0, SMOOTH_UNION, 0.4f)
  };

  requireExtent(
    talus::field::boundedExtent(primitives),
    glm::vec3(-1.4f),
    glm::vec3(1.4f)
  );
}

TEST_CASE("an unbounded primitive contributes nothing", "[bounds]") {
  const std::vector<Primitive> planeOnly{
    talus::field::makePlane(glm::vec3(0.0f, -1.0f, 0.0f))
  };

  REQUIRE_FALSE(talus::field::boundedExtent(planeOnly).has_value());
  REQUIRE_FALSE(talus::field::boundedExtent({}).has_value());

  const std::vector<Primitive> mixed{
    talus::field::makePlane(glm::vec3(0.0f, -1.0f, 0.0f)),
    talus::field::makeSphere(glm::vec3(0.0f), 2.0f)
  };

  requireExtent(
    talus::field::boundedExtent(mixed),
    glm::vec3(-2.0f),
    glm::vec3(2.0f)
  );
}

TEST_CASE("the grid box carries the margin past the extent", "[bounds]") {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(0.0f), 0.5f)
  };

  const float reach = 0.5f + talus::core::FIELD_GRID_MARGIN;
  const Aabb box = talus::objectmodel::gridBox(primitives);

  REQUIRE_THAT(box.minimum.x, WithinAbs(-reach, ANALYTIC_TOLERANCE));
  REQUIRE_THAT(box.maximum.y, WithinAbs(reach, ANALYTIC_TOLERANCE));
}

TEST_CASE("a carve reaching outside does not grow the extent", "[bounds]") {
  const glm::vec3 unitLow(-1.0f);
  const glm::vec3 unitHigh(1.0f);

  const std::vector<Primitive> hard{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f),
    talus::field::makeSphere(glm::vec3(2.0f, 0.0f, 0.0f), 1.0f, 0, SUBTRACTION)
  };

  requireExtent(talus::field::boundedExtent(hard), unitLow, unitHigh);

  const std::vector<Primitive> smooth{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f),
    talus::field::makeSphere(
      glm::vec3(0.0f, 2.0f, 0.0f),
      1.0f,
      0,
      SMOOTH_SUBTRACTION,
      0.3f
    )
  };

  requireExtent(talus::field::boundedExtent(smooth), unitLow, unitHigh);
}

TEST_CASE("an intersection does not grow the extent", "[bounds]") {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f),
    talus::field::makeBox(
      glm::vec3(0.0f),
      glm::vec3(9.0f),
      0.0f,
      glm::vec3(0.0f, 1.0f, 0.0f),
      0,
      INTERSECTION
    )
  };

  requireExtent(
    talus::field::boundedExtent(primitives),
    glm::vec3(-1.0f),
    glm::vec3(1.0f)
  );
}

TEST_CASE("an all-subtractive list bounds nothing", "[bounds]") {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f, 0, SUBTRACTION)
  };

  REQUIRE_FALSE(talus::field::boundedExtent(primitives).has_value());
}
