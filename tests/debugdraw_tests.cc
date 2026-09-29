#include <catch2/catch_test_macros.hpp>

#include <talus/debugdraw/debugdraw/debugdraw.h>

#include <glm/glm.hpp>

using talus::debugdraw::DebugDraw;
using talus::debugdraw::MAX_LINES;
using talus::debugdraw::SPHERE_SEGMENTS;

namespace {

const glm::vec4 WHITE(1.0f);

const glm::vec3 ORIGIN(0.0f);
const glm::vec3 UNIT_X(1.0f, 0.0f, 0.0f);

}

TEST_CASE("a line with no duration lives for exactly one advance", "[debugdraw]") {
  DebugDraw draw;

  REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 0.0f, "test"));
  REQUIRE(draw.tested().size() == 2u);

  draw.advance(1.0f / 60.0f);

  REQUIRE(draw.tested().empty());
  REQUIRE(draw.count() == 0u);
}

TEST_CASE("a timed line outlives the advances inside its duration", "[debugdraw]") {
  DebugDraw draw;

  REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 0.5f, "test"));

  draw.advance(0.2f);
  REQUIRE(draw.tested().size() == 2u);

  draw.advance(0.2f);
  REQUIRE(draw.tested().size() == 2u);

  draw.advance(0.2f);
  REQUIRE(draw.tested().empty());
}

TEST_CASE("a hidden category is kept but not handed out", "[debugdraw]") {
  DebugDraw draw;

  REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 1.0f, "a"));
  REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 1.0f, "b"));

  draw.show("a", false);

  REQUIRE_FALSE(draw.shown("a"));
  REQUIRE(draw.shown("b"));
  REQUIRE(draw.count() == 2u);
  REQUIRE(draw.tested().size() == 2u);

  draw.show("a", true);

  REQUIRE(draw.tested().size() == 4u);
}

TEST_CASE("depth-tested and overlay lines are handed out apart", "[debugdraw]") {
  DebugDraw draw;

  const glm::vec3 unitY(0.0f, 1.0f, 0.0f);

  REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 1.0f, "test", true));
  REQUIRE(draw.line(ORIGIN, unitY, WHITE, 1.0f, "test", false));

  REQUIRE(draw.tested().size() == 2u);
  REQUIRE(draw.untested().size() == 2u);

  REQUIRE(draw.tested()[1].position == UNIT_X);
  REQUIRE(draw.untested()[1].position == unitY);
}

TEST_CASE("the cap drops whole primitives and counts them", "[debugdraw]") {
  DebugDraw draw;

  for (size_t index = 0u; index != MAX_LINES; ++index) {
    REQUIRE(draw.line(ORIGIN, UNIT_X, WHITE, 1.0f, "fill"));
  }

  REQUIRE(draw.count() == MAX_LINES);
  REQUIRE_FALSE(draw.sphere(ORIGIN, 1.0f, WHITE, 1.0f, "fill"));
  REQUIRE(draw.count() == MAX_LINES);
  REQUIRE(draw.dropped() == 1u);

  draw.advance(0.0f);

  REQUIRE(draw.dropped() == 0u);
}

TEST_CASE("a sphere is three rings and a box is twelve edges", "[debugdraw]") {
  DebugDraw draw;

  REQUIRE(draw.sphere(ORIGIN, 1.0f, WHITE, 1.0f, "test"));
  REQUIRE(draw.count() == 3u * SPHERE_SEGMENTS);

  REQUIRE(draw.box(ORIGIN, glm::vec3(1.0f), WHITE, 1.0f, "test"));
  REQUIRE(draw.count() == 3u * SPHERE_SEGMENTS + 12u);

  REQUIRE(draw.point(ORIGIN, 0.1f, WHITE, 1.0f, "test"));
  REQUIRE(draw.count() == 3u * SPHERE_SEGMENTS + 15u);
}
