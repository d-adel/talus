#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/field.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>

#include "tolerances.h"

#include <span>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::Aabb;
using talus::field::Primitive;
using talus::objectmodel::SdfGrid;

namespace {

std::vector<Primitive> sphere(float radius) {
  return {talus::field::makeSphere(glm::vec3(0.0f), radius)};
}

SdfGrid fitted(std::span<const Primitive> primitives, uint32_t resolution) {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(resolution);

  talus::objectmodel::fitToPrimitives(grid, primitives);

  return grid;
}

float marginInCells(float radius, uint32_t resolution) {
  const std::vector<Primitive> solid = sphere(radius);

  const SdfGrid grid = fitted(solid, resolution);

  const float margin = talus::objectmodel::gridMargin(grid, solid);

  const float coarsest =
    std::max({grid.voxelSize.x, grid.voxelSize.y, grid.voxelSize.z});

  return margin / coarsest;
}

}

TEST_CASE("an unauthored margin is the same cells at any size", "[bounds]") {
  const float cells = float(talus::core::FIELD_GRID_MARGIN_CELLS);

  REQUIRE_THAT(marginInCells(0.15f, 33u), WithinAbs(cells, MARCH_TOLERANCE));
  REQUIRE_THAT(marginInCells(1.5f, 33u), WithinAbs(cells, MARCH_TOLERANCE));
  REQUIRE_THAT(marginInCells(12.0f, 33u), WithinAbs(cells, MARCH_TOLERANCE));
}

TEST_CASE(
  "an unauthored margin is the same cells at any resolution",
  "[bounds]"
) {
  const float cells = float(talus::core::FIELD_GRID_MARGIN_CELLS);

  REQUIRE_THAT(marginInCells(0.15f, 17u), WithinAbs(cells, MARCH_TOLERANCE));
  REQUIRE_THAT(marginInCells(0.15f, 65u), WithinAbs(cells, MARCH_TOLERANCE));
  REQUIRE_THAT(marginInCells(0.15f, 128u), WithinAbs(cells, MARCH_TOLERANCE));
}

TEST_CASE("a finer grid pads a smaller distance", "[bounds]") {
  const std::vector<Primitive> solid = sphere(0.15f);

  const float coarse =
    talus::objectmodel::gridMargin(fitted(solid, 17u), solid);

  const float fine = talus::objectmodel::gridMargin(fitted(solid, 128u), solid);

  REQUIRE(fine < coarse);
  REQUIRE(fine > 0.0f);
}

TEST_CASE("an authored margin wins over the fitted one", "[bounds]") {
  const std::vector<Primitive> solid = sphere(0.15f);

  SdfGrid grid = fitted(solid, 33u);
  grid.margin = 2.0f;

  REQUIRE_THAT(
    talus::objectmodel::gridMargin(grid, solid),
    WithinAbs(2.0f, ANALYTIC_TOLERANCE)
  );
}

TEST_CASE("the caster box never sits inside the grid box", "[bounds]") {
  for (const float radius : {0.15f, 1.5f, 12.0f}) {
    const std::vector<Primitive> solid = sphere(radius);

    const SdfGrid grid = fitted(solid, 33u);

    const Aabb caster = talus::objectmodel::casterBox(solid, grid);

    const Aabb box = talus::objectmodel::gridBox(
      solid,
      talus::objectmodel::gridMargin(grid, solid)
    );

    REQUIRE(caster.maximum.x >= box.maximum.x);
    REQUIRE(caster.minimum.y <= box.minimum.y);
  }
}

TEST_CASE(
  "a lattice without primitives casts from its lattice box",
  "[bounds]"
) {
  for (const float radius : {0.15f, 1.5f, 12.0f}) {
    const std::vector<Primitive> solid = sphere(radius);

    SdfGrid grid = fitted(solid, 33u);
    grid.margin = talus::objectmodel::gridMargin(grid, solid);

    const Aabb fromPrimitives = talus::objectmodel::casterBox(solid, grid);

    const Aabb fromLattice = talus::objectmodel::casterBox({}, grid);

    for (glm::length_t axis = 0; axis != 3; ++axis) {
      REQUIRE_THAT(
        fromLattice.minimum[axis],
        WithinAbs(
          fromPrimitives.minimum[axis],
          ANALYTIC_TOLERANCE * (1.0f + radius)
        )
      );
      REQUIRE_THAT(
        fromLattice.maximum[axis],
        WithinAbs(
          fromPrimitives.maximum[axis],
          ANALYTIC_TOLERANCE * (1.0f + radius)
        )
      );
    }
  }
}

TEST_CASE(
  "a lattice caster box pads to an authored shadow cull margin",
  "[bounds]"
) {
  SdfGrid grid{};
  grid.resolution = glm::uvec3(9u, 5u, 17u);
  grid.voxelSize = glm::vec3(0.25f, 0.5f, 0.125f);
  grid.origin = glm::vec4(-1.0f, 2.0f, 3.0f, 1.0f);
  grid.margin = 0.2f;
  grid.shadowCullMargin = 1.5f;

  const Aabb caster = talus::objectmodel::casterBox({}, grid);

  const glm::vec3 low(-1.0f, 2.0f, 3.0f);
  const glm::vec3 high(1.0f, 4.0f, 5.0f);
  const float pad = 1.5f - 0.2f;

  for (glm::length_t axis = 0; axis != 3; ++axis) {
    REQUIRE_THAT(
      caster.minimum[axis],
      WithinAbs(low[axis] - pad, ANALYTIC_TOLERANCE)
    );
    REQUIRE_THAT(
      caster.maximum[axis],
      WithinAbs(high[axis] + pad, ANALYTIC_TOLERANCE)
    );
  }
}

TEST_CASE("an unauthored grid stores no margin of its own", "[bounds]") {
  const SdfGrid grid{};

  REQUIRE(!grid.margin.has_value());
  REQUIRE(!grid.shadowCullMargin.has_value());
}
