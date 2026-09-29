#include <catch2/catch_test_macros.hpp>

#include <talus/renderer/sdfrecord/sdfrecord.h>
#include <talus/renderer/shadowgrid/shadowgrid.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

using talus::renderer::RecordBounds;
using talus::renderer::SHADOW_GRID_CELLS;
using talus::renderer::SHADOW_GRID_INITIAL_INDICES;
using talus::renderer::ShadowCell;
using talus::renderer::ShadowGrid;
using talus::renderer::ShadowGridUniform;

namespace {

const glm::vec3 SUN_OVERHEAD(0.0f, 1.0f, 0.0f);

const glm::vec3 SUN_SLANTED = glm::normalize(glm::vec3(0.4f, 1.0f, 0.3f));

RecordBounds boxAt(const glm::vec3& low, const glm::vec3& high) {
  return {glm::vec4(low, 0.0f), glm::vec4(high, 0.0f)};
}

RecordBounds pointAt(const glm::vec3& at) {
  return boxAt(at, at);
}

RecordBounds unplaced() {
  return {glm::vec4(1.0f), glm::vec4(-1.0f)};
}

bool binned(const ShadowGrid& grid) {
  const ShadowGridUniform& uniform = grid.uniform();

  return uniform.cell.z == float(SHADOW_GRID_CELLS)
         && uniform.cell.w == float(SHADOW_GRID_CELLS)
         && std::isfinite(uniform.cell.x) && uniform.cell.x > 0.0f
         && std::isfinite(uniform.cell.y) && uniform.cell.y > 0.0f
         && grid.cells().size() == SHADOW_GRID_CELLS * SHADOW_GRID_CELLS;
}

uint32_t cellAlong(float across, float inverseSize) {
  const float at = std::floor(across * inverseSize);

  return static_cast<uint32_t>(
    std::clamp(at, 0.0f, float(SHADOW_GRID_CELLS - 1u))
  );
}

bool listedAt(const ShadowGrid& grid, const glm::vec3& point, uint32_t record) {
  const ShadowGridUniform& uniform = grid.uniform();

  const uint32_t column = cellAlong(
    glm::dot(point, glm::vec3(uniform.axisU)) - uniform.axisU.w,
    uniform.cell.x
  );
  const uint32_t row = cellAlong(
    glm::dot(point, glm::vec3(uniform.axisV)) - uniform.axisV.w,
    uniform.cell.y
  );

  const ShadowCell& cell = grid.cells()[row * SHADOW_GRID_CELLS + column];
  const std::span<const uint32_t> listed =
    grid.indices().subspan(cell.offset, cell.count);

  return std::find(listed.begin(), listed.end(), record) != listed.end();
}

bool everyRecordListedAtItsCorners(
  const ShadowGrid& grid,
  std::span<const RecordBounds> bounds,
  uint32_t stride = 1u
) {
  for (uint32_t record = 0u; record < bounds.size(); record += stride) {
    const glm::vec3 low(bounds[record].minimum);
    const glm::vec3 high(bounds[record].maximum);

    if (glm::any(glm::greaterThan(low, high))) {
      continue;
    }

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::vec3 at(
        (corner & 1u) != 0u ? high.x : low.x,
        (corner & 2u) != 0u ? high.y : low.y,
        (corner & 4u) != 0u ? high.z : low.z
      );

      if (!listedAt(grid, at, record)) {
        return false;
      }
    }
  }

  return true;
}

uint32_t fullestCell(const ShadowGrid& grid) {
  uint32_t fullest = 0u;

  for (const ShadowCell& cell : grid.cells()) {
    fullest = std::max(fullest, cell.count);
  }

  return fullest;
}

}

TEST_CASE("the shadow grid bins a world with no records", "[shadowgrid]") {
  ShadowGrid grid;

  grid.build({}, 0u, SUN_SLANTED);

  REQUIRE(binned(grid));
  REQUIRE(grid.indices().empty());
  REQUIRE(fullestCell(grid) == 0u);
}

TEST_CASE("the shadow grid bins records that have no bounds", "[shadowgrid]") {
  const std::vector<RecordBounds> bounds(16u, unplaced());

  ShadowGrid grid;

  grid.build(bounds, static_cast<uint32_t>(bounds.size()), SUN_SLANTED);

  REQUIRE(binned(grid));
  REQUIRE(grid.indices().empty());
}

TEST_CASE(
  "the shadow grid bins a world whose records all sit at one point",
  "[shadowgrid]"
) {
  const glm::vec3 at(3.0f, -2.0f, 7.5f);
  const std::vector<RecordBounds> bounds(1000u, pointAt(at));

  for (const glm::vec3& sun : {SUN_OVERHEAD, SUN_SLANTED}) {
    ShadowGrid grid;

    grid.build(bounds, static_cast<uint32_t>(bounds.size()), sun);

    REQUIRE(binned(grid));
    REQUIRE(grid.indices().size() == bounds.size());
    REQUIRE(fullestCell(grid) == bounds.size());
    REQUIRE(everyRecordListedAtItsCorners(grid, bounds));
  }
}

TEST_CASE(
  "the shadow grid bins records stacked on one vertical line",
  "[shadowgrid]"
) {
  std::vector<RecordBounds> bounds;

  for (uint32_t record = 0u; record != 64u; ++record) {
    bounds.push_back(pointAt(glm::vec3(1.0f, float(record), -4.0f)));
  }

  ShadowGrid grid;

  grid.build(bounds, static_cast<uint32_t>(bounds.size()), SUN_OVERHEAD);

  REQUIRE(binned(grid));
  REQUIRE(grid.indices().size() == bounds.size());
  REQUIRE(everyRecordListedAtItsCorners(grid, bounds));
}

TEST_CASE(
  "a degenerate span takes the smallest record's size as its cell",
  "[shadowgrid]"
) {
  std::vector<RecordBounds> bounds;

  for (uint32_t record = 0u; record != 32u; ++record) {
    const float x = 0.5f * float(record);

    bounds.push_back(
      boxAt(glm::vec3(x, 0.0f, 2.0f), glm::vec3(x + 0.25f, 0.0f, 2.0f))
    );
  }

  ShadowGrid grid;

  grid.build(bounds, static_cast<uint32_t>(bounds.size()), SUN_OVERHEAD);

  REQUIRE(binned(grid));
  REQUIRE(grid.uniform().cell.x == 1.0f / 0.25f);
  REQUIRE(everyRecordListedAtItsCorners(grid, bounds));
}

TEST_CASE(
  "the shadow grid bins past its initial index budget",
  "[shadowgrid]"
) {
  const uint32_t side = 400u;

  std::vector<RecordBounds> bounds;
  bounds.reserve(side * side);

  for (uint32_t row = 0u; row != side; ++row) {
    for (uint32_t column = 0u; column != side; ++column) {
      const glm::vec3 low(0.6f * float(column), 0.0f, 0.6f * float(row));

      bounds.push_back(boxAt(low - 0.4f, low + 0.4f));
    }
  }

  ShadowGrid grid;

  grid.build(bounds, static_cast<uint32_t>(bounds.size()), SUN_SLANTED);

  REQUIRE(binned(grid));
  REQUIRE(grid.indices().size() > SHADOW_GRID_INITIAL_INDICES);
  REQUIRE(fullestCell(grid) < bounds.size() / 100u);
  REQUIRE(everyRecordListedAtItsCorners(grid, bounds, 101u));
}
