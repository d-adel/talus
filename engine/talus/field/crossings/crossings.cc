#include "crossings.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace talus::field {

namespace {

constexpr float SURFACE_ULPS = 32.0f;

struct LatticeLayout {
  glm::uvec3 size{0u};
  glm::uvec3 stride{0u};
  float tolerance = 0.0f;
};

float surfaceTolerance(const SampledSdf& field) {
  const glm::vec3 upper =
    field.origin
    + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));
  const glm::vec3 corner = glm::max(glm::abs(field.origin), glm::abs(upper));

  return SURFACE_ULPS * std::numeric_limits<float>::epsilon()
         * std::max({corner.x, corner.y, corner.z});
}

void scanSample(
  const SampledSdf& field,
  const LatticeLayout& layout,
  const glm::uvec3& at,
  LatticeSurface& found
) {
  const glm::uvec3& size = layout.size;
  const glm::uvec3& stride = layout.stride;
  const float tolerance = layout.tolerance;

  const uint32_t index = at.x + stride.y * at.y + stride.z * at.z;
  const float here = field.distances[index];
  const bool inside = here < tolerance;

  bool border = false;

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    if (at[axis] == 0u || at[axis] + 1u == size[axis]) {
      border = true;
    } else if (!(field.distances[index - stride[axis]] < tolerance)) {
      border = true;
    }

    if (at[axis] + 1u >= size[axis]) {
      continue;
    }

    const float there = field.distances[index + stride[axis]];

    if (inside == (there < tolerance)) {
      continue;
    }

    border = true;

    glm::vec3 lattice(at);
    lattice[axis] += std::clamp(here / (here - there), 0.0f, 1.0f);

    found.crossings.push_back(Crossing{lattice, axis});
  }

  if (inside && border) {
    found.inside.push_back(at);
  }
}

void scanRun(
  const SampledSdf& field,
  const LatticeLayout& layout,
  uint32_t y,
  uint32_t z,
  uint32_t begin,
  uint32_t end,
  LatticeSurface& found
) {
  for (uint32_t x = begin; x != end; ++x) {
    scanSample(field, layout, glm::uvec3(x, y, z), found);
  }
}

void keepFaceSamples(
  const LatticeLayout& layout,
  uint32_t y,
  uint32_t z,
  uint32_t begin,
  uint32_t end,
  LatticeSurface& found
) {
  const glm::uvec3& size = layout.size;

  const bool face = y == 0u || y + 1u == size.y || z == 0u || z + 1u == size.z;

  for (uint32_t x = begin; x != end; ++x) {
    if (face || x == 0u || x + 1u == size.x) {
      found.inside.push_back(glm::uvec3(x, y, z));
    }
  }
}

bool bricksCoverLattice(const SampledSdf& field, const glm::uvec3& bricks) {
  const size_t count = static_cast<size_t>(bricks.x) * bricks.y * bricks.z;

  return count != 0u && field.brickMinimum.size() == count
         && field.brickMaximum.size() == count;
}

}

LatticeSurface latticeSurface(const SampledSdf& field) {
  LatticeSurface found;

  const glm::uvec3 size = field.resolution;

  const uint32_t alongY = size.x;
  const uint32_t alongZ = size.x * size.y;

  const float tolerance = surfaceTolerance(field);

  const LatticeLayout layout{size, glm::uvec3(1u, alongY, alongZ), tolerance};
  const glm::uvec3 bricks = brickCounts(field);

  if (!bricksCoverLattice(field, bricks)) {
    for (uint32_t z = 0u; z != size.z; ++z) {
      for (uint32_t y = 0u; y != size.y; ++y) {
        scanRun(field, layout, y, z, 0u, size.x, found);
      }
    }

    return found;
  }

  for (uint32_t z = 0u; z != size.z; ++z) {
    const uint32_t brickZ = std::min(z / BRICK_CELLS, bricks.z - 1u);

    for (uint32_t y = 0u; y != size.y; ++y) {
      const uint32_t brickY = std::min(y / BRICK_CELLS, bricks.y - 1u);
      const uint32_t row = bricks.x * (brickY + bricks.y * brickZ);

      for (uint32_t brickX = 0u; brickX != bricks.x; ++brickX) {
        const uint32_t begin = brickX * BRICK_CELLS;
        const uint32_t end =
          brickX + 1u == bricks.x ? size.x : begin + BRICK_CELLS;
        const uint32_t brick = row + brickX;

        if (!(field.brickMinimum[brick] < tolerance)) {
          continue;
        }

        if (field.brickMaximum[brick] < tolerance) {
          keepFaceSamples(layout, y, z, begin, end, found);
          continue;
        }

        scanRun(field, layout, y, z, begin, end, found);
      }
    }
  }

  return found;
}

glm::vec3 latticeToField(const SampledSdf& field, const glm::vec3& lattice) {
  return field.origin + field.voxelSize * lattice;
}

}
