#include "sampledsdf.ih"

namespace talus::field {

namespace {

constexpr float GRADIENT_FLOOR = 1e-6f;

uint32_t latticeIndex(const SampledSdf& field, const glm::uvec3& at) {
  return at.x + field.resolution.x * (at.y + field.resolution.y * at.z);
}

glm::vec3 maximumCorner(const SampledSdf& field) {
  return field.origin
         + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));
}

float outsideDistance(const SampledSdf& field, const glm::vec3& point) {
  const glm::vec3 clamped =
    glm::clamp(point, field.origin, maximumCorner(field));

  return glm::length(point - clamped);
}

template<typename Real>
struct LatticeCell {
  glm::uvec3 base;
  glm::vec<3, Real> t;
};

using Cell = LatticeCell<float>;

template<typename Real>
struct GridClamp {
  glm::vec<3, Real> clamped;
  glm::vec<3, Real> gap;
  Real outside;
};

glm::uvec3 cellCounts(const SampledSdf& field) {
  return field.resolution - glm::uvec3(1u);
}

uint32_t brickIndex(const glm::uvec3& counts, const glm::uvec3& at) {
  return at.x + counts.x * (at.y + counts.y * at.z);
}

template<typename Real>
void readCorners(
  const SampledSdf& field,
  const glm::uvec3& base,
  Real (&corner)[8]
) {
  for (uint32_t i = 0; i < 8u; ++i) {
    const glm::uvec3 offset(i & 1u, (i >> 1) & 1u, (i >> 2) & 1u);
    corner[i] =
      static_cast<Real>(field.distances[latticeIndex(field, base + offset)]);
  }
}

float cellLipschitz(
  const SampledSdf& field,
  uint32_t lowest,
  uint32_t alongY,
  uint32_t alongZ
) {
  const float corner[8] = {
    field.distances[lowest],
    field.distances[lowest + 1u],
    field.distances[lowest + alongY],
    field.distances[lowest + alongY + 1u],
    field.distances[lowest + alongZ],
    field.distances[lowest + alongZ + 1u],
    field.distances[lowest + alongZ + alongY],
    field.distances[lowest + alongZ + alongY + 1u]
  };

  glm::vec3 slope(0.0f);

  for (uint32_t i = 0; i < 8u; ++i) {
    if ((i & 1u) == 0u) {
      slope.x = std::max(slope.x, std::abs(corner[i | 1u] - corner[i]));
    }
    if ((i & 2u) == 0u) {
      slope.y = std::max(slope.y, std::abs(corner[i | 2u] - corner[i]));
    }
    if ((i & 4u) == 0u) {
      slope.z = std::max(slope.z, std::abs(corner[i | 4u] - corner[i]));
    }
  }

  return glm::length(slope / field.voxelSize);
}

struct CellReach {
  glm::uvec3 minimum{0u};
  glm::uvec3 maximum{0u};
};

CellReach reachOf(const SampledSdf& field, const SampleBox& box) {
  const glm::uvec3 cells = cellCounts(field);
  const glm::uvec3 end = box.minimum + box.extent;

  const glm::uvec3 cellMinimum =
    glm::max(box.minimum, glm::uvec3(1u)) - glm::uvec3(1u);
  const glm::uvec3 cellMaximum =
    glm::min(end - glm::uvec3(1u), cells - glm::uvec3(1u));

  return {
    glm::max(cellMinimum, glm::uvec3(1u)) - glm::uvec3(1u),
    glm::min(cellMaximum + glm::uvec3(1u), cells - glm::uvec3(1u))
  };
}

BrickRange rebuildBricks(
  SampledSdf& field,
  const glm::uvec3& cellMinimum,
  const glm::uvec3& cellMaximum
) {
  const glm::uvec3 cells = cellCounts(field);
  const glm::uvec3 bricks = brickCounts(field);

  const glm::uvec3 first = cellMinimum / glm::uvec3(BRICK_CELLS);
  const glm::uvec3 last = cellMaximum / glm::uvec3(BRICK_CELLS);

  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;

  glm::uvec3 spanStart(0u);
  glm::uvec3 span(0u);
  std::vector<float> lipschitz;

  if (glm::all(glm::lessThanEqual(first, last))) {
    spanStart = glm::max(first * glm::uvec3(BRICK_CELLS), glm::uvec3(1u))
                - glm::uvec3(1u);
    span = glm::min(
             last * glm::uvec3(BRICK_CELLS) + glm::uvec3(BRICK_CELLS + 1u),
             cells
           )
           - spanStart;

    lipschitz.resize(static_cast<size_t>(span.x) * span.y * span.z);

    size_t cached = 0u;

    for (uint32_t z = spanStart.z; z < spanStart.z + span.z; ++z) {
      for (uint32_t y = spanStart.y; y < spanStart.y + span.y; ++y) {
        const uint32_t row = alongY * y + alongZ * z;

        for (uint32_t x = spanStart.x; x < spanStart.x + span.x; ++x) {
          lipschitz[cached] = cellLipschitz(field, row + x, alongY, alongZ);
          ++cached;
        }
      }
    }
  }

  for (uint32_t bz = first.z; bz <= last.z; ++bz) {
    for (uint32_t by = first.y; by <= last.y; ++by) {
      for (uint32_t bx = first.x; bx <= last.x; ++bx) {
        const glm::uvec3 base =
          glm::uvec3(bx, by, bz) * glm::uvec3(BRICK_CELLS);

        const glm::uvec3 start =
          glm::max(base, glm::uvec3(1u)) - glm::uvec3(1u);
        const glm::uvec3 end =
          glm::min(base + glm::uvec3(BRICK_CELLS + 1u), cells);

        float worst = 0.0f;

        for (uint32_t z = start.z; z < end.z; ++z) {
          for (uint32_t y = start.y; y < end.y; ++y) {
            const size_t row =
              (start.x - spanStart.x)
              + static_cast<size_t>(span.x)
                  * ((y - spanStart.y)
                     + static_cast<size_t>(span.y) * (z - spanStart.z));

            for (uint32_t x = start.x; x < end.x; ++x) {
              worst = std::max(worst, lipschitz[row + (x - start.x)]);
            }
          }
        }

        float lowest = std::numeric_limits<float>::max();
        float highest = std::numeric_limits<float>::lowest();

        for (uint32_t z = start.z; z <= end.z; ++z) {
          for (uint32_t y = start.y; y <= end.y; ++y) {
            const uint32_t row = alongY * y + alongZ * z;

            for (uint32_t x = start.x; x <= end.x; ++x) {
              const float value = field.distances[row + x];

              lowest = std::min(lowest, value);
              highest = std::max(highest, value);
            }
          }
        }

        const uint32_t brick = brickIndex(bricks, glm::uvec3(bx, by, bz));

        field.brickMinimum[brick] = lowest;
        field.brickMaximum[brick] = highest;
        field.brickLipschitz[brickIndex(bricks, glm::uvec3(bx, by, bz))] =
          worst;
      }
    }
  }

  field.globalLipschitz = 0.0f;

  for (float bound : field.brickLipschitz) {
    field.globalLipschitz = std::max(field.globalLipschitz, bound);
  }

  return {first, last + glm::uvec3(1u)};
}

void brickBox(
  const SampledSdf& field,
  const glm::uvec3& brick,
  glm::vec3& minimum,
  glm::vec3& maximum
) {
  const glm::uvec3 base = brick * glm::uvec3(BRICK_CELLS);
  const glm::uvec3 end =
    glm::min(base + glm::uvec3(BRICK_CELLS), cellCounts(field));

  minimum = field.origin + field.voxelSize * glm::vec3(base);
  maximum = field.origin + field.voxelSize * glm::vec3(end);
}

template<typename Real>
LatticeCell<Real> locate(
  const SampledSdf& field,
  const glm::vec<3, Real>& point
) {
  using Vector = glm::vec<3, Real>;

  const Vector lattice =
    (point - Vector(field.origin)) / Vector(field.voxelSize);

  const glm::uvec3 base = glm::min(
    glm::uvec3(glm::floor(lattice)),
    field.resolution - glm::uvec3(2u)
  );

  return {base, lattice - Vector(base)};
}

template<typename Real>
Real interpolate(const SampledSdf& field, const LatticeCell<Real>& cell) {
  Real corner[8];
  readCorners(field, cell.base, corner);

  const Real x00 = glm::mix(corner[0], corner[1], cell.t.x);
  const Real x10 = glm::mix(corner[2], corner[3], cell.t.x);
  const Real x01 = glm::mix(corner[4], corner[5], cell.t.x);
  const Real x11 = glm::mix(corner[6], corner[7], cell.t.x);

  const Real y0 = glm::mix(x00, x10, cell.t.y);
  const Real y1 = glm::mix(x01, x11, cell.t.y);

  return glm::mix(y0, y1, cell.t.z);
}

template<typename Real>
glm::vec<3, Real> slope(
  const SampledSdf& field,
  const LatticeCell<Real>& cell
) {
  Real c[8];
  readCorners(field, cell.base, c);

  const Real one = Real(1);
  const Real u = cell.t.x;
  const Real v = cell.t.y;
  const Real w = cell.t.z;

  const glm::vec<3, Real> derivative(
    (c[1] - c[0]) * (one - v) * (one - w) + (c[3] - c[2]) * v * (one - w)
      + (c[5] - c[4]) * (one - v) * w + (c[7] - c[6]) * v * w,
    (c[2] - c[0]) * (one - u) * (one - w) + (c[3] - c[1]) * u * (one - w)
      + (c[6] - c[4]) * (one - u) * w + (c[7] - c[5]) * u * w,
    (c[4] - c[0]) * (one - u) * (one - v) + (c[5] - c[1]) * u * (one - v)
      + (c[6] - c[2]) * (one - u) * v + (c[7] - c[3]) * u * v
  );

  return derivative / glm::vec<3, Real>(field.voxelSize);
}

template<typename Real>
GridClamp<Real> clampToGrid(
  const SampledSdf& field,
  const glm::vec<3, Real>& point
) {
  using Vector = glm::vec<3, Real>;

  const Vector clamped =
    glm::clamp(point, Vector(field.origin), Vector(maximumCorner(field)));
  const Vector gap = point - clamped;

  return {clamped, gap, glm::length(gap)};
}
}

SampleBox merge(const SampleBox& first, const SampleBox& second) {
  if (glm::any(glm::equal(first.extent, glm::uvec3(0u)))) {
    return second;
  }

  if (glm::any(glm::equal(second.extent, glm::uvec3(0u)))) {
    return first;
  }

  const glm::uvec3 minimum = glm::min(first.minimum, second.minimum);

  const glm::uvec3 beyond =
    glm::max(first.minimum + first.extent, second.minimum + second.extent);

  return {minimum, beyond - minimum};
}

glm::vec3 voxelSize(
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution
) {
  return (maximum - minimum) / glm::vec3(resolution - glm::uvec3(1u));
}

SampledSdf bake(
  std::span<const Primitive> primitives,
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution
) {
  if (resolution.x < 2u || resolution.y < 2u || resolution.z < 2u) {
    throw std::runtime_error(
      "A sampled field needs at least two lattice "
      "points on every axis"
    );
  }

  SampledSdf field;
  field.origin = minimum;
  field.resolution = resolution;
  field.voxelSize = voxelSize(minimum, maximum, resolution);

  const size_t count =
    static_cast<size_t>(resolution.x) * resolution.y * resolution.z;

  field.distances.resize(count);
  field.materials.resize(count);

  for (uint32_t z = 0; z < resolution.z; ++z) {
    for (uint32_t y = 0; y < resolution.y; ++y) {
      for (uint32_t x = 0; x < resolution.x; ++x) {
        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(x, y, z);

        const AnalyticSample baked = sample(primitives, point);
        const uint32_t index = latticeIndex(field, glm::uvec3(x, y, z));

        field.distances[index] = baked.distance;
        field.materials[index] = static_cast<uint8_t>(baked.material);
      }
    }
  }

  finishBricks(field);

  return field;
}

float distance(const SampledSdf& field, const glm::vec3& point) {
  const float outside = outsideDistance(field, point);

  if (outside > 0.0f) {
    return outside;
  }

  return interpolate(field, locate(field, point));
}

uint32_t material(const SampledSdf& field, const glm::vec3& point) {
  if (outsideDistance(field, point) > 0.0f) {
    return 0;
  }

  const Cell cell = locate(field, point);

  const glm::uvec3 nearest = cell.base
                             + glm::uvec3(
                               cell.t.x < 0.5f ? 0u : 1u,
                               cell.t.y < 0.5f ? 0u : 1u,
                               cell.t.z < 0.5f ? 0u : 1u
                             );

  return field.materials[latticeIndex(field, nearest)];
}

glm::vec3 gradient(const SampledSdf& field, const glm::vec3& point) {
  const float outside = outsideDistance(field, point);

  if (outside > 0.0f) {
    const glm::vec3 clamped =
      glm::clamp(point, field.origin, maximumCorner(field));

    return (point - clamped) / outside;
  }

  return slope(field, locate(field, point));
}

float probeDistance(const SampledSdf& field, const glm::vec3& point) {
  const GridClamp<float> at = clampToGrid(field, point);
  const float inner = interpolate(field, locate(field, at.clamped));

  return at.outside <= 0.0f ? inner : inner + at.outside;
}

float probeLipschitz(const SampledSdf& field) {
  if (field.brickLipschitz.empty()) {
    return std::numeric_limits<float>::infinity();
  }

  return std::sqrt(field.globalLipschitz * field.globalLipschitz + 1.0f);
}

template<typename Real>
FieldProbeOf<Real> probe(
  const SampledSdf& field,
  const glm::vec<3, Real>& point
) {
  using Vector = glm::vec<3, Real>;

  const GridClamp<Real> at = clampToGrid(field, point);
  const LatticeCell<Real> cell = locate(field, at.clamped);
  const Real inner = interpolate(field, cell);
  const Real measured = at.outside <= Real(0) ? inner : inner + at.outside;

  const Vector derivative = slope(field, cell);
  const Real steepness = glm::length(derivative);
  const Real floor = static_cast<Real>(GRADIENT_FLOOR);

  const bool usable = steepness > floor;

  if (at.outside <= Real(0)) {
    return {
      measured,
      usable ? derivative / steepness : Vector(Real(0), Real(1), Real(0)),
      derivative
    };
  }

  const Vector blended =
    (usable ? derivative / steepness * std::abs(inner) : Vector(Real(0)))
    + at.gap;

  const Real reach = glm::length(blended);

  Vector gradient = derivative;

  for (int axis = 0; axis != 3; ++axis) {
    if (at.gap[axis] != Real(0)) {
      gradient[axis] = at.gap[axis] / at.outside;
    }
  }

  return {
    measured,
    reach > floor ? blended / reach : at.gap / at.outside,
    gradient
  };
}

template FieldProbeOf<float> probe<float>(
  const SampledSdf& field,
  const glm::vec3& point
);

double outsideGapBound(const SampledSdf& field, const glm::dvec3& point) {
  const double unknown = -std::numeric_limits<double>::infinity();

  if (field.resolution.x < 2u || field.resolution.y < 2u
      || field.resolution.z < 2u) {
    return unknown;
  }

  const GridClamp<double> at = clampToGrid(field, point);

  if (!(at.outside > 0.0)) {
    return unknown;
  }

  const glm::uvec3 bricks = brickCounts(field);
  const size_t count = static_cast<size_t>(bricks.x) * bricks.y * bricks.z;

  if (field.brickMinimum.size() != count
      || field.brickLipschitz.size() != count) {
    return unknown;
  }

  const LatticeCell<double> cell = locate(field, at.clamped);
  const uint32_t brick =
    brickIndex(bricks, cell.base / glm::uvec3(BRICK_CELLS));
  const double lowest = field.brickMinimum[brick];
  const double steepest = field.brickLipschitz[brick];
  const glm::dvec3 magnitude = glm::max(
    glm::abs(glm::dvec3(field.origin)),
    glm::abs(glm::dvec3(maximumCorner(field)))
  );
  const double reach =
    lowest + at.outside - 1.0e-9 * (1.0 + std::abs(lowest) + at.outside)
    - 1.0e-6 * steepest
        * (1.0 + std::max({magnitude.x, magnitude.y, magnitude.z}));
  const double slope =
    std::sqrt(1.0 + steepest * steepest) * (1.0 + 1.0e-5) + 1.0e-9;

  return reach > 0.0 ? reach / slope : reach;
}

template FieldProbeOf<double> probe<double>(
  const SampledSdf& field,
  const glm::dvec3& point
);

float stepBound(
  const SampledSdf& field,
  const glm::vec3& point,
  const glm::vec3& direction
) {
  const float outside = outsideDistance(field, point);

  if (outside > 0.0f) {
    return outside;
  }

  const Cell cell = locate(field, point);
  const glm::uvec3 bricks = brickCounts(field);

  const glm::uvec3 brick =
    glm::min(cell.base / glm::uvec3(BRICK_CELLS), bricks - glm::uvec3(1u));

  glm::vec3 minimum(0.0f);
  glm::vec3 maximum(0.0f);
  brickBox(field, brick, minimum, maximum);

  float exit = std::numeric_limits<float>::max();

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1e-8f) {
      continue;
    }

    const float wall = direction[axis] > 0.0f ? maximum[axis] : minimum[axis];

    exit = std::min(exit, (wall - point[axis]) / direction[axis]);
  }

  const glm::vec3 voxel = field.voxelSize;

  exit = std::max(exit, 0.5f * std::min(voxel.x, std::min(voxel.y, voxel.z)));

  const float bound = field.brickLipschitz[brickIndex(bricks, brick)];

  if (bound <= 0.0f) {
    return exit;
  }

  return std::min(std::abs(distance(field, point)) / bound, exit);
}

glm::uvec3 brickCounts(const SampledSdf& field) {
  return (cellCounts(field) + glm::uvec3(BRICK_CELLS - 1u))
         / glm::uvec3(BRICK_CELLS);
}

bool brickHoldsSurface(const SampledSdf& field, uint32_t brick) {
  return field.brickMinimum[brick] <= 0.0f && field.brickMaximum[brick] >= 0.0f;
}

void finishBricks(SampledSdf& field) {
  const glm::uvec3& resolution = field.resolution;

  if (resolution.x < 2u || resolution.y < 2u || resolution.z < 2u) {
    throw std::invalid_argument(
      "A sampled field needs at least two lattice points on every axis"
    );
  }

  if (field.distances.size()
      != static_cast<size_t>(resolution.x) * resolution.y * resolution.z) {
    throw std::invalid_argument(
      "A sampled field needs one distance per lattice point"
    );
  }

  const glm::uvec3 bricks = brickCounts(field);
  const size_t count = static_cast<size_t>(bricks.x) * bricks.y * bricks.z;

  field.brickLipschitz.resize(count);
  field.brickMinimum.resize(count);
  field.brickMaximum.resize(count);

  rebuildBricks(field, glm::uvec3(0u), cellCounts(field) - glm::uvec3(1u));
}

BrickRange brickRange(const SampledSdf& field, const SampleBox& box) {
  const CellReach reach = reachOf(field, box);

  return {
    reach.minimum / glm::uvec3(BRICK_CELLS),
    reach.maximum / glm::uvec3(BRICK_CELLS) + glm::uvec3(1u)
  };
}

WriteReport write(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const float> distances,
  std::span<const uint8_t> materials
) {
  const glm::uvec3 end = box.minimum + box.extent;

  if (
    end.x > field.resolution.x || end.y > field.resolution.y
    || end.z > field.resolution.z
  ) {
    throw std::runtime_error("A write must stay inside the lattice");
  }

  const size_t count =
    static_cast<size_t>(box.extent.x) * box.extent.y * box.extent.z;

  if (distances.size() != count || materials.size() != count) {
    throw std::runtime_error(
      "A write needs one distance and one material per sample in the box"
    );
  }

  if (count == 0u) {
    return {box, glm::uvec3(0u), glm::uvec3(0u)};
  }

  size_t source = 0;

  for (uint32_t z = 0; z < box.extent.z; ++z) {
    for (uint32_t y = 0; y < box.extent.y; ++y) {
      for (uint32_t x = 0; x < box.extent.x; ++x) {
        const uint32_t index =
          latticeIndex(field, box.minimum + glm::uvec3(x, y, z));

        field.distances[index] = distances[source];
        field.materials[index] = materials[source];

        ++source;
      }
    }
  }

  const CellReach reach = reachOf(field, box);

  const BrickRange bricks = rebuildBricks(field, reach.minimum, reach.maximum);

  return {box, bricks.begin, bricks.end};
}

}
