#include "fieldshape.ih"

namespace talus::physics {

namespace {

using talus::field::SampledSdf;

constexpr float DENSITY = 1000.0f;

constexpr float DIRECTION_FLOOR = 1.0e-6f;

constexpr uint32_t SURFACE_STEPS = 8u;
constexpr float SURFACE_TOLERANCE = 1.0e-5f;

constexpr uint32_t NO_SLOT = UINT32_MAX;

constexpr float FACING = 0.70710678f;

constexpr float REACH_ULPS = 32.0f;

glm::vec3 toGlm(JPH::Vec3Arg v) {
  return glm::vec3(v.GetX(), v.GetY(), v.GetZ());
}

JPH::Vec3 toJph(const glm::vec3& v) {
  return JPH::Vec3(v.x, v.y, v.z);
}

glm::vec3 gridMaximum(const SampledSdf& field) {
  return field.origin
         + field.voxelSize * glm::vec3(field.resolution - glm::uvec3(1u));
}

glm::vec3 brickOrigin(const SampledSdf& field, const glm::uvec3& brick) {
  return field.origin
         + field.voxelSize * glm::vec3(brick * talus::field::BRICK_CELLS);
}

glm::vec3 latticePoint(const SampledSdf& field, const glm::uvec3& at) {
  return field.origin + field.voxelSize * glm::vec3(at);
}

float coarsestVoxel(const SampledSdf& field) {
  return std::max({field.voxelSize.x, field.voxelSize.y, field.voxelSize.z});
}

glm::vec3 ontoSurface(const SampledSdf& field, glm::vec3 point) {
  for (uint32_t step = 0; step != SURFACE_STEPS; ++step) {
    const talus::field::FieldProbe hit = talus::field::probe(field, point);

    if (std::fabs(hit.distance) < SURFACE_TOLERANCE) {
      break;
    }

    point -= hit.distance * hit.normal;
  }

  return point;
}

float nearestPossible(const SampledSdf& field, const glm::vec3& point) {
  const glm::vec3 clamped = glm::clamp(point, field.origin, gridMaximum(field));

  const float outside = glm::length(point - clamped);
  const float inner = talus::field::distance(field, clamped);

  return std::max(outside, inner - outside)
         - std::sqrt(3.0f) * coarsestVoxel(field);
}

glm::vec3 latticeExtent(const SampledSdf& field) {
  return glm::abs(field.origin) + glm::abs(gridMaximum(field));
}

struct Reach {
  float distance = 0.0f;
  float rounding = 0.0f;

  bool within(float value, float magnitude) const {
    return value < distance + rounding * magnitude;
  }
};

Reach reachFor(const SampledSdf& field) {
  const glm::vec3 scale = latticeExtent(field) / field.voxelSize;

  return {
    JPH::PhysicsSettings().mSpeculativeContactDistance,
    REACH_ULPS * std::numeric_limits<float>::epsilon()
      * (1.0f + std::max({scale.x, scale.y, scale.z}))
  };
}

float brickMagnitude(const SampledSdf& field, uint32_t index) {
  return std::max(
    std::fabs(field.brickMinimum[index]),
    std::fabs(field.brickMaximum[index])
  );
}

void include(ReachCells& reached, const glm::uvec3& at) {
  reached.minimum = glm::min(reached.minimum, at);
  reached.maximum = glm::max(reached.maximum, at);
}

void appendCellSeeds(
  const SampledSdf& field,
  const glm::uvec3& brick,
  uint32_t index,
  const Reach& reach,
  std::vector<FieldSeed>& seeds,
  ReachCells& reached
) {
  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);

  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;

  const glm::uvec3 base = brick * glm::uvec3(talus::field::BRICK_CELLS);
  const glm::uvec3 last =
    glm::min(base + glm::uvec3(talus::field::BRICK_CELLS), cells);

  const float magnitude = brickMagnitude(field, index);

  for (uint32_t z = base.z; z < last.z; ++z) {
    for (uint32_t y = base.y; y < last.y; ++y) {
      for (uint32_t x = base.x; x < last.x; ++x) {
        const uint32_t lowest = x + alongY * y + alongZ * z;

        const std::array<float, 8> corners{
          field.distances[lowest],
          field.distances[lowest + 1u],
          field.distances[lowest + alongY],
          field.distances[lowest + alongY + 1u],
          field.distances[lowest + alongZ],
          field.distances[lowest + alongZ + 1u],
          field.distances[lowest + alongZ + alongY],
          field.distances[lowest + alongZ + alongY + 1u]
        };

        const auto [low, high] =
          std::minmax_element(corners.begin(), corners.end());

        if (reach.within(*low, magnitude)) {
          include(reached, glm::uvec3(x, y, z));
        }

        if (
          !(*low < 0.0f && *high >= 0.0f) && !(*low <= 0.0f && *high > 0.0f)
        ) {
          continue;
        }

        const glm::vec3 centre =
          field.origin
          + field.voxelSize * (glm::vec3(x, y, z) + glm::vec3(0.5f));

        const glm::vec3 onSurface = ontoSurface(field, centre);

        seeds.push_back(
          FieldSeed{
            onSurface,
            index,
            talus::field::probe(field, onSurface).normal
          }
        );
      }
    }
  }
}

ReachCells reachCellsOf(
  const SampledSdf& field,
  const glm::uvec3& brick,
  uint32_t index,
  const Reach& reach
) {
  const glm::uvec3 base = brick * glm::uvec3(talus::field::BRICK_CELLS);
  const glm::uvec3 last = glm::min(
    base + glm::uvec3(talus::field::BRICK_CELLS),
    field.resolution - glm::uvec3(1u)
  );

  if (field.brickMaximum[index] < 0.0f) {
    return {base, last - glm::uvec3(1u)};
  }

  const float magnitude = brickMagnitude(field, index);

  if (!reach.within(field.brickMinimum[index], magnitude)) {
    return {};
  }

  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;

  ReachCells samples;

  for (uint32_t z = base.z; z <= last.z; ++z) {
    for (uint32_t y = base.y; y <= last.y; ++y) {
      for (uint32_t x = base.x; x <= last.x; ++x) {
        if (
          reach.within(field.distances[x + alongY * y + alongZ * z], magnitude)
        ) {
          include(samples, glm::uvec3(x, y, z));
        }
      }
    }
  }

  if (glm::any(glm::greaterThan(samples.minimum, samples.maximum))) {
    return {};
  }

  return {
    glm::max(samples.minimum, base + glm::uvec3(1u)) - glm::uvec3(1u),
    glm::min(samples.maximum, last - glm::uvec3(1u))
  };
}

JPH::AABox reachBox(
  const SampledSdf& field,
  std::span<const ReachCells> bricks,
  const glm::vec3& centre
) {
  ReachCells reached;

  for (const ReachCells& brick : bricks) {
    reached.minimum = glm::min(reached.minimum, brick.minimum);
    reached.maximum = glm::max(reached.maximum, brick.maximum);
  }

  if (glm::any(glm::greaterThan(reached.minimum, reached.maximum))) {
    return JPH::AABox();
  }

  const glm::vec3 extent = latticeExtent(field);
  const glm::vec3 pad(
    REACH_ULPS * std::numeric_limits<float>::epsilon()
    * std::max({extent.x, extent.y, extent.z})
  );

  return JPH::AABox(
    toJph(latticePoint(field, reached.minimum) - centre - pad),
    toJph(latticePoint(field, reached.maximum + glm::uvec3(1u)) - centre + pad)
  );
}

struct AxisSums {
  double count = 0.0;
  double first = 0.0;
  double second = 0.0;
};

AxisSums axisSums(float origin, float voxel, uint32_t begin, uint32_t end) {
  AxisSums sums;

  for (uint32_t i = begin; i != end; ++i) {
    const double at = double(origin + voxel * (float(i) + 0.5f));

    sums.count += 1.0;
    sums.first += at;
    sums.second += at * at;
  }

  return sums;
}

SolidIntegral integrateBrick(const SampledSdf& field, uint32_t brick) {
  SolidIntegral solid;

  if (field.brickMinimum[brick] > 0.0f) {
    return solid;
  }

  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);
  const glm::uvec3 bricks = talus::field::brickCounts(field);

  const float cellVolume =
    field.voxelSize.x * field.voxelSize.y * field.voxelSize.z;
  const double cellMass = double(cellVolume) * DENSITY;

  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;

  const glm::uvec3 at(
    brick % bricks.x,
    (brick / bricks.x) % bricks.y,
    brick / (bricks.x * bricks.y)
  );

  const glm::uvec3 base = at * glm::uvec3(talus::field::BRICK_CELLS);
  const glm::uvec3 last =
    glm::min(base + glm::uvec3(talus::field::BRICK_CELLS), cells);

  if (field.brickMaximum[brick] < 0.0f) {
    const AxisSums sx =
      axisSums(field.origin.x, field.voxelSize.x, base.x, last.x);
    const AxisSums sy =
      axisSums(field.origin.y, field.voxelSize.y, base.y, last.y);
    const AxisSums sz =
      axisSums(field.origin.z, field.voxelSize.z, base.z, last.z);

    const double xx = sx.second * sy.count * sz.count;
    const double yy = sx.count * sy.second * sz.count;
    const double zz = sx.count * sy.count * sz.second;

    solid.mass = cellMass * sx.count * sy.count * sz.count;

    solid.firstMoment = cellMass
                        * glm::dvec3(
                          sx.first * sy.count * sz.count,
                          sx.count * sy.first * sz.count,
                          sx.count * sy.count * sz.first
                        );

    solid.secondMoment[0][0] = cellMass * (yy + zz);
    solid.secondMoment[1][1] = cellMass * (xx + zz);
    solid.secondMoment[2][2] = cellMass * (xx + yy);
    solid.secondMoment[0][1] = -cellMass * sx.first * sy.first * sz.count;
    solid.secondMoment[0][2] = -cellMass * sx.first * sy.count * sz.first;
    solid.secondMoment[1][2] = -cellMass * sx.count * sy.first * sz.first;

    return solid;
  }

  for (uint32_t z = base.z; z < last.z; ++z) {
    for (uint32_t y = base.y; y < last.y; ++y) {
      for (uint32_t x = base.x; x < last.x; ++x) {
        const uint32_t lowest = x + alongY * y + alongZ * z;

        const float corners = field.distances[lowest]
                              + field.distances[lowest + 1u]
                              + field.distances[lowest + alongY]
                              + field.distances[lowest + alongY + 1u]
                              + field.distances[lowest + alongZ]
                              + field.distances[lowest + alongZ + 1u]
                              + field.distances[lowest + alongZ + alongY]
                              + field.distances[lowest + alongZ + alongY + 1u];

        if (corners >= 0.0f) {
          continue;
        }

        const glm::vec3 centre =
          field.origin
          + field.voxelSize * (glm::vec3(x, y, z) + glm::vec3(0.5f));

        solid.mass += cellMass;

        const glm::dvec3 p(centre);

        solid.firstMoment += cellMass * p;

        solid.secondMoment[0][0] += cellMass * (p.y * p.y + p.z * p.z);
        solid.secondMoment[1][1] += cellMass * (p.x * p.x + p.z * p.z);
        solid.secondMoment[2][2] += cellMass * (p.x * p.x + p.y * p.y);
        solid.secondMoment[0][1] -= cellMass * p.x * p.y;
        solid.secondMoment[0][2] -= cellMass * p.x * p.z;
        solid.secondMoment[1][2] -= cellMass * p.y * p.z;
      }
    }
  }

  return solid;
}

bool inRange(
  uint32_t brick,
  const glm::uvec3& counts,
  const glm::uvec3& begin,
  const glm::uvec3& end
) {
  const glm::uvec3 at(
    brick % counts.x,
    (brick / counts.x) % counts.y,
    brick / (counts.x * counts.y)
  );

  return glm::all(glm::greaterThanEqual(at, begin))
         && glm::all(glm::lessThan(at, end));
}

bool samplesHold(
  const SampledSdf& field,
  const glm::uvec3& brickBegin,
  const glm::uvec3& brickEnd,
  uint8_t material
) {
  if (glm::any(glm::greaterThanEqual(brickBegin, brickEnd))) {
    return true;
  }

  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);

  const glm::uvec3 first =
    glm::min(brickBegin * glm::uvec3(talus::field::BRICK_CELLS), cells);
  const glm::uvec3 last =
    glm::min(brickEnd * glm::uvec3(talus::field::BRICK_CELLS), cells);

  for (uint32_t z = first.z; z <= last.z; ++z) {
    for (uint32_t y = first.y; y <= last.y; ++y) {
      const auto row =
        field.materials.begin()
        + (first.x + field.resolution.x * (y + field.resolution.y * z));
      const auto rowEnd = row + (last.x - first.x + 1u);

      if (
        std::find_if(
          row,
          rowEnd,
          [material](uint8_t held) { return held != material; }
        )
        != rowEnd
      ) {
        return false;
      }
    }
  }

  return true;
}

SolidIntegral totalOf(std::span<const SolidIntegral> bricks) {
  SolidIntegral solid;

  for (const SolidIntegral& brick : bricks) {
    solid.mass += brick.mass;
    solid.firstMoment += brick.firstMoment;
    solid.secondMoment += brick.secondMoment;
  }

  return solid;
}

bool boxesTouch(
  const glm::vec3& lowA,
  const glm::vec3& highA,
  const glm::vec3& lowB,
  const glm::vec3& highB
) {
  return glm::all(glm::lessThanEqual(lowA, highB))
         && glm::all(glm::lessThanEqual(lowB, highA));
}

template<typename Visitor>
void forEachNearSeed(
  const FieldShape& seedShape,
  const FieldShape& probeShape,
  JPH::Mat44Arg intoProbe,
  const glm::vec3& overlapMinimum,
  const glm::vec3& overlapMaximum,
  float reach,
  Visitor&& visit
) {
  const std::span<const FieldSeed> seeds = seedShape.seeds();

  for (const SurfaceBrick& brick : seedShape.surfaceBricks()) {
    if (!boxesTouch(
          brick.minimum,
          brick.maximum,
          overlapMinimum,
          overlapMaximum
        )) {
      continue;
    }

    const glm::vec3 centre = 0.5f * (brick.minimum + brick.maximum);
    const float radius = 0.5f * glm::length(brick.maximum - brick.minimum);

    const glm::vec3 probed =
      toGlm(intoProbe * toJph(centre - seedShape.centerOfMass()))
      + probeShape.centerOfMass();

    if (nearestPossible(probeShape.field(), probed) > radius + reach) {
      continue;
    }

    for (uint32_t index = brick.begin; index != brick.end; ++index) {
      const FieldSeed& seed = seeds[index];

      if (
        glm::any(glm::lessThan(seed.point, overlapMinimum))
        || glm::any(glm::greaterThan(seed.point, overlapMaximum))
      ) {
        continue;
      }

      visit(index, seed);
    }
  }
}

JPH::AABox overlapIn(
  const FieldShape& near,
  const FieldShape& far,
  JPH::Mat44Arg centerOfMassNear,
  JPH::Mat44Arg centerOfMassFar,
  float separation
) {
  JPH::AABox brought = far.GetLocalBounds().Transformed(
    centerOfMassNear.Inversed() * centerOfMassFar
  );

  brought.ExpandBy(JPH::Vec3::sReplicate(separation));

  JPH::AABox clipped = near.GetLocalBounds();

  clipped.mMin = JPH::Vec3::sMax(clipped.mMin, brought.mMin);
  clipped.mMax = JPH::Vec3::sMin(clipped.mMax, brought.mMax);

  clipped.Translate(toJph(near.centerOfMass()));

  return clipped;
}

constexpr float HALF_VOXEL = 0.5f;
constexpr float HULL_TOLERANCE_VOXELS = 0.1f;

constexpr size_t CONTACT_FACE_VERTICES =
  JPH::Shape::SupportingFace::Capacity / 2u;

float hullTolerance(const SampledSdf& field) {
  return HULL_TOLERANCE_VOXELS
         * std::min({field.voxelSize.x, field.voxelSize.y, field.voxelSize.z});
}

struct SlicePoint {
  double u = 0.0;
  double v = 0.0;
  uint32_t index = 0u;
};

double turn(const SlicePoint& a, const SlicePoint& b, const SlicePoint& c) {
  return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
}

std::vector<SlicePoint> outline(std::vector<SlicePoint>& points) {
  std::sort(
    points.begin(),
    points.end(),
    [](const SlicePoint& a, const SlicePoint& b) {
      return a.u < b.u || (a.u == b.u && a.v < b.v);
    }
  );

  if (points.size() < 3u) {
    return points;
  }

  std::vector<SlicePoint> hull(2u * points.size());
  size_t count = 0u;

  for (const SlicePoint& point : points) {
    while (count >= 2u
           && turn(hull[count - 2u], hull[count - 1u], point) <= 0.0) {
      --count;
    }

    hull[count++] = point;
  }

  const size_t lower = count + 1u;

  for (size_t index = points.size() - 1u; index-- != 0u;) {
    while (count >= lower
           && turn(hull[count - 2u], hull[count - 1u], points[index]) <= 0.0) {
      --count;
    }

    hull[count++] = points[index];
  }

  hull.resize(count - 1u);

  return hull;
}

bool withinOutline(
  const std::vector<SlicePoint>& hull,
  std::span<const SlicePoint> points,
  uint32_t judged
) {
  const size_t edges = hull.size();

  if (edges < 3u) {
    return true;
  }

  std::vector<std::array<double, 3>> inward(edges);

  for (size_t edge = 0u; edge != edges; ++edge) {
    const SlicePoint& a = hull[edge];
    const SlicePoint& b = hull[(edge + 1u) % edges];

    const double du = b.u - a.u;
    const double dv = b.v - a.v;
    const double length = std::sqrt(du * du + dv * dv);

    inward[edge] = {-dv / length, du / length, (dv * a.u - du * a.v) / length};
  }

  size_t last = 0u;

  for (const SlicePoint& point : points) {
    if (point.index >= judged) {
      continue;
    }

    const auto depth = [&](size_t edge) {
      return inward[edge][0] * point.u + inward[edge][1] * point.v
             + inward[edge][2];
    };

    if (depth(last) <= HALF_VOXEL) {
      continue;
    }

    bool near = false;

    for (size_t edge = 0u; edge != edges; ++edge) {
      if (depth(edge) <= HALF_VOXEL) {
        last = edge;
        near = true;
        break;
      }
    }

    if (!near) {
      return false;
    }
  }

  return true;
}

struct LatticePlane {
  glm::vec3 normal{0.0f};
  float offset = 0.0f;
};

std::vector<LatticePlane> latticePlanes(
  const SampledSdf& field,
  const JPH::ConvexHullShape& hull
) {
  const glm::vec3 centre = toGlm(hull.GetCenterOfMass());

  std::vector<LatticePlane> planes;
  planes.reserve(hull.GetPlanes().size());

  for (const JPH::Plane& plane : hull.GetPlanes()) {
    const glm::vec3 normal = toGlm(plane.GetNormal());
    const glm::vec3 scaled = normal * field.voxelSize;
    const float length = glm::length(scaled);

    if (length <= 0.0f) {
      continue;
    }

    planes.push_back(
      LatticePlane{
        scaled / length,
        (plane.GetConstant() + glm::dot(normal, field.origin - centre)) / length
      }
    );
  }

  return planes;
}

bool crossingsOnHull(
  std::span<const LatticePlane> planes,
  std::span<const talus::field::Crossing> crossings
) {
  for (const talus::field::Crossing& crossing : crossings) {
    float highest = std::numeric_limits<float>::lowest();

    for (const LatticePlane& plane : planes) {
      const float height =
        glm::dot(plane.normal, crossing.lattice) + plane.offset;

      if (height > HALF_VOXEL) {
        return false;
      }

      highest = std::max(highest, height);
    }

    if (highest < -HALF_VOXEL) {
      return false;
    }
  }

  return true;
}

glm::vec3 surfacePoint(
  const talus::field::LatticeSurface& surface,
  uint32_t index
) {
  const uint32_t crossings = static_cast<uint32_t>(surface.crossings.size());

  return index < crossings ? surface.crossings[index].lattice
                           : glm::vec3(surface.inside[index - crossings]);
}

JPH::Vec3 hullOffset(const FieldShape& shape) {
  return shape.hull()->GetCenterOfMass() - shape.GetCenterOfMass();
}

JPH::Mat44 hullAt(const FieldShape& shape, JPH::Mat44Arg centerOfMass) {
  return centerOfMass.PreTranslated(hullOffset(shape));
}

struct HullSide {
  const JPH::ConvexHullShape* hull = nullptr;
  JPH::Mat44 at = JPH::Mat44::sIdentity();
  float tolerance = 0.0f;
};

HullSide hullSide(const FieldShape& shape, JPH::Mat44Arg at) {
  return {shape.hull(), at, hullTolerance(shape.field())};
}

bool collectsFaces(const JPH::CollideSettingsBase& settings) {
  return settings.mCollectFacesMode == JPH::ECollectFacesMode::CollectFaces;
}

void widenFace(const HullSide& side, JPH::Shape::SupportingFace& face) {
  const JPH::uint count = face.size();

  if (count < 3u) {
    return;
  }

  JPH::Vec3 centre = JPH::Vec3::sZero();

  for (const JPH::Vec3& corner : face) {
    centre += corner;
  }

  centre /= static_cast<float>(count);

  JPH::Vec3 normal = JPH::Vec3::sZero();

  for (JPH::uint index = 0u; index != count; ++index) {
    normal += (face[index] - centre).Cross(face[(index + 1u) % count] - centre);
  }

  if (!(normal.LengthSq() > 0.0f)) {
    return;
  }

  normal = normal.Normalized();

  thread_local std::vector<JPH::Vec3> candidates;

  candidates.assign(face.begin(), face.end());

  for (JPH::uint point = 0u; point != side.hull->GetNumPoints(); ++point) {
    const JPH::Vec3 at = side.at * side.hull->GetPoint(point);

    if (std::fabs(normal.Dot(at - centre)) > side.tolerance) {
      continue;
    }

    if (std::find(face.begin(), face.end(), at) != face.end()) {
      continue;
    }

    candidates.push_back(at);
  }

  if (candidates.size() == count) {
    return;
  }

  const JPH::Vec3 across = normal.GetNormalizedPerpendicular();
  const JPH::Vec3 along = normal.Cross(across);

  thread_local std::vector<SlicePoint> inPlane;

  inPlane.clear();

  for (size_t index = 0u; index != candidates.size(); ++index) {
    const JPH::Vec3 offset = candidates[index] - centre;

    inPlane.push_back(
      SlicePoint{
        double(offset.Dot(across)),
        double(offset.Dot(along)),
        static_cast<uint32_t>(index)
      }
    );
  }

  const std::vector<SlicePoint> polygon = outline(inPlane);
  const size_t corners = polygon.size();

  if (corners < 3u || corners > CONTACT_FACE_VERTICES) {
    return;
  }

  size_t start = 0u;
  double widest = std::numeric_limits<double>::lowest();

  for (size_t corner = 0u; corner != corners; ++corner) {
    const double spread = turn(
      polygon[corner],
      polygon[(corner + 1u) % corners],
      polygon[(corner + 2u) % corners]
    );

    if (spread > widest) {
      widest = spread;
      start = corner;
    }
  }

  face.clear();

  for (size_t step = 0u; step != corners; ++step) {
    const JPH::Vec3 kept = candidates[polygon[(start + step) % corners].index];

    face.push_back(kept - normal * normal.Dot(kept - centre));
  }
}

class CollideFaceCollector final : public JPH::CollideShapeCollector {
public:
  CollideFaceCollector(
    JPH::CollideShapeCollector& caller,
    const HullSide& first,
    const HullSide& second
  )
      : JPH::CollideShapeCollector(caller),
        m_caller(caller),
        m_first(first),
        m_second(second) {}

  CollideFaceCollector(const CollideFaceCollector&) = delete;
  CollideFaceCollector& operator=(const CollideFaceCollector&) = delete;

  void AddHit(const JPH::CollideShapeResult& hit) override {
    JPH::CollideShapeResult widened = hit;

    widenFace(m_first, widened.mShape1Face);
    widenFace(m_second, widened.mShape2Face);

    m_caller.AddHit(widened);

    ResetEarlyOutFraction(m_caller.GetEarlyOutFraction());
  }

private:
  JPH::CollideShapeCollector& m_caller;
  HullSide m_first;
  HullSide m_second;
};

struct HullSpan {
  float high = std::numeric_limits<float>::lowest();
  JPH::Vec3 highest = JPH::Vec3::sZero();
  float low = std::numeric_limits<float>::max();
};

struct PlacedHull {
  std::vector<JPH::Vec3> points;
  float radius = 0.0f;
};

void placeHull(const HullSide& side, PlacedHull& placed) {
  placed.points.clear();

  for (JPH::uint point = 0u; point != side.hull->GetNumPoints(); ++point) {
    placed.points.push_back(side.at * side.hull->GetPoint(point));
  }

  placed.radius = side.hull->GetConvexRadius();
}

HullSpan hullSpan(const PlacedHull& placed, JPH::Vec3Arg axis) {
  HullSpan span;

  for (const JPH::Vec3& at : placed.points) {
    const float along = at.Dot(axis);

    if (along > span.high) {
      span.high = along;
      span.highest = at;
    }

    span.low = std::min(span.low, along);
  }

  span.high += placed.radius;
  span.highest += placed.radius * axis;
  span.low -= placed.radius;

  return span;
}

float hullOverlap(
  const PlacedHull& first,
  const PlacedHull& second,
  JPH::Vec3Arg axis
) {
  return hullSpan(first, axis).high - hullSpan(second, axis).low;
}

void hullAxes(
  const HullSide& side,
  std::vector<JPH::Vec3>& normals,
  std::vector<JPH::Vec3>& edges
) {
  for (const JPH::Plane& plane : side.hull->GetPlanes()) {
    normals.push_back(side.at.Multiply3x3(plane.GetNormal()));
  }

  const size_t points = side.hull->GetNumPoints();

  thread_local std::vector<uint8_t> seen;

  seen.assign(points * points, 0u);

  std::array<JPH::uint, JPH::ConvexHullShape::cMaxPointsInHull> corners{};

  for (JPH::uint face = 0u; face != side.hull->GetNumFaces(); ++face) {
    const JPH::uint count = side.hull->GetFaceVertices(
      face,
      static_cast<JPH::uint>(corners.size()),
      corners.data()
    );

    for (JPH::uint corner = 0u; corner != count; ++corner) {
      const JPH::uint from = corners[corner];
      const JPH::uint to = corners[(corner + 1u) % count];
      const size_t edge =
        static_cast<size_t>(std::min(from, to)) * points + std::max(from, to);

      if (seen[edge] != 0u) {
        continue;
      }

      seen[edge] = 1u;

      edges.push_back(
        side.at.Multiply3x3(side.hull->GetPoint(to) - side.hull->GetPoint(from))
      );
    }
  }
}

HullAxis separatingAxis(
  const HullSide& first,
  const PlacedHull& firstPlaced,
  const HullSide& second,
  const PlacedHull& secondPlaced,
  JPH::Vec3Arg reported
) {
  thread_local std::vector<JPH::Vec3> candidates;
  thread_local std::vector<JPH::Vec3> firstEdges;
  thread_local std::vector<JPH::Vec3> secondEdges;

  candidates.assign(1u, reported);
  firstEdges.clear();
  secondEdges.clear();

  hullAxes(first, candidates, firstEdges);
  hullAxes(second, candidates, secondEdges);

  for (const JPH::Vec3& one : firstEdges) {
    for (const JPH::Vec3& two : secondEdges) {
      candidates.push_back(one.Cross(two));
    }
  }

  HullAxis best{reported, std::numeric_limits<float>::max(), candidates.size()};

  for (const JPH::Vec3& candidate : candidates) {
    if (!(candidate.LengthSq() > DIRECTION_FLOOR * DIRECTION_FLOOR)) {
      continue;
    }

    const JPH::Vec3 unit = candidate.Normalized();

    for (const JPH::Vec3& axis : {unit, -unit}) {
      const float overlap = hullOverlap(firstPlaced, secondPlaced, axis);

      if (overlap < best.overlap) {
        best.overlap = overlap;
        best.axis = axis;
      }
    }
  }

  return best;
}

JPH::CollideShapeResult consistentHit(
  const JPH::CollideShapeResult& hit,
  const HullSide& first,
  const HullSide& second,
  float tolerance,
  bool faces
) {
  const JPH::Vec3 reported =
    hit.mPenetrationAxis.NormalizedOr(JPH::Vec3::sZero());

  if (reported.IsNearZero()) {
    return hit;
  }

  thread_local PlacedHull firstPlaced;
  thread_local PlacedHull secondPlaced;

  placeHull(first, firstPlaced);
  placeHull(second, secondPlaced);

  if (
    std::fabs(
      hullOverlap(firstPlaced, secondPlaced, reported) - hit.mPenetrationDepth
    )
    <= tolerance
  ) {
    return hit;
  }

  const JPH::Vec3 axis =
    separatingAxis(first, firstPlaced, second, secondPlaced, reported).axis;
  const HullSpan span = hullSpan(firstPlaced, axis);
  const float depth = span.high - hullSpan(secondPlaced, axis).low;

  JPH::CollideShapeResult repaired = hit;

  repaired.mPenetrationAxis = axis;
  repaired.mPenetrationDepth = depth;
  repaired.mContactPointOn1 = span.highest;
  repaired.mContactPointOn2 = span.highest - depth * axis;

  if (faces) {
    repaired.mShape1Face.clear();
    repaired.mShape2Face.clear();

    first.hull->GetSupportingFace(
      JPH::SubShapeID(),
      first.at.Multiply3x3Transposed(-axis),
      JPH::Vec3::sReplicate(1.0f),
      first.at,
      repaired.mShape1Face
    );
    second.hull->GetSupportingFace(
      JPH::SubShapeID(),
      second.at.Multiply3x3Transposed(axis),
      JPH::Vec3::sReplicate(1.0f),
      second.at,
      repaired.mShape2Face
    );
  }

  return repaired;
}

class HullHitCollector final : public JPH::CollideShapeCollector {
public:
  HullHitCollector(
    JPH::CollideShapeCollector& caller,
    const HullSide& first,
    const HullSide& second,
    float tolerance,
    bool faces
  )
      : JPH::CollideShapeCollector(caller),
        m_caller(caller),
        m_first(first),
        m_second(second),
        m_tolerance(tolerance),
        m_faces(faces) {}

  HullHitCollector(const HullHitCollector&) = delete;
  HullHitCollector& operator=(const HullHitCollector&) = delete;

  void AddHit(const JPH::CollideShapeResult& hit) override {
    m_caller.AddHit(
      consistentHit(hit, m_first, m_second, m_tolerance, m_faces)
    );

    ResetEarlyOutFraction(m_caller.GetEarlyOutFraction());
  }

private:
  JPH::CollideShapeCollector& m_caller;
  HullSide m_first;
  HullSide m_second;
  float m_tolerance = 0.0f;
  bool m_faces = false;
};

class CastFaceCollector final : public JPH::CastShapeCollector {
public:
  CastFaceCollector(
    JPH::CastShapeCollector& caller,
    const HullSide& moving,
    JPH::Vec3Arg direction,
    const HullSide& fixed
  )
      : JPH::CastShapeCollector(caller),
        m_caller(caller),
        m_moving(moving),
        m_direction(direction),
        m_fixed(fixed) {}

  CastFaceCollector(const CastFaceCollector&) = delete;
  CastFaceCollector& operator=(const CastFaceCollector&) = delete;

  void AddHit(const JPH::ShapeCastResult& hit) override {
    JPH::Mat44 reached = m_moving.at;
    reached.SetTranslation(
      reached.GetTranslation() + hit.mFraction * m_direction
    );

    JPH::ShapeCastResult widened = hit;

    widenFace(
      HullSide{m_moving.hull, m_fixed.at * reached, m_moving.tolerance},
      widened.mShape1Face
    );
    widenFace(m_fixed, widened.mShape2Face);

    m_caller.AddHit(widened);

    ResetEarlyOutFraction(m_caller.GetEarlyOutFraction());
  }

private:
  JPH::CastShapeCollector& m_caller;
  HullSide m_moving;
  JPH::Vec3 m_direction;
  HullSide m_fixed;
};

struct FieldHit {
  JPH::Vec3 contactPointOn1;
  JPH::Vec3 contactPointOn2;
  JPH::Vec3 penetrationAxis;
  float penetrationDepth = 0.0f;
  JPH::SubShapeID subShapeID1;
  JPH::SubShapeID subShapeID2;
  JPH::BodyID bodyID2;

  JPH::CollideShapeResult result() const {
    return JPH::CollideShapeResult(
      contactPointOn1,
      contactPointOn2,
      penetrationAxis,
      penetrationDepth,
      subShapeID1,
      subShapeID2,
      bodyID2
    );
  }
};

[[nodiscard]] bool collideOneWay(
  const FieldShape& seedShape,
  const FieldShape& probeShape,
  JPH::Mat44Arg seedAt,
  JPH::Mat44Arg probeAt,
  bool flip,
  const JPH::SubShapeIDCreator& creator1,
  const JPH::SubShapeIDCreator& creator2,
  float separation,
  std::vector<FieldHit>& held,
  JPH::CollideShapeCollector& collector
) {
  const JPH::AABox overlap =
    overlapIn(seedShape, probeShape, seedAt, probeAt, separation);

  if (!overlap.IsValid()) {
    return false;
  }

  bool penetrates = false;

  const JPH::Mat44 intoProbe = probeAt.Inversed() * seedAt;
  const JPH::Mat44 outOfProbe = probeAt.GetRotation();

  const uint32_t bits = seedShape.GetSubShapeIDBitsRecursive();

  forEachNearSeed(
    seedShape,
    probeShape,
    intoProbe,
    toGlm(overlap.mMin),
    toGlm(overlap.mMax),
    separation,
    [&](uint32_t index, const FieldSeed& seed) {
      if (collector.ShouldEarlyOut()) {
        return;
      }

      const glm::vec3 fromSeedCentre = seed.point - seedShape.centerOfMass();

      const JPH::Vec3 probed = intoProbe * toJph(fromSeedCentre);

      if (probeShape.beyondReach(probed, separation)) {
        return;
      }

      const glm::vec3 probedAt = toGlm(probed) + probeShape.centerOfMass();

      if (
        talus::field::probeDistance(probeShape.field(), probedAt) >= separation
      ) {
        return;
      }

      const talus::field::FieldProbe hit =
        talus::field::probe(probeShape.field(), probedAt);

      const JPH::Vec3 normal = (outOfProbe * toJph(hit.normal)).Normalized();

      if (normal.Dot(seedAt.Multiply3x3(toJph(seed.normal))) > -FACING) {
        return;
      }

      const JPH::Vec3 onSeed = seedAt * toJph(fromSeedCentre);
      const JPH::Vec3 onProbe = onSeed - hit.distance * normal;

      const FieldHit found{
        flip ? onProbe : onSeed,
        flip ? onSeed : onProbe,
        flip ? normal : -normal,
        -hit.distance,
        flip ? creator1.GetID() : creator1.PushID(index, bits).GetID(),
        flip ? creator2.PushID(index, bits).GetID() : creator2.GetID(),
        JPH::TransformedShape::sGetBodyID(collector.GetContext())
      };

      if (found.penetrationDepth > 0.0f) {
        collector.AddHit(found.result());
        penetrates = true;
        return;
      }

      held.push_back(found);
    }
  );

  return penetrates;
}

void collideFieldVsField(
  const JPH::Shape* inShape1,
  const JPH::Shape* inShape2,
  JPH::Vec3Arg,
  JPH::Vec3Arg,
  JPH::Mat44Arg centerOfMass1,
  JPH::Mat44Arg centerOfMass2,
  const JPH::SubShapeIDCreator& creator1,
  const JPH::SubShapeIDCreator& creator2,
  const JPH::CollideShapeSettings& settings,
  JPH::CollideShapeCollector& collector,
  const JPH::ShapeFilter& filter
) {
  const FieldShape& shape1 = *static_cast<const FieldShape*>(inShape1);
  const FieldShape& shape2 = *static_cast<const FieldShape*>(inShape2);

  if (shape1.hull() != nullptr && shape2.hull() != nullptr) {
    const JPH::Mat44 at1 = hullAt(shape1, centerOfMass1);
    const JPH::Mat44 at2 = hullAt(shape2, centerOfMass2);

    CollideFaceCollector faces(
      collector,
      hullSide(shape1, at1),
      hullSide(shape2, at2)
    );
    HullHitCollector consistent(
      collectsFaces(settings) ? static_cast<JPH::CollideShapeCollector&>(faces)
                              : collector,
      hullSide(shape1, at1),
      hullSide(shape2, at2),
      settings.mCollisionTolerance,
      collectsFaces(settings)
    );

    JPH::CollisionDispatch::sCollideShapeVsShape(
      shape1.hull(),
      shape2.hull(),
      JPH::Vec3::sReplicate(1.0f),
      JPH::Vec3::sReplicate(1.0f),
      at1,
      at2,
      creator1,
      creator2,
      settings,
      consistent,
      filter
    );

    return;
  }

  const float separation = std::max(settings.mMaxSeparationDistance, 0.0f);

  const float volume1 = shape1.GetLocalBounds().GetVolume();
  const float volume2 = shape2.GetLocalBounds().GetVolume();

  thread_local std::vector<FieldHit> held;

  held.clear();

  bool penetrates = false;

  if (!(volume1 > volume2)) {
    penetrates = collideOneWay(
      shape1,
      shape2,
      centerOfMass1,
      centerOfMass2,
      false,
      creator1,
      creator2,
      separation,
      held,
      collector
    );
  }

  if (!(volume2 > volume1)) {
    penetrates = collideOneWay(
                   shape2,
                   shape1,
                   centerOfMass2,
                   centerOfMass1,
                   true,
                   creator1,
                   creator2,
                   separation,
                   held,
                   collector
                 )
                 || penetrates;
  }

  if (penetrates) {
    return;
  }

  for (const FieldHit& hit : held) {
    if (collector.ShouldEarlyOut()) {
      return;
    }

    collector.AddHit(hit.result());
  }
}

void castFieldVsField(
  const JPH::ShapeCast& shapeCast,
  const JPH::ShapeCastSettings& settings,
  const JPH::Shape* inShape,
  JPH::Vec3Arg,
  const JPH::ShapeFilter& filter,
  JPH::Mat44Arg toWorld,
  const JPH::SubShapeIDCreator& creator1,
  const JPH::SubShapeIDCreator& creator2,
  JPH::CastShapeCollector& collector
) {
  const FieldShape& moving = *static_cast<const FieldShape*>(shapeCast.mShape);
  const FieldShape& fixed = *static_cast<const FieldShape*>(inShape);

  if (moving.hull() != nullptr && fixed.hull() != nullptr) {
    const JPH::ShapeCast hullCast(
      moving.hull(),
      JPH::Vec3::sReplicate(1.0f),
      hullAt(moving, shapeCast.mCenterOfMassStart)
        .PostTranslated(-hullOffset(fixed)),
      shapeCast.mDirection
    );

    const JPH::Mat44 fixedAt = hullAt(fixed, toWorld);

    CastFaceCollector faces(
      collector,
      hullSide(moving, hullCast.mCenterOfMassStart),
      hullCast.mDirection,
      hullSide(fixed, fixedAt)
    );

    JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(
      hullCast,
      settings,
      fixed.hull(),
      JPH::Vec3::sReplicate(1.0f),
      filter,
      fixedAt,
      creator1,
      creator2,
      collectsFaces(settings) ? faces : collector
    );

    return;
  }

  const float travel = shapeCast.mDirection.Length();

  if (travel < DIRECTION_FLOOR) {
    return;
  }

  const std::span<const FieldSeed> seeds = moving.seeds();

  float fraction = 0.0f;

  for (uint32_t iteration = 0; iteration != 64u; ++iteration) {
    const JPH::Mat44 at = shapeCast.mCenterOfMassStart.PostTranslated(
      shapeCast.mDirection * fraction
    );

    float closest = std::numeric_limits<float>::max();
    JPH::Vec3 closestPoint = JPH::Vec3::sZero();
    glm::vec3 closestNormal(0.0f, 1.0f, 0.0f);
    uint32_t closestSeed = 0u;

    for (const SurfaceBrick& brick : moving.surfaceBricks()) {
      const glm::vec3 centre = 0.5f * (brick.minimum + brick.maximum);
      const float radius = 0.5f * glm::length(brick.maximum - brick.minimum);

      const glm::vec3 probedCentre =
        toGlm(at * toJph(centre - moving.centerOfMass()))
        + fixed.centerOfMass();

      if (nearestPossible(fixed.field(), probedCentre) - radius >= closest) {
        continue;
      }

      for (uint32_t index = brick.begin; index != brick.end; ++index) {
        const FieldSeed& seed = seeds[index];

        const JPH::Vec3 probed = at * toJph(seed.point - moving.centerOfMass());
        const talus::field::FieldProbe hit = talus::field::probe(
          fixed.field(),
          toGlm(probed) + fixed.centerOfMass()
        );

        if (hit.distance < closest) {
          closest = hit.distance;
          closestPoint = probed;
          closestNormal = hit.normal;
          closestSeed = index;
        }
      }
    }

    if (closest <= 1.0e-3f) {
      JPH::ShapeCastResult result;

      result.mFraction = fraction;
      result.mContactPointOn1 = toWorld * closestPoint;
      result.mContactPointOn2 =
        toWorld * (closestPoint - closest * toJph(closestNormal));
      result.mPenetrationAxis = toWorld.Multiply3x3(-toJph(closestNormal));
      result.mPenetrationDepth = -closest;
      result.mSubShapeID1 =
        creator1.PushID(closestSeed, moving.GetSubShapeIDBitsRecursive())
          .GetID();
      result.mSubShapeID2 = creator2.GetID();
      result.mIsBackFaceHit = false;
      result.mBodyID2 =
        JPH::TransformedShape::sGetBodyID(collector.GetContext());

      collector.AddHit(result);

      return;
    }

    fraction += closest / travel;

    if (fraction >= 1.0f) {
      return;
    }
  }
}

constexpr uint32_t NO_HEX = UINT32_MAX;
constexpr uint32_t NO_NODE = UINT32_MAX;
constexpr int NODE_SEARCH_RINGS = 3;
constexpr float DISPLACED_HULL_TOLERANCE = 1.0e-4f;

JPH::RefConst<JPH::ConvexHullShape> hullFrom(
  const JPH::Array<JPH::Vec3>& input,
  float tolerance
) {
  JPH::ConvexHullShapeSettings settings(input, 0.0f);
  settings.mHullTolerance = tolerance;

  const JPH::ShapeSettings::ShapeResult result = settings.Create();

  if (result.HasError()) {
    return nullptr;
  }

  return static_cast<const JPH::ConvexHullShape*>(result.Get().GetPtr());
}

size_t sampleIndex(const glm::uvec3& size, const glm::ivec3& at) {
  return static_cast<size_t>(at.x)
         + static_cast<size_t>(size.x)
             * (static_cast<size_t>(at.y) + static_cast<size_t>(size.y) * static_cast<size_t>(at.z));
}

bool withinSize(const glm::uvec3& size, const glm::ivec3& at) {
  return glm::all(glm::greaterThanEqual(at, glm::ivec3(0)))
         && glm::all(glm::lessThan(at, glm::ivec3(size)));
}

struct RestLattice {
  glm::uvec3 cells{0u};
  std::vector<uint32_t> hexOf;
  std::vector<glm::vec3> low;
  std::vector<glm::vec3> high;
  std::vector<uint32_t> nodeOf;
};

RestLattice restLattice(
  const SampledSdf& field,
  const objectmodel::PreparedMesh& mesh
) {
  RestLattice held;

  held.cells = field.resolution - glm::uvec3(1u);
  held.hexOf.assign(
    static_cast<size_t>(held.cells.x) * held.cells.y * held.cells.z,
    NO_HEX
  );
  held.nodeOf.assign(
    static_cast<size_t>(field.resolution.x) * field.resolution.y
      * field.resolution.z,
    NO_NODE
  );
  held.low.resize(mesh.hexes.size());
  held.high.resize(mesh.hexes.size());

  for (uint32_t hex = 0u; hex != mesh.hexes.size(); ++hex) {
    glm::vec3 low(std::numeric_limits<float>::max());
    glm::vec3 high(std::numeric_limits<float>::lowest());

    for (const uint32_t node : mesh.hexes[hex].nodes) {
      low = glm::min(low, mesh.restPositions[node]);
      high = glm::max(high, mesh.restPositions[node]);
    }

    held.low[hex] = low;
    held.high[hex] = high;

    const glm::ivec3 cell(glm::round((low - field.origin) / field.voxelSize));

    if (withinSize(held.cells, cell)) {
      uint32_t& slot = held.hexOf[sampleIndex(held.cells, cell)];

      if (slot == NO_HEX) {
        slot = hex;
      }
    }
  }

  for (uint32_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::ivec3 at(
      glm::round((mesh.restPositions[node] - field.origin) / field.voxelSize)
    );

    if (withinSize(field.resolution, at)) {
      uint32_t& slot = held.nodeOf[sampleIndex(field.resolution, at)];

      if (slot == NO_NODE) {
        slot = node;
      }
    }
  }

  return held;
}

uint32_t containingHex(
  const SampledSdf& field,
  const RestLattice& lattice,
  const glm::vec3& point
) {
  const glm::ivec3 base(glm::floor((point - field.origin) / field.voxelSize));

  uint32_t found = NO_HEX;

  for (int z = -1; z <= 1; ++z) {
    for (int y = -1; y <= 1; ++y) {
      for (int x = -1; x <= 1; ++x) {
        const glm::ivec3 cell = base + glm::ivec3(x, y, z);

        if (!withinSize(lattice.cells, cell)) {
          continue;
        }

        const uint32_t hex = lattice.hexOf[sampleIndex(lattice.cells, cell)];

        if (
          hex != NO_HEX && hex < found
          && glm::all(glm::lessThanEqual(lattice.low[hex], point))
          && glm::all(glm::lessThanEqual(point, lattice.high[hex]))
        ) {
          found = hex;
        }
      }
    }
  }

  return found;
}

uint32_t nearestNode(
  const SampledSdf& field,
  const objectmodel::PreparedMesh& mesh,
  const RestLattice& lattice,
  const glm::vec3& point
) {
  const glm::ivec3 top = glm::ivec3(field.resolution) - glm::ivec3(1);
  const glm::ivec3 centre = glm::clamp(
    glm::ivec3(glm::round((point - field.origin) / field.voxelSize)),
    glm::ivec3(0),
    top
  );
  const float step =
    std::min({field.voxelSize.x, field.voxelSize.y, field.voxelSize.z});

  uint32_t best = NO_NODE;
  float bestGap = std::numeric_limits<float>::infinity();

  const auto consider = [&](uint32_t node) {
    const float gap = glm::distance(mesh.restPositions[node], point);

    if (gap < bestGap || (gap == bestGap && node < best)) {
      best = node;
      bestGap = gap;
    }
  };

  for (int ring = 0; ring <= NODE_SEARCH_RINGS; ++ring) {
    if (best != NO_NODE && (static_cast<float>(ring) - 0.5f) * step > bestGap) {
      return best;
    }

    for (int z = -ring; z <= ring; ++z) {
      for (int y = -ring; y <= ring; ++y) {
        for (int x = -ring; x <= ring; ++x) {
          if (std::max({std::abs(x), std::abs(y), std::abs(z)}) != ring) {
            continue;
          }

          const glm::ivec3 at = centre + glm::ivec3(x, y, z);

          if (!withinSize(field.resolution, at)) {
            continue;
          }

          const uint32_t node =
            lattice.nodeOf[sampleIndex(field.resolution, at)];

          if (node != NO_NODE) {
            consider(node);
          }
        }
      }
    }
  }

  if (
    best != NO_NODE
    && (static_cast<float>(NODE_SEARCH_RINGS) + 0.5f) * step > bestGap
  ) {
    return best;
  }

  for (uint32_t node = 0u; node != mesh.restPositions.size(); ++node) {
    consider(node);
  }

  return best;
}

glm::dvec3 restDisplacement(
  const SampledSdf& field,
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> shape,
  const RestLattice& lattice,
  const glm::vec3& point
) {
  const uint32_t hex = containingHex(field, lattice, point);

  if (hex == NO_HEX) {
    const uint32_t node = nearestNode(field, mesh, lattice, point);

    return glm::dvec3(shape[node]) - glm::dvec3(mesh.restPositions[node]);
  }

  const glm::vec3& low = lattice.low[hex];
  const glm::vec3& high = lattice.high[hex];
  const glm::vec3 middle = 0.5f * (low + high);
  const glm::dvec3 across = glm::dvec3(point - low) / glm::dvec3(high - low);

  glm::dvec3 moved(0.0);

  for (const uint32_t node : mesh.hexes[hex].nodes) {
    const glm::vec3& rest = mesh.restPositions[node];

    double weight = 1.0;

    for (int axis = 0; axis != 3; ++axis) {
      weight *= rest[axis] > middle[axis] ? across[axis] : 1.0 - across[axis];
    }

    moved += weight * (glm::dvec3(shape[node]) - glm::dvec3(rest));
  }

  return moved;
}
}

SliceHulls sliceHulls(
  const talus::field::SampledSdf& field,
  const talus::field::LatticeSurface& surface
) {
  const glm::uvec3 size = field.resolution;
  const glm::uvec3 first(0u, size.x, size.x + size.y);
  const uint32_t slices = size.x + size.y + size.z;

  const uint32_t crossings = static_cast<uint32_t>(surface.crossings.size());
  const uint32_t points =
    crossings + static_cast<uint32_t>(surface.inside.size());

  const auto across = [&](uint32_t index, uint32_t axis) {
    return index >= crossings || surface.crossings[index].axis != axis;
  };

  const auto sliceOf = [&](uint32_t index, uint32_t axis) {
    return first[axis]
           + static_cast<uint32_t>(surfacePoint(surface, index)[axis]);
  };

  std::vector<uint32_t> begin(slices + 1u, 0u);

  for (uint32_t index = 0u; index != points; ++index) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      if (across(index, axis)) {
        ++begin[sliceOf(index, axis) + 1u];
      }
    }
  }

  for (uint32_t slice = 0u; slice != slices; ++slice) {
    begin[slice + 1u] += begin[slice];
  }

  std::vector<uint32_t> members(begin[slices]);
  std::vector<uint32_t> filled(begin.begin(), begin.end() - 1);

  for (uint32_t index = 0u; index != points; ++index) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      if (across(index, axis)) {
        members[filled[sliceOf(index, axis)]++] = index;
      }
    }
  }

  SliceHulls result;

  std::vector<bool> vertex(points, false);
  std::vector<SlicePoint> inSlice;

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    const uint32_t u = (axis + 1u) % 3u;
    const uint32_t v = (axis + 2u) % 3u;

    for (uint32_t at = 0u; at != size[axis]; ++at) {
      const uint32_t slice = first[axis] + at;

      inSlice.clear();

      for (uint32_t member = begin[slice]; member != begin[slice + 1u];
           ++member) {
        const glm::vec3 lattice = surfacePoint(surface, members[member]);

        inSlice.push_back(
          SlicePoint{double(lattice[u]), double(lattice[v]), members[member]}
        );
      }

      if (inSlice.empty()) {
        continue;
      }

      const std::vector<SlicePoint> hull = outline(inSlice);

      if (!withinOutline(hull, inSlice, crossings)) {
        return result;
      }

      for (const SlicePoint& corner : hull) {
        vertex[corner.index] = true;
      }
    }
  }

  for (uint32_t index = 0u; index != points; ++index) {
    if (vertex[index]) {
      result.vertices.push_back(index);
    }
  }

  result.convex = true;

  return result;
}

JPH::RefConst<JPH::ConvexHullShape> hullThrough(
  const talus::field::SampledSdf& field,
  const talus::field::LatticeSurface& surface,
  std::span<const uint32_t> vertices
) {
  JPH::Array<JPH::Vec3> input;
  input.reserve(vertices.size());

  for (const uint32_t index : vertices) {
    input.push_back(
      toJph(talus::field::latticeToField(field, surfacePoint(surface, index)))
    );
  }

  return hullFrom(input, hullTolerance(field));
}

bool hullAgrees(
  const talus::field::SampledSdf& field,
  std::span<const talus::field::Crossing> crossings,
  const JPH::ConvexHullShape& hull
) {
  const std::vector<LatticePlane> planes = latticePlanes(field, hull);

  if (planes.empty()) {
    return false;
  }

  return crossingsOnHull(planes, crossings);
}

FieldHull convexHullOf(const talus::field::SampledSdf& field) {
  FieldHull result;

  const talus::field::LatticeSurface surface =
    talus::field::latticeSurface(field);

  result.crossings = surface.crossings.size();
  result.inside = surface.inside.size();

  if (surface.crossings.empty() && surface.inside.empty()) {
    return result;
  }

  const SliceHulls slices = sliceHulls(field, surface);

  if (!slices.convex) {
    return result;
  }

  result.points = slices.vertices.size();
  result.built = true;

  JPH::RefConst<JPH::ConvexHullShape> hull =
    hullThrough(field, surface, slices.vertices);

  if (hull == nullptr || !hullAgrees(field, surface.crossings, *hull)) {
    return result;
  }

  result.shape = std::move(hull);

  return result;
}

HullAxis hullSeparatingAxis(
  const JPH::ConvexHullShape& first,
  JPH::Mat44Arg firstAt,
  const JPH::ConvexHullShape& second,
  JPH::Mat44Arg secondAt,
  JPH::Vec3Arg reported
) {
  const HullSide one{&first, firstAt, 0.0f};
  const HullSide two{&second, secondAt, 0.0f};

  PlacedHull onePlaced;
  PlacedHull twoPlaced;

  placeHull(one, onePlaced);
  placeHull(two, twoPlaced);

  return separatingAxis(one, onePlaced, two, twoPlaced, reported);
}

JPH::RefConst<JPH::ConvexHullShape> displacedHull(
  const talus::field::SampledSdf& field,
  const objectmodel::PreparedMesh& mesh,
  std::span<const glm::vec3> shape
) {
  if (mesh.restPositions.empty() || shape.size() != mesh.restPositions.size()) {
    return nullptr;
  }

  const talus::field::LatticeSurface surface =
    talus::field::latticeSurface(field);
  const uint32_t count =
    static_cast<uint32_t>(surface.crossings.size() + surface.inside.size());

  if (count == 0u) {
    return nullptr;
  }

  const RestLattice lattice = restLattice(field, mesh);

  JPH::Array<JPH::Vec3> input;
  input.reserve(count);

  for (uint32_t index = 0u; index != count; ++index) {
    const glm::vec3 at =
      talus::field::latticeToField(field, surfacePoint(surface, index));

    input.push_back(
      toJph(at + glm::vec3(restDisplacement(field, mesh, shape, lattice, at)))
    );
  }

  JPH::ConvexHullBuilder builder(input);

  const char* error = nullptr;

  if (
    builder.Initialize(
      JPH::ConvexHullShape::cMaxPointsInHull,
      DISPLACED_HULL_TOLERANCE,
      error
    )
    != JPH::ConvexHullBuilder::EResult::Success
  ) {
    return nullptr;
  }

  return hullFrom(input, DISPLACED_HULL_TOLERANCE);
}

FieldShape::FieldShape(const talus::field::SampledSdf& field)
    : FieldShape(field, nullptr, glm::uvec3(0u), glm::uvec3(0u), nullptr) {}

FieldShape::FieldShape(
  const talus::field::SampledSdf& field,
  const FieldShape& previous,
  const glm::uvec3& changedBegin,
  const glm::uvec3& changedEnd
)
    : FieldShape(field, &previous, changedBegin, changedEnd, nullptr) {}

FieldShape::FieldShape(
  const FieldShape& authored,
  JPH::RefConst<JPH::ConvexHullShape> hull
)
    : FieldShape(
        authored.field(),
        &authored,
        glm::uvec3(0u),
        glm::uvec3(0u),
        std::move(hull)
      ) {}

FieldShape::FieldShape(
  const talus::field::SampledSdf& field,
  const FieldShape* previous,
  const glm::uvec3& changedBegin,
  const glm::uvec3& changedEnd,
  JPH::RefConst<JPH::ConvexHullShape> hull
)
    : JPH::Shape(JPH::EShapeType::User1, JPH::EShapeSubType::User1),
      m_field(&field) {
  if (glm::any(glm::lessThan(field.resolution, glm::uvec3(2u)))) {
    throw std::runtime_error(
      "FieldShape: the field has no cells to collide with"
    );
  }

  if (
    previous != nullptr && previous->m_uniformMaterial
    && !field.materials.empty()
  ) {
    m_material = previous->m_material;
    m_uniformMaterial =
      samplesHold(field, changedBegin, changedEnd, m_material);
  } else {
    m_uniformMaterial = std::adjacent_find(
                          field.materials.begin(),
                          field.materials.end(),
                          std::not_equal_to<>()
                        )
                        == field.materials.end();
    m_material =
      field.materials.empty() ? uint8_t{0u} : field.materials.front();
  }

  const glm::uvec3 counts = talus::field::brickCounts(field);
  const uint64_t bricks = static_cast<uint64_t>(counts.x) * counts.y * counts.z;

  while ((uint64_t{1} << m_subShapeBits) < bricks) {
    ++m_subShapeBits;
  }

  if (m_subShapeBits > JPH::SubShapeID::MaxBits) {
    throw std::runtime_error(
      "FieldShape: the grid needs more sub shape bits than Jolt has"
    );
  }

  float deepest = 0.0f;

  for (float low : field.brickMinimum) {
    deepest = std::min(deepest, low);
  }

  m_innerRadius = std::max(-deepest, 1.0e-3f);

  m_brickIntegral.resize(static_cast<size_t>(bricks));

  for (uint32_t index = 0u; index != bricks; ++index) {
    m_brickIntegral[index] =
      previous != nullptr && !inRange(index, counts, changedBegin, changedEnd)
        ? previous->m_brickIntegral[index]
        : integrateBrick(field, index);
  }

  const SolidIntegral solid = totalOf(m_brickIntegral);

  if (solid.mass > 0.0) {
    m_centerOfMass = glm::vec3(solid.firstMoment / double(solid.mass));

    const glm::dvec3 d(m_centerOfMass);
    const double m = solid.mass;

    glm::dmat3 aboutCentre = solid.secondMoment;

    aboutCentre[0][0] -= m * (d.y * d.y + d.z * d.z);
    aboutCentre[1][1] -= m * (d.x * d.x + d.z * d.z);
    aboutCentre[2][2] -= m * (d.x * d.x + d.y * d.y);
    aboutCentre[0][1] += m * d.x * d.y;
    aboutCentre[0][2] += m * d.x * d.z;
    aboutCentre[1][2] += m * d.y * d.z;

    aboutCentre[1][0] = aboutCentre[0][1];
    aboutCentre[2][0] = aboutCentre[0][2];
    aboutCentre[2][1] = aboutCentre[1][2];

    m_massProperties.mMass = static_cast<float>(solid.mass);
    m_massProperties.mInertia = JPH::Mat44::sIdentity();

    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        m_massProperties.mInertia(row, column) =
          static_cast<float>(aboutCentre[column][row]);
      }
    }
  }

  const glm::vec3 span = field.voxelSize * float(talus::field::BRICK_CELLS);

  m_brickSlot.assign(static_cast<size_t>(bricks), NO_SLOT);
  m_brickReach.assign(static_cast<size_t>(bricks), ReachCells{});

  const Reach reach = reachFor(field);

  glm::vec3 solidLow(std::numeric_limits<float>::max());
  glm::vec3 solidHigh(std::numeric_limits<float>::lowest());

  bool anySolid = false;

  for (uint32_t index = 0u; index != bricks; ++index) {
    const glm::uvec3 brick(
      index % counts.x,
      (index / counts.x) % counts.y,
      index / (counts.x * counts.y)
    );

    const bool unchanged =
      previous != nullptr && !inRange(index, counts, changedBegin, changedEnd);

    if (unchanged) {
      m_brickReach[index] = previous->m_brickReach[index];
    } else if (!talus::field::brickHoldsSurface(field, index)) {
      m_brickReach[index] = reachCellsOf(field, brick, index, reach);
    }

    if (field.brickMinimum[index] > 0.0f) {
      continue;
    }

    const glm::vec3 corner = brickOrigin(field, brick);

    solidLow = glm::min(solidLow, corner);
    solidHigh = glm::max(solidHigh, corner + span);

    anySolid = true;

    if (!talus::field::brickHoldsSurface(field, index)) {
      continue;
    }

    const bool reusable = unchanged && previous->m_brickSlot[index] != NO_SLOT;

    SurfaceBrick held;

    held.brick = index;
    held.begin = static_cast<uint32_t>(m_seeds.size());

    if (reusable) {
      const SurfaceBrick& before =
        previous->m_surfaceBricks[previous->m_brickSlot[index]];

      m_seeds.insert(
        m_seeds.end(),
        previous->m_seeds.begin() + before.begin,
        previous->m_seeds.begin() + before.end
      );
    } else {
      m_brickReach[index] = ReachCells{};

      appendCellSeeds(field, brick, index, reach, m_seeds, m_brickReach[index]);

      if (m_seeds.size() == held.begin) {
        const glm::vec3 onSurface = ontoSurface(field, corner + span * 0.5f);

        m_seeds.push_back(
          FieldSeed{
            onSurface,
            index,
            talus::field::probe(field, onSurface).normal
          }
        );
      }
    }

    held.end = static_cast<uint32_t>(m_seeds.size());

    held.minimum = m_seeds[held.begin].point;
    held.maximum = m_seeds[held.begin].point;

    for (uint32_t seed = held.begin + 1u; seed != held.end; ++seed) {
      held.minimum = glm::min(held.minimum, m_seeds[seed].point);
      held.maximum = glm::max(held.maximum, m_seeds[seed].point);
    }

    m_brickSlot[index] = static_cast<uint32_t>(m_surfaceBricks.size());
    m_surfaceBricks.push_back(held);
  }

  while ((uint64_t{1} << m_subShapeBits) <= m_seeds.size()) {
    ++m_subShapeBits;
  }

  if (m_subShapeBits > JPH::SubShapeID::MaxBits) {
    throw std::runtime_error(
      "FieldShape: the surface needs more sub shape bits than Jolt has"
    );
  }

  const bool carried = hull != nullptr;

  m_hull = carried ? std::move(hull) : convexHullOf(field).shape;

  m_grid = JPH::AABox(
    toJph(field.origin - m_centerOfMass),
    toJph(gridMaximum(field) - m_centerOfMass)
  );

  m_reach = reachBox(field, m_brickReach, m_centerOfMass);
  m_reachDistance = reach.distance;

  if (anySolid) {
    m_bounds = JPH::AABox(
      toJph(glm::max(solidLow, field.origin) - m_centerOfMass),
      toJph(glm::min(solidHigh, gridMaximum(field)) - m_centerOfMass)
    );
  } else {
    m_bounds = m_grid;
  }

  if (carried) {
    JPH::AABox around = m_hull->GetLocalBounds();

    around.Translate(hullOffset(*this));

    m_bounds.Encapsulate(around);
  }
}

const talus::field::SampledSdf& FieldShape::field() const noexcept {
  return *m_field;
}

std::span<const SurfaceBrick> FieldShape::surfaceBricks() const noexcept {
  return m_surfaceBricks;
}

std::span<const FieldSeed> FieldShape::seeds() const noexcept {
  return m_seeds;
}

bool FieldShape::beyondReach(
  JPH::Vec3Arg point,
  float separation
) const noexcept {
  return separation <= m_reachDistance && m_grid.Contains(point)
         && !m_reach.Contains(point);
}

bool FieldShape::uniformMaterial() const noexcept {
  return m_uniformMaterial;
}

const JPH::ConvexHullShape* FieldShape::hull() const noexcept {
  return m_hull.GetPtr();
}

const glm::vec3& FieldShape::centerOfMass() const noexcept {
  return m_centerOfMass;
}

JPH::AABox FieldShape::GetLocalBounds() const {
  return m_bounds;
}

JPH::uint FieldShape::GetSubShapeIDBitsRecursive() const {
  return m_subShapeBits;
}

float FieldShape::GetInnerRadius() const {
  return m_innerRadius;
}

JPH::MassProperties FieldShape::GetMassProperties() const {
  return m_massProperties;
}

JPH::Vec3 FieldShape::GetCenterOfMass() const {
  return toJph(m_centerOfMass);
}

const JPH::PhysicsMaterial* FieldShape::GetMaterial(
  const JPH::SubShapeID&
) const {
  return JPH::PhysicsMaterial::sDefault;
}

JPH::Vec3 FieldShape::GetSurfaceNormal(
  const JPH::SubShapeID&,
  JPH::Vec3Arg localSurfacePosition
) const {
  return toJph(
    talus::field::probe(*m_field, toGlm(localSurfacePosition) + m_centerOfMass)
      .normal
  );
}

void FieldShape::GetSubmergedVolume(
  JPH::Mat44Arg,
  JPH::Vec3Arg,
  const JPH::Plane&,
  float& totalVolume,
  float& submergedVolume,
  JPH::Vec3& centerOfBuoyancy
#ifdef JPH_DEBUG_RENDERER
  ,
  JPH::RVec3Arg
#endif
) const {
  totalVolume = 0.0f;
  submergedVolume = 0.0f;
  centerOfBuoyancy = JPH::Vec3::sZero();
}

#ifdef JPH_DEBUG_RENDERER

void FieldShape::Draw(
  JPH::DebugRenderer*,
  JPH::RMat44Arg,
  JPH::Vec3Arg,
  JPH::ColorArg,
  bool,
  bool
) const {}

#endif

bool FieldShape::CastRay(
  const JPH::RayCast&,
  const JPH::SubShapeIDCreator&,
  JPH::RayCastResult&
) const {
  return false;
}

void FieldShape::CastRay(
  const JPH::RayCast&,
  const JPH::RayCastSettings&,
  const JPH::SubShapeIDCreator&,
  JPH::CastRayCollector&,
  const JPH::ShapeFilter&
) const {}

void FieldShape::CollidePoint(
  JPH::Vec3Arg point,
  const JPH::SubShapeIDCreator& subShapeIDCreator,
  JPH::CollidePointCollector& collector,
  const JPH::ShapeFilter&
) const {
  if (
    talus::field::probe(*m_field, toGlm(point) + m_centerOfMass).distance
    >= 0.0f
  ) {
    return;
  }

  collector.AddHit(
    {JPH::TransformedShape::sGetBodyID(collector.GetContext()),
     subShapeIDCreator.GetID()}
  );
}

void FieldShape::CollideSoftBodyVertices(
  JPH::Mat44Arg centerOfMassTransform,
  JPH::Vec3Arg,
  const JPH::CollideSoftBodyVertexIterator& vertices,
  JPH::uint numVertices,
  int collidingShapeIndex
) const {
  if (m_field == nullptr) {
    return;
  }

  const JPH::Mat44 toLocal = centerOfMassTransform.Inversed();

  for (JPH::CollideSoftBodyVertexIterator v = vertices,
                                          end = vertices + numVertices;
       v != end;
       ++v) {
    if (v.GetInvMass() <= 0.0f) {
      continue;
    }

    const JPH::Vec3 local = toLocal * v.GetPosition();

    const talus::field::FieldProbe probed =
      talus::field::probe(*m_field, toGlm(local) + m_centerOfMass);

    const float penetration = -probed.distance;

    if (!v.UpdatePenetration(penetration)) {
      continue;
    }

    const JPH::Vec3 normal =
      centerOfMassTransform.Multiply3x3(toJph(probed.normal))
        .NormalizedOr(JPH::Vec3::sAxisY());

    const JPH::Vec3 point = v.GetPosition() + normal * penetration;

    v.SetCollision(
      JPH::Plane::sFromPointAndNormal(point, normal),
      collidingShapeIndex
    );
  }
}

void FieldShape::GetTrianglesStart(
  GetTrianglesContext&,
  const JPH::AABox&,
  JPH::Vec3Arg,
  JPH::QuatArg,
  JPH::Vec3Arg
) const {}

int FieldShape::GetTrianglesNext(
  GetTrianglesContext&,
  int,
  JPH::Float3*,
  const JPH::PhysicsMaterial**
) const {
  return 0;
}

JPH::Shape::Stats FieldShape::GetStats() const {
  return Stats(
    sizeof(*this) + m_seeds.size() * sizeof(FieldSeed)
      + m_surfaceBricks.size() * sizeof(SurfaceBrick)
      + m_brickSlot.size() * sizeof(uint32_t),
    0u
  );
}

float FieldShape::GetVolume() const {
  return 0.0f;
}

void registerFieldShape() {
  JPH::ShapeFunctions& functions =
    JPH::ShapeFunctions::sGet(JPH::EShapeSubType::User1);

  functions.mConstruct = []() -> JPH::Shape* {
    return nullptr;
  };
  functions.mColor = JPH::Color::sGreen;

  JPH::CollisionDispatch::sRegisterCollideShape(
    JPH::EShapeSubType::User1,
    JPH::EShapeSubType::User1,
    collideFieldVsField
  );

  JPH::CollisionDispatch::sRegisterCastShape(
    JPH::EShapeSubType::User1,
    JPH::EShapeSubType::User1,
    castFieldVsField
  );
}

}
