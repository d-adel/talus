#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/field/meshsdf/meshsdf.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

struct Mesh {
  std::vector<glm::vec3> vertices;
  std::vector<uint32_t> indices;
};

Mesh icosphere(float radius, uint32_t subdivisions) {
  const double golden = (1.0 + std::sqrt(5.0)) / 2.0;

  std::vector<glm::dvec3> unit{
    {-1.0, golden, 0.0},
    {1.0, golden, 0.0},
    {-1.0, -golden, 0.0},
    {1.0, -golden, 0.0},
    {0.0, -1.0, golden},
    {0.0, 1.0, golden},
    {0.0, -1.0, -golden},
    {0.0, 1.0, -golden},
    {golden, 0.0, -1.0},
    {golden, 0.0, 1.0},
    {-golden, 0.0, -1.0},
    {-golden, 0.0, 1.0}
  };

  for (glm::dvec3& corner : unit) {
    corner = glm::normalize(corner);
  }

  std::vector<uint32_t> faces{0,  11, 5,  0, 5, 1,  0, 1,  7, 0,  7,  10,
                              0,  10, 11, 1, 5, 9,  5, 11, 4, 11, 10, 2,
                              10, 7,  6,  7, 1, 8,  3, 9,  4, 3,  4,  2,
                              3,  2,  6,  3, 6, 8,  3, 8,  9, 4,  9,  5,
                              2,  4,  11, 6, 2, 10, 8, 6,  7, 9,  8,  1};

  for (uint32_t level = 0u; level != subdivisions; ++level) {
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> middles;
    std::vector<uint32_t> finer;

    const auto middle = [&](uint32_t a, uint32_t b) {
      const std::pair<uint32_t, uint32_t> key{std::min(a, b), std::max(a, b)};
      const auto found = middles.find(key);

      if (found != middles.end()) {
        return found->second;
      }

      unit.push_back(glm::normalize(unit[a] + unit[b]));

      const uint32_t made = static_cast<uint32_t>(unit.size() - 1u);
      middles.emplace(key, made);

      return made;
    };

    for (size_t face = 0u; face < faces.size(); face += 3u) {
      const uint32_t a = faces[face];
      const uint32_t b = faces[face + 1u];
      const uint32_t c = faces[face + 2u];
      const uint32_t ab = middle(a, b);
      const uint32_t bc = middle(b, c);
      const uint32_t ca = middle(c, a);

      finer.insert(finer.end(), {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca});
    }

    faces = std::move(finer);
  }

  Mesh mesh;

  for (const glm::dvec3& corner : unit) {
    mesh.vertices.push_back(glm::vec3(corner * static_cast<double>(radius)));
  }

  mesh.indices = std::move(faces);

  return mesh;
}

Mesh prism(
  const std::vector<glm::vec2>& outline,
  uint32_t fan,
  float low,
  float high
) {
  const uint32_t count = static_cast<uint32_t>(outline.size());

  Mesh mesh;

  for (const glm::vec2& corner : outline) {
    mesh.vertices.emplace_back(corner, low);
  }

  for (const glm::vec2& corner : outline) {
    mesh.vertices.emplace_back(corner, high);
  }

  for (uint32_t step = 1u; step + 1u < count; ++step) {
    const uint32_t b = (fan + step) % count;
    const uint32_t c = (fan + step + 1u) % count;

    mesh.indices.insert(
      mesh.indices.end(),
      {count + fan, count + b, count + c}
    );
    mesh.indices.insert(mesh.indices.end(), {fan, c, b});
  }

  for (uint32_t i = 0u; i != count; ++i) {
    const uint32_t next = (i + 1u) % count;

    mesh.indices.insert(
      mesh.indices.end(),
      {i, next, count + next, i, count + next, count + i}
    );
  }

  return mesh;
}

Mesh box(const glm::vec3& low, const glm::vec3& high) {
  return prism(
    {{low.x, low.y}, {high.x, low.y}, {high.x, high.y}, {low.x, high.y}},
    0u,
    low.z,
    high.z
  );
}

double boxDistance(
  const glm::vec3& low,
  const glm::vec3& high,
  const glm::vec3& point
) {
  const glm::dvec3 at(point);
  const glm::dvec3 beyond =
    glm::max(glm::dvec3(low) - at, at - glm::dvec3(high));

  return glm::length(glm::max(beyond, glm::dvec3(0.0)))
         + std::min(std::max({beyond.x, beyond.y, beyond.z}), 0.0);
}

bool insideOutline(const std::vector<glm::vec2>& outline, const glm::vec3& at) {
  bool inside = false;

  for (size_t i = 0u, j = outline.size() - 1u; i != outline.size(); j = i++) {
    const glm::dvec2 a(outline[i]);
    const glm::dvec2 b(outline[j]);

    if ((a.y > at.y) == (b.y > at.y)) {
      continue;
    }

    const double crossing = a.x + (b.x - a.x) * (at.y - a.y) / (b.y - a.y);

    if (at.x < crossing) {
      inside = !inside;
    }
  }

  return inside;
}

talus::field::SampledSdf baked(
  const Mesh& mesh,
  const glm::vec3& low,
  const glm::vec3& high,
  const glm::uvec3& resolution
) {
  return talus::field::bakeMesh(
    mesh.vertices,
    mesh.indices,
    low,
    high,
    resolution
  );
}

template<typename Visit>
void eachPoint(const talus::field::SampledSdf& field, Visit&& visit) {
  for (uint32_t z = 0u; z != field.resolution.z; ++z) {
    for (uint32_t y = 0u; y != field.resolution.y; ++y) {
      for (uint32_t x = 0u; x != field.resolution.x; ++x) {
        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(x, y, z);

        visit(
          point,
          field.distances[x + field.resolution.x * (y + field.resolution.y * z)]
        );
      }
    }
  }
}

double facetDepth(const Mesh& mesh, double radius) {
  double nearest = radius;

  for (size_t face = 0u; face < mesh.indices.size(); face += 3u) {
    const glm::dvec3 a(mesh.vertices[mesh.indices[face]]);
    const glm::dvec3 b(mesh.vertices[mesh.indices[face + 1u]]);
    const glm::dvec3 c(mesh.vertices[mesh.indices[face + 2u]]);
    const glm::dvec3 normal = glm::normalize(glm::cross(b - a, c - a));

    nearest = std::min(nearest, std::abs(glm::dot(normal, a)));
  }

  return radius - nearest;
}

double angleBetween(const glm::vec3& first, const glm::vec3& second) {
  const double cosine = glm::dot(
    glm::normalize(glm::dvec3(first)),
    glm::normalize(glm::dvec3(second))
  );

  return std::acos(std::clamp(cosine, -1.0, 1.0));
}

const Mesh& sphereMesh() {
  static const Mesh mesh = icosphere(0.25f, 4u);

  return mesh;
}

const talus::field::SampledSdf& sphereField() {
  static const talus::field::SampledSdf field =
    baked(sphereMesh(), glm::vec3(-0.35f), glm::vec3(0.35f), glm::uvec3(71u));

  return field;
}

}

TEST_CASE("bakeMesh matches the analytic sphere", "[field][meshsdf]") {
  const talus::field::SampledSdf& field = sphereField();

  REQUIRE(sphereMesh().indices.size() == 3u * 5120u);
  REQUIRE(field.distances.size() == 71u * 71u * 71u);

  double worst = 0.0;

  eachPoint(field, [&](const glm::vec3& point, float value) {
    const double expected = glm::length(glm::dvec3(point)) - 0.25;

    worst = std::max(worst, std::abs(static_cast<double>(value) - expected));
  });

  CHECK(worst <= 3.0e-4);
}

TEST_CASE("bakeMesh matches the analytic box exactly", "[field][meshsdf]") {
  const glm::vec3 centre(0.0537f, 0.0461f, 0.0519f);
  const glm::vec3 half(0.1f, 0.2f, 0.05f);
  const glm::vec3 low = centre - half;
  const glm::vec3 high = centre + half;
  const Mesh mesh = box(low, high);

  REQUIRE(mesh.indices.size() == 36u);

  const talus::field::SampledSdf field =
    baked(mesh, glm::vec3(-0.2f), glm::vec3(0.3f), glm::uvec3(51u));

  double worst = 0.0;

  eachPoint(field, [&](const glm::vec3& point, float value) {
    worst = std::max(
      worst,
      std::abs(static_cast<double>(value) - boxDistance(low, high, point))
    );
  });

  CHECK(worst <= 1.0e-5);
}

TEST_CASE("signs stay correct in a non-convex mesh", "[field][meshsdf]") {
  const glm::vec2 shift(-0.1477f, -0.1383f);
  const float low = -0.0611f;
  const float high = 0.0789f;

  std::vector<glm::vec2> outline{
    {0.0f, 0.0f},
    {0.3f, 0.0f},
    {0.3f, 0.1f},
    {0.1f, 0.1f},
    {0.1f, 0.3f},
    {0.0f, 0.3f}
  };

  for (glm::vec2& corner : outline) {
    corner += shift;
  }

  const Mesh mesh = prism(outline, 3u, low, high);
  const talus::field::SampledSdf field =
    baked(mesh, glm::vec3(-0.2f), glm::vec3(0.2f), glm::uvec3(41u));

  uint32_t inside = 0u;
  uint32_t wrong = 0u;

  eachPoint(field, [&](const glm::vec3& point, float value) {
    const bool expected =
      point.z > low && point.z < high && insideOutline(outline, point);

    inside += expected ? 1u : 0u;
    wrong += (value < 0.0f) == expected && value != 0.0f ? 0u : 1u;
  });

  CHECK(inside > 0u);
  CHECK(wrong == 0u);
}

TEST_CASE(
  "signs stay correct when the mesh lies on lattice lines",
  "[field][meshsdf]"
) {
  const glm::vec3 minimum(-0.2f);
  const glm::vec3 maximum(0.3f);
  const glm::uvec3 resolution(51u);
  const glm::vec3 voxel = talus::field::voxelSize(minimum, maximum, resolution);
  const glm::vec3 low = minimum + voxel * glm::vec3(10.0f, 5.0f, 20.0f);
  const glm::vec3 high = minimum + voxel * glm::vec3(30.0f, 45.0f, 30.0f);

  const talus::field::SampledSdf field =
    baked(box(low, high), minimum, maximum, resolution);

  uint32_t surface = 0u;
  uint32_t wrong = 0u;

  eachPoint(field, [&](const glm::vec3& point, float value) {
    const double exact = boxDistance(low, high, point);

    if (exact == 0.0) {
      ++surface;
      wrong += std::abs(value) <= 1.0e-6f ? 0u : 1u;
    } else {
      wrong += (value < 0.0f) == (exact < 0.0) && value != 0.0f ? 0u : 1u;
    }
  });

  CHECK(surface > 0u);
  CHECK(wrong == 0u);
}

TEST_CASE("bakeMesh is deterministic", "[field][meshsdf]") {
  const talus::field::SampledSdf& first = sphereField();
  const talus::field::SampledSdf second =
    baked(sphereMesh(), glm::vec3(-0.35f), glm::vec3(0.35f), glm::uvec3(71u));

  const auto same =
    [](const std::vector<float>& a, const std::vector<float>& b) {
      return a.size() == b.size()
             && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
    };

  CHECK(same(first.distances, second.distances));
  CHECK(same(first.brickLipschitz, second.brickLipschitz));
  CHECK(same(first.brickMinimum, second.brickMinimum));
  CHECK(same(first.brickMaximum, second.brickMaximum));
  CHECK(
    std::memcmp(&first.globalLipschitz, &second.globalLipschitz, sizeof(float))
    == 0
  );
  CHECK(first.materials == second.materials);
}

TEST_CASE("a baked field supports probes and rays", "[field][meshsdf]") {
  const talus::field::SampledSdf& field = sphereField();
  const glm::vec3 outside(0.3f, 0.0f, 0.0f);

  const std::vector<talus::field::Primitive> sphere{
    fixture::sphere(glm::vec3(0.0f), 0.25f)
  };
  const talus::field::SampledSdf analytic = talus::field::bake(
    sphere,
    glm::vec3(-0.35f),
    glm::vec3(0.35f),
    glm::uvec3(71u)
  );

  const talus::field::FieldProbe probe = talus::field::probe(field, outside);
  const talus::field::FieldProbe reference =
    talus::field::probe(analytic, outside);

  const double tilt =
    std::sqrt(3.0) * facetDepth(sphereMesh(), 0.25) / field.voxelSize.x;
  const double allowed =
    std::asin(tilt / glm::length(glm::dvec3(reference.gradient)));

  CHECK(std::abs(probe.distance - 0.05f) <= 4.0e-4f);
  CHECK(angleBetween(probe.normal, reference.normal) <= allowed);

  const glm::uvec3 bricks = talus::field::brickCounts(field);
  const size_t brickCount = static_cast<size_t>(bricks.x) * bricks.y * bricks.z;

  CHECK(field.brickLipschitz.size() == brickCount);
  CHECK(field.brickMinimum.size() == brickCount);
  CHECK(field.brickMaximum.size() == brickCount);
  CHECK(field.materials.size() == field.distances.size());
  CHECK(
    talus::field::stepBound(field, outside, glm::vec3(-1.0f, 0.0f, 0.0f)) > 0.0f
  );
}

TEST_CASE("bakeMesh refuses meshes that are not closed", "[field][meshsdf]") {
  const Mesh closed = box(glm::vec3(-0.1f), glm::vec3(0.1f));
  const glm::vec3 low(-0.2f);
  const glm::vec3 high(0.2f);
  const glm::uvec3 resolution(8u);

  CHECK_NOTHROW(baked(closed, low, high, resolution));

  Mesh removed = closed;
  removed.indices.resize(removed.indices.size() - 3u);
  CHECK_THROWS_AS(baked(removed, low, high, resolution), std::invalid_argument);

  Mesh flipped = closed;
  std::swap(flipped.indices[1], flipped.indices[2]);
  CHECK_THROWS_AS(baked(flipped, low, high, resolution), std::invalid_argument);

  Mesh duplicated = closed;
  duplicated.indices.insert(
    duplicated.indices.end(),
    {closed.indices[0], closed.indices[1], closed.indices[2]}
  );
  CHECK_THROWS_AS(
    baked(duplicated, low, high, resolution),
    std::invalid_argument
  );

  const Mesh folded{
    {glm::vec3(0.0f), glm::vec3(0.1f, 0.0f, 0.0f)},
    {0u, 0u, 1u}
  };
  CHECK_THROWS_AS(baked(folded, low, high, resolution), std::invalid_argument);

  Mesh noVertices = closed;
  noVertices.vertices.clear();
  CHECK_THROWS_AS(
    baked(noVertices, low, high, resolution),
    std::invalid_argument
  );

  Mesh noIndices = closed;
  noIndices.indices.clear();
  CHECK_THROWS_AS(
    baked(noIndices, low, high, resolution),
    std::invalid_argument
  );

  Mesh partial = closed;
  partial.indices.push_back(0u);
  CHECK_THROWS_AS(baked(partial, low, high, resolution), std::invalid_argument);

  Mesh beyond = closed;
  beyond.indices[0] = static_cast<uint32_t>(closed.vertices.size());
  CHECK_THROWS_AS(baked(beyond, low, high, resolution), std::invalid_argument);

  for (const float bad :
       {std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::infinity()}) {
    Mesh broken = closed;
    broken.vertices[3].y = bad;
    CHECK_THROWS_AS(
      baked(broken, low, high, resolution),
      std::invalid_argument
    );
  }

  for (uint32_t axis = 0u; axis != 3u; ++axis) {
    glm::uvec3 thin = resolution;
    thin[axis] = 1u;
    CHECK_THROWS_AS(baked(closed, low, high, thin), std::invalid_argument);
  }
}

TEST_CASE("mesh bake cost", "[.][meshsdfcost]") {
  const Mesh& mesh = sphereMesh();

  for (const uint32_t points : {71u, 117u}) {
    const auto start = std::chrono::steady_clock::now();
    const talus::field::SampledSdf field =
      baked(mesh, glm::vec3(-0.35f), glm::vec3(0.35f), glm::uvec3(points));
    const auto stop = std::chrono::steady_clock::now();

    std::printf(
      "meshsdfcost triangles %zu points %u voxel_mm %.2f ms %.1f\n",
      mesh.indices.size() / 3u,
      points,
      1000.0 * field.voxelSize.x,
      std::chrono::duration<double, std::milli>(stop - start).count()
    );
  }
}
