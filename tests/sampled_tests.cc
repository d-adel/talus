#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/field/analytic/analytic.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include "tolerances.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::AnalyticSample;
using talus::field::Primitive;
using talus::field::SampledSdf;

namespace {

constexpr uint32_t SPHERE = 0;
constexpr uint32_t UNION = 0;

Primitive makeSphere(
  const glm::vec3& centre,
  float radius,
  uint32_t material,
  uint32_t operation = UNION
) {
  Primitive primitive{};
  primitive.inverseModel =
    glm::inverse(glm::translate(glm::mat4(1.0f), centre));
  primitive.shape = glm::vec4(radius, 0.0f, 0.0f, 0.0f);
  primitive.shapeConfig = glm::uvec4(SPHERE, material, operation, 0);
  talus::field::updateBounds(primitive);

  return primitive;
}

}

TEST_CASE(
  "a bake reproduces the field exactly at lattice points",
  "[sampled]"
) {
  const std::vector<Primitive> primitives{
    makeSphere(glm::vec3(0.0f), 1.0f, 3),
    makeSphere(glm::vec3(1.2f, 0.3f, 0.0f), 0.6f, 5)
  };

  const glm::vec3 minimum(-2.0f);
  const glm::vec3 maximum(2.0f);
  const glm::uvec3 resolution(17u);

  const SampledSdf field =
    talus::field::bake(primitives, minimum, maximum, resolution);

  for (uint32_t z = 0; z < resolution.z; ++z) {
    for (uint32_t y = 0; y < resolution.y; ++y) {
      for (uint32_t x = 0; x < resolution.x; ++x) {
        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(x, y, z);

        const AnalyticSample exact = talus::field::sample(primitives, point);

        REQUIRE_THAT(
          talus::field::distance(field, point),
          WithinAbs(exact.distance, ANALYTIC_TOLERANCE)
        );
        REQUIRE(talus::field::material(field, point) == exact.material);
      }
    }
  }
}

TEST_CASE(
  "interpolation error between lattice points is bounded",
  "[sampled]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  float worst = 0.0f;

  for (float x = -1.9f; x <= 1.9f; x += 0.137f) {
    for (float y = -1.9f; y <= 1.9f; y += 0.137f) {
      for (float z = -1.9f; z <= 1.9f; z += 0.271f) {
        const glm::vec3 point(x, y, z);

        const float exact = talus::field::sample(primitives, point).distance;
        const float baked = talus::field::distance(field, point);

        worst = std::max(worst, std::abs(baked - exact));
      }
    }
  }

  REQUIRE(worst < 0.015f);
}

TEST_CASE("interpolation overestimates, so a march must not trust it") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  float worstOvershoot = 0.0f;

  for (float x = -1.9f; x <= 1.9f; x += 0.137f) {
    for (float y = -1.9f; y <= 1.9f; y += 0.137f) {
      for (float z = -1.9f; z <= 1.9f; z += 0.271f) {
        const glm::vec3 point(x, y, z);

        const float exact = talus::field::sample(primitives, point).distance;
        const float baked = talus::field::distance(field, point);

        worstOvershoot = std::max(worstOvershoot, baked - exact);
      }
    }
  }

  REQUIRE(worstOvershoot > 0.0f);
  REQUIRE(worstOvershoot < 0.015f);
}

TEST_CASE("outside the grid the distance is a lower bound", "[sampled]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(17u)
  );

  const std::vector<glm::vec3> outsidePoints{
    glm::vec3(3.0f, 0.0f, 0.0f),
    glm::vec3(-4.5f, 0.0f, 0.0f),
    glm::vec3(0.0f, 6.0f, 0.0f),
    glm::vec3(0.0f, 0.0f, -2.5f),
    glm::vec3(3.0f, 3.0f, 3.0f),
    glm::vec3(-2.5f, 4.0f, -3.5f)
  };

  for (const glm::vec3& point : outsidePoints) {
    const float exact = talus::field::sample(primitives, point).distance;
    const float outside = talus::field::distance(field, point);

    REQUIRE(outside > 0.0f);
    REQUIRE(outside <= exact + ANALYTIC_TOLERANCE);
  }
}

TEST_CASE("material ids are never interpolated", "[sampled]") {
  const std::vector<Primitive> primitives{
    makeSphere(glm::vec3(-0.5f, 0.0f, 0.0f), 0.8f, 2),
    makeSphere(glm::vec3(0.5f, 0.0f, 0.0f), 0.8f, 9)
  };

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(17u)
  );

  for (float x = -1.9f; x <= 1.9f; x += 0.083f) {
    const uint32_t baked =
      talus::field::material(field, glm::vec3(x, 0.05f, -0.05f));

    REQUIRE((baked == 2u || baked == 9u));
  }
}

TEST_CASE("the global bound is the largest brick bound", "[sampled][bound]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  REQUIRE(field.brickLipschitz.size() == 64u);

  float worst = 0.0f;
  for (float bound : field.brickLipschitz) {
    worst = std::max(worst, bound);
  }

  REQUIRE_THAT(field.globalLipschitz, WithinAbs(worst, ANALYTIC_TOLERANCE));

  REQUIRE(field.brickLipschitz[0] > 0.95f);
  REQUIRE(field.brickLipschitz[0] < 1.10f);

  REQUIRE_THAT(field.globalLipschitz, WithinAbs(1.7320508f, 1e-4f));
}

TEST_CASE(
  "the sampled gradient is the interpolant's own",
  "[sampled][gradient]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 probes[] = {
    glm::vec3(1.5f, 0.13f, -0.07f),
    glm::vec3(-0.11f, 1.42f, 0.23f),
    glm::vec3(0.31f, -0.19f, -1.61f),
    glm::vec3(0.83f, 0.79f, 0.71f)
  };

  for (const glm::vec3& probe : probes) {
    const glm::vec3 g = talus::field::gradient(field, probe);

    REQUIRE_THAT(glm::length(g), WithinAbs(1.0f, 0.05f));
    REQUIRE(glm::dot(glm::normalize(g), glm::normalize(probe)) > 0.99f);
  }
}

TEST_CASE("a stepBound step never crosses the surface", "[sampled][bound]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 targets[] = {
    glm::vec3(0.0f),
    glm::vec3(0.94f, 0.0f, 0.0f),
    glm::vec3(0.0f, -0.97f, 0.12f),
    glm::vec3(0.6f, 0.6f, 0.6f)
  };

  uint32_t stepsTaken = 0;

  for (float a = 0.0f; a < 6.2f; a += 0.4f) {
    for (float b = -1.2f; b < 1.3f; b += 0.6f) {
      const glm::vec3 origin(
        1.9f * std::cos(a) * std::cos(b),
        1.9f * std::sin(b),
        1.9f * std::sin(a) * std::cos(b)
      );

      for (const glm::vec3& target : targets) {
        const glm::vec3 direction = glm::normalize(target - origin);

        glm::vec3 point = origin;

        for (int i = 0; i < 64; ++i) {
          const float before = talus::field::distance(field, point);

          if (before <= 0.0f) {
            break;
          }

          const float step = talus::field::stepBound(field, point, direction);

          for (int s = 1; s <= 16; ++s) {
            const glm::vec3 along =
              point + direction * (step * static_cast<float>(s) / 16.0f);

            REQUIRE(talus::field::distance(field, along) > -ANALYTIC_TOLERANCE);
          }

          point += direction * step;
          ++stepsTaken;
        }
      }
    }
  }

  REQUIRE(stepsTaken > 1000u);
}

TEST_CASE(
  "a step stops at the wall of the brick it was measured in",
  "[sampled][bound]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const talus::field::SampleBox carve{
    glm::uvec3(12u, 30u, 16u),
    glm::uvec3(1u)
  };
  const std::vector<float> inside{-5.0f};
  const std::vector<uint8_t> material{3u};

  talus::field::write(field, carve, inside, material);

  const glm::vec3 point(-1.05f, 1.75f, 0.0f);
  const glm::vec3 direction(1.0f, 0.0f, 0.0f);

  REQUIRE(talus::field::distance(field, point) > 1.0f);

  const float step = talus::field::stepBound(field, point, direction);

  for (int s = 1; s <= 64; ++s) {
    const glm::vec3 along =
      point + direction * (step * static_cast<float>(s) / 64.0f);

    REQUIRE(talus::field::distance(field, along) > -ANALYTIC_TOLERANCE);
  }
}

TEST_CASE(
  "a step never shrinks to nothing at a brick wall",
  "[sampled][bound]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 direction(1.0f, 0.0f, 0.0f);
  const glm::vec3 point(-1.0001f, 1.8f, 0.0f);

  const float step = talus::field::stepBound(field, point, direction);

  const glm::vec3 voxel = field.voxelSize;
  const float smallest = std::min(voxel.x, std::min(voxel.y, voxel.z));

  REQUIRE(step >= 0.5f * smallest);

  REQUIRE(step <= talus::field::distance(field, point));
}

TEST_CASE(
  "a flat brick does not license a step into a carved neighbour",
  "[sampled][bound]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const talus::field::SampleBox carve{
    glm::uvec3(17u, 30u, 16u),
    glm::uvec3(1u)
  };
  const std::vector<float> inside{-5.0f};
  const std::vector<uint8_t> material{3u};

  talus::field::write(field, carve, inside, material);

  const float wall = -2.0f + 16.0f * field.voxelSize.x;
  const glm::vec3 point(wall - 0.0001f, 1.75f, 0.0f);
  const glm::vec3 direction(1.0f, 0.0f, 0.0f);

  REQUIRE(talus::field::distance(field, point) > 0.0f);

  const float step = talus::field::stepBound(field, point, direction);

  for (int s = 1; s <= 64; ++s) {
    const glm::vec3 along =
      point + direction * (step * static_cast<float>(s) / 64.0f);

    REQUIRE(talus::field::distance(field, along) > -ANALYTIC_TOLERANCE);
  }
}

TEST_CASE(
  "a write rebuilds the bricks on both sides of a boundary",
  "[sampled][write]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(17u)
  );

  const std::vector<float> before = field.brickLipschitz;

  const talus::field::SampleBox box{glm::uvec3(8u, 4u, 4u), glm::uvec3(1u)};
  const std::vector<float> spike{5.0f};
  const std::vector<uint8_t> material{3u};

  talus::field::write(field, box, spike, material);

  const uint32_t touched = 0u;
  const uint32_t neighbour = 1u;
  const uint32_t untouched = 6u;

  REQUIRE(field.brickLipschitz[touched] > before[touched]);
  REQUIRE(field.brickLipschitz[neighbour] > before[neighbour]);
  REQUIRE_THAT(
    field.brickLipschitz[untouched],
    WithinAbs(before[untouched], ANALYTIC_TOLERANCE)
  );

  REQUIRE_THAT(
    field.globalLipschitz,
    WithinAbs(
      std::max(field.brickLipschitz[touched], field.brickLipschitz[neighbour]),
      ANALYTIC_TOLERANCE
    )
  );
}

namespace {

uint32_t bricksPerAxis(const SampledSdf& field) {
  const uint32_t cells = field.resolution.x - 1u;

  return (cells + talus::field::BRICK_CELLS - 1u) / talus::field::BRICK_CELLS;
}

uint32_t brickAt(const SampledSdf& field, const glm::vec3& point) {
  const glm::vec3 cell = (point - field.origin) / field.voxelSize;
  const glm::uvec3 brick =
    glm::uvec3(glm::floor(cell)) / glm::uvec3(talus::field::BRICK_CELLS);
  const uint32_t counts = bricksPerAxis(field);

  return brick.x + counts * (brick.y + counts * brick.z);
}

}

TEST_CASE("every brick the surface crosses reports it", "[sampled][bound]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  for (uint32_t i = 0; i < 400u; ++i) {
    const float k = (static_cast<float>(i) + 0.5f) / 400.0f;
    const float phi = std::acos(1.0f - 2.0f * k);
    const float theta = 6.28318531f * 0.618034f * static_cast<float>(i);

    const glm::vec3 on(
      std::sin(phi) * std::cos(theta),
      std::sin(phi) * std::sin(theta),
      std::cos(phi)
    );

    REQUIRE(talus::field::brickHoldsSurface(field, brickAt(field, on)));
  }

  uint32_t holding = 0;

  for (uint32_t brick = 0; brick < field.brickMinimum.size(); ++brick) {
    if (talus::field::brickHoldsSurface(field, brick)) {
      ++holding;
    }
  }

  REQUIRE(holding < field.brickMinimum.size());
}

TEST_CASE("a brick clear of the surface holds none", "[sampled][bound]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  REQUIRE_FALSE(talus::field::brickHoldsSurface(field, 0u));
  REQUIRE(field.brickMinimum[0] > 0.0f);
}

TEST_CASE("an edit refreshes the value range", "[sampled][bound]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  REQUIRE_FALSE(talus::field::brickHoldsSurface(field, 0u));

  const talus::field::SampleBox box{glm::uvec3(2u), glm::uvec3(2u)};
  const std::vector<float>
    values{-1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  const std::vector<uint8_t> materials(8u, 3u);

  talus::field::write(field, box, values, materials);

  REQUIRE(talus::field::brickHoldsSurface(field, 0u));
  REQUIRE(field.brickMinimum[0] <= -1.0f);
}

TEST_CASE("a probe agrees with the field inside the grid", "[sampled][probe]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 points[] = {
    glm::vec3(0.0f),
    glm::vec3(1.5f, 0.13f, -0.07f),
    glm::vec3(-0.11f, 1.42f, 0.23f),
    glm::vec3(0.83f, 0.79f, 0.71f)
  };

  for (const glm::vec3& point : points) {
    const talus::field::FieldProbe hit = talus::field::probe(field, point);
    const glm::vec3 g = talus::field::gradient(field, point);

    REQUIRE_THAT(
      hit.distance,
      WithinAbs(talus::field::distance(field, point), ANALYTIC_TOLERANCE)
    );
    REQUIRE_THAT(glm::length(hit.normal), WithinAbs(1.0f, ANALYTIC_TOLERANCE));
    REQUIRE(glm::dot(hit.normal, glm::normalize(g)) > 0.9999f);
  }
}

TEST_CASE(
  "outside the grid a probe is accurate where the distance is not",
  "[sampled][probe]"
) {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 far(5.0f, 0.0f, 0.0f);

  REQUIRE_THAT(talus::field::distance(field, far), WithinAbs(3.0f, 1e-4f));

  const talus::field::FieldProbe hit = talus::field::probe(field, far);

  REQUIRE_THAT(
    hit.distance,
    WithinAbs(talus::field::sample(primitives, far).distance, 1e-3f)
  );
  REQUIRE(hit.distance > talus::field::distance(field, far));
  REQUIRE_THAT(hit.normal.x, WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("a probe always names a direction", "[sampled][probe]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  const SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const glm::vec3 points[] = {
    glm::vec3(0.0f),
    glm::vec3(2.0f, 2.0f, 2.0f),
    glm::vec3(-9.0f, 4.0f, 7.0f),
    glm::vec3(0.0f, 40.0f, 0.0f)
  };

  for (const glm::vec3& point : points) {
    const talus::field::FieldProbe hit = talus::field::probe(field, point);

    REQUIRE_THAT(glm::length(hit.normal), WithinAbs(1.0f, 1e-5f));
  }
}

namespace {

void trueBrickRange(
  const SampledSdf& field,
  const glm::uvec3& brick,
  float& lowest,
  float& highest
) {
  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);
  const glm::uvec3 base = brick * glm::uvec3(talus::field::BRICK_CELLS);

  const glm::uvec3 start = glm::max(base, glm::uvec3(1u)) - glm::uvec3(1u);
  const glm::uvec3 end =
    glm::min(base + glm::uvec3(talus::field::BRICK_CELLS + 1u), cells);

  lowest = std::numeric_limits<float>::max();
  highest = std::numeric_limits<float>::lowest();

  for (uint32_t z = start.z; z <= end.z; ++z) {
    for (uint32_t y = start.y; y <= end.y; ++y) {
      for (uint32_t x = start.x; x <= end.x; ++x) {
        const uint32_t at =
          x + field.resolution.x * (y + field.resolution.y * z);

        lowest = std::min(lowest, field.distances[at]);
        highest = std::max(highest, field.distances[at]);
      }
    }
  }
}

}

TEST_CASE("a write reports exactly the bricks it moved", "[sampled][write]") {
  const std::vector<Primitive> primitives{makeSphere(glm::vec3(0.0f), 1.0f, 3)};

  SampledSdf field = talus::field::bake(
    primitives,
    glm::vec3(-2.0f),
    glm::vec3(2.0f),
    glm::uvec3(33u)
  );

  const std::vector<float> lowBefore = field.brickMinimum;
  const std::vector<float> highBefore = field.brickMaximum;

  const uint32_t wall = talus::field::BRICK_CELLS;

  const talus::field::SampleBox carve{glm::uvec3(wall), glm::uvec3(1u)};
  const std::vector<float> inside{-5.0f};
  const std::vector<uint8_t> material{3u};

  const talus::field::WriteReport report =
    talus::field::write(field, carve, inside, material);

  const glm::uvec3 counts = talus::field::brickCounts(field);

  glm::uvec3 movedLow(counts);
  glm::uvec3 movedHigh(0u);
  uint32_t moved = 0;

  for (uint32_t bz = 0; bz < counts.z; ++bz) {
    for (uint32_t by = 0; by < counts.y; ++by) {
      for (uint32_t bx = 0; bx < counts.x; ++bx) {
        const uint32_t index = bx + counts.x * (by + counts.y * bz);

        float lowest = 0.0f;
        float highest = 0.0f;
        trueBrickRange(field, glm::uvec3(bx, by, bz), lowest, highest);

        REQUIRE(field.brickMinimum[index] == lowest);
        REQUIRE(field.brickMaximum[index] == highest);

        if (
          field.brickMinimum[index] == lowBefore[index]
          && field.brickMaximum[index] == highBefore[index]
        ) {
          continue;
        }

        ++moved;
        movedLow = glm::min(movedLow, glm::uvec3(bx, by, bz));
        movedHigh = glm::max(movedHigh, glm::uvec3(bx, by, bz));
      }
    }
  }

  REQUIRE(moved > 0u);

  REQUIRE(report.brickBegin.x == movedLow.x);
  REQUIRE(report.brickBegin.y == movedLow.y);
  REQUIRE(report.brickBegin.z == movedLow.z);

  REQUIRE(report.brickEnd.x == movedHigh.x + 1u);
  REQUIRE(report.brickEnd.y == movedHigh.y + 1u);
  REQUIRE(report.brickEnd.z == movedHigh.z + 1u);

  REQUIRE(report.brickBegin.x == 0u);
  REQUIRE(report.samples.minimum.x == wall);
  REQUIRE(report.samples.extent.x == 1u);
}
