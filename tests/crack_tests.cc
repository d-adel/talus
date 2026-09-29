#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/deform/deform.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/physics/fieldshape/fieldshape.h>
#include <talus/physics/joltlibrary/joltlibrary.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using talus::field::Primitive;
using talus::field::SampledSdf;
using talus::physics::FieldShape;
using talus::physics::JoltLibrary;

namespace {

constexpr float OBJECT_HALF = 0.45f;
constexpr float CRACK_RADIANS = 0.4f;

constexpr uint32_t PROBE_SAMPLES = 4096u;
constexpr float PROBE_REACH = 0.42f;

constexpr uint32_t NO_COMPONENT = UINT32_MAX;

glm::vec3 crackNormal() {
  return glm::vec3(std::sin(CRACK_RADIANS), 0.0f, std::cos(CRACK_RADIANS));
}

class UnionFind {
public:
  explicit UnionFind(size_t count) : m_parent(count), m_rank(count, 0u) {
    std::iota(m_parent.begin(), m_parent.end(), uint32_t{0});
  }

  uint32_t find(uint32_t node) {
    while (m_parent[node] != node) {
      m_parent[node] = m_parent[m_parent[node]];
      node = m_parent[node];
    }

    return node;
  }

  void join(uint32_t first, uint32_t second) {
    uint32_t a = find(first);
    uint32_t b = find(second);

    if (a == b) {
      return;
    }

    if (m_rank[a] < m_rank[b]) {
      std::swap(a, b);
    }

    m_parent[b] = a;

    if (m_rank[a] == m_rank[b]) {
      ++m_rank[a];
    }
  }

private:
  std::vector<uint32_t> m_parent;
  std::vector<uint8_t> m_rank;
};

struct Components {
  std::vector<uint32_t> label;
  std::vector<uint32_t> population;
};

Components solidComponents(const SampledSdf& field) {
  const glm::uvec3 resolution = field.resolution;

  const uint32_t alongY = resolution.x;
  const uint32_t alongZ = resolution.x * resolution.y;

  const uint32_t count = static_cast<uint32_t>(field.distances.size());

  UnionFind solid(count);

  for (uint32_t z = 0u; z != resolution.z; ++z) {
    for (uint32_t y = 0u; y != resolution.y; ++y) {
      for (uint32_t x = 0u; x != resolution.x; ++x) {
        const uint32_t here = x + alongY * y + alongZ * z;

        if (field.distances[here] >= 0.0f) {
          continue;
        }

        if (x != 0u && field.distances[here - 1u] < 0.0f) {
          solid.join(here, here - 1u);
        }

        if (y != 0u && field.distances[here - alongY] < 0.0f) {
          solid.join(here, here - alongY);
        }

        if (z != 0u && field.distances[here - alongZ] < 0.0f) {
          solid.join(here, here - alongZ);
        }
      }
    }
  }

  std::vector<uint32_t> sizeOfRoot(count, 0u);

  for (uint32_t index = 0u; index != count; ++index) {
    if (field.distances[index] < 0.0f) {
      ++sizeOfRoot[solid.find(index)];
    }
  }

  std::vector<std::pair<uint32_t, uint32_t>> found;

  for (uint32_t root = 0u; root != count; ++root) {
    if (sizeOfRoot[root] != 0u) {
      found.emplace_back(sizeOfRoot[root], root);
    }
  }

  std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) {
    return a.first > b.first;
  });

  std::vector<uint32_t> rankOfRoot(count, NO_COMPONENT);

  Components components;
  components.population.reserve(found.size());

  for (uint32_t rank = 0u; rank != found.size(); ++rank) {
    rankOfRoot[found[rank].second] = rank;
    components.population.push_back(found[rank].first);
  }

  components.label.assign(count, NO_COMPONENT);

  for (uint32_t index = 0u; index != count; ++index) {
    if (field.distances[index] < 0.0f) {
      components.label[index] = rankOfRoot[solid.find(index)];
    }
  }

  return components;
}

struct CrackReport {
  uint32_t resolution = 0u;
  float voxel = 0.0f;
  float brickSpan = 0.0f;
  float crackThickness = 0.0f;

  uint32_t signChanges = 0u;
  float deepestBefore = 0.0f;
  float peakInCrack = 0.0f;
  float deepestAfter = 0.0f;

  float residual = 0.0f;

  uint32_t componentCount = 0u;
  uint32_t largestPopulation = 0u;
  uint32_t secondPopulation = 0u;

  uint32_t bricks = 0u;
  uint32_t strictSurfaceBricks = 0u;
  uint32_t gateSurfaceBricks = 0u;
  uint32_t seededBricks = 0u;

  uint32_t seedsLargestOnly = 0u;
  uint32_t seedsSecondOnly = 0u;
  uint32_t seedsBoth = 0u;
  uint32_t seedsNoSolid = 0u;

  uint32_t crackBricks = 0u;
  uint32_t crackBricksExclusive = 0u;
};

CrackReport measureCrack(uint32_t resolution, float crackVoxels) {
  CrackReport report;
  report.resolution = resolution;

  const std::vector<Primitive> primitives{
    fixture::box(glm::vec3(0.0f), glm::vec3(OBJECT_HALF))
  };

  const talus::field::Aabb box = talus::objectmodel::gridBox(primitives);

  SampledSdf field = talus::field::bake(
    primitives,
    box.minimum,
    box.maximum,
    glm::uvec3(resolution)
  );

  report.voxel = field.voxelSize.x;
  report.brickSpan =
    field.voxelSize.x * static_cast<float>(talus::field::BRICK_CELLS);

  const float halfThickness = 0.5f * crackVoxels * field.voxelSize.x;
  report.crackThickness = 2.0f * halfThickness;

  const Primitive cutter = talus::field::makeBox(
    glm::vec3(0.0f),
    glm::vec3(2.0f, 2.0f, halfThickness),
    CRACK_RADIANS,
    glm::vec3(0.0f, 1.0f, 0.0f),
    1u,
    talus::core::FIELD_OP_SUBTRACTION
  );

  const talus::field::DeformReport carved =
    talus::field::deformAndRepair(field, cutter);

  report.residual = carved.converged;

  const glm::vec3 normal = crackNormal();

  report.deepestBefore = std::numeric_limits<float>::max();
  report.deepestAfter = std::numeric_limits<float>::max();
  report.peakInCrack = std::numeric_limits<float>::lowest();

  bool previousNegative = false;

  for (uint32_t step = 0u; step <= PROBE_SAMPLES; ++step) {
    const float at =
      PROBE_REACH
      * (2.0f * static_cast<float>(step) / static_cast<float>(PROBE_SAMPLES) - 1.0f);

    const float here = talus::field::distance(field, normal * at);

    if (at < 0.0f) {
      report.deepestBefore = std::min(report.deepestBefore, here);
    } else {
      report.deepestAfter = std::min(report.deepestAfter, here);
    }

    report.peakInCrack = std::max(report.peakInCrack, here);

    const bool negative = here < 0.0f;

    if (step != 0u && negative != previousNegative) {
      ++report.signChanges;
    }

    previousNegative = negative;
  }

  const Components components = solidComponents(field);

  report.componentCount = static_cast<uint32_t>(components.population.size());

  report.largestPopulation =
    components.population.empty() ? 0u : components.population[0];

  report.secondPopulation =
    components.population.size() < 2u ? 0u : components.population[1];

  const glm::uvec3 counts = talus::field::brickCounts(field);
  const glm::uvec3 cells = field.resolution - glm::uvec3(1u);

  report.bricks = counts.x * counts.y * counts.z;

  for (uint32_t brick = 0u; brick != report.bricks; ++brick) {
    if (field.brickMinimum[brick] < 0.0f && field.brickMaximum[brick] > 0.0f) {
      ++report.strictSurfaceBricks;
    }

    if (talus::field::brickHoldsSurface(field, brick)) {
      ++report.gateSurfaceBricks;
    }
  }

  const FieldShape shape(field);

  report.seededBricks = static_cast<uint32_t>(shape.surfaceBricks().size());

  const uint32_t alongY = field.resolution.x;
  const uint32_t alongZ = field.resolution.x * field.resolution.y;

  for (const talus::physics::SurfaceBrick& surface : shape.surfaceBricks()) {
    const glm::uvec3 at(
      surface.brick % counts.x,
      (surface.brick / counts.x) % counts.y,
      surface.brick / (counts.x * counts.y)
    );

    const glm::uvec3 base = at * glm::uvec3(talus::field::BRICK_CELLS);
    const glm::uvec3 last =
      glm::min(base + glm::uvec3(talus::field::BRICK_CELLS), cells);

    bool sawLargest = false;
    bool sawSecond = false;
    bool sawSolid = false;
    bool sawCrack = false;

    for (uint32_t z = base.z; z <= last.z; ++z) {
      for (uint32_t y = base.y; y <= last.y; ++y) {
        for (uint32_t x = base.x; x <= last.x; ++x) {
          const uint32_t index = x + alongY * y + alongZ * z;

          const glm::vec3 point =
            field.origin + field.voxelSize * glm::vec3(x, y, z);

          const bool insideObject =
            glm::all(glm::lessThan(glm::abs(point), glm::vec3(OBJECT_HALF)));

          if (
            insideObject && std::fabs(glm::dot(point, normal)) < halfThickness
          ) {
            sawCrack = true;
          }

          const uint32_t label = components.label[index];

          if (label == NO_COMPONENT) {
            continue;
          }

          sawSolid = true;

          if (label == 0u) {
            sawLargest = true;
          } else if (label == 1u) {
            sawSecond = true;
          }
        }
      }
    }

    if (!sawSolid) {
      ++report.seedsNoSolid;
    } else if (sawLargest && sawSecond) {
      ++report.seedsBoth;
    } else if (sawLargest) {
      ++report.seedsLargestOnly;
    } else if (sawSecond) {
      ++report.seedsSecondOnly;
    }

    if (sawCrack) {
      ++report.crackBricks;

      if (sawSolid && !(sawLargest && sawSecond)) {
        ++report.crackBricksExclusive;
      }
    }
  }

  return report;
}

std::string describe(const CrackReport& report) {
  std::ostringstream out;

  out << std::fixed << std::setprecision(6);

  out << "\nresolution " << report.resolution << "\n"
      << "  voxel                   " << report.voxel << " m\n"
      << "  brick span              " << report.brickSpan << " m\n"
      << "  crack thickness         " << report.crackThickness
      << " m = " << (report.crackThickness / report.voxel)
      << " voxels = " << (report.crackThickness / report.brickSpan)
      << " bricks\n"
      << "  (a) sign changes        " << report.signChanges << "\n"
      << "      deepest before      " << report.deepestBefore << "\n"
      << "      peak in crack       " << report.peakInCrack << "\n"
      << "      deepest after       " << report.deepestAfter << "\n"
      << "  (b) redistance residual " << report.residual << "\n"
      << "  (c) solid components    " << report.componentCount << "\n"
      << "      largest             " << report.largestPopulation
      << " samples\n"
      << "      second              " << report.secondPopulation << " samples\n"
      << "  (d) bricks              " << report.bricks << "\n"
      << "      strict surface      " << report.strictSurfaceBricks << "\n"
      << "      shape surface gate  " << report.gateSurfaceBricks << "\n"
      << "      seeded bricks       " << report.seededBricks << "\n"
      << "      seeds largest only  " << report.seedsLargestOnly << "\n"
      << "      seeds second only   " << report.seedsSecondOnly << "\n"
      << "      seeds straddling    " << report.seedsBoth << "\n"
      << "      seeds no solid      " << report.seedsNoSolid << "\n"
      << "      seeded crack bricks " << report.crackBricks << "\n"
      << "      of those exclusive  " << report.crackBricksExclusive << "\n";

  return out.str();
}

void expectACrack(const CrackReport& report) {
  REQUIRE(report.signChanges == 2u);
  REQUIRE(report.deepestBefore < 0.0f);
  REQUIRE(report.peakInCrack > 0.0f);
  REQUIRE(report.deepestAfter < 0.0f);

  REQUIRE(report.residual < report.voxel);

  REQUIRE(report.componentCount == 2u);
  REQUIRE(report.secondPopulation * 2u > report.largestPopulation);

  REQUIRE(report.seededBricks == report.gateSurfaceBricks);
  REQUIRE(report.seedsLargestOnly > 0u);
  REQUIRE(report.seedsSecondOnly > 0u);
  REQUIRE(report.seedsBoth > 0u);
}

}

TEST_CASE("a two voxel crack at resolution 65", "[crack]") {
  JoltLibrary library;

  const CrackReport report = measureCrack(65u, 2.0f);

  WARN(describe(report));

  expectACrack(report);
}

TEST_CASE("a two voxel crack at resolution 128", "[crack]") {
  JoltLibrary library;

  const CrackReport report = measureCrack(128u, 2.0f);

  WARN(describe(report));

  expectACrack(report);
}

TEST_CASE("the same physical crack, resolved twice as finely", "[crack]") {
  JoltLibrary library;

  const CrackReport report = measureCrack(128u, 4.0f);

  WARN(describe(report));

  expectACrack(report);
}
