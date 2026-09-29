#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <talus/core/config/config.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/deform/deform.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <optional>
#include <vector>

using Catch::Matchers::WithinAbs;
using talus::field::Primitive;
using talus::field::SampledSdf;

namespace {

constexpr float SPAN = 2.0f;
constexpr uint32_t RESOLUTION = 65u;
constexpr float VOXEL = 2.0f * SPAN / static_cast<float>(RESOLUTION - 1u);

Primitive carver(const glm::vec3& centre, float radius) {
  Primitive cutter = talus::field::makeSphere(
    centre,
    radius,
    7u,
    talus::core::FIELD_OP_SUBTRACTION
  );

  talus::field::updateBounds(cutter);

  return cutter;
}

SampledSdf unitSphere() {
  const std::vector<Primitive> primitives{
    talus::field::makeSphere(glm::vec3(0.0f), 1.0f, 3u)
  };

  return talus::field::bake(
    primitives,
    glm::vec3(-SPAN),
    glm::vec3(SPAN),
    glm::uvec3(RESOLUTION)
  );
}

std::optional<float> crossingAlongX(const SampledSdf& field, float until) {
  float previous = talus::field::distance(field, glm::vec3(0.0f));

  for (uint32_t step = 1u; step <= 4096u; ++step) {
    const float x = until * static_cast<float>(step) / 4096.0f;
    const float here = talus::field::distance(field, glm::vec3(x, 0.0f, 0.0f));

    if (previous < 0.0f && here >= 0.0f) {
      return x;
    }

    previous = here;
  }

  return std::nullopt;
}

}

TEST_CASE(
  "a subtraction moves the surface to the cutter's near wall",
  "[deform]"
) {
  SampledSdf field = unitSphere();

  const std::optional<float> before = crossingAlongX(field, 1.5f);

  REQUIRE(before.has_value());
  REQUIRE_THAT(*before, WithinAbs(1.0f, VOXEL));

  talus::field::deform(field, carver(glm::vec3(1.0f, 0.0f, 0.0f), 0.35f));

  const std::optional<float> after = crossingAlongX(field, 1.5f);

  REQUIRE(after.has_value());
  REQUIRE_THAT(*after, WithinAbs(0.65f, VOXEL));
}

TEST_CASE(
  "a subtraction agrees with max(old, -cutter) inside the band",
  "[deform]"
) {
  SampledSdf field = unitSphere();

  const SampledSdf before = field;

  const glm::vec3 centre(1.0f, 0.0f, 0.0f);
  const float radius = 0.35f;

  talus::field::deform(field, carver(centre, radius));

  const glm::uvec3 resolution = field.resolution;
  const float margin =
    static_cast<float>(talus::field::DEFORM_BAND_VOXELS) * VOXEL;

  const talus::field::SampleBox box = talus::field::affectedBox(
    field,
    carver(centre, radius),
    talus::field::DEFORM_BAND_VOXELS
  );

  const glm::uvec3 beyond = box.minimum + box.extent;

  for (uint32_t z = 0u; z < resolution.z; ++z) {
    for (uint32_t y = 0u; y < resolution.y; ++y) {
      for (uint32_t x = 0u; x < resolution.x; ++x) {
        const uint32_t index = x + resolution.x * (y + resolution.y * z);

        const glm::vec3 point =
          field.origin + field.voxelSize * glm::vec3(x, y, z);

        const float cutter = glm::length(point - centre) - radius;
        const float want = std::max(before.distances[index], -cutter);

        const bool written = x >= box.minimum.x && x < beyond.x
                             && y >= box.minimum.y && y < beyond.y
                             && z >= box.minimum.z && z < beyond.z;

        if (written) {
          REQUIRE(field.distances[index] == want);
        } else {
          REQUIRE(field.distances[index] == before.distances[index]);
          REQUIRE((want == before.distances[index] || want <= -margin));
        }
      }
    }
  }
}

TEST_CASE("a repair reports the cells it removed", "[deform]") {
  SampledSdf field = unitSphere();

  const SampledSdf before = field;

  const talus::field::DeformReport report = talus::field::deformAndRepair(
    field,
    carver(glm::vec3(1.0f, 0.0f, 0.0f), 0.35f)
  );

  uint32_t flipped = 0u;

  for (size_t i = 0; i < field.distances.size(); ++i) {
    if (before.distances[i] < 0.0f && field.distances[i] >= 0.0f) {
      ++flipped;
    }
  }

  REQUIRE(report.cellsRemoved > 0u);
  REQUIRE(report.cellsRemoved == flipped);

  SampledSdf missed = unitSphere();

  REQUIRE(
    talus::field::deformAndRepair(
      missed,
      carver(glm::vec3(100.0f, 0.0f, 0.0f), 0.35f)
    )
      .cellsRemoved
    == 0u
  );
}

TEST_CASE("a deformation that misses the grid writes nothing", "[deform]") {
  SampledSdf field = unitSphere();

  const SampledSdf before = field;

  const talus::field::WriteReport report =
    talus::field::deform(field, carver(glm::vec3(100.0f, 0.0f, 0.0f), 0.35f));

  REQUIRE(report.samples.extent.x == 0u);
  REQUIRE(report.brickBegin.x == report.brickEnd.x);
  REQUIRE(report.brickBegin.y == report.brickEnd.y);
  REQUIRE(report.brickBegin.z == report.brickEnd.z);

  REQUIRE(field.distances == before.distances);
  REQUIRE(field.materials == before.materials);
}

TEST_CASE("a deformation over the lattice edge clamps", "[deform]") {
  SampledSdf field = unitSphere();

  REQUIRE_NOTHROW(talus::field::deform(field, carver(glm::vec3(-SPAN), 0.5f)));

  const talus::field::SampleBox box =
    talus::field::affectedBox(field, carver(glm::vec3(-SPAN), 0.5f), 0u);

  REQUIRE(box.minimum.x == 0u);
  REQUIRE(box.minimum.y == 0u);
  REQUIRE(box.minimum.z == 0u);
  REQUIRE(box.extent.x < RESOLUTION);
}

TEST_CASE("subtraction exposes material, it does not paint it", "[deform]") {
  SampledSdf field = unitSphere();

  const SampledSdf before = field;

  talus::field::deform(field, carver(glm::vec3(1.0f, 0.0f, 0.0f), 0.35f));

  uint32_t moved = 0u;

  for (size_t i = 0; i < field.distances.size(); ++i) {
    REQUIRE(field.materials[i] == before.materials[i]);

    if (field.distances[i] != before.distances[i]) {
      ++moved;
    }
  }

  REQUIRE(moved > 0u);
}

TEST_CASE("a union brings its own material with it", "[deform]") {
  SampledSdf field = unitSphere();

  const SampledSdf before = field;

  Primitive blob = talus::field::makeSphere(
    glm::vec3(1.0f, 0.0f, 0.0f),
    0.35f,
    7u,
    talus::core::FIELD_OP_UNION
  );

  talus::field::updateBounds(blob);
  talus::field::deform(field, blob);

  uint32_t painted = 0u;

  for (size_t i = 0; i < field.materials.size(); ++i) {
    if (field.materials[i] != before.materials[i]) {
      REQUIRE(field.materials[i] == 7u);
      ++painted;
    }
  }

  REQUIRE(talus::field::material(field, glm::vec3(1.2f, 0.0f, 0.0f)) == 7u);
  REQUIRE(talus::field::material(field, glm::vec3(0.0f)) == 3u);

  REQUIRE(painted > 0u);
  REQUIRE(painted < 15000u);
}

TEST_CASE("a union welds material on, at the far wall", "[deform]") {
  SampledSdf field = unitSphere();

  const std::optional<float> before = crossingAlongX(field, 2.5f);

  REQUIRE(before.has_value());
  REQUIRE_THAT(*before, WithinAbs(1.0f, VOXEL));

  Primitive blob = talus::field::makeSphere(
    glm::vec3(1.0f, 0.0f, 0.0f),
    0.35f,
    7u,
    talus::core::FIELD_OP_UNION
  );

  talus::field::updateBounds(blob);
  talus::field::deform(field, blob);

  const std::optional<float> after = crossingAlongX(field, 2.5f);

  REQUIRE(after.has_value());
  REQUIRE_THAT(*after, WithinAbs(1.35f, VOXEL));
}

TEST_CASE("an operation only the general form can express", "[deform]") {
  SampledSdf field = unitSphere();

  Primitive chisel = talus::field::makeBox(
    glm::vec3(1.0f, 0.0f, 0.0f),
    glm::vec3(0.4f),
    0.0f,
    glm::vec3(0.0f, 1.0f, 0.0f),
    7u,
    talus::core::FIELD_OP_SMOOTH_SUBTRACTION,
    0.1f
  );

  talus::field::updateBounds(chisel);

  const talus::field::WriteReport report = talus::field::deform(field, chisel);

  REQUIRE(report.samples.extent.x > 0u);
  REQUIRE(report.brickEnd.x > report.brickBegin.x);

  const std::optional<float> after = crossingAlongX(field, 1.5f);

  REQUIRE(after.has_value());
  REQUIRE_THAT(*after, WithinAbs(0.6f, VOXEL));
}

TEST_CASE(
  "a box tool reaches its own box, not its bounding sphere",
  "[deform]"
) {
  SampledSdf field = unitSphere();

  Primitive slab = talus::field::makeBox(
    glm::vec3(0.0f),
    glm::vec3(1.5f, 0.1f, 0.1f),
    0.0f,
    glm::vec3(0.0f, 1.0f, 0.0f),
    7u,
    talus::core::FIELD_OP_SUBTRACTION,
    0.0f
  );

  talus::field::updateBounds(slab);

  const talus::field::SampleBox box =
    talus::field::affectedBox(field, slab, 0u);

  const glm::vec3 lowest =
    field.origin + field.voxelSize * glm::vec3(box.minimum);
  const glm::vec3 highest =
    field.origin
    + field.voxelSize * glm::vec3(box.minimum + box.extent - glm::uvec3(1u));

  REQUIRE(highest.x - lowest.x > 2.0f);
  REQUIRE(highest.y - lowest.y < 0.5f);
  REQUIRE(highest.z - lowest.z < 0.5f);

  REQUIRE(slab.bounds.w > 1.5f);
}

TEST_CASE("a sphere tool reaches exactly what it used to", "[deform]") {
  SampledSdf field = unitSphere();

  const Primitive cutter = carver(glm::vec3(0.25f, 0.0f, 0.0f), 0.4f);

  const talus::field::SampleBox box =
    talus::field::affectedBox(field, cutter, 0u);

  const glm::vec3 centre(cutter.bounds);
  const glm::vec3 reach(cutter.bounds.w);

  const glm::vec3 lowest =
    glm::floor((centre - reach - field.origin) / field.voxelSize);
  const glm::vec3 highest =
    glm::ceil((centre + reach - field.origin) / field.voxelSize);

  REQUIRE(box.minimum == glm::uvec3(glm::max(lowest, glm::vec3(0.0f))));
  REQUIRE(box.minimum + box.extent == glm::uvec3(highest) + glm::uvec3(1u));
}

TEST_CASE("a cylinder tool no longer sweeps the whole grid", "[deform]") {
  SampledSdf field = unitSphere();

  Primitive drill = talus::field::makeCylinder(
    glm::vec3(0.0f),
    0.15f,
    0.2f,
    0.0f,
    glm::vec3(0.0f, 1.0f, 0.0f),
    7u,
    talus::core::FIELD_OP_SUBTRACTION,
    0.0f
  );

  talus::field::updateBounds(drill);

  REQUIRE_THAT(
    drill.bounds.w,
    Catch::Matchers::WithinAbs(std::sqrt(0.15f * 0.15f + 0.2f * 0.2f), 1e-6)
  );

  const talus::field::SampleBox box =
    talus::field::affectedBox(field, drill, 0u);

  REQUIRE(box.extent.x < field.resolution.x / 2u);
  REQUIRE(box.extent.y < field.resolution.y / 2u);
  REQUIRE(box.extent.z < field.resolution.z / 2u);
}

TEST_CASE("a cylinder is not culled by its own bound", "[deform]") {
  Primitive drill = talus::field::makeCylinder(
    glm::vec3(0.0f),
    0.15f,
    0.2f,
    0.0f,
    glm::vec3(0.0f, 1.0f, 0.0f),
    7u,
    talus::core::FIELD_OP_SUBTRACTION,
    0.0f
  );

  talus::field::updateBounds(drill);

  const glm::vec3 onTheRim(0.15f, 0.2f, 0.0f);

  REQUIRE(
    drill.bounds.w >= glm::length(onTheRim - glm::vec3(drill.bounds)) - 1e-6f
  );
}
