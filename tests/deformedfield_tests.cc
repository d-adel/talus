#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/deformedfield/deformedfield.h>
#include <talus/deformable/solver/solver.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>
#include <talus/pose.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

constexpr float VOXEL = 0.025f;
constexpr float SINK = 0.005f;
const glm::vec3 HALF(0.1f, 0.05f, 0.1f);

talus::PhysicalMaterial barMaterial() {
  talus::PhysicalMaterial material;

  material.density = 1000.0f;
  material.youngsModulus = 2.0e8f;
  material.poissonsRatio = 0.3f;
  material.yieldStress = 2.0e5f;

  return material;
}

struct Block {
  talus::field::SampledSdf field;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

Block block(talus::deformable::VolumeElementType element) {
  const glm::vec3 reach = HALF + glm::vec3(2.0f * VOXEL);
  const std::vector<talus::PhysicalMaterial> lanes{barMaterial()};
  Block made;

  made.field = talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(glm::vec3(0.0f), HALF)},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / VOXEL)) + glm::uvec3(1u)
  );
  made.mesh = talus::deformable::buildMesh(
    made.field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    element
  );

  return made;
}

std::vector<glm::vec3> restOf(const talus::PreparedMesh& mesh) {
  return std::vector<glm::vec3>(mesh.restPositions.begin(), mesh.restPositions.end());
}

float largestChange(const talus::field::SampledSdf& first, const talus::field::SampledSdf& second) {
  float largest = 0.0f;

  for (size_t index = 0u; index != first.distances.size(); ++index) {
    largest = std::max(largest, std::abs(first.distances[index] - second.distances[index]));
  }

  return largest;
}

}

TEST_CASE("a lattice at rest rebuilds its own field", "[deformable][deformedfield]") {
  const Block made = block(talus::deformable::VolumeElementType::Hex);

  REQUIRE(made.mesh != nullptr);
  REQUIRE(!made.mesh->hexes.empty());

  const std::vector<glm::vec3> position = restOf(*made.mesh);
  talus::field::SampledSdf current = made.field;

  const talus::field::WriteReport report = talus::deformable::bakeDeformedField(
    made.field, *made.mesh, position, talus::Pose{}, current
  );

  CHECK(report.samples.extent == glm::uvec3(0u));
  CHECK(largestChange(current, made.field) <= 1.0e-5f);
}

TEST_CASE("a lattice moved and turned rigidly rebuilds its own field", "[deformable][deformedfield]") {
  const Block made = block(talus::deformable::VolumeElementType::Hex);

  REQUIRE(made.mesh != nullptr);

  talus::Pose pose;

  pose.position = glm::vec3(0.3f, 0.2f, -0.1f);
  pose.rotation = glm::angleAxis(0.5f, glm::normalize(glm::vec3(0.2f, 1.0f, 0.1f)));

  std::vector<glm::vec3> position = restOf(*made.mesh);

  for (glm::vec3& node : position) {
    node = pose.position + pose.rotation * node;
  }

  talus::field::SampledSdf current = made.field;

  static_cast<void>(talus::deformable::bakeDeformedField(made.field, *made.mesh, position, pose, current));

  CHECK(largestChange(current, made.field) <= 1.0e-5f);
}

TEST_CASE("a lattice whose top sinks rebuilds a field with a lower top", "[deformable][deformedfield]") {
  const Block made = block(talus::deformable::VolumeElementType::Hex);

  REQUIRE(made.mesh != nullptr);

  float top = -1.0f;

  for (const glm::vec3& node : made.mesh->restPositions) {
    top = std::max(top, node.y);
  }

  std::vector<glm::vec3> position = restOf(*made.mesh);

  for (glm::vec3& node : position) {
    if (std::abs(node.y - top) < 1.0e-5f) {
      node.y -= SINK;
    }
  }

  talus::field::SampledSdf current = made.field;

  const talus::field::WriteReport report = talus::deformable::bakeDeformedField(
    made.field, *made.mesh, position, talus::Pose{}, current
  );

  const glm::vec3 bottom(0.0f, -top, 0.0f);

  CHECK(std::abs(talus::field::distance(current, glm::vec3(0.0f, top - SINK, 0.0f))) <= 2.0e-4f);
  CHECK(talus::field::distance(current, glm::vec3(0.0f, top, 0.0f)) > 0.8f * SINK);
  CHECK(std::abs(talus::field::distance(current, bottom) - talus::field::distance(made.field, bottom)) <= 1.0e-6f);
  CHECK(report.samples.extent.y > 0u);
  CHECK(report.samples.extent.y < current.resolution.y);
}

TEST_CASE("a tetrahedral lattice leaves its field unchanged", "[deformable][deformedfield]") {
  const Block made = block(talus::deformable::VolumeElementType::Tet);

  REQUIRE(made.mesh != nullptr);

  std::vector<glm::vec3> position = restOf(*made.mesh);

  for (glm::vec3& node : position) {
    node.y -= SINK;
  }

  talus::field::SampledSdf current = made.field;

  const talus::field::WriteReport report = talus::deformable::bakeDeformedField(
    made.field, *made.mesh, position, talus::Pose{}, current
  );

  CHECK(report.samples.extent == glm::uvec3(0u));
  CHECK(largestChange(current, made.field) == 0.0f);
}

TEST_CASE("a rebuild refuses a current field laid out on a different lattice", "[deformable][deformedfield]") {
  const Block made = block(talus::deformable::VolumeElementType::Hex);

  REQUIRE(made.mesh != nullptr);

  const std::vector<glm::vec3> position = restOf(*made.mesh);
  talus::field::SampledSdf shifted = made.field;

  shifted.origin += glm::vec3(VOXEL, 0.0f, 0.0f);

  CHECK_THROWS_AS(
    talus::deformable::bakeDeformedField(made.field, *made.mesh, position, talus::Pose{}, shifted),
    std::invalid_argument
  );

  talus::field::SampledSdf finer = made.field;

  finer.voxelSize *= 0.5f;

  CHECK_THROWS_AS(
    talus::deformable::bakeDeformedField(made.field, *made.mesh, position, talus::Pose{}, finer),
    std::invalid_argument
  );
}
