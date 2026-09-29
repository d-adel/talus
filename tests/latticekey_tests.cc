#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/core/config/config.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/renderer/volumepool/volumepool.h>

#include <glm/glm.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

using talus::field::SampledSdf;
using talus::renderer::latticeKey;
using talus::renderer::sameLattice;
using talus::renderer::volumeCapacityFor;

namespace {

SampledSdf roundLattice() {
  const std::array<talus::field::Primitive, 1> solid{
    fixture::sphere(glm::vec3(0.0f), 0.3f)
  };

  return talus::field::bake(
    solid,
    glm::vec3(-0.5f),
    glm::vec3(0.5f),
    glm::uvec3(9u)
  );
}

}

TEST_CASE("equal lattices have one key and are the same", "[latticekey]") {
  const SampledSdf first = roundLattice();
  const SampledSdf second = roundLattice();

  REQUIRE(&first != &second);
  REQUIRE(latticeKey(first) == latticeKey(second));
  REQUIRE(sameLattice(first, second));
}

TEST_CASE("one changed sample makes another lattice", "[latticekey]") {
  const SampledSdf first = roundLattice();

  SampledSdf distance = first;
  distance.distances[40] =
    std::nextafter(distance.distances[40], std::numeric_limits<float>::max());

  REQUIRE_FALSE(latticeKey(distance) == latticeKey(first));
  REQUIRE_FALSE(sameLattice(distance, first));

  SampledSdf material = first;
  material.materials[40] = uint8_t(material.materials[40] + 1u);

  REQUIRE_FALSE(latticeKey(material) == latticeKey(first));
  REQUIRE_FALSE(sameLattice(material, first));
}

TEST_CASE(
  "equal samples placed differently make another lattice",
  "[latticekey]"
) {
  const SampledSdf first = roundLattice();

  SampledSdf moved = first;
  moved.origin.x += 1.0f;

  REQUIRE_FALSE(latticeKey(moved) == latticeKey(first));
  REQUIRE_FALSE(sameLattice(moved, first));

  SampledSdf scaled = first;
  scaled.voxelSize *= 2.0f;

  REQUIRE_FALSE(latticeKey(scaled) == latticeKey(first));
  REQUIRE_FALSE(sameLattice(scaled, first));
}

TEST_CASE("a lattice key never matches a primitive key", "[latticekey]") {
  const talus::renderer::VolumeKey lattice = latticeKey(roundLattice());

  talus::renderer::VolumeKey primitive = lattice;
  primitive.lattice = false;

  REQUIRE_FALSE(primitive == lattice);
}

TEST_CASE("the volume capacity doubles up to its ceiling", "[latticekey]") {
  const uint32_t first = talus::renderer::INITIAL_SDF_VOLUMES;
  const uint32_t ceiling = talus::core::MAX_SDF_VOLUMES;

  REQUIRE(ceiling == 4095u);
  REQUIRE(
    uint64_t(ceiling) * talus::core::BRICK_TABLE_STRIDE < (uint64_t(1) << 24)
  );
  REQUIRE(
    uint64_t(ceiling + 1u) * talus::core::BRICK_TABLE_STRIDE
    >= (uint64_t(1) << 24)
  );

  REQUIRE(volumeCapacityFor(0u) == first);
  REQUIRE(volumeCapacityFor(first) == first);
  REQUIRE(volumeCapacityFor(first + 1u) == 2u * first);
  REQUIRE(volumeCapacityFor(2u * first + 1u) == ceiling);
  REQUIRE(volumeCapacityFor(ceiling) == ceiling);
  REQUIRE(volumeCapacityFor(ceiling + 1u) == ceiling);
}
