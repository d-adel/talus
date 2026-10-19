#include <catch2/catch_test_macros.hpp>

#include <talus/destruction/building/building.h>
#include <talus/destruction/chunker/chunker.h>
#include <fixtures/district.h>
#include <talus/destruction/materials/materials.h>
#include <talus/destruction/stress/factor.h>
#include <talus/destruction/stress/stress.h>
#include <talus/destruction/structure/structure.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <limits>
#include <span>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using talus::destruction::Anisotropy;
using talus::destruction::ModeLimits;
using talus::destruction::ShearRule;
using talus::destruction::StructuralMaterial;

bool sameModes(const ModeLimits& a, const ModeLimits& b) {
  return a.compression == b.compression && a.tension == b.tension && a.shear == b.shear;
}

}

TEST_CASE("the destruction materials carry the cited strengths", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const StructuralMaterial masonry = talus::destruction::brickMasonry();
  const StructuralMaterial timber = talus::destruction::timber();
  const StructuralMaterial glass = talus::destruction::glass();
  const StructuralMaterial steel = talus::destruction::steel();

  CHECK(concrete.density == 2450.0);
  CHECK(sameModes(concrete.elastic, ModeLimits{33.0e6, 2.6e6, 0.45 * 2.6e6}));
  CHECK(sameModes(concrete.fatal, concrete.elastic));
  CHECK(concrete.shearRule == ShearRule::Coulomb);
  CHECK(concrete.friction == 0.7);
  CHECK(concrete.anisotropy == Anisotropy::None);
  CHECK(concrete.variation == 0.30);
  CHECK(concrete.chunkSize == 0.5);

  CHECK(masonry.density == 1850.0);
  CHECK(sameModes(masonry.elastic, ModeLimits{8.0e6, 0.25e6, 0.35e6}));
  CHECK(sameModes(masonry.fatal, masonry.elastic));
  CHECK(masonry.shearRule == ShearRule::Coulomb);
  CHECK(masonry.friction == 0.7);
  CHECK(masonry.anisotropy == Anisotropy::Courses);
  CHECK(masonry.headJoints == 2.0);
  CHECK(masonry.variation == 0.30);
  CHECK(masonry.chunkSize == 0.5);

  CHECK(timber.density == 420.0);
  CHECK(sameModes(timber.elastic, ModeLimits{25.0e6, 22.0e6, 4.0e6}));
  CHECK(sameModes(timber.fatal, timber.elastic));
  CHECK(sameModes(timber.acrossElastic, ModeLimits{2.5e6, 0.4e6, 0.8e6}));
  CHECK(sameModes(timber.acrossFatal, timber.acrossElastic));
  CHECK(timber.shearRule == ShearRule::Fixed);
  CHECK(timber.friction == 0.4);
  CHECK(timber.anisotropy == Anisotropy::Grain);
  CHECK(timber.variation == 0.25);
  CHECK(timber.chunkSize == 0.5);

  CHECK(glass.density == 2500.0);
  CHECK(sameModes(glass.elastic, ModeLimits{500.0e6, 45.0e6, 45.0e6}));
  CHECK(sameModes(glass.fatal, glass.elastic));
  CHECK(glass.shearRule == ShearRule::Fixed);
  CHECK(glass.friction == 0.5);
  CHECK(glass.variation == 0.30);
  CHECK(glass.chunkSize == 0.25);

  CHECK(steel.density == 7850.0);
  CHECK(sameModes(steel.elastic, ModeLimits{380.0e6, 380.0e6, 380.0e6 / std::sqrt(3.0)}));
  CHECK(sameModes(steel.fatal, ModeLimits{560.0e6, 560.0e6, 560.0e6 / std::sqrt(3.0)}));
  CHECK(steel.shearRule == ShearRule::Fixed);
  CHECK(steel.friction == 0.2);
  CHECK(steel.variation == 0.07);
  CHECK(steel.chunkSize == 0.5);
}

TEST_CASE("reinforced concrete's tension is its steel's at the reinforcement ratio", "[destruction]") {
  const StructuralMaterial plain = talus::destruction::concrete();
  const StructuralMaterial reinforced = talus::destruction::reinforcedConcrete();
  const StructuralMaterial doubled = talus::destruction::reinforcedConcrete(0.02);

  CHECK(reinforced.density == 2550.0);
  CHECK(reinforced.elastic.tension == 0.01 * 380.0e6);
  CHECK(reinforced.fatal.tension == 0.01 * 560.0e6);
  CHECK(doubled.elastic.tension == 0.02 * 380.0e6);
  CHECK(doubled.fatal.tension == 0.02 * 560.0e6);
  CHECK(reinforced.elastic.compression == plain.elastic.compression);
  CHECK(reinforced.elastic.shear == plain.elastic.shear);
  CHECK(reinforced.shearRule == ShearRule::Coulomb);
  CHECK(reinforced.friction == plain.friction);
  CHECK(reinforced.variation == plain.variation);
}

TEST_CASE("a structural material it cannot use is refused", "[destruction]") {
  for (const StructuralMaterial& material : {
         talus::destruction::concrete(),
         talus::destruction::reinforcedConcrete(),
         talus::destruction::brickMasonry(),
         talus::destruction::timber(),
         talus::destruction::glass(),
         talus::destruction::steel(),
       }) {
    CHECK_NOTHROW(talus::destruction::validate(material));
  }

  StructuralMaterial inverted = talus::destruction::steel();

  inverted.fatal.tension = 0.5 * inverted.elastic.tension;

  StructuralMaterial weightless = talus::destruction::concrete();

  weightless.density = 0.0;

  StructuralMaterial scattered = talus::destruction::concrete();

  scattered.variation = 0.6;

  StructuralMaterial grainless = talus::destruction::timber();

  grainless.acrossElastic.tension = 0.0;

  CHECK_THROWS_AS(talus::destruction::validate(inverted), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::validate(weightless), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::validate(scattered), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::validate(grainless), std::invalid_argument);
}

namespace {

using talus::destruction::Bond;
using talus::destruction::Building;
using talus::destruction::Chunk;
using talus::destruction::ChunkedBuilding;
using talus::destruction::ChunkerSettings;
using talus::destruction::Member;
using talus::destruction::Opening;
using talus::destruction::Role;

ChunkerSettings plain() {
  ChunkerSettings settings;

  settings.variety = false;

  return settings;
}

bool near(double value, double expected, double tolerance) {
  return std::abs(value - expected) <= tolerance;
}

bool sameLimits(const ModeLimits& limits, const ModeLimits& strengths, double area) {
  const double tolerance = 1.0e-12;

  return near(limits.compression, strengths.compression * area, tolerance * strengths.compression * area)
         && near(limits.tension, strengths.tension * area, tolerance * strengths.tension * area)
         && near(limits.shear, strengths.shear * area, tolerance * strengths.shear * area);
}

Building wall() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(2.0, 1.5, 0.15), 0u, Role::Wall});

  return building;
}

uint32_t axisOf(const glm::dvec3& normal) {
  return std::abs(normal.x) > 0.5 ? 0u : (std::abs(normal.y) > 0.5 ? 1u : 2u);
}

bool hasChunkAt(const ChunkedBuilding& chunked, const glm::dvec3& centre) {
  return std::any_of(chunked.chunks.begin(), chunked.chunks.end(), [&](const Chunk& chunk) {
    return glm::length(chunk.centre - centre) <= 1.0e-12;
  });
}

}

TEST_CASE("a 4 by 3 m wall cuts into 8 by 6 by 1 chunks with 82 bonds and an anchored bottom row", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const std::vector<StructuralMaterial> materials{concrete};
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(wall(), materials, plain());

  REQUIRE(chunked.chunks.size() == 48u);

  uint32_t anchored = 0u;

  for (const Chunk& chunk : chunked.chunks) {
    CHECK(glm::length(chunk.halfExtents - glm::dvec3(0.25, 0.25, 0.15)) <= 1.0e-12);
    CHECK(near(chunk.volume, 0.5 * 0.5 * 0.3, 1.0e-12));
    CHECK(near(chunk.mass, concrete.density * chunk.volume, 1.0e-9));
    CHECK(chunk.anchored == (chunk.centre.y < 0.5));
    anchored += chunk.anchored ? 1u : 0u;
  }

  CHECK(anchored == 8u);
  REQUIRE(chunked.bonds.size() == 82u);

  uint32_t across = 0u;
  uint32_t up = 0u;

  for (const Bond& bond : chunked.bonds) {
    const Chunk& first = chunked.chunks[bond.first];
    const Chunk& second = chunked.chunks[bond.second];

    CHECK(bond.first < bond.second);
    CHECK(near(bond.area, 0.15, 1.0e-12));
    CHECK(glm::length(bond.centroid - 0.5 * (first.centre + second.centre)) <= 1.0e-12);
    CHECK(glm::length(bond.normal - glm::normalize(second.centre - first.centre)) <= 1.0e-12);
    CHECK(sameLimits(bond.elastic, concrete.elastic, bond.area));
    CHECK(sameLimits(bond.fatal, concrete.fatal, bond.area));
    CHECK(bond.shearRule == ShearRule::Coulomb);
    CHECK(bond.friction == concrete.friction);
    across += axisOf(bond.normal) == 0u ? 1u : 0u;
    up += axisOf(bond.normal) == 1u ? 1u : 0u;
  }

  CHECK(across == 42u);
  CHECK(up == 40u);
}

TEST_CASE("an opening removes exactly the cells whose centres lie inside it", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  Building building = wall();

  building.openings.push_back(Opening{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.5)});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  CHECK(chunked.chunks.size() == 44u);
  CHECK(chunked.bonds.size() == 70u);

  for (uint32_t j = 0u; j != 6u; ++j) {
    for (uint32_t i = 0u; i != 8u; ++i) {
      const glm::dvec3 centre(-1.75 + 0.5 * i, 0.25 + 0.5 * j, 0.0);
      const bool cut = std::abs(centre.x) < 0.5 && std::abs(centre.y - 1.5) < 0.5;

      CHECK(hasChunkAt(chunked, centre) == !cut);
    }
  }
}

TEST_CASE("the chunker refuses a building it cannot cut", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  Building missing = wall();

  missing.members.front().material = 1u;

  Building flat = wall();

  flat.members.front().halfExtents.z = 0.0;

  Building inverted = wall();

  inverted.openings.push_back(Opening{glm::dvec3(0.0), glm::dvec3(-1.0)});

  StructuralMaterial weak = talus::destruction::concrete();

  weak.fatal.compression = 0.5 * weak.elastic.compression;

  const std::vector<StructuralMaterial> broken{weak};

  CHECK_THROWS_AS(talus::destruction::chunkBuilding(missing, materials, plain()), std::out_of_range);
  CHECK_THROWS_AS(talus::destruction::chunkBuilding(flat, materials, plain()), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::chunkBuilding(inverted, materials, plain()), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::chunkBuilding(wall(), broken, plain()), std::invalid_argument);
}

TEST_CASE("bond variety scales each bond's limits by one factor from its chunk ids", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const std::vector<StructuralMaterial> materials{concrete};
  const ChunkedBuilding even = talus::destruction::chunkBuilding(wall(), materials, plain());
  const ChunkedBuilding varied = talus::destruction::chunkBuilding(wall(), materials);
  const ChunkedBuilding again = talus::destruction::chunkBuilding(wall(), materials);
  const double spread = std::sqrt(3.0) * concrete.variation;
  double lowest = 2.0;
  double highest = 0.0;

  REQUIRE(varied.bonds.size() == even.bonds.size());
  CHECK(talus::destruction::varietyFactor(3u, 4u, 0.0) == 1.0);

  for (size_t index = 0u; index != varied.bonds.size(); ++index) {
    const Bond& bond = varied.bonds[index];
    const Bond& base = even.bonds[index];
    const double factor = talus::destruction::varietyFactor(bond.first, bond.second, concrete.variation);
    ModeLimits scaled = base.elastic;

    scaled.compression *= factor;
    scaled.tension *= factor;
    scaled.shear *= factor;
    CHECK(sameLimits(bond.elastic, scaled, 1.0));
    CHECK(near(bond.fatal.tension, factor * base.fatal.tension, 1.0e-12 * base.fatal.tension));
    CHECK(bond.friction == base.friction);
    CHECK(factor >= 1.0 - spread);
    CHECK(factor <= 1.0 + spread);
    CHECK(sameModes(bond.elastic, again.bonds[index].elastic));
    lowest = std::min(lowest, factor);
    highest = std::max(highest, factor);
  }

  CHECK(lowest < highest);
}

namespace {

Building house() {
  Building building;

  building.members = {
    Member{glm::dvec3(0.0, 1.5, -2.85), glm::dvec3(3.0, 1.5, 0.15), 0u, Role::Wall},
    Member{glm::dvec3(0.0, 1.5, 2.85), glm::dvec3(3.0, 1.5, 0.15), 0u, Role::Wall},
    Member{glm::dvec3(-2.85, 1.5, 0.0), glm::dvec3(0.15, 1.5, 2.7), 0u, Role::Wall},
    Member{glm::dvec3(2.85, 1.5, 0.0), glm::dvec3(0.15, 1.5, 2.7), 0u, Role::Wall},
    Member{glm::dvec3(0.0, 3.1, 0.0), glm::dvec3(3.0, 0.1, 3.0), 1u, Role::Slab},
    Member{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.15, 1.5, 0.15), 3u, Role::Column},
    Member{glm::dvec3(0.0, 1.5, -2.85), glm::dvec3(0.5, 0.5, 0.005), 2u, Role::Pane},
  };
  building.openings = {Opening{glm::dvec3(0.0, 1.5, -2.85), glm::dvec3(0.5)}};

  return building;
}

std::vector<StructuralMaterial> houseMaterials() {
  return {
    talus::destruction::brickMasonry(),
    talus::destruction::reinforcedConcrete(),
    talus::destruction::glass(),
    talus::destruction::steel(),
  };
}

ChunkedBuilding normalised(const ChunkedBuilding& chunked) {
  std::vector<uint32_t> order(chunked.chunks.size());

  for (uint32_t index = 0u; index != order.size(); ++index) {
    order[index] = index;
  }

  std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
    const glm::dvec3& p = chunked.chunks[a].centre;
    const glm::dvec3& q = chunked.chunks[b].centre;

    return std::tie(p.x, p.y, p.z) < std::tie(q.x, q.y, q.z);
  });

  std::vector<uint32_t> rank(order.size());
  ChunkedBuilding result;

  for (uint32_t index = 0u; index != order.size(); ++index) {
    rank[order[index]] = index;
    result.chunks.push_back(chunked.chunks[order[index]]);
    result.chunks.back().member = 0u;
  }

  for (Bond bond : chunked.bonds) {
    bond.first = rank[bond.first];
    bond.second = rank[bond.second];

    if (bond.first > bond.second) {
      std::swap(bond.first, bond.second);
      bond.normal = -bond.normal;
    }

    result.bonds.push_back(bond);
  }

  std::sort(result.bonds.begin(), result.bonds.end(), [](const Bond& a, const Bond& b) {
    return std::tie(a.first, a.second) < std::tie(b.first, b.second);
  });

  return result;
}

}

TEST_CASE("a column standing on a slab is bonded to the slab chunks under it with the overlap area", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete(), talus::destruction::steel()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(1.0, 0.1, 1.0), 0u, Role::Slab});
  building.members.push_back(Member{glm::dvec3(0.0, 1.2, 0.0), glm::dvec3(0.15, 1.0, 0.15), 1u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 20u);

  uint32_t between = 0u;

  for (const Bond& bond : chunked.bonds) {
    if (chunked.chunks[bond.first].member == chunked.chunks[bond.second].member) {
      continue;
    }

    between += 1u;
    CHECK(chunked.chunks[bond.first].member == 0u);
    CHECK(bond.second == 16u);
    CHECK(near(bond.area, 0.15 * 0.15, 1.0e-12));
    CHECK(bond.normal == glm::dvec3(0.0, 1.0, 0.0));
    CHECK(near(bond.centroid.y, 0.2, 1.0e-12));
    CHECK(near(std::abs(bond.centroid.x), 0.075, 1.0e-12));
    CHECK(near(std::abs(bond.centroid.z), 0.075, 1.0e-12));
    CHECK(sameLimits(bond.elastic, materials[0].elastic, bond.area));
    CHECK(sameLimits(bond.fatal, materials[0].fatal, bond.area));
    CHECK(bond.shearRule == ShearRule::Coulomb);
    CHECK(bond.friction == materials[0].friction);
  }

  CHECK(between == 4u);
}

TEST_CASE("a pane set in a wall's opening is bonded to the wall around it", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete(), talus::destruction::glass()};
  Building building = wall();

  building.openings.push_back(Opening{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.5)});
  building.members.push_back(Member{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.5, 0.5, 0.005), 1u, Role::Pane});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 44u + 16u);

  uint32_t between = 0u;

  for (const Bond& bond : chunked.bonds) {
    if (chunked.chunks[bond.first].member == chunked.chunks[bond.second].member) {
      continue;
    }

    between += 1u;
    CHECK(near(bond.area, 0.25 * 0.01, 1.0e-12));
    CHECK(sameLimits(bond.elastic, materials[0].elastic, bond.area));
  }

  CHECK(between == 16u);
}

TEST_CASE("members whose chunks overlap are refused", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(1.0, 0.1, 1.0), 0u, Role::Slab});
  building.members.push_back(Member{glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.15, 1.0, 0.15), 0u, Role::Column});

  CHECK_THROWS_AS(talus::destruction::chunkBuilding(building, materials, plain()), std::invalid_argument);
}

TEST_CASE("timber bonds across the grain carry the across-grain limits", "[destruction]") {
  const StructuralMaterial timber = talus::destruction::timber();
  const std::vector<StructuralMaterial> materials{timber};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 2.0, 0.0), glm::dvec3(2.0, 0.5, 0.25), 0u, Role::Beam});
  building.members.push_back(Member{glm::dvec3(5.0, 2.0, 0.0), glm::dvec3(0.25, 2.0, 0.5), 0u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const glm::dvec3 grains[2] = {glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0, 1.0, 0.0)};
  uint32_t along[2] = {0u, 0u};
  uint32_t across[2] = {0u, 0u};

  for (const Bond& bond : chunked.bonds) {
    const uint32_t member = chunked.chunks[bond.first].member;
    const bool withGrain = std::abs(glm::dot(bond.normal, grains[member])) > 0.5;

    CHECK(sameLimits(bond.elastic, withGrain ? timber.elastic : timber.acrossElastic, bond.area));
    CHECK(sameLimits(bond.fatal, withGrain ? timber.fatal : timber.acrossFatal, bond.area));
    along[member] += withGrain ? 1u : 0u;
    across[member] += withGrain ? 0u : 1u;
  }

  CHECK(along[0] == 14u);
  CHECK(across[0] == 8u);
  CHECK(along[1] == 14u);
  CHECK(across[1] == 8u);
}

TEST_CASE("a timber member whose longest sides tie takes its grain along y, then x, then z, whatever their last bit", "[destruction]") {
  const StructuralMaterial timber = talus::destruction::timber();
  const std::vector<StructuralMaterial> materials{timber};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.5, std::nextafter(0.5, 0.0), 0.5), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(5.0, 0.25, 0.0), glm::dvec3(0.5, 0.25, std::nextafter(0.5, 1.0)), 0u, Role::Slab});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const glm::dvec3 grains[2] = {glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(1.0, 0.0, 0.0)};
  uint32_t along[2] = {0u, 0u};
  uint32_t across[2] = {0u, 0u};

  for (const Bond& bond : chunked.bonds) {
    const uint32_t member = chunked.chunks[bond.first].member;
    const bool withGrain = std::abs(glm::dot(bond.normal, grains[member])) > 0.5;

    CHECK(sameLimits(bond.elastic, withGrain ? timber.elastic : timber.acrossElastic, bond.area));
    CHECK(sameLimits(bond.fatal, withGrain ? timber.fatal : timber.acrossFatal, bond.area));
    along[member] += withGrain ? 1u : 0u;
    across[member] += withGrain ? 0u : 1u;
  }

  CHECK(along[0] == 4u);
  CHECK(across[0] == 8u);
  CHECK(along[1] == 2u);
  CHECK(across[1] == 2u);
}

TEST_CASE("masonry head joints carry twice the tension of bed joints", "[destruction]") {
  const StructuralMaterial masonry = talus::destruction::brickMasonry();
  const std::vector<StructuralMaterial> materials{masonry};
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(wall(), materials, plain());
  uint32_t bed = 0u;
  uint32_t head = 0u;

  for (const Bond& bond : chunked.bonds) {
    const bool bedJoint = axisOf(bond.normal) == 1u;
    ModeLimits expected = masonry.elastic;

    expected.tension *= bedJoint ? 1.0 : 2.0;
    CHECK(sameLimits(bond.elastic, expected, bond.area));
    bed += bedJoint ? 1u : 0u;
    head += bedJoint ? 0u : 1u;
  }

  CHECK(bed == 40u);
  CHECK(head == 42u);
}

TEST_CASE("the same building in another member order gives the same chunks and bonds", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  Building reversed = house();

  std::reverse(reversed.members.begin(), reversed.members.end());

  const ChunkedBuilding a = normalised(talus::destruction::chunkBuilding(house(), materials, plain()));
  const ChunkedBuilding b = normalised(talus::destruction::chunkBuilding(reversed, materials, plain()));

  REQUIRE(a.chunks.size() == 438u);
  REQUIRE(a.chunks.size() == b.chunks.size());
  REQUIRE(a.bonds.size() == b.bonds.size());

  for (size_t index = 0u; index != a.chunks.size(); ++index) {
    CHECK(a.chunks[index].centre == b.chunks[index].centre);
    CHECK(a.chunks[index].halfExtents == b.chunks[index].halfExtents);
    CHECK(a.chunks[index].material == b.chunks[index].material);
    CHECK(a.chunks[index].mass == b.chunks[index].mass);
    CHECK(a.chunks[index].anchored == b.chunks[index].anchored);
  }

  for (size_t index = 0u; index != a.bonds.size(); ++index) {
    const Bond& p = a.bonds[index];
    const Bond& q = b.bonds[index];

    CHECK(std::tie(p.first, p.second) == std::tie(q.first, q.second));
    CHECK(p.centroid == q.centroid);
    CHECK(p.normal == q.normal);
    CHECK(p.area == q.area);
    CHECK(sameModes(p.elastic, q.elastic));
    CHECK(sameModes(p.fatal, q.fatal));
    CHECK(p.shearRule == q.shearRule);
    CHECK(p.friction == q.friction);
  }
}

namespace {

using talus::destruction::StructureState;

constexpr double SIZE = 0.5;
constexpr double MASS = 125.0;

ChunkedBuilding chain(uint32_t free, const glm::dvec3& direction, bool clamped) {
  ChunkedBuilding chained;
  const uint32_t count = free + (clamped ? 2u : 1u);

  for (uint32_t index = 0u; index != count; ++index) {
    Chunk chunk;

    chunk.centre = (SIZE * index) * direction;
    chunk.halfExtents = glm::dvec3(0.5 * SIZE);
    chunk.volume = SIZE * SIZE * SIZE;
    chunk.mass = MASS;
    chunk.anchored = index == 0u || (clamped && index + 1u == count);
    chained.chunks.push_back(chunk);
  }

  for (uint32_t index = 0u; index + 1u != count; ++index) {
    Bond bond;

    bond.first = index;
    bond.second = index + 1u;
    bond.centroid = (SIZE * (index + 0.5)) * direction;
    bond.normal = direction;
    bond.area = SIZE * SIZE;
    bond.wu = SIZE;
    bond.wv = SIZE;
    chained.bonds.push_back(bond);
  }

  return chained;
}

ChunkedBuilding twoColumns() {
  ChunkedBuilding both = chain(3u, glm::dvec3(0.0, 1.0, 0.0), false);
  const ChunkedBuilding other = chain(3u, glm::dvec3(0.0, 1.0, 0.0), false);

  for (Chunk chunk : other.chunks) {
    chunk.centre.x += 5.0;
    both.chunks.push_back(chunk);
  }

  for (Bond bond : other.bonds) {
    bond.first += 4u;
    bond.second += 4u;
    bond.centroid.x += 5.0;
    both.bonds.push_back(bond);
  }

  return both;
}

}

TEST_CASE("a column of chunks splits into two components when one bond breaks, and the upper one is not anchored", "[destruction]") {
  const ChunkedBuilding column = chain(5u, glm::dvec3(0.0, 1.0, 0.0), false);
  StructureState state = talus::destruction::makeStructure(column);

  for (const uint32_t component : state.component) {
    CHECK(component == 0u);
  }

  CHECK(talus::destruction::isAnchored(state, 0u));

  talus::destruction::breakBond(state, column, 2u);

  CHECK(state.broken[2] == 1u);
  CHECK(state.health[2] == 0.0);
  CHECK(talus::destruction::updateComponents(state, column) == std::vector<uint32_t>{0u, 3u});

  for (uint32_t chunk = 0u; chunk != 6u; ++chunk) {
    CHECK(state.component[chunk] == (chunk < 3u ? 0u : 3u));
  }

  CHECK(talus::destruction::isAnchored(state, 0u));
  CHECK(!talus::destruction::isAnchored(state, 3u));
}

TEST_CASE("removing a chunk breaks all its bonds", "[destruction]") {
  const ChunkedBuilding column = chain(5u, glm::dvec3(0.0, 1.0, 0.0), false);
  StructureState state = talus::destruction::makeStructure(column);

  talus::destruction::removeChunk(state, column, 3u);

  CHECK(state.broken == std::vector<uint8_t>{0u, 0u, 1u, 1u, 0u});
  CHECK(talus::destruction::updateComponents(state, column) == std::vector<uint32_t>{0u, 4u});
  CHECK(state.alive[3] == 0u);
  CHECK(state.component[3] == talus::destruction::noComponent);
  CHECK(state.component[4] == 4u);
  CHECK(state.component[5] == 4u);
  CHECK(!talus::destruction::isAnchored(state, 4u));

  const uint64_t revision = state.revision;

  talus::destruction::removeChunk(state, column, 3u);

  CHECK(state.revision == revision);
  CHECK(talus::destruction::updateComponents(state, column).empty());

  talus::destruction::removeChunk(state, column, 0u);

  CHECK(talus::destruction::updateComponents(state, column) == std::vector<uint32_t>{1u});
  CHECK(!talus::destruction::isAnchored(state, 1u));
  CHECK_THROWS_AS(talus::destruction::removeChunk(state, column, 6u), std::out_of_range);
  CHECK_THROWS_AS(talus::destruction::breakBond(state, column, 5u), std::out_of_range);
}

TEST_CASE("only the components that held a broken bond are recomputed", "[destruction]") {
  const ChunkedBuilding both = twoColumns();
  StructureState first = talus::destruction::makeStructure(both);
  StructureState second = talus::destruction::makeStructure(both);

  CHECK(first.component == std::vector<uint32_t>{0u, 0u, 0u, 0u, 4u, 4u, 4u, 4u});

  talus::destruction::breakBond(first, both, 3u);

  CHECK(talus::destruction::updateComponents(first, both) == std::vector<uint32_t>{4u, 5u});
  CHECK(first.component == std::vector<uint32_t>{0u, 0u, 0u, 0u, 4u, 5u, 5u, 5u});

  talus::destruction::breakBond(first, both, 1u);
  talus::destruction::breakBond(second, both, 1u);
  talus::destruction::breakBond(second, both, 3u);

  static_cast<void>(talus::destruction::updateComponents(first, both));
  static_cast<void>(talus::destruction::updateComponents(second, both));

  CHECK(first.component == second.component);
  CHECK(first.anchoredComponent == second.anchoredComponent);
}

namespace {

using talus::destruction::ChunkLoad;
using talus::destruction::LoadSharing;
using talus::destruction::MassWeighting;
using talus::destruction::StressReport;
using talus::destruction::StressSettings;
using talus::destruction::StressState;

constexpr double G = 9.81;

StressSettings deferred() {
  StressSettings settings;

  settings.roundsPerCall = 1u;

  return settings;
}

}

TEST_CASE("a column of chunks on an anchor carries k m g under its k-th chunk from the top", "[destruction]") {
  for (const MassWeighting weighting : {MassWeighting::Real, MassWeighting::Equal}) {
    for (const LoadSharing sharing : {LoadSharing::Area, LoadSharing::Equal}) {
      const ChunkedBuilding column = chain(10u, glm::dvec3(0.0, 1.0, 0.0), false);
      const StructureState structure = talus::destruction::makeStructure(column);
      StressState stress = talus::destruction::makeStress(column, weighting, sharing);

      talus::destruction::solveStress(column, structure, stress, {}, StressSettings{});

      for (uint32_t index = 0u; index != 10u; ++index) {
        const double expected = (10.0 - index) * MASS * G;
        const glm::dvec3 force = stress.force[index];

        CHECK(near(force.y, expected, 1.0e-9 * expected));
        CHECK(near(force.x, 0.0, 1.0e-9 * expected));
        CHECK(near(force.z, 0.0, 1.0e-9 * expected));
        CHECK(glm::length(stress.moment[index]) <= 1.0e-9 * expected * SIZE);
      }
    }
  }
}

TEST_CASE("a cantilever of chunks carries k m g of shear and m g d k^2 / 2 of moment", "[destruction]") {
  for (const MassWeighting weighting : {MassWeighting::Real, MassWeighting::Equal}) {
    for (const LoadSharing sharing : {LoadSharing::Area, LoadSharing::Equal}) {
      const ChunkedBuilding beam = chain(8u, glm::dvec3(1.0, 0.0, 0.0), false);
      const StructureState structure = talus::destruction::makeStructure(beam);
      StressState stress = talus::destruction::makeStress(beam, weighting, sharing);

      talus::destruction::solveStress(beam, structure, stress, {}, StressSettings{});

      for (uint32_t index = 0u; index != 8u; ++index) {
        const double beyond = 8.0 - index;
        const double shear = beyond * MASS * G;
        const double moment = MASS * G * SIZE * beyond * beyond / 2.0;

        CHECK(near(stress.force[index].y, shear, 1.0e-9 * shear));
        CHECK(near(stress.force[index].x, 0.0, 1.0e-9 * shear));
        CHECK(near(stress.force[index].z, 0.0, 1.0e-9 * shear));
        CHECK(near(stress.moment[index].z, moment, 1.0e-9 * moment));
        CHECK(near(stress.moment[index].x, 0.0, 1.0e-9 * moment));
        CHECK(near(stress.moment[index].y, 0.0, 1.0e-9 * moment));
      }
    }
  }
}

TEST_CASE("an update stopped by its round limit is pending, keeps its loads and ignores new ones", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const std::vector<ChunkLoad> offCentre{ChunkLoad{1u, glm::dvec3(0.0, -200.0e3, 0.0), glm::dvec3(0.0, 0.0, -200.0e3 * 0.3)}};
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings = deferred();

  REQUIRE(chunked.bonds.size() == 1u);

  const StressReport first = talus::destruction::updateStress(chunked, structure, stress, offCentre, settings);

  CHECK(first.ran);
  CHECK(first.pending);
  CHECK(stress.pending);
  CHECK(first.rounds == 1u);
  CHECK(first.cracked == std::vector<uint32_t>{0u});
  CHECK(first.broken.empty());
  CHECK(structure.broken[0] == 0u);

  const StressReport second = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

  CHECK(second.ran);
  CHECK(!second.pending);
  CHECK(!stress.pending);
  CHECK(second.rounds == 1u);
  CHECK(second.cracked.empty());
  CHECK(second.broken == std::vector<uint32_t>{0u});
  CHECK(structure.broken[0] == 1u);
  REQUIRE(stress.loads.size() == 1u);
  CHECK(stress.loads[0].chunk == 1u);
  CHECK(stress.loads[0].force == offCentre[0].force);
  CHECK(stress.loads[0].torque == offCentre[0].torque);
}

TEST_CASE("a chunk load naming a chunk the building does not hold is refused", "[destruction]") {
  const ChunkedBuilding column = chain(2u, glm::dvec3(0.0, 1.0, 0.0), false);
  const StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const std::vector<ChunkLoad> stray{ChunkLoad{3u, glm::dvec3(1.0), glm::dvec3(0.0)}};

  CHECK_THROWS_AS(talus::destruction::solveStress(column, structure, stress, stray, StressSettings{}), std::out_of_range);
}

TEST_CASE("a chunk load whose force or torque is not finite is refused", "[destruction]") {
  const ChunkedBuilding column = chain(2u, glm::dvec3(0.0, 1.0, 0.0), false);
  const double infinite = std::numeric_limits<double>::infinity();
  const double unknown = std::numeric_limits<double>::quiet_NaN();

  for (const ChunkLoad& load : {
         ChunkLoad{1u, glm::dvec3(unknown, 0.0, 0.0), glm::dvec3(0.0)},
         ChunkLoad{1u, glm::dvec3(0.0, -infinite, 0.0), glm::dvec3(0.0)},
         ChunkLoad{2u, glm::dvec3(0.0), glm::dvec3(0.0, 0.0, unknown)},
         ChunkLoad{2u, glm::dvec3(0.0), glm::dvec3(infinite, 0.0, 0.0)},
       }) {
    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);
    const std::vector<ChunkLoad> loads{load};

    CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, loads, StressSettings{}), std::invalid_argument);
    CHECK_THROWS_AS(talus::destruction::solveStress(column, structure, stress, loads, StressSettings{}), std::invalid_argument);
    CHECK(!stress.solved);
    CHECK(stress.loads.empty());
  }
}

TEST_CASE("a stress state reports the memory its factor holds", "[destruction]") {
  for (const uint32_t free : {5u, 9u}) {
    const ChunkedBuilding column = chain(free, glm::dvec3(0.0, 1.0, 0.0), false);
    const StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);

    CHECK(talus::destruction::factorBytes(stress) == 0u);

    talus::destruction::solveStress(column, structure, stress, {}, StressSettings{});

    CHECK(talus::destruction::factorBytes(stress) == sizeof(double) * 36u * (2u * free - 1u));
  }
}

namespace {

using talus::destruction::BondStress;

}

TEST_CASE("a bond's face extents are the overlap of its two chunk boxes across its normal", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete(), talus::destruction::steel()};
  const ChunkedBuilding walled = talus::destruction::chunkBuilding(wall(), materials, plain());
  Building standing;

  standing.members.push_back(Member{glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(1.0, 0.1, 1.0), 0u, Role::Slab});
  standing.members.push_back(Member{glm::dvec3(0.0, 1.2, 0.0), glm::dvec3(0.15, 1.0, 0.15), 1u, Role::Column});

  const ChunkedBuilding column = talus::destruction::chunkBuilding(standing, materials, plain());

  REQUIRE(walled.bonds.size() == 82u);
  REQUIRE(column.chunks.size() == 20u);

  for (const Bond& bond : walled.bonds) {
    const bool across = axisOf(bond.normal) == 0u;

    CHECK(near(bond.wu, across ? 0.5 : 0.3, 1.0e-12));
    CHECK(near(bond.wv, across ? 0.3 : 0.5, 1.0e-12));
  }

  for (const Bond& bond : column.bonds) {
    const uint32_t firstMember = column.chunks[bond.first].member;
    const uint32_t secondMember = column.chunks[bond.second].member;
    const bool alongX = axisOf(bond.normal) == 0u;
    const double wu = firstMember != secondMember ? 0.15 : (firstMember == 0u ? (alongX ? 0.2 : 0.5) : 0.3);
    const double wv = firstMember != secondMember ? 0.15 : (firstMember == 0u ? (alongX ? 0.5 : 0.2) : 0.3);

    CHECK(near(bond.wu, wu, 1.0e-12));
    CHECK(near(bond.wv, wv, 1.0e-12));
    CHECK(near(bond.wu * bond.wv, bond.area, 1.0e-12));
  }
}

TEST_CASE("a bond's stresses include its bending and twist moments over its remaining area", "[destruction]") {
  Bond bond;

  bond.normal = glm::dvec3(1.0, 0.0, 0.0);
  bond.area = 0.25;
  bond.wu = 0.25;
  bond.wv = 1.0;

  Bond turned;

  turned.normal = glm::dvec3(0.0, -1.0, 0.0);
  turned.area = 0.25;
  turned.wu = 1.0;
  turned.wv = 0.25;

  const glm::dvec3 force(1000.0, 300.0, 400.0);
  const glm::dvec3 moment(50.0, 25.0, 100.0);
  const BondStress whole = talus::destruction::bondStress(bond, 1.0, force, moment);
  const BondStress half = talus::destruction::bondStress(bond, 0.5, force, moment);
  const BondStress pulled = talus::destruction::bondStress(bond, 1.0, -force, moment);
  const BondStress gone = talus::destruction::bondStress(bond, 0.0, force, moment);
  const BondStress other = talus::destruction::bondStress(
    turned,
    1.0,
    glm::dvec3(300.0, -1000.0, 400.0),
    glm::dvec3(25.0, -50.0, 100.0)
  );

  CHECK(near(whole.pressure, 4000.0, 1.0e-9));
  CHECK(near(whole.compression, 14200.0, 1.0e-9));
  CHECK(near(whole.tension, 6200.0, 1.0e-9));
  CHECK(near(whole.shear, 4760.0, 1.0e-9));
  CHECK(near(half.pressure, 8000.0, 1.0e-9));
  CHECK(near(half.compression, 28400.0, 1.0e-9));
  CHECK(near(half.tension, 12400.0, 1.0e-9));
  CHECK(near(half.shear, 9520.0, 1.0e-9));
  CHECK(near(pulled.pressure, -4000.0, 1.0e-9));
  CHECK(near(pulled.compression, 6200.0, 1.0e-9));
  CHECK(near(pulled.tension, 14200.0, 1.0e-9));
  CHECK(near(pulled.shear, 4760.0, 1.0e-9));
  CHECK(gone.compression == 0.0);
  CHECK(gone.tension == 0.0);
  CHECK(gone.shear == 0.0);
  CHECK(near(other.pressure, 4000.0, 1.0e-9));
  CHECK(near(other.compression, 14200.0, 1.0e-9));
  CHECK(near(other.tension, 6200.0, 1.0e-9));
  CHECK(near(other.shear, 4760.0, 1.0e-9));
}

TEST_CASE("a bond between its elastic and fatal limits is damaged in proportion and breaks at the fatal limit", "[destruction]") {
  Bond bond;

  bond.normal = glm::dvec3(0.0, 1.0, 0.0);
  bond.area = 0.25;
  bond.elastic = ModeLimits{10.0e6 * 0.25, 1.0e6 * 0.25, 3.0e6 * 0.25};
  bond.fatal = ModeLimits{20.0e6 * 0.25, 2.0e6 * 0.25, 4.0e6 * 0.25};

  const auto damage = [&](double compression, double tension, double shear) {
    BondStress stress;

    stress.compression = compression;
    stress.tension = tension;
    stress.shear = shear;

    return talus::destruction::damageOf(bond, stress);
  };

  CHECK(damage(9.0e6, 0.9e6, 2.9e6) == 0.0);
  CHECK(near(damage(15.0e6, 0.0, 0.0), 0.5, 1.0e-12));
  CHECK(near(damage(0.0, 1.5e6, 0.0), 0.5, 1.0e-12));
  CHECK(near(damage(0.0, 0.0, 3.5e6), 0.5, 1.0e-12));
  CHECK(near(damage(0.0, 1.25e6, 3.25e6), 0.5, 1.0e-12));
  CHECK(damage(0.0, 1.5e6, 3.5e6) == 1.0);
  CHECK(damage(0.0, 2.0e6, 0.0) == 1.0);
  CHECK(damage(25.0e6, 0.0, 0.0) == 1.0);

  bond.fatal = bond.elastic;

  CHECK(damage(0.0, 0.999e6, 0.0) == 0.0);
  CHECK(damage(0.0, 1.0e6, 0.0) == 1.0);
}

TEST_CASE("a Coulomb bond's shear limit rises with compression and loses its cohesion in tension", "[destruction]") {
  Bond bond;

  bond.normal = glm::dvec3(0.0, 1.0, 0.0);
  bond.area = 0.25;
  bond.elastic = ModeLimits{30.0e6 * 0.25, 2.0e6 * 0.25, 1.0e6 * 0.25};
  bond.fatal = bond.elastic;
  bond.shearRule = ShearRule::Coulomb;
  bond.friction = 0.7;

  const auto damage = [&](double pressure, double shear) {
    BondStress stress;

    stress.pressure = pressure;
    stress.compression = std::max(0.0, pressure);
    stress.tension = std::max(0.0, -pressure);
    stress.shear = shear;

    return talus::destruction::damageOf(bond, stress);
  };

  CHECK(damage(0.0, 0.9e6) == 0.0);
  CHECK(damage(-1.0, 0.9e6) == 0.0);
  CHECK(damage(0.0, 1.1e6) == 1.0);
  CHECK(damage(1.0e6, 1.6e6) == 0.0);
  CHECK(damage(1.0e6, 1.8e6) == 1.0);
  CHECK(damage(-1.0e6, 0.45e6) == 0.0);
  CHECK(damage(-1.0e6, 0.55e6) == 1.0);

  bond.shearRule = ShearRule::Fixed;

  CHECK(damage(1.0e6, 1.1e6) == 1.0);
  CHECK(damage(-1.0e6, 0.9e6) == 0.0);
}

TEST_CASE("a cantilever's root bond breaks first, at the tip load beam theory predicts for its face", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const std::vector<StructuralMaterial> materials{concrete};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.25, 0.0), glm::dvec3(0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(2.25, 1.25, 0.0), glm::dvec3(2.0, 0.1, 0.25), 0u, Role::Beam});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 9u);
  REQUIRE(chunked.bonds.size() == 8u);
  REQUIRE(chunked.bonds[0].first == 0u);
  REQUIRE(chunked.bonds[0].second == 1u);

  const double width = 0.5;
  const double depth = 0.2;
  const double arm = chunked.chunks[8].centre.x - chunked.bonds[0].centroid.x;
  const double predicted = concrete.elastic.tension * (width * depth * depth / 6.0) / arm;
  StressSettings settings;

  settings.gravity = glm::dvec3(0.0);

  REQUIRE(near(arm, 3.75, 1.0e-12));

  uint32_t breaking = 0u;
  std::vector<uint32_t> first;

  for (uint32_t step = 1u; step != 201u && breaking == 0u; ++step) {
    const double tip = (static_cast<double>(step) - 0.5) / 100.0 * predicted;
    const std::vector<ChunkLoad> load{ChunkLoad{8u, glm::dvec3(0.0, -tip, 0.0), glm::dvec3(0.0)}};
    const StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);

    talus::destruction::solveStress(chunked, structure, stress, load, settings);

    for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
      const Bond& bond = chunked.bonds[index];
      const BondStress stressed = talus::destruction::bondStress(bond, 1.0, stress.force[index], stress.moment[index]);

      if (talus::destruction::damageOf(bond, stressed) >= 1.0) {
        first.push_back(index);
      }
    }

    breaking = first.empty() ? 0u : step;
  }

  CHECK(breaking == 101u);
  CHECK(first == std::vector<uint32_t>{0u});
}

TEST_CASE("a cantilever's root bond breaks first at the predicted tip load in the game's update flow too", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const std::vector<StructuralMaterial> materials{concrete};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.25, 0.0), glm::dvec3(0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(2.25, 1.25, 0.0), glm::dvec3(2.0, 0.1, 0.25), 0u, Role::Beam});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.bonds.size() == 8u);

  const double arm = chunked.chunks[8].centre.x - chunked.bonds[0].centroid.x;
  const double predicted = concrete.elastic.tension * (0.5 * 0.2 * 0.2 / 6.0) / arm;
  StressSettings settings;

  settings.gravity = glm::dvec3(0.0);

  uint32_t breaking = 0u;
  std::vector<uint32_t> first;

  for (uint32_t step = 1u; step != 201u && breaking == 0u; ++step) {
    const double tip = (static_cast<double>(step) - 0.5) / 100.0 * predicted;
    const std::vector<ChunkLoad> load{ChunkLoad{8u, glm::dvec3(0.0, -tip, 0.0), glm::dvec3(0.0)}};
    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    bool settled = false;

    for (uint32_t update = 0u; update != 2000u && !settled; ++update) {
      const talus::destruction::StressReport report = talus::destruction::updateStress(chunked, structure, stress, load, settings);

      settled = !report.ran;
      first.insert(first.end(), report.broken.begin(), report.broken.end());
      static_cast<void>(talus::destruction::updateComponents(structure, chunked));
    }

    REQUIRE(settled);

    breaking = first.empty() ? 0u : step;
  }

  CHECK(breaking == 101u);
  CHECK(first == std::vector<uint32_t>{0u});
}

TEST_CASE("a stress state refuses a bond whose face has no extent", "[destruction]") {
  ChunkedBuilding column = chain(2u, glm::dvec3(0.0, 1.0, 0.0), false);

  column.bonds[1].wv = 0.0;

  CHECK_THROWS_AS(talus::destruction::makeStress(column), std::invalid_argument);
}

TEST_CASE("a stress state refuses a bond that does not join two chunks of its building in order", "[destruction]") {
  ChunkedBuilding reversed = chain(2u, glm::dvec3(0.0, 1.0, 0.0), false);
  ChunkedBuilding stray = chain(2u, glm::dvec3(0.0, 1.0, 0.0), false);

  std::swap(reversed.bonds[1].first, reversed.bonds[1].second);
  stray.bonds[1].second = 3u;

  CHECK_THROWS_AS(talus::destruction::makeStress(reversed), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::makeStress(stray), std::invalid_argument);
}

namespace {

constexpr uint32_t ANY_MEMBER = 0xFFFFFFFFu;
constexpr uint32_t PANE = 6u;

uint32_t pastLimits(const ChunkedBuilding& chunked, const StressState& stress, uint32_t member) {
  uint32_t past = 0u;

  for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
    const Bond& bond = chunked.bonds[index];
    const bool touches = member == ANY_MEMBER || chunked.chunks[bond.first].member == member
                         || chunked.chunks[bond.second].member == member;
    const double damage = talus::destruction::assessBond(
      bond,
      talus::destruction::bondStress(bond, 1.0, stress.force[index], stress.moment[index]),
      false
    ).damage;

    past += touches && damage > 0.0 ? 1u : 0u;
  }

  return past;
}

}

TEST_CASE("a beam of chunks clamped at both ends carries its largest moments at the supports", "[destruction]") {
  for (const MassWeighting weighting : {MassWeighting::Real, MassWeighting::Equal}) {
    for (const uint32_t free : {16u, 32u}) {
      const ChunkedBuilding beam = chain(free, glm::dvec3(1.0, 0.0, 0.0), true);
      const StructureState structure = talus::destruction::makeStructure(beam);
      StressState stress = talus::destruction::makeStress(beam, weighting);

      talus::destruction::solveStress(beam, structure, stress, {}, StressSettings{});

      const double n = free;
      const double support = MASS * G * SIZE * n * (n - 1.0) / 12.0;
      double largest = 0.0;

      for (uint32_t index = 0u; index <= free; ++index) {
        const double k = index;
        const double shear = (n / 2.0 - k) * MASS * G;
        const double moment = MASS * G * SIZE * (n * (n - 1.0) / 12.0 - k * (n - k) / 2.0);

        CHECK(near(stress.force[index].y, shear, 1.0e-9 * n * MASS * G));
        CHECK(near(stress.moment[index].z, moment, 1.0e-9 * support));
        CHECK(glm::length(glm::dvec3(stress.force[index].x, 0.0, stress.force[index].z)) <= 1.0e-9 * n * MASS * G);
        CHECK(glm::length(glm::dvec3(stress.moment[index].x, stress.moment[index].y, 0.0)) <= 1.0e-9 * support);
        largest = std::max(largest, std::abs(stress.moment[index].z));
      }

      CHECK(std::abs(stress.moment[0].z) >= largest * (1.0 - 1.0e-12));
      CHECK(std::abs(stress.moment[free].z) >= largest * (1.0 - 1.0e-12));

      if (free == 32u) {
        const double load = MASS * G / SIZE;
        const double span = n * SIZE;

        CHECK(near(std::abs(stress.moment[0].z), load * span * span / 12.0, 0.1 * load * span * span / 12.0));
        CHECK(near(std::abs(stress.moment[free / 2u].z), load * span * span / 24.0, 0.1 * load * span * span / 24.0));
      }
    }
  }
}

TEST_CASE("a falling piece of unequal chunks feels no stress from gravity", "[destruction]") {
  ChunkedBuilding piece = chain(1u, glm::dvec3(1.0, 0.0, 0.0), false);

  piece.chunks[0].anchored = false;
  piece.chunks[1].mass = 4.0 * MASS;

  const StructureState structure = talus::destruction::makeStructure(piece);
  StressState byDefault = talus::destruction::makeStress(piece);
  StressState equal = talus::destruction::makeStress(piece, MassWeighting::Equal);

  talus::destruction::solveStress(piece, structure, byDefault, {}, StressSettings{});
  talus::destruction::solveStress(piece, structure, equal, {}, StressSettings{});

  CHECK(glm::length(byDefault.force[0]) == 0.0);
  CHECK(glm::length(byDefault.moment[0]) == 0.0);
  CHECK(glm::length(equal.force[0]) > 0.0);
}

namespace {

struct BondLoads {
  std::vector<glm::dvec3> force;
  std::vector<glm::dvec3> moment;
};

int faceAxis(const glm::dvec3& normal) {
  int axis = 0;

  for (int candidate = 1; candidate != 3; ++candidate) {
    if (std::abs(normal[candidate]) > std::abs(normal[axis])) {
      axis = candidate;
    }
  }

  return axis;
}

void addCross(Eigen::MatrixXd& matrix, int row, int column, const glm::dvec3& arm, double sign, double scale) {
  matrix(row + 0, column + 1) -= sign * scale * arm.z;
  matrix(row + 0, column + 2) += sign * scale * arm.y;
  matrix(row + 1, column + 0) += sign * scale * arm.z;
  matrix(row + 1, column + 2) -= sign * scale * arm.x;
  matrix(row + 2, column + 0) -= sign * scale * arm.y;
  matrix(row + 2, column + 1) += sign * scale * arm.x;
}

BondLoads leastNormBondLoads(
  const ChunkedBuilding& chunked,
  const StructureState& structure,
  std::span<const ChunkLoad> loads,
  const glm::dvec3& gravity
) {
  const size_t chunks = chunked.chunks.size();
  const size_t bonds = chunked.bonds.size();
  double reach = 0.0;
  double area = 0.0;

  for (const Bond& bond : chunked.bonds) {
    reach += glm::length(bond.centroid - chunked.chunks[bond.first].centre);
    reach += glm::length(bond.centroid - chunked.chunks[bond.second].centre);
    area += bond.area;
  }

  const double lever = reach / (2.0 * static_cast<double>(bonds));
  const double meanArea = area / static_cast<double>(bonds);
  std::vector<int> block(chunks, -1);
  std::vector<int> slot(bonds, -1);
  int rows = 0;
  int columns = 0;

  for (size_t index = 0u; index != chunks; ++index) {
    if (!chunked.chunks[index].anchored) {
      block[index] = 6 * rows++;
    }
  }

  for (size_t index = 0u; index != bonds; ++index) {
    if (structure.broken[index] == 0u) {
      slot[index] = 6 * columns++;
    }
  }

  std::vector<glm::dvec3> linearRoot(chunks, glm::dvec3(0.0));
  std::vector<glm::dvec3> angularRoot(chunks, glm::dvec3(0.0));
  std::vector<glm::dvec3> linear(chunks, glm::dvec3(0.0));
  std::vector<glm::dvec3> angular(chunks, glm::dvec3(0.0));

  for (size_t index = 0u; index != chunks; ++index) {
    const Chunk& chunk = chunked.chunks[index];
    const glm::dvec3 h = chunk.halfExtents;
    const glm::dvec3 inertia = (chunk.mass / 3.0) * glm::dvec3(h.y * h.y + h.z * h.z, h.x * h.x + h.z * h.z, h.x * h.x + h.y * h.y);

    linearRoot[index] = glm::dvec3(std::sqrt(chunk.mass));
    angularRoot[index] = glm::sqrt(inertia);
    linear[index] = gravity;
  }

  for (const ChunkLoad& load : loads) {
    const Chunk& chunk = chunked.chunks[load.chunk];
    const glm::dvec3 h = chunk.halfExtents;
    const glm::dvec3 inertia = (chunk.mass / 3.0) * glm::dvec3(h.y * h.y + h.z * h.z, h.x * h.x + h.z * h.z, h.x * h.x + h.y * h.y);

    linear[load.chunk] += load.force / chunk.mass;
    angular[load.chunk] += load.torque / inertia;
  }

  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(6 * rows, 6 * columns);
  Eigen::VectorXd target = Eigen::VectorXd::Zero(6 * rows);
  std::vector<glm::dvec3> forceScale(bonds, glm::dvec3(0.0));
  std::vector<double> momentScale(bonds, 0.0);

  for (size_t index = 0u; index != bonds; ++index) {
    const Bond& bond = chunked.bonds[index];

    const double health = structure.health[index];

    forceScale[index] = glm::dvec3(std::sqrt(bond.area / meanArea) * health);
    momentScale[index] = bond.area / meanArea * health * lever;

    if (structure.cracked[index] != 0u) {
      const int axis = faceAxis(bond.normal);

      forceScale[index][(axis + 1) % 3] *= 1.0e-2;
      forceScale[index][(axis + 2) % 3] *= 1.0e-2;
      momentScale[index] *= 1.0e-2;
    }

    if (slot[index] < 0) {
      continue;
    }

    for (const uint32_t end : {bond.first, bond.second}) {
      if (block[end] < 0) {
        continue;
      }

      const double sign = end == bond.second ? 1.0 : -1.0;
      const glm::dvec3 arm = bond.centroid - chunked.chunks[end].centre;
      const int r = block[end];
      const int c = slot[index];

      for (int axis = 0; axis != 3; ++axis) {
        matrix(r + axis, c + axis) += sign * forceScale[index][axis] / linearRoot[end][axis];
        matrix(r + 3 + axis, c + 3 + axis) += sign * momentScale[index] / angularRoot[end][axis];
      }

      Eigen::MatrixXd cross = Eigen::MatrixXd::Zero(3, 3);

      addCross(cross, 0, 0, arm, sign, 1.0);

      for (int a = 0; a != 3; ++a) {
        for (int b = 0; b != 3; ++b) {
          matrix(r + 3 + a, c + b) += cross(a, b) * forceScale[index][b] / angularRoot[end][a];
        }
      }
    }
  }

  for (size_t index = 0u; index != chunks; ++index) {
    if (block[index] < 0) {
      continue;
    }

    for (int axis = 0; axis != 3; ++axis) {
      target(block[index] + axis) = -linearRoot[index][axis] * linear[index][axis];
      target(block[index] + 3 + axis) = -angularRoot[index][axis] * angular[index][axis];
    }
  }

  const Eigen::VectorXd solution = matrix.completeOrthogonalDecomposition().solve(target);
  BondLoads result;

  result.force.assign(bonds, glm::dvec3(0.0));
  result.moment.assign(bonds, glm::dvec3(0.0));

  for (size_t index = 0u; index != bonds; ++index) {
    if (slot[index] < 0) {
      continue;
    }

    for (int axis = 0; axis != 3; ++axis) {
      result.force[index][axis] = forceScale[index][axis] * solution(slot[index] + axis);
      result.moment[index][axis] = momentScale[index] * solution(slot[index] + 3 + axis);
    }
  }

  return result;
}

Building table(const glm::dvec3& at) {
  Building building;

  for (const double x : {-0.75, 0.75}) {
    for (const double z : {-0.75, 0.75}) {
      building.members.push_back(Member{at + glm::dvec3(x, 0.75, z), glm::dvec3(0.25, 0.75, 0.25), 0u, Role::Column});
    }
  }

  building.members.push_back(Member{at + glm::dvec3(0.0, 1.75, 0.0), glm::dvec3(1.0, 0.25, 1.0), 0u, Role::Slab});

  return building;
}

Building unevenTable() {
  Building building;

  building.members.push_back(Member{glm::dvec3(-0.75, 0.75, -0.75), glm::dvec3(0.15, 0.75, 0.15), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(0.75, 0.75, -0.75), glm::dvec3(0.25, 0.75, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(-0.5, 0.75, 0.5), glm::dvec3(0.2, 0.75, 0.2), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(0.75, 0.75, 0.75), glm::dvec3(0.1, 0.75, 0.1), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(0.0, 1.65, 0.0), glm::dvec3(1.0, 0.15, 1.0), 0u, Role::Slab});

  return building;
}

uint32_t bondAt(const ChunkedBuilding& chunked, const glm::dvec3& centroid) {
  for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
    if (glm::length(chunked.bonds[index].centroid - centroid) <= 1.0e-9) {
      return index;
    }
  }

  return 0xFFFFFFFFu;
}

uint32_t chunkAt(const ChunkedBuilding& chunked, const glm::dvec3& centre) {
  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    if (glm::length(chunked.chunks[index].centre - centre) <= 1.0e-9) {
      return index;
    }
  }

  return 0xFFFFFFFFu;
}

struct Agreement {
  double force = 0.0;
  double largestForce = 0.0;
  double moment = 0.0;
  double largestMoment = 0.0;
};

Agreement agreement(const ChunkedBuilding& chunked, const StructureState& structure, const StressState& stress, const BondLoads& reference) {
  Agreement result;

  for (size_t index = 0u; index != chunked.bonds.size(); ++index) {
    if (structure.broken[index] != 0u) {
      continue;
    }

    result.force = std::max(result.force, glm::length(stress.force[index] - reference.force[index]));
    result.moment = std::max(result.moment, glm::length(stress.moment[index] - reference.moment[index]));
    result.largestForce = std::max(result.largestForce, glm::length(reference.force[index]));
    result.largestMoment = std::max(result.largestMoment, glm::length(reference.moment[index]));
  }

  return result;
}

}

TEST_CASE("the exact solve gives the weighted smallest bond loads a dense least-squares solve finds", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  const ChunkedBuilding standing = talus::destruction::chunkBuilding(table(glm::dvec3(0.0)), materials, plain());
  Building both = table(glm::dvec3(0.0));

  for (const Member& member : table(glm::dvec3(6.0, 3.0, 0.0)).members) {
    both.members.push_back(member);
  }

  const ChunkedBuilding pair = talus::destruction::chunkBuilding(both, materials, plain());

  REQUIRE(standing.chunks.size() == 28u);
  REQUIRE(standing.bonds.size() == 36u);
  REQUIRE(pair.chunks.size() == 56u);

  const uint32_t corner = chunkAt(standing, glm::dvec3(0.75, 1.75, 0.75));
  const uint32_t edge = chunkAt(standing, glm::dvec3(-0.25, 1.75, 0.75));
  const uint32_t lifted = chunkAt(pair, glm::dvec3(6.75, 4.75, -0.75));
  const uint32_t leg = chunkAt(pair, glm::dvec3(5.25, 3.25, 0.75));

  REQUIRE(corner != 0xFFFFFFFFu);
  REQUIRE(edge != 0xFFFFFFFFu);
  REQUIRE(lifted != 0xFFFFFFFFu);
  REQUIRE(leg != 0xFFFFFFFFu);

  const std::vector<ChunkLoad> pushed{
    ChunkLoad{corner, glm::dvec3(4.0e3, -9.0e3, 2.5e3), glm::dvec3(0.0)},
    ChunkLoad{edge, glm::dvec3(0.0), glm::dvec3(1.5e3, -2.0e3, 0.8e3)},
  };
  const std::vector<ChunkLoad> thrown{
    ChunkLoad{corner, glm::dvec3(4.0e3, -9.0e3, 2.5e3), glm::dvec3(0.0)},
    ChunkLoad{lifted, glm::dvec3(-3.0e3, 6.0e3, 1.0e3), glm::dvec3(0.5e3, 0.0, -1.2e3)},
    ChunkLoad{leg, glm::dvec3(2.0e3, 0.0, -4.0e3), glm::dvec3(0.0)},
  };

  StructureState intact = talus::destruction::makeStructure(standing);
  StructureState cracked = talus::destruction::makeStructure(standing);
  const StructureState apart = talus::destruction::makeStructure(pair);

  for (uint32_t index = 0u; index != standing.bonds.size(); ++index) {
    const Bond& bond = standing.bonds[index];

    if (standing.chunks[bond.first].member != standing.chunks[bond.second].member || index % 5u == 0u) {
      talus::destruction::crackBond(cracked, index);
    }
  }

  REQUIRE(std::count(cracked.cracked.begin(), cracked.cracked.end(), uint8_t{1}) >= 10);
  REQUIRE(std::count_if(pair.chunks.begin(), pair.chunks.end(), [](const Chunk& chunk) { return chunk.anchored; }) == 4);

  const auto breakAt = [](const ChunkedBuilding& chunked, StructureState& structure, std::initializer_list<glm::dvec3> centroids) {
    for (const glm::dvec3& centroid : centroids) {
      const uint32_t bond = bondAt(chunked, centroid);

      REQUIRE(bond != 0xFFFFFFFFu);

      talus::destruction::breakBond(structure, chunked, bond);
    }

    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  };

  StructureState broken = talus::destruction::makeStructure(standing);
  StructureState removed = talus::destruction::makeStructure(standing);
  StructureState freed = talus::destruction::makeStructure(standing);
  const uint32_t post = chunkAt(standing, glm::dvec3(-0.75, 1.25, -0.75));

  REQUIRE(post != 0xFFFFFFFFu);

  breakAt(standing, broken, {glm::dvec3(-0.75, 1.5, -0.75), glm::dvec3(0.0, 1.75, 0.25), glm::dvec3(0.5, 1.75, -0.75)});
  talus::destruction::removeChunk(removed, standing, edge);
  breakAt(
    standing,
    freed,
    {glm::dvec3(0.0, 1.75, 0.25),
     glm::dvec3(0.0, 1.75, 0.75),
     glm::dvec3(0.25, 1.75, 0.0),
     glm::dvec3(0.75, 1.75, 0.0),
     glm::dvec3(0.75, 1.5, 0.75),
     glm::dvec3(-0.75, 0.5, -0.75),
     glm::dvec3(-0.75, 1.5, -0.75)}
  );

  REQUIRE(talus::destruction::isAnchored(broken, broken.component[corner]));
  REQUIRE(talus::destruction::isAnchored(freed, freed.component[edge]));
  REQUIRE(!talus::destruction::isAnchored(freed, freed.component[corner]));
  REQUIRE(!talus::destruction::isAnchored(freed, freed.component[post]));
  REQUIRE(freed.component[corner] != freed.component[post]);

  const std::vector<ChunkLoad> shaken{
    ChunkLoad{corner, glm::dvec3(4.0e3, -9.0e3, 2.5e3), glm::dvec3(0.0)},
    ChunkLoad{edge, glm::dvec3(0.0), glm::dvec3(1.5e3, -2.0e3, 0.8e3)},
    ChunkLoad{post, glm::dvec3(1.0e3, 2.0e3, -0.5e3), glm::dvec3(0.0, 0.3e3, 0.0)},
  };

  const ChunkedBuilding uneven = talus::destruction::chunkBuilding(unevenTable(), materials, plain());
  const auto [smallest, largest] = std::minmax_element(uneven.bonds.begin(), uneven.bonds.end(), [](const Bond& a, const Bond& b) {
    return a.area < b.area;
  });

  REQUIRE(uneven.chunks.size() == 28u);
  REQUIRE(uneven.bonds.size() == 39u);
  REQUIRE(largest->area >= 6.0 * smallest->area);

  const uint32_t rim = chunkAt(uneven, glm::dvec3(0.75, 1.65, 0.75));
  const uint32_t side = chunkAt(uneven, glm::dvec3(-0.25, 1.65, 0.75));
  const uint32_t gone = chunkAt(uneven, glm::dvec3(-0.25, 1.65, -0.25));

  REQUIRE(rim != 0xFFFFFFFFu);
  REQUIRE(side != 0xFFFFFFFFu);
  REQUIRE(gone != 0xFFFFFFFFu);

  const std::vector<ChunkLoad> leaned{
    ChunkLoad{rim, glm::dvec3(-3.0e3, -6.0e3, 1.5e3), glm::dvec3(0.0)},
    ChunkLoad{side, glm::dvec3(0.0), glm::dvec3(0.9e3, -1.2e3, 0.4e3)},
    ChunkLoad{gone, glm::dvec3(5.0e3, 0.0, 0.0), glm::dvec3(0.0)},
  };
  const StructureState even = talus::destruction::makeStructure(uneven);
  StructureState split = talus::destruction::makeStructure(uneven);

  talus::destruction::removeChunk(split, uneven, gone);
  breakAt(uneven, split, {glm::dvec3(0.5, 1.65, 0.75), glm::dvec3(0.75, 1.65, 0.5), glm::dvec3(0.75, 1.0, 0.75)});

  REQUIRE(!talus::destruction::isAnchored(split, split.component[rim]));
  REQUIRE(talus::destruction::isAnchored(split, split.component[side]));

  const auto wear = [](const ChunkedBuilding& chunked, StructureState& structure, std::initializer_list<std::pair<glm::dvec3, double>> healths) {
    for (const auto& [centroid, health] : healths) {
      const uint32_t bond = bondAt(chunked, centroid);

      REQUIRE(bond != 0xFFFFFFFFu);

      structure.health[bond] = health;
    }
  };
  const std::initializer_list<std::pair<glm::dvec3, double>> tableWear{
    {glm::dvec3(-0.75, 1.5, -0.75), 0.7},
    {glm::dvec3(0.75, 1.5, -0.75), 0.35},
    {glm::dvec3(-0.75, 1.5, 0.75), 0.1},
    {glm::dvec3(0.0, 1.75, 0.25), 0.02},
    {glm::dvec3(0.5, 1.75, -0.75), 0.5},
  };
  StructureState worn = talus::destruction::makeStructure(standing);
  StructureState wornCracked = cracked;
  StructureState wornUneven = talus::destruction::makeStructure(uneven);
  StructureState faint = talus::destruction::makeStructure(standing);

  wear(standing, worn, tableWear);
  wear(standing, wornCracked, tableWear);
  wear(uneven, wornUneven, {{glm::dvec3(0.75, 1.5, -0.75), 0.3}, {glm::dvec3(-0.75, 1.5, -0.75), 0.6}, {glm::dvec3(0.5, 1.65, 0.75), 0.05}});
  wear(standing, faint, {{glm::dvec3(0.75, 1.5, 0.75), 1.0e-6}});

  const auto finite = [](const std::vector<glm::dvec3>& values) {
    return std::all_of(values.begin(), values.end(), [](const glm::dvec3& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    });
  };

  struct Case {
    const char* name;
    const ChunkedBuilding* chunked;
    const StructureState* structure;
    const std::vector<ChunkLoad>* loads;
  };

  for (const Case& test : {
         Case{"redundant anchored frame", &standing, &intact, &pushed},
         Case{"cracked joints", &standing, &cracked, &pushed},
         Case{"free piece beside an anchored frame", &pair, &apart, &thrown},
         Case{"broken joints", &standing, &broken, &pushed},
         Case{"a removed chunk", &standing, &removed, &pushed},
         Case{"free groups inside an anchored frame", &standing, &freed, &shaken},
         Case{"unequal bond areas", &uneven, &even, &leaned},
         Case{"unequal areas and masses, a free group and a removed chunk", &uneven, &split, &leaned},
         Case{"worn joints", &standing, &worn, &pushed},
         Case{"worn and cracked joints", &standing, &wornCracked, &pushed},
         Case{"worn bonds of unequal areas", &uneven, &wornUneven, &leaned},
         Case{"one bond at health 1e-6", &standing, &faint, &pushed},
       }) {
    INFO(test.name);

    StressState stress = talus::destruction::makeStress(*test.chunked);

    talus::destruction::solveStress(*test.chunked, *test.structure, stress, *test.loads, StressSettings{});

    const BondLoads reference = leastNormBondLoads(*test.chunked, *test.structure, *test.loads, StressSettings{}.gravity);
    const Agreement result = agreement(*test.chunked, *test.structure, stress, reference);

    CHECK(finite(stress.force));
    CHECK(finite(stress.moment));
    CHECK(result.largestForce > 0.0);
    CHECK(result.largestMoment > 0.0);
    CHECK(result.force <= 1.0e-9 * result.largestForce);
    CHECK(result.moment <= 1.0e-9 * result.largestMoment);
  }
}

TEST_CASE("of two parallel bonds of equal area, one at half health carries a fifth of their axial load", "[destruction]") {
  ChunkedBuilding pair = chain(1u, glm::dvec3(0.0, 1.0, 0.0), false);

  pair.bonds.push_back(pair.bonds[0]);

  StructureState structure = talus::destruction::makeStructure(pair);
  StressState stress = talus::destruction::makeStress(pair);
  const double push = 2.0e3;
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -push, 0.0), glm::dvec3(0.0)}};

  structure.health[0] = 0.5;
  talus::destruction::solveStress(pair, structure, stress, load, StressSettings{});

  const double worn = glm::dot(stress.force[0], pair.bonds[0].normal);
  const double whole = glm::dot(stress.force[1], pair.bonds[1].normal);
  const double carried = push + MASS * G;
  const double share = worn / (worn + whole);

  CHECK(std::abs(share - 0.2) <= 1.0e-9 * 0.2);
  CHECK(std::abs(std::abs(worn + whole) - carried) <= 1.0e-9 * carried);
}

TEST_CASE("a worn bond with no other path keeps its whole load", "[destruction]") {
  const ChunkedBuilding column = chain(4u, glm::dvec3(0.0, 1.0, 0.0), false);
  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const double push = 2.0e3;
  const std::vector<ChunkLoad> load{ChunkLoad{4u, glm::dvec3(0.0, -push, 0.0), glm::dvec3(0.0)}};

  REQUIRE(column.bonds.size() == 4u);

  structure.health[1] = 0.25;
  talus::destruction::solveStress(column, structure, stress, load, StressSettings{});

  for (uint32_t bond = 0u; bond != 4u; ++bond) {
    INFO("bond " << bond);

    const double above = static_cast<double>(4u - bond) * MASS * G + push;

    CHECK(std::abs(std::abs(glm::dot(stress.force[bond], column.bonds[bond].normal)) - above) <= 1.0e-9 * above);
  }
}

TEST_CASE("a factor rebuilt after breaks and removals holds fewer blocks and still gives the weighted smallest bond loads", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  const ChunkedBuilding standing = talus::destruction::chunkBuilding(table(glm::dvec3(0.0)), materials, plain());
  const uint32_t corner = chunkAt(standing, glm::dvec3(0.75, 1.75, 0.75));
  const uint32_t edge = chunkAt(standing, glm::dvec3(0.25, 1.75, -0.25));

  REQUIRE(corner != 0xFFFFFFFFu);
  REQUIRE(edge != 0xFFFFFFFFu);

  const std::vector<ChunkLoad> pushed{
    ChunkLoad{corner, glm::dvec3(4.0e3, -9.0e3, 2.5e3), glm::dvec3(0.0)},
    ChunkLoad{edge, glm::dvec3(0.0), glm::dvec3(1.5e3, -2.0e3, 0.8e3)},
  };
  StructureState structure = talus::destruction::makeStructure(standing);
  StressState stress = talus::destruction::makeStress(standing);

  talus::destruction::solveStress(standing, structure, stress, pushed, StressSettings{});

  uint64_t bytes = talus::destruction::factorBytes(stress);

  REQUIRE(bytes > 0u);

  const auto breakAt = [&](const glm::dvec3& centroid) {
    REQUIRE(bondAt(standing, centroid) != 0xFFFFFFFFu);

    talus::destruction::breakBond(structure, standing, bondAt(standing, centroid));
  };
  const auto shrinks = [&](const char* step) {
    INFO(step);

    StressState fresh = talus::destruction::makeStress(standing);

    talus::destruction::solveStress(standing, structure, stress, pushed, StressSettings{});
    talus::destruction::solveStress(standing, structure, fresh, pushed, StressSettings{});

    const BondLoads reference = leastNormBondLoads(standing, structure, pushed, StressSettings{}.gravity);
    const Agreement result = agreement(standing, structure, stress, reference);

    CHECK(talus::destruction::factorBytes(stress) < bytes);
    CHECK(talus::destruction::factorBytes(stress) == talus::destruction::factorBytes(fresh));
    CHECK(stress.force == fresh.force);
    CHECK(stress.moment == fresh.moment);
    CHECK(result.largestForce > 0.0);
    CHECK(result.force <= 1.0e-9 * result.largestForce);
    CHECK(result.moment <= 1.0e-9 * result.largestMoment);

    bytes = talus::destruction::factorBytes(stress);
  };

  breakAt(glm::dvec3(0.25, 1.75, -0.5));
  static_cast<void>(talus::destruction::updateComponents(structure, standing));

  REQUIRE(structure.component[corner] == structure.component[edge]);
  REQUIRE(talus::destruction::isAnchored(structure, structure.component[corner]));

  shrinks("a bond broken inside the table");

  for (const glm::dvec3& centroid : {
         glm::dvec3(0.0, 1.75, 0.25),
         glm::dvec3(0.0, 1.75, 0.75),
         glm::dvec3(0.25, 1.75, 0.0),
         glm::dvec3(0.75, 1.75, 0.0),
         glm::dvec3(0.75, 1.5, 0.75),
       }) {
    breakAt(centroid);
  }

  shrinks("a corner broken free");

  for (uint32_t index = 0u; index != standing.chunks.size(); ++index) {
    if (standing.chunks[index].centre.x < -0.5) {
      talus::destruction::removeChunk(structure, standing, index);
    }
  }

  shrinks("a third of the chunks removed");
}

TEST_CASE("a glazed house stands under its own weight", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), materials);
  const StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);

  talus::destruction::solveStress(chunked, structure, stress, {}, StressSettings{});

  CHECK(pastLimits(chunked, stress, ANY_MEMBER) == 0u);
}

TEST_CASE("a glazed house's pane carries none of the wall when load follows bond area", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), materials);
  const StructureState structure = talus::destruction::makeStructure(chunked);
  StressState byDefault = talus::destruction::makeStress(chunked);
  StressState equal = talus::destruction::makeStress(chunked, MassWeighting::Real, LoadSharing::Equal);

  talus::destruction::solveStress(chunked, structure, byDefault, {}, StressSettings{});
  talus::destruction::solveStress(chunked, structure, equal, {}, StressSettings{});

  CHECK(pastLimits(chunked, byDefault, PANE) == 0u);
  CHECK(pastLimits(chunked, equal, PANE) > 0u);
}

TEST_CASE("destruction weighting probe", "[.][destructionweighting]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding glazed = talus::destruction::chunkBuilding(house(), materials);
  const ChunkedBuilding beam = chain(32u, glm::dvec3(1.0, 0.0, 0.0), true);
  const ChunkedBuilding column = chain(60u, glm::dvec3(0.0, 1.0, 0.0), false);
  const StressSettings settings;

  for (const MassWeighting weighting : {MassWeighting::Real, MassWeighting::Equal}) {
    for (const LoadSharing sharing : {LoadSharing::Area, LoadSharing::Equal}) {
      for (const ChunkedBuilding* chunked : {&glazed, &beam, &column}) {
        const StructureState structure = talus::destruction::makeStructure(*chunked);
        StressState stress = talus::destruction::makeStress(*chunked, weighting, sharing);

        talus::destruction::solveStress(*chunked, structure, stress, {}, settings);

        std::printf(
          "destructionweighting %s weighting %s sharing %s past limits %u\n",
          chunked == &glazed ? "house" : (chunked == &beam ? "beam32" : "column60"),
          weighting == MassWeighting::Real ? "real" : "equal",
          sharing == LoadSharing::Area ? "area" : "equal",
          chunked == &glazed ? pastLimits(*chunked, stress, ANY_MEMBER) : 0u
        );
      }
    }
  }
}

namespace {

using talus::destruction::StressReport;

ChunkedBuilding unbreakable(ChunkedBuilding chunked) {
  for (Bond& bond : chunked.bonds) {
    bond.elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
    bond.fatal = bond.elastic;
  }

  return chunked;
}

StructuralMaterial stronger(StructuralMaterial material, double factor) {
  for (ModeLimits* limits : {&material.elastic, &material.fatal, &material.acrossElastic, &material.acrossFatal}) {
    limits->compression *= factor;
    limits->tension *= factor;
    limits->shear *= factor;
  }

  return material;
}

StructuralMaterial ductile() {
  StructuralMaterial material;

  material.density = 1000.0;
  material.elastic = ModeLimits{10.0e6, 1.0e6, 10.0e6};
  material.fatal = ModeLimits{20.0e6, 2.0e6, 20.0e6};

  return material;
}

Building cantilever() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.25, 0.0), glm::dvec3(0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(2.25, 1.25, 0.0), glm::dvec3(2.0, 0.25, 0.25), 0u, Role::Beam});

  return building;
}

Building hanging() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 3.25, 0.0), glm::dvec3(0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(0.0, 2.75, 0.0), glm::dvec3(0.25), 0u, Role::Column});

  return building;
}

struct Outcome {
  bool settled = false;
  size_t broken = 0u;
};

Outcome settle(
  const ChunkedBuilding& chunked,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings,
  uint32_t updates
) {
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  Outcome outcome;

  for (uint32_t update = 0u; update != updates && !outcome.settled; ++update) {
    const StressReport report = talus::destruction::updateStress(chunked, structure, stress, loads, settings);

    outcome.settled = !report.ran;
    outcome.broken += report.broken.size();
    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  }

  return outcome;
}

Outcome loadCantilever(const StructuralMaterial& material) {
  const std::vector<StructuralMaterial> materials{material};
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(cantilever(), materials, plain());
  const std::vector<ChunkLoad> tip{ChunkLoad{8u, glm::dvec3(0.0, -8000.0, 0.0), glm::dvec3(0.0)}};

  return settle(chunked, tip, StressSettings{}, 50u);
}

}

TEST_CASE("a structure whose loads are unchanged is not solved again, and new loads on it keep its factor", "[destruction]") {
  const ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));
  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const StressSettings settings;
  const std::vector<ChunkLoad> push{ChunkLoad{5u, glm::dvec3(100.0, 0.0, 0.0), glm::dvec3(0.0)}};
  const StressReport standing = talus::destruction::updateStress(column, structure, stress, {}, settings);

  CHECK(standing.ran);
  CHECK(standing.rounds == 1u);
  CHECK(standing.factorizations == 1u);
  CHECK(standing.solvedChunks == 5u);
  CHECK(standing.factorizedChunks == 5u);
  CHECK(!talus::destruction::updateStress(column, structure, stress, {}, settings).ran);

  const StressReport pushed = talus::destruction::updateStress(column, structure, stress, push, settings);

  CHECK(pushed.ran);
  CHECK(pushed.rounds == 1u);
  CHECK(pushed.factorizations == 0u);
  CHECK(pushed.solvedChunks == 5u);
  CHECK(pushed.factorizedChunks == 0u);
  CHECK(!talus::destruction::updateStress(column, structure, stress, push, settings).ran);

  talus::destruction::breakBond(structure, column, 4u);

  const StressReport parted = talus::destruction::updateStress(column, structure, stress, push, settings);

  CHECK(parted.ran);
  CHECK(parted.factorizations == 1u);
  CHECK(parted.factorizedChunks == 4u);
  CHECK(parted.solvedChunks == 4u * parted.rounds);
  CHECK(!talus::destruction::updateStress(column, structure, stress, push, settings).ran);
}

TEST_CASE("an update under a new gravity runs although its loads are unchanged", "[destruction]") {
  const ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));
  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  StressState fresh = talus::destruction::makeStress(column);
  StressSettings tilted;

  tilted.gravity = glm::dvec3(3.0, -9.81, 0.0);

  REQUIRE(talus::destruction::updateStress(column, structure, stress, {}, StressSettings{}).ran);

  const StressReport turned = talus::destruction::updateStress(column, structure, stress, {}, tilted);

  talus::destruction::solveStress(column, structure, fresh, {}, tilted);

  CHECK(turned.ran);
  CHECK(turned.rounds == 1u);
  CHECK(turned.factorizations == 0u);
  CHECK(stress.force == fresh.force);
  CHECK(stress.moment == fresh.moment);
  CHECK(!talus::destruction::updateStress(column, structure, stress, {}, tilted).ran);
}

TEST_CASE("an update that throws leaves its state unsolved, so a retry runs again", "[destruction]") {
  ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));
  const glm::dvec3 centre = column.chunks[3].centre;
  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const StressSettings settings;

  column.chunks[3].centre.x = std::numeric_limits<double>::quiet_NaN();

  CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::runtime_error);
  CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::runtime_error);

  column.chunks[3].centre = centre;

  REQUIRE(talus::destruction::updateStress(column, structure, stress, {}, settings).ran);

  talus::destruction::crackBond(structure, 1u);
  column.chunks[3].centre.x = std::numeric_limits<double>::quiet_NaN();

  CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::runtime_error);
  CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::runtime_error);

  column.chunks[3].centre = centre;

  StressState fresh = talus::destruction::makeStress(column);
  const StressReport retried = talus::destruction::updateStress(column, structure, stress, {}, settings);

  talus::destruction::solveStress(column, structure, fresh, {}, settings);

  CHECK(retried.ran);
  CHECK(retried.factorizations == 1u);
  CHECK(stress.force == fresh.force);
  CHECK(stress.moment == fresh.moment);
}

TEST_CASE("a throw at a revision that was already solved leaves the state unsolved, so a retry runs again", "[destruction]") {
  ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));
  const glm::dvec3 centre = column.chunks[3].centre;
  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  StressSettings weightless;
  const StressSettings settings;

  weightless.gravity = glm::dvec3(0.0);

  REQUIRE(talus::destruction::updateStress(column, structure, stress, {}, weightless).ran);
  REQUIRE(stress.solved);
  REQUIRE(talus::destruction::factorBytes(stress) == 0u);

  column.chunks[3].centre.x = std::numeric_limits<double>::quiet_NaN();

  CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::runtime_error);

  column.chunks[3].centre = centre;

  StressState fresh = talus::destruction::makeStress(column);
  const StressReport retried = talus::destruction::updateStress(column, structure, stress, {}, settings);

  talus::destruction::solveStress(column, structure, fresh, {}, settings);

  REQUIRE(fresh.force[0] != glm::dvec3(0.0));
  CHECK(retried.ran);
  CHECK(stress.force == fresh.force);
  CHECK(stress.moment == fresh.moment);
}

TEST_CASE("a gravity that is not finite is refused before the state changes", "[destruction]") {
  const ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));

  for (const glm::dvec3& gravity : {
         glm::dvec3(std::numeric_limits<double>::quiet_NaN(), -9.81, 0.0),
         glm::dvec3(0.0, -std::numeric_limits<double>::infinity(), 0.0),
       }) {
    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);
    StressSettings settings;

    settings.gravity = gravity;

    CHECK_THROWS_AS(talus::destruction::updateStress(column, structure, stress, {}, settings), std::invalid_argument);
    CHECK_THROWS_AS(talus::destruction::solveStress(column, structure, stress, {}, settings), std::invalid_argument);
    CHECK(!stress.solved);
    CHECK(stress.gravity == glm::dvec3(0.0));
    CHECK(std::count(structure.broken.begin(), structure.broken.end(), uint8_t{1}) == 0);
  }
}

TEST_CASE("a stress state used with another structure state at its factor's revision is refused", "[destruction]") {
  const ChunkedBuilding column = unbreakable(chain(5u, glm::dvec3(0.0, 1.0, 0.0), false));
  StructureState first = talus::destruction::makeStructure(column);
  StructureState second = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  StressState fresh = talus::destruction::makeStress(column);
  const std::vector<ChunkLoad> push{ChunkLoad{4u, glm::dvec3(100.0, 0.0, 0.0), glm::dvec3(0.0)}};

  talus::destruction::breakBond(first, column, 2u);
  talus::destruction::breakBond(second, column, 4u);

  REQUIRE(first.revision == second.revision);

  talus::destruction::solveStress(column, first, stress, {}, StressSettings{});

  REQUIRE(talus::destruction::factorBytes(stress) > 0u);
  CHECK_THROWS_AS(talus::destruction::solveStress(column, second, stress, {}, StressSettings{}), std::logic_error);
  CHECK_THROWS_AS(talus::destruction::updateStress(column, second, stress, push, StressSettings{}), std::logic_error);

  talus::destruction::solveStress(column, first, stress, {}, StressSettings{});
  talus::destruction::solveStress(column, first, fresh, {}, StressSettings{});

  CHECK(stress.force == fresh.force);
  CHECK(stress.moment == fresh.moment);
}

TEST_CASE("a refactorization after cracks, wear or both gives the bits a fresh factorization gives", "[destruction]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(
    talus::destruction::makeBuilding(
      talus::destruction::BuildingPlan{talus::destruction::BuildingType::ConcreteFrame, 3u, 2u, 2u}
    ),
    materials
  );
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  uint32_t loaded = 0u;

  while (chunked.chunks[loaded].anchored) {
    ++loaded;
  }

  const std::vector<ChunkLoad> push{ChunkLoad{loaded, glm::dvec3(4.0e3, -1.0e3, 2.0e3), glm::dvec3(0.0)}};
  const size_t bonds = chunked.bonds.size();
  uint32_t anchoredPair = 0u;

  while (!(chunked.chunks[chunked.bonds[anchoredPair].first].anchored && chunked.chunks[chunked.bonds[anchoredPair].second].anchored)) {
    ++anchoredPair;
  }

  talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{});

  const auto changes = std::vector<std::pair<const char*, std::function<void()>>>{
    {"one crack", [&] { talus::destruction::crackBond(structure, static_cast<uint32_t>(bonds / 3u)); }},
    {"three cracks far apart",
     [&] {
       talus::destruction::crackBond(structure, 7u);
       talus::destruction::crackBond(structure, static_cast<uint32_t>(bonds / 2u));
       talus::destruction::crackBond(structure, static_cast<uint32_t>(bonds - 5u));
     }},
    {"one bond worn",
     [&] {
       structure.health[bonds / 4u] *= 0.5;
       structure.revision += 1u;
     }},
    {"a crack and a worn bond",
     [&] {
       talus::destruction::crackBond(structure, static_cast<uint32_t>(bonds / 5u));
       structure.health[3u * bonds / 4u] *= 0.25;
       structure.revision += 1u;
     }},
    {"a crack between two anchored chunks", [&] { talus::destruction::crackBond(structure, anchoredPair); }},
  };

  for (const auto& [name, change] : changes) {
    INFO(name);

    change();
    talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{});

    StressState fresh = talus::destruction::makeStress(chunked);

    talus::destruction::solveStress(chunked, structure, fresh, push, StressSettings{});

    CHECK(stress.force == fresh.force);
    CHECK(stress.moment == fresh.moment);
  }
}

TEST_CASE("a factorization on 8 workers or with four-lane sums gives the bits one worker with two-lane sums gives, whole and after cracks", "[destruction]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  StressSettings narrow;
  StressSettings parallel;

  narrow.lanes = talus::destruction::Lanes::Two;
  parallel.workers = talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);

  REQUIRE(parallel.workers.count == 8u);

  for (const talus::destruction::BuildingPlan& plan : {
         talus::destruction::BuildingPlan{talus::destruction::BuildingType::ConcreteFrame, 3u, 2u, 2u},
         talus::destruction::BuildingPlan{talus::destruction::BuildingType::SteelWarehouse, 1u, 5u, 3u},
       }) {
    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState serial = talus::destruction::makeStress(chunked);
    StressState lanes = talus::destruction::makeStress(chunked);
    StressState wide = talus::destruction::makeStress(chunked);
    uint32_t loaded = 0u;

    while (chunked.chunks[loaded].anchored) {
      ++loaded;
    }

    const std::vector<ChunkLoad> push{ChunkLoad{loaded, glm::dvec3(3.0e3, -2.0e3, 1.0e3), glm::dvec3(0.0)}};
    const size_t bonds = chunked.bonds.size();

    for (uint32_t round = 0u; round != 4u; ++round) {
      INFO("storeys " << plan.storeys << " round " << round);

      if (round != 0u) {
        for (uint32_t crack = 0u; crack != round; ++crack) {
          talus::destruction::crackBond(structure, static_cast<uint32_t>((round * 7u + crack * 3u) * bonds / 37u % bonds));
        }
      }

      talus::destruction::solveStress(chunked, structure, serial, push, narrow);
      talus::destruction::solveStress(chunked, structure, lanes, push, StressSettings{});
      talus::destruction::solveStress(chunked, structure, wide, push, parallel);

      StressState fresh = talus::destruction::makeStress(chunked);

      talus::destruction::solveStress(chunked, structure, fresh, push, parallel);

      CHECK(lanes.force == serial.force);
      CHECK(lanes.moment == serial.moment);
      CHECK(wide.force == serial.force);
      CHECK(wide.moment == serial.moment);
      CHECK(fresh.force == serial.force);
      CHECK(fresh.moment == serial.moment);
    }
  }
}

TEST_CASE("a rerun after erosion refactorizes the values, keeps the pattern and moves load off the worn bond", "[destruction]") {
  const std::vector<StructuralMaterial> materials{ductile()};
  ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());

  REQUIRE(chunked.bonds.size() == 1u);

  Bond firm = chunked.bonds[0];

  firm.elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
  firm.fatal = firm.elastic;
  chunked.bonds.push_back(firm);

  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const double pull = 2.4e6 * chunked.bonds[0].area - chunked.chunks[1].mass * G;
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
  const StressSettings settings;
  const auto wornShare = [&]() {
    const double worn = glm::dot(stress.force[0], chunked.bonds[0].normal);
    const double held = glm::dot(stress.force[1], chunked.bonds[1].normal);

    return worn / (worn + held);
  };
  const StressReport first = talus::destruction::updateStress(chunked, structure, stress, load, settings);

  REQUIRE(first.broken.empty());
  CHECK(first.factorizations == 1u);
  CHECK(near(wornShare(), 0.5, 1.0e-12));
  CHECK(stress.eroding);
  CHECK(near(structure.health[0], 0.8, 1.0e-9));
  CHECK(structure.health[1] == 1.0);

  const double health = structure.health[0];
  const double expected = health * health / (1.0 + health * health);
  const uint64_t bytes = talus::destruction::factorBytes(stress);
  const StressReport second = talus::destruction::updateStress(chunked, structure, stress, load, settings);

  CHECK(second.ran);
  CHECK(second.rounds == 1u);
  CHECK(second.factorizations == 1u);
  CHECK(talus::destruction::factorBytes(stress) == bytes);
  CHECK(std::abs(wornShare() - expected) <= 1.0e-9 * expected);
}

TEST_CASE("a worn bond beside stiffer paths stops wearing, so its structure settles", "[destruction]") {
  const std::vector<StructuralMaterial> materials{ductile()};
  ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());

  REQUIRE(chunked.bonds.size() == 1u);

  Bond firm = chunked.bonds[0];

  firm.elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
  firm.fatal = firm.elastic;
  chunked.bonds.push_back(firm);
  chunked.bonds.push_back(firm);

  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const double pull = 3.5e6 * chunked.bonds[0].area - chunked.chunks[1].mass * G;
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
  const double fixed = (3.5 - std::sqrt(3.5 * 3.5 - 8.0)) / 2.0;
  uint32_t calls = 0u;
  bool settled = false;

  for (; calls != 1000u && !settled; ++calls) {
    settled = !talus::destruction::updateStress(chunked, structure, stress, load, StressSettings{}).ran;
  }

  REQUIRE(settled);
  CHECK(calls <= 18u);
  CHECK(structure.broken[0] == 0u);
  CHECK(structure.health[0] >= fixed);
  CHECK(structure.health[0] - fixed <= 2.0e-6 * fixed);
}

TEST_CASE("a worn bond whose wear steps shrink below the tolerance stops counting as eroding, so its structure settles", "[destruction]") {
  struct Case {
    double fatal;
    uint32_t limit;
  };

  for (const Case& test : {Case{5.0, 73u}, Case{11.0, 169u}, Case{101.0, 1307u}}) {
    INFO("fatal tension " << test.fatal << " times the elastic");

    const std::vector<StructuralMaterial> materials{ductile()};
    ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());

    REQUIRE(chunked.bonds.size() == 1u);

    chunked.bonds[0].fatal.tension = test.fatal * chunked.bonds[0].elastic.tension;

    Bond firm = chunked.bonds[0];

    firm.elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
    firm.fatal = firm.elastic;
    chunked.bonds.push_back(firm);
    chunked.bonds.push_back(firm);

    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    const double pull = 3.5e6 * chunked.bonds[0].area - chunked.chunks[1].mass * G;
    const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
    uint32_t calls = 0u;
    bool settled = false;

    for (; calls != 20000u && !settled; ++calls) {
      settled = !talus::destruction::updateStress(chunked, structure, stress, load, StressSettings{}).ran;
    }

    REQUIRE(settled);
    CHECK(calls <= test.limit);
    CHECK(structure.broken[0] == 0u);
  }
}

TEST_CASE("a bond worn below the health floor on a path with no alternative breaks, and its update finishes", "[destruction]") {
  ChunkedBuilding column = unbreakable(chain(4u, glm::dvec3(0.0, 1.0, 0.0), false));
  const double area = column.bonds[1].area;

  column.bonds[1].elastic = ModeLimits{10.0e6 * area, 1.0e6 * area, 10.0e6 * area};
  column.bonds[1].fatal = ModeLimits{20.0e6 * area, 2.0e6 * area, 20.0e6 * area};

  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const double pressure = 20.0e6 - 1.0e-10 * 10.0e6;
  const double push = pressure * area - 3.0 * MASS * G;
  const std::vector<ChunkLoad> load{ChunkLoad{4u, glm::dvec3(0.0, -push, 0.0), glm::dvec3(0.0)}};
  StressReport worn;

  REQUIRE_NOTHROW(worn = talus::destruction::updateStress(column, structure, stress, load, StressSettings{}));
  CHECK(worn.broken == std::vector<uint32_t>{1u});
  CHECK(structure.broken[1] == 1u);

  bool settled = false;

  for (uint32_t call = 0u; call != 100u && !settled; ++call) {
    StressReport next;

    REQUIRE_NOTHROW(next = talus::destruction::updateStress(column, structure, stress, load, StressSettings{}));
    settled = !next.ran;
  }

  CHECK(settled);
}

TEST_CASE("a determinate bond worn past what the solver can represent breaks in the update that wears it, plain, cracked or narrow, and no update throws", "[destruction]") {
  struct Variant {
    const char* name;
    bool cracked;
    bool narrow;
    double fails;
  };

  for (const Variant& variant : {
         Variant{"plain", false, false, 1.78e-8},
         Variant{"cracked", true, false, 1.78e-6},
         Variant{"1/100 area", false, true, 1.33e-6},
         Variant{"cracked, 1/100 area", true, true, 1.33e-4},
       }) {
    for (const double left : {1.6e-8, 0.6 * variant.fails}) {
      INFO(variant.name << ", worn to " << left);

      ChunkedBuilding column = unbreakable(chain(4u, glm::dvec3(0.0, 1.0, 0.0), false));

      if (variant.narrow) {
        column.bonds[1].area *= 0.01;
        column.bonds[1].wu *= 0.1;
        column.bonds[1].wv *= 0.1;
      }

      const double area = column.bonds[1].area;

      column.bonds[1].elastic = ModeLimits{10.0e6 * area, 1.0e6 * area, 10.0e6 * area};
      column.bonds[1].fatal = ModeLimits{20.0e6 * area, 2.0e6 * area, 20.0e6 * area};
      column.bonds[1].crackFriction = 0.7;

      StructureState structure = talus::destruction::makeStructure(column);
      StressState stress = talus::destruction::makeStress(column);

      if (variant.cracked) {
        talus::destruction::crackBond(structure, 1u);
      }

      const double push = (20.0e6 - left * 10.0e6) * area - 3.0 * MASS * G;
      const std::vector<ChunkLoad> load{ChunkLoad{4u, glm::dvec3(0.0, -push, 0.0), glm::dvec3(0.0)}};
      uint32_t brokenAt = 0u;
      bool settled = false;

      for (uint32_t call = 1u; call != 50u && !settled; ++call) {
        StressReport report;

        REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, StressSettings{}));

        settled = !report.ran;
        brokenAt = brokenAt == 0u && structure.broken[1] != 0u ? call : brokenAt;
      }

      CHECK(settled);
      CHECK(brokenAt == 1u);
    }
  }
}

TEST_CASE("a determinate bond worn past what the solver can represent breaks in that update when the stiffer bond that sets its limit is far from it", "[destruction]") {
  ChunkedBuilding column = unbreakable(chain(16u, glm::dvec3(0.0, 1.0, 0.0), false));

  column.bonds[15].area *= 100.0;
  column.bonds[15].wu *= 10.0;
  column.bonds[15].wv *= 10.0;

  const double area = column.bonds[1].area;

  column.bonds[1].elastic = ModeLimits{10.0e6 * area, 1.0e6 * area, 10.0e6 * area};
  column.bonds[1].fatal = ModeLimits{20.0e6 * area, 2.0e6 * area, 20.0e6 * area};

  StructureState structure = talus::destruction::makeStructure(column);
  StressState stress = talus::destruction::makeStress(column);
  const double left = 2.3e-6;
  const double push = (20.0e6 - left * 10.0e6) * area - 15.0 * MASS * G;
  const std::vector<ChunkLoad> load{ChunkLoad{16u, glm::dvec3(0.0, -push, 0.0), glm::dvec3(0.0)}};
  uint32_t brokenAt = 0u;
  bool settled = false;

  for (uint32_t call = 1u; call != 50u && !settled; ++call) {
    StressReport report;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, StressSettings{}));

    settled = !report.ran;
    brokenAt = brokenAt == 0u && structure.broken[1] != 0u ? call : brokenAt;
  }

  CHECK(settled);
  CHECK(brokenAt == 1u);
}

TEST_CASE("a factorization that fails breaks the softest worn or too thin bond of the failing piece and factorizes again, and a NaN still throws", "[destruction]") {
  const ChunkedBuilding column = unbreakable(chain(4u, glm::dvec3(0.0, 1.0, 0.0), false));
  const std::vector<ChunkLoad> load{ChunkLoad{4u, glm::dvec3(0.0, -2.0e3, 0.0), glm::dvec3(0.0)}};
  const StressSettings settings;

  {
    INFO("a bond written past what K can hold");

    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);
    StressReport report;

    structure.health[1] = 1.0e-9;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, settings));
    CHECK(report.ran);
    CHECK(structure.broken[1] == 1u);
    CHECK(std::count(report.broken.begin(), report.broken.end(), 1u) == 1);
  }

  {
    INFO("a worn bond that cracks past what K can hold");

    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);
    StressReport report;

    structure.health[1] = 1.0e-6;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, settings));
    REQUIRE(structure.broken[1] == 0u);

    talus::destruction::crackBond(structure, 1u);

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, settings));
    CHECK(structure.broken[1] == 1u);
    CHECK(std::count(report.broken.begin(), report.broken.end(), 1u) == 1);
  }

  {
    INFO("two bonds written past what K can hold");

    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);
    StressReport report;

    structure.health[1] = 1.0e-9;
    structure.health[2] = 1.0e-9;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(column, structure, stress, load, settings));
    CHECK(structure.broken[1] == 1u);
    CHECK(structure.broken[2] == 1u);
  }

  struct Thin {
    double ratio;
    bool cracks;
  };

  for (const Thin& thin : {Thin{1.0e-9, false}, Thin{8.0e-7, true}}) {
    INFO("an unworn bond of " << thin.ratio << " times its neighbours' area" << (thin.cracks ? ", cracked after an update" : ""));

    ChunkedBuilding sliver = column;

    sliver.bonds[1].area *= thin.ratio;
    sliver.bonds[1].wu *= std::sqrt(thin.ratio);
    sliver.bonds[1].wv *= std::sqrt(thin.ratio);

    StructureState structure = talus::destruction::makeStructure(sliver);
    StressState stress = talus::destruction::makeStress(sliver);
    StressReport report;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(sliver, structure, stress, load, settings));

    if (thin.cracks) {
      REQUIRE(structure.broken[1] == 0u);

      talus::destruction::crackBond(structure, 1u);

      REQUIRE_NOTHROW(report = talus::destruction::updateStress(sliver, structure, stress, load, settings));
    }

    CHECK(structure.broken[1] == 1u);
    CHECK(std::count(report.broken.begin(), report.broken.end(), 1u) == 1);
  }

  {
    INFO("a softer worn bond in another piece is left to the assessment");

    ChunkedBuilding pieces = column;
    Chunk anchor = pieces.chunks[0];
    Chunk hung = pieces.chunks[1];
    Bond joint = pieces.bonds[0];

    anchor.centre.x += 5.0;
    hung.centre.x += 5.0;
    joint.centroid.x += 5.0;
    joint.first = 5u;
    joint.second = 6u;
    pieces.chunks.push_back(anchor);
    pieces.chunks.push_back(hung);
    pieces.bonds.push_back(joint);

    StructureState structure = talus::destruction::makeStructure(pieces);
    StressState stress = talus::destruction::makeStress(pieces);
    StressReport report;

    structure.health[1] = 1.0e-9;
    structure.health[4] = 1.0e-10;

    REQUIRE_NOTHROW(report = talus::destruction::updateStress(pieces, structure, stress, load, settings));
    CHECK(report.factorizations == 2u);
    CHECK(structure.broken[1] == 1u);
    CHECK(structure.broken[4] == 1u);
  }

  {
    INFO("a NaN centre beside a worn bond");

    ChunkedBuilding spoiled = column;
    StructureState structure = talus::destruction::makeStructure(spoiled);
    StressState stress = talus::destruction::makeStress(spoiled);

    spoiled.chunks[2].centre.x = std::numeric_limits<double>::quiet_NaN();
    structure.health[1] = 0.5;

    CHECK_THROWS_AS(talus::destruction::updateStress(spoiled, structure, stress, load, settings), std::runtime_error);
    CHECK(structure.broken[1] == 0u);
  }
}

TEST_CASE("under equal load sharing a worn bond's weights scale by its health alone", "[destruction]") {
  ChunkedBuilding pair = chain(1u, glm::dvec3(0.0, 1.0, 0.0), false);

  pair.bonds.push_back(pair.bonds[0]);
  pair.bonds[1].area *= 0.25;
  pair.bonds[1].wu *= 0.5;
  pair.bonds[1].wv *= 0.5;

  StructureState structure = talus::destruction::makeStructure(pair);
  StressState stress = talus::destruction::makeStress(pair, MassWeighting::Real, LoadSharing::Equal);
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -2.0e3, 0.0), glm::dvec3(0.0)}};

  structure.health[0] = 0.5;
  talus::destruction::solveStress(pair, structure, stress, load, StressSettings{});

  const double worn = glm::dot(stress.force[0], pair.bonds[0].normal);
  const double other = glm::dot(stress.force[1], pair.bonds[1].normal);

  CHECK(std::abs(worn / (worn + other) - 0.2) <= 1.0e-9 * 0.2);
}

TEST_CASE("an untied glazed house settles under its own weight with every chunk anchored, losing only corner joints to uplift", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const Building building = house();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials);
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings;
  bool settled = false;
  std::vector<uint32_t> broken;
  std::vector<talus::destruction::BondStress> breaking;

  for (uint32_t update = 0u; update != 1000u && !settled; ++update) {
    const std::vector<double> health = structure.health;
    const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

    settled = !report.ran;

    for (const uint32_t index : report.broken) {
      broken.push_back(index);
      breaking.push_back(talus::destruction::bondStress(
        chunked.bonds[index],
        health[index],
        stress.force[index],
        stress.moment[index]
      ));
    }

    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  }

  REQUIRE(settled);

  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    CHECK(talus::destruction::isAnchored(structure, structure.component[index]));
  }

  for (size_t slot = 0u; slot != broken.size(); ++slot) {
    const Bond& bond = chunked.bonds[broken[slot]];
    const Chunk& first = chunked.chunks[bond.first];
    const Chunk& second = chunked.chunks[bond.second];
    const Chunk& wallChunk = first.member < 4u ? first : second;
    const Chunk& other = first.member < 4u ? second : first;
    const Member& wall = building.members[wallChunk.member];
    const int along = wall.halfExtents.x > wall.halfExtents.z ? 0 : 2;
    const double fromEnd = wall.halfExtents[along] - std::abs(wallChunk.centre[along] - wall.centre[along]);

    CHECK(wallChunk.member < 4u);
    CHECK(other.member == 4u);
    CHECK(breaking[slot].pressure < 0.0);
    CHECK(breaking[slot].compression < bond.fatal.compression / bond.area);
    CHECK(fromEnd < 4.0 * wallChunk.halfExtents[along]);
  }
}

TEST_CASE("a glazed house on a reinforced concrete ring beam settles under its own weight without breaking a bond", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  Building building = house();

  for (uint32_t wall = 0u; wall != 4u; ++wall) {
    building.members[wall].centre.y = 1.25;
    building.members[wall].halfExtents.y = 1.25;
  }

  building.members.push_back(Member{glm::dvec3(0.0, 2.75, -2.85), glm::dvec3(3.0, 0.25, 0.15), 1u, Role::Beam});
  building.members.push_back(Member{glm::dvec3(0.0, 2.75, 2.85), glm::dvec3(3.0, 0.25, 0.15), 1u, Role::Beam});
  building.members.push_back(Member{glm::dvec3(-2.85, 2.75, 0.0), glm::dvec3(0.15, 0.25, 2.7), 1u, Role::Beam});
  building.members.push_back(Member{glm::dvec3(2.85, 2.75, 0.0), glm::dvec3(0.15, 0.25, 2.7), 1u, Role::Beam});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials);
  const Outcome outcome = settle(chunked, {}, StressSettings{}, 1000u);

  REQUIRE(chunked.chunks.size() == 438u);
  REQUIRE(outcome.settled);
  CHECK(outcome.broken == 0u);
}

TEST_CASE("a material twice as strong breaks no bond where the weaker one breaks some", "[destruction]") {
  const Outcome weak = loadCantilever(talus::destruction::concrete());
  const Outcome strong = loadCantilever(stronger(talus::destruction::concrete(), 2.0));

  REQUIRE(weak.settled);
  REQUIRE(strong.settled);
  CHECK(weak.broken > 0u);
  CHECK(strong.broken == 0u);
}

TEST_CASE("a load that does not break a bond still damages it, so the same load again breaks it sooner", "[destruction]") {
  const std::vector<StructuralMaterial> materials{ductile()};
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());

  REQUIRE(chunked.chunks.size() == 2u);
  REQUIRE(chunked.bonds.size() == 1u);

  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const double pull = 1.6e6 * chunked.bonds[0].area - chunked.chunks[1].mass * G;
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
  const StressSettings settings;

  REQUIRE(talus::destruction::updateStress(chunked, structure, stress, load, settings).broken.empty());
  CHECK(near(structure.health[0], 0.4, 1.0e-9));
  REQUIRE(talus::destruction::updateStress(chunked, structure, stress, {}, settings).broken.empty());
  CHECK(near(structure.health[0], 0.4, 1.0e-9));
  CHECK(talus::destruction::updateStress(chunked, structure, stress, load, settings).broken == std::vector<uint32_t>{0u});
  CHECK(structure.broken[0] == 1u);
}

TEST_CASE("a steady load past the elastic limit erodes its bond until it breaks", "[destruction]") {
  const std::vector<StructuralMaterial> materials{ductile()};
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const double pull = 1.2e6 * chunked.bonds[0].area - chunked.chunks[1].mass * G;
  const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
  const StressSettings settings;

  REQUIRE(talus::destruction::updateStress(chunked, structure, stress, load, settings).broken.empty());
  CHECK(near(structure.health[0], 0.8, 1.0e-9));
  REQUIRE(talus::destruction::updateStress(chunked, structure, stress, load, settings).broken.empty());
  CHECK(near(structure.health[0], 0.4, 1.0e-9));
  CHECK(talus::destruction::updateStress(chunked, structure, stress, load, settings).broken == std::vector<uint32_t>{0u});
  CHECK(talus::destruction::updateStress(chunked, structure, stress, load, settings).ran);
  CHECK(!talus::destruction::updateStress(chunked, structure, stress, load, settings).ran);
}

namespace {

uint64_t mixBits(uint64_t hash, double value) {
  uint64_t bits = 0u;

  std::memcpy(&bits, &value, sizeof(bits));

  for (int byte = 0; byte != 8; ++byte) {
    hash ^= (bits >> (8 * byte)) & 0xFFu;
    hash *= 1099511628211ull;
  }

  return hash;
}

uint32_t firstChunkOf(const ChunkedBuilding& chunked, uint32_t member) {
  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    if (chunked.chunks[index].member == member) {
      return index;
    }
  }

  return 0u;
}

struct Demolition {
  uint64_t hash = 1469598103934665603ull;
  size_t broken = 0u;
};

Demolition demolish() {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), materials);
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings;
  const std::vector<ChunkLoad> hit{
    ChunkLoad{firstChunkOf(chunked, 0u) + 30u, glm::dvec3(0.0, 0.0, 2.0e5), glm::dvec3(0.0)},
    ChunkLoad{firstChunkOf(chunked, 5u) + 3u, glm::dvec3(4.0e6, 0.0, 0.0), glm::dvec3(0.0, 1.0e4, 0.0)},
  };
  Demolition demolition;

  for (uint32_t update = 0u; update != 40u; ++update) {
    const bool hitting = update >= 5u && update < 8u;
    const StressReport report = talus::destruction::updateStress(
      chunked,
      structure,
      stress,
      hitting ? std::span<const ChunkLoad>(hit) : std::span<const ChunkLoad>(),
      settings
    );

    demolition.broken += report.broken.size();
    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  }

  for (size_t index = 0u; index != chunked.bonds.size(); ++index) {
    for (int axis = 0; axis != 3; ++axis) {
      demolition.hash = mixBits(demolition.hash, stress.force[index][axis]);
      demolition.hash = mixBits(demolition.hash, stress.moment[index][axis]);
    }

    demolition.hash = mixBits(demolition.hash, structure.health[index]);
    demolition.hash = mixBits(demolition.hash, static_cast<double>(structure.broken[index]));
    demolition.hash = mixBits(demolition.hash, static_cast<double>(structure.cracked[index]));
  }

  for (const uint32_t component : structure.component) {
    demolition.hash = mixBits(demolition.hash, static_cast<double>(component));
  }

  return demolition;
}

}

TEST_CASE("the same building and loads give bit-identical stresses in two runs", "[destruction]") {
  const Demolition first = demolish();
  const Demolition second = demolish();

  CHECK(first.broken > 0u);
  CHECK(first.hash == second.hash);
}

namespace {

struct Settled {
  uint64_t hash = 1469598103934665603ull;
  size_t broken = 0u;
  size_t cracked = 0u;
};

Settled settledHash(const ChunkedBuilding& chunked) {
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings;
  Settled settled;

  for (uint32_t update = 0u; update != 1000u; ++update) {
    const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

    static_cast<void>(talus::destruction::updateComponents(structure, chunked));

    if (!report.ran) {
      break;
    }
  }

  for (size_t index = 0u; index != chunked.bonds.size(); ++index) {
    for (int axis = 0; axis != 3; ++axis) {
      settled.hash = mixBits(settled.hash, stress.force[index][axis]);
      settled.hash = mixBits(settled.hash, stress.moment[index][axis]);
    }

    settled.hash = mixBits(settled.hash, structure.health[index]);
    settled.hash = mixBits(settled.hash, static_cast<double>(structure.broken[index]));
    settled.hash = mixBits(settled.hash, static_cast<double>(structure.cracked[index]));
    settled.broken += structure.broken[index];
    settled.cracked += structure.cracked[index];
  }

  return settled;
}

}

TEST_CASE("stress solve hash", "[.][stresshash]") {
  const Settled glazed = settledHash(talus::destruction::chunkBuilding(house(), houseMaterials()));
  const talus::destruction::BuildingPlan threeStoreys{talus::destruction::BuildingType::ConcreteFrame, 3u, 2u, 2u};
  const Settled frame = settledHash(
    talus::destruction::chunkBuilding(talus::destruction::makeBuilding(threeStoreys), talus::destruction::districtMaterials())
  );

  std::printf(
    "stresshash house broken %zu cracked %zu hash %016llx\n",
    glazed.broken,
    glazed.cracked,
    static_cast<unsigned long long>(glazed.hash)
  );
  std::printf(
    "stresshash frame broken %zu cracked %zu hash %016llx\n",
    frame.broken,
    frame.cracked,
    static_cast<unsigned long long>(frame.hash)
  );
}

TEST_CASE("destruction state hash", "[.][destructionhash]") {
  const Demolition demolition = demolish();

  std::printf(
    "destructionhash broken %zu hash %016llx\n",
    demolition.broken,
    static_cast<unsigned long long>(demolition.hash)
  );
}

namespace {

using talus::destruction::BondAssessment;

constexpr uint32_t NO_BOND = 0xFFFFFFFFu;

uint32_t bondJoining(const ChunkedBuilding& chunked, uint32_t first, uint32_t second) {
  for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
    if (chunked.bonds[index].first == first && chunked.bonds[index].second == second) {
      return index;
    }
  }

  return NO_BOND;
}

struct Pressed {
  bool settled = false;
  uint8_t cracked = 0u;
  uint8_t broken = 0u;
};

}

TEST_CASE("a bond with a friction rule cracks when bending opens it under compression, and a cracked bond breaks when pulled, slid, rocked or crushed", "[destruction]") {
  Bond joint;

  joint.normal = glm::dvec3(0.0, 1.0, 0.0);
  joint.area = 0.25;
  joint.wu = 0.5;
  joint.wv = 0.5;
  joint.elastic = ModeLimits{8.0e6 * 0.25, 0.25e6 * 0.25, 0.35e6 * 0.25};
  joint.fatal = joint.elastic;
  joint.shearRule = ShearRule::Coulomb;
  joint.friction = 0.7;
  joint.crackFriction = 0.7;

  Bond fixed = joint;

  fixed.shearRule = ShearRule::Fixed;
  fixed.friction = 0.0;
  fixed.crackFriction = 0.0;

  Bond seat = fixed;

  seat.crackFriction = 0.4;

  Bond flat = joint;

  flat.wu = 1.0;
  flat.wv = 0.25;

  const glm::dvec3 bearing(0.0, 50.0e3, 0.0);
  const auto assess = [](const Bond& bond, const glm::dvec3& force, const glm::dvec3& moment, bool cracked) {
    return talus::destruction::assessBond(bond, talus::destruction::bondStress(bond, 1.0, force, moment), cracked);
  };

  const BondAssessment holding = assess(joint, bearing, glm::dvec3(0.0, 0.0, 5.0e3), false);
  const BondAssessment opening = assess(joint, bearing, glm::dvec3(0.0, 0.0, 15.0e3), false);
  const BondAssessment pulled = assess(joint, glm::dvec3(0.0, -100.0e3, 0.0), glm::dvec3(0.0), false);
  const BondAssessment brittle = assess(fixed, bearing, glm::dvec3(0.0, 0.0, 15.0e3), false);

  CHECK(holding.damage == 0.0);
  CHECK(!holding.cracks);
  CHECK(opening.cracks);
  CHECK(opening.damage == 0.0);
  CHECK(!pulled.cracks);
  CHECK(pulled.damage == 1.0);
  CHECK(!brittle.cracks);
  CHECK(brittle.damage == 1.0);

  CHECK(assess(flat, bearing, glm::dvec3(0.0, 0.0, 7.0e3), true).damage == 1.0);
  CHECK(assess(flat, bearing, glm::dvec3(7.0e3, 0.0, 0.0), true).damage == 0.0);
  CHECK(assess(joint, bearing + glm::dvec3(20.0e3, 0.0, 0.0), glm::dvec3(0.0), true).damage == 0.0);
  CHECK(assess(joint, bearing + glm::dvec3(40.0e3, 0.0, 0.0), glm::dvec3(0.0), true).damage == 1.0);
  CHECK(assess(joint, glm::dvec3(0.0, -1.0, 0.0), glm::dvec3(0.0), true).damage == 1.0);
  CHECK(assess(joint, glm::dvec3(0.0, 2.1e6, 0.0), glm::dvec3(0.0), true).damage == 1.0);
  CHECK(assess(joint, glm::dvec3(0.0), glm::dvec3(0.0), true).damage == 0.0);
  CHECK(!assess(joint, bearing, glm::dvec3(0.0, 0.0, 15.0e3), true).cracks);

  CHECK(assess(seat, bearing, glm::dvec3(0.0, 0.0, 15.0e3), false).cracks);
  CHECK(assess(seat, bearing + glm::dvec3(15.0e3, 0.0, 0.0), glm::dvec3(0.0), true).damage == 0.0);
  CHECK(assess(seat, bearing + glm::dvec3(25.0e3, 0.0, 0.0), glm::dvec3(0.0), true).damage == 1.0);
}

TEST_CASE("a reinforced concrete slab on two masonry walls cracks its end joints into hinges and carries the simply supported moment", "[destruction]") {
  const std::vector<StructuralMaterial> materials{
    talus::destruction::brickMasonry(),
    talus::destruction::reinforcedConcrete(0.02),
  };
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(15.75, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(8.0, 1.35, 0.0), glm::dvec3(8.0, 0.35, 0.25), 1u, Role::Slab});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 36u);

  const uint32_t left = bondJoining(chunked, 1u, 4u);
  const uint32_t right = bondJoining(chunked, 3u, 35u);
  const uint32_t middle = bondJoining(chunked, 19u, 20u);

  REQUIRE(left != NO_BOND);
  REQUIRE(right != NO_BOND);
  REQUIRE(middle != NO_BOND);

  for (const StressSettings& settings : {StressSettings{}, deferred()}) {
    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    bool settled = false;
    size_t broken = 0u;

    for (uint32_t update = 0u; update != 2000u && !settled; ++update) {
      const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

      settled = !report.ran;
      broken += report.broken.size();
      static_cast<void>(talus::destruction::updateComponents(structure, chunked));
    }

    const double load = 32.0 * chunked.chunks[4].mass * G / 16.0;
    const double span = chunked.bonds[right].centroid.x - chunked.bonds[left].centroid.x;
    const double simple = load * span * span / 8.0;

    REQUIRE(settled);
    CHECK(broken == 0u);
    CHECK(structure.cracked[left] == 1u);
    CHECK(structure.cracked[right] == 1u);
    CHECK(near(span, 15.5, 1.0e-9));
    CHECK(near(std::abs(stress.moment[middle].z), simple, 0.1 * simple));
  }
}

TEST_CASE("a cantilever slab on a masonry wall cracks its root joint, then tips off the wall", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry(), talus::destruction::reinforcedConcrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.25, 0.0), glm::dvec3(0.25), 0u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(2.0, 0.65, 0.0), glm::dvec3(2.0, 0.15, 0.25), 1u, Role::Slab});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 9u);
  REQUIRE(chunked.chunks[0].anchored);

  const uint32_t root = bondJoining(chunked, 0u, 1u);

  REQUIRE(root != NO_BOND);

  for (const StressSettings& settings : {StressSettings{}, deferred()}) {
    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    uint32_t crackedAt = NO_BOND;
    uint32_t brokenAt = NO_BOND;
    bool settled = false;

    for (uint32_t update = 0u; update != 2000u && !settled; ++update) {
      const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

      settled = !report.ran;

      if (crackedAt == NO_BOND && std::find(report.cracked.begin(), report.cracked.end(), root) != report.cracked.end()) {
        crackedAt = update;
      }

      if (brokenAt == NO_BOND && std::find(report.broken.begin(), report.broken.end(), root) != report.broken.end()) {
        brokenAt = update;
      }

      static_cast<void>(talus::destruction::updateComponents(structure, chunked));
    }

    REQUIRE(settled);
    CHECK(crackedAt != NO_BOND);
    CHECK(brokenAt != NO_BOND);
    CHECK(crackedAt <= brokenAt);
    CHECK(talus::destruction::isAnchored(structure, structure.component[0]));
    CHECK(!talus::destruction::isAnchored(structure, structure.component[1]));
  }
}

TEST_CASE("a masonry pier stands under a central load, cracks under a load past its kern and breaks under a load past its rocking capacity", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 2u);
  REQUIRE(chunked.bonds.size() == 1u);

  for (const StressSettings& settings : {StressSettings{}, deferred()}) {
    const auto press = [&](double offset) {
      const double load = 200.0e3;
      const std::vector<ChunkLoad> loads{ChunkLoad{1u, glm::dvec3(0.0, -load, 0.0), glm::dvec3(0.0, 0.0, -load * offset)}};
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);
      Pressed pressed;

      for (uint32_t update = 0u; update != 2000u && !pressed.settled; ++update) {
        pressed.settled = !talus::destruction::updateStress(chunked, structure, stress, loads, settings).ran;
      }

      pressed.cracked = structure.cracked[0];
      pressed.broken = structure.broken[0];

      return pressed;
    };

    const Pressed centred = press(0.0);
    const Pressed kern = press(0.15);
    const Pressed beyond = press(0.3);

    CHECK(centred.settled);
    CHECK(centred.cracked == 0u);
    CHECK(centred.broken == 0u);
    CHECK(kern.settled);
    CHECK(kern.cracked == 1u);
    CHECK(kern.broken == 0u);
    CHECK(beyond.settled);
    CHECK(beyond.broken == 1u);
  }
}

namespace {

Building storeys(uint32_t count) {
  Building building;

  for (uint32_t storey = 0u; storey != count; ++storey) {
    const double base = 3.2 * storey;
    Building one = house();

    for (Member member : one.members) {
      member.centre.y += base;
      building.members.push_back(member);
    }

    for (Opening opening : one.openings) {
      opening.centre.y += base;
      building.openings.push_back(opening);
    }
  }

  return building;
}

}

TEST_CASE("destruction settle probe", "[.][destructionsettle]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();

  for (const uint32_t rounds : {StressSettings{}.crackRounds, 100000u}) {
    for (const uint32_t count : {1u, 16u}) {
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(storeys(count), materials);
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);
      StressSettings settings;
      uint32_t updates = 0u;
      uint64_t rounded = 0u;
      uint64_t factorizations = 0u;
      uint32_t most = 0u;
      uint32_t capped = 0u;
      size_t broken = 0u;
      size_t cracked = 0u;
      bool settled = false;

      settings.crackRounds = rounds;

      const auto start = std::chrono::steady_clock::now();

      for (; updates != 20000u && !settled; ++updates) {
        const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

        settled = !report.ran;
        rounded += report.rounds;
        factorizations += report.factorizations;
        most = std::max(most, report.rounds);
        capped += report.rounds == rounds ? 1u : 0u;
        broken += report.broken.size();
        cracked += report.cracked.size();
        static_cast<void>(talus::destruction::updateComponents(structure, chunked));
      }

      const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

      std::printf(
        "destructionsettle cap %u storeys %u chunks %zu bonds %zu settled %d updates %u rounds %llu factorizations %llu "
        "most rounds %u capped %u broken %zu cracked %zu ms %.1f factor bytes %llu\n",
        rounds, count, chunked.chunks.size(), chunked.bonds.size(), settled ? 1 : 0, updates,
        static_cast<unsigned long long>(rounded), static_cast<unsigned long long>(factorizations), most, capped, broken,
        cracked, milliseconds, static_cast<unsigned long long>(talus::destruction::factorBytes(stress))
      );

      if (count != 1u || !settled) {
        continue;
      }

      uint32_t target = 0u;

      for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
        const Chunk& chunk = chunked.chunks[index];

        if (chunk.member == 2u && std::abs(chunk.centre.y - 1.75) < 0.01
            && (chunked.chunks[target].member != 2u || std::abs(chunk.centre.z) < std::abs(chunked.chunks[target].centre.z))) {
          target = index;
        }
      }

      for (const double force : {3.0e5, 1.0e6}) {
        StructureState hitStructure = structure;
        StressState hitStress = stress;
        const std::vector<ChunkLoad> hit{ChunkLoad{target, glm::dvec3(force, 0.0, 0.0), glm::dvec3(0.0)}};
        const auto hitStart = std::chrono::steady_clock::now();
        const StressReport report = talus::destruction::updateStress(chunked, hitStructure, hitStress, hit, settings);
        const double hitMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - hitStart).count();

        std::printf(
          "destructionsettle cap %u hit %.0f N rounds %u factorizations %u cracked %zu broken %zu ms %.2f\n",
          rounds, force, report.rounds, report.factorizations, report.cracked.size(), report.broken.size(), hitMilliseconds
        );
      }
    }
  }
}

namespace {

bool settleUnder(
  const ChunkedBuilding& chunked,
  StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
) {
  for (uint32_t update = 0u; update != 2000u; ++update) {
    const StressReport report = talus::destruction::updateStress(chunked, structure, stress, loads, settings);

    static_cast<void>(talus::destruction::updateComponents(structure, chunked));

    if (!report.ran) {
      return true;
    }
  }

  return false;
}

StressReport wholeUpdate(
  const ChunkedBuilding& chunked,
  StructureState& structure,
  StressState& stress,
  std::span<const ChunkLoad> loads,
  const StressSettings& settings
) {
  StressReport whole = talus::destruction::updateStress(chunked, structure, stress, loads, settings);

  for (uint32_t call = 0u; call != 1000u && whole.pending; ++call) {
    const StressReport next = talus::destruction::updateStress(chunked, structure, stress, loads, settings);

    whole.rounds += next.rounds;
    whole.factorizations += next.factorizations;
    whole.pending = next.pending;
    whole.cracked.insert(whole.cracked.end(), next.cracked.begin(), next.cracked.end());
    whole.broken.insert(whole.broken.end(), next.broken.begin(), next.broken.end());
  }

  return whole;
}

uint32_t sideWallChunk(const ChunkedBuilding& chunked) {
  uint32_t target = NO_BOND;

  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    const Chunk& chunk = chunked.chunks[index];

    if (chunk.member == 2u && std::abs(chunk.centre.y - 1.75) < 0.01
        && (target == NO_BOND || std::abs(chunk.centre.z) < std::abs(chunked.chunks[target].centre.z))) {
      target = index;
    }
  }

  return target;
}

struct SettledHouse {
  ChunkedBuilding chunked;
  StructureState structure;
  StressState stress;
  std::vector<ChunkLoad> hit;
  uint64_t rounds = 0u;
  bool settled = false;
};

SettledHouse settledHouse() {
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), houseMaterials());
  SettledHouse result{
    chunked,
    talus::destruction::makeStructure(chunked),
    talus::destruction::makeStress(chunked),
    {ChunkLoad{sideWallChunk(chunked), glm::dvec3(3.0e5, 0.0, 0.0), glm::dvec3(0.0)}},
  };

  for (uint32_t update = 0u; update != 2000u && !result.settled; ++update) {
    const StressReport report = talus::destruction::updateStress(chunked, result.structure, result.stress, {}, StressSettings{});

    result.settled = !report.ran;
    result.rounds += report.rounds;
    static_cast<void>(talus::destruction::updateComponents(result.structure, chunked));
  }

  return result;
}

}

TEST_CASE("a cracked hinge carries the whole moment its overhang needs, so a slab held down by too little load tips off", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry(), talus::destruction::reinforcedConcrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.25, 0.0), glm::dvec3(0.25), 0u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(2.0, 0.65, 0.0), glm::dvec3(2.0, 0.15, 0.25), 1u, Role::Slab});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const uint32_t root = bondJoining(chunked, 0u, 1u);

  REQUIRE(chunked.chunks.size() == 9u);
  REQUIRE(root != NO_BOND);

  const double hinge = chunked.bonds[root].centroid.x;
  double weight = 0.0;
  double overturning = 0.0;

  for (uint32_t index = 1u; index != 9u; ++index) {
    weight += chunked.chunks[index].mass * G;
    overturning += chunked.chunks[index].mass * G * (chunked.chunks[index].centre.x - hinge);
  }

  const double holding = overturning / (0.5 * 0.5) - weight;

  REQUIRE(near(holding, 90055.8, 1.0));

  const auto loadOf = [](double load) {
    return std::vector<ChunkLoad>{ChunkLoad{1u, glm::dvec3(0.0, -load, 0.0), glm::dvec3(0.0)}};
  };

  for (const StressSettings& settings : {StressSettings{}, deferred()}) {
    for (const double fraction : {0.5, 0.8}) {
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);

      REQUIRE(settleUnder(chunked, structure, stress, loadOf(fraction * holding), settings));
      CHECK(structure.broken[root] == 1u);
      CHECK(!talus::destruction::isAnchored(structure, structure.component[1]));
    }

    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);

    REQUIRE(settleUnder(chunked, structure, stress, loadOf(1.2 * holding), settings));
    CHECK(structure.cracked[root] == 1u);
    CHECK(structure.broken[root] == 0u);
    CHECK(talus::destruction::isAnchored(structure, structure.component[1]));

    REQUIRE(settleUnder(chunked, structure, stress, loadOf(0.8 * holding), settings));
    CHECK(structure.broken[root] == 1u);
    CHECK(!talus::destruction::isAnchored(structure, structure.component[1]));
  }
}

TEST_CASE("a cantilever past its breaking load breaks beside a column however heavily the column is loaded", "[destruction]") {
  const StructuralMaterial concrete = talus::destruction::concrete();
  const std::vector<StructuralMaterial> materials{concrete};
  Building building = cantilever();

  building.members.push_back(Member{glm::dvec3(20.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const uint32_t column = bondJoining(chunked, 9u, 10u);

  REQUIRE(chunked.chunks.size() == 11u);
  REQUIRE(chunked.bonds[0].first == 0u);
  REQUIRE(chunked.bonds[0].second == 1u);
  REQUIRE(column != NO_BOND);

  chunked.bonds[column].elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
  chunked.bonds[column].fatal = chunked.bonds[column].elastic;

  double selfWeight = 0.0;

  for (uint32_t index = 1u; index != 9u; ++index) {
    selfWeight += chunked.chunks[index].mass * G * (chunked.chunks[index].centre.x - chunked.bonds[0].centroid.x);
  }

  const double arm = chunked.chunks[8].centre.x - chunked.bonds[0].centroid.x;
  const double breaking = (concrete.elastic.tension * (0.5 * 0.5 * 0.5 / 6.0) - selfWeight) / arm;

  REQUIRE(near(breaking, 1626.0, 1.0));

  for (const double columnLoad : {1.0e5, 1.0e9}) {
    const std::vector<ChunkLoad> loads{
      ChunkLoad{10u, glm::dvec3(0.0, -columnLoad, 0.0), glm::dvec3(0.0)},
      ChunkLoad{8u, glm::dvec3(0.0, -3.0 * breaking, 0.0), glm::dvec3(0.0)},
    };

    for (const StressSettings& settings : {StressSettings{}, deferred()}) {
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);

      REQUIRE(settleUnder(chunked, structure, stress, {}, settings));
      REQUIRE(structure.broken[0] == 0u);
      REQUIRE(settleUnder(chunked, structure, stress, loads, settings));
      CHECK(structure.broken[0] == 1u);
    }
  }
}

TEST_CASE("updates spread over calls end with exactly the cracks, breaks and forces of unlimited ones", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), materials);
  const uint32_t target = sideWallChunk(chunked);

  REQUIRE(target != NO_BOND);

  const std::vector<ChunkLoad> hit{ChunkLoad{target, glm::dvec3(3.0e5, 0.0, 0.0), glm::dvec3(0.0)}};

  struct Run {
    StructureState structure;
    StressState stress;
    bool settled = false;
    uint32_t pending = 0u;
    uint64_t rounds = 0u;
    size_t broken = 0u;
  };

  const auto run = [&](const StressSettings& settings) {
    Run result{talus::destruction::makeStructure(chunked), talus::destruction::makeStress(chunked)};

    for (const bool struck : {false, true}) {
      result.settled = false;

      for (uint32_t call = 0u; call != 2000u && !result.settled; ++call) {
        const std::span<const ChunkLoad> loads = struck && call == 0u ? std::span<const ChunkLoad>(hit) : std::span<const ChunkLoad>();
        const StressReport report = talus::destruction::updateStress(chunked, result.structure, result.stress, loads, settings);

        result.settled = !report.ran;
        result.pending += report.pending ? 1u : 0u;
        result.rounds += report.rounds;
        result.broken += report.broken.size();
        static_cast<void>(talus::destruction::updateComponents(result.structure, chunked));
      }
    }

    return result;
  };

  const Run whole = run(StressSettings{});
  const Run spread = run(deferred());

  REQUIRE(whole.settled);
  REQUIRE(spread.settled);
  CHECK(whole.pending == 0u);
  CHECK(spread.pending > 0u);
  CHECK(whole.broken > 20u);
  CHECK(spread.rounds == whole.rounds);
  CHECK(spread.structure.cracked == whole.structure.cracked);
  CHECK(spread.structure.broken == whole.structure.broken);
  CHECK(spread.structure.health == whole.structure.health);
  CHECK(spread.stress.force == whole.stress.force);
  CHECK(spread.stress.moment == whole.stress.moment);
}

TEST_CASE("an update's crack round limit counts its rounds across calls", "[destruction]") {
  const SettledHouse settled = settledHouse();

  REQUIRE(settled.settled);
  REQUIRE(settled.hit[0].chunk != NO_BOND);

  StressSettings limited;

  limited.crackRounds = 3u;

  StressSettings spread = limited;

  spread.roundsPerCall = 1u;

  StructureState unlimitedStructure = settled.structure;
  StressState unlimitedStress = settled.stress;
  StructureState wholeStructure = settled.structure;
  StressState wholeStress = settled.stress;
  StructureState spreadStructure = settled.structure;
  StressState spreadStress = settled.stress;
  const StressReport unlimited = talus::destruction::updateStress(
    settled.chunked,
    unlimitedStructure,
    unlimitedStress,
    settled.hit,
    StressSettings{}
  );
  const StressReport whole = talus::destruction::updateStress(settled.chunked, wholeStructure, wholeStress, settled.hit, limited);
  const StressReport parts = wholeUpdate(settled.chunked, spreadStructure, spreadStress, settled.hit, spread);

  REQUIRE(unlimited.rounds > limited.crackRounds);
  CHECK(whole.rounds == limited.crackRounds);
  CHECK(parts.rounds == limited.crackRounds);
  CHECK(parts.cracked == whole.cracked);
  CHECK(parts.broken == whole.broken);
  CHECK(spreadStructure.health == wholeStructure.health);
  CHECK(spreadStress.force == wholeStress.force);
  CHECK(spreadStress.moment == wholeStress.moment);
}

TEST_CASE("each update counts its crack rounds from zero, whatever updates ran before it", "[destruction]") {
  SettledHouse settled = settledHouse();

  REQUIRE(settled.settled);
  REQUIRE(settled.hit[0].chunk != NO_BOND);

  StructureState copied = settled.structure;
  StressState fresh = talus::destruction::makeStress(settled.chunked);
  const StressReport later = talus::destruction::updateStress(
    settled.chunked,
    settled.structure,
    settled.stress,
    settled.hit,
    StressSettings{}
  );
  const StressReport first = talus::destruction::updateStress(settled.chunked, copied, fresh, settled.hit, StressSettings{});

  REQUIRE(settled.rounds + first.rounds > StressSettings{}.crackRounds);
  CHECK(later.rounds == first.rounds);
  CHECK(later.cracked == first.cracked);
  CHECK(later.broken == first.broken);
  CHECK(settled.structure.health == copied.health);
  CHECK(settled.stress.force == fresh.force);
  CHECK(settled.stress.moment == fresh.moment);
}

TEST_CASE("a pending update keeps the gravity it started with", "[destruction]") {
  const SettledHouse settled = settledHouse();

  REQUIRE(settled.settled);
  REQUIRE(settled.hit[0].chunk != NO_BOND);

  StressSettings tilted = deferred();

  tilted.gravity = glm::dvec3(2.0, -9.81, 0.0);

  StructureState wholeStructure = settled.structure;
  StressState wholeStress = settled.stress;
  StructureState spreadStructure = settled.structure;
  StressState spreadStress = settled.stress;
  const StressReport whole = talus::destruction::updateStress(settled.chunked, wholeStructure, wholeStress, settled.hit, StressSettings{});
  StressReport parts = talus::destruction::updateStress(settled.chunked, spreadStructure, spreadStress, settled.hit, deferred());

  REQUIRE(parts.pending);

  for (uint32_t call = 0u; call != 1000u && parts.pending; ++call) {
    const StressReport next = talus::destruction::updateStress(settled.chunked, spreadStructure, spreadStress, settled.hit, tilted);

    CHECK(spreadStress.gravity == StressSettings{}.gravity);
    parts.rounds += next.rounds;
    parts.pending = next.pending;
    parts.cracked.insert(parts.cracked.end(), next.cracked.begin(), next.cracked.end());
    parts.broken.insert(parts.broken.end(), next.broken.begin(), next.broken.end());
  }

  CHECK(!parts.pending);
  CHECK(parts.rounds == whole.rounds);
  CHECK(parts.cracked == whole.cracked);
  CHECK(parts.broken == whole.broken);
  CHECK(spreadStress.force == wholeStress.force);
  CHECK(spreadStress.moment == wholeStress.moment);
}

TEST_CASE("a one-update load that cracks a joint and overloads the crack breaks it in that update", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
  const std::vector<ChunkLoad> loads{ChunkLoad{1u, glm::dvec3(0.0, -200.0e3, 0.0), glm::dvec3(0.0, 0.0, -200.0e3 * 0.3)}};

  REQUIRE(chunked.bonds.size() == 1u);

  for (const StressSettings& settings : {StressSettings{}, deferred()}) {
    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    const StressReport report = wholeUpdate(chunked, structure, stress, loads, settings);

    CHECK(report.cracked == std::vector<uint32_t>{0u});
    CHECK(report.broken == std::vector<uint32_t>{0u});
    CHECK(structure.broken[0] == 1u);
  }
}

TEST_CASE("a one-update hit on a house wall breaks bonds in that update", "[destruction]") {
  const std::vector<StructuralMaterial> materials = houseMaterials();
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), materials);
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings;

  REQUIRE(settleUnder(chunked, structure, stress, {}, settings));

  uint32_t target = NO_BOND;

  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    const Chunk& chunk = chunked.chunks[index];

    if (chunk.member == 2u && std::abs(chunk.centre.y - 1.75) < 0.01
        && (target == NO_BOND || std::abs(chunk.centre.z) < std::abs(chunked.chunks[target].centre.z))) {
      target = index;
    }
  }

  REQUIRE(target != NO_BOND);

  const std::vector<ChunkLoad> hit{ChunkLoad{target, glm::dvec3(3.0e5, 0.0, 0.0), glm::dvec3(0.0)}};
  const StressReport report = talus::destruction::updateStress(chunked, structure, stress, hit, settings);

  CHECK(!report.cracked.empty());
  CHECK(!report.broken.empty());
}

TEST_CASE("a timber floor on timber walls and on concrete walls carries a residential imposed load without losing an anchor", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::timber(), talus::destruction::concrete()};

  for (const uint32_t wallMaterial : {0u, 1u}) {
    Building building;

    building.members.push_back(Member{glm::dvec3(0.15, 1.5, 0.0), glm::dvec3(0.15, 1.5, 0.25), wallMaterial, Role::Wall});
    building.members.push_back(Member{glm::dvec3(5.85, 1.5, 0.0), glm::dvec3(0.15, 1.5, 0.25), wallMaterial, Role::Wall});
    building.members.push_back(Member{glm::dvec3(3.0, 3.1, 0.0), glm::dvec3(3.0, 0.1, 0.25), 0u, Role::Slab});

    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
    std::vector<ChunkLoad> imposed;

    for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
      if (chunked.chunks[index].member == 2u) {
        imposed.push_back(ChunkLoad{index, glm::dvec3(0.0, -2.0e3 * 0.5 * 0.5, 0.0), glm::dvec3(0.0)});
      }
    }

    REQUIRE(imposed.size() == 12u);

    for (const StressSettings& settings : {StressSettings{}, deferred()}) {
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);

      REQUIRE(settleUnder(chunked, structure, stress, imposed, settings));

      for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
        CHECK(structure.broken[index] == 0u);
      }

      for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
        CHECK(talus::destruction::isAnchored(structure, structure.component[index]));
      }
    }
  }
}

namespace {

using talus::destruction::DistrictMaterial;

const StructuralMaterial& districtMaterial(const std::vector<StructuralMaterial>& materials, DistrictMaterial material) {
  return materials[static_cast<uint32_t>(material)];
}

bool sameMaterial(const StructuralMaterial& a, const StructuralMaterial& b) {
  return a.density == b.density && sameModes(a.elastic, b.elastic) && sameModes(a.fatal, b.fatal)
         && sameModes(a.acrossElastic, b.acrossElastic) && sameModes(a.acrossFatal, b.acrossFatal)
         && a.shearRule == b.shearRule && a.friction == b.friction && a.anisotropy == b.anisotropy
         && a.headJoints == b.headJoints && a.variation == b.variation && a.chunkSize == b.chunkSize;
}

}

TEST_CASE("the district's materials are the cited picks with its reinforcement and glass chunk settings", "[destruction][district]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  REQUIRE(materials.size() == talus::destruction::districtMaterialCount);

  const StructuralMaterial& concrete = districtMaterial(materials, DistrictMaterial::Concrete);
  const StructuralMaterial reinforced = talus::destruction::reinforcedConcrete(0.02);
  StructuralMaterial tiles = talus::destruction::glass();

  tiles.chunkSize = 0.5;

  CHECK(concrete.density == reinforced.density);
  CHECK(concrete.elastic.compression == reinforced.elastic.compression);
  CHECK(concrete.fatal.compression == reinforced.fatal.compression);
  CHECK(concrete.elastic.tension == 0.02 * 380.0e6);
  CHECK(concrete.fatal.tension == 0.02 * 560.0e6);
  CHECK(near(concrete.elastic.shear, 0.45 * 2.6e6 + 0.005 * 380.0e6 * 0.7, 1.0e-9 * 2.5e6));
  CHECK(near(concrete.fatal.shear, 0.45 * 2.6e6 + 0.005 * 560.0e6 * 0.7, 1.0e-9 * 3.13e6));
  CHECK(concrete.shearRule == ShearRule::Coulomb);
  CHECK(concrete.friction == 0.7);
  CHECK(concrete.variation == 0.30);
  CHECK(concrete.chunkSize == 0.5);
  CHECK(sameMaterial(districtMaterial(materials, DistrictMaterial::Brick), talus::destruction::brickMasonry()));
  CHECK(sameMaterial(districtMaterial(materials, DistrictMaterial::Timber), talus::destruction::timber()));
  CHECK(sameMaterial(districtMaterial(materials, DistrictMaterial::Glass), tiles));
  CHECK(sameMaterial(districtMaterial(materials, DistrictMaterial::Steel), talus::destruction::steel()));

  for (const StructuralMaterial& material : materials) {
    CHECK_NOTHROW(talus::destruction::validate(material));
  }
}

namespace {

using talus::destruction::BuildingPlan;
using talus::destruction::BuildingType;

constexpr std::array<BuildingType, 4> DISTRICT_TYPES{
  BuildingType::ConcreteFrame,
  BuildingType::TimberHouse,
  BuildingType::GlassOffice,
  BuildingType::SteelWarehouse,
};

const char* typeName(BuildingType type) {
  switch (type) {
    case BuildingType::ConcreteFrame:
      return "frame";
    case BuildingType::TimberHouse:
      return "house";
    case BuildingType::GlassOffice:
      return "office";
    case BuildingType::SteelWarehouse:
      return "warehouse";
  }

  return "unknown";
}

struct Members {
  size_t columns = 0u;
  size_t beams = 0u;
  size_t walls = 0u;
  size_t panes = 0u;
  size_t slabs = 0u;
  size_t openings = 0u;
  glm::dvec3 low{1.0e300};
  glm::dvec3 high{-1.0e300};
};

Members membersOf(const Building& building) {
  Members members;

  members.openings = building.openings.size();

  for (const Member& member : building.members) {
    members.columns += member.role == Role::Column ? 1u : 0u;
    members.beams += member.role == Role::Beam ? 1u : 0u;
    members.walls += member.role == Role::Wall ? 1u : 0u;
    members.panes += member.role == Role::Pane ? 1u : 0u;
    members.slabs += member.role == Role::Slab ? 1u : 0u;
    members.low = glm::min(members.low, member.centre - member.halfExtents);
    members.high = glm::max(members.high, member.centre + member.halfExtents);
  }

  return members;
}

}

TEST_CASE("each building type's plan has the members its bays and storeys give", "[destruction][district]") {
  struct Expected {
    BuildingPlan plan;
    Members members;
  };

  const std::array<Expected, 4> expected{{
    {{BuildingType::ConcreteFrame, 3u, 2u, 2u}, {81u, 36u, 24u, 48u, 48u, 49u, glm::dvec3(-4.25, 0.0, -4.25), glm::dvec3(4.25, 9.75, 4.25)}},
    {{BuildingType::TimberHouse, 2u, 4u, 4u}, {100u, 80u, 48u, 62u, 112u, 40u, glm::dvec3(-4.3, 0.0, -4.3), glm::dvec3(4.3, 5.4, 4.3)}},
    {{BuildingType::GlassOffice, 5u, 2u, 1u}, {90u, 35u, 0u, 31u, 45u, 0u, glm::dvec3(-4.25, 0.0, -2.25), glm::dvec3(4.25, 18.75, 2.25)}},
    {{BuildingType::SteelWarehouse, 1u, 4u, 3u}, {20u, 19u, 0u, 0u, 1u, 0u, glm::dvec3(-10.15, 0.0, -7.65), glm::dvec3(10.15, 6.65, 7.65)}},
  }};

  for (const Expected& want : expected) {
    INFO(typeName(want.plan.type));

    const Building building = talus::destruction::makeBuilding(want.plan);
    const Members got = membersOf(building);

    CHECK(got.columns == want.members.columns);
    CHECK(got.beams == want.members.beams);
    CHECK(got.walls == want.members.walls);
    CHECK(got.panes == want.members.panes);
    CHECK(got.slabs == want.members.slabs);
    CHECK(got.openings == want.members.openings);
    CHECK(glm::length(got.low - want.members.low) <= 1.0e-12);
    CHECK(glm::length(got.high - want.members.high) <= 1.0e-12);
    CHECK(building.position == glm::dvec3(0.0));
    CHECK(building.rotation == glm::dquat(1.0, 0.0, 0.0, 0.0));
  }
}

TEST_CASE("every building plan chunks into 1,500 to 4,000 chunks", "[destruction][district]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  for (const BuildingType type : DISTRICT_TYPES) {
    const std::span<const BuildingPlan> plans = talus::destruction::buildingPlans(type);

    REQUIRE(!plans.empty());

    for (const BuildingPlan& plan : plans) {
      INFO(typeName(type) << " storeys " << plan.storeys << " bays " << plan.baysX << " x " << plan.baysZ);

      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);

      CHECK(plan.type == type);
      CHECK(chunked.chunks.size() >= 1500u);
      CHECK(chunked.chunks.size() <= 4000u);
    }
  }
}

TEST_CASE("a building plan the district does not list is refused", "[destruction][district]") {
  CHECK_THROWS_AS(
    talus::destruction::makeBuilding(BuildingPlan{BuildingType::ConcreteFrame, 9u, 2u, 2u}),
    std::invalid_argument
  );
  CHECK_THROWS_AS(
    talus::destruction::makeBuilding(BuildingPlan{BuildingType::SteelWarehouse, 1u, 3u, 3u}),
    std::invalid_argument
  );
  CHECK_THROWS_AS(
    talus::destruction::makeBuilding(BuildingPlan{static_cast<BuildingType>(7), 3u, 2u, 2u}),
    std::invalid_argument
  );
  CHECK(talus::destruction::buildingPlans(static_cast<BuildingType>(7)).empty());
}

namespace {

constexpr uint32_t STAND_UPDATES = 10000u;

size_t floatingChunks(const StructureState& structure) {
  size_t floating = 0u;

  for (uint32_t chunk = 0u; chunk != structure.alive.size(); ++chunk) {
    if (structure.alive[chunk] != 0u && !talus::destruction::isAnchored(structure, structure.component[chunk])) {
      ++floating;
    }
  }

  return floating;
}

struct Standing {
  bool anchored = false;
  bool settled = false;
  uint32_t updates = 0u;
  size_t broken = 0u;
  size_t cracked = 0u;
  size_t floating = 0u;
};

Standing standUp(
  const ChunkedBuilding& chunked,
  StructureState& structure,
  StressState& stress,
  const StressSettings& settings = {}
) {
  Standing standing;

  standing.anchored = floatingChunks(structure) == 0u;

  while (!standing.settled && standing.updates != STAND_UPDATES) {
    const StressReport report = talus::destruction::updateStress(chunked, structure, stress, {}, settings);

    standing.settled = !report.ran;
    standing.updates += report.ran ? 1u : 0u;
    standing.broken += report.broken.size();
    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  }

  standing.floating = floatingChunks(structure);
  standing.cracked = static_cast<size_t>(std::count(structure.cracked.begin(), structure.cracked.end(), uint8_t{1}));

  return standing;
}

Standing standUp(const ChunkedBuilding& chunked, const StressSettings& settings = {}) {
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);

  return standUp(chunked, structure, stress, settings);
}

void checkEveryPlanStands(BuildingType type) {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
    INFO(typeName(type) << " storeys " << plan.storeys << " bays " << plan.baysX << " x " << plan.baysZ);

    const Standing standing = standUp(talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials));

    CHECK(standing.anchored);
    REQUIRE(standing.settled);
    CHECK(standing.broken == 0u);
    CHECK(standing.floating == 0u);
  }
}

double breakingUse(const Bond& bond, double health, const glm::dvec3& force, const glm::dvec3& moment) {
  const BondStress stress = talus::destruction::bondStress(bond, health, force, moment);
  const bool cracks = bond.shearRule == ShearRule::Coulomb && stress.pressure > 0.0;
  const double tension = cracks ? 0.0 : stress.tension / (bond.elastic.tension / bond.area);
  const double compression = stress.compression / (bond.elastic.compression / bond.area);
  double shearLimit = bond.elastic.shear / bond.area;

  if (bond.shearRule == ShearRule::Coulomb) {
    shearLimit = stress.pressure >= 0.0
                   ? shearLimit + bond.friction * stress.pressure
                   : shearLimit * std::max(0.0, 1.0 + stress.pressure / (bond.elastic.tension / bond.area));
  }

  const double shear = shearLimit > 0.0 ? stress.shear / shearLimit : 1.0e300;

  return std::max(tension, std::max(compression, shear));
}

}

TEST_CASE("a concrete frame of every plan stands under its own weight with every chunk anchored and no broken bond", "[destruction][district]") {
  checkEveryPlanStands(BuildingType::ConcreteFrame);
}

TEST_CASE("a timber house of every plan stands under its own weight with every chunk anchored and no broken bond", "[destruction][district]") {
  checkEveryPlanStands(BuildingType::TimberHouse);
}

TEST_CASE("a glass office of every plan stands under its own weight with every chunk anchored and no broken bond", "[destruction][district]") {
  checkEveryPlanStands(BuildingType::GlassOffice);
}

TEST_CASE("a steel warehouse of every plan stands under its own weight with every chunk anchored and no broken bond", "[destruction][district]") {
  checkEveryPlanStands(BuildingType::SteelWarehouse);
}

TEST_CASE("district stand probe", "[.][districtstand]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  ChunkerSettings mean;

  mean.variety = false;

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      const Building building = talus::destruction::makeBuilding(plan);
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials);
      const auto began = std::chrono::steady_clock::now();
      const Standing standing = standUp(chunked);
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      const ChunkedBuilding average = talus::destruction::chunkBuilding(building, materials, mean);
      StructureState structure = talus::destruction::makeStructure(average);
      StressState stress = talus::destruction::makeStress(average);
      const Standing settled = standUp(average, structure, stress);
      double worst = 0.0;

      for (size_t index = 0u; index != average.bonds.size(); ++index) {
        if (structure.broken[index] == 0u && structure.cracked[index] == 0u) {
          worst = std::max(
            worst,
            breakingUse(average.bonds[index], structure.health[index], stress.force[index], stress.moment[index])
          );
        }
      }

      std::printf(
        "districtstand %s storeys %u bays %ux%u chunks %zu bonds %zu updates %u broken %zu cracked %zu floating %zu "
        "seconds %.2f mean broken %zu cracked %zu worst %.3f\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        chunked.chunks.size(),
        chunked.bonds.size(),
        standing.updates,
        standing.broken,
        standing.cracked,
        standing.floating,
        seconds,
        settled.broken,
        settled.cracked,
        worst
      );
    }
  }
}

namespace {

using talus::destruction::District;
using talus::destruction::DistrictCounts;
using talus::destruction::GroundBox;
using talus::destruction::Prop;

constexpr uint64_t DISTRICT_SEED = 2026u;
constexpr uint64_t LAST_SEED = 0xFFFFFFFFFFFFFFFFull;

bool sameChunked(const ChunkedBuilding& a, const ChunkedBuilding& b) {
  if (a.chunks.size() != b.chunks.size() || a.bonds.size() != b.bonds.size()) {
    return false;
  }

  for (size_t index = 0u; index != a.chunks.size(); ++index) {
    const Chunk& p = a.chunks[index];
    const Chunk& q = b.chunks[index];

    if (p.centre != q.centre || p.halfExtents != q.halfExtents || p.member != q.member || p.material != q.material
        || p.volume != q.volume || p.mass != q.mass || p.anchored != q.anchored) {
      return false;
    }
  }

  for (size_t index = 0u; index != a.bonds.size(); ++index) {
    const Bond& p = a.bonds[index];
    const Bond& q = b.bonds[index];

    if (p.first != q.first || p.second != q.second || p.centroid != q.centroid || p.normal != q.normal
        || p.area != q.area || p.wu != q.wu || p.wv != q.wv || !sameModes(p.elastic, q.elastic)
        || !sameModes(p.fatal, q.fatal) || p.shearRule != q.shearRule || p.friction != q.friction) {
      return false;
    }
  }

  return true;
}

uint64_t mixWord(uint64_t hash, uint64_t word) {
  for (int byte = 0; byte != 8; ++byte) {
    hash ^= (word >> (8 * byte)) & 0xFFu;
    hash *= 1099511628211ull;
  }

  return hash;
}

uint64_t mixVector(uint64_t hash, const glm::dvec3& value) {
  for (int axis = 0; axis != 3; ++axis) {
    hash = mixBits(hash, value[axis]);
  }

  return hash;
}

uint64_t mixRotation(uint64_t hash, const glm::dquat& rotation) {
  hash = mixBits(hash, rotation.w);
  hash = mixBits(hash, rotation.x);
  hash = mixBits(hash, rotation.y);

  return mixBits(hash, rotation.z);
}

uint64_t mixModes(uint64_t hash, const ModeLimits& limits) {
  hash = mixBits(hash, limits.compression);
  hash = mixBits(hash, limits.tension);

  return mixBits(hash, limits.shear);
}

uint64_t districtHash(const District& district) {
  uint64_t hash = 1469598103934665603ull;

  for (const StructuralMaterial& material : district.materials) {
    hash = mixBits(hash, material.density);
    hash = mixModes(hash, material.elastic);
    hash = mixModes(hash, material.fatal);
    hash = mixBits(hash, material.chunkSize);
  }

  for (size_t index = 0u; index != district.buildings.size(); ++index) {
    const BuildingPlan& plan = district.plans[index];
    const Building& building = district.buildings[index];

    hash = mixWord(hash, static_cast<uint64_t>(plan.type));
    hash = mixWord(hash, plan.storeys);
    hash = mixWord(hash, plan.baysX);
    hash = mixWord(hash, plan.baysZ);
    hash = mixVector(hash, building.position);
    hash = mixRotation(hash, building.rotation);

    for (const Member& member : building.members) {
      hash = mixVector(hash, member.centre);
      hash = mixVector(hash, member.halfExtents);
      hash = mixWord(hash, member.material);
      hash = mixWord(hash, static_cast<uint64_t>(member.role));
    }

    for (const Opening& opening : building.openings) {
      hash = mixVector(hash, opening.centre);
      hash = mixVector(hash, opening.halfExtents);
    }
  }

  for (const GroundBox& ground : district.ground) {
    hash = mixVector(hash, ground.centre);
    hash = mixVector(hash, ground.halfExtents);
    hash = mixWord(hash, static_cast<uint64_t>(ground.surface));
  }

  for (const Prop& prop : district.props) {
    hash = mixVector(hash, prop.centre);
    hash = mixVector(hash, prop.halfExtents);
    hash = mixRotation(hash, prop.rotation);
    hash = mixBits(hash, prop.density);
    hash = mixWord(hash, static_cast<uint64_t>(prop.kind));
  }

  return hash;
}

struct Bounds {
  glm::dvec3 low{0.0};
  glm::dvec3 high{0.0};
};

Bounds boundsOf(const glm::dvec3& centre, const glm::dvec3& halfExtents, const glm::dquat& rotation) {
  const glm::dmat3 turn = glm::mat3_cast(rotation);
  glm::dvec3 reach(0.0);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      reach[row] += std::abs(turn[column][row]) * halfExtents[column];
    }
  }

  return Bounds{centre - reach, centre + reach};
}

Bounds boundsOf(const Building& building) {
  Bounds bounds{glm::dvec3(1.0e300), glm::dvec3(-1.0e300)};

  for (const Member& member : building.members) {
    const Bounds part = boundsOf(building.position + building.rotation * member.centre, member.halfExtents, building.rotation);

    bounds.low = glm::min(bounds.low, part.low);
    bounds.high = glm::max(bounds.high, part.high);
  }

  return bounds;
}

double separation(const Bounds& a, const Bounds& b) {
  double gap = -1.0e300;

  for (int axis = 0; axis != 3; ++axis) {
    gap = std::max(gap, std::max(b.low[axis] - a.high[axis], a.low[axis] - b.high[axis]));
  }

  return gap;
}

double groundTop(const District& district, const glm::dvec3& point) {
  double top = -1.0e300;

  for (const GroundBox& ground : district.ground) {
    if (std::abs(point.x - ground.centre.x) <= ground.halfExtents.x
        && std::abs(point.z - ground.centre.z) <= ground.halfExtents.z) {
      top = std::max(top, ground.centre.y + ground.halfExtents.y);
    }
  }

  return top;
}

}

TEST_CASE("the default district holds 10 to 20 buildings of all four types and a few thousand props", "[destruction][district]") {
  const DistrictCounts counts;
  const District district = talus::destruction::makeDistrict(DISTRICT_SEED, counts);

  CHECK(counts.buildings >= 10u);
  CHECK(counts.buildings <= 20u);
  CHECK(counts.props >= 2000u);
  CHECK(counts.props <= 5000u);
  CHECK(district.buildings.size() == counts.buildings);
  CHECK(district.plans.size() == counts.buildings);
  CHECK(district.props.size() == counts.props);
  CHECK(district.ground.size() == 7u);
  CHECK(district.materials.size() == talus::destruction::districtMaterialCount);

  for (const BuildingType type : DISTRICT_TYPES) {
    INFO(typeName(type));
    CHECK(std::count_if(district.plans.begin(), district.plans.end(), [type](const BuildingPlan& plan) {
      return plan.type == type;
    }) >= 1);
  }
}

TEST_CASE("the generator refuses counts it cannot meet", "[destruction][district]") {
  CHECK_THROWS_AS(talus::destruction::propCapacity(talus::destruction::districtBuildings + 1u), std::invalid_argument);
  CHECK_THROWS_AS(
    talus::destruction::makeDistrict(DISTRICT_SEED, DistrictCounts{talus::destruction::districtBuildings + 1u, 0u}),
    std::invalid_argument
  );

  for (const uint32_t buildings : {0u, 10u, 16u, talus::destruction::districtBuildings}) {
    INFO("buildings " << buildings);

    const uint32_t capacity = talus::destruction::propCapacity(buildings);

    CHECK(capacity >= DistrictCounts{}.props);
    CHECK_THROWS_AS(
      talus::destruction::makeDistrict(DISTRICT_SEED, DistrictCounts{buildings, capacity + 1u}),
      std::invalid_argument
    );

    const District full = talus::destruction::makeDistrict(DISTRICT_SEED, DistrictCounts{buildings, capacity});

    CHECK(full.buildings.size() == buildings);
    CHECK(full.props.size() == capacity);
  }
}

TEST_CASE("the same seed gives a bit-identical district and another seed a different one", "[destruction][district]") {
  const uint64_t first = districtHash(talus::destruction::makeDistrict(DISTRICT_SEED));
  const uint64_t again = districtHash(talus::destruction::makeDistrict(DISTRICT_SEED));
  const uint64_t other = districtHash(talus::destruction::makeDistrict(DISTRICT_SEED + 1u));

  CHECK(first == again);
  CHECK(first != other);
}

TEST_CASE("a district building chunks exactly as its plan does on its own, wherever the district puts it", "[destruction][district]") {
  const District district = talus::destruction::makeDistrict(DISTRICT_SEED);

  REQUIRE(district.plans.size() == district.buildings.size());

  for (size_t index = 0u; index != district.buildings.size(); ++index) {
    INFO("building " << index);

    const ChunkedBuilding placed = talus::destruction::chunkBuilding(district.buildings[index], district.materials);
    const ChunkedBuilding alone = talus::destruction::chunkBuilding(
      talus::destruction::makeBuilding(district.plans[index]),
      district.materials
    );

    CHECK(sameChunked(placed, alone));
  }
}

TEST_CASE("props overlap nothing and rest on their ground, and buildings stand 3 m apart on their pavements", "[destruction][district]") {
  const std::array<std::pair<uint64_t, uint32_t>, 3> cases{{
    {0u, talus::destruction::districtBuildings},
    {DISTRICT_SEED, 10u},
    {LAST_SEED, talus::destruction::districtBuildings},
  }};

  for (const auto& [seed, buildings] : cases) {
    INFO("seed " << seed << " buildings " << buildings);

    const District district = talus::destruction::makeDistrict(
      seed,
      DistrictCounts{buildings, talus::destruction::propCapacity(buildings)}
    );
    std::vector<Bounds> houses;
    std::vector<Bounds> props;

    for (const Building& building : district.buildings) {
      const Bounds bounds = boundsOf(building);

      CHECK(building.position.y == 0.15);
      CHECK(near(bounds.low.y, 0.15, 1.0e-12));
      CHECK(groundTop(district, bounds.low) == 0.15);
      CHECK(groundTop(district, bounds.high) == 0.15);
      houses.push_back(bounds);
    }

    for (const Prop& prop : district.props) {
      const Bounds bounds = boundsOf(prop.centre, prop.halfExtents, prop.rotation);

      CHECK(near(bounds.low.y, groundTop(district, prop.centre), 1.0e-12));
      CHECK(prop.density > 0.0);
      props.push_back(bounds);
    }

    size_t crowded = 0u;

    for (size_t a = 0u; a != houses.size(); ++a) {
      for (size_t b = a + 1u; b != houses.size(); ++b) {
        crowded += separation(houses[a], houses[b]) < 3.0 - 1.0e-9 ? 1u : 0u;
      }

      for (const Bounds& prop : props) {
        crowded += separation(houses[a], prop) < 0.0 ? 1u : 0u;
      }
    }

    for (size_t a = 0u; a != props.size(); ++a) {
      for (size_t b = a + 1u; b != props.size(); ++b) {
        crowded += separation(props[a], props[b]) < 0.0 ? 1u : 0u;
      }
    }

    CHECK(crowded == 0u);
  }
}

TEST_CASE("district state hash", "[.][districthash]") {
  const District district = talus::destruction::makeDistrict(DISTRICT_SEED);
  uint64_t chunks = 1469598103934665603ull;
  size_t chunkCount = 0u;
  size_t bondCount = 0u;

  for (const Building& building : district.buildings) {
    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, district.materials);

    chunkCount += chunked.chunks.size();
    bondCount += chunked.bonds.size();

    for (const Chunk& chunk : chunked.chunks) {
      chunks = mixVector(chunks, chunk.centre);
      chunks = mixVector(chunks, chunk.halfExtents);
      chunks = mixBits(chunks, chunk.mass);
    }

    for (const Bond& bond : chunked.bonds) {
      chunks = mixWord(chunks, bond.first);
      chunks = mixWord(chunks, bond.second);
      chunks = mixBits(chunks, bond.area);
      chunks = mixModes(chunks, bond.elastic);
      chunks = mixModes(chunks, bond.fatal);
    }
  }

  std::printf(
    "districthash buildings %zu props %zu chunks %zu bonds %zu district %016llx chunked %016llx\n",
    district.buildings.size(),
    district.props.size(),
    chunkCount,
    bondCount,
    static_cast<unsigned long long>(districtHash(district)),
    static_cast<unsigned long long>(chunks)
  );
}

TEST_CASE("every bond from a wall or a pane to another member is vertical, with the infill on top and nothing standing on a pane", "[destruction][district]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  size_t infill = 0u;

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      INFO(typeName(type) << " storeys " << plan.storeys << " bays " << plan.baysX << " x " << plan.baysZ);

      const Building building = talus::destruction::makeBuilding(plan);
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials);
      size_t carrying = 0u;

      for (const Bond& bond : chunked.bonds) {
        const Chunk& first = chunked.chunks[bond.first];
        const Chunk& second = chunked.chunks[bond.second];
        const bool firstBelow = first.centre.y < second.centre.y;
        const Chunk& lower = firstBelow ? first : second;
        const Chunk& upper = firstBelow ? second : first;
        const Role below = building.members[lower.member].role;
        const Role above = building.members[upper.member].role;
        const bool belowInfill = below == Role::Wall || below == Role::Pane;
        const bool aboveInfill = above == Role::Wall || above == Role::Pane;

        if (first.member == second.member || !(belowInfill || aboveInfill)) {
          continue;
        }

        ++infill;
        carrying += std::abs(bond.normal.y) > 0.5 && aboveInfill && below != Role::Pane ? 0u : 1u;
      }

      CHECK(carrying == 0u);
    }
  }

  CHECK(infill > 0u);
}

namespace {

double storeyHeight(BuildingType type) {
  switch (type) {
    case BuildingType::ConcreteFrame:
      return 3.25;
    case BuildingType::TimberHouse:
      return 2.7;
    case BuildingType::GlassOffice:
      return 3.75;
    case BuildingType::SteelWarehouse:
      return 6.65;
  }

  return 0.0;
}

}

TEST_CASE("a building of more than one storey without its ground-storey columns keeps no chunk above that storey anchored", "[destruction][district]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  size_t buildings = 0u;

  for (const BuildingType type : DISTRICT_TYPES) {
    const double storey = storeyHeight(type);

    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      if (plan.storeys < 2u) {
        continue;
      }

      INFO(typeName(type) << " storeys " << plan.storeys << " bays " << plan.baysX << " x " << plan.baysZ);
      ++buildings;

      const Building building = talus::destruction::makeBuilding(plan);
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials);
      StructureState structure = talus::destruction::makeStructure(chunked);
      size_t removed = 0u;

      for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
        const Member& member = building.members[chunked.chunks[index].member];

        if (member.role == Role::Column && member.centre.y + member.halfExtents.y <= storey + 1.0e-9) {
          talus::destruction::removeChunk(structure, chunked, index);
          ++removed;
        }
      }

      static_cast<void>(talus::destruction::updateComponents(structure, chunked));

      size_t above = 0u;
      size_t held = 0u;

      for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
        if (structure.alive[index] != 0u && chunked.chunks[index].centre.y > storey) {
          ++above;
          held += talus::destruction::isAnchored(structure, structure.component[index]) ? 1u : 0u;
        }
      }

      CHECK(removed > 0u);
      CHECK(above > 0u);
      CHECK(held == 0u);
    }
  }

  CHECK(buildings == 12u);
}

TEST_CASE("every bond between two chunks of one house post carries timber's along-grain limits", "[destruction][district]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  const StructuralMaterial timber = talus::destruction::timber();

  for (const BuildingPlan& plan : talus::destruction::buildingPlans(BuildingType::TimberHouse)) {
    INFO("house storeys " << plan.storeys << " bays " << plan.baysX << " x " << plan.baysZ);

    const Building building = talus::destruction::makeBuilding(plan);
    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());
    size_t posts = 0u;
    size_t across = 0u;

    for (const Bond& bond : chunked.bonds) {
      const Member& a = building.members[chunked.chunks[bond.first].member];
      const Member& b = building.members[chunked.chunks[bond.second].member];

      if (a.role != Role::Column || b.role != Role::Column || a.centre.x != b.centre.x || a.centre.z != b.centre.z) {
        continue;
      }

      ++posts;
      across += sameLimits(bond.elastic, timber.elastic, bond.area) && sameLimits(bond.fatal, timber.fatal, bond.area) ? 0u : 1u;
    }

    CHECK(posts > 0u);
    CHECK(across == 0u);
  }
}

TEST_CASE("district settle probe", "[.][districtsettle]") {
  const District district = talus::destruction::makeDistrict(DISTRICT_SEED);
  for (const auto& [name, settings] : {std::pair<const char*, StressSettings>{"unlimited", StressSettings{}}, std::pair<const char*, StressSettings>{"deferred", deferred()}}) {
    size_t chunks = 0u;
    size_t bonds = 0u;
    uint32_t updates = 0u;
    size_t broken = 0u;
    size_t cracked = 0u;
    size_t floating = 0u;
    double total = 0.0;

    for (size_t index = 0u; index != district.buildings.size(); ++index) {
      const BuildingPlan& plan = district.plans[index];
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(district.buildings[index], district.materials);
      const auto began = std::chrono::steady_clock::now();
      const Standing standing = standUp(chunked, settings);
      const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();

      chunks += chunked.chunks.size();
      bonds += chunked.bonds.size();
      updates += standing.updates;
      broken += standing.broken;
      cracked += standing.cracked;
      floating += standing.floating;
      total += seconds;
      std::printf(
        "districtsettle %s building %zu %s storeys %u bays %ux%u chunks %zu updates %u broken %zu cracked %zu floating %zu seconds %.2f\n",
        name,
        index,
        typeName(plan.type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        chunked.chunks.size(),
        standing.updates,
        standing.broken,
        standing.cracked,
        standing.floating,
        seconds
      );
    }

    std::printf(
      "districtsettle %s total chunks %zu bonds %zu updates %u broken %zu cracked %zu floating %zu seconds %.2f\n",
      name,
      chunks,
      bonds,
      updates,
      broken,
      cracked,
      floating,
      total
    );
  }
}

TEST_CASE("stress pattern bench", "[.][patternbench]") {
  const auto timedBuilds = [](int32_t columns, const std::vector<std::array<int32_t, 2>>& links, uint32_t repeats) {
    std::vector<double> samples;

    for (uint32_t repeat = 0u; repeat != repeats; ++repeat) {
      const auto began = std::chrono::steady_clock::now();
      const talus::destruction::BlockPattern pattern = talus::destruction::blockPattern(columns, links);

      samples.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - began).count());
      REQUIRE(pattern.start.size() == static_cast<size_t>(columns) + 1u);
    }

    std::sort(samples.begin(), samples.end());

    return samples[samples.size() / 2u];
  };

  for (const int32_t columns : {1, 2, 4, 8, 32, 128}) {
    std::vector<std::array<int32_t, 2>> links;

    for (int32_t column = 0; column + 1 < columns; ++column) {
      links.push_back({column, column + 1});

      if (column + 3 < columns) {
        links.push_back({column, column + 3});
      }
    }

    std::printf("patternbench piece columns %d links %zu us %.2f\n", columns, links.size(), timedBuilds(columns, links, 2001u));
  }

  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  for (const BuildingType type : DISTRICT_TYPES) {
    const BuildingPlan& plan = talus::destruction::buildingPlans(type).front();
    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
    std::vector<int32_t> dof(chunked.chunks.size(), -1);
    int32_t columns = 0;

    for (size_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
      if (!chunked.chunks[chunk].anchored && chunked.chunks[chunk].mass > 0.0) {
        dof[chunk] = columns++;
      }
    }

    std::vector<std::array<int32_t, 2>> links;

    for (const Bond& bond : chunked.bonds) {
      if (dof[bond.first] >= 0 && dof[bond.second] >= 0) {
        links.push_back({dof[bond.first], dof[bond.second]});
      }
    }

    std::printf("patternbench %s columns %d links %zu us %.1f\n", typeName(type), columns, links.size(), timedBuilds(columns, links, 21u));
  }
}

TEST_CASE("stress factor quick bench", "[.][factorquick]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers wide = talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);

  for (const talus::destruction::BuildingPlan& plan : {
         talus::destruction::BuildingPlan{talus::destruction::BuildingType::ConcreteFrame, 3u, 2u, 2u},
         talus::destruction::BuildingPlan{talus::destruction::BuildingType::TimberHouse, 3u, 5u, 4u},
         talus::destruction::BuildingPlan{talus::destruction::BuildingType::SteelWarehouse, 1u, 5u, 3u},
       }) {
    const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
    std::vector<int32_t> dof(chunked.chunks.size(), -1);
    int32_t columns = 0;

    for (size_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
      if (!chunked.chunks[chunk].anchored && chunked.chunks[chunk].mass > 0.0) {
        dof[chunk] = columns++;
      }
    }

    std::vector<std::array<int32_t, 2>> links;

    for (const Bond& bond : chunked.bonds) {
      if (dof[bond.first] >= 0 && dof[bond.second] >= 0) {
        links.push_back({dof[bond.first], dof[bond.second]});
      }
    }

    const talus::destruction::BlockPattern pattern = talus::destruction::blockPattern(columns, links);
    std::vector<double> values(static_cast<size_t>(pattern.panel.back()), 0.0);
    std::vector<int32_t> degree(static_cast<size_t>(columns), 0);
    std::vector<int32_t> all(static_cast<size_t>(columns));

    for (const std::array<int32_t, 2>& link : links) {
      degree[static_cast<size_t>(link[0])] += 1;
      degree[static_cast<size_t>(link[1])] += 1;
    }

    for (int32_t column = 0; column != columns; ++column) {
      talus::destruction::MatrixBlock block{};

      for (int32_t axis = 0; axis != 6; ++axis) {
        block[static_cast<size_t>(7 * axis)] = 1.0 + degree[static_cast<size_t>(column)];
      }

      talus::destruction::addBlock(pattern, values, pattern.place[static_cast<size_t>(column)], pattern.place[static_cast<size_t>(column)], block);
      all[static_cast<size_t>(column)] = column;
    }

    for (const std::array<int32_t, 2>& link : links) {
      talus::destruction::MatrixBlock block{};

      for (int32_t axis = 0; axis != 6; ++axis) {
        block[static_cast<size_t>(7 * axis)] = -0.9;
      }

      talus::destruction::addBlock(pattern, values, pattern.place[static_cast<size_t>(link[0])], pattern.place[static_cast<size_t>(link[1])], block);
    }

    std::array<std::vector<double>, 4> samples;
    const talus::dynamics::Workers one;

    for (uint32_t repeat = 0u; repeat != 31u; ++repeat) {
      for (size_t variant = 0u; variant != samples.size(); ++variant) {
        const talus::destruction::Lanes lanes = variant % 2u == 0u ? talus::destruction::Lanes::Two : talus::destruction::Lanes::Widest;
        std::vector<double> factor = values;
        const auto began = std::chrono::steady_clock::now();

        REQUIRE(talus::destruction::factorBlocks(pattern, factor, all, variant < 2u ? one : wide, lanes));
        samples[variant].push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count());
      }
    }

    for (std::vector<double>& variant : samples) {
      std::sort(variant.begin(), variant.end());
    }

    std::printf(
      "factorquick %s columns %d serial two lanes min %.3f median %.3f widest min %.3f median %.3f ms 8 workers two lanes min %.3f "
      "median %.3f widest min %.3f median %.3f ms\n",
      typeName(plan.type),
      columns,
      samples[0].front(),
      samples[0][samples[0].size() / 2u],
      samples[1].front(),
      samples[1][samples[1].size() / 2u],
      samples[2].front(),
      samples[2][samples[2].size() / 2u],
      samples[3].front(),
      samples[3][samples[3].size() / 2u]
    );
  }
}

TEST_CASE("stress factor bench", "[.][factorbench]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  const auto median = [](std::vector<double> samples) {
    std::sort(samples.begin(), samples.end());

    return samples[samples.size() / 2u];
  };
  const auto timed = [](const auto& work) {
    const auto began = std::chrono::steady_clock::now();

    work();

    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  };

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
      std::vector<int32_t> dof(chunked.chunks.size(), -1);
      int32_t columns = 0;

      for (size_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
        if (!chunked.chunks[chunk].anchored && chunked.chunks[chunk].mass > 0.0) {
          dof[chunk] = columns++;
        }
      }

      std::vector<std::array<int32_t, 2>> links;

      for (const Bond& bond : chunked.bonds) {
        if (dof[bond.first] >= 0 && dof[bond.second] >= 0) {
          links.push_back({dof[bond.first], dof[bond.second]});
        }
      }

      std::vector<double> patternMs;
      talus::destruction::BlockPattern pattern;

      for (uint32_t repeat = 0u; repeat != 9u; ++repeat) {
        patternMs.push_back(timed([&] { pattern = talus::destruction::blockPattern(columns, links); }));
      }

      std::vector<double> values(static_cast<size_t>(pattern.panel.back()), 0.0);
      std::vector<int32_t> degree(static_cast<size_t>(columns), 0);

      for (const std::array<int32_t, 2>& link : links) {
        degree[static_cast<size_t>(link[0])] += 1;
        degree[static_cast<size_t>(link[1])] += 1;
      }

      for (int32_t column = 0; column != columns; ++column) {
        talus::destruction::MatrixBlock block{};

        for (int32_t axis = 0; axis != 6; ++axis) {
          block[static_cast<size_t>(7 * axis)] = 1.0 + degree[static_cast<size_t>(column)];

          if (axis != 5) {
            block[static_cast<size_t>(7 * axis + 1)] = 0.1;
            block[static_cast<size_t>(7 * axis + 6)] = 0.1;
          }
        }

        const int32_t place = pattern.place[static_cast<size_t>(column)];

        talus::destruction::addBlock(pattern, values, place, place, block);
      }

      for (const std::array<int32_t, 2>& link : links) {
        talus::destruction::MatrixBlock block{};

        for (int32_t axis = 0; axis != 6; ++axis) {
          block[static_cast<size_t>(7 * axis)] = -0.9;
        }

        talus::destruction::addBlock(
          pattern,
          values,
          pattern.place[static_cast<size_t>(link[0])],
          pattern.place[static_cast<size_t>(link[1])],
          block
        );
      }

      std::vector<double> numericMs;
      std::vector<double> numericWideMs;
      std::vector<double> factor;
      std::vector<int32_t> all(static_cast<size_t>(columns));
      std::shared_ptr<talus::dynamics::Scheduler> scheduler;
      StressSettings wide;

      wide.workers = talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);

      for (int32_t column = 0; column != columns; ++column) {
        all[static_cast<size_t>(column)] = column;
      }

      for (uint32_t repeat = 0u; repeat != 9u; ++repeat) {
        factor = values;
        numericWideMs.push_back(timed([&] { REQUIRE(talus::destruction::factorBlocks(pattern, factor, all, wide.workers)); }));
        factor = values;
        numericMs.push_back(timed([&] { REQUIRE(!talus::destruction::factorBlocks(pattern, factor).has_value()); }));
      }

      std::vector<double> solveMs;
      std::vector<double> rhs(6u * static_cast<size_t>(columns), 1.0);

      for (uint32_t repeat = 0u; repeat != 15u; ++repeat) {
        std::vector<double> x = rhs;

        solveMs.push_back(timed([&] { talus::destruction::solveBlocks(pattern, factor, x); }));
      }

      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);

      static_cast<void>(standUp(chunked, structure, stress));

      uint32_t loaded = 0u;

      while (loaded != chunked.chunks.size() && chunked.chunks[loaded].anchored) {
        ++loaded;
      }

      std::vector<double> crackMs;
      std::vector<double> crackWideMs;
      std::vector<double> resolveMs;
      StructureState twin = structure;
      StressState twinStress = stress;

      for (uint32_t repeat = 0u; repeat != 9u; ++repeat) {
        const std::vector<ChunkLoad> push{ChunkLoad{loaded, glm::dvec3(1.0e3 + repeat, 0.0, 0.0), glm::dvec3(0.0)}};
        const uint32_t bond = repeat * static_cast<uint32_t>(chunked.bonds.size()) / 9u;

        talus::destruction::crackBond(structure, bond);
        talus::destruction::crackBond(twin, bond);
        crackMs.push_back(timed([&] { talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{}); }));
        crackWideMs.push_back(timed([&] { talus::destruction::solveStress(chunked, twin, twinStress, push, wide); }));
      }

      for (uint32_t repeat = 0u; repeat != 15u; ++repeat) {
        const std::vector<ChunkLoad> push{ChunkLoad{loaded, glm::dvec3(2.0e3 + repeat, 0.0, 0.0), glm::dvec3(0.0)}};

        resolveMs.push_back(timed([&] { talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{}); }));
      }

      std::printf(
        "factorbench %s storeys %u bays %ux%u columns %d pattern ms %.3f numeric ms %.3f numeric 8 ms %.3f solve ms %.3f "
        "crack round ms %.3f crack round 8 ms %.3f re-solve ms %.3f\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        columns,
        median(patternMs),
        median(numericMs),
        median(numericWideMs),
        median(solveMs),
        median(crackMs),
        median(crackWideMs),
        median(resolveMs)
      );
    }
  }
}

TEST_CASE("stress update cost probe", "[.][stressupdatecost]") {
  for (const bool free : {false, true}) {
    for (const uint32_t chunks : {1u, 2u, 4u, 8u, 16u, 32u}) {
      ChunkedBuilding piece = unbreakable(chain(chunks, glm::dvec3(0.0, 1.0, 0.0), false));

      if (free) {
        piece.chunks[0].anchored = false;
      }

      StructureState structure = talus::destruction::makeStructure(piece);
      StressState stress = talus::destruction::makeStress(piece);
      const StressSettings settings;
      const uint32_t calls = 20000u;

      static_cast<void>(talus::destruction::updateStress(piece, structure, stress, {}, settings));

      const auto began = std::chrono::steady_clock::now();
      uint32_t ran = 0u;

      for (uint32_t call = 0u; call != calls; ++call) {
        const std::vector<ChunkLoad> load{
          ChunkLoad{chunks, glm::dvec3(10.0 + (call & 1u), -100.0, 0.0), glm::dvec3(0.0)},
          ChunkLoad{chunks / 2u, glm::dvec3(0.0, -50.0, 1.0 + (call & 1u)), glm::dvec3(0.0)},
        };

        ran += talus::destruction::updateStress(piece, structure, stress, load, settings).ran ? 1u : 0u;
      }

      const double micro = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - began).count();

      std::printf(
        "stressupdatecost %s chunks %u bonds %zu calls %u ran %u us per update %.2f\n",
        free ? "free" : "anchored",
        chunks + 1u,
        piece.bonds.size(),
        calls,
        ran,
        micro / calls
      );
    }
  }
}

TEST_CASE("stress factor shape probe", "[.][factorshape]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
      std::vector<int32_t> dof(chunked.chunks.size(), -1);
      int32_t columns = 0;

      for (size_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
        if (!chunked.chunks[chunk].anchored && chunked.chunks[chunk].mass > 0.0) {
          dof[chunk] = columns++;
        }
      }

      std::vector<std::array<int32_t, 2>> links;

      for (const Bond& bond : chunked.bonds) {
        if (dof[bond.first] >= 0 && dof[bond.second] >= 0) {
          links.push_back({dof[bond.first], dof[bond.second]});
        }
      }

      const auto began = std::chrono::steady_clock::now();
      const talus::destruction::BlockPattern pattern = talus::destruction::blockPattern(columns, links);
      const double patternMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
      const size_t n = static_cast<size_t>(columns);
      std::vector<int32_t> parent(n, -1);
      std::vector<int32_t> children(n, 0);
      std::vector<double> work(n, 0.0);
      double total = 0.0;
      int32_t widest = 0;

      for (size_t k = 0u; k != n; ++k) {
        const int32_t first = pattern.start[k];
        const int32_t below = pattern.start[k + 1u] - first - 1;

        widest = std::max(widest, below);
        work[k] += 108.0 * below + 36.0;

        if (below > 0) {
          parent[k] = pattern.rows[static_cast<size_t>(first + 1)];
          children[static_cast<size_t>(parent[k])] += 1;
        }

        for (int32_t t = 1; t <= below; ++t) {
          work[static_cast<size_t>(pattern.rows[static_cast<size_t>(first + t)])] += 216.0 * (below - t + 1);
        }
      }

      for (const double w : work) {
        total += w;
      }

      std::vector<double> path(n, 0.0);
      double critical = 0.0;

      for (size_t j = 0u; j != n; ++j) {
        path[j] += work[j];
        critical = std::max(critical, path[j]);

        if (parent[j] >= 0) {
          path[static_cast<size_t>(parent[j])] = std::max(path[static_cast<size_t>(parent[j])], path[j]);
        }
      }

      const auto affected = [&](std::span<const size_t> bonds) {
        std::vector<uint8_t> mark(n, 0u);
        double sum = 0.0;

        for (const size_t bond : bonds) {
          for (const uint32_t chunk : {chunked.bonds[bond].first, chunked.bonds[bond].second}) {
            if (dof[chunk] < 0) {
              continue;
            }

            for (int32_t j = pattern.place[static_cast<size_t>(dof[chunk])]; j >= 0 && mark[static_cast<size_t>(j)] == 0u;
                 j = parent[static_cast<size_t>(j)]) {
              mark[static_cast<size_t>(j)] = 1u;
              sum += work[static_cast<size_t>(j)];
            }
          }
        }

        return sum / total;
      };

      std::vector<size_t> spread;
      double single = 0.0;
      double worstSingle = 0.0;

      for (size_t at = 0u; at != 20u; ++at) {
        spread.push_back(at * chunked.bonds.size() / 20u);

        const size_t one[] = {spread.back()};
        const double share = affected(one);

        single += share / 20.0;
        worstSingle = std::max(worstSingle, share);
      }

      std::vector<double> subtree(work);
      std::vector<std::vector<int32_t>> kids(n);
      std::vector<int32_t> depth(n, 1);
      int32_t height = 0;

      for (size_t j = 0u; j != n; ++j) {
        if (parent[j] >= 0) {
          subtree[static_cast<size_t>(parent[j])] += subtree[j];
          kids[static_cast<size_t>(parent[j])].push_back(static_cast<int32_t>(j));
          depth[static_cast<size_t>(parent[j])] = std::max(depth[static_cast<size_t>(parent[j])], depth[j] + 1);
        }
      }

      for (size_t j = 0u; j != n; ++j) {
        height = std::max(height, depth[j]);
      }

      std::vector<int32_t> cut;
      std::vector<int32_t> top;

      for (size_t j = 0u; j != n; ++j) {
        if (parent[j] < 0) {
          cut.push_back(static_cast<int32_t>(j));
        }
      }

      for (;;) {
        const auto largest = std::max_element(cut.begin(), cut.end(), [&](int32_t a, int32_t b) {
          return subtree[static_cast<size_t>(a)] < subtree[static_cast<size_t>(b)];
        });

        if (largest == cut.end() || subtree[static_cast<size_t>(*largest)] <= total / 32.0
            || kids[static_cast<size_t>(*largest)].empty()) {
          break;
        }

        const int32_t split = *largest;

        cut.erase(largest);
        top.push_back(split);
        cut.insert(cut.end(), kids[static_cast<size_t>(split)].begin(), kids[static_cast<size_t>(split)].end());
      }

      double topWork = 0.0;
      double largestCut = 0.0;
      size_t heavyTop = 0u;
      std::vector<int32_t> topDepth(n, 0);
      int32_t topHeight = 0;

      std::sort(top.begin(), top.end());

      for (const int32_t j : top) {
        topWork += work[static_cast<size_t>(j)];
        heavyTop += work[static_cast<size_t>(j)] > 50000.0 ? 1u : 0u;
        topDepth[static_cast<size_t>(j)] = std::max(topDepth[static_cast<size_t>(j)], 1);
        topHeight = std::max(topHeight, topDepth[static_cast<size_t>(j)]);

        if (parent[static_cast<size_t>(j)] >= 0) {
          topDepth[static_cast<size_t>(parent[static_cast<size_t>(j)])] =
            std::max(topDepth[static_cast<size_t>(parent[static_cast<size_t>(j)])], topDepth[static_cast<size_t>(j)] + 1);
        }
      }

      for (const int32_t root : cut) {
        largestCut = std::max(largestCut, subtree[static_cast<size_t>(root)]);
      }

      std::vector<int32_t> topNodes;

      for (const int32_t j : top) {
        topNodes.push_back(pattern.nodeOf[static_cast<size_t>(j)]);
      }

      std::sort(topNodes.begin(), topNodes.end());
      topNodes.erase(std::unique(topNodes.begin(), topNodes.end()), topNodes.end());

      std::printf(
        "factorshape %s storeys %u bays %ux%u supernodes %zu top supernodes %zu\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        pattern.nodeStart.size() - 1u,
        topNodes.size()
      );

      std::printf(
        "factorshape %s storeys %u bays %ux%u height %d subtrees %zu largest share %.3f top columns %zu top share %.3f "
        "top height %d heavy top columns %zu\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        height,
        cut.size(),
        largestCut / total,
        top.size(),
        topWork / total,
        topHeight,
        heavyTop
      );

      size_t supernodes = 0u;
      size_t wide = 0u;
      int32_t widestNode = 0;
      int32_t run = 0;

      for (size_t j = 0u; j != n; ++j) {
        const bool joins = j > 0u && parent[j - 1u] == static_cast<int32_t>(j) && children[j] == 1
                           && pattern.start[j] - pattern.start[j - 1u] - 2 == pattern.start[j + 1u] - pattern.start[j] - 1;

        if (joins) {
          ++run;
        } else {
          ++supernodes;
          wide += run >= 1 ? 1u : 0u;
          run = 0;
        }

        widestNode = std::max(widestNode, run + 1);
      }

      std::printf(
        "factorshape %s storeys %u bays %ux%u columns %d blocks %d pattern ms %.2f mflop %.1f critical share %.3f "
        "widest column %d supernodes %zu of them wider than one %zu widest supernode %d one crack share %.3f worst %.3f "
        "twenty cracks share %.3f\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        columns,
        pattern.start.back(),
        patternMs,
        2.0 * total / 1.0e6,
        critical / total,
        widest,
        supernodes,
        wide,
        widestNode,
        single,
        worstSingle,
        affected(spread)
      );
    }
  }
}

TEST_CASE("stress factor probe", "[.][stressfactor]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();
  const auto millisecondsSince = [](std::chrono::steady_clock::time_point began) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  };

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
      StructureState structure = talus::destruction::makeStructure(chunked);
      StressState stress = talus::destruction::makeStress(chunked);
      const auto settling = std::chrono::steady_clock::now();
      const Standing standing = standUp(chunked, structure, stress);
      const double settle = millisecondsSince(settling);
      uint32_t loaded = 0u;

      while (loaded != chunked.chunks.size() && chunked.chunks[loaded].anchored) {
        ++loaded;
      }

      const std::vector<ChunkLoad> push{ChunkLoad{loaded, glm::dvec3(1.0e3, 0.0, 0.0), glm::dvec3(0.0)}};
      const auto solving = std::chrono::steady_clock::now();

      talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{});

      const double solve = millisecondsSince(solving);

      talus::destruction::crackBond(structure, 0u);

      const auto refactoring = std::chrono::steady_clock::now();

      talus::destruction::solveStress(chunked, structure, stress, push, StressSettings{});

      const double refactor = millisecondsSince(refactoring);

      std::printf(
        "stressfactor %s storeys %u bays %ux%u chunks %zu bonds %zu updates %u cracked %zu broken %zu settle ms %.1f "
        "solve ms %.2f factorize and solve ms %.1f factor MB %.2f\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        chunked.chunks.size(),
        chunked.bonds.size(),
        standing.updates,
        standing.cracked,
        standing.broken,
        settle,
        solve,
        refactor,
        static_cast<double>(talus::destruction::factorBytes(stress)) / 1.0e6
      );
    }
  }
}

TEST_CASE("stress shrink probe", "[.][stressshrink]") {
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(
    talus::destruction::makeBuilding(BuildingPlan{BuildingType::ConcreteFrame, 3u, 2u, 2u}),
    talus::destruction::districtMaterials()
  );
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const Standing standing = standUp(chunked, structure, stress);
  const auto millisecondsSince = [](std::chrono::steady_clock::time_point began) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  };
  const auto crackRound = [&]() {
    uint32_t bond = 0u;

    while (bond != chunked.bonds.size() && (structure.broken[bond] != 0u || structure.cracked[bond] != 0u)) {
      ++bond;
    }

    talus::destruction::crackBond(structure, bond);

    const auto began = std::chrono::steady_clock::now();

    talus::destruction::solveStress(chunked, structure, stress, {}, StressSettings{});

    return millisecondsSince(began);
  };
  const auto medianRound = [&]() {
    std::vector<double> times;

    for (uint32_t round = 0u; round != 5u; ++round) {
      times.push_back(crackRound());
    }

    std::sort(times.begin(), times.end());

    return times[2];
  };

  REQUIRE(standing.settled);

  const double whole = medianRound();
  const double wholeMegabytes = static_cast<double>(talus::destruction::factorBytes(stress)) / 1.0e6;
  std::vector<uint32_t> order;

  for (uint32_t index = 0u; index != chunked.chunks.size(); ++index) {
    order.push_back(index);
  }

  std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
    return std::pair(-chunked.chunks[a].centre.y, a) < std::pair(-chunked.chunks[b].centre.y, b);
  });

  const size_t removed = chunked.chunks.size() / 3u;

  for (size_t index = 0u; index != removed; ++index) {
    talus::destruction::removeChunk(structure, chunked, order[index]);
  }

  static_cast<void>(talus::destruction::updateComponents(structure, chunked));

  const auto rebuilding = std::chrono::steady_clock::now();

  talus::destruction::solveStress(chunked, structure, stress, {}, StressSettings{});

  const double rebuild = millisecondsSince(rebuilding);
  const double part = medianRound();

  std::printf(
    "stressshrink frame chunks %zu bonds %zu settled broken %zu cracked %zu crack round ms %.2f factor MB %.2f "
    "removed %zu first solve ms %.2f crack round ms %.2f factor MB %.2f\n",
    chunked.chunks.size(),
    chunked.bonds.size(),
    standing.broken,
    standing.cracked,
    whole,
    wholeMegabytes,
    removed,
    rebuild,
    part,
    static_cast<double>(talus::destruction::factorBytes(stress)) / 1.0e6
  );
}

TEST_CASE("stress worn bond probe", "[.][stressworn]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  const ChunkedBuilding standing = talus::destruction::chunkBuilding(table(glm::dvec3(0.0)), materials, plain());
  const ChunkedBuilding column = chain(4u, glm::dvec3(0.0, 1.0, 0.0), false);
  const uint32_t corner = chunkAt(standing, glm::dvec3(0.75, 1.75, 0.75));
  const uint32_t edge = chunkAt(standing, glm::dvec3(-0.25, 1.75, 0.75));
  const uint32_t leg = bondAt(standing, glm::dvec3(0.75, 1.5, 0.75));
  const std::vector<ChunkLoad> pushed{
    ChunkLoad{corner, glm::dvec3(4.0e3, -9.0e3, 2.5e3), glm::dvec3(0.0)},
    ChunkLoad{edge, glm::dvec3(0.0), glm::dvec3(1.5e3, -2.0e3, 0.8e3)},
  };
  const std::vector<ChunkLoad> top{ChunkLoad{4u, glm::dvec3(0.0, -2.0e3, 0.0), glm::dvec3(0.0)}};
  const auto finite = [](const StressState& stress) {
    for (size_t index = 0u; index != stress.force.size(); ++index) {
      for (int axis = 0; axis != 3; ++axis) {
        if (!std::isfinite(stress.force[index][axis]) || !std::isfinite(stress.moment[index][axis])) {
          return false;
        }
      }
    }

    return true;
  };

  REQUIRE(corner != 0xFFFFFFFFu);
  REQUIRE(edge != 0xFFFFFFFFu);
  REQUIRE(leg != 0xFFFFFFFFu);

  for (const double health : {1.0e-3, 1.0e-6, 1.0e-7, 3.0e-8, 1.5e-8, 1.0e-8, 3.0e-9, 1.0e-9, 1.0e-12, 1.0e-15, 1.0e-18, 1.0e-30, 1.0e-100, 1.0e-300, 5.0e-324}) {
    for (const bool cracked : {false, true}) {
      StructureState structure = talus::destruction::makeStructure(standing);
      StressState stress = talus::destruction::makeStress(standing);

      structure.health[leg] = health;

      if (cracked) {
        talus::destruction::crackBond(structure, leg);
      }

      try {
        talus::destruction::solveStress(standing, structure, stress, pushed, StressSettings{});

        const BondLoads reference = leastNormBondLoads(standing, structure, pushed, StressSettings{}.gravity);
        const Agreement result = agreement(standing, structure, stress, reference);

        std::printf(
          "stressworn table health %g cracked %d finite %d force %.3g moment %.3g worn force %.3g\n",
          health,
          cracked ? 1 : 0,
          finite(stress) ? 1 : 0,
          result.force / result.largestForce,
          result.moment / result.largestMoment,
          glm::length(stress.force[leg])
        );
      } catch (const std::exception& error) {
        std::printf("stressworn table health %g cracked %d threw %s\n", health, cracked ? 1 : 0, error.what());
      }
    }

    StructureState structure = talus::destruction::makeStructure(column);
    StressState stress = talus::destruction::makeStress(column);

    structure.health[1] = health;

    try {
      talus::destruction::solveStress(column, structure, stress, top, StressSettings{});

      const double above = 3.0 * MASS * G + 2.0e3;

      std::printf(
        "stressworn column health %g finite %d worn axial error %.3g\n",
        health,
        finite(stress) ? 1 : 0,
        std::abs(std::abs(glm::dot(stress.force[1], column.bonds[1].normal)) - above) / above
      );
    } catch (const std::exception& error) {
      std::printf("stressworn column health %g threw %s\n", health, error.what());
    }
  }
}

TEST_CASE("stress endless wear probe", "[.][stresswear]") {
  struct Wear {
    const char* name;
    uint32_t firm;
    double tension;
    double fatal;
  };

  for (const Wear& wear : {
         Wear{"three bonds", 2u, 2.5e6, 0.0},
         Wear{"three bonds", 2u, 3.5e6, 0.0},
         Wear{"three bonds", 2u, 4.5e6, 0.0},
         Wear{"three bonds", 2u, 6.5e6, 0.0},
         Wear{"three bonds", 2u, 3.5e6, 5.0},
         Wear{"three bonds", 2u, 3.5e6, 11.0},
         Wear{"three bonds", 2u, 3.5e6, 101.0},
         Wear{"two bonds, steel-like", 1u, 2.02e6, 1.47},
         Wear{"two bonds, steel-like", 1u, 2.002e6, 1.47},
         Wear{"two bonds, steel-like", 1u, 2.0002e6, 1.47},
         Wear{"two bonds, steel-like", 1u, 2.00002e6, 1.47},
       }) {
    const std::vector<StructuralMaterial> materials{ductile()};
    ChunkedBuilding chunked = talus::destruction::chunkBuilding(hanging(), materials, plain());

    REQUIRE(chunked.bonds.size() == 1u);

    if (wear.fatal > 0.0) {
      chunked.bonds[0].fatal.tension = wear.fatal * chunked.bonds[0].elastic.tension;
    }

    Bond firm = chunked.bonds[0];

    firm.elastic = ModeLimits{1.0e12, 1.0e12, 1.0e12};
    firm.fatal = firm.elastic;

    for (uint32_t index = 0u; index != wear.firm; ++index) {
      chunked.bonds.push_back(firm);
    }

    StructureState structure = talus::destruction::makeStructure(chunked);
    StressState stress = talus::destruction::makeStress(chunked);
    const double pull = wear.tension * chunked.bonds[0].area - chunked.chunks[1].mass * G;
    const std::vector<ChunkLoad> load{ChunkLoad{1u, glm::dvec3(0.0, -pull, 0.0), glm::dvec3(0.0)}};
    uint32_t calls = 0u;
    uint32_t factorizations = 0u;
    bool settled = false;

    for (; calls != 20000u && !settled; ++calls) {
      const StressReport report = talus::destruction::updateStress(chunked, structure, stress, load, StressSettings{});

      settled = !report.ran;
      factorizations += report.factorizations;
    }

    std::printf(
      "stresswear %s tension %.5f MPa fatal %.3g times elastic calls %u settled %d factorizations %u broken %u health %.9g\n",
      wear.name,
      wear.tension / 1.0e6,
      chunked.bonds[0].fatal.tension / chunked.bonds[0].elastic.tension,
      calls,
      settled ? 1 : 0,
      factorizations,
      static_cast<unsigned>(structure.broken[0]),
      structure.health[0]
    );
  }
}

TEST_CASE("stress erosion cost probe", "[.][stresserosion]") {
  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(house(), houseMaterials());
  StructureState structure = talus::destruction::makeStructure(chunked);
  StressState stress = talus::destruction::makeStress(chunked);
  const StressSettings settings;
  const std::vector<ChunkLoad> hit{
    ChunkLoad{firstChunkOf(chunked, 0u) + 30u, glm::dvec3(0.0, 0.0, 2.0e5), glm::dvec3(0.0)},
    ChunkLoad{firstChunkOf(chunked, 5u) + 3u, glm::dvec3(4.0e6, 0.0, 0.0), glm::dvec3(0.0, 1.0e4, 0.0)},
  };
  uint32_t ran = 0u;
  uint32_t rounds = 0u;
  uint32_t factorizations = 0u;
  uint32_t erosionUpdates = 0u;
  uint32_t erosionFactorizations = 0u;
  size_t broken = 0u;
  bool erodedOnly = false;
  double milliseconds = 0.0;

  for (uint32_t update = 0u; update != 40u; ++update) {
    const bool hitting = update >= 5u && update < 8u;
    const auto began = std::chrono::steady_clock::now();
    const StressReport report = talus::destruction::updateStress(
      chunked,
      structure,
      stress,
      hitting ? std::span<const ChunkLoad>(hit) : std::span<const ChunkLoad>(),
      settings
    );

    milliseconds += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
    ran += report.ran ? 1u : 0u;
    rounds += report.rounds;
    factorizations += report.factorizations;
    broken += report.broken.size();

    if (erodedOnly && report.ran) {
      erosionUpdates += 1u;
      erosionFactorizations += report.factorizations != 0u ? 1u : 0u;
    }

    erodedOnly = stress.eroding && report.broken.empty() && report.cracked.empty();
    static_cast<void>(talus::destruction::updateComponents(structure, chunked));
  }

  std::printf(
    "stresserosion demolition updates run %u rounds %u factorizations %u updates after erosion alone %u "
    "their first-round factorizations %u broken %zu ms %.1f\n",
    ran,
    rounds,
    factorizations,
    erosionUpdates,
    erosionFactorizations,
    broken,
    milliseconds
  );
}

TEST_CASE("stress factor threshold probe", "[.][stressthreshold]") {
  struct Path {
    const char* name;
    ChunkedBuilding chunked;
    uint32_t worn;
    bool cracked;
  };

  const auto narrowed = [](ChunkedBuilding chunked, uint32_t bond) {
    chunked.bonds[bond].area *= 0.01;
    chunked.bonds[bond].wu *= 0.1;
    chunked.bonds[bond].wv *= 0.1;

    return chunked;
  };
  const auto widened = [](ChunkedBuilding chunked, uint32_t bond, double factor) {
    chunked.bonds[bond].area *= factor;
    chunked.bonds[bond].wu *= std::sqrt(factor);
    chunked.bonds[bond].wv *= std::sqrt(factor);

    return chunked;
  };
  const ChunkedBuilding column = chain(4u, glm::dvec3(0.0, 1.0, 0.0), false);
  const ChunkedBuilding tall = chain(16u, glm::dvec3(0.0, 1.0, 0.0), false);
  const ChunkedBuilding arm = chain(4u, glm::dvec3(1.0, 0.0, 0.0), false);

  for (const Path& path : {
         Path{"4-chunk column", column, 1u, false},
         Path{"4-chunk column, cracked", column, 1u, true},
         Path{"4-chunk column, 1/100 area", narrowed(column, 1u), 1u, false},
         Path{"4-chunk column, cracked, 1/100 area", narrowed(column, 1u), 1u, true},
         Path{"16-chunk column, worn low", tall, 1u, false},
         Path{"16-chunk column, worn in the middle", tall, 8u, false},
         Path{"16-chunk column, worn low, cracked", tall, 1u, true},
         Path{"4-chunk cantilever", arm, 1u, false},
         Path{"4-chunk cantilever, cracked", arm, 1u, true},
         Path{"4-chunk column, top bond 100 times the area", widened(column, 3u, 100.0), 1u, false},
         Path{"4-chunk column, top bond 10^4 times the area", widened(column, 3u, 1.0e4), 1u, false},
         Path{"4-chunk column, cracked, top bond 100 times the area", widened(column, 3u, 100.0), 1u, true},
         Path{"16-chunk column, worn low, top bond 100 times the area", widened(tall, 15u, 100.0), 1u, false},
         Path{"4-chunk cantilever, tip bond 100 times the area", widened(arm, 3u, 100.0), 1u, false},
         Path{"64-chunk column, worn low", chain(64u, glm::dvec3(0.0, 1.0, 0.0), false), 1u, false},
         Path{"64-chunk column, worn low, top bond 100 times the area", widened(chain(64u, glm::dvec3(0.0, 1.0, 0.0), false), 63u, 100.0), 1u, false},
         Path{"256-chunk column, worn low, top bond 100 times the area", widened(chain(256u, glm::dvec3(0.0, 1.0, 0.0), false), 255u, 100.0), 1u, false},
         Path{"1024-chunk column, worn low, top bond 100 times the area", widened(chain(1024u, glm::dvec3(0.0, 1.0, 0.0), false), 1023u, 100.0), 1u, false},
         Path{"256-chunk column, worn low, top bond 10^4 times the area", widened(chain(256u, glm::dvec3(0.0, 1.0, 0.0), false), 255u, 1.0e4), 1u, false},
         Path{"16-chunk column, worn low, bond 3 100 times the area", widened(tall, 3u, 100.0), 1u, false},
         Path{"16-chunk cantilever, worn low, tip bond 100 times the area", widened(chain(16u, glm::dvec3(1.0, 0.0, 0.0), false), 15u, 100.0), 1u, false},
       }) {
    const StressState weights = talus::destruction::makeStress(path.chunked);
    const Bond& worn = path.chunked.bonds[path.worn];
    const double crack = path.cracked ? 1.0e-2 : 1.0;
    const double own = std::min(weights.forceShare[path.worn], weights.momentShare[path.worn]) * crack;
    double neighbours = 0.0;
    double stiffest = 0.0;

    for (uint32_t bond = 0u; bond != path.chunked.bonds.size(); ++bond) {
      const Bond& other = path.chunked.bonds[bond];
      const double share = std::max(weights.forceShare[bond], weights.momentShare[bond]);

      stiffest = std::max(stiffest, share);

      if (other.first == worn.first || other.first == worn.second || other.second == worn.first || other.second == worn.second) {
        neighbours = std::max(neighbours, share);
      }
    }

    const uint32_t top = static_cast<uint32_t>(path.chunked.chunks.size() - 1u);
    const std::vector<ChunkLoad> load{ChunkLoad{top, glm::dvec3(0.0, -2.0e3, 0.0), glm::dvec3(0.0)}};
    double failing = 0.0;

    for (double health = 0.1; health > 1.0e-14 && failing == 0.0; health /= std::pow(10.0, 0.125)) {
      StructureState structure = talus::destruction::makeStructure(path.chunked);
      StressState stress = talus::destruction::makeStress(path.chunked);

      if (path.cracked) {
        talus::destruction::crackBond(structure, path.worn);
      }

      structure.health[path.worn] = health;

      try {
        talus::destruction::solveStress(path.chunked, structure, stress, load, StressSettings{});
      } catch (const std::exception&) {
        failing = health;
      }
    }

    std::printf(
      "stressthreshold %s own %.3g neighbours %.3g structure %.3g fails at %.3g ratio %.3g structure ratio %.3g\n",
      path.name,
      own,
      neighbours,
      stiffest,
      failing,
      failing * own / (std::sqrt(std::numeric_limits<double>::epsilon()) * neighbours),
      failing * own / (std::sqrt(std::numeric_limits<double>::epsilon()) * stiffest)
    );
  }
}

TEST_CASE("stress sliver probe", "[.][stresssliver]") {
  for (const bool cracked : {false, true}) {
    double failing = 0.0;
    double last = 0.0;

    for (double ratio = 1.0e-3; ratio > 1.0e-12 && failing == 0.0; ratio /= std::pow(10.0, 0.125)) {
      ChunkedBuilding column = chain(4u, glm::dvec3(0.0, 1.0, 0.0), false);

      column.bonds[1].area *= ratio;
      column.bonds[1].wu *= std::sqrt(ratio);
      column.bonds[1].wv *= std::sqrt(ratio);

      StructureState structure = talus::destruction::makeStructure(column);
      StressState stress = talus::destruction::makeStress(column);

      if (cracked) {
        talus::destruction::crackBond(structure, 1u);
      }

      const std::vector<ChunkLoad> load{ChunkLoad{4u, glm::dvec3(0.0, -2.0e3, 0.0), glm::dvec3(0.0)}};

      try {
        talus::destruction::solveStress(column, structure, stress, load, StressSettings{});
        last = ratio;
      } catch (const std::exception&) {
        failing = ratio;
      }
    }

    std::printf("stresssliver cracked %d fails at area ratio %.3g, last solved %.3g\n", cracked ? 1 : 0, failing, last);
  }
}

TEST_CASE("stress floor spread probe", "[.][stressfloor]") {
  const std::vector<StructuralMaterial> materials = talus::destruction::districtMaterials();

  for (const BuildingType type : DISTRICT_TYPES) {
    for (const BuildingPlan& plan : talus::destruction::buildingPlans(type)) {
      const ChunkedBuilding chunked = talus::destruction::chunkBuilding(talus::destruction::makeBuilding(plan), materials);
      const StressState stress = talus::destruction::makeStress(chunked);
      double stiffest = 0.0;

      for (size_t bond = 0u; bond != chunked.bonds.size(); ++bond) {
        stiffest = std::max(stiffest, std::max(stress.forceShare[bond], stress.momentShare[bond]));
      }

      std::vector<double> floors(chunked.bonds.size());
      size_t largest = 0u;
      size_t widest = 0u;
      size_t crackable = 0u;
      size_t crackableAbove = 0u;
      size_t wearing = 0u;

      for (size_t bond = 0u; bond != chunked.bonds.size(); ++bond) {
        const Bond& b = chunked.bonds[bond];

        floors[bond] = 0x1.0p-21 * stiffest / std::min(stress.forceShare[bond], stress.momentShare[bond]);
        largest = floors[bond] > floors[largest] ? bond : largest;
        widest = b.area > chunked.bonds[widest].area ? bond : widest;
        crackable += b.crackFriction > 0.0 ? 1u : 0u;
        crackableAbove += b.crackFriction > 0.0 && floors[bond] > 1.0e-4 ? 1u : 0u;
        wearing += floors[bond] > 1.0e-4 && b.elastic.compression < b.fatal.compression ? 1u : 0u;
      }

      const Bond& thin = chunked.bonds[largest];

      std::printf(
        "stressfloor %s largest-floor bond area %.3g crack friction %.2f elastic/fatal tension %.3g compression %.3g; "
        "widest bond area %.3g; crackable %zu, of them with a cracked floor above 1e-2 %zu; "
        "bonds with a plain floor above 1e-4 that wear in compression %zu\n",
        typeName(type),
        thin.area,
        thin.crackFriction,
        thin.elastic.tension / thin.fatal.tension,
        thin.elastic.compression / thin.fatal.compression,
        chunked.bonds[widest].area,
        crackable,
        crackableAbove,
        wearing
      );

      std::sort(floors.begin(), floors.end());

      const auto above = [&](double limit) {
        return static_cast<size_t>(floors.end() - std::upper_bound(floors.begin(), floors.end(), limit));
      };

      std::printf(
        "stressfloor %s storeys %u bays %u x %u bonds %zu plain floor median %.3g largest %.3g cracked largest %.3g "
        "cracked above 1e-3 %zu above 1e-2 %zu above 1e-1 %zu\n",
        typeName(type),
        plan.storeys,
        plan.baysX,
        plan.baysZ,
        floors.size(),
        floors[floors.size() / 2u],
        floors.back(),
        floors.back() * 100.0,
        above(1.0e-5),
        above(1.0e-4),
        above(1.0e-3)
      );
    }
  }
}

namespace {

using talus::destruction::FinerLevel;
using talus::destruction::Fracture;

}

TEST_CASE("each material's finer level is the spec's subdivision, along the grain for timber and in the pane's plane for glass", "[destruction]") {
  const Member cube{glm::dvec3(0.0), glm::dvec3(0.25), 0u, Role::Wall};
  const Member beam{glm::dvec3(0.0), glm::dvec3(1.0, 0.15, 0.1), 0u, Role::Beam};
  const Member post{glm::dvec3(0.0), glm::dvec3(0.1, 1.2, 0.1), 0u, Role::Column};
  const Member joist{glm::dvec3(0.0), glm::dvec3(0.1, 0.15, 2.0), 0u, Role::Beam};
  const Member window{glm::dvec3(0.0), glm::dvec3(0.5, 0.75, 0.005), 0u, Role::Pane};
  const Member side{glm::dvec3(0.0), glm::dvec3(0.005, 0.75, 0.5), 0u, Role::Pane};

  CHECK(talus::destruction::concrete().fracture == Fracture::Blocks);
  CHECK(talus::destruction::concrete().pieces == 2u);
  CHECK(talus::destruction::steel().fracture == Fracture::None);
  CHECK(talus::destruction::steel().pieces == 1u);
  CHECK(talus::destruction::finerCells(talus::destruction::concrete(), cube) == glm::uvec3(2u));
  CHECK(talus::destruction::finerCells(talus::destruction::reinforcedConcrete(0.02), cube) == glm::uvec3(2u));
  CHECK(talus::destruction::finerCells(talus::destruction::brickMasonry(), cube) == glm::uvec3(2u));
  CHECK(talus::destruction::finerCells(talus::destruction::timber(), beam) == glm::uvec3(1u, 3u, 3u));
  CHECK(talus::destruction::finerCells(talus::destruction::timber(), post) == glm::uvec3(3u, 1u, 3u));
  CHECK(talus::destruction::finerCells(talus::destruction::timber(), joist) == glm::uvec3(3u, 3u, 1u));
  CHECK(talus::destruction::finerCells(talus::destruction::glass(), window) == glm::uvec3(2u, 2u, 1u));
  CHECK(talus::destruction::finerCells(talus::destruction::glass(), side) == glm::uvec3(1u, 2u, 2u));
  CHECK(talus::destruction::finerCells(talus::destruction::steel(), beam) == glm::uvec3(1u));
  const glm::dvec3 up(0.0, 1.0, 0.0);
  const glm::dvec3 east(1.0, 0.0, 0.0);
  const glm::dvec3 slant = glm::normalize(glm::dvec3(1.0, 1.0, 0.0));
  const glm::dvec3 thin(0.25, 0.1, 0.1);
  const double endLoad = 22.0e6 * 4.0 * 0.1 * 0.1;
  const double sideLoad = 0.4e6 * 4.0 * 0.25 * 0.1;

  CHECK(talus::destruction::fractureStrength(talus::destruction::concrete(), cube, up) == 2.6e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::concrete(), cube, east) == 2.6e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::brickMasonry(), cube, up) == 0.25e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::brickMasonry(), cube, east) == 0.5e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::timber(), beam, east) == 22.0e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::timber(), beam, up) == 0.4e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::timber(), post, up) == 22.0e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::glass(), window, glm::dvec3(0.0, 0.0, 1.0)) == 45.0e6);
  CHECK(talus::destruction::fractureStrength(talus::destruction::reinforcedConcrete(0.02), cube, up) == 0.02 * 380.0e6);
  CHECK(std::abs(talus::destruction::fractureLoad(talus::destruction::timber(), beam, thin, east) - endLoad) <= 1.0e-12 * endLoad);
  CHECK(std::abs(talus::destruction::fractureLoad(talus::destruction::timber(), beam, thin, up) - sideLoad) <= 1.0e-12 * sideLoad);
  CHECK(
    std::abs(talus::destruction::fractureLoad(talus::destruction::timber(), beam, thin, slant) - slant.x * (endLoad + sideLoad))
    <= 1.0e-12 * (endLoad + sideLoad)
  );

  for (const StructuralMaterial& material : talus::destruction::districtMaterials()) {
    CHECK_NOTHROW(talus::destruction::validate(material));
  }

  StructuralMaterial single = talus::destruction::concrete();
  StructuralMaterial crowded = talus::destruction::concrete();
  StructuralMaterial unnamed = talus::destruction::steel();

  single.pieces = 1u;
  crowded.pieces = 9u;
  unnamed.pieces = 2u;

  CHECK_THROWS_AS(talus::destruction::validate(single), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::validate(crowded), std::invalid_argument);
  CHECK_THROWS_AS(talus::destruction::validate(unnamed), std::invalid_argument);
}

TEST_CASE("a chunk's sub-boxes fill it in x-then-y-then-z order, with its material, member and density, anchored only on the ground", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.5, 0.25), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 2u);

  const std::vector<Chunk> low = talus::destruction::finerChunks(building, materials, chunked.chunks[0], glm::uvec3(2u));
  const std::vector<Chunk> high = talus::destruction::finerChunks(building, materials, chunked.chunks[1], glm::uvec3(2u));
  double volume = 0.0;

  REQUIRE(low.size() == 8u);
  REQUIRE(high.size() == 8u);

  for (uint32_t cell = 0u; cell != 8u; ++cell) {
    INFO("cell " << cell);

    const glm::dvec3 expected(
      0.125 + 0.25 * static_cast<double>(cell & 1u),
      0.625 + 0.25 * static_cast<double>((cell >> 1u) & 1u),
      0.125 + 0.25 * static_cast<double>((cell >> 2u) & 1u)
    );

    CHECK(high[cell].centre == expected);
    CHECK(high[cell].halfExtents == glm::dvec3(0.125));
    CHECK(high[cell].material == 0u);
    CHECK(high[cell].member == 0u);
    CHECK(std::abs(high[cell].mass - 2450.0 * 0.015625) <= 1.0e-12);
    CHECK(!high[cell].anchored);
    CHECK(low[cell].anchored == (((cell >> 1u) & 1u) == 0u));
    volume += high[cell].volume;
  }

  CHECK(std::abs(volume - chunked.chunks[1].volume) <= 1.0e-12 * chunked.chunks[1].volume);
  CHECK(talus::destruction::finerChunks(building, materials, chunked.chunks[0], glm::uvec3(1u)).empty());
}

TEST_CASE("a fractured chunk's sub-boxes are bonded to their neighbours as the chunker bonds chunks", "[destruction]") {
  const std::vector<StructuralMaterial> materials{
    talus::destruction::concrete(),
    talus::destruction::brickMasonry(),
    talus::destruction::timber()
  };
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.75, 0.25), glm::dvec3(0.25), 0u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(1.25, 0.75, 0.25), glm::dvec3(0.25), 1u, Role::Wall});
  building.members.push_back(Member{glm::dvec3(3.0, 0.65, 0.1), glm::dvec3(0.25, 0.15, 0.1), 2u, Role::Beam});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 3u);

  const auto innerOf = [&](uint32_t chunk, const ChunkerSettings& settings) {
    const Chunk& whole = chunked.chunks[chunk];
    const glm::uvec3 cells = talus::destruction::finerCells(materials[whole.material], building.members[whole.member]);

    return talus::destruction::innerBonds(
      building, materials, settings, talus::destruction::finerChunks(building, materials, whole, cells), cells, 100u
    );
  };

  const std::vector<Bond> solid = innerOf(0u, plain());

  REQUIRE(solid.size() == 12u);

  for (const Bond& bond : solid) {
    CHECK(bond.first >= 100u);
    CHECK(bond.first < bond.second);
    CHECK(bond.second < 108u);
    CHECK(bond.area == 0.0625);
    CHECK(bond.wu == 0.25);
    CHECK(bond.wv == 0.25);
    CHECK(bond.elastic.tension == 2.6e6 * 0.0625);
    CHECK(bond.elastic.compression == 33.0e6 * 0.0625);
    CHECK(bond.shearRule == ShearRule::Coulomb);
    CHECK(bond.crackFriction == 0.7);
  }

  const std::vector<Bond> bricks = innerOf(1u, plain());
  size_t bed = 0u;

  REQUIRE(bricks.size() == 12u);

  for (const Bond& bond : bricks) {
    const bool flat = std::abs(bond.normal.y) > 0.5;

    bed += flat ? 1u : 0u;
    CHECK(bond.elastic.tension == (flat ? 0.25e6 : 0.5e6) * 0.0625);
  }

  CHECK(bed == 4u);

  const std::vector<Bond> splinters = innerOf(2u, plain());

  REQUIRE(splinters.size() == 12u);

  for (const Bond& bond : splinters) {
    CHECK(bond.normal.x == 0.0);
    CHECK(std::abs(bond.elastic.tension - 0.4e6 * bond.area) <= 1.0e-9 * 0.4e6 * bond.area);
    CHECK(bond.crackFriction == 0.0);
  }

  ChunkerSettings varied;

  varied.variety = true;

  const std::vector<Bond> uneven = innerOf(0u, varied);

  REQUIRE(uneven.size() == 12u);

  for (const Bond& bond : uneven) {
    const double factor = talus::destruction::varietyFactor(bond.first, bond.second, 0.30);

    CHECK(std::abs(bond.elastic.tension - 2.6e6 * bond.area * factor) <= 1.0e-9 * 2.6e6 * bond.area);
  }
}

TEST_CASE("a bond splits over the sub-boxes on its face, each with its share of the area and limits", "[destruction]") {
  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 1.0, 0.25), glm::dvec3(0.25, 1.0, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(0.375, 2.125, 0.125), glm::dvec3(0.125), 0u, Role::Wall});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 5u);

  const uint32_t middle = bondJoining(chunked, 2u, 3u);
  const uint32_t top = bondJoining(chunked, 3u, 4u);

  REQUIRE(middle != NO_BOND);
  REQUIRE(top != NO_BOND);

  const std::vector<Chunk> upper = talus::destruction::finerChunks(building, materials, chunked.chunks[3], glm::uvec3(2u));
  const std::vector<Chunk> lower = talus::destruction::finerChunks(building, materials, chunked.chunks[2], glm::uvec3(2u));
  const std::vector<Bond> overUpper = talus::destruction::splitBond(chunked.bonds[middle], 3u, upper, 40u);
  double area = 0.0;

  REQUIRE(overUpper.size() == 4u);

  for (const Bond& piece : overUpper) {
    CHECK(piece.first == 2u);
    CHECK(piece.second >= 40u);
    CHECK(piece.second < 48u);
    CHECK((((piece.second - 40u) >> 1u) & 1u) == 0u);
    CHECK(piece.normal == glm::dvec3(0.0, 1.0, 0.0));
    CHECK(piece.centroid.y == chunked.bonds[middle].centroid.y);
    CHECK(piece.area == 0.0625);
    CHECK(piece.elastic.tension == chunked.bonds[middle].elastic.tension / 4.0);
    area += piece.area;
  }

  CHECK(area == chunked.bonds[middle].area);

  const std::vector<Bond> overLower = talus::destruction::splitBond(chunked.bonds[middle], 2u, lower, 60u);

  REQUIRE(overLower.size() == 4u);

  for (const Bond& piece : overLower) {
    CHECK(piece.first == 3u);
    CHECK((((piece.second - 60u) >> 1u) & 1u) == 1u);
    CHECK(piece.normal == glm::dvec3(0.0, -1.0, 0.0));
  }

  const std::vector<Bond> quarter = talus::destruction::splitBond(chunked.bonds[top], 3u, upper, 40u);

  REQUIRE(quarter.size() == 1u);
  CHECK(quarter[0].first == 4u);
  CHECK(quarter[0].second == 43u);
  CHECK(quarter[0].area == chunked.bonds[top].area);
  CHECK(quarter[0].normal == glm::dvec3(0.0, -1.0, 0.0));
  CHECK_THROWS_AS(talus::destruction::splitBond(chunked.bonds[top], 1u, upper, 40u), std::invalid_argument);
}

TEST_CASE("a building's finer level reserves each chunk's sub-boxes after its chunks, in chunk order, and none for steel", "[destruction]") {
  const std::vector<StructuralMaterial> materials{
    talus::destruction::concrete(),
    talus::destruction::steel(),
    talus::destruction::glass()
  };
  Building building;

  building.members.push_back(Member{glm::dvec3(0.25, 0.5, 0.25), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(1.5, 0.25, 0.25), glm::dvec3(0.5, 0.25, 0.25), 1u, Role::Beam});
  building.members.push_back(Member{glm::dvec3(3.0, 0.25, 0.005), glm::dvec3(0.25, 0.25, 0.005), 2u, Role::Pane});

  const ChunkedBuilding chunked = talus::destruction::chunkBuilding(building, materials, plain());

  REQUIRE(chunked.chunks.size() == 8u);

  const FinerLevel level = talus::destruction::finerLevel(building, chunked, materials);
  const FinerLevel again = talus::destruction::finerLevel(building, chunked, materials);

  CHECK(level.start == std::vector<uint32_t>{0u, 8u, 16u, 16u, 16u, 20u, 24u, 28u, 32u});
  CHECK(level.cells[2] == glm::uvec3(1u));
  CHECK(level.cells[4] == glm::uvec3(2u, 2u, 1u));
  CHECK(again.start == level.start);
}
