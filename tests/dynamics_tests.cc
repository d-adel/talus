#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/material/material.h>
#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/continuous/continuous.h>
#include <talus/dynamics/dynamictree/dynamictree.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/sleep/sleep.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/ledger/ledger.h>
#include <talus/dynamics/manifold/manifold.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/neohookean/neohookean.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/softstep/softstep.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/screen/screen.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <numbers>
#include <random>
#include <set>
#include <span>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

using talus::dynamics::Corners;
using talus::dynamics::Moduli;

namespace {

constexpr double CELL_HALF = 0.0125;
constexpr double CELL_VOLUME = 8.0 * CELL_HALF * CELL_HALF * CELL_HALF;

talus::PhysicalMaterial rubber() {
  talus::PhysicalMaterial material;

  material.density = 1000.0f;
  material.youngsModulus = 1.2854547e6f;
  material.poissonsRatio = 0.45f;

  return material;
}

talus::PhysicalMaterial softRubber() {
  talus::PhysicalMaterial material = rubber();

  material.youngsModulus = 4.0e4f;

  return material;
}

Corners restCorners() {
  Corners rest{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    rest[corner] = CELL_HALF * glm::dvec3(
      (corner & 1u) != 0u ? 1.0 : -1.0,
      (corner & 2u) != 0u ? 1.0 : -1.0,
      (corner & 4u) != 0u ? 1.0 : -1.0
    );
  }

  return rest;
}

Corners deformedCorners() {
  const glm::dmat3 shape(
    glm::dvec3(1.08, 0.03, -0.02),
    glm::dvec3(-0.05, 0.93, 0.04),
    glm::dvec3(0.02, -0.01, 1.11)
  );
  const Corners rest = restCorners();
  Corners deformed{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    deformed[corner] = shape * rest[corner]
                       + 1.0e-3 * glm::dvec3(
                         std::sin(1.0 + corner),
                         std::cos(2.0 * corner),
                         std::sin(3.0 * corner)
                       );
  }

  return deformed;
}

}

TEST_CASE(
  "a dynamics cell's energy and node gradient match Talus's neo-Hookean hex",
  "[dynamics]"
) {
  const Moduli moduli = talus::dynamics::moduliOf(rubber());
  const Corners rest = restCorners();
  const Corners deformed = deformedCorners();

  talus::PreparedHex hex;
  hex.halfExtents = glm::vec3(static_cast<float>(CELL_HALF));
  hex.volume = static_cast<float>(CELL_VOLUME);

  std::array<glm::vec3, 8> deformedFloat{};
  std::array<glm::vec3, 8> restFloat{};
  std::array<glm::vec3, 8> still{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    deformedFloat[corner] = glm::vec3(deformed[corner]);
    restFloat[corner] = glm::vec3(rest[corner]);
    still[corner] = glm::vec3(0.0f);
  }

  const talus::deformable::ElasticParams elastic{
    rubber().youngsModulus,
    rubber().poissonsRatio,
    rubber().density
  };
  const talus::deformable::NeoHookeanHex expected =
    talus::deformable::neoHookeanHex(
      deformedFloat,
      restFloat,
      still,
      hex,
      elastic,
      0.0f,
      0.0f
    );
  const double energy = talus::dynamics::cellEnergy(
    deformed,
    rest,
    glm::dvec3(CELL_HALF),
    CELL_VOLUME,
    moduli
  );

  CHECK(
    std::abs(energy - expected.energy) <= 1.0e-4 * std::abs(expected.energy)
  );

  double largest = 0.0;

  for (const glm::vec3& force : expected.force) {
    largest = std::max(largest, static_cast<double>(glm::length(force)));
  }

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const talus::dynamics::NodeTerms terms = talus::dynamics::nodeTerms(
      deformed,
      rest,
      glm::dvec3(CELL_HALF),
      CELL_VOLUME,
      moduli,
      corner
    );

    CHECK(
      glm::length(terms.gradient + glm::dvec3(expected.force[corner]))
      <= 1.0e-4 * largest
    );
  }
}

TEST_CASE(
  "a dynamics node Hessian matches finite differences of its gradient",
  "[dynamics]"
) {
  const Moduli moduli = talus::dynamics::moduliOf(rubber());
  const Corners rest = restCorners();
  const double step = 1.0e-7;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const Corners deformed = deformedCorners();
    const talus::dynamics::NodeTerms terms = talus::dynamics::nodeTerms(
      deformed,
      rest,
      glm::dvec3(CELL_HALF),
      CELL_VOLUME,
      moduli,
      corner
    );

    double scale = 0.0;

    for (int column = 0; column != 3; ++column) {
      scale = std::max(scale, glm::length(terms.hessian[column]));
    }

    for (int axis = 0; axis != 3; ++axis) {
      Corners plus = deformed;
      Corners minus = deformed;

      plus[corner][axis] += step;
      minus[corner][axis] -= step;

      const glm::dvec3 difference =
        (talus::dynamics::nodeTerms(
           plus,
           rest,
           glm::dvec3(CELL_HALF),
           CELL_VOLUME,
           moduli,
           corner
         )
           .gradient
         - talus::dynamics::nodeTerms(
             minus,
             rest,
             glm::dvec3(CELL_HALF),
             CELL_VOLUME,
             moduli,
             corner
         )
             .gradient)
        / (2.0 * step);

      CHECK(glm::length(difference - terms.hessian[axis]) <= 1.0e-6 * scale);
    }
  }
}

TEST_CASE(
  "a dynamics cell at rest or rigidly turned carries no node force",
  "[dynamics]"
) {
  const Moduli moduli = talus::dynamics::moduliOf(rubber());
  const Corners rest = restCorners();
  const glm::dmat3 turn = glm::mat3_cast(
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 2.0, -0.5)))
  );
  Corners turned{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    turned[corner] = turn * rest[corner] + glm::dvec3(0.3, -0.1, 0.2);
  }

  const double unit = moduli.shear * CELL_VOLUME / CELL_HALF;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    CHECK(
      glm::length(
        talus::dynamics::nodeTerms(
          rest,
          rest,
          glm::dvec3(CELL_HALF),
          CELL_VOLUME,
          moduli,
          corner
        )
          .gradient
      )
      <= 1.0e-12 * unit
    );
    CHECK(
      glm::length(
        talus::dynamics::nodeTerms(
          turned,
          rest,
          glm::dvec3(CELL_HALF),
          CELL_VOLUME,
          moduli,
          corner
        )
          .gradient
      )
      <= 1.0e-9 * unit
    );
  }
}

TEST_CASE("an inverted cell keeps finite node terms", "[dynamics]") {
  const Moduli moduli = talus::dynamics::moduliOf(rubber());
  const Corners rest = restCorners();
  Corners inverted = rest;

  for (glm::dvec3& corner : inverted) {
    corner.y = -corner.y;
  }

  CHECK(
    talus::dynamics::volumeRatio(inverted, rest, glm::dvec3(CELL_HALF)) < 0.0
  );

  const double energy = talus::dynamics::cellEnergy(
    inverted,
    rest,
    glm::dvec3(CELL_HALF),
    CELL_VOLUME,
    moduli
  );

  CHECK(std::isfinite(energy));

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const talus::dynamics::NodeTerms terms = talus::dynamics::nodeTerms(
      inverted,
      rest,
      glm::dvec3(CELL_HALF),
      CELL_VOLUME,
      moduli,
      corner
    );

    for (int column = 0; column != 3; ++column) {
      CHECK(std::isfinite(terms.gradient[column]));
      CHECK(std::isfinite(glm::length(terms.hessian[column])));
    }
  }
}

namespace {

std::vector<talus::PhysicalMaterial> rubberLanes() {
  std::vector<talus::PhysicalMaterial> lanes(1);

  lanes[0] = rubber();

  return lanes;
}

std::shared_ptr<const talus::PreparedMesh> latticeBall(
  double radius,
  double voxel,
  std::span<const talus::PhysicalMaterial> lanes,
  talus::field::SampledSdf& field
) {
  talus::field::Primitive primitive =
    fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius));

  primitive.shapeConfig.y = 0u;

  const float reach = static_cast<float>(radius + 2.0 * voxel);
  const uint32_t cells =
    static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel)))
    + 1u;

  field = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(cells)
  );

  return talus::deformable::buildMesh(
    field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

talus::field::SampledSdf boxField(const glm::dvec3& half, double voxel) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(half))
    },
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

talus::field::SampledSdf ridgeField(
  const glm::vec3& centre,
  const glm::vec3& half,
  double voxel
) {
  const glm::vec3 reach = half + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(centre, half)},
    centre - reach,
    centre + reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

std::shared_ptr<const talus::PreparedMesh> latticeBlock(
  const glm::dvec3& half,
  double voxel,
  std::span<const talus::PhysicalMaterial> lanes,
  talus::field::SampledSdf& field
) {
  field = boxField(half, voxel);

  return talus::deformable::buildMesh(
    field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

}

TEST_CASE(
  "a lattice body's node colours never repeat inside a cell",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto mesh = latticeBall(0.1, 0.025, lanes, field);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addLattice(
    bodies,
    mesh,
    lanes,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.0
  );

  const talus::dynamics::LatticeBody& body = bodies.lattices.front();

  REQUIRE(!mesh->hexes.empty());

  for (const talus::PreparedHex& hex : mesh->hexes) {
    uint32_t seen = 0u;

    for (const uint32_t node : hex.nodes) {
      seen |= 1u << body.colour[node];
    }

    CHECK(seen == 0xFFu);
  }

  double mass = 0.0;

  for (const float weight : mesh->nodeMass) {
    mass += weight;
  }

  CHECK(body.position.size() == mesh->restPositions.size());
  CHECK(body.cellStart.size() == mesh->restPositions.size() + 1u);
  CHECK(body.cellOf.size() == 8u * mesh->hexes.size());
  CHECK(mass > 0.0);
}

TEST_CASE(
  "a rigid box has the closed-form mass and principal moments",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05, 0.1, 0.2);
  const talus::field::SampledSdf field = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(
    bodies,
    field,
    half,
    1.5,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    4u
  );

  const talus::dynamics::RigidBody& box = bodies.rigids.front();
  const glm::dvec3 size = 2.0 * half;

  CHECK(box.mass == 1.5);
  CHECK(
    std::abs(box.moment.x - 1.5 * (size.y * size.y + size.z * size.z) / 12.0)
    <= 1.0e-15
  );
  CHECK(
    std::abs(box.moment.y - 1.5 * (size.x * size.x + size.z * size.z) / 12.0)
    <= 1.0e-15
  );
  CHECK(
    std::abs(box.moment.z - 1.5 * (size.x * size.x + size.y * size.y) / 12.0)
    <= 1.0e-15
  );

  for (const glm::dvec3& sample : box.samples) {
    const glm::dvec3 outside = glm::abs(sample) - half;

    CHECK(std::abs(std::max({outside.x, outside.y, outside.z})) <= 1.0e-12);
  }
}

TEST_CASE("the dynamics energy ledger sums its terms", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto mesh = latticeBall(0.1, 0.025, lanes, field);
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const glm::dvec3 gravity(0.0, -9.81, 0.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addLattice(
    bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.5, -1.0, 0.0),
    0.0
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    2.0,
    glm::dvec3(1.0, 2.0, 0.0),
    glm::dvec3(0.0, 0.0, 3.0),
    0.5,
    2u
  );
  bodies.rigids.front().angularVelocity = glm::dvec3(0.0, 4.0, 0.0);

  talus::dynamics::LatticeBody& ball = bodies.lattices.front();

  ball.position[0] += glm::dvec3(0.002, 0.0, 0.0);

  double kinetic = 0.0;
  double lift = 0.0;
  double elastic = 0.0;

  for (size_t node = 0u; node != ball.position.size(); ++node) {
    kinetic += 0.5 * mesh->nodeMass[node]
               * glm::dot(ball.velocity[node], ball.velocity[node]);
    lift += mesh->nodeMass[node] * 9.81 * ball.position[node].y;
  }

  for (size_t cell = 0u; cell != mesh->hexes.size(); ++cell) {
    talus::dynamics::Corners deformed{};
    talus::dynamics::Corners rest{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const uint32_t node = mesh->hexes[cell].nodes[corner];

      deformed[corner] = ball.position[node] - glm::dvec3(0.0, 1.0, 0.0);
      rest[corner] = glm::dvec3(mesh->restPositions[node]);
    }

    elastic += talus::dynamics::cellEnergy(
      deformed,
      rest,
      glm::dvec3(mesh->hexes[cell].halfExtents),
      mesh->hexes[cell].volume,
      ball.moduli[cell]
    );
  }

  const talus::dynamics::RigidBody& box = bodies.rigids.front();

  kinetic += 0.5 * 2.0 * 9.0 + 0.5 * box.moment.y * 16.0;
  lift += 2.0 * 9.81 * 2.0;

  const talus::dynamics::Energy energy =
    talus::dynamics::energyOf(bodies, gravity);

  CHECK(std::abs(energy.kinetic - kinetic) <= 1.0e-12 * kinetic);
  CHECK(std::abs(energy.gravity - lift) <= 1.0e-12 * lift);
  CHECK(std::abs(energy.elastic - elastic) <= 1.0e-12 * elastic);
  CHECK(elastic > 0.0);
  CHECK(energy.total() == energy.kinetic + energy.gravity + energy.elastic);
}

TEST_CASE(
  "a spinning cube keeps a unit rotation and its angular momentum",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    2u
  );
  world.bodies.rigids.front().angularVelocity = glm::dvec3(0.0, 0.0, 7.0);

  const talus::dynamics::RigidBody& cube = world.bodies.rigids.front();
  const glm::dvec3 before =
    talus::dynamics::worldInertia(cube) * cube.angularVelocity;

  for (uint32_t frame = 0u; frame != 600u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 after =
    talus::dynamics::worldInertia(cube) * cube.angularVelocity;

  CHECK(std::abs(glm::length(cube.rotation) - 1.0) <= 1.0e-12);
  CHECK(glm::length(after - before) <= 1.0e-6 * glm::length(before));
}

TEST_CASE(
  "a lattice body keeps one gauss gradient table per cell size",
  "[dynamics]"
) {
  auto mesh = std::make_shared<talus::PreparedMesh>();
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  const std::array<glm::vec3, 3> sizes{
    glm::vec3(0.05f),
    glm::vec3(0.025f, 0.05f, 0.1f),
    glm::vec3(0.05f)
  };

  for (uint32_t node = 0u; node != 16u; ++node) {
    mesh->restPositions.push_back(glm::vec3(
      static_cast<float>(node & 1u),
      static_cast<float>((node >> 1u) & 1u),
      static_cast<float>(node >> 2u)
    ));
  }

  mesh->nodeMass.assign(16u, 1.0f);

  for (uint32_t cell = 0u; cell != 3u; ++cell) {
    talus::PreparedHex hex;

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      hex.nodes[corner] = (cell == 1u ? 8u : 0u) + corner;
    }

    hex.halfExtents = sizes[cell];
    hex.volume = 8.0f * sizes[cell].x * sizes[cell].y * sizes[cell].z;
    mesh->hexes.push_back(hex);
  }

  const talus::dynamics::LatticeBody body = talus::dynamics::makeLattice(
    mesh,
    lanes,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    nullptr
  );

  REQUIRE(body.gaussGradientsOf.size() == 3u);
  CHECK(body.gaussGradients.size() == 2u);

  for (uint32_t cell = 0u; cell != 3u; ++cell) {
    CHECK(
      body.gaussGradients[body.gaussGradientsOf[cell]]
      == talus::dynamics::gaussGradients(glm::dvec3(sizes[cell]))
    );
  }
}

TEST_CASE("a fully anchored lattice stays where it is", "[dynamics]") {
  auto mesh = std::make_shared<talus::PreparedMesh>();
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto ball = latticeBall(0.1, 0.05, lanes, field);

  *mesh = *ball;
  mesh->anchored.assign(mesh->restPositions.size(), 1u);

  talus::dynamics::World world;

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.0
  );

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    CHECK(
      world.bodies.lattices.front().position[node]
      == glm::dvec3(mesh->restPositions[node])
    );
  }
}

TEST_CASE(
  "the dynamics broadphase pairs exactly the overlapping boxes",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor =
    boxField(glm::dvec3(1.0, 0.1, 1.0), 0.05);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 0.5, 0.0),
    glm::dvec3(0.0),
    0.5,
    2u
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.09, 0.5, 0.0),
    glm::dvec3(0.0),
    0.5,
    2u
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(3.0, 0.5, 0.0),
    glm::dvec3(0.0),
    0.5,
    2u
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 0.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    2u
  );
  talus::dynamics::addStatic(bodies, floor, 0.5);

  const std::vector<talus::dynamics::BodyPair> pairs =
    talus::dynamics::broadphase(bodies, 0.0);
  std::vector<std::pair<uint32_t, uint32_t>> found;

  for (const talus::dynamics::BodyPair& pair : pairs) {
    const uint32_t a = pair.first.side == talus::dynamics::Side::Static
                         ? 100u + pair.first.index
                         : pair.first.index;
    const uint32_t b = pair.second.side == talus::dynamics::Side::Static
                         ? 100u + pair.second.index
                         : pair.second.index;

    found.emplace_back(std::min(a, b), std::max(a, b));
  }

  std::sort(found.begin(), found.end());

  const std::vector<std::pair<uint32_t, uint32_t>> expected{
    {0u, 1u},
    {3u, 100u}
  };

  CHECK(found == expected);
}

namespace {

talus::field::SampledSdf floorField(double scale) {
  const glm::vec3 low(-1.04f, -0.14f, -1.04f);
  const glm::vec3 high(1.04f, 0.34f, 1.04f);
  talus::field::SampledSdf floor = talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(1.0f, 0.1f, 1.0f))
    },
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );

  for (float& value : floor.distances) {
    value *= static_cast<float>(scale);
  }

  return floor;
}

double restingHeight(double scale) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(scale);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  return world.bodies.rigids.front().position.y;
}

}

TEST_CASE(
  "a rigid box resting on a static floor carries its weight",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const double load = talus::dynamics::normalForceOn(
    world,
    {talus::dynamics::Side::Rigid, 0u}
  );

  CHECK(std::abs(load - 1.5 * 9.81) <= 0.10 * 1.5 * 9.81);
  CHECK(std::abs(world.bodies.rigids.front().position.y - 0.15) <= 0.01);
  CHECK(glm::length(world.bodies.rigids.front().velocity) <= 0.05);
}

TEST_CASE(
  "a field whose gradient is not unit length gives the same resting height",
  "[dynamics]"
) {
  CHECK(std::abs(restingHeight(2.0) - restingHeight(1.0)) <= 1.0e-6);
}

TEST_CASE("a lattice far from a static field makes no contacts", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto mesh = latticeBall(0.1, 0.05, lanes, field);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 50.0, 0.0),
    glm::dvec3(0.0),
    0.0
  );

  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  CHECK(report.contacts == 0u);
  CHECK(!report.nonFinite);
}

TEST_CASE(
  "islands join bodies through contacts and an island with a lattice goes to strainstep",
  "[dynamics]"
) {
  using talus::dynamics::Contact;
  using talus::dynamics::Owner;
  using talus::dynamics::Side;

  talus::dynamics::BodySet bodies;

  bodies.lattices.resize(1u);
  bodies.rigids.resize(4u);
  bodies.statics.resize(1u);

  const auto contact = [](Side as, uint32_t a, Side bs, uint32_t b) {
    Contact made;

    made.a.side = as;
    made.a.body = a;
    made.b.side = bs;
    made.b.body = b;

    return made;
  };

  const std::vector<Contact> contacts{
    contact(Side::Rigid, 0u, Side::Rigid, 1u),
    contact(Side::Rigid, 1u, Side::Static, 0u),
    contact(Side::Rigid, 2u, Side::Lattice, 0u),
    contact(Side::Rigid, 3u, Side::Static, 0u)
  };

  const std::vector<talus::dynamics::Island> islands =
    talus::dynamics::islandsOf(bodies, talus::dynamics::ManifoldCache{}, contacts);

  REQUIRE(islands.size() == 3u);
  CHECK(islands[0].owner == Owner::Strainstep);
  CHECK(islands[0].bodies.size() == 2u);
  CHECK(islands[0].contacts == std::vector<uint32_t>{2u});
  CHECK(islands[1].owner == Owner::Softstep);
  CHECK(islands[1].bodies.size() == 2u);
  CHECK(islands[1].contacts == std::vector<uint32_t>{0u, 1u});
  CHECK(islands[1].external);
  CHECK(islands[2].owner == Owner::Softstep);
  CHECK(islands[2].bodies.size() == 1u);
  CHECK(islands[2].bodies[0].index == 3u);
}

TEST_CASE("a flat field gradient creates no contact", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto mesh = latticeBall(0.1, 0.05, lanes, field);
  talus::field::SampledSdf flat = floorField(1.0);

  std::fill(flat.distances.begin(), flat.distances.end(), -0.01f);

  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, flat, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.0, 0.0),
    glm::dvec3(0.0),
    0.0
  );

  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  CHECK(report.contacts == 0u);
  CHECK(!report.nonFinite);
}

TEST_CASE(
  "a lattice cube hitting a rigid box in free space keeps the momentum",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf cubeField = boxField(half, 0.0125);
  const auto mesh = talus::deformable::buildMesh(
    cubeField,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(-0.2, 0.0, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    0.0
  );
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 0.01, 0.0),
    glm::dvec3(0.0),
    0.0,
    4u
  );

  const auto momentum = [&]() {
    glm::dvec3 total(0.0);
    double mass = 0.0;

    for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
      total += static_cast<double>(mesh->nodeMass[node])
               * world.bodies.lattices.front().velocity[node];
      mass += mesh->nodeMass[node];
    }

    total += world.bodies.rigids.front().mass
             * world.bodies.rigids.front().velocity;
    mass += world.bodies.rigids.front().mass;

    return std::pair<glm::dvec3, double>(total, mass);
  };

  const auto [before, mass] = momentum();
  bool touched = false;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (const talus::dynamics::Contact& contact : world.contacts) {
      touched = touched || contact.memory.normal > 0.0;
    }
  }

  CHECK(touched);
  CHECK(glm::length(momentum().first - before) / mass <= 1.0e-4);
  CHECK(world.bodies.rigids.front().velocity.x > 0.0);
}

TEST_CASE(
  "two lattice cubes stacked on a static floor come to rest without passing "
  "through",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf cubeField = boxField(half, 0.025);
  const auto mesh = talus::deformable::buildMesh(
    cubeField,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.151, 0.0),
    glm::dvec3(0.0),
    0.5,
    &cubeField
  );
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.252, 0.0),
    glm::dvec3(0.0),
    0.5,
    &cubeField
  );

  double lowestTop = std::numeric_limits<double>::infinity();
  double highestBottom = -std::numeric_limits<double>::infinity();
  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 600u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame >= 300u) {
      for (const glm::dvec3& velocity : world.bodies.lattices[0].velocity) {
        fastest = std::max(fastest, glm::length(velocity));
      }

      for (const glm::dvec3& velocity : world.bodies.lattices[1].velocity) {
        fastest = std::max(fastest, glm::length(velocity));
      }
    }
  }

  for (const glm::dvec3& position : world.bodies.lattices[1].position) {
    lowestTop = std::min(lowestTop, position.y);
  }

  for (const glm::dvec3& position : world.bodies.lattices[0].position) {
    highestBottom = std::max(highestBottom, position.y);
  }

  CHECK(lowestTop >= highestBottom - 0.01);
  CHECK(fastest < 0.05);
}

TEST_CASE(
  "a spinning lattice ball in free fall does not spin up",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes =
    rubberLanes();
  talus::field::SampledSdf field;
  const auto mesh = latticeBall(0.1, 0.05, lanes, field);
  talus::dynamics::World world;
  const glm::dvec3 start(0.0, 5.0, 0.0);
  const glm::dvec3 spin(0.0, 0.0, 5.0);

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    start,
    glm::dvec3(0.0),
    0.0
  );

  talus::dynamics::LatticeBody& ball = world.bodies.lattices.front();

  for (size_t node = 0u; node != ball.velocity.size(); ++node) {
    ball.velocity[node] = glm::cross(spin, ball.position[node] - start);
  }

  ball.previousVelocity = ball.velocity;

  const auto relative = [&]() {
    glm::dvec3 momentum(0.0);
    double mass = 0.0;
    double kinetic = 0.0;

    for (size_t node = 0u; node != ball.velocity.size(); ++node) {
      const double m = mesh->nodeMass[node];

      momentum += m * ball.velocity[node];
      mass += m;
      kinetic += 0.5 * m * glm::dot(ball.velocity[node], ball.velocity[node]);
    }

    return kinetic - 0.5 * glm::dot(momentum, momentum) / mass;
  };

  const double before = relative();

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(relative() <= 1.02 * before);
}

TEST_CASE(
  "a box on a floor tilted below its friction angle stays put",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;
  const double tilt = 20.0 * 3.14159265358979323846 / 180.0;

  world.parameters.gravity =
    9.81 * glm::dvec3(std::sin(tilt), -std::cos(tilt), 0.0);
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const glm::dvec3 start = world.bodies.rigids.front().position;
  double drift = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    drift = std::max(
      drift,
      glm::length(world.bodies.rigids.front().position - start)
    );
  }

  CHECK(drift < 0.01);
}

TEST_CASE(
  "two spheres get one manifold point halfway between their surfaces",
  "[dynamics]"
) {
  const talus::dynamics::Manifold manifold = talus::dynamics::collideSpheres(
    glm::dvec3(0.0, 0.29, 0.0), 0.1,
    glm::dvec3(0.0, 0.0, 0.0), 0.2,
    0.02
  );

  REQUIRE(manifold.count == 1u);
  CHECK(glm::length(manifold.normal - glm::dvec3(0.0, 1.0, 0.0)) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].separation + 0.01) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].point.y - 0.195) <= 1.0e-12);
  CHECK(talus::dynamics::collideSpheres(
          glm::dvec3(0.0, 0.33, 0.0), 0.1, glm::dvec3(0.0), 0.2, 0.02
        ).count == 0u);
}

TEST_CASE(
  "a sphere on a box face gets one point along the face normal",
  "[dynamics]"
) {
  const talus::dynamics::BoxPose box{
    glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(0.5, 0.25, 0.5)
  };
  const talus::dynamics::Manifold manifold =
    talus::dynamics::collideBoxAndSphere(glm::dvec3(0.1, 0.34, 0.0), 0.1, box, 0.02);

  REQUIRE(manifold.count == 1u);
  CHECK(glm::length(manifold.normal - glm::dvec3(0.0, 1.0, 0.0)) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].separation + 0.01) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].point.y - 0.245) <= 1.0e-12);
}

TEST_CASE(
  "a sphere on a box corner gets one point with a unit normal",
  "[dynamics]"
) {
  const talus::dynamics::BoxPose box{
    glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(0.5)
  };
  const glm::dvec3 corner(0.5);
  const glm::dvec3 centre = corner + 0.1 * glm::normalize(glm::dvec3(1.0));
  const talus::dynamics::Manifold manifold =
    talus::dynamics::collideBoxAndSphere(centre, 0.1, box, 0.02);

  REQUIRE(manifold.count == 1u);
  CHECK(std::abs(glm::length(manifold.normal) - 1.0) <= 1.0e-12);
  CHECK(glm::length(manifold.normal - glm::normalize(glm::dvec3(1.0))) <= 1.0e-9);
  CHECK(std::abs(manifold.points[0].separation) <= 1.0e-9);
}

TEST_CASE(
  "a sphere sunk into a box is pushed out through the nearest face",
  "[dynamics]"
) {
  const talus::dynamics::BoxPose box{
    glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(0.5)
  };
  const talus::dynamics::Manifold manifold =
    talus::dynamics::collideBoxAndSphere(glm::dvec3(0.0, 0.0, 0.45), 0.1, box, 0.02);

  REQUIRE(manifold.count == 1u);
  CHECK(glm::length(manifold.normal - glm::dvec3(0.0, 0.0, 1.0)) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].separation + 0.15) <= 1.0e-12);
}

namespace {

talus::dynamics::BoxPose unitBox(const glm::dvec3& centre, double turnY = 0.0) {
  return {centre, glm::angleAxis(turnY, glm::dvec3(0.0, 1.0, 0.0)), glm::dvec3(0.5)};
}

}

TEST_CASE(
  "a box resting on a box gets one four-point manifold from the narrowphase",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.0995, 0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<talus::dynamics::Contact> contacts = talus::dynamics::narrowphase(bodies, pairs, 0.02);

  REQUIRE(contacts.size() == 4u);

  for (const talus::dynamics::Contact& contact : contacts) {
    CHECK(contact.manifold == contacts.front().manifold);
    CHECK(std::abs(contact.gap + 0.0005) <= 1.0e-9);
    CHECK(std::abs(std::abs(contact.basis[0].y) - 1.0) <= 1.0e-12);
  }
}

TEST_CASE(
  "a box pair gets the same contacts in either broadphase order",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 0.0995, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const talus::dynamics::BodyRef upper{talus::dynamics::Side::Rigid, 0u};
  const talus::dynamics::BodyRef lower{talus::dynamics::Side::Rigid, 1u};
  const std::vector<talus::dynamics::BodyPair> forward{{upper, lower}};
  const std::vector<talus::dynamics::BodyPair> backward{{lower, upper}};
  const std::vector<talus::dynamics::Contact> one =
    talus::dynamics::narrowphase(bodies, forward, 0.02);
  const std::vector<talus::dynamics::Contact> other =
    talus::dynamics::narrowphase(bodies, backward, 0.02);

  REQUIRE(one.size() == 4u);
  REQUIRE(other.size() == one.size());

  for (size_t index = 0u; index != one.size(); ++index) {
    CHECK(one[index].key == other[index].key);
    CHECK(one[index].a.body == other[index].a.body);
    CHECK(glm::length(one[index].basis[0] - other[index].basis[0]) <= 1.0e-12);
  }
}

TEST_CASE("a box resting on a box gets four points on the shared face", "[dynamics]") {
  const talus::dynamics::Manifold manifold = talus::dynamics::collideBoxes(
    unitBox(glm::dvec3(0.0, 0.99, 0.0)), unitBox(glm::dvec3(0.0)), 0.02
  );

  REQUIRE(manifold.count == 4u);
  CHECK(glm::length(manifold.normal - glm::dvec3(0.0, 1.0, 0.0)) <= 1.0e-12);

  for (uint32_t index = 0u; index != 4u; ++index) {
    CHECK(std::abs(manifold.points[index].separation + 0.01) <= 1.0e-9);
    CHECK(std::abs(manifold.points[index].point.y - 0.495) <= 1.0e-9);
    CHECK(std::abs(std::abs(manifold.points[index].point.x) - 0.5) <= 1.0e-9);
    CHECK(std::abs(std::abs(manifold.points[index].point.z) - 0.5) <= 1.0e-9);
  }
}

TEST_CASE("a box turned on a box keeps four points inside the overlap", "[dynamics]") {
  const talus::dynamics::Manifold manifold = talus::dynamics::collideBoxes(
    unitBox(glm::dvec3(0.0, 0.99, 0.0), 0.25 * 3.14159265358979323846), unitBox(glm::dvec3(0.0)), 0.02
  );

  REQUIRE(manifold.count == 4u);

  for (uint32_t index = 0u; index != 4u; ++index) {
    const glm::dvec3 p = manifold.points[index].point;

    CHECK(std::abs(p.x) <= 0.5 + 1.0e-9);
    CHECK(std::abs(p.z) <= 0.5 + 1.0e-9);
    CHECK(std::abs(p.x) + std::abs(p.z) <= 0.5 * std::sqrt(2.0) + 1.0e-9);
  }
}

TEST_CASE("crossed boxes touching edge to edge get one point", "[dynamics]") {
  const glm::dquat roll = glm::angleAxis(0.25 * 3.14159265358979323846, glm::dvec3(0.0, 0.0, 1.0));
  const glm::dquat yaw = glm::angleAxis(0.5 * 3.14159265358979323846, glm::dvec3(0.0, 1.0, 0.0));
  const double reach = 0.5 * std::sqrt(2.0);
  const talus::dynamics::BoxPose top{glm::dvec3(0.0, 2.0 * reach - 0.005, 0.0), yaw * roll, glm::dvec3(0.5)};
  const talus::dynamics::BoxPose bottom{glm::dvec3(0.0), roll, glm::dvec3(0.5)};
  const talus::dynamics::Manifold manifold = talus::dynamics::collideBoxes(top, bottom, 0.02);

  REQUIRE(manifold.count == 1u);
  CHECK(std::abs(glm::length(manifold.normal) - 1.0) <= 1.0e-12);
  CHECK(std::abs(manifold.points[0].separation + 0.005) <= 1.0e-9);
  CHECK(glm::length(manifold.points[0].point - glm::dvec3(0.0, reach - 0.0025, 0.0)) <= 1.0e-9);
}

TEST_CASE("separated boxes beyond the speculative distance get no points", "[dynamics]") {
  CHECK(talus::dynamics::collideBoxes(
          unitBox(glm::dvec3(0.0, 1.03, 0.0)), unitBox(glm::dvec3(0.0)), 0.02
        ).count == 0u);
}

TEST_CASE("box contact features stay the same under a small slide", "[dynamics]") {
  const talus::dynamics::Manifold before = talus::dynamics::collideBoxes(
    unitBox(glm::dvec3(0.1, 0.99, 0.0)), unitBox(glm::dvec3(0.0)), 0.02
  );
  const talus::dynamics::Manifold after = talus::dynamics::collideBoxes(
    unitBox(glm::dvec3(0.1001, 0.99, 0.0)), unitBox(glm::dvec3(0.0)), 0.02
  );

  REQUIRE(before.count == after.count);

  std::array<uint32_t, 4> first{};
  std::array<uint32_t, 4> second{};

  for (uint32_t index = 0u; index != before.count; ++index) {
    first[index] = before.points[index].feature;
    second[index] = after.points[index].feature;
  }

  std::sort(first.begin(), first.end());
  std::sort(second.begin(), second.end());
  CHECK(first == second);
}

TEST_CASE(
  "contact matching starts new keys at zero and keeps duplicates apart",
  "[dynamics]"
) {
  std::vector<talus::dynamics::Contact> previous(3);

  previous[0].key = 7u;
  previous[0].memory.normal = 2.0;
  previous[1].key = 9u;
  previous[1].memory.normal = 3.0;
  previous[2].key = 9u;
  previous[2].memory.normal = 4.0;

  const talus::dynamics::ContactMatch match(previous);

  CHECK(match.find(7u) == 0);
  CHECK(match.find(9u) == -1);
  CHECK(match.find(11u) == -1);

  std::vector<talus::dynamics::Contact> current(3);

  current[0].key = 7u;
  current[1].key = 9u;
  current[2].key = 11u;
  current[2].memory.normal = 5.0;
  talus::dynamics::carryMemory(current, previous);

  CHECK(current[0].memory.normal == 2.0);
  CHECK(current[1].memory.normal == 0.0);
  CHECK(current[2].memory.normal == 0.0);
}

TEST_CASE(
  "contact keys stay unique past thirty-one bodies",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (uint32_t index = 0u; index != 32u; ++index) {
    const glm::dvec3 place = index == 0u    ? glm::dvec3(0.0, 0.15, 0.0)
                             : index == 31u ? glm::dvec3(0.0, 0.25, 0.0)
                                            : glm::dvec3(5.0 + index, 5.0, 0.0);

    talus::dynamics::addRigidBox(
      world.bodies,
      crate,
      half,
      1.5,
      place,
      glm::dvec3(0.0),
      0.5,
      3u
    );
  }

  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const std::vector<talus::dynamics::Contact> records =
    talus::dynamics::contactRecords(world);
  std::set<uint64_t> keys;

  for (const talus::dynamics::Contact& contact : records) {
    keys.insert(contact.key);
  }

  REQUIRE(!records.empty());
  CHECK(keys.size() == records.size());
}

namespace {

talus::field::SampledSdf plateField() {
  const glm::vec3 low(-1.1f, -0.2f, -1.1f);
  const glm::vec3 high(1.1f, 0.4f, 1.1f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f), glm::vec3(1.0f, 0.02f, 1.0f))
    },
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );
}

}

TEST_CASE(
  "a fast rigid box meets a thin static plate's contact before crossing it",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf plate = plateField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, plate, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.22, 0.0),
    glm::dvec3(0.0, -20.0, 0.0),
    0.5,
    3u
  );

  bool touched = false;

  for (uint32_t frame = 0u; frame != 3u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    touched = touched || report.activeContacts != 0u;
  }

  CHECK(touched);
}

namespace {

std::vector<glm::dvec3> sphereSamples(double radius, uint32_t count) {
  std::vector<glm::dvec3> samples;
  const double golden = 3.14159265358979323846 * (3.0 - std::sqrt(5.0));

  samples.emplace_back(0.0, -radius, 0.0);

  for (uint32_t index = 0u; index != count; ++index) {
    const double y = 1.0 - 2.0 * (index + 0.5) / count;
    const double ring = std::sqrt(1.0 - y * y);
    const double turn = golden * index;

    samples.push_back(
      radius * glm::dvec3(ring * std::cos(turn), y, ring * std::sin(turn))
    );
  }

  return samples;
}

talus::field::SampledSdf sphereField(double radius, double voxel) {
  const float reach = static_cast<float>(radius + 2.0 * voxel);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::sphere(glm::vec3(0.0f), static_cast<float>(radius))
    },
    glm::vec3(-reach),
    glm::vec3(reach),
    glm::uvec3(
      static_cast<uint32_t>(std::round(2.0f * reach / static_cast<float>(voxel)))
      + 1u
    )
  );
}

uint32_t addBall(
  talus::dynamics::BodySet& bodies,
  const talus::field::SampledSdf& field,
  const std::vector<glm::dvec3>& samples,
  double radius,
  const glm::dvec3& position,
  const glm::dvec3& velocity,
  double restitution
) {
  (void)samples;

  return talus::dynamics::addRigidSphere(
    bodies,
    field,
    radius,
    1.0,
    position,
    velocity,
    0.5,
    restitution
  );
}

}

TEST_CASE(
  "a rigid ball leaves a floor at its restitution times its impact speed",
  "[dynamics]"
) {
  const double radius = 0.05;
  const talus::field::SampledSdf ball = sphereField(radius, 0.01);
  const std::vector<glm::dvec3> samples = sphereSamples(radius, 64u);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0);
  addBall(
    world.bodies,
    ball,
    samples,
    radius,
    glm::dvec3(0.0, 0.1 + radius + 5.0e-4, 0.0),
    glm::dvec3(0.0, -2.0, 0.0),
    0.5
  );

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 velocity = world.bodies.rigids.front().velocity;

  CHECK(std::abs(velocity.y - 1.0) <= 1.0e-5);
  CHECK(std::abs(velocity.x) <= 1.0e-6);
  CHECK(std::abs(velocity.z) <= 1.0e-6);
}

TEST_CASE("a resting rigid ball does not bounce", "[dynamics]") {
  const double radius = 0.05;
  const talus::field::SampledSdf ball = sphereField(radius, 0.01);
  const std::vector<glm::dvec3> samples = sphereSamples(radius, 64u);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;
  const glm::dvec3 start(0.0, 0.1 + radius, 0.0);

  talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0);
  addBall(world.bodies, ball, samples, radius, start, glm::dvec3(0.0), 0.9);

  double drift = 0.0;
  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    drift = std::max(
      drift,
      glm::length(world.bodies.rigids.front().position - start)
    );
    fastest =
      std::max(fastest, glm::length(world.bodies.rigids.front().velocity));
  }

  CHECK(drift < 0.01);
  CHECK(fastest < 0.05);
}

namespace {

std::vector<uint32_t> rigidIndices(const talus::dynamics::Island& island) {
  std::vector<uint32_t> rigids;

  for (const talus::dynamics::BodyRef& ref : island.bodies) {
    if (ref.side == talus::dynamics::Side::Rigid) {
      rigids.push_back(ref.index);
    }
  }

  std::sort(rigids.begin(), rigids.end());

  return rigids;
}

talus::dynamics::Island islandOfAll(
  const talus::dynamics::BodySet& bodies,
  size_t contacts
) {
  talus::dynamics::Island island;

  for (uint32_t index = 0u; index != bodies.rigids.size(); ++index) {
    island.bodies.push_back({talus::dynamics::Side::Rigid, index});
  }

  for (uint32_t index = 0u; index != contacts; ++index) {
    island.contacts.push_back(index);
  }

  return island;
}

talus::dynamics::Contact floorContact(
  uint32_t rigid,
  const glm::dvec3& local,
  double gap,
  double friction,
  double restitution,
  uint32_t manifold
) {
  talus::dynamics::Contact contact;

  contact.a.side = talus::dynamics::Side::Rigid;
  contact.a.body = rigid;
  contact.a.local = local;
  contact.b.side = talus::dynamics::Side::Static;
  contact.basis = {
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(1.0, 0.0, 0.0),
    glm::dvec3(0.0, 0.0, -1.0)
  };
  contact.gap = gap;
  contact.friction = friction;
  contact.restitution = restitution;
  contact.manifold = manifold;

  return contact;
}

void solveRecords(
  talus::dynamics::BodySet& bodies,
  std::vector<talus::dynamics::Contact>& contacts,
  const talus::dynamics::IntegrationParameters& parameters
) {
  talus::dynamics::ManifoldCache cache;
  std::vector<talus::dynamics::BodyPair> pairs;
  talus::dynamics::Island island = islandOfAll(bodies, 0u);

  for (const talus::dynamics::Contact& contact : contacts) {
    const talus::dynamics::BodyPair pair{{contact.a.side, contact.a.body}, {contact.b.side, contact.b.body}};
    const bool known = std::any_of(pairs.begin(), pairs.end(), [&](const talus::dynamics::BodyPair& other) {
      return other.first.side == pair.first.side && other.first.index == pair.first.index
             && other.second.side == pair.second.side && other.second.index == pair.second.index;
    });

    if (!known) {
      pairs.push_back(pair);
    }
  }

  cache.sync(bodies, pairs);

  std::vector<std::vector<size_t>> members(pairs.size());

  for (size_t index = 0u; index != contacts.size(); ++index) {
    for (size_t pair = 0u; pair != pairs.size(); ++pair) {
      if (contacts[index].a.side == pairs[pair].first.side && contacts[index].a.body == pairs[pair].first.index
          && contacts[index].b.side == pairs[pair].second.side && contacts[index].b.body == pairs[pair].second.index) {
        members[pair].push_back(index);
      }
    }
  }

  for (size_t pair = 0u; pair != pairs.size(); ++pair) {
    const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[pair].first, pairs[pair].second));
    std::vector<talus::dynamics::Contact> records;

    for (const size_t index : members[pair]) {
      records.push_back(contacts[index]);
    }

    cache.replaceManifolds(bodies, slot, records);
    island.pairs.push_back(slot);
  }

  std::vector<uint32_t> stateOf;

  const std::vector<uint32_t> one = rigidIndices(island);

  talus::dynamics::solveSoftstep(bodies, cache, one, parameters, stateOf, talus::dynamics::Workers{}, talus::dynamics::resolveSimdWidth(0u));

  for (size_t pair = 0u; pair != pairs.size(); ++pair) {
    const std::vector<talus::dynamics::Contact> records =
      cache.records(island.pairs[pair]);

    for (size_t member = 0u; member != members[pair].size(); ++member) {
      contacts[members[pair][member]].memory = records[member].memory;
    }
  }
}

}

TEST_CASE("a lone rigid box in softstep falls freely", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;
  talus::dynamics::IntegrationParameters parameters;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  std::vector<talus::dynamics::Contact> none;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    solveRecords(bodies, none, parameters);
  }

  const double n = 60.0 * parameters.softstepSubsteps;
  const double h = parameters.dt / parameters.softstepSubsteps;
  const double expected = 1.0 - 9.81 * h * h * n * (n + 1.0) / 2.0;

  CHECK(std::abs(bodies.rigids[0].velocity.y + 9.81) <= 1.0e-4);
  CHECK(std::abs(bodies.rigids[0].position.y - expected) <= 1.0e-5);
}

TEST_CASE("softstep treats a zero-mass rigid body as immovable", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;
  talus::dynamics::IntegrationParameters parameters;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    0.0,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  std::vector<talus::dynamics::Contact> none;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    solveRecords(bodies, none, parameters);
  }

  CHECK(bodies.rigids[0].position == glm::dvec3(0.0));
  CHECK(bodies.rigids[0].velocity == glm::dvec3(0.0));
}

TEST_CASE(
  "a ball meeting a floor in softstep leaves at its restitution times its speed",
  "[dynamics]"
) {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.01);
  talus::dynamics::BodySet bodies;
  talus::dynamics::IntegrationParameters parameters;

  parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addRigidSphere(
    bodies,
    ball,
    0.05,
    1.0,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0, -2.0, 0.0),
    0.5,
    0.5
  );

  std::vector<talus::dynamics::Contact> contacts{
    floorContact(0u, glm::dvec3(0.0, -0.05, 0.0), 0.0, 0.5, 0.5, 0u)
  };

  solveRecords(bodies, contacts, parameters);

  CHECK(std::abs(bodies.rigids[0].velocity.y - 1.0) <= 1.0e-5);
  CHECK(std::abs(bodies.rigids[0].velocity.x) <= 1.0e-6);
}

TEST_CASE(
  "a box resting on a floor in softstep keeps its weight in memory",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;
  talus::dynamics::IntegrationParameters parameters;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.05, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  std::vector<talus::dynamics::Contact> contacts;

  for (uint32_t corner = 0u; corner != 4u; ++corner) {
    const glm::dvec3 local(
      corner & 1u ? 0.05 : -0.05,
      -0.05,
      corner & 2u ? 0.05 : -0.05
    );

    contacts.push_back(floorContact(0u, local, 0.0, 0.5, 0.0, 0u));
  }

  double load = 0.0;
  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    for (talus::dynamics::Contact& contact : contacts) {
      const talus::dynamics::RigidBody& body = bodies.rigids[0];

      contact.gap =
        (body.position + glm::mat3_cast(body.rotation) * contact.a.local).y;
    }

    solveRecords(bodies, contacts, parameters);
    fastest = std::max(fastest, glm::length(bodies.rigids[0].velocity));
  }

  for (const talus::dynamics::Contact& contact : contacts) {
    load += contact.memory.normal;
  }

  CHECK(std::abs(load - 1.5 * 9.81) <= 1.0e-3 * 1.5 * 9.81);
  CHECK(std::abs(bodies.rigids[0].position.y - 0.05) <= 0.01);
  CHECK(fastest < 0.05);
}

TEST_CASE("a spinning box in softstep keeps its angular momentum", "[dynamics]") {
  const glm::dvec3 half(0.1, 0.05, 0.025);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;
  talus::dynamics::IntegrationParameters parameters;

  parameters.gravity = glm::dvec3(0.0);
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );
  bodies.rigids[0].angularVelocity = glm::dvec3(0.0, 0.0, 5.0);

  const glm::dvec3 start = talus::dynamics::worldInertia(bodies.rigids[0])
                           * bodies.rigids[0].angularVelocity;
  std::vector<talus::dynamics::Contact> none;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    solveRecords(bodies, none, parameters);
  }

  const talus::dynamics::RigidBody& body = bodies.rigids[0];
  const glm::dvec3 end = talus::dynamics::worldInertia(body) * body.angularVelocity;

  CHECK(std::abs(glm::length(body.rotation) - 1.0) <= 1.0e-6);
  CHECK(glm::length(end - start) <= 1.0e-4 * glm::length(start));
}

TEST_CASE(
  "a rigid-only world steps in softstep",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  CHECK(report.softstepIslands == 1u);
  CHECK(report.strainstepIslands == 0u);
  CHECK(report.substeps == 0u);
}

TEST_CASE("ten stacked boxes stand at rest in softstep", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (uint32_t level = 0u; level != 10u; ++level) {
    talus::dynamics::addRigidBox(
      world.bodies,
      crate,
      half,
      1.0,
      glm::dvec3(0.0, 0.15 + 0.1 * level, 0.0),
      glm::dvec3(0.0),
      0.5,
      3u
    );
  }

  std::vector<glm::dvec3> settled;
  double drift = 0.0;
  double fastest = 0.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (frame + 1u == 60u) {
      for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
        settled.push_back(body.position);
      }
    }

    if (frame + 1u > 60u) {
      for (size_t index = 0u; index != settled.size(); ++index) {
        const talus::dynamics::RigidBody& body = world.bodies.rigids[index];

        drift = std::max(drift, glm::length(body.position - settled[index]));
        fastest = std::max(fastest, glm::length(body.velocity));
      }
    }
  }

  CHECK(drift < 0.01);
  CHECK(fastest < 0.05);
}

TEST_CASE(
  "a crate on a lattice block rests with the soft-island settings",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const glm::dvec3 half(0.025);
  const talus::field::SampledSdf crate = boxField(half, 0.005);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    &blockField
  );
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    0.5,
    glm::dvec3(0.0, 0.175, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  double peak = 0.0;
  glm::dvec3 settled(0.0);
  double drift = 0.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    CHECK(report.strainstepIslands == 1u);

    if (frame == 59u) {
      settled = world.bodies.rigids[0].position;
    } else if (frame > 59u) {
      peak = std::max(peak, glm::length(world.bodies.rigids[0].velocity));
      drift =
        std::max(drift, glm::length(world.bodies.rigids[0].position - settled));
    }
  }

  CHECK(drift < 0.01);
  CHECK(peak < 0.05);
}


TEST_CASE(
  "the time of impact of a box falling through a floor is where its bottom "
  "reaches the slop",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.0, 0.3, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const talus::dynamics::RigidPose start{
    glm::dvec3(0.0, 0.3, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  };
  const talus::dynamics::RigidPose end{
    glm::dvec3(0.0, -0.1, 0.0),
    glm::dquat(1.0, 0.0, 0.0, 0.0)
  };
  const double fraction = talus::dynamics::timeOfImpact(
    bodies.rigids[0],
    start,
    end,
    floor,
    0.005
  );

  CHECK(std::abs(fraction - 0.3625) <= 0.25 * 0.005 / 0.4);
}

TEST_CASE(
  "the swept test's turn bound for the turn on which MSVC and g++ parted at the collapse's step 112 is the same "
  "exactly rounded value on every compiler",
  "[dynamics]"
) {
  const double sine = std::bit_cast<double>(0x3f8b51b34f8d6470ull);
  const double cosine = std::bit_cast<double>(0x3fefff4568668dfcull);
  const glm::dquat from(1.0, 0.0, 0.0, 0.0);
  const glm::dquat to(cosine, sine, 0.0, 0.0);
  const glm::dquat relative = glm::conjugate(from) * to;

  REQUIRE(glm::length(glm::dvec3(relative.x, relative.y, relative.z)) == sine);
  REQUIRE(std::abs(relative.w) == cosine);
  CHECK(std::bit_cast<uint64_t>(talus::dynamics::turnBound(from, to)) == 0x3f9b52529f7d379eull);
}

TEST_CASE(
  "for 10,000 rotation pairs, near-zero and near-half turns among them, the swept test's turn bound is at least "
  "the turn and at most twice it",
  "[dynamics]"
) {
  constexpr uint32_t PAIRS = 10000u;
  constexpr double FACTOR = 2.000000001;
  std::mt19937_64 random(20261018u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_real_distribution<double> decade(1.0, 12.0);
  const auto rotation = [&]() {
    for (;;) {
      const glm::dquat drawn(unit(random), unit(random), unit(random), unit(random));
      const double size = glm::dot(drawn, drawn);

      if (size > 1.0e-6 && size <= 1.0) {
        return glm::normalize(drawn);
      }
    }
  };
  const auto direction = [&]() {
    for (;;) {
      const glm::dvec3 drawn(unit(random), unit(random), unit(random));
      const double size = glm::dot(drawn, drawn);

      if (size > 1.0e-6 && size <= 1.0) {
        return glm::normalize(drawn);
      }
    }
  };
  uint32_t failures = 0u;
  double widest = 1.0;

  for (uint32_t pair = 0u; pair != PAIRS; ++pair) {
    const glm::dquat from = rotation();
    glm::dquat to = rotation();

    if (pair % 4u == 1u) {
      to = glm::normalize(from * glm::angleAxis(std::pow(10.0, -decade(random)), direction()));
    } else if (pair % 4u == 2u) {
      to = glm::normalize(from * glm::angleAxis(std::numbers::pi - std::pow(10.0, -decade(random)), direction()));
    } else if (pair % 4u == 3u) {
      to = pair % 8u == 3u ? from : -from;
    }

    const glm::dquat relative = glm::conjugate(from) * to;
    const double angle =
      2.0 * std::atan2(glm::length(glm::dvec3(relative.x, relative.y, relative.z)), std::abs(relative.w));
    const double bound = talus::dynamics::turnBound(from, to);

    if (!(bound >= angle) || !(bound <= FACTOR * angle)) {
      ++failures;
      UNSCOPED_INFO("pair " << pair << " angle " << angle << " bound " << bound);
    }

    if (angle > 0.0) {
      widest = std::max(widest, bound / angle);
    }
  }

  INFO("widest bound over the turn " << widest);
  CHECK(failures == 0u);
  CHECK(widest <= FACTOR);
}

TEST_CASE(
  "a fast box dropped onto a thin static plate lands and comes to rest",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf plate = plateField();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, plate, 0.5);
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    half,
    1.5,
    glm::dvec3(0.0, 0.22, 0.0),
    glm::dvec3(0.0, -20.0, 0.0),
    0.5,
    3u
  );

  double lowestTop = 1.0;
  glm::dvec3 settled(0.0);
  double drift = 0.0;
  double peak = 0.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
    const glm::dmat3 turn = glm::mat3_cast(box.rotation);
    const double reach = half.y
                         * (std::abs(turn[0][1]) + std::abs(turn[1][1])
                            + std::abs(turn[2][1]));

    lowestTop = std::min(lowestTop, box.position.y + reach);

    if (frame + 1u == 60u) {
      settled = box.position;
    } else if (frame + 1u > 60u) {
      drift = std::max(drift, glm::length(box.position - settled));
      peak = std::max(peak, glm::length(box.velocity));
    }
  }

  CHECK(lowestTop >= -0.02);
  CHECK(drift < 0.01);
  CHECK(peak < 0.05);
}

TEST_CASE(
  "lattice points under a box push straight up even on the box's edges",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const glm::dvec3 half(0.025);
  const talus::field::SampledSdf crate = boxField(half, 0.005);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addLattice(
    bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    &blockField
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    0.5,
    glm::dvec3(0.0, 0.175, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const std::vector<talus::dynamics::BodyPair> pairs =
    talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<talus::dynamics::Contact> contacts =
    talus::dynamics::narrowphase(bodies, pairs, 0.02);
  uint32_t touching = 0u;

  for (const talus::dynamics::Contact& contact : contacts) {
    if (contact.gap > 1.0e-6
        || contact.a.side != talus::dynamics::Side::Lattice) {
      continue;
    }

    touching += 1u;
    CHECK(glm::dot(contact.basis[0], glm::dvec3(0.0, -1.0, 0.0)) >= 0.999);
  }

  CHECK(touching == 9u);
}

TEST_CASE(
  "a box on a lattice block is supported at its own bottom corners",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const glm::dvec3 half(0.025);
  const talus::field::SampledSdf crate = boxField(half, 0.005);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addLattice(
    bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    &blockField
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    0.5,
    glm::dvec3(0.0, 0.175, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const std::vector<talus::dynamics::BodyPair> pairs =
    talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<talus::dynamics::Contact> contacts =
    talus::dynamics::narrowphase(bodies, pairs, 0.02);
  uint32_t corners = 0u;

  for (const talus::dynamics::Contact& contact : contacts) {
    if (contact.a.side != talus::dynamics::Side::Rigid
        || contact.b.side != talus::dynamics::Side::Lattice) {
      continue;
    }

    const glm::dvec3& local = contact.a.local;

    if (std::abs(std::abs(local.x) - half.x) <= 1.0e-9
        && std::abs(local.y + half.y) <= 1.0e-9
        && std::abs(std::abs(local.z) - half.z) <= 1.0e-9) {
      corners += 1u;
      CHECK(std::abs(contact.gap) <= 1.0e-3);
      CHECK(glm::dot(contact.basis[0], glm::dvec3(0.0, 1.0, 0.0)) >= 0.99);
    }
  }

  CHECK(corners == 4u);
}

namespace {

struct RestRecord {
  double peak = 0.0;
  double drift = 0.0;
  double energyRatio = 0.0;
  double milliseconds = 0.0;
  bool finite = true;
};

std::shared_ptr<const talus::PreparedMesh> anchoredExceptFarthest(
  const std::shared_ptr<const talus::PreparedMesh>& mesh
) {
  auto copy = std::make_shared<talus::PreparedMesh>();
  uint32_t farthest = 0u;
  double distance = -1.0;

  *copy = *mesh;
  copy->anchored.assign(copy->restPositions.size(), 1u);

  for (uint32_t node = 0u; node != copy->restPositions.size(); ++node) {
    const double away = glm::length(
      glm::dvec3(copy->restPositions[node]) - glm::dvec3(0.0, 0.025, 0.0)
    );

    if (away > distance) {
      distance = away;
      farthest = node;
    }
  }

  copy->anchored[farthest] = 0u;

  return copy;
}

std::shared_ptr<const talus::PreparedMesh> giveBlock(
  bool anchored,
  std::span<const talus::PhysicalMaterial> lanes,
  talus::field::SampledSdf& field
) {
  const auto block =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, field);

  return anchored ? anchoredExceptFarthest(block) : block;
}

struct GiveScene {
  explicit GiveScene(
    bool anchored = false,
    const talus::PhysicalMaterial& material = rubber()
  )
    : lanes{material}, mesh(giveBlock(anchored, lanes, blockField)) {
    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    talus::dynamics::addLattice(
      world.bodies,
      mesh,
      lanes,
      glm::dvec3(0.0, 0.125, 0.0),
      glm::dvec3(0.0),
      0.5,
      &blockField
    );
  }

  GiveScene(const GiveScene&) = delete;
  GiveScene& operator=(const GiveScene&) = delete;

  void addCrate(const glm::dvec3& position, double mass) {
    talus::dynamics::addRigidBox(
      world.bodies,
      crate,
      glm::dvec3(0.025),
      mass,
      position,
      glm::dvec3(0.0),
      0.5,
      3u
    );
  }

  void addPebble(const glm::dvec3& position) {
    talus::dynamics::addRigidSphere(
      world.bodies,
      pebble,
      0.01,
      0.005,
      position,
      glm::dvec3(0.0),
      0.5,
      0.0
    );
  }

  talus::field::SampledSdf floor = floorField(1.0);
  talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::field::SampledSdf pebble = sphereField(0.01, 0.0025);
  talus::field::SampledSdf blockField;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
  talus::dynamics::World world;
};

RestRecord measureRest(talus::dynamics::World& world) {
  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  std::vector<glm::dvec3> settled(world.bodies.rigids.size(), glm::dvec3(0.0));
  RestRecord record;
  double elapsed = 0.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    const auto before = std::chrono::steady_clock::now();
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - before
    )
                 .count();
    record.finite = record.finite && !report.nonFinite;
    record.energyRatio = std::max(
      record.energyRatio,
      talus::dynamics::energyOf(world.bodies, g).total() / start
    );

    for (size_t body = 0u; body != world.bodies.rigids.size(); ++body) {
      const talus::dynamics::RigidBody& rigid = world.bodies.rigids[body];

      if (frame == 59u) {
        settled[body] = rigid.position;
      } else if (frame > 59u) {
        record.peak = std::max(record.peak, glm::length(rigid.velocity));
        record.drift =
          std::max(record.drift, glm::length(rigid.position - settled[body]));
      }
    }
  }

  record.milliseconds = elapsed / 360.0;

  return record;
}

bool passesRest(const RestRecord& record) {
  return record.finite && record.drift < 0.01 && record.peak < 0.05
         && record.energyRatio <= 1.02;
}

void printGiveRow(const char* label, const RestRecord& record) {
  std::printf(
    "givecase %s peak %.5f drift %.5f energy %.5f ms %.3f pass %d\n",
    label,
    record.peak,
    record.drift,
    record.energyRatio,
    record.milliseconds,
    passesRest(record) ? 1 : 0
  );
}

}

TEST_CASE(
  "a crate off the node grid rests on a lattice block and creates no energy",
  "[dynamics]"
) {
  GiveScene scene;

  scene.world.parameters.sleep = false;
  scene.addCrate(glm::dvec3(0.003, 0.175, 0.002), 0.5);

  const RestRecord record = measureRest(scene.world);

  REQUIRE(record.finite);
  CHECK(record.drift < 0.01);
  CHECK(record.peak < 0.05);
  CHECK(record.energyRatio <= 1.02);
}

TEST_CASE("lattice give case table", "[.][givecases]") {
  {
    GiveScene scene;

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.5);
    printGiveRow("1 aligned", measureRest(scene.world));
  }
  {
    GiveScene scene;

    scene.addCrate(glm::dvec3(0.003, 0.175, 0.002), 0.5);
    printGiveRow("2 offgrid", measureRest(scene.world));
  }
  {
    GiveScene scene;

    scene.addCrate(glm::dvec3(0.0, 0.225, 0.0), 0.5);
    printGiveRow("4 dropped", measureRest(scene.world));
  }
  {
    GiveScene scene;

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 5.0);
    printGiveRow("5 heavy", measureRest(scene.world));
  }
  {
    GiveScene scene;

    scene.addCrate(glm::dvec3(-0.03, 0.175, 0.0), 0.5);
    scene.addCrate(glm::dvec3(0.03, 0.175, 0.0), 0.5);
    printGiveRow("6 shared", measureRest(scene.world));
  }
  {
    GiveScene scene;

    scene.addPebble(glm::dvec3(0.0, 0.16, 0.0));
    printGiveRow("7 pebble", measureRest(scene.world));
  }
}

TEST_CASE("tier switching before rows", "[.][tierbefore]") {
  {
    GiveScene scene(false, softRubber());

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.8);
    printGiveRow("before 4 soft block, 0.8 kg crate", measureRest(scene.world));
  }
  {
    GiveScene scene(false, softRubber());

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.4);
    printGiveRow("before 6 soft block, 0.4 kg crate", measureRest(scene.world));
  }
  {
    const std::vector<talus::PhysicalMaterial> lanes{softRubber()};
    talus::field::SampledSdf matField;
    const auto mat =
      latticeBlock(glm::dvec3(0.15, 0.0125, 0.1), 0.025, lanes, matField);
    const talus::field::SampledSdf floor = floorField(1.0);
    const talus::field::SampledSdf left = ridgeField(
      glm::vec3(-0.12f, 0.11f, 0.0f),
      glm::vec3(0.02f, 0.01f, 0.15f),
      0.005
    );
    const talus::field::SampledSdf right = ridgeField(
      glm::vec3(0.12f, 0.11f, 0.0f),
      glm::vec3(0.02f, 0.01f, 0.15f),
      0.005
    );
    const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
    talus::dynamics::World world;

    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    talus::dynamics::addStatic(world.bodies, left, 0.5);
    talus::dynamics::addStatic(world.bodies, right, 0.5);
    talus::dynamics::addLattice(
      world.bodies,
      mat,
      lanes,
      glm::dvec3(0.0, 0.1325, 0.0),
      glm::dvec3(0.0),
      0.5,
      &matField
    );
    talus::dynamics::addRigidBox(
      world.bodies,
      crate,
      glm::dvec3(0.025),
      0.5,
      glm::dvec3(0.0, 0.17, 0.0),
      glm::dvec3(0.0),
      0.5,
      3u
    );
    printGiveRow("before 3b mat on ridges, 0.5 kg crate", measureRest(world));
  }
}

TEST_CASE(
  "a contact keeps its key and its body when an earlier body leaves the set",
  "[dynamics]"
) {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(-0.3, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );
  talus::dynamics::addRigidBox(
    bodies,
    crate,
    half,
    1.0,
    glm::dvec3(0.3, 0.15, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  const std::vector<double> stillBefore(2u, 0.0);
  const std::vector<talus::dynamics::BodyPair> before{
    {{talus::dynamics::Side::Rigid, 1u}, {talus::dynamics::Side::Static, 0u}}
  };
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, before);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, stillBefore, 0.0);

  const uint32_t slot =
    cache.find(talus::dynamics::pairKey(bodies, before[0].first, before[0].second));

  REQUIRE(slot != talus::dynamics::noSlot);

  const std::vector<talus::dynamics::Contact> first = cache.records(slot);

  REQUIRE(!first.empty());

  bodies.rigids.erase(bodies.rigids.begin());
  cache.renumber(talus::dynamics::Side::Rigid, 0u);

  const std::vector<double> stillAfter(1u, 0.0);
  const std::vector<talus::dynamics::BodyPair> after{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Static, 0u}}
  };

  cache.sync(bodies, after);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, stillAfter, 0.0);

  REQUIRE(cache.find(talus::dynamics::pairKey(bodies, after[0].first, after[0].second)) == slot);

  const std::vector<talus::dynamics::Contact> kept = cache.records(slot);

  REQUIRE(kept.size() == first.size());

  for (size_t index = 0u; index != kept.size(); ++index) {
    CHECK(kept[index].key == first[index].key);
    CHECK(kept[index].a.side == talus::dynamics::Side::Rigid);
    CHECK(kept[index].a.body == 0u);
  }
}

TEST_CASE(
  "a cell's rest stiffness matches finite differences of its corner forces",
  "[dynamics]"
) {
  const glm::dvec3 half(0.0125, 0.01, 0.015);
  const double volume = 8.0 * half.x * half.y * half.z;
  const talus::dynamics::Moduli moduli = talus::dynamics::moduliOf(rubber());
  talus::dynamics::Corners rest{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    rest[corner] = glm::dvec3(
      (corner & 1u) != 0u ? half.x : -half.x,
      (corner & 2u) != 0u ? half.y : -half.y,
      (corner & 4u) != 0u ? half.z : -half.z
    );
  }

  const auto stiffness = talus::dynamics::restStiffness(half, volume, moduli);
  const double step = 1.0e-7 * half.x;
  double largest = 0.0;
  double worst = 0.0;

  for (uint32_t moved = 0u; moved != 8u; ++moved) {
    for (int axis = 0; axis != 3; ++axis) {
      talus::dynamics::Corners plus = rest;
      talus::dynamics::Corners minus = rest;

      plus[moved][axis] += step;
      minus[moved][axis] -= step;

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        const glm::dvec3 column =
          (talus::dynamics::nodeTerms(plus, rest, half, volume, moduli, corner)
             .gradient
           - talus::dynamics::nodeTerms(minus, rest, half, volume, moduli, corner)
               .gradient)
          / (2.0 * step);
        const glm::dvec3 exact = stiffness[corner][moved][axis];

        for (int row = 0; row != 3; ++row) {
          largest = std::max(largest, std::abs(exact[row]));
          worst = std::max(worst, std::abs(exact[row] - column[row]));
        }
      }
    }
  }

  CHECK(worst <= 1.0e-6 * largest);
}

TEST_CASE(
  "a free block squeezed between equal pressures shortens by F L over E A",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf field;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, field);
  const std::vector<talus::dynamics::Moduli> moduli(
    mesh->hexes.size(),
    talus::dynamics::moduliOf(rubber())
  );
  const talus::dynamics::RestBody rest = talus::dynamics::restBodyOf(*mesh);
  const auto factor = talus::dynamics::factorScreen(*mesh, moduli);
  glm::dvec3 low(1.0e30);
  glm::dvec3 high(-1.0e30);

  for (const glm::vec3& position : mesh->restPositions) {
    low = glm::min(low, glm::dvec3(position));
    high = glm::max(high, glm::dvec3(position));
  }

  const double load = 4.905;
  std::vector<glm::dvec3> forces(mesh->restPositions.size(), glm::dvec3(0.0));
  double topShare = 0.0;
  double bottomShare = 0.0;

  for (size_t node = 0u; node != forces.size(); ++node) {
    const glm::dvec3 position(mesh->restPositions[node]);
    const auto edge = [](double value, double a, double b) {
      return std::abs(value - a) < 1.0e-9 || std::abs(value - b) < 1.0e-9 ? 0.5
                                                                          : 1.0;
    };
    const double share =
      edge(position.x, low.x, high.x) * edge(position.z, low.z, high.z);

    if (std::abs(position.y - high.y) < 1.0e-9) {
      forces[node].y = -share;
      topShare += share;
    } else if (std::abs(position.y - low.y) < 1.0e-9) {
      forces[node].y = share;
      bottomShare += share;
    }
  }

  for (glm::dvec3& force : forces) {
    force.y *= load / (force.y < 0.0 ? topShare : bottomShare);
  }

  const talus::dynamics::ScreenResult result =
    talus::dynamics::screenLoad(*factor, rest, *mesh, forces);
  double top = 0.0;
  double bottom = 0.0;
  double topCount = 0.0;
  double bottomCount = 0.0;

  for (size_t node = 0u; node != forces.size(); ++node) {
    const double y = mesh->restPositions[node].y;

    if (std::abs(y - high.y) < 1.0e-9) {
      top += result.displacement[node].y;
      topCount += 1.0;
    } else if (std::abs(y - low.y) < 1.0e-9) {
      bottom += result.displacement[node].y;
      bottomCount += 1.0;
    }
  }

  const double shortening = bottom / bottomCount - top / topCount;
  const double expected = load * (high.y - low.y)
                          / (static_cast<double>(rubber().youngsModulus)
                             * (high.x - low.x) * (high.z - low.z));

  CHECK(std::abs(shortening - expected) <= 1.0e-3 * expected);
}

TEST_CASE("a load that only accelerates a free body makes no dent", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf field;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, field);
  const std::vector<talus::dynamics::Moduli> moduli(
    mesh->hexes.size(),
    talus::dynamics::moduliOf(rubber())
  );
  const talus::dynamics::RestBody rest = talus::dynamics::restBodyOf(*mesh);
  const auto factor = talus::dynamics::factorScreen(*mesh, moduli);
  const glm::dvec3 linear(1.0, 2.0, -3.0);
  const glm::dvec3 angular(4.0, -1.0, 2.0);
  std::vector<glm::dvec3> forces(mesh->restPositions.size());

  for (size_t node = 0u; node != forces.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh->restPositions[node]) - rest.centre;

    forces[node] = rest.mass[node] * (linear + glm::cross(angular, arm));
  }

  const talus::dynamics::ScreenResult result =
    talus::dynamics::screenLoad(*factor, rest, *mesh, forces);

  CHECK(result.dent < 1.0e-9);
}

TEST_CASE("a rigid body's field frame moves the surface its partners touch", "[dynamics]") {
  const talus::field::SampledSdf ball = sphereField(0.05, 0.005);
  const std::vector<glm::dvec3> origin{glm::dvec3(0.0)};
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBody(
    bodies,
    ball,
    origin,
    1.0,
    glm::dvec3(1.0e-3),
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  talus::dynamics::addRigidBody(
    bodies,
    ball,
    origin,
    1.0,
    glm::dvec3(1.0e-3),
    glm::dvec3(-0.09, 0.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  bodies.rigids[0].fieldOrigin = glm::dvec3(0.03, 0.0, 0.0);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 1u}, {talus::dynamics::Side::Rigid, 0u}}
  };
  const std::vector<double> still(2u, 0.0);
  const std::vector<talus::dynamics::Contact> contacts =
    talus::dynamics::narrowphase(bodies, pairs, 0.02, still);

  REQUIRE(contacts.size() == 1u);
  CHECK(std::abs(contacts[0].gap - 0.01) <= 1.0e-3);
  CHECK(contacts[0].basis[0].x < -0.99);
}

TEST_CASE(
  "a switchable body starts rigid with the lattice's mass, centre and inertia",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::BodySet bodies;
  const glm::dvec3 offset(0.0, 0.125, 0.0);
  const uint32_t id = talus::dynamics::addSwitchable(
    bodies,
    mesh,
    lanes,
    blockField,
    offset,
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  REQUIRE(bodies.rigids.size() == 1u);
  REQUIRE(bodies.switchables.size() == 1u);
  CHECK(bodies.lattices.empty());

  const talus::dynamics::RigidBody& rigid = bodies.rigids[0];
  double mass = 0.0;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    mass += mesh->nodeMass[node];
    centre += static_cast<double>(mesh->nodeMass[node])
              * glm::dvec3(mesh->restPositions[node]);
  }

  centre /= mass;

  glm::dmat3 inertia(0.0);

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh->restPositions[node]) - centre;

    inertia += static_cast<double>(mesh->nodeMass[node])
               * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const glm::dmat3 world = talus::dynamics::worldInertia(rigid);
  double largest = 0.0;
  double worst = 0.0;

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      largest = std::max(largest, std::abs(inertia[column][row]));
      worst = std::max(worst, std::abs(inertia[column][row] - world[column][row]));
    }
  }

  CHECK(rigid.id == id);
  CHECK(std::abs(rigid.mass - mass) <= 1.0e-12 * mass);
  CHECK(glm::length(rigid.position - (offset + centre)) <= 1.0e-12);
  CHECK(worst <= 1.0e-9 * largest);
  CHECK(!rigid.samples.empty());
}

TEST_CASE("a switchable block rests on a floor as a rigid body", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  (void)talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.126, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  const RestRecord record = measureRest(world);
  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  REQUIRE(record.finite);
  CHECK(record.drift < 0.01);
  CHECK(record.peak < 0.05);
  CHECK(record.energyRatio <= 1.02);
  CHECK(report.strainstepIslands == 0u);
  CHECK(world.bodies.lattices.empty());
}

namespace {

struct TierScene {
  explicit TierScene(const talus::PhysicalMaterial& material)
    : lanes{material},
      mesh(latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField)) {
    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    block = talus::dynamics::addSwitchable(
      world.bodies,
      mesh,
      lanes,
      blockField,
      glm::dvec3(0.0, 0.125, 0.0),
      glm::dvec3(0.0),
      0.5,
      0.0
    );
  }

  TierScene(const TierScene&) = delete;
  TierScene& operator=(const TierScene&) = delete;

  uint32_t addCrate(const glm::dvec3& position, double mass) {
    const uint32_t index = talus::dynamics::addRigidBox(
      world.bodies,
      crate,
      glm::dvec3(0.025),
      mass,
      position,
      glm::dvec3(0.0),
      0.5,
      3u
    );

    return world.bodies.rigids[index].id;
  }

  void addPebble(const glm::dvec3& position) {
    talus::dynamics::addRigidSphere(
      world.bodies,
      pebble,
      0.01,
      0.005,
      position,
      glm::dvec3(0.0),
      0.5,
      0.0
    );
  }

  const talus::dynamics::SwitchableBody& record() const {
    return world.bodies.switchables[0];
  }

  bool promoted() const {
    return record().tier == talus::dynamics::Tier::Lattice;
  }

  talus::field::SampledSdf floor = floorField(1.0);
  talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::field::SampledSdf pebble = sphereField(0.01, 0.0025);
  talus::field::SampledSdf blockField;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
  talus::dynamics::World world;
  uint32_t block = 0u;
};

}

TEST_CASE(
  "a light crate keeps a switchable block rigid with a small screened dent",
  "[dynamics]"
) {
  TierScene scene(rubber());

  scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.5);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  CHECK(!scene.promoted());
  CHECK(scene.record().checks >= 1u);
  CHECK(scene.record().dent > 0.0);
  CHECK(scene.record().dent < scene.world.parameters.promoteDent);
}

TEST_CASE(
  "the screen checks again only when the load changes by more than a tenth",
  "[dynamics]"
) {
  TierScene scene(rubber());

  scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.5);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  const uint32_t settled = scene.record().checks;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  CHECK(scene.record().checks == settled);

  scene.addCrate(glm::dvec3(0.06, 0.175, 0.0), 0.5);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  CHECK(scene.record().checks > settled);
}

TEST_CASE("promotion keeps a switchable body's momentum", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.3, -0.2, 0.1),
    0.5,
    0.0
  );
  talus::dynamics::RigidBody& rigid = world.bodies.rigids[0];

  rigid.rotation =
    glm::angleAxis(0.4, glm::normalize(glm::dvec3(1.0, 2.0, 0.5))) * rigid.rotation;
  rigid.angularVelocity = glm::dvec3(0.5, 1.0, -0.7);

  const glm::dvec3 momentum = rigid.mass * rigid.velocity;
  const glm::dvec3 spin = talus::dynamics::worldInertia(rigid) * rigid.angularVelocity;
  const glm::dvec3 centre = rigid.position;

  talus::dynamics::promote(world, id);

  REQUIRE(world.bodies.rigids.empty());
  REQUIRE(world.bodies.lattices.size() == 1u);

  const talus::dynamics::LatticeBody& lattice = world.bodies.lattices[0];
  double mass = 0.0;
  glm::dvec3 moved(0.0);
  glm::dvec3 middle(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double nodeMass = lattice.mesh->nodeMass[node];

    mass += nodeMass;
    moved += nodeMass * lattice.velocity[node];
    middle += nodeMass * lattice.position[node];
  }

  middle /= mass;

  glm::dvec3 turning(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    turning += static_cast<double>(lattice.mesh->nodeMass[node])
               * glm::cross(lattice.position[node] - middle, lattice.velocity[node]);
  }

  CHECK(lattice.id == id);
  CHECK(glm::length(moved - momentum) <= 1.0e-12 * glm::length(momentum));
  CHECK(glm::length(middle - centre) <= 1.0e-12);
  CHECK(glm::length(turning - spin) <= 1.0e-9 * glm::length(spin));
}

TEST_CASE("a heavy crate promotes a switchable block after three frames", "[dynamics]") {
  TierScene scene(softRubber());
  const uint32_t crate = scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 2.0);
  int above = -1;
  int promoted = -1;

  for (int frame = 0; frame != 60 && promoted < 0; ++frame) {
    (void)talus::dynamics::step(scene.world);

    if (above < 0 && scene.record().dent >= scene.world.parameters.promoteDent) {
      above = frame;
    }

    if (scene.promoted()) {
      promoted = frame;
    }
  }

  REQUIRE(above >= 0);
  REQUIRE(promoted >= 0);
  CHECK(promoted == above + 2);

  double peak = 0.0;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    (void)talus::dynamics::step(scene.world);

    const auto ref = talus::dynamics::findBody(scene.world.bodies, crate);

    REQUIRE(ref.has_value());
    peak = std::max(
      peak,
      glm::length(scene.world.bodies.rigids[ref->index].velocity)
    );
  }

  CHECK(peak < 0.05);
}

TEST_CASE("demotion keeps a lattice body's momentum", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  talus::dynamics::promote(world, id);

  talus::dynamics::LatticeBody& lattice = world.bodies.lattices[0];
  double mass = 0.0;
  glm::dvec3 moved(0.0);
  glm::dvec3 middle(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double phase = 0.37 * static_cast<double>(node);

    lattice.position[node] += 1.0e-4 * glm::dvec3(std::sin(phase), std::cos(phase), 0.0);
    lattice.velocity[node] =
      glm::dvec3(0.2, -0.1, 0.05) + 0.01 * glm::dvec3(std::cos(phase), 0.0, std::sin(phase));
  }

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    const double nodeMass = lattice.mesh->nodeMass[node];

    mass += nodeMass;
    moved += nodeMass * lattice.velocity[node];
    middle += nodeMass * lattice.position[node];
  }

  middle /= mass;

  glm::dvec3 turning(0.0);

  for (size_t node = 0u; node != lattice.position.size(); ++node) {
    turning += static_cast<double>(lattice.mesh->nodeMass[node])
               * glm::cross(lattice.position[node] - middle, lattice.velocity[node]);
  }

  talus::dynamics::demote(world, id);

  REQUIRE(world.bodies.lattices.empty());
  REQUIRE(world.bodies.rigids.size() == 1u);

  const talus::dynamics::RigidBody& rigid = world.bodies.rigids[0];

  CHECK(rigid.id == id);
  CHECK(glm::length(rigid.mass * rigid.velocity - moved) <= 1.0e-12 * glm::length(moved));
  CHECK(glm::length(rigid.position - middle) <= 1.0e-12);
  CHECK(
    glm::length(talus::dynamics::worldInertia(rigid) * rigid.angularVelocity - turning)
    <= 1.0e-9 * glm::length(turning)
  );
}

TEST_CASE(
  "a free switchable body demotes in free fall and follows the ballistic path",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  const glm::dvec3 start(0.2, 0.0, -0.1);
  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 5.0, 0.0),
    start,
    0.5,
    0.0
  );

  talus::dynamics::promote(world, id);

  uint32_t demotions = 0u;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    demotions += talus::dynamics::step(world).demotions;
  }

  const auto ref = talus::dynamics::findBody(world.bodies, id);
  const glm::dvec3 expected = start + 10.0 * world.parameters.dt * world.parameters.gravity;

  REQUIRE(ref.has_value());
  CHECK(demotions == 1u);
  CHECK(ref->side == talus::dynamics::Side::Rigid);
  CHECK(glm::length(world.bodies.rigids[ref->index].velocity - expected) <= 1.0e-4);
}

TEST_CASE("two switchable blocks demote in the same frame", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  const uint32_t first = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(-1.0, 5.0, 0.0),
    glm::dvec3(0.1, 0.0, 0.0),
    0.5,
    0.0
  );
  const uint32_t second = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(1.0, 5.0, 0.0),
    glm::dvec3(-0.1, 0.0, 0.0),
    0.5,
    0.0
  );

  talus::dynamics::promote(world, first);
  talus::dynamics::promote(world, second);

  uint32_t demotions = 0u;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    demotions += talus::dynamics::step(world).demotions;
  }

  const auto a = talus::dynamics::findBody(world.bodies, first);
  const auto b = talus::dynamics::findBody(world.bodies, second);
  const glm::dvec3 fall = 10.0 * world.parameters.dt * world.parameters.gravity;

  REQUIRE(a.has_value());
  REQUIRE(b.has_value());
  CHECK(demotions == 2u);
  CHECK(
    glm::length(world.bodies.rigids[a->index].velocity - (glm::dvec3(0.1, 0.0, 0.0) + fall))
    <= 1.0e-4
  );
  CHECK(
    glm::length(world.bodies.rigids[b->index].velocity - (glm::dvec3(-0.1, 0.0, 0.0) + fall))
    <= 1.0e-4
  );
}

TEST_CASE("tier switching case table", "[.][tiercases]") {
  for (const glm::dvec3& offset :
       {glm::dvec3(0.0), glm::dvec3(0.003, 0.0, 0.002)}) {
    TierScene scene(rubber());

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0) + offset, 0.5);

    const RestRecord record = measureRest(scene.world);

    std::printf(
      "tiercase 1 offset %.3f promoted %d ",
      offset.x,
      scene.promoted() ? 1 : 0
    );
    printGiveRow("crate", record);
  }

  {
    TierScene scene(rubber());

    scene.addCrate(glm::dvec3(-0.03, 0.175, 0.0), 0.5);
    scene.addCrate(glm::dvec3(0.03, 0.175, 0.0), 0.5);

    const RestRecord record = measureRest(scene.world);

    std::printf("tiercase 2 shared promoted %d ", scene.promoted() ? 1 : 0);
    printGiveRow("crates", record);
  }

  {
    TierScene scene(rubber());

    scene.addPebble(glm::dvec3(0.0, 0.16, 0.0));

    const RestRecord record = measureRest(scene.world);

    std::printf("tiercase 2 pebble promoted %d ", scene.promoted() ? 1 : 0);
    printGiveRow("pebble", record);
  }

  {
    TierScene screened(softRubber());
    GiveScene settled(false, softRubber());

    screened.world.parameters.promoteDent = std::numeric_limits<double>::infinity();
    screened.addCrate(glm::dvec3(0.0, 0.175, 0.0), 1.4);
    settled.addCrate(glm::dvec3(0.0, 0.175, 0.0), 1.4);

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      (void)talus::dynamics::step(screened.world);
      (void)talus::dynamics::step(settled.world);
    }

    const double screen = screened.record().dent;
    const double lattice =
      talus::dynamics::fitLattice(
        settled.world.bodies.lattices[0],
        talus::dynamics::restBodyOf(*settled.mesh)
      )
        .dent;

    std::printf(
      "tiercase 3a screen %.6f lattice %.6f pass %d\n",
      screen,
      lattice,
      std::abs(screen - lattice) <= 0.1 * lattice ? 1 : 0
    );
  }

  {
    const std::vector<talus::PhysicalMaterial> lanes{softRubber()};
    talus::field::SampledSdf matField;
    const auto mat =
      latticeBlock(glm::dvec3(0.15, 0.0125, 0.1), 0.025, lanes, matField);
    const talus::field::SampledSdf floor = floorField(1.0);
    const talus::field::SampledSdf left = ridgeField(
      glm::vec3(-0.12f, 0.11f, 0.0f),
      glm::vec3(0.02f, 0.01f, 0.15f),
      0.005
    );
    const talus::field::SampledSdf right = ridgeField(
      glm::vec3(0.12f, 0.11f, 0.0f),
      glm::vec3(0.02f, 0.01f, 0.15f),
      0.005
    );
    const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
    talus::dynamics::World screened;
    talus::dynamics::World settled;

    for (talus::dynamics::World* world : {&screened, &settled}) {
      talus::dynamics::addStatic(world->bodies, floor, 0.5);
      talus::dynamics::addStatic(world->bodies, left, 0.5);
      talus::dynamics::addStatic(world->bodies, right, 0.5);
    }

    screened.parameters.promoteDent = std::numeric_limits<double>::infinity();
    (void)talus::dynamics::addSwitchable(
      screened.bodies,
      mat,
      lanes,
      matField,
      glm::dvec3(0.0, 0.1325, 0.0),
      glm::dvec3(0.0),
      0.5,
      0.0
    );
    talus::dynamics::addLattice(
      settled.bodies,
      mat,
      lanes,
      glm::dvec3(0.0, 0.1325, 0.0),
      glm::dvec3(0.0),
      0.5,
      &matField
    );

    for (talus::dynamics::World* world : {&screened, &settled}) {
      talus::dynamics::addRigidBox(
        world->bodies,
        crate,
        glm::dvec3(0.025),
        0.5,
        glm::dvec3(0.0, 0.17, 0.0),
        glm::dvec3(0.0),
        0.5,
        3u
      );
    }

    for (uint32_t frame = 0u; frame != 120u; ++frame) {
      (void)talus::dynamics::step(screened);
      (void)talus::dynamics::step(settled);
    }

    const double screen = screened.bodies.switchables[0].dent;
    const double lattice =
      talus::dynamics::fitLattice(
        settled.bodies.lattices[0],
        talus::dynamics::restBodyOf(*mat)
      )
        .dent;

    std::printf(
      "tiercase 3b screen %.6f lattice %.6f pass %d\n",
      screen,
      lattice,
      std::abs(screen - lattice) <= 0.1 * lattice ? 1 : 0
    );
  }

  {
    TierScene scene(softRubber());
    const uint32_t crate = scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 1.4);
    int above = -1;
    int switched = -1;
    glm::dvec3 position(0.0);
    glm::dquat rotation(1.0, 0.0, 0.0, 0.0);

    for (int frame = 0; frame != 60 && switched < 0; ++frame) {
      const auto ref = talus::dynamics::findBody(scene.world.bodies, scene.block);

      if (ref && ref->side == talus::dynamics::Side::Rigid) {
        position = scene.world.bodies.rigids[ref->index].position;
        rotation = scene.world.bodies.rigids[ref->index].rotation;
      }

      (void)talus::dynamics::step(scene.world);

      if (above < 0 && scene.record().dent >= scene.world.parameters.promoteDent) {
        above = frame;
      }

      if (scene.promoted()) {
        switched = frame;
      }
    }

    double jump = 0.0;
    double tolerance = 0.0;

    if (switched >= 0) {
      const talus::dynamics::SwitchableBody& body = scene.record();
      const talus::dynamics::LatticeBody& lattice = scene.world.bodies.lattices[0];
      const glm::dmat3 meshToWorld =
        glm::mat3_cast(rotation) * glm::transpose(glm::mat3_cast(body.rest.axes));

      for (size_t node = 0u; node != lattice.position.size(); ++node) {
        const glm::dvec3 start =
          position
          + meshToWorld
              * (glm::dvec3(scene.mesh->restPositions[node]) - body.rest.centre
                 + body.displacement[node]);

        jump = std::max(jump, glm::length(lattice.position[node] - start));
      }

      tolerance = std::max(0.1 * body.dent * body.rest.thickness, 0.002);
    }

    const RestRecord record = measureRest(scene.world);

    std::printf(
      "tiercase 4 screen %.6f above %d switched %d jump %.6f tolerance %.6f jumpPass %d promoted %d ",
      scene.record().dent,
      above,
      switched,
      jump,
      tolerance,
      switched >= 0 && jump <= tolerance ? 1 : 0,
      scene.promoted() ? 1 : 0
    );
    printGiveRow("soft block, 1.4 kg crate", record);
    (void)crate;
  }

  {
    talus::PhysicalMaterial medium = rubber();

    medium.youngsModulus = 1.6e5f;

    TierScene scene(medium);
    const uint32_t crate = scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 6.0);
    int promoted = -1;

    for (int frame = 0; frame != 120; ++frame) {
      (void)talus::dynamics::step(scene.world);

      if (promoted < 0 && scene.promoted()) {
        promoted = frame;
      }
    }

    int demoted = -1;

    (void)talus::dynamics::removeBody(scene.world, crate);

    for (int frame = 0; frame != 120 && demoted < 0; ++frame) {
      if (talus::dynamics::step(scene.world).demotions > 0u) {
        demoted = frame;
      }
    }

    std::printf(
      "tiercase 5 medium block, 6 kg crate promoted at frame %d, crate removed, demoted at frame %d pass %d\n",
      promoted,
      demoted,
      promoted >= 0 && demoted >= 0 ? 1 : 0
    );
  }

  {
    const std::vector<talus::PhysicalMaterial> lanes{rubber()};
    talus::field::SampledSdf blockField;
    const auto mesh =
      latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
    talus::dynamics::World world;
    const glm::dvec3 start(0.2, 0.0, -0.1);
    const uint32_t id = talus::dynamics::addSwitchable(
      world.bodies,
      mesh,
      lanes,
      blockField,
      glm::dvec3(0.0, 5.0, 0.0),
      start,
      0.5,
      0.0
    );

    talus::dynamics::promote(world, id);

    uint32_t demotions = 0u;

    for (uint32_t frame = 0u; frame != 10u; ++frame) {
      demotions += talus::dynamics::step(world).demotions;
    }

    const auto ref = talus::dynamics::findBody(world.bodies, id);
    const glm::dvec3 expected =
      start + 10.0 * world.parameters.dt * world.parameters.gravity;
    const double error =
      ref && ref->side == talus::dynamics::Side::Rigid
        ? glm::length(world.bodies.rigids[ref->index].velocity - expected)
        : 1.0e30;

    std::printf(
      "tiercase 5 free fall demotions %u velocity error %.3e pass %d\n",
      demotions,
      error,
      demotions == 1u && error <= 1.0e-4 ? 1 : 0
    );
  }

  {
    TierScene scene(softRubber());
    uint32_t switches = 0u;

    scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.7);

    for (uint32_t frame = 0u; frame != 300u; ++frame) {
      const talus::dynamics::StepReport report = talus::dynamics::step(scene.world);

      switches += report.promotions + report.demotions;
    }

    std::printf(
      "tiercase 6 screen %.6f switches %u pass %d\n",
      scene.record().dent,
      switches,
      switches <= 1u ? 1 : 0
    );
  }

  for (const double speed : {1.0, 10.0}) {
    const std::vector<talus::PhysicalMaterial> lanes{rubber()};
    talus::field::SampledSdf ballField;
    const auto ball = latticeBall(0.1, 0.05, lanes, ballField);
    const talus::field::SampledSdf floor = floorField(1.0);
    talus::dynamics::World world;
    uint32_t promotions = 0u;

    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    (void)talus::dynamics::addSwitchable(
      world.bodies,
      ball,
      lanes,
      ballField,
      glm::dvec3(0.0, 0.21, 0.0),
      glm::dvec3(0.0, -speed, 0.0),
      0.5,
      0.0
    );

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      promotions += talus::dynamics::step(world).promotions;
    }

    std::printf(
      "tiercase 7 speed %.1f promotions %u pass %d\n",
      speed,
      promotions,
      promotions == 0u ? 1 : 0
    );
  }

  for (const glm::dvec3& half :
       {glm::dvec3(0.1, 0.025, 0.1), glm::dvec3(0.225, 0.05, 0.225)}) {
    const std::vector<talus::PhysicalMaterial> lanes{rubber()};
    talus::field::SampledSdf field;
    const auto mesh = latticeBlock(half, 0.025, lanes, field);
    talus::dynamics::BodySet bodies;
    const auto before = std::chrono::steady_clock::now();

    (void)talus::dynamics::addSwitchable(
      bodies,
      mesh,
      lanes,
      field,
      glm::dvec3(0.0),
      glm::dvec3(0.0),
      0.5,
      0.0
    );

    const double load = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - before
    )
                          .count();
    const talus::dynamics::SwitchableBody& body = bodies.switchables[0];
    std::vector<glm::dvec3> forces(body.rest.mass.size());

    for (size_t node = 0u; node != forces.size(); ++node) {
      forces[node] = body.rest.mass[node] * glm::dvec3(0.0, -9.81, 0.0);
    }

    const auto start = std::chrono::steady_clock::now();

    for (uint32_t call = 0u; call != 100u; ++call) {
      (void)talus::dynamics::screenLoad(*body.screen, body.rest, *mesh, forces);
    }

    const double each = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start
    )
                          .count()
                        / 100.0;

    std::printf(
      "tiercase 9 nodes %zu add ms %.3f screen ms %.4f\n",
      forces.size(),
      load,
      each
    );
  }
}

TEST_CASE("the screen holds a block where it touches the floor", "[dynamics]") {
  TierScene scene(softRubber());

  scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.1);

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  const talus::dynamics::SwitchableBody& body = scene.record();
  double low = 1.0e30;

  for (const glm::vec3& position : scene.mesh->restPositions) {
    low = std::min(low, static_cast<double>(position.y));
  }

  double bottom = 0.0;

  REQUIRE(body.displacement.size() == body.rest.mass.size());

  for (size_t node = 0u; node != body.displacement.size(); ++node) {
    if (std::abs(scene.mesh->restPositions[node].y - low) < 1.0e-9) {
      bottom = std::max(bottom, glm::length(body.displacement[node]));
    }
  }

  CHECK(!scene.promoted());
  CHECK(bottom <= 1.0e-9);
}

TEST_CASE(
  "a block resting on a rail one node row wide keeps a bounded screened dent",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{softRubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf rail = ridgeField(
    glm::vec3(0.025f, 0.09f, 0.0f),
    glm::vec3(0.003f, 0.01f, 0.15f),
    0.002
  );
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, rail, 0.5);
  (void)talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  double worst = 0.0;

  for (uint32_t frame = 0u; frame != 6u; ++frame) {
    (void)talus::dynamics::step(world);
    worst = std::max(worst, world.bodies.switchables[0].dent);
  }

  CHECK(std::isfinite(worst));
  CHECK(worst < 1.0);
}

TEST_CASE(
  "promotion seeds from the rest shape when the screened dent is out of range",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 1.0, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  talus::dynamics::SwitchableBody& body = world.bodies.switchables[0];

  body.displacement.assign(body.rest.mass.size(), glm::dvec3(0.0));
  body.displacement[0] = glm::dvec3(1.0e4, 0.0, 0.0);
  body.dent = 2.0e5;
  talus::dynamics::promote(world, id);

  REQUIRE(world.bodies.lattices.size() == 1u);

  double reach = 0.0;

  for (const glm::dvec3& position : world.bodies.lattices[0].position) {
    reach = std::max(reach, glm::length(position - glm::dvec3(0.0, 1.0, 0.0)));
  }

  CHECK(reach < 0.2);
}

TEST_CASE(
  "promotion leaves a crate it rests on in place",
  "[dynamics]"
) {
  const std::vector<talus::PhysicalMaterial> lanes{softRubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf stand = boxField(glm::dvec3(0.06, 0.025, 0.06), 0.005);
  const talus::field::SampledSdf weight = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t below = world.bodies.rigids[talus::dynamics::addRigidBox(
    world.bodies,
    stand,
    glm::dvec3(0.06, 0.025, 0.06),
    1.0,
    glm::dvec3(0.0, 0.125, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  )].id;

  (void)talus::dynamics::addSwitchable(
    world.bodies,
    mesh,
    lanes,
    blockField,
    glm::dvec3(0.0, 0.175, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  talus::dynamics::addRigidBox(
    world.bodies,
    weight,
    glm::dvec3(0.025),
    2.0,
    glm::dvec3(0.0, 0.225, 0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );

  bool promoted = false;

  for (uint32_t frame = 0u; frame != 60u && !promoted; ++frame) {
    promoted = talus::dynamics::step(world).promotions > 0u;
  }

  REQUIRE(promoted);

  double peak = 0.0;

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    (void)talus::dynamics::step(world);

    const auto ref = talus::dynamics::findBody(world.bodies, below);

    REQUIRE(ref.has_value());
    peak = std::max(peak, glm::length(world.bodies.rigids[ref->index].velocity));
  }

  CHECK(peak < 0.05);
}

TEST_CASE(
  "a promotion between frames leaves last frame's contacts naming the right bodies",
  "[dynamics]"
) {
  TierScene scene(rubber());

  scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 0.5);
  scene.addCrate(glm::dvec3(0.3, 0.125, 0.0), 0.5);

  for (uint32_t frame = 0u; frame != 5u; ++frame) {
    (void)talus::dynamics::step(scene.world);
  }

  const auto identify = [&](const talus::dynamics::ContactSide& side) -> int64_t {
    if (side.side == talus::dynamics::Side::Static) {
      return -1 - static_cast<int64_t>(side.body);
    }

    return side.side == talus::dynamics::Side::Rigid
             ? scene.world.bodies.rigids[side.body].id
             : scene.world.bodies.lattices[side.body].id;
  };
  const auto touchesBlock = [&](const talus::dynamics::Contact& contact) {
    return identify(contact.a) == scene.block || identify(contact.b) == scene.block;
  };
  const std::vector<talus::dynamics::Contact> earlier =
    talus::dynamics::contactRecords(scene.world);
  std::vector<std::pair<int64_t, int64_t>> before;

  for (const talus::dynamics::Contact& contact : earlier) {
    if (!touchesBlock(contact)) {
      before.emplace_back(identify(contact.a), identify(contact.b));
    }
  }

  REQUIRE(!before.empty());

  talus::dynamics::promote(scene.world, scene.block);

  const std::vector<talus::dynamics::Contact> later =
    talus::dynamics::contactRecords(scene.world);

  std::vector<std::pair<int64_t, int64_t>> after;
  bool inRange = true;

  for (const talus::dynamics::Contact& contact : later) {
    for (const talus::dynamics::ContactSide* side : {&contact.a, &contact.b}) {
      if (side->side == talus::dynamics::Side::Rigid) {
        inRange = inRange && side->body < scene.world.bodies.rigids.size();
      } else if (side->side == talus::dynamics::Side::Lattice) {
        inRange = inRange && side->body < scene.world.bodies.lattices.size();
      }
    }
  }

  REQUIRE(inRange);

  for (const talus::dynamics::Contact& contact : later) {
    if (!touchesBlock(contact)) {
      after.emplace_back(identify(contact.a), identify(contact.b));
    }
  }

  CHECK(after == before);
}

TEST_CASE("a box keeps its speed when its island moves to strainstep and back", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf cubeField;
  auto cube = latticeBlock(glm::dvec3(0.025), 0.025, lanes, cubeField);
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(world.bodies, crate, half, 2.0, glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addLattice(world.bodies, cube, lanes, glm::dvec3(-0.5, 0.125, 0.0), glm::dvec3(0.0), 0.5, &cubeField);

  const auto shiftCube = [&](const glm::dvec3& shift) {
    talus::dynamics::LatticeBody& body = world.bodies.lattices[0];

    for (size_t node = 0u; node != body.position.size(); ++node) {
      body.position[node] += shift;
      body.velocity[node] = glm::dvec3(0.0);
    }
  };

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const double start = talus::dynamics::energyOf(world.bodies, world.parameters.gravity).total();
  double peak = 1.0;
  double worstJump = 0.0;
  uint32_t strainstepFrames = 0u;

  for (uint32_t phase = 0u; phase != 3u; ++phase) {
    if (phase == 1u) {
      shiftCube(glm::dvec3(0.5 - 0.05 - 0.025 - 0.001, 0.0, 0.0));
    } else if (phase == 2u) {
      shiftCube(glm::dvec3(-0.5 + 0.05 + 0.025 + 0.001, 0.0, 0.0));
    }

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      const glm::dvec3 before = world.bodies.rigids[0].velocity;
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      REQUIRE(!report.nonFinite);
      strainstepFrames += phase == 1u && report.softstepIslands == 0u ? 1u : 0u;
      worstJump = std::max(worstJump, glm::length(world.bodies.rigids[0].velocity - before));
      peak = std::max(peak, talus::dynamics::energyOf(world.bodies, world.parameters.gravity).total() / start);
    }
  }

  CHECK(strainstepFrames == 60u);
  CHECK(worstJump < 0.05);
  CHECK(peak <= 1.02);
}

TEST_CASE("soft island cost scenes", "[.][islandcost]") {
  for (int scene = 0; scene != 2; ++scene) {
    const std::vector<talus::PhysicalMaterial> lanes{rubber()};
    talus::field::SampledSdf blockField;
    const glm::dvec3 half =
      scene == 0 ? glm::dvec3(0.1, 0.025, 0.1) : glm::dvec3(0.225, 0.05, 0.225);
    const auto mesh = latticeBlock(half, 0.025, lanes, blockField);
    const talus::field::SampledSdf floor = floorField(1.0);
    const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
    talus::dynamics::World world;

    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    talus::dynamics::addLattice(
      world.bodies,
      mesh,
      lanes,
      glm::dvec3(0.0, 0.1 + half.y, 0.0),
      glm::dvec3(0.0),
      0.5,
      &blockField
    );

    const int across = scene == 0 ? 1 : 4;

    for (int row = 0; row != across; ++row) {
      for (int column = 0; column != across; ++column) {
        const double x = scene == 0 ? 0.0 : -0.15 + 0.1 * row;
        const double z = scene == 0 ? 0.0 : -0.15 + 0.1 * column;

        talus::dynamics::addRigidBox(
          world.bodies,
          crate,
          glm::dvec3(0.025),
          0.5,
          glm::dvec3(x, 0.1 + 2.0 * half.y + 0.025, z),
          glm::dvec3(0.0),
          0.5,
          3u
        );
      }
    }

    for (uint32_t frame = 0u; frame != 30u; ++frame) {
      (void)talus::dynamics::step(world);
    }

    double elapsed = 0.0;
    uint32_t substeps = 0u;

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      const auto before = std::chrono::steady_clock::now();
      const talus::dynamics::StepReport report = talus::dynamics::step(world);

      elapsed += std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - before
      )
                   .count();
      substeps += report.substeps;
    }

    std::printf(
      "islandcost nodes %zu crates %zu ms %.3f substeps %.2f\n",
      world.bodies.lattices[0].position.size(),
      world.bodies.rigids.size(),
      elapsed / 60.0,
      substeps / 60.0
    );
  }
}

TEST_CASE("a zero-mass rigid box under a lattice block stays put", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh =
    latticeBlock(glm::dvec3(0.05, 0.025, 0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf stand = boxField(glm::dvec3(0.1, 0.025, 0.1), 0.005);
  talus::dynamics::World world;

  talus::dynamics::addRigidBox(
    world.bodies,
    stand,
    glm::dvec3(0.1, 0.025, 0.1),
    0.0,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    3u
  );
  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0, 0.051, 0.0),
    glm::dvec3(0.0),
    0.5,
    &blockField
  );

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
  double lowest = 1.0e30;

  for (const glm::dvec3& node : world.bodies.lattices[0].position) {
    lowest = std::min(lowest, node.y);
  }

  CHECK(glm::length(box.position) <= 1.0e-12);
  CHECK(glm::length(box.velocity) <= 1.0e-12);
  CHECK(lowest > 0.02);
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

uint64_t stateHash(const talus::dynamics::World& world) {
  uint64_t hash = 1469598103934665603ull;

  for (const talus::dynamics::LatticeBody& body : world.bodies.lattices) {
    for (size_t node = 0u; node != body.position.size(); ++node) {
      for (int axis = 0; axis != 3; ++axis) {
        hash = mixBits(hash, body.position[node][axis]);
        hash = mixBits(hash, body.velocity[node][axis]);
      }
    }
  }

  for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
    for (int axis = 0; axis != 3; ++axis) {
      hash = mixBits(hash, body.position[axis]);
      hash = mixBits(hash, body.velocity[axis]);
      hash = mixBits(hash, body.angularVelocity[axis]);
    }

    for (int axis = 0; axis != 4; ++axis) {
      hash = mixBits(hash, body.rotation[axis]);
    }
  }

  return hash;
}

std::vector<std::pair<uint32_t, uint32_t>> widthRuns() {
  std::vector<std::pair<uint32_t, uint32_t>> runs{{1u, 1u}, {1u, 2u}, {1u, 4u}, {1u, 8u}, {4u, 1u}, {4u, 8u}};

  if (talus::dynamics::hasAvx2()) {
    runs.emplace_back(8u, 1u);
    runs.emplace_back(8u, 8u);
  }

  return runs;
}

}

TEST_CASE("soft island state hash", "[.][speedhash]") {
  std::array<uint64_t, 3> first{};

  for (const auto& [width, workers] : widthRuns()) {
    for (int scene = 0; scene != 3; ++scene) {
      const std::vector<talus::PhysicalMaterial> lanes{rubber()};
      talus::field::SampledSdf blockField;
      const glm::dvec3 half = scene == 1 ? glm::dvec3(0.225, 0.05, 0.225)
                                         : glm::dvec3(0.1, 0.025, 0.1);
      const auto mesh = latticeBlock(half, 0.025, lanes, blockField);
      const talus::field::SampledSdf floor = floorField(1.0);
      const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
      talus::dynamics::World world;

      world.workerCount = workers;
      world.simdWidth = width;

      talus::dynamics::addStatic(world.bodies, floor, 0.5);
      talus::dynamics::addLattice(
        world.bodies,
        mesh,
        lanes,
        glm::dvec3(0.0, 0.1 + half.y + (scene == 2 ? 0.05 : 0.0), 0.0),
        glm::dvec3(0.0, scene == 2 ? -2.0 : 0.0, 0.0),
        0.5,
        &blockField
      );

      const int across = scene == 1 ? 4 : 1;

      for (int row = 0; row != across; ++row) {
        for (int column = 0; column != across; ++column) {
          const double x = scene == 1 ? -0.15 + 0.1 * row : 0.003;
          const double z = scene == 1 ? -0.15 + 0.1 * column : 0.002;

          talus::dynamics::addRigidBox(
            world.bodies,
            crate,
            glm::dvec3(0.025),
            0.5,
            glm::dvec3(x, 0.1 + 2.0 * half.y + 0.025 + (scene == 2 ? 0.1 : 0.0), z),
            glm::dvec3(0.0),
            0.5,
            3u
          );
        }
      }

      for (uint32_t frame = 0u; frame != 60u; ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
      }

      const uint64_t hash = stateHash(world);

      std::printf(
        "speedhash width %u workers %u scene %d hash %016llx\n",
        width,
        workers,
        scene,
        static_cast<unsigned long long>(hash)
      );

      if (width == 1u && workers == 1u) {
        first[static_cast<size_t>(scene)] = hash;
      } else {
        CHECK(hash == first[static_cast<size_t>(scene)]);
      }
    }
  }
}

TEST_CASE("profile scene: a pile of 1,000 boxes", "[.][pileprofile]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int row = 0; row != 10; ++row) {
    for (int column = 0; column != 10; ++column) {
      for (int level = 0; level != 10; ++level) {
        talus::dynamics::addRigidBox(
          world.bodies,
          brick,
          glm::dvec3(0.03),
          0.5,
          glm::dvec3(
            -0.3 + 0.0605 * row,
            0.13 + 0.0605 * level,
            -0.3 + 0.0605 * column
          ),
          glm::dvec3(0.0),
          0.5,
          3u
        );
      }
    }
  }

  double elapsed = 0.0;
  talus::dynamics::StepReport last;

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    const auto before = std::chrono::steady_clock::now();

    last = talus::dynamics::step(world);
    REQUIRE(!last.nonFinite);

    if (frame >= 30u) {
      elapsed += std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - before
      )
                   .count();
    }
  }

  std::printf(
    "pileprofile boxes %zu islands %u contacts %zu ms %.3f\n",
    world.bodies.rigids.size(),
    last.softstepIslands,
    static_cast<size_t>(world.manifolds.pointCount()),
    elapsed / 60.0
  );
}

TEST_CASE("rigid state hash", "[.][rigidhash]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const talus::field::SampledSdf block = boxField(glm::dvec3(0.05), 0.01);

  std::array<uint64_t, 3> first{};

  for (const auto& [width, workers] : widthRuns()) {
    for (int scene = 0; scene != 3; ++scene) {
      talus::dynamics::World world;

      world.workerCount = workers;
      world.simdWidth = width;

      talus::dynamics::addStatic(world.bodies, floor, 0.5);

      if (scene == 0) {
        for (int row = 0; row != 10; ++row) {
          for (int column = 0; column != 10; ++column) {
            for (int level = 0; level != 10; ++level) {
              talus::dynamics::addRigidBox(
                world.bodies,
                brick,
                glm::dvec3(0.03),
                0.5,
                glm::dvec3(
                  -0.3 + 0.0605 * row,
                  0.13 + 0.0605 * level,
                  -0.3 + 0.0605 * column
                ),
                glm::dvec3(0.0),
                0.5,
                3u
              );
            }
          }
        }
      } else if (scene == 1) {
        for (int row = 0; row != 16; ++row) {
          for (int column = 0; column != 16; ++column) {
            for (int level = 0; level != 4; ++level) {
              talus::dynamics::addRigidBox(
                world.bodies,
                brick,
                glm::dvec3(0.03),
                0.5,
                glm::dvec3(
                  -0.9 + 0.12 * row,
                  0.13 + 0.0605 * level,
                  -0.9 + 0.12 * column
                ),
                glm::dvec3(0.0),
                0.5,
                3u
              );
            }
          }
        }
      } else {
        for (int level = 0; level != 10; ++level) {
          for (int index = 0; index != 10 - level; ++index) {
            talus::dynamics::addRigidBox(
              world.bodies,
              block,
              glm::dvec3(0.05),
              1.0,
              glm::dvec3(
                -0.45 + 0.05 * level + 0.1 * index,
                0.15 + 0.1 * level,
                0.0
              ),
              glm::dvec3(0.0),
              0.6,
              3u
            );
          }
        }
      }

      for (uint32_t frame = 0u; frame != 60u; ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
      }

      const uint64_t hash = stateHash(world);

      std::printf(
        "rigidhash width %u workers %u scene %d hash %016llx\n",
        width,
        workers,
        scene,
        static_cast<unsigned long long>(hash)
      );

      if (width == 1u && workers == 1u) {
        first[static_cast<size_t>(scene)] = hash;
      } else {
        CHECK(hash == first[static_cast<size_t>(scene)]);
      }
    }
  }
}

namespace {

talus::dynamics::BodySet twoBoxesOnFloor() {
  static const talus::field::SampledSdf crate =
    boxField(glm::dvec3(0.05), 0.01);
  static const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.0, 0.2495, 0.0), glm::dvec3(0.0), 0.5, 3u);

  return bodies;
}

talus::dynamics::Contact pointRecord(
  talus::dynamics::Side sideA,
  uint32_t bodyA,
  talus::dynamics::Side sideB,
  uint32_t bodyB,
  uint64_t key,
  uint32_t manifold,
  double gap
) {
  talus::dynamics::Contact contact;

  contact.a.side = sideA;
  contact.a.body = bodyA;
  contact.a.local = glm::dvec3(0.01 * static_cast<double>(key), -0.05, 0.0);
  contact.b.side = sideB;
  contact.b.body = bodyB;
  contact.b.local = glm::dvec3(0.0, 0.05, 0.02 * static_cast<double>(key));
  contact.gap = gap;
  contact.friction = 0.5;
  contact.restitution = 0.25;
  contact.key = key;
  contact.manifold = manifold;
  contact.memory.normal = 3.0 + static_cast<double>(key);
  contact.memory.friction = glm::dvec2(0.5, -0.25);

  return contact;
}

const talus::dynamics::BodyRef floorRef{talus::dynamics::Side::Static, 0u};
const talus::dynamics::BodyRef lowerRef{talus::dynamics::Side::Rigid, 0u};
const talus::dynamics::BodyRef upperRef{talus::dynamics::Side::Rigid, 1u};

}

TEST_CASE("the store creates a pair the frame it appears and removes it the frame it leaves", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  const std::vector<talus::dynamics::BodyPair> both{{lowerRef, floorRef}, {lowerRef, upperRef}};
  const std::vector<talus::dynamics::BodyPair> one{{lowerRef, upperRef}};

  cache.sync(bodies, both);

  const uint32_t ground = cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef));
  const uint32_t stack = cache.find(talus::dynamics::pairKey(bodies, lowerRef, upperRef));

  REQUIRE(ground == 0u);
  REQUIRE(stack == 1u);
  CHECK(cache.at(ground).first.side == talus::dynamics::Side::Rigid);
  CHECK(cache.at(ground).second.side == talus::dynamics::Side::Static);
  CHECK(cache.at(stack).firstId < cache.at(stack).secondId);

  cache.sync(bodies, one);

  CHECK(!cache.live(ground));
  CHECK(cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef)) == talus::dynamics::noSlot);
  CHECK(cache.live(stack));

  cache.sync(bodies, both);

  CHECK(cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef)) == ground);
  CHECK(cache.size() == 2u);
}

TEST_CASE("each body lists its store pairs in the order they were made", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}, {lowerRef, upperRef}});

  const std::span<const uint32_t> lower = cache.pairsOf(bodies.rigids[0].id);
  const std::span<const uint32_t> upper = cache.pairsOf(bodies.rigids[1].id);

  REQUIRE(lower.size() == 2u);
  CHECK(lower[0] == 0u);
  CHECK(lower[1] == 1u);
  REQUIRE(upper.size() == 1u);
  CHECK(upper[0] == 1u);
}

TEST_CASE("records survive a round trip through the store", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  using talus::dynamics::Side;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, upperRef}});

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, lowerRef, upperRef));
  std::vector<talus::dynamics::Contact> records{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 1u, 7u, -0.001),
    pointRecord(Side::Rigid, 1u, Side::Rigid, 0u, 2u, 9u, 0.002),
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 3u, 7u, -0.003)
  };

  records[1].basis[0] = glm::dvec3(0.0, -1.0, 0.0);
  records[0].cached.valid = true;
  records[0].cached.relativePosition = glm::dvec3(0.0, 0.0995, 0.0);
  records[0].cached.separation = -0.001;
  records[1].cached.separation = 0.002;
  records[2].cached.separation = -0.003;
  cache.replaceManifolds(bodies, slot, records);

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);

  REQUIRE(pair.manifolds.size() == 2u);
  CHECK(!pair.manifolds[0].flipped);
  CHECK(pair.manifolds[0].count == 2u);
  CHECK(pair.manifolds[1].flipped);
  CHECK(pair.pose.valid);

  const std::vector<talus::dynamics::Contact> back = cache.records(slot);
  const std::array<size_t, 3> from{0u, 2u, 1u};

  REQUIRE(back.size() == 3u);

  for (size_t index = 0u; index != back.size(); ++index) {
    const talus::dynamics::Contact& original = records[from[index]];

    CHECK(back[index].a.side == original.a.side);
    CHECK(back[index].a.body == original.a.body);
    CHECK(back[index].a.local == original.a.local);
    CHECK(back[index].b.body == original.b.body);
    CHECK(back[index].b.local == original.b.local);
    CHECK(back[index].basis[0] == original.basis[0]);
    CHECK(back[index].gap == original.gap);
    CHECK(back[index].key == original.key);
    CHECK(back[index].memory.normal == original.memory.normal);
    CHECK(back[index].memory.friction == original.memory.friction);
    CHECK(back[index].friction == 0.5);
    CHECK(back[index].restitution == 0.25);
    CHECK(back[index].pair == pair.pair);
    CHECK(back[index].cached.separation == original.cached.separation);
  }

  CHECK(back[0].manifold == back[1].manifold);
  CHECK(back[0].manifold != back[2].manifold);
}

TEST_CASE("a rebuilt manifold keeps each point's memory by key", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  using talus::dynamics::Side;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}});

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef));

  cache.replaceManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 4u, 0u, 0.0),
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 5u, 0u, 0.0)
  });

  std::vector<talus::dynamics::Contact> fresh{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 5u, 0u, 0.001),
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 6u, 0u, 0.001)
  };

  for (talus::dynamics::Contact& contact : fresh) {
    contact.memory = talus::dynamics::ContactMemory{};
  }

  cache.rebuildManifolds(bodies, slot, fresh);

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);

  REQUIRE(pair.points.size() == 2u);
  CHECK(pair.manifoldPoints[0].key == 5u);
  CHECK(pair.points[0].memory.normal == 8.0);
  CHECK(pair.points[0].memory.friction == glm::dvec2(0.5, -0.25));
  CHECK(pair.manifoldPoints[1].key == 6u);
  CHECK(pair.points[1].memory.normal == 0.0);
}

TEST_CASE("a touching pair keeps its colour and a pair that stops touching gives its colour back", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  using talus::dynamics::Side;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}, {lowerRef, upperRef}});

  const uint32_t ground = cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef));
  const uint32_t stack = cache.find(talus::dynamics::pairKey(bodies, lowerRef, upperRef));

  CHECK(cache.at(stack).colour == talus::dynamics::noColour);

  cache.replaceManifolds(bodies, stack, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 1u, 0u, 0.0)
  });
  cache.replaceManifolds(bodies, ground, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 2u, 0u, 0.0)
  });

  CHECK(cache.at(stack).colour == 0u);
  CHECK(cache.at(ground).colour == 22u);

  cache.replaceManifolds(bodies, stack, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 3u, 0u, 0.0)
  });

  CHECK(cache.at(stack).colour == 0u);

  cache.replaceManifolds(bodies, stack, std::vector<talus::dynamics::Contact>{});

  CHECK(cache.at(stack).colour == talus::dynamics::noColour);

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, upperRef}});
  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, upperRef}, {lowerRef, floorRef}});

  const uint32_t again = cache.find(talus::dynamics::pairKey(bodies, lowerRef, floorRef));

  cache.replaceManifolds(bodies, again, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 2u, 0u, 0.0)
  });
  cache.replaceManifolds(bodies, stack, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 1u, 0u, 0.0)
  });

  CHECK(cache.at(again).colour == 22u);
  CHECK(cache.at(stack).colour == 0u);
}

TEST_CASE("a zero-mass body takes colours like a static one", "[dynamics]") {
  talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  using talus::dynamics::Side;

  bodies.rigids[0].mass = 0.0;
  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, upperRef}});

  const uint32_t stack = cache.find(talus::dynamics::pairKey(bodies, lowerRef, upperRef));

  cache.replaceManifolds(bodies, stack, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 1u, 0u, 0.0)
  });

  CHECK(cache.at(stack).colour == 22u);
}

TEST_CASE("tier switching removes exactly the switched body's store pairs", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  const talus::dynamics::BodyRef third{talus::dynamics::Side::Rigid, 2u};
  talus::dynamics::BodySet three = bodies;
  static const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);

  talus::dynamics::addRigidBox(three, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.5, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);
  cache.sync(three, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}, {lowerRef, upperRef}, {third, floorRef}});

  const uint64_t kept = talus::dynamics::pairKey(three, third, floorRef);

  cache.removeBody(three.rigids[1].id);
  cache.renumber(talus::dynamics::Side::Rigid, 1u);

  CHECK(cache.live(0u));
  CHECK(!cache.live(1u));
  REQUIRE(cache.find(kept) == 2u);
  CHECK(cache.at(2u).first.index == 1u);
  CHECK(cache.at(0u).first.index == 0u);
  CHECK(cache.pairsOf(three.rigids[1].id).empty());
}

TEST_CASE("a stored box pair keeps its contact points while the boxes move a little", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.0995, 0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Rigid, 1u}}
  };
  const std::vector<double> still(2u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[0].first, pairs[0].second));

  REQUIRE(slot != talus::dynamics::noSlot);

  const std::vector<talus::dynamics::Contact> first = cache.records(slot);

  REQUIRE(first.size() == 4u);

  talus::dynamics::RigidBody& upper = bodies.rigids[0];

  upper.position += glm::dvec3(0.003, -0.001, 0.0);
  upper.rotation = glm::angleAxis(0.01, glm::dvec3(0.0, 0.0, 1.0));
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const std::vector<talus::dynamics::Contact> kept = cache.records(slot);

  REQUIRE(kept.size() == first.size());

  for (size_t index = 0u; index != kept.size(); ++index) {
    const talus::dynamics::Contact& contact = kept[index];
    const glm::dvec3 apart = talus::dynamics::sidePoint(bodies, contact.a)
                             - talus::dynamics::sidePoint(bodies, contact.b);

    CHECK(contact.key == first[index].key);
    CHECK(contact.a.local == first[index].a.local);
    CHECK(contact.b.local == first[index].b.local);
    CHECK(contact.basis[0] == first[index].basis[0]);
    CHECK(std::abs(contact.gap - glm::dot(apart, contact.basis[0])) <= 1.0e-12);
  }

  upper.position += glm::dvec3(0.06, 0.0, 0.0);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const std::vector<talus::dynamics::Contact> rebuilt = cache.records(slot);
  bool moved = rebuilt.size() != kept.size();

  for (size_t index = 0u; !moved && index != rebuilt.size(); ++index) {
    moved = rebuilt[index].b.local != kept[index].b.local;
  }

  CHECK(moved);
}

TEST_CASE("a stored box on a static floor keeps its contact points while it moves a little", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Static, 0u}}
  };
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[0].first, pairs[0].second));
  const std::vector<talus::dynamics::Contact> first = cache.records(slot);

  REQUIRE(!first.empty());

  talus::dynamics::RigidBody& box = bodies.rigids[0];
  const glm::dvec3 start = box.position;
  const glm::dquat turn0 = box.rotation;

  box.position += glm::dvec3(0.003, -0.001, 0.0);
  box.rotation = glm::angleAxis(0.01, glm::dvec3(0.0, 0.0, 1.0));
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const std::vector<talus::dynamics::Contact> kept = cache.records(slot);

  REQUIRE(kept.size() == first.size());

  for (size_t index = 0u; index != kept.size(); ++index) {
    const talus::dynamics::Contact& contact = kept[index];
    const glm::dvec3 then = start + glm::mat3_cast(turn0) * contact.a.local;
    const glm::dvec3 now = box.position + glm::mat3_cast(box.rotation) * contact.a.local;

    CHECK(contact.key == first[index].key);
    CHECK(contact.a.local == first[index].a.local);
    CHECK(contact.basis[0] == first[index].basis[0]);
    CHECK(std::abs(contact.gap - (first[index].gap + glm::dot(now - then, contact.basis[0]))) <= 1.0e-12);
  }

  box.position += glm::dvec3(0.06, 0.0, 0.0);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const std::vector<talus::dynamics::Contact> rebuilt = cache.records(slot);

  REQUIRE(!rebuilt.empty());
  CHECK(kept.front().cached.relativePosition == start);
  CHECK(rebuilt.front().cached.relativePosition == box.position);
}

TEST_CASE("a stored box pair without points is not tested again while the boxes barely move", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.125, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Rigid, 1u}}
  };
  const std::vector<double> still(2u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[0].first, pairs[0].second));

  REQUIRE(slot != talus::dynamics::noSlot);
  REQUIRE(cache.records(slot).empty());

  bodies.rigids[1].position.y -= 0.01;
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  CHECK(cache.records(slot).empty());

  bodies.rigids[1].position.y -= 0.0105;
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  CHECK(!cache.records(slot).empty());
}

TEST_CASE("a stored box pair without points is tested again when the boxes will travel this frame", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.125, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Rigid, 1u}}
  };
  const std::vector<double> still(2u, 0.0);
  const std::vector<double> moving{0.0, 0.011};
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[0].first, pairs[0].second));

  REQUIRE(cache.records(slot).empty());

  bodies.rigids[1].position.y -= 0.01;
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, moving, 0.0);

  CHECK(!cache.records(slot).empty());
}

TEST_CASE("a stored box above a static floor is not tested again while it barely moves", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.175, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Static, 0u}}
  };
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[0].first, pairs[0].second));

  REQUIRE(slot != talus::dynamics::noSlot);
  REQUIRE(cache.records(slot).empty());

  bodies.rigids[0].position.y -= 0.01;
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  CHECK(cache.records(slot).empty());

  bodies.rigids[0].position.y -= 0.0105;
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  CHECK(!cache.records(slot).empty());
}

TEST_CASE("a stored box on a static floor gets one manifold for its floor contacts", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  REQUIRE(cache.size() == 1u);

  const std::vector<talus::dynamics::Contact> contacts = cache.records(0u);

  REQUIRE(contacts.size() == 4u);

  for (const talus::dynamics::Contact& contact : contacts) {
    CHECK(contact.manifold == contacts.front().manifold);
  }
}

TEST_CASE("a stored pair keeps a point's memory when it is tested again", "[dynamics]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxField(half, 0.01);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addRigidBox(bodies, crate, half, 1.0, glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs{
    {{talus::dynamics::Side::Rigid, 0u}, {talus::dynamics::Side::Static, 0u}}
  };
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  for (talus::dynamics::CachedContactPoint& point : cache.at(0u).points) {
    point.memory.normal = 7.0;
  }

  bodies.rigids[0].position += glm::dvec3(0.002, 0.0, 0.0);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  for (const talus::dynamics::CachedContactPoint& point : cache.at(0u).points) {
    CHECK(point.memory.normal == 7.0);
  }

  bodies.rigids[0].position += glm::dvec3(0.06, 0.0, 0.0);
  talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  REQUIRE(!cache.at(0u).points.empty());
  CHECK(cache.at(0u).pose.relativePosition == bodies.rigids[0].position);

  for (const talus::dynamics::CachedContactPoint& point : cache.at(0u).points) {
    CHECK(point.memory.normal == 7.0);
  }
}

TEST_CASE("islands join bodies through touching store pairs", "[dynamics]") {
  const talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  using talus::dynamics::Side;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}, {lowerRef, upperRef}});
  cache.replaceManifolds(bodies, 1u, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 1u, 0u, 0.0)
  });

  const std::vector<talus::dynamics::Island> apart =
    talus::dynamics::islandsOf(bodies, cache, {});

  REQUIRE(apart.size() == 1u);
  CHECK(apart[0].bodies.size() == 2u);
  CHECK(apart[0].pairs == std::vector<uint32_t>{1u});
  CHECK(!apart[0].external);

  cache.replaceManifolds(bodies, 0u, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 2u, 0u, 0.0)
  });

  const std::vector<talus::dynamics::Island> grounded =
    talus::dynamics::islandsOf(bodies, cache, {});

  REQUIRE(grounded.size() == 1u);
  CHECK(grounded[0].pairs == std::vector<uint32_t>{0u, 1u});
  CHECK(grounded[0].external);
}

TEST_CASE("profile scene: 4,096 loose boxes", "[.][looseprofile]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  for (int row = 0; row != 16; ++row) {
    for (int column = 0; column != 16; ++column) {
      for (int level = 0; level != 16; ++level) {
        talus::dynamics::addRigidBox(
          world.bodies,
          brick,
          glm::dvec3(0.03),
          0.5,
          glm::dvec3(
            -4.0 + 0.5 * row,
            20.0 + 0.5 * level,
            -4.0 + 0.5 * column
          ),
          glm::dvec3(0.0),
          0.5,
          3u
        );
      }
    }
  }

  double elapsed = 0.0;
  talus::dynamics::StepReport last;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    const auto before = std::chrono::steady_clock::now();

    last = talus::dynamics::step(world);
    REQUIRE(!last.nonFinite);
    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - before
    )
                 .count();
  }

  std::printf(
    "looseprofile boxes %zu islands %u ms %.3f\n",
    world.bodies.rigids.size(),
    last.softstepIslands,
    elapsed / 30.0
  );
}

TEST_CASE("softstep leaves the state table it is given all zero", "[dynamics]") {
  talus::dynamics::BodySet bodies = twoBoxesOnFloor();
  talus::dynamics::ManifoldCache cache;
  talus::dynamics::IntegrationParameters parameters;
  std::vector<uint32_t> stateOf;
  using talus::dynamics::Side;

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{lowerRef, floorRef}, {lowerRef, upperRef}});
  cache.replaceManifolds(bodies, 0u, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Static, 0u, 1u, 0u, 0.0)
  });
  cache.replaceManifolds(bodies, 1u, std::vector<talus::dynamics::Contact>{
    pointRecord(Side::Rigid, 0u, Side::Rigid, 1u, 2u, 0u, 0.0)
  });

  const std::vector<talus::dynamics::Island> islands =
    talus::dynamics::islandsOf(bodies, cache, {});

  REQUIRE(islands.size() == 1u);

  const std::vector<uint32_t> one = rigidIndices(islands[0]);

  talus::dynamics::solveSoftstep(bodies, cache, one, parameters, stateOf, talus::dynamics::Workers{}, talus::dynamics::resolveSimdWidth(0u));

  CHECK(stateOf.size() == bodies.rigids.size());
  CHECK(std::all_of(stateOf.begin(), stateOf.end(), [](uint32_t value) { return value == 0u; }));
}

namespace {

talus::dynamics::Contact tiltedRecord(
  talus::dynamics::Side sideB,
  uint32_t bodyA,
  uint32_t bodyB,
  uint64_t key,
  uint32_t manifold,
  double restitution
) {
  talus::dynamics::Contact contact = pointRecord(
    talus::dynamics::Side::Rigid,
    bodyA,
    sideB,
    bodyB,
    key,
    manifold,
    -0.002 + 0.001 * static_cast<double>(key % 3u)
  );
  const glm::dvec3 normal = glm::normalize(glm::dvec3(0.1, 1.0, 0.2));
  const glm::dvec3 tangent1 =
    glm::normalize(glm::cross(glm::dvec3(0.0, 0.0, 1.0), normal));

  contact.basis = {normal, tangent1, glm::cross(normal, tangent1)};
  contact.restitution = restitution;

  return contact;
}

struct WideRun {
  std::vector<uint64_t> bits;
  uint32_t widePairs = 0u;
};

WideRun wideRun(uint32_t width) {
  static const talus::field::SampledSdf crate = boxField(glm::dvec3(0.05), 0.01);
  static const talus::field::SampledSdf floor = floorField(1.0);
  using talus::dynamics::Side;
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);

  for (uint32_t index = 0u; index != 9u; ++index) {
    const double sign = index % 4u < 2u ? 1.0 : -1.0;
    const double rise = index % 2u == 0u ? 1.5 : -1.5;
    const uint32_t body = talus::dynamics::addRigidBox(
      bodies,
      crate,
      glm::dvec3(0.05),
      1.0,
      glm::dvec3(0.5 * static_cast<double>(index / 2u), index % 2u == 0u ? 0.15 : 0.2495, 0.0),
      glm::dvec3(0.0, sign * rise, 0.1),
      0.5,
      3u
    );

    bodies.rigids[body].rotation = glm::normalize(glm::dquat(1.0, 0.05 * index, 0.02, -0.03));
    bodies.rigids[body].angularVelocity = glm::dvec3(0.3, -0.2 * index, 0.1);
  }

  std::vector<std::vector<talus::dynamics::Contact>> records{
    {tiltedRecord(Side::Rigid, 0u, 1u, 1u, 0u, 0.0)},
    {
      tiltedRecord(Side::Rigid, 2u, 3u, 1u, 0u, 0.6),
      tiltedRecord(Side::Rigid, 2u, 3u, 2u, 0u, 0.6),
      tiltedRecord(Side::Rigid, 2u, 3u, 3u, 0u, 0.6)
    },
    {
      tiltedRecord(Side::Rigid, 4u, 5u, 1u, 0u, 0.6),
      tiltedRecord(Side::Rigid, 4u, 5u, 2u, 0u, 0.6),
      tiltedRecord(Side::Rigid, 4u, 5u, 3u, 0u, 0.6),
      tiltedRecord(Side::Rigid, 4u, 5u, 4u, 0u, 0.6)
    },
    {
      tiltedRecord(Side::Rigid, 6u, 7u, 1u, 0u, 0.0),
      tiltedRecord(Side::Rigid, 6u, 7u, 2u, 0u, 0.0),
      tiltedRecord(Side::Rigid, 6u, 7u, 3u, 1u, 0.0)
    },
    {
      tiltedRecord(Side::Static, 8u, 0u, 1u, 0u, 0.0),
      tiltedRecord(Side::Static, 8u, 0u, 2u, 0u, 0.0)
    }
  };
  std::vector<talus::dynamics::BodyPair> pairs;

  for (const std::vector<talus::dynamics::Contact>& list : records) {
    pairs.push_back({{list[0].a.side, list[0].a.body}, {list[0].b.side, list[0].b.body}});
  }

  talus::dynamics::ManifoldCache cache;
  talus::dynamics::Island island = islandOfAll(bodies, 0u);

  cache.sync(bodies, pairs);

  for (size_t pair = 0u; pair != pairs.size(); ++pair) {
    const uint32_t slot = cache.find(talus::dynamics::pairKey(bodies, pairs[pair].first, pairs[pair].second));

    cache.replaceManifolds(bodies, slot, records[pair]);
    island.pairs.push_back(slot);
  }

  const talus::dynamics::IntegrationParameters parameters;
  std::vector<uint32_t> stateOf;
  const std::vector<uint32_t> one = rigidIndices(island);
  WideRun run;

  run.widePairs = talus::dynamics::solveSoftstep(
                    bodies,
                    cache,
                    one,
                    parameters,
                    stateOf,
                    talus::dynamics::Workers{},
                    width
  )
                    .widePairs;

  const auto put = [&](double value) {
    run.bits.push_back(std::bit_cast<uint64_t>(value));
  };

  for (const talus::dynamics::RigidBody& body : bodies.rigids) {
    for (int axis = 0; axis != 3; ++axis) {
      put(body.position[axis]);
      put(body.velocity[axis]);
      put(body.angularVelocity[axis]);
    }

    for (int axis = 0; axis != 4; ++axis) {
      put(body.rotation[axis]);
    }
  }

  for (const uint32_t slot : island.pairs) {
    for (const talus::dynamics::CachedContactPoint& point : cache.at(slot).points) {
      put(point.memory.normal);
      put(point.memory.friction.x);
      put(point.memory.friction.y);
    }
  }

  return run;
}

}

TEST_CASE("wide batches solve contacts bit for bit like the scalar code", "[dynamics]") {
  const WideRun scalar = wideRun(1u);
  const WideRun four = wideRun(4u);

  REQUIRE(!scalar.bits.empty());
  CHECK(scalar.widePairs == 0u);
  CHECK(four.widePairs == 4u);
  CHECK(four.bits == scalar.bits);

  if (talus::dynamics::hasAvx2()) {
    const WideRun eight = wideRun(8u);

    CHECK(eight.widePairs == 4u);
    CHECK(eight.bits == scalar.bits);
  }
}

TEST_CASE("loose boxes state hash", "[.][loosehash]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  for (int row = 0; row != 16; ++row) {
    for (int column = 0; column != 16; ++column) {
      for (int level = 0; level != 16; ++level) {
        talus::dynamics::addRigidBox(
          world.bodies,
          brick,
          glm::dvec3(0.03),
          0.5,
          glm::dvec3(-4.0 + 0.5 * row, 20.0 + 0.5 * level, -4.0 + 0.5 * column),
          glm::dvec3(0.1 * (row % 3), 0.0, 0.0),
          0.5,
          3u
        );
      }
    }
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  std::printf(
    "loosehash hash %016llx\n",
    static_cast<unsigned long long>(stateHash(world))
  );
}

TEST_CASE("threading cost scenes", "[.][threadcost]") {
  std::vector<std::pair<uint32_t, uint32_t>> runs{{1u, 1u}, {1u, 8u}, {4u, 1u}, {4u, 8u}};

  if (talus::dynamics::hasAvx2()) {
    runs.emplace_back(8u, 1u);
    runs.emplace_back(8u, 8u);
  }

  for (const auto& [width, workers] : runs) {
    for (const int scene : {0, 1, 2}) {
      const talus::field::SampledSdf floor = floorField(1.0);
      const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
      const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
      const std::vector<talus::PhysicalMaterial> lanes{rubber()};
      talus::field::SampledSdf blockField;
      talus::dynamics::World world;

      world.workerCount = workers;
      world.simdWidth = width;
      world.parameters.sleep = false;

      if (scene == 2) {
        const glm::dvec3 half(0.225, 0.05, 0.225);
        const auto mesh = latticeBlock(half, 0.025, lanes, blockField);

        talus::dynamics::addStatic(world.bodies, floor, 0.5);
        talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.0, 0.1 + half.y, 0.0), glm::dvec3(0.0), 0.5, &blockField);

        for (int row = 0; row != 4; ++row) {
          for (int column = 0; column != 4; ++column) {
            talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.15 + 0.1 * row, 0.1 + 2.0 * half.y + 0.025, -0.15 + 0.1 * column), glm::dvec3(0.0), 0.5, 3u);
          }
        }
      } else {
        if (scene == 0) {
          talus::dynamics::addStatic(world.bodies, floor, 0.5);
        }

        const int across = scene == 1 ? 16 : 10;

        for (int row = 0; row != across; ++row) {
          for (int column = 0; column != across; ++column) {
            for (int level = 0; level != across; ++level) {
              const glm::dvec3 place = scene == 1
                ? glm::dvec3(-4.0 + 0.5 * row, 20.0 + 0.5 * level, -4.0 + 0.5 * column)
                : glm::dvec3(-0.3 + 0.0605 * row, 0.13 + 0.0605 * level, -0.3 + 0.0605 * column);

              talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, place, glm::dvec3(0.0), 0.5, 3u);
            }
          }
        }
      }

      const uint32_t settle = scene == 1 ? 0u : 30u;
      const uint32_t measured = scene == 1 ? 30u : 60u;

      for (uint32_t frame = 0u; frame != settle; ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
      }

      double elapsed = 0.0;
      uint32_t wide = 0u;
      talus::dynamics::StepProfile sum;
      const char* name = scene == 0 ? "pile" : scene == 1 ? "loose" : "soft";

      for (uint32_t frame = 0u; frame != measured; ++frame) {
        const auto before = std::chrono::steady_clock::now();
        const talus::dynamics::StepReport report = talus::dynamics::step(world);

        elapsed += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
        REQUIRE(!report.nonFinite);
        wide = report.widePairs;
        sum.travel += report.profile.travel;
        sum.broadphase += report.profile.broadphase;
        sum.storeSync += report.profile.storeSync;
        sum.storeUpdate += report.profile.storeUpdate;
        sum.lattice += report.profile.lattice;
        sum.islands += report.profile.islands;
        sum.solverSetup += report.profile.solverSetup;
        sum.prepare += report.profile.prepare;
        sum.substeps += report.profile.substeps;
        sum.storeBack += report.profile.storeBack;
        sum.continuous += report.profile.continuous;
        sum.events += report.profile.events;
        sum.sleep += report.profile.sleep;
        sum.total += report.profile.total;
      }

      std::printf("threadcost width %u workers %u scene %s ms %.3f wide %u\n", width, workers, name, elapsed / measured, wide);
      std::printf("threadphase width %u workers %u scene %s travel %.3f broadphase %.3f sync %.3f update %.3f lattice %.3f islands %.3f setup %.3f prepare %.3f substeps %.3f store %.3f continuous %.3f events %.3f sleep %.3f total %.3f\n", width, workers, name, sum.travel / measured, sum.broadphase / measured, sum.storeSync / measured, sum.storeUpdate / measured, sum.lattice / measured, sum.islands / measured, sum.solverSetup / measured, sum.prepare / measured, sum.substeps / measured, sum.storeBack / measured, sum.continuous / measured, sum.events / measured, sum.sleep / measured, sum.total / measured);
    }
  }
}

TEST_CASE("the box tree finds exactly the boxes a brute-force search finds", "[dynamics]") {
  std::mt19937 random(7u);
  std::uniform_real_distribution<double> place(-10.0, 10.0);
  std::uniform_real_distribution<double> size(0.01, 1.0);
  const auto randomBox = [&] {
    const glm::dvec3 low(place(random), place(random), place(random));

    return talus::dynamics::Bounds{low, low + glm::dvec3(size(random), size(random), size(random))};
  };
  talus::dynamics::DynamicTree tree;
  std::vector<talus::dynamics::Bounds> boxes;
  std::vector<uint32_t> proxies;
  std::vector<uint8_t> alive;

  for (uint32_t user = 0u; user != 500u; ++user) {
    boxes.push_back(randomBox());
    proxies.push_back(tree.create(boxes.back(), user));
    alive.push_back(1u);
  }

  for (uint32_t round = 0u; round != 300u; ++round) {
    const uint32_t user = static_cast<uint32_t>(random() % 500u);

    if (alive[user] != 0u && round % 3u == 0u) {
      tree.destroy(proxies[user]);
      alive[user] = 0u;
    } else if (alive[user] != 0u) {
      boxes[user] = randomBox();
      tree.move(proxies[user], boxes[user]);
    } else {
      boxes[user] = randomBox();
      proxies[user] = tree.create(boxes[user], user);
      alive[user] = 1u;
    }

    const talus::dynamics::Bounds probe = talus::dynamics::grown(randomBox(), 1.0);
    std::vector<uint32_t> found;
    std::vector<uint32_t> expected;

    tree.query(probe, [&](uint32_t, uint32_t hit) { found.push_back(hit); });

    for (uint32_t other = 0u; other != 500u; ++other) {
      if (alive[other] != 0u && talus::dynamics::overlaps(boxes[other], probe)) {
        expected.push_back(other);
      }
    }

    std::sort(found.begin(), found.end());
    CHECK(found == expected);
  }

  const auto leaves = static_cast<double>(std::count(alive.begin(), alive.end(), uint8_t{1}));

  CHECK(tree.count() == static_cast<uint32_t>(leaves));
  CHECK(tree.height() <= static_cast<uint32_t>(4.0 * std::log2(leaves)));
}

TEST_CASE("the box tree still finds exactly the brute-force boxes after leaves are resized and the tree rebuilt", "[dynamics]") {
  std::mt19937 random(9u);
  std::uniform_real_distribution<double> place(-10.0, 10.0);
  std::uniform_real_distribution<double> size(0.01, 1.0);
  const auto randomBox = [&] {
    const glm::dvec3 low(place(random), place(random), place(random));

    return talus::dynamics::Bounds{low, low + glm::dvec3(size(random), size(random), size(random))};
  };
  talus::dynamics::DynamicTree tree;
  std::vector<talus::dynamics::Bounds> boxes;
  std::vector<uint32_t> proxies;

  for (uint32_t user = 0u; user != 500u; ++user) {
    boxes.push_back(randomBox());
    proxies.push_back(tree.create(boxes.back(), user));
  }

  for (uint32_t round = 0u; round != 40u; ++round) {
    const uint32_t changes = round % 4u == 0u ? 500u : 1u + static_cast<uint32_t>(random() % 40u);

    for (uint32_t change = 0u; change != changes; ++change) {
      const uint32_t user = changes == 500u ? change : static_cast<uint32_t>(random() % 500u);

      boxes[user] = randomBox();
      tree.resize(proxies[user], boxes[user]);
    }

    tree.rebuild();

    for (uint32_t probeIndex = 0u; probeIndex != 10u; ++probeIndex) {
      const talus::dynamics::Bounds probe = talus::dynamics::grown(randomBox(), 1.0);
      std::vector<uint32_t> found;
      std::vector<uint32_t> expected;

      tree.query(probe, [&](uint32_t, uint32_t hit) { found.push_back(hit); });

      for (uint32_t other = 0u; other != 500u; ++other) {
        if (talus::dynamics::overlaps(boxes[other], probe)) {
          expected.push_back(other);
        }
      }

      std::sort(found.begin(), found.end());
      CHECK(found == expected);
    }

    CHECK(tree.height() <= static_cast<uint32_t>(4.0 * std::log2(500.0)));
  }

  CHECK(tree.count() == 500u);
}

TEST_CASE("a body that slows down sheds the pairs its fast motion made", "[dynamics]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers workers = talus::dynamics::resolveWorkers(1u, nullptr, nullptr, nullptr, scheduler);

  (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.0), glm::dvec3(0.0, 2.0, 0.0), 0.5, 3u);
  (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.12, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const talus::dynamics::BodyRef a{talus::dynamics::Side::Rigid, 0u};
  const talus::dynamics::BodyRef b{talus::dynamics::Side::Rigid, 1u};
  const uint64_t key = talus::dynamics::pairKey(world.bodies, a, b);
  const auto advance = [&](double speed) {
    const std::vector<double> travel{speed * world.parameters.dt, 0.0};

    world.broadPhase.update(world.bodies, world.parameters, travel, workers, &world.manifolds);
    world.manifolds.createPairs(world.bodies, world.broadPhase.created());

    for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
      if (world.manifolds.live(slot)
          && world.broadPhase.separated(world.bodies, world.manifolds.at(slot).first, world.manifolds.at(slot).second)) {
        world.manifolds.destroyPair(slot);
      }
    }
  };

  advance(2.0);
  REQUIRE(world.manifolds.find(key) != talus::dynamics::noSlot);

  advance(0.0);
  CHECK(world.manifolds.find(key) == talus::dynamics::noSlot);
}

TEST_CASE("box and sphere bounds equal the bounds of their samples", "[dynamics]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03, 0.05, 0.02), 0.006);
  const talus::field::SampledSdf ball = sphereField(0.04, 0.008);
  talus::dynamics::BodySet bodies;
  std::mt19937 random(11u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);

  (void)talus::dynamics::addRigidBox(bodies, brick, glm::dvec3(0.03, 0.05, 0.02), 0.5, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);
  (void)talus::dynamics::addRigidSphere(bodies, ball, 0.04, 0.5, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 0.0);

  for (int trial = 0; trial != 200; ++trial) {
    for (uint32_t index = 0u; index != 2u; ++index) {
      talus::dynamics::RigidBody& body = bodies.rigids[index];

      body.position = glm::dvec3(unit(random), unit(random), unit(random));
      body.rotation = glm::normalize(glm::dquat(unit(random), unit(random), unit(random), unit(random)));

      const glm::dmat3 turn = glm::mat3_cast(body.rotation);
      talus::dynamics::Bounds expected{glm::dvec3(1e300), glm::dvec3(-1e300)};

      for (const glm::dvec3& sample : body.samples) {
        expected.low = glm::min(expected.low, body.position + turn * sample);
        expected.high = glm::max(expected.high, body.position + turn * sample);
      }

      if (body.radius > 0.0) {
        expected.low = glm::min(expected.low, body.position - glm::dvec3(body.radius));
        expected.high = glm::max(expected.high, body.position + glm::dvec3(body.radius));
      }

      const talus::dynamics::Bounds found =
        talus::dynamics::boundsOf(bodies, talus::dynamics::BodyRef{talus::dynamics::Side::Rigid, index}, 0.0);

      CHECK(glm::all(glm::lessThanEqual(glm::abs(found.low - expected.low), glm::dvec3(1e-12))));
      CHECK(glm::all(glm::lessThanEqual(glm::abs(found.high - expected.high), glm::dvec3(1e-12))));
    }
  }
}

TEST_CASE("the contact store holds every pair the sweep finds, through motion and edits", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const talus::field::SampledSdf ball = sphereField(0.04, 0.008);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.1, 0.03, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  std::mt19937 random(5u);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::vector<uint32_t> ids;

  world.workerCount = 4u;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.4, 0.05, 0.4), glm::dvec3(0.0), 0.5, &blockField);

  for (int index = 0; index != 64; ++index) {
    const glm::dvec3 place(-0.12 + 0.07 * (index % 4), 0.05 + 0.07 * (index / 16), -0.12 + 0.07 * ((index / 4) % 4));

    (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, place, glm::dvec3(unit(random), unit(random), unit(random)), 0.5, 3u);
    ids.push_back(world.bodies.rigids.back().id);
  }

  for (int index = 0; index != 16; ++index) {
    const double speed = index % 4 == 0 ? 8.0 : 1.0;

    (void)talus::dynamics::addRigidSphere(world.bodies, ball, 0.04, 0.5, glm::dvec3(0.3 + 0.1 * index, 0.5, 0.0), glm::dvec3(unit(random), -speed, unit(random)), 0.5, 0.0);
    ids.push_back(world.bodies.rigids.back().id);
  }

  const double dt = world.parameters.dt;
  const double band = world.parameters.speculativeDistance + world.parameters.margin;
  const talus::dynamics::Workers workers = talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, world.scheduler);

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    if (frame == 20u) {
      REQUIRE(talus::dynamics::setRigidPose(world, ids[0], glm::dvec3(5.0, 5.0, 5.0), glm::dquat(1.0, 0.0, 0.0, 0.0)));
    } else if (frame == 30u) {
      REQUIRE(talus::dynamics::setRigidPose(world, ids[0], glm::dvec3(0.0, 0.1, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0)));
    } else if (frame == 40u) {
      REQUIRE(talus::dynamics::removeBody(world, ids[1]));
    } else if (frame == 50u) {
      REQUIRE(talus::dynamics::setMass(world, ids[2], 2.0));
    } else if (frame == 60u) {
      REQUIRE(talus::dynamics::replaceRigidShape(world, ids[3], world.bodies.rigids.back()));
    } else if (frame == 70u) {
      (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.0, 0.4, 0.0), glm::dvec3(0.0, -3.0, 0.0), 0.5, 3u);
    } else if (frame == 80u) {
      REQUIRE(talus::dynamics::setStaticPose(world, world.bodies.statics[0].id, glm::dvec3(0.0, -0.01, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0)));
    }

    for (talus::dynamics::RigidBody& body : world.bodies.rigids) {
      body.position += dt * body.velocity;
      body.rotation = glm::normalize(glm::dquat(1.0, 0.3 * dt * unit(random), 0.3 * dt * unit(random), 0.3 * dt * unit(random)) * body.rotation);
    }

    for (glm::dvec3& node : world.bodies.lattices[0].position) {
      node += glm::dvec3(0.0, 0.001 * unit(random), 0.0);
    }

    std::vector<double> travel(world.bodies.lattices.size() + world.bodies.rigids.size(), 0.0);

    for (size_t index = 0u; index != world.bodies.rigids.size(); ++index) {
      travel[world.bodies.lattices.size() + index] = glm::length(world.bodies.rigids[index].velocity) * dt;
    }

    world.broadPhase.update(world.bodies, world.parameters, travel, workers, &world.manifolds);
    world.manifolds.createPairs(world.bodies, world.broadPhase.created());

    for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
      if (world.manifolds.live(slot)
          && world.broadPhase.separated(world.bodies, world.manifolds.at(slot).first, world.manifolds.at(slot).second)) {
        world.manifolds.destroyPair(slot);
      }
    }

    const std::vector<talus::dynamics::BodyPair> sweep =
      talus::dynamics::broadphase(world.bodies, band, travel, workers);
    std::vector<uint64_t> latticeSweep;
    std::vector<uint64_t> latticeFound;

    for (const talus::dynamics::BodyPair& pair : sweep) {
      const bool lattice = pair.first.side == talus::dynamics::Side::Lattice
                           || pair.second.side == talus::dynamics::Side::Lattice;

      if (lattice) {
        latticeSweep.push_back(talus::dynamics::pairKey(world.bodies, pair.first, pair.second));
      } else {
        CHECK(world.manifolds.find(talus::dynamics::pairKey(world.bodies, pair.first, pair.second)) != talus::dynamics::noSlot);
      }
    }

    for (const talus::dynamics::BodyPair& pair : world.broadPhase.latticePairs()) {
      latticeFound.push_back(talus::dynamics::pairKey(world.bodies, pair.first, pair.second));
    }

    std::sort(latticeSweep.begin(), latticeSweep.end());
    std::sort(latticeFound.begin(), latticeFound.end());
    CHECK(latticeFound == latticeSweep);

    for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
      if (world.manifolds.live(slot)) {
        CHECK(world.broadPhase.overlaps(world.bodies, world.manifolds.at(slot).first, world.manifolds.at(slot).second));
      }
    }
  }
}

namespace {

void checkColourArrays(const talus::dynamics::World& world) {
  const talus::dynamics::ManifoldCache& cache = world.manifolds;
  const talus::dynamics::BodySet& bodies = world.bodies;
  std::vector<uint8_t> listed(cache.size(), 0u);

  for (uint32_t colour = 0u; colour != talus::dynamics::colourCount; ++colour) {
    std::vector<uint32_t> users;

    for (const bool wide : {true, false}) {
      const std::span<const uint32_t> slots = wide ? cache.widePairs(colour) : cache.otherPairs(colour);

      for (uint32_t index = 0u; index != slots.size(); ++index) {
        const talus::dynamics::CachedBodyPair& pair = cache.at(slots[index]);

        REQUIRE(cache.live(slots[index]));
        CHECK(listed[slots[index]] == 0u);
        listed[slots[index]] = 1u;
        CHECK(pair.colour == colour);
        CHECK(pair.colourIndex == index);
        CHECK(pair.wide == wide);
        CHECK(!pair.points.empty());

        for (const talus::dynamics::BodyRef& ref : {pair.first, pair.second}) {
          if (ref.side == talus::dynamics::Side::Rigid && bodies.rigids[ref.index].mass > 0.0) {
            users.push_back(bodies.rigids[ref.index].id);
          }
        }
      }
    }

    if (colour != talus::dynamics::overflowColour) {
      std::sort(users.begin(), users.end());
      CHECK(std::adjacent_find(users.begin(), users.end()) == users.end());
    }
  }

  for (uint32_t slot = 0u; slot != cache.size(); ++slot) {
    if (!cache.live(slot)) {
      continue;
    }

    const talus::dynamics::CachedBodyPair& pair = cache.at(slot);
    const bool awake = !talus::dynamics::resting(bodies, pair.first)
                       || !talus::dynamics::resting(bodies, pair.second);

    CHECK((listed[slot] != 0u) == (!pair.points.empty() && awake));
  }
}

}

TEST_CASE("every touching awake pair sits in exactly one colour array, through sleep, wake and edits", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;
  std::vector<uint32_t> ids;

  world.workerCount = 4u;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int index = 0; index != 27; ++index) {
    (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.06 + 0.0605 * (index % 3), 0.04 + 0.0605 * (index / 9), -0.06 + 0.0605 * ((index / 3) % 3)), glm::dvec3(0.0), 0.5, 3u);
    ids.push_back(world.bodies.rigids.back().id);
  }

  bool slept = false;

  for (uint32_t frame = 0u; frame != 400u; ++frame) {
    if (frame == 250u) {
      (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.0, 0.4, 0.0), glm::dvec3(0.0, -2.0, 0.0), 0.5, 3u);
    } else if (frame == 300u) {
      REQUIRE(talus::dynamics::setMass(world, ids[13], 3.0));
    } else if (frame == 320u) {
      REQUIRE(talus::dynamics::removeBody(world, ids[4]));
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    slept = slept || world.bodies.rigids[0].asleep;
    checkColourArrays(world);
  }

  CHECK(slept);
}

TEST_CASE("a sleeping pile woken by a falling box is held by its contacts in the step it wakes", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int index = 0; index != 27; ++index) {
    (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.06 + 0.0605 * (index % 3), 0.04 + 0.0605 * (index / 9), -0.06 + 0.0605 * ((index / 3) % 3)), glm::dvec3(0.0), 0.5, 3u);
  }

  for (uint32_t frame = 0u; frame != 400u && !world.bodies.rigids[0].asleep; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

  }

  REQUIRE(world.bodies.rigids[0].asleep);
  (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.0, 0.215, 0.0), glm::dvec3(0.0, -1.0, 0.0), 0.5, 3u);

  for (uint32_t frame = 0u; frame != 30u && world.bodies.rigids[0].asleep; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(!world.bodies.rigids[0].asleep);

  for (uint32_t index = 0u; index != 27u; ++index) {
    CHECK(world.bodies.rigids[index].velocity.y > -0.05);
  }
}

TEST_CASE("a world that switches widths and workers between steps ends like one that never switched", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const auto pile = [&](talus::dynamics::World& world) {
    world.parameters.sleep = false;
    talus::dynamics::addStatic(world.bodies, floor, 0.5);

    for (int index = 0; index != 125; ++index) {
      (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.12 + 0.0605 * (index % 5), 0.04 + 0.0605 * (index / 25), -0.12 + 0.0605 * ((index / 5) % 5)), glm::dvec3(0.0), 0.5, 3u);
    }
  };
  talus::dynamics::World steady;
  talus::dynamics::World switching;
  const std::array<std::pair<uint32_t, uint32_t>, 4> plan{{{4u, 8u}, {1u, 2u}, {8u, 1u}, {4u, 4u}}};

  pile(steady);
  pile(switching);
  steady.simdWidth = 1u;
  steady.workerCount = 1u;

  for (uint32_t frame = 0u; frame != 80u; ++frame) {
    const auto [width, workers] = plan[(frame / 20u) % plan.size()];

    switching.simdWidth = width == 8u && !talus::dynamics::hasAvx2() ? 4u : width;
    switching.workerCount = workers;
    REQUIRE(!talus::dynamics::step(steady).nonFinite);
    REQUIRE(!talus::dynamics::step(switching).nonFinite);
  }

  CHECK(stateHash(switching) == stateHash(steady));
}

TEST_CASE("crates on a soft block are left to the lattice solver", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  const glm::dvec3 half(0.225, 0.05, 0.225);
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(half, 0.025, lanes, blockField);
  talus::dynamics::World world;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.0, 0.1 + half.y, 0.0), glm::dvec3(0.0), 0.5, &blockField);

  for (int index = 0; index != 4; ++index) {
    talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.05 + 0.1 * (index % 2), 0.1 + 2.0 * half.y + 0.025 + 0.055 * (index / 2), 0.0), glm::dvec3(0.0), 0.5, 3u);
  }

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);

    if (frame >= 10u) {
      CHECK(report.softstepIslands == 0u);
      CHECK(report.widePairs == 0u);
    }
  }
}

namespace {

uint32_t contactId(const talus::dynamics::BodySet& bodies, const talus::dynamics::ContactSide& side) {
  return side.side == talus::dynamics::Side::Rigid ? bodies.rigids[side.body].id : bodies.lattices[side.body].id;
}

std::vector<uint32_t> freshComponents(const talus::dynamics::World& world) {
  const talus::dynamics::BodySet& bodies = world.bodies;
  std::vector<uint32_t> parent(bodies.nextId);

  std::iota(parent.begin(), parent.end(), 0u);

  const auto root = [&](uint32_t node) {
    while (parent[node] != node) {
      parent[node] = parent[parent[node]];
      node = parent[node];
    }

    return node;
  };

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot) || world.manifolds.at(slot).points.empty()) {
      continue;
    }

    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    if (pair.second.side == talus::dynamics::Side::Rigid) {
      parent[root(pair.firstId)] = root(pair.secondId);
    }
  }

  for (const talus::dynamics::Contact& contact : world.contacts) {
    if (contact.a.side != talus::dynamics::Side::Static && contact.b.side != talus::dynamics::Side::Static) {
      parent[root(contactId(bodies, contact.a))] = root(contactId(bodies, contact.b));
    }
  }

  for (uint32_t id = 0u; id != parent.size(); ++id) {
    parent[id] = root(id);
  }

  return parent;
}

void checkIslands(const talus::dynamics::World& world) {
  const talus::dynamics::IslandStore& store = world.islandStore;
  const talus::dynamics::BodySet& bodies = world.bodies;
  const std::vector<uint32_t> fresh = freshComponents(world);

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot) || world.manifolds.at(slot).points.empty()) {
      continue;
    }

    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    if (pair.second.side == talus::dynamics::Side::Rigid) {
      CHECK(store.islandOf(pair.firstId) == store.islandOf(pair.secondId));
    }
  }

  for (const talus::dynamics::Contact& contact : world.contacts) {
    if (contact.a.side != talus::dynamics::Side::Static && contact.b.side != talus::dynamics::Side::Static) {
      CHECK(store.islandOf(contactId(bodies, contact.a)) == store.islandOf(contactId(bodies, contact.b)));
    }
  }

  std::vector<uint8_t> asleep(bodies.nextId, 1u);
  std::vector<uint8_t> present(bodies.nextId, 0u);

  for (const talus::dynamics::RigidBody& body : bodies.rigids) {
    asleep[body.id] = body.asleep ? 1u : 0u;
    present[body.id] = 1u;
  }

  for (const talus::dynamics::LatticeBody& body : bodies.lattices) {
    asleep[body.id] = body.asleep ? 1u : 0u;
    present[body.id] = 1u;
  }

  for (uint32_t id = 0u; id != bodies.nextId; ++id) {
    if (present[id] == 0u) {
      continue;
    }

    const std::span<const uint32_t> members = store.members(store.islandOf(id));
    const bool sleeping = std::all_of(members.begin(), members.end(), [&](uint32_t member) {
      return asleep[member] != 0u;
    });

    if (!sleeping) {
      continue;
    }

    for (uint32_t other = 0u; other != bodies.nextId; ++other) {
      if (present[other] != 0u) {
        const bool inIsland = std::find(members.begin(), members.end(), other) != members.end();

        CHECK(inIsland == (fresh[other] == fresh[id]));
      }
    }
  }
}

}

TEST_CASE("stored islands never split a touching pair, and sleeping islands are exact", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.1, 0.03, 0.1), 0.025, lanes, blockField);
  talus::dynamics::World world;
  std::vector<uint32_t> ids;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.6, 0.05, 0.0), glm::dvec3(0.0), 0.5, &blockField);

  for (int index = 0; index != 18; ++index) {
    const double x = index < 9 ? -0.2 : 0.2;

    (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(x + 0.0605 * (index % 3), 0.04 + 0.0605 * ((index % 9) / 3), 0.0), glm::dvec3(0.0), 0.5, 3u);
    ids.push_back(world.bodies.rigids.back().id);
  }

  (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.6, 0.2, 0.0), glm::dvec3(0.0), 0.5, 3u);

  bool slept = false;

  for (uint32_t frame = 0u; frame != 500u; ++frame) {
    if (frame == 60u) {
      REQUIRE(talus::dynamics::setVelocity(world, ids[2], glm::dvec3(4.0, 0.0, 0.0)));
    } else if (frame == 200u) {
      REQUIRE(talus::dynamics::removeBody(world, ids[11]));
    } else if (frame == 300u) {
      (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.14, 0.4, 0.0), glm::dvec3(0.0, -2.0, 0.0), 0.5, 3u);
    } else if (frame == 400u) {
      REQUIRE(talus::dynamics::setRigidPose(world, ids[0], glm::dvec3(0.2, 0.3, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0)));
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    slept = slept || world.bodies.rigids[0].asleep;
    checkIslands(world);
  }

  CHECK(slept);
}

TEST_CASE("a split asked for a body that is then removed is dropped, and one asked for a member splits", "[dynamics]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::BodySet bodies;
  const talus::dynamics::ManifoldCache cache;
  talus::dynamics::IslandStore store;
  std::vector<uint32_t> ids;

  for (uint32_t index = 0u; index != 4u; ++index) {
    (void)talus::dynamics::addRigidBox(bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.1 * index, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);
    ids.push_back(bodies.rigids.back().id);
  }

  store.sync(bodies);
  store.link(ids[0], ids[1]);
  store.link(ids[1], ids[2]);
  store.link(ids[2], ids[3]);
  store.markRemoved(ids[0]);
  REQUIRE(store.removed(store.islandOf(ids[0])) > 0u);

  store.requestSplit(ids[1]);
  CHECK(store.pendingSplit() == ids[1]);
  bodies.rigids.erase(bodies.rigids.begin() + 1);
  store.sync(bodies);
  CHECK(store.takeSplit() == talus::dynamics::noIsland);
  CHECK(store.removed(store.islandOf(ids[0])) > 0u);
  CHECK(store.members(store.islandOf(ids[0])).size() == 3u);

  store.requestSplit(ids[2]);

  const uint32_t island = store.takeSplit();

  REQUIRE(island == store.islandOf(ids[2]));
  (void)store.split(bodies, island, cache, {}, talus::dynamics::JointSet{});
  CHECK(store.islandOf(ids[0]) != store.islandOf(ids[2]));
  CHECK(store.islandOf(ids[2]) != store.islandOf(ids[3]));
  CHECK(store.pendingSplit() == talus::dynamics::noBody);
  CHECK(store.takeSplit() == talus::dynamics::noIsland);
}

TEST_CASE("a split pending over a removal, wake or move of its body keeps islands exact, alike on 1 and 8 workers", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.1, 0.03, 0.1), 0.025, lanes, blockField);
  std::vector<uint64_t> hashes;

  for (const uint32_t workers : {1u, 8u}) {
    talus::dynamics::World world;
    std::vector<uint32_t> ids;
    uint32_t pendingSteps = 0u;
    uint32_t removed = 0u;
    uint32_t woken = 0u;
    uint32_t moved = 0u;

    world.workerCount = workers;
    talus::dynamics::addStatic(world.bodies, floor, 0.5);
    talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.6, 0.05, 0.0), glm::dvec3(0.0), 0.5, &blockField);

    for (int index = 0; index != 18; ++index) {
      const double x = index < 9 ? -0.2 : 0.2;

      (void)talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(x + 0.0605 * (index % 3), 0.04 + 0.0605 * ((index % 9) / 3), 0.0), glm::dvec3(0.0), 0.5, 3u);
      ids.push_back(world.bodies.rigids.back().id);
    }

    for (uint32_t frame = 0u; frame != 600u; ++frame) {
      if (frame == 60u) {
        REQUIRE(talus::dynamics::setVelocity(world, ids[2], glm::dvec3(4.0, 0.0, 0.0)));
      } else if (frame == 250u) {
        REQUIRE(talus::dynamics::setVelocity(world, ids[11], glm::dvec3(-3.0, 0.0, 0.0)));
      }

      REQUIRE(!talus::dynamics::step(world).nonFinite);
      checkIslands(world);

      const uint32_t pending = world.islandStore.pendingSplit();

      if (pending == talus::dynamics::noBody) {
        continue;
      }

      pendingSteps += 1u;

      if (removed < 2u && pendingSteps % 3u == 1u) {
        REQUIRE(talus::dynamics::removeBody(world, pending));
        removed += 1u;
      } else if (woken < 2u && pendingSteps % 3u == 2u) {
        REQUIRE(talus::dynamics::setVelocity(world, pending, glm::dvec3(0.0, 0.3, 0.0)));
        woken += 1u;
      } else if (moved < 2u && pendingSteps % 3u == 0u) {
        REQUIRE(talus::dynamics::setRigidPose(world, pending, glm::dvec3(0.0, 0.3, 0.3), glm::dquat(1.0, 0.0, 0.0, 0.0)));
        moved += 1u;
      }
    }

    CHECK(removed == 2u);
    CHECK(woken == 2u);
    CHECK(moved == 2u);
    hashes.push_back(stateHash(world));
  }

  CHECK(hashes[0] == hashes[1]);
}

TEST_CASE("splitting an island costs time in proportion to its members and contacts", "[dynamics]") {
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  const uint32_t count = 8000u;
  talus::dynamics::BodySet bodies;
  talus::dynamics::ManifoldCache cache;
  talus::dynamics::IslandStore store;
  std::vector<talus::dynamics::Contact> contacts;

  for (uint32_t index = 0u; index != count; ++index) {
    (void)talus::dynamics::addRigidBox(bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(0.1 * index, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);
  }

  store.sync(bodies);

  for (uint32_t index = 1u; index != count; ++index) {
    store.link(bodies.rigids[0].id, bodies.rigids[index].id);
  }

  for (uint32_t index = 0u; index != 4u * count; ++index) {
    talus::dynamics::Contact contact;

    contact.a.side = talus::dynamics::Side::Rigid;
    contact.a.body = index % count;
    contact.b.side = talus::dynamics::Side::Rigid;
    contact.b.body = (index + 1u) % count;
    contacts.push_back(contact);
  }

  const auto start = std::chrono::steady_clock::now();
  const std::vector<uint32_t> pieces = store.split(
    bodies,
    store.islandOf(bodies.rigids[0].id),
    cache,
    contacts,
    talus::dynamics::JointSet{}
  );
  const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  CHECK(pieces.size() == 1u);
  CHECK(store.members(pieces[0]).size() == count);
  CHECK(elapsed < 20.0);
}

TEST_CASE("a tier switch keeps the store, the colour arrays and the islands consistent", "[dynamics]") {
  TierScene scene(softRubber());
  (void)scene.addCrate(glm::dvec3(0.0, 0.175, 0.0), 2.0);
  bool promoted = false;

  for (int frame = 0; frame != 90; ++frame) {
    REQUIRE(!talus::dynamics::step(scene.world).nonFinite);
    promoted = promoted || scene.promoted();
    checkColourArrays(scene.world);
    checkIslands(scene.world);
  }

  CHECK(promoted);
}

TEST_CASE("crates lifted off a soft block go back to the rigid solver the next step", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  const glm::dvec3 half(0.225, 0.05, 0.225);
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(half, 0.025, lanes, blockField);
  talus::dynamics::World world;
  std::vector<uint32_t> crates;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.0, 0.1 + half.y, 0.0), glm::dvec3(0.0), 0.5, &blockField);

  for (int index = 0; index != 4; ++index) {
    (void)talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.05 + 0.1 * (index % 2), 0.1 + 2.0 * half.y + 0.025 + 0.055 * (index / 2), 0.0), glm::dvec3(0.0), 0.5, 3u);
    crates.push_back(world.bodies.rigids.back().id);
  }

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(talus::dynamics::step(world).softstepIslands == 0u);

  for (uint32_t index = 0u; index != crates.size(); ++index) {
    REQUIRE(talus::dynamics::setRigidPose(world, crates[index], glm::dvec3(2.0 + 0.2 * index, 1.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0)));
  }

  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  CHECK(report.softstepIslands == 4u);
  CHECK(report.strainstepIslands == 1u);
}

TEST_CASE("the step profile times every stage of a pile step", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int row = 0; row != 3; ++row) {
    for (int column = 0; column != 3; ++column) {
      for (int level = 0; level != 3; ++level) {
        talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.06 + 0.0605 * row, 0.04 + 0.0605 * level, -0.06 + 0.0605 * column), glm::dvec3(0.0), 0.5, 3u);
      }
    }
  }

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::StepProfile profile = talus::dynamics::step(world).profile;
  const std::array<double, 14> stages{
    profile.travel, profile.broadphase, profile.storeSync, profile.storeUpdate,
    profile.lattice, profile.islands, profile.solverSetup, profile.gather,
    profile.prepare, profile.substeps, profile.storeBack, profile.continuous,
    profile.events, profile.sleep
  };
  double sum = 0.0;

  for (const double stage : stages) {
    CHECK(stage >= -1e-9);
    sum += stage;
  }

  CHECK(profile.total > 0.0);
  CHECK(profile.prepare > 0.0);
  CHECK(profile.substeps > 0.0);
  CHECK(profile.integrate > 0.0);
  CHECK(profile.integrate + profile.restitute <= profile.substeps * 1.000001 + 1e-9);
  CHECK(sum <= profile.total * 1.000001 + 1e-9);
  CHECK(sum >= profile.total * 0.999999 - 1e-9);
}

namespace {

void* inlineEnqueue(
  talus::dynamics::TaskCallback* task,
  void* taskContext,
  void* userContext,
  const char*
) {
  ++*static_cast<int*>(userContext);
  task(taskContext);
  return nullptr;
}

void inlineFinish(void*, void*) {}

struct DeferredTask {
  talus::dynamics::TaskCallback* task = nullptr;
  void* context = nullptr;
  bool done = false;
};

void* deferredEnqueue(
  talus::dynamics::TaskCallback* task,
  void* taskContext,
  void* userContext,
  const char*
) {
  auto& queue = *static_cast<std::vector<std::unique_ptr<DeferredTask>>*>(userContext);

  queue.push_back(std::make_unique<DeferredTask>(DeferredTask{task, taskContext, false}));

  return queue.back().get();
}

void deferredFinish(void* userTask, void*) {
  DeferredTask& task = *static_cast<DeferredTask*>(userTask);

  if (!task.done) {
    task.done = true;
    task.task(task.context);
  }
}

}

TEST_CASE("a world with one worker and no callbacks runs serially", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers workers =
    talus::dynamics::resolveWorkers(1u, nullptr, nullptr, nullptr, scheduler);

  CHECK(workers.count == 1u);
  CHECK(workers.scheduler == nullptr);
  CHECK(workers.hostEnqueue == nullptr);
  CHECK(scheduler == nullptr);
}

TEST_CASE("the built-in scheduler follows the worker count", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers four =
    talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, scheduler);
  talus::dynamics::Scheduler* first = scheduler.get();
  const talus::dynamics::Workers again =
    talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, scheduler);

  REQUIRE(first != nullptr);
  CHECK(four.count == 4u);
  CHECK(four.scheduler == first);
  CHECK(again.scheduler == first);
  CHECK(first->workerCount() == 4u);

  const talus::dynamics::Workers two =
    talus::dynamics::resolveWorkers(2u, nullptr, nullptr, nullptr, scheduler);

  CHECK(two.count == 2u);
  CHECK(scheduler->workerCount() == 2u);

  const talus::dynamics::Workers many =
    talus::dynamics::resolveWorkers(100u, nullptr, nullptr, nullptr, scheduler);

  CHECK(many.count == 32u);
}

TEST_CASE("a world with host callbacks uses them", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  int calls = 0;
  const talus::dynamics::Workers workers =
    talus::dynamics::resolveWorkers(8u, &inlineEnqueue, &inlineFinish, &calls, scheduler);

  CHECK(workers.count == 8u);
  CHECK(workers.scheduler == nullptr);
  CHECK(workers.hostEnqueue == &inlineEnqueue);
  CHECK(scheduler == nullptr);
}

TEST_CASE("a parallel-for visits every item once", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  int calls = 0;
  std::vector<std::unique_ptr<DeferredTask>> queue;
  const std::array<talus::dynamics::Workers, 4> modes{
    talus::dynamics::resolveWorkers(1u, nullptr, nullptr, nullptr, scheduler),
    talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, scheduler),
    talus::dynamics::resolveWorkers(8u, &inlineEnqueue, &inlineFinish, &calls, scheduler),
    talus::dynamics::resolveWorkers(8u, &deferredEnqueue, &deferredFinish, &queue, scheduler)
  };

  for (const talus::dynamics::Workers& workers : modes) {
    for (const uint32_t count : {0u, 1u, 7u, 1000u}) {
      std::vector<std::atomic<int>> hits(count);
      std::atomic<int> outOfRange{0};

      talus::dynamics::parallelFor(workers, count, 7u, [&](uint32_t begin, uint32_t end, uint32_t worker) {
        if (worker >= workers.count || end > count || begin >= end) {
          outOfRange.fetch_add(1);
          return;
        }

        for (uint32_t item = begin; item != end; ++item) {
          hits[item].fetch_add(1);
        }
      });

      CHECK(outOfRange.load() == 0);

      for (const std::atomic<int>& hit : hits) {
        CHECK(hit.load() == 1);
      }
    }
  }
}

namespace {

bool runStagesOnce(const talus::dynamics::Workers& workers) {
  const std::vector<uint32_t> blocks{5u, 0u, 9u, 1u, 64u};
  std::vector<std::vector<std::atomic<int>>> hits;
  std::vector<std::atomic<int>> finished(blocks.size());
  std::atomic<bool> outOfOrder{false};
  std::vector<int> serial;

  for (const uint32_t count : blocks) {
    hits.emplace_back(count);
  }

  talus::dynamics::StageRunner runner(workers, blocks, [&](uint32_t stage, uint32_t block, uint32_t worker) {
    if (worker >= workers.count) {
      outOfOrder.store(true);
    }

    for (uint32_t earlier = 0u; earlier != stage; ++earlier) {
      if (finished[earlier].load() != static_cast<int>(blocks[earlier])) {
        outOfOrder.store(true);
      }
    }

    hits[stage][block].fetch_add(1);
    finished[stage].fetch_add(1);
  });

  runner.run([&] {
    for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
      serial.push_back(static_cast<int>(stage));
      runner.execute(stage);
    }
  });

  bool once = serial.size() == blocks.size();

  for (const std::vector<std::atomic<int>>& stage : hits) {
    for (const std::atomic<int>& hit : stage) {
      once = once && hit.load() == 1;
    }
  }

  return once && !outOfOrder.load();
}

}

TEST_CASE("a stage runner runs every block of every stage once and in stage order", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;

  CHECK(runStagesOnce(talus::dynamics::resolveWorkers(1u, nullptr, nullptr, nullptr, scheduler)));
  CHECK(runStagesOnce(talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler)));
}

TEST_CASE("a stage runner finishes when the host runs tasks inline", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  int calls = 0;

  CHECK(runStagesOnce(talus::dynamics::resolveWorkers(8u, &inlineEnqueue, &inlineFinish, &calls, scheduler)));
  CHECK(calls == 8);
}

TEST_CASE("a stage runner finishes when the host runs tasks only on finish", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  std::vector<std::unique_ptr<DeferredTask>> queue;

  CHECK(runStagesOnce(talus::dynamics::resolveWorkers(8u, &deferredEnqueue, &deferredFinish, &queue, scheduler)));
  CHECK(queue.size() == 8u);
}

TEST_CASE("scheduler finish and both stage waits wake after work lasts longer than the spin limit", "[dynamics][schedulerwaits]") {
  struct Delayed {
    std::atomic<bool> entered{false};
    std::atomic<bool> released{false};
    std::atomic<bool> done{false};
  } delayed;
  talus::dynamics::Scheduler scheduler(2u);
  void* task = scheduler.enqueue([](void* context) {
    auto& state = *static_cast<Delayed*>(context);
    state.entered.store(true);
    state.entered.notify_all();
    state.released.wait(false);
    state.done.store(true);
  }, &delayed);

  delayed.entered.wait(false);
  std::jthread release([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    delayed.released.store(true);
    delayed.released.notify_all();
  });
  scheduler.finish(task);
  CHECK(delayed.done.load());

  for (const auto claiming : {talus::dynamics::Claiming::Shared, talus::dynamics::Claiming::Ranges}) {
    std::shared_ptr<talus::dynamics::Scheduler> stages;
    const auto workers = talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, stages);
    std::array<std::atomic<bool>, 2> peerStarted{};
    std::atomic<uint32_t> completed{0u};
    talus::dynamics::StageRunner runner;
    const std::array<uint32_t, 2> blocks{8u, 8u};

    runner.reset(workers, blocks, [&](uint32_t stage, uint32_t, uint32_t worker) {
      if (worker == 0u) {
        peerStarted[stage].wait(false);
      } else {
        peerStarted[stage].store(true);
        peerStarted[stage].notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
      }

      completed.fetch_add(1u);
    }, claiming);
    runner.run([&] {
      for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        runner.execute(stage);
      }
    });
    CHECK(completed.load() == 16u);
    CHECK(peerStarted[0].load());
    CHECK(peerStarted[1].load());
  }
}

namespace {

bool runsAlongsideOnce(const talus::dynamics::Workers& workers, talus::dynamics::Claiming claiming) {
  const std::vector<uint32_t> blocks{7u, 0u, 33u, 2u};
  std::vector<std::atomic<int>> finished(blocks.size());
  std::atomic<int> alongside{0};
  std::atomic<int> visits{0};
  talus::dynamics::StageRunner runner;

  runner.reset(workers, blocks, [&](uint32_t stage, uint32_t, uint32_t) {
    finished[stage].fetch_add(1);
    visits.fetch_add(1);
  }, claiming);

  const std::function<void()> job = [&] {
    alongside.fetch_add(1);
  };

  runner.run([&] {
    for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
      runner.execute(stage);
    }
  }, &job);

  bool complete = alongside.load() == 1;

  for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
    complete = complete && finished[stage].load() == static_cast<int>(blocks[stage]);
  }

  return complete && visits.load() == 42;
}

}

TEST_CASE("a stage runner runs its alongside job exactly once beside its stages however the host runs tasks", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  int calls = 0;
  std::vector<std::unique_ptr<DeferredTask>> queue;

  for (const talus::dynamics::Claiming claiming : {talus::dynamics::Claiming::Shared, talus::dynamics::Claiming::Ranges}) {
    CHECK(runsAlongsideOnce(talus::dynamics::resolveWorkers(1u, nullptr, nullptr, nullptr, scheduler), claiming));
    CHECK(runsAlongsideOnce(talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler), claiming));
    CHECK(runsAlongsideOnce(talus::dynamics::resolveWorkers(8u, &inlineEnqueue, &inlineFinish, &calls, scheduler), claiming));
    CHECK(runsAlongsideOnce(talus::dynamics::resolveWorkers(8u, &deferredEnqueue, &deferredFinish, &queue, scheduler), claiming));
  }
}

namespace {

bool alongsideRunsSerially(const talus::dynamics::Workers& workers, talus::dynamics::Claiming claiming, const int* enqueued) {
  const std::vector<uint32_t> blocks{64u, 128u, 32u};
  const std::vector<uint32_t> innerBlocks{40u, 8u};
  std::vector<std::atomic<int>> finished(blocks.size());
  std::vector<int> looped(5000u, 0);
  std::vector<int> innerFinished(innerBlocks.size(), 0);
  std::atomic<int> foreign{0};
  int nestedEnqueues = 0;
  talus::dynamics::StageRunner runner;
  talus::dynamics::StageRunner inner;

  runner.reset(workers, blocks, [&](uint32_t stage, uint32_t, uint32_t) {
    finished[stage].fetch_add(1);
  }, claiming);

  const std::function<void()> job = [&] {
    const std::thread::id own = std::this_thread::get_id();
    const int before = enqueued != nullptr ? *enqueued : 0;

    talus::dynamics::parallelFor(workers, static_cast<uint32_t>(looped.size()), 1u, [&](uint32_t begin, uint32_t end, uint32_t) {
      if (std::this_thread::get_id() != own) {
        foreign.fetch_add(1);
      }

      for (uint32_t index = begin; index != end; ++index) {
        looped[index] += 1;
      }
    });

    inner.reset(workers, innerBlocks, [&](uint32_t stage, uint32_t, uint32_t) {
      if (std::this_thread::get_id() != own) {
        foreign.fetch_add(1);
      }

      innerFinished[stage] += 1;
    }, claiming);
    inner.run([&] {
      for (uint32_t stage = 0u; stage != innerBlocks.size(); ++stage) {
        inner.execute(stage);
      }
    });

    nestedEnqueues = enqueued != nullptr ? *enqueued - before : 0;
  };

  runner.run([&] {
    for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
      runner.execute(stage);
    }
  }, &job);

  bool serial = foreign.load() == 0 && nestedEnqueues == 0
                && std::all_of(looped.begin(), looped.end(), [](int count) { return count == 1; });

  for (uint32_t stage = 0u; stage != blocks.size(); ++stage) {
    serial = serial && finished[stage].load() == static_cast<int>(blocks[stage]);
  }

  for (uint32_t stage = 0u; stage != innerBlocks.size(); ++stage) {
    serial = serial && innerFinished[stage] == static_cast<int>(innerBlocks[stage]);
  }

  std::vector<std::atomic<int>> after(1000u);

  talus::dynamics::parallelFor(workers, static_cast<uint32_t>(after.size()), 1u, [&](uint32_t begin, uint32_t end, uint32_t) {
    for (uint32_t index = begin; index != end; ++index) {
      after[index].fetch_add(1);
    }
  });

  return serial && std::all_of(after.begin(), after.end(), [](const std::atomic<int>& count) { return count.load() == 1; });
}

}

TEST_CASE("a parallel loop or a stage runner inside an alongside job runs on the job's own thread without new tasks, and the stages around it still run every block once", "[dynamics]") {
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  int calls = 0;

  for (const talus::dynamics::Claiming claiming : {talus::dynamics::Claiming::Shared, talus::dynamics::Claiming::Ranges}) {
    CHECK(alongsideRunsSerially(talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler), claiming, nullptr));
    CHECK(alongsideRunsSerially(talus::dynamics::resolveWorkers(8u, &inlineEnqueue, &inlineFinish, &calls, scheduler), claiming, &calls));
  }
}

TEST_CASE("schedulers are destroyed right after stage runs, through changing worker counts", "[.][schedulerstress]") {
  constexpr std::array<uint32_t, 12> counts{8u, 2u, 32u, 5u, 16u, 3u, 24u, 8u, 12u, 32u, 4u, 7u};
  constexpr uint32_t rounds = 20000u;
  std::atomic<uint32_t> done{0u};
  std::jthread watchdog([&done](std::stop_token stop) {
    uint32_t seen = 0u;
    auto moved = std::chrono::steady_clock::now();

    while (!stop.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      if (done.load() != seen) {
        seen = done.load();
        moved = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - moved > std::chrono::seconds(20)) {
        std::fprintf(stderr, "schedulerstress: round %u made no progress for 20 s\n", seen);
        std::fflush(stderr);
        std::_Exit(3);
      }
    }
  });
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  uint32_t wrong = 0u;

  for (uint32_t round = 0u; round != rounds; ++round) {
    const talus::dynamics::Workers workers =
      talus::dynamics::resolveWorkers(counts[round % counts.size()], nullptr, nullptr, nullptr, scheduler);

    if (!runStagesOnce(workers)) {
      ++wrong;
    }

    if (round % 3u == 2u) {
      scheduler.reset();
    }

    done.store(round + 1u);
  }

  scheduler.reset();

  CHECK(wrong == 0u);
}

TEST_CASE("the lattice narrowphase gives the same contacts on 1 and 4 workers", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto block = latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addLattice(bodies, block, lanes, glm::dvec3(0.0, 0.125, 0.0), glm::dvec3(0.0), 0.5, &blockField);
  talus::dynamics::addLattice(bodies, block, lanes, glm::dvec3(0.5, 0.125, 0.0), glm::dvec3(0.0), 0.5);

  for (int index = 0; index != 6; ++index) {
    talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.06 + 0.03 * index, 0.176, 0.0), glm::dvec3(0.0), 0.5, 3u);
  }

  talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(0.5, 0.176, 0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.05);
  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers four =
    talus::dynamics::resolveWorkers(4u, nullptr, nullptr, nullptr, scheduler);
  const std::vector<talus::dynamics::Contact> serial =
    talus::dynamics::narrowphase(bodies, pairs, 0.05);
  const std::vector<talus::dynamics::Contact> threaded =
    talus::dynamics::narrowphase(bodies, pairs, 0.05, four);

  REQUIRE(!serial.empty());
  REQUIRE(threaded.size() == serial.size());

  for (size_t index = 0u; index != serial.size(); ++index) {
    CHECK(threaded[index].key == serial[index].key);
    CHECK(threaded[index].pair == serial[index].pair);
    CHECK(threaded[index].manifold == serial[index].manifold);
    CHECK(threaded[index].gap == serial[index].gap);
    CHECK(threaded[index].basis[0] == serial[index].basis[0]);
  }
}

namespace {

struct SmallScene {
  talus::field::SampledSdf floor = floorField(1.0);
  talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::field::SampledSdf blockField;
  std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::dynamics::World world;
};

std::unique_ptr<SmallScene> smallScene(uint32_t workers, talus::dynamics::EnqueueTaskCallback* enqueue, talus::dynamics::FinishTaskCallback* finish, void* context) {
  auto scene = std::make_unique<SmallScene>();
  const auto block = latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, scene->lanes, scene->blockField);

  talus::dynamics::addStatic(scene->world.bodies, scene->floor, 0.5);
  talus::dynamics::addLattice(scene->world.bodies, block, scene->lanes, glm::dvec3(0.6, 0.125, 0.0), glm::dvec3(0.0), 0.5, &scene->blockField);
  talus::dynamics::addRigidBox(scene->world.bodies, scene->crate, glm::dvec3(0.025), 0.5, glm::dvec3(0.6, 0.176, 0.0), glm::dvec3(0.0), 0.5, 3u);

  for (int level = 0; level != 6; ++level) {
    talus::dynamics::addRigidBox(scene->world.bodies, scene->crate, glm::dvec3(0.025), 0.5, glm::dvec3(0.0, 0.125 + 0.0505 * level, 0.0), glm::dvec3(0.0), 0.5, 3u);
  }

  for (int index = 0; index != 8; ++index) {
    talus::dynamics::addRigidBox(scene->world.bodies, scene->crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.6 + 0.15 * index, 0.3, -0.4), glm::dvec3(0.2, 0.0, 0.0), 0.5, 3u);
  }

  scene->world.workerCount = workers;
  scene->world.enqueueTask = enqueue;
  scene->world.finishTask = finish;
  scene->world.userTaskContext = context;

  return scene;
}

uint64_t smallSceneHash(uint32_t workers, talus::dynamics::EnqueueTaskCallback* enqueue, talus::dynamics::FinishTaskCallback* finish, void* context, uint32_t width = 0u) {
  const std::unique_ptr<SmallScene> scene = smallScene(workers, enqueue, finish, context);

  scene->world.simdWidth = width;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    if (talus::dynamics::step(scene->world).nonFinite) {
      return 0u;
    }
  }

  return stateHash(scene->world);
}

}

TEST_CASE("a world steps the same with 1, 4 and host workers", "[dynamics]") {
  int calls = 0;
  const uint64_t serial = smallSceneHash(1u, nullptr, nullptr, nullptr);

  REQUIRE(serial != 0u);
  CHECK(smallSceneHash(4u, nullptr, nullptr, nullptr) == serial);
  CHECK(smallSceneHash(8u, &inlineEnqueue, &inlineFinish, &calls) == serial);
  CHECK(calls > 0);
}

namespace {

uint64_t sampledBoxesHash(uint32_t workers, uint32_t width) {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::World world;

  world.workerCount = workers;
  world.simdWidth = width;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int column = 0; column != 4; ++column) {
    for (int level = 0; level != 4; ++level) {
      const uint32_t index = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.1 + 0.0505 * column, 0.125 + 0.0505 * level, 0.0), glm::dvec3(0.0), 0.5, 3u);

      world.bodies.rigids[index].shape = talus::dynamics::RigidShape::Sampled;
    }
  }

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    if (talus::dynamics::step(world).nonFinite) {
      return 0u;
    }
  }

  return stateHash(world);
}

}

TEST_CASE("sampled boxes step the same on 1, 2, 4 and 8 workers", "[dynamics]") {
  const uint64_t serial = sampledBoxesHash(1u, 0u);

  REQUIRE(serial != 0u);

  for (const uint32_t workers : {2u, 4u, 8u}) {
    CHECK(sampledBoxesHash(workers, 0u) == serial);
  }
}

namespace {

uint64_t pileHash(uint32_t workers, talus::dynamics::EnqueueTaskCallback* enqueue, talus::dynamics::FinishTaskCallback* finish, void* context, bool loose, uint32_t width = 0u) {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  world.workerCount = workers;
  world.enqueueTask = enqueue;
  world.finishTask = finish;
  world.userTaskContext = context;
  world.simdWidth = width;

  if (!loose) {
    talus::dynamics::addStatic(world.bodies, floor, 0.5);
  }

  const int across = loose ? 16 : 10;

  for (int row = 0; row != across; ++row) {
    for (int column = 0; column != across; ++column) {
      for (int level = 0; level != across; ++level) {
        const glm::dvec3 place = loose
          ? glm::dvec3(-4.0 + 0.5 * row, 20.0 + 0.5 * level, -4.0 + 0.5 * column)
          : glm::dvec3(-0.3 + 0.0605 * row, 0.13 + 0.0605 * level, -0.3 + 0.0605 * column);

        talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, place, glm::dvec3(loose ? 0.1 * (row % 3) : 0.0, 0.0, 0.0), 0.5, 3u);
      }
    }
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    if (talus::dynamics::step(world).nonFinite) {
      return 0u;
    }
  }

  return stateHash(world);
}

}

TEST_CASE("threaded state hashes equal the serial ones", "[.][threadhash]") {
  int calls = 0;
  std::vector<uint32_t> widths{4u};

  if (talus::dynamics::hasAvx2()) {
    widths.push_back(8u);
  }

  for (const bool loose : {false, true}) {
    const uint64_t serial = pileHash(1u, nullptr, nullptr, nullptr, loose, 1u);

    std::printf("threadhash %s serial %016llx\n", loose ? "loose" : "pile", static_cast<unsigned long long>(serial));
    REQUIRE(serial != 0u);

    for (const uint32_t workers : {2u, 4u, 8u}) {
      CHECK(pileHash(workers, nullptr, nullptr, nullptr, loose) == serial);
    }

    for (const uint32_t width : widths) {
      CHECK(pileHash(1u, nullptr, nullptr, nullptr, loose, width) == serial);
      CHECK(pileHash(8u, nullptr, nullptr, nullptr, loose, width) == serial);
    }

    CHECK(pileHash(8u, &inlineEnqueue, &inlineFinish, &calls, loose) == serial);
  }

  for (const uint32_t workers : {2u, 4u, 8u}) {
    CHECK(smallSceneHash(workers, nullptr, nullptr, nullptr) == smallSceneHash(1u, nullptr, nullptr, nullptr, 1u));
  }
}

namespace {

uint64_t platformHash(uint32_t workers, uint32_t width = 0u) {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf slab = boxField(glm::dvec3(0.4, 0.05, 0.1), 0.01);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::World world;

  world.workerCount = workers;
  world.simdWidth = width;
  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addRigidBox(world.bodies, slab, glm::dvec3(0.4, 0.05, 0.1), 0.0, glm::dvec3(0.0, 0.3, 0.0), glm::dvec3(0.0), 0.5, 3u);

  for (int index = 0; index != 6; ++index) {
    talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.5, glm::dvec3(-0.3 + 0.12 * index, 0.3755, 0.0), glm::dvec3(0.0), 0.5, 3u);
  }

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    if (talus::dynamics::step(world).nonFinite) {
      return 0u;
    }
  }

  return stateHash(world);
}

}

TEST_CASE("boxes on a zero-mass platform step the same on 1 and 4 workers", "[dynamics]") {
  const uint64_t serial = platformHash(1u);

  REQUIRE(serial != 0u);
  CHECK(platformHash(4u) == serial);
}

TEST_CASE("worlds step the same at contact widths 1, 4 and 8", "[dynamics]") {
  std::vector<uint32_t> widths{4u};

  if (talus::dynamics::hasAvx2()) {
    widths.push_back(8u);
  }

  const uint64_t small = smallSceneHash(1u, nullptr, nullptr, nullptr, 1u);
  const uint64_t platform = platformHash(1u, 1u);
  const uint64_t sampled = sampledBoxesHash(1u, 1u);

  REQUIRE(small != 0u);
  REQUIRE(platform != 0u);
  REQUIRE(sampled != 0u);

  for (const uint32_t width : widths) {
    for (const uint32_t workers : {1u, 8u}) {
      CHECK(smallSceneHash(workers, nullptr, nullptr, nullptr, width) == small);
      CHECK(platformHash(workers, width) == platform);
      CHECK(sampledBoxesHash(workers, width) == sampled);
    }
  }

  const std::unique_ptr<SmallScene> scene = smallScene(1u, nullptr, nullptr, nullptr);
  uint32_t wide = 0u;

  scene->world.simdWidth = 4u;

  for (uint32_t frame = 0u; frame != 5u; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(scene->world);

    REQUIRE(!report.nonFinite);
    wide += report.widePairs;
  }

  CHECK(wide > 0u);
}

TEST_CASE("profile scene: a lone soft block on the floor", "[.][lonesoft]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField(1.0);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.0, 0.125, 0.0), glm::dvec3(0.0), 0.5, &blockField);

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  double elapsed = 0.0;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    const auto before = std::chrono::steady_clock::now();

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    elapsed += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
  }

  std::printf("lonesoft ms %.3f\n", elapsed / 60.0);
}

TEST_CASE("the contact solver width follows the request and the CPU", "[dynamics]") {
  CHECK(talus::dynamics::simdWidthFor(1u, true) == 1u);
  CHECK(talus::dynamics::simdWidthFor(1u, false) == 1u);
  CHECK(talus::dynamics::simdWidthFor(4u, true) == 4u);
  CHECK(talus::dynamics::simdWidthFor(8u, true) == 8u);
  CHECK(talus::dynamics::simdWidthFor(8u, false) == 4u);
  CHECK(talus::dynamics::simdWidthFor(0u, true) == 8u);
  CHECK(talus::dynamics::simdWidthFor(0u, false) == 4u);
  CHECK(talus::dynamics::simdWidthFor(2u, true) == 8u);
  CHECK(talus::dynamics::simdWidthFor(16u, false) == 4u);
  CHECK(talus::dynamics::resolveSimdWidth(1u) == 1u);
  CHECK(talus::dynamics::resolveSimdWidth(0u) == (talus::dynamics::hasAvx2() ? 8u : 4u));
  CHECK(talus::dynamics::resolveSimdWidth(8u) == (talus::dynamics::hasAvx2() ? 8u : 4u));
}

namespace {

double boxOverlap(const talus::dynamics::RigidBody& a, const talus::dynamics::RigidBody& b) {
  const glm::dmat3 ra = glm::mat3_cast(a.rotation);
  const glm::dmat3 rb = glm::mat3_cast(b.rotation);
  const glm::dvec3 d = b.position - a.position;
  std::vector<glm::dvec3> axes;

  for (int i = 0; i != 3; ++i) {
    axes.push_back(ra[i]);
    axes.push_back(rb[i]);

    for (int j = 0; j != 3; ++j) {
      const glm::dvec3 c = glm::cross(ra[i], rb[j]);

      if (glm::length(c) > 1.0e-6) {
        axes.push_back(glm::normalize(c));
      }
    }
  }

  double depth = std::numeric_limits<double>::max();

  for (const glm::dvec3& axis : axes) {
    double reach = 0.0;

    for (int i = 0; i != 3; ++i) {
      reach += a.halfExtents[i] * std::abs(glm::dot(ra[i], axis))
               + b.halfExtents[i] * std::abs(glm::dot(rb[i], axis));
    }

    depth = std::min(depth, reach - std::abs(glm::dot(d, axis)));
  }

  return std::max(depth, 0.0);
}

double floorOverlap(const talus::dynamics::RigidBody& body) {
  const glm::dmat3 turn = glm::mat3_cast(body.rotation);
  double lowest = std::numeric_limits<double>::max();

  for (int corner = 0; corner != 8; ++corner) {
    const glm::dvec3 sign(
      (corner & 1) != 0 ? 1.0 : -1.0,
      (corner & 2) != 0 ? 1.0 : -1.0,
      (corner & 4) != 0 ? 1.0 : -1.0
    );

    lowest = std::min(lowest, (body.position + turn * (sign * body.halfExtents)).y);
  }

  return std::max(0.1 - lowest, 0.0);
}

struct RowResult {
  double pair = 0.0;
  double floor = 0.0;
  glm::dvec3 last{0.0};
};

RowResult runRow(const std::vector<std::pair<glm::dvec3, glm::dvec3>>& boxes, double friction) {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;
  RowResult result;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (const auto& [place, velocity] : boxes) {
    talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, place, velocity, friction, 3u);
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    if (world.bodies.rigids.size() > 1u) {
      result.pair = std::max(result.pair, boxOverlap(world.bodies.rigids[0], world.bodies.rigids[1]));
    }

    for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
      result.floor = std::max(result.floor, floorOverlap(body));
    }
  }

  result.last = world.bodies.rigids.back().position;

  return result;
}

}

TEST_CASE("recycle row: a box slides into a resting box", "[.][recyclerows]") {
  const RowResult row = runRow({{glm::dvec3(0.0, 0.13, 0.0), glm::dvec3(0.0)}, {glm::dvec3(0.09, 0.13, 0.0), glm::dvec3(-1.0, 0.0, 0.0)}}, 0.0);

  std::printf("recyclerow slide | max box overlap mm %.3f | max floor overlap mm %.3f | last box x %.4f z %.4f\n", 1000.0 * row.pair, 1000.0 * row.floor, row.last.x, row.last.z);
  CHECK(row.pair <= 0.002);
  CHECK(row.floor <= 0.002);
}

TEST_CASE("recycle row: two boxes meet edge first", "[.][recyclerows]") {
  const RowResult row = runRow({{glm::dvec3(0.0, 0.13, 0.0), glm::dvec3(0.0)}, {glm::dvec3(0.08, 0.13, 0.08), glm::dvec3(-0.7, 0.0, -0.7)}}, 0.0);

  std::printf("recyclerow diagonal | max box overlap mm %.3f | max floor overlap mm %.3f | last box x %.4f z %.4f\n", 1000.0 * row.pair, 1000.0 * row.floor, row.last.x, row.last.z);
  CHECK(row.pair <= 0.010347);
  CHECK(row.floor <= 0.002);
}

TEST_CASE("recycle row: two boxes meet edge first, slowly", "[.][recyclerows]") {
  const RowResult row = runRow({{glm::dvec3(0.0, 0.13, 0.0), glm::dvec3(0.0)}, {glm::dvec3(0.0605, 0.13, 0.0605), glm::dvec3(-0.05, 0.0, -0.05)}}, 0.0);

  std::printf("recyclerow diagonal-slow | max box overlap mm %.3f | max floor overlap mm %.3f | last box x %.4f z %.4f\n", 1000.0 * row.pair, 1000.0 * row.floor, row.last.x, row.last.z);
  CHECK(row.pair <= 0.013667);
  CHECK(row.floor <= 0.002);
}

TEST_CASE("recycle row: a box drops 3 cm onto the floor", "[.][recyclerows]") {
  const RowResult row = runRow({{glm::dvec3(0.0, 0.16, 0.0), glm::dvec3(0.0)}}, 0.5);

  std::printf("recyclerow drop | max floor overlap mm %.3f | last box x %.4f z %.4f\n", 1000.0 * row.floor, row.last.x, row.last.z);
  CHECK(row.floor <= 0.002);
}

TEST_CASE("recycle row: the settled pile", "[.][recyclerows]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floor, 0.5);

  for (int row = 0; row != 10; ++row) {
    for (int column = 0; column != 10; ++column) {
      for (int level = 0; level != 10; ++level) {
        talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.3 + 0.0605 * row, 0.13 + 0.0605 * level, -0.3 + 0.0605 * column), glm::dvec3(0.0), 0.5, 3u);
      }
    }
  }

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  double pair = 0.0;
  double lowest = 0.0;
  const std::vector<talus::dynamics::RigidBody>& rigids = world.bodies.rigids;

  for (size_t i = 0u; i != rigids.size(); ++i) {
    lowest = std::max(lowest, floorOverlap(rigids[i]));

    for (size_t j = i + 1u; j != rigids.size(); ++j) {
      if (glm::length(rigids[j].position - rigids[i].position) < 0.12) {
        pair = std::max(pair, boxOverlap(rigids[i], rigids[j]));
      }
    }
  }

  uint32_t touching = 0u;

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (world.manifolds.live(slot) && !world.manifolds.at(slot).points.empty()) {
      ++touching;
    }
  }

  std::printf("recyclerow pile | max box overlap mm %.3f | max floor overlap mm %.3f | touching pairs %u\n", 1000.0 * pair, 1000.0 * lowest, touching);
  CHECK(pair <= 0.005140);
  CHECK(lowest <= 0.002);
}

TEST_CASE("the broadphase gives the same pairs in the same order on 1, 4 and 8 workers", "[dynamics]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto block = latticeBlock(glm::dvec3(0.1, 0.025, 0.1), 0.025, lanes, blockField);
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);
  talus::dynamics::addLattice(bodies, block, lanes, glm::dvec3(0.0, 0.125, 0.0), glm::dvec3(0.0), 0.5, &blockField);
  talus::dynamics::addLattice(bodies, block, lanes, glm::dvec3(0.4, 0.125, 0.0), glm::dvec3(0.0), 0.5);

  for (int index = 0; index != 300; ++index) {
    const glm::dvec3 place(-0.8 + 0.055 * (index % 30), 0.13 + 0.055 * (index / 30 % 5), -0.3 + 0.21 * (index / 150) + 0.01 * (index % 7));

    talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.025), 0.5, place, glm::dvec3(0.1 * (index % 3), 0.0, 0.0), 0.5, 3u);
  }

  std::vector<double> travel(bodies.lattices.size() + bodies.rigids.size());

  for (size_t index = 0u; index != travel.size(); ++index) {
    travel[index] = 0.001 * static_cast<double>(index % 5u);
  }

  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const std::vector<talus::dynamics::BodyPair> serial = talus::dynamics::broadphase(bodies, 0.02, travel);

  REQUIRE(serial.size() > 300u);

  for (const uint32_t count : {4u, 8u}) {
    const talus::dynamics::Workers workers =
      talus::dynamics::resolveWorkers(count, nullptr, nullptr, nullptr, scheduler);
    const std::vector<talus::dynamics::BodyPair> threaded =
      talus::dynamics::broadphase(bodies, 0.02, travel, workers);

    REQUIRE(threaded.size() == serial.size());

    for (size_t index = 0u; index != serial.size(); ++index) {
      CHECK(threaded[index].first.side == serial[index].first.side);
      CHECK(threaded[index].first.index == serial[index].first.index);
      CHECK(threaded[index].second.side == serial[index].second.side);
      CHECK(threaded[index].second.index == serial[index].second.index);
    }
  }
}

TEST_CASE("a store synced on 8 workers gets the same slots as one synced serially", "[dynamics]") {
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf crate = boxField(glm::dvec3(0.025), 0.005);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floor, 0.5);

  for (int index = 0; index != 200; ++index) {
    const glm::dvec3 place(-0.5 + 0.052 * (index % 20), 0.13 + 0.052 * (index / 20 % 5), -0.1 + 0.052 * (index / 100));

    talus::dynamics::addRigidBox(bodies, crate, glm::dvec3(0.025), 0.5, place, glm::dvec3(0.0), 0.5, 3u);
  }

  std::vector<talus::dynamics::BodyPair> first = talus::dynamics::broadphase(bodies, 0.02);

  REQUIRE(first.size() > 200u);
  first.push_back(first[3]);

  std::vector<talus::dynamics::BodyPair> second;

  for (size_t index = 0u; index != first.size(); ++index) {
    if (index % 5u != 0u) {
      second.push_back(first[index]);
    }
  }

  second.push_back(first[10]);
  second.push_back(first[0]);

  std::shared_ptr<talus::dynamics::Scheduler> scheduler;
  const talus::dynamics::Workers workers =
    talus::dynamics::resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);
  talus::dynamics::ManifoldCache serial;
  talus::dynamics::ManifoldCache threaded;

  for (const std::vector<talus::dynamics::BodyPair>* list : {&first, &second, &first}) {
    serial.sync(bodies, *list);
    threaded.sync(bodies, *list, workers);

    REQUIRE(threaded.size() == serial.size());

    for (uint32_t slot = 0u; slot != serial.size(); ++slot) {
      REQUIRE(threaded.live(slot) == serial.live(slot));

      if (serial.live(slot)) {
        CHECK(threaded.at(slot).pair == serial.at(slot).pair);
        CHECK(threaded.at(slot).first.index == serial.at(slot).first.index);
        CHECK(threaded.at(slot).second.index == serial.at(slot).second.index);
      }
    }
  }
}

TEST_CASE("a world that keeps its softstep arena steps the same as with a fresh arena", "[dynamics]") {
  const std::unique_ptr<SmallScene> kept = smallScene(4u, nullptr, nullptr, nullptr);
  const std::unique_ptr<SmallScene> fresh = smallScene(4u, nullptr, nullptr, nullptr);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(kept->world).nonFinite);
    fresh->world.softstepArena = talus::dynamics::makeSoftstepArena();
    REQUIRE(!talus::dynamics::step(fresh->world).nonFinite);
  }

  CHECK(kept->world.softstepArena != nullptr);
  CHECK(stateHash(kept->world) == stateHash(fresh->world));

  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World big;

  big.workerCount = 4u;
  talus::dynamics::addStatic(big.bodies, floor, 0.5);

  for (int index = 0; index != 125; ++index) {
    talus::dynamics::addRigidBox(big.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.12 + 0.0605 * (index % 5), 0.13 + 0.0605 * (index / 25), -0.12 + 0.0605 * (index / 5 % 5)), glm::dvec3(0.0), 0.5, 3u);
  }

  for (uint32_t frame = 0u; frame != 5u; ++frame) {
    REQUIRE(!talus::dynamics::step(big).nonFinite);
  }

  const std::unique_ptr<SmallScene> borrowed = smallScene(4u, nullptr, nullptr, nullptr);

  borrowed->world.softstepArena = big.softstepArena;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(borrowed->world).nonFinite);
  }

  CHECK(stateHash(borrowed->world) == smallSceneHash(4u, nullptr, nullptr, nullptr));
}

TEST_CASE("islands filled into a reused list match freshly built islands", "[dynamics]") {
  const std::unique_ptr<SmallScene> small = smallScene(1u, nullptr, nullptr, nullptr);
  const talus::field::SampledSdf floor = floorField(1.0);
  const talus::field::SampledSdf brick = boxField(glm::dvec3(0.03), 0.006);
  talus::dynamics::World spread;

  talus::dynamics::addStatic(spread.bodies, floor, 0.5);

  for (int index = 0; index != 64; ++index) {
    talus::dynamics::addRigidBox(spread.bodies, brick, glm::dvec3(0.03), 0.5, glm::dvec3(-0.8 + 0.2 * (index % 8), 0.13 + 0.0605 * (index % 3), -0.8 + 0.2 * (index / 8)), glm::dvec3(0.0), 0.5, 3u);
  }

  for (uint32_t frame = 0u; frame != 10u; ++frame) {
    REQUIRE(!talus::dynamics::step(small->world).nonFinite);
    REQUIRE(!talus::dynamics::step(spread).nonFinite);
  }

  std::vector<talus::dynamics::Island> reused;

  for (const talus::dynamics::World* world : {&spread, &small->world, &spread}) {
    talus::dynamics::islandsOf(world->bodies, world->manifolds, world->contacts, reused);

    const std::vector<talus::dynamics::Island> fresh =
      talus::dynamics::islandsOf(world->bodies, world->manifolds, world->contacts);

    REQUIRE(reused.size() == fresh.size());

    for (size_t island = 0u; island != fresh.size(); ++island) {
      REQUIRE(reused[island].bodies.size() == fresh[island].bodies.size());

      for (size_t body = 0u; body != fresh[island].bodies.size(); ++body) {
        CHECK(reused[island].bodies[body].side == fresh[island].bodies[body].side);
        CHECK(reused[island].bodies[body].index == fresh[island].bodies[body].index);
      }

      CHECK(reused[island].contacts == fresh[island].contacts);
      CHECK(reused[island].pairs == fresh[island].pairs);
      CHECK(reused[island].external == fresh[island].external);
      CHECK(reused[island].owner == fresh[island].owner);
    }
  }

  CHECK(reused.size() > 60u);
}
