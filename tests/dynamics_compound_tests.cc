#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"
#include "pinning.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/jacobi/jacobi.h>
#include <talus/dynamics/ledger/ledger.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/rays/rays.h>
#include <talus/dynamics/softstep/simdwidth.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

glm::dmat3 diagonal(const glm::dvec3& values) {
  glm::dmat3 matrix(0.0);

  matrix[0][0] = values.x;
  matrix[1][1] = values.y;
  matrix[2][2] = values.z;

  return matrix;
}

double largestEntry(const glm::dmat3& matrix) {
  double largest = 0.0;

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      largest = std::max(largest, std::abs(matrix[column][row]));
    }
  }

  return largest;
}

}

TEST_CASE("the Jacobi solver leaves a diagonal matrix as it is", "[dynamics][compound]") {
  const glm::dvec3 values(3.0, 1.5, 7.25);
  const talus::dynamics::SymmetricEigen eigen = talus::dynamics::jacobiEigen(diagonal(values));

  CHECK(eigen.values == values);
  CHECK(eigen.vectors == glm::dmat3(1.0));
}

TEST_CASE("the Jacobi solver finds the principal axes of a turned tensor", "[dynamics][compound]") {
  const std::array<glm::dvec3, 3> spectra{
    glm::dvec3(1.0, 2.0, 3.0),
    glm::dvec3(5.0, 5.0, 0.25),
    glm::dvec3(0.001, 40.0, 40.0)
  };
  const std::array<glm::dquat, 3> turns{
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 2.0, -0.5))),
    glm::angleAxis(2.9, glm::normalize(glm::dvec3(-0.3, 0.1, 1.0))),
    glm::angleAxis(1.3, glm::normalize(glm::dvec3(0.0, 1.0, 1.0)))
  };

  for (const glm::dvec3& spectrum : spectra) {
    for (const glm::dquat& turn : turns) {
      const glm::dmat3 frame = glm::mat3_cast(turn);
      const glm::dmat3 tensor = frame * diagonal(spectrum) * glm::transpose(frame);
      const talus::dynamics::SymmetricEigen eigen = talus::dynamics::jacobiEigen(tensor);
      const double scale = std::max({spectrum.x, spectrum.y, spectrum.z});
      std::array<double, 3> expected{spectrum.x, spectrum.y, spectrum.z};
      std::array<double, 3> found{eigen.values.x, eigen.values.y, eigen.values.z};

      std::sort(expected.begin(), expected.end());
      std::sort(found.begin(), found.end());

      for (size_t index = 0u; index != expected.size(); ++index) {
        CHECK(std::abs(found[index] - expected[index]) <= 1.0e-12 * scale);
      }

      CHECK(largestEntry(glm::transpose(eigen.vectors) * eigen.vectors - glm::dmat3(1.0)) <= 1.0e-12);
      CHECK(glm::determinant(eigen.vectors) > 0.0);

      for (int axis = 0; axis != 3; ++axis) {
        const glm::dvec3 residual =
          tensor * eigen.vectors[axis] - eigen.values[axis] * eigen.vectors[axis];

        CHECK(glm::length(residual) <= 1.0e-12 * scale);
      }
    }
  }
}

namespace {

talus::dynamics::CompoundChild part(
  const glm::dvec3& centre,
  const glm::dvec3& half,
  double density,
  uint32_t key,
  const glm::dquat& rotation = glm::dquat(1.0, 0.0, 0.0, 0.0)
) {
  talus::dynamics::CompoundChild child;

  child.centre = centre;
  child.rotation = rotation;
  child.half = half;
  child.density = density;
  child.key = key;

  return child;
}

std::vector<talus::dynamics::CompoundChild> cubeOfEight(double half, double density, uint32_t firstKey) {
  std::vector<talus::dynamics::CompoundChild> children;

  for (uint32_t index = 0u; index != 8u; ++index) {
    const glm::dvec3 sign(
      (index & 1u) != 0u ? 1.0 : -1.0,
      (index & 2u) != 0u ? 1.0 : -1.0,
      (index & 4u) != 0u ? 1.0 : -1.0
    );

    children.push_back(part(half * sign, glm::dvec3(half), density, firstKey + index));
  }

  return children;
}

std::vector<talus::dynamics::CompoundChild> lShape() {
  return {
    part(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.3, 0.05, 0.1), 1000.0, 1u),
    part(glm::dvec3(-0.25, 0.25, 0.0), glm::dvec3(0.05, 0.2, 0.1), 2500.0, 2u),
    part(glm::dvec3(0.2, 0.1, 0.15), glm::dvec3(0.08, 0.05, 0.05), 400.0, 3u, glm::angleAxis(0.4, glm::dvec3(0.0, 1.0, 0.0)))
  };
}

std::array<glm::dvec3, 8> cornersOf(
  const glm::dvec3& position,
  const glm::dquat& rotation,
  const talus::dynamics::CompoundChild& child
) {
  std::array<glm::dvec3, 8> corners{};
  const glm::dmat3 body = glm::mat3_cast(rotation);
  const glm::dmat3 turn = glm::mat3_cast(child.rotation);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 sign(
      (corner & 1u) != 0u ? 1.0 : -1.0,
      (corner & 2u) != 0u ? 1.0 : -1.0,
      (corner & 4u) != 0u ? 1.0 : -1.0
    );

    corners[corner] = position + body * (child.centre + turn * (sign * child.half));
  }

  return corners;
}

}

TEST_CASE("a compound shape keeps its children in key order with cached bounds", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> children{
    part(glm::dvec3(0.5, 0.0, 0.0), glm::dvec3(0.25, 0.125, 0.125), 1000.0, 7u),
    part(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.25, 0.125, 0.125), 1000.0, 2u),
    part(glm::dvec3(0.0, 0.375, 0.0), glm::dvec3(0.125, 0.25, 0.125), 1000.0, 4u)
  };
  const std::shared_ptr<const talus::dynamics::CompoundShape> shape =
    talus::dynamics::makeCompoundShape(children);

  REQUIRE(shape->children.size() == 3u);
  CHECK(shape->children[0].key == 2u);
  CHECK(shape->children[1].key == 4u);
  CHECK(shape->children[2].key == 7u);
  CHECK(shape->bounds.low == glm::dvec3(-0.25, -0.125, -0.125));
  CHECK(shape->bounds.high == glm::dvec3(0.75, 0.625, 0.125));
  CHECK(shape->thinnest == 0.125);
  CHECK(shape->reach == glm::length(glm::dvec3(0.75, 0.125, 0.125)));
  CHECK(talus::dynamics::findChild(*shape, 4u) == std::optional<uint32_t>(1u));
  CHECK(!talus::dynamics::findChild(*shape, 5u).has_value());
}

TEST_CASE("a compound shape refuses children it cannot hold", "[dynamics][compound]") {
  using talus::dynamics::makeCompoundShape;

  CHECK_THROWS_AS((void)makeCompoundShape({}), std::runtime_error);
  CHECK_THROWS_AS(
    (void)makeCompoundShape({part(glm::dvec3(0.0), glm::dvec3(0.1), 1.0, 3u), part(glm::dvec3(1.0), glm::dvec3(0.1), 1.0, 3u)}),
    std::runtime_error
  );
  CHECK_THROWS_AS(
    (void)makeCompoundShape({part(glm::dvec3(0.0), glm::dvec3(0.1, 0.0, 0.1), 1.0, 3u)}),
    std::runtime_error
  );
  CHECK_THROWS_AS(
    (void)makeCompoundShape({part(glm::dvec3(0.0), glm::dvec3(0.1), -1.0, 3u)}),
    std::runtime_error
  );
}

TEST_CASE("the child tree finds exactly the children a brute-force search finds", "[dynamics][compound]") {
  std::vector<talus::dynamics::CompoundChild> children;
  uint32_t count = 0u;

  for (int x = 0; x != 6; ++x) {
    for (int y = 0; y != 5; ++y) {
      for (int z = 0; z != 4; ++z) {
        const glm::dquat turn = glm::angleAxis(
          0.3 * static_cast<double>(x + y + z),
          glm::normalize(glm::dvec3(1.0, 0.5 * y, 0.25 * z + 0.1))
        );

        children.push_back(part(glm::dvec3(0.3 * x, 0.3 * y, 0.3 * z), glm::dvec3(0.12, 0.08, 0.05), 1000.0, (count * 37u) % 1009u, turn));
        count += 1u;
      }
    }
  }

  const std::shared_ptr<const talus::dynamics::CompoundShape> shape =
    talus::dynamics::makeCompoundShape(children);
  std::vector<uint32_t> found;
  uint64_t state = 12345u;
  const auto next = [&state] {
    state = state * 6364136223846793005ull + 1442695040888963407ull;

    return static_cast<double>(state >> 11u) / 9007199254740992.0;
  };

  for (int probe = 0; probe != 64; ++probe) {
    const double x = -0.3 + 2.1 * next();
    const double y = -0.3 + 1.8 * next();
    const double z = -0.3 + 1.5 * next();
    const double width = 0.05 + 0.4 * next();
    const double height = 0.05 + 0.4 * next();
    const double depth = 0.05 + 0.4 * next();
    const glm::dvec3 centre(x, y, z);
    const glm::dvec3 reach(width, height, depth);
    const talus::dynamics::Bounds box{centre - reach, centre + reach};
    std::vector<uint32_t> expected;

    for (uint32_t index = 0u; index != shape->children.size(); ++index) {
      if (talus::dynamics::overlaps(shape->childBounds[index], box)) {
        expected.push_back(index);
      }
    }

    talus::dynamics::overlappingChildren(*shape, box, found);
    CHECK(found == expected);
  }
}

TEST_CASE("eight half-size children have the mass and moments of the box they fill", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> whole{part(glm::dvec3(0.0), glm::dvec3(0.05), 1000.0, 0u)};
  const talus::dynamics::PrincipalFrame one = talus::dynamics::principalFrame(whole);
  const talus::dynamics::PrincipalFrame eight =
    talus::dynamics::principalFrame(cubeOfEight(0.025, 1000.0, 0u));
  const double moment = one.mass * (0.1 * 0.1 + 0.1 * 0.1) / 12.0;

  CHECK(std::abs(one.mass - 1.0) <= 1.0e-12);
  CHECK(std::abs(eight.mass - one.mass) <= 1.0e-12 * one.mass);
  CHECK(glm::length(eight.centre) <= 1.0e-15);

  for (int axis = 0; axis != 3; ++axis) {
    CHECK(std::abs(one.moment[axis] - moment) <= 1.0e-12 * moment);
    CHECK(std::abs(eight.moment[axis] - moment) <= 1.0e-12 * moment);
  }
}

TEST_CASE("the principal frame of an L-shaped compound makes its inertia diagonal", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> children = lShape();
  const talus::dynamics::PrincipalFrame frame = talus::dynamics::principalFrame(children);
  const std::vector<talus::dynamics::CompoundChild> placed =
    talus::dynamics::childrenInFrame(children, frame.centre, frame.rotation);
  const talus::dynamics::CompoundMass inertia = talus::dynamics::compoundMass(placed);
  const double trace = frame.moment.x + frame.moment.y + frame.moment.z;

  CHECK(std::abs(inertia.mass - frame.mass) <= 1.0e-12 * frame.mass);
  CHECK(glm::length(inertia.centre) <= 1.0e-12);

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      const double expected = row == column ? frame.moment[row] : 0.0;

      CHECK(std::abs(inertia.inertia[column][row] - expected) <= 1.0e-12 * trace);
    }
  }

  for (size_t index = 0u; index != children.size(); ++index) {
    const std::array<glm::dvec3, 8> before =
      cornersOf(glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), children[index]);
    const std::array<glm::dvec3, 8> after = cornersOf(frame.centre, frame.rotation, placed[index]);

    for (size_t corner = 0u; corner != before.size(); ++corner) {
      CHECK(glm::length(after[corner] - before[corner]) <= 1.0e-12);
    }
  }
}

TEST_CASE("a child's samples are the box body's 26 samples in its own frame", "[dynamics][compound]") {
  const talus::field::SampledSdf field{};
  const glm::dvec3 half(0.05, 0.08, 0.03);
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, field, half, 1.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u);

  const std::array<glm::dvec3, talus::dynamics::childSampleCount> centred =
    talus::dynamics::childSamples(part(glm::dvec3(0.0), half, 1000.0, 0u));

  REQUIRE(bodies.rigids[0].samples.size() == talus::dynamics::childSampleCount);

  for (size_t index = 0u; index != centred.size(); ++index) {
    CHECK(centred[index] == bodies.rigids[0].samples[index]);
  }

  const talus::dynamics::CompoundChild moved = part(
    glm::dvec3(0.3, -0.2, 0.1),
    half,
    1000.0,
    0u,
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 1.0, 0.0)))
  );
  const std::array<glm::dvec3, talus::dynamics::childSampleCount> placed =
    talus::dynamics::childSamples(moved);

  for (size_t index = 0u; index != placed.size(); ++index) {
    CHECK(placed[index] == moved.centre + glm::mat3_cast(moved.rotation) * bodies.rigids[0].samples[index]);
  }
}

namespace {

const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);

talus::field::SampledSdf boxFieldAt(const glm::dvec3& half, double voxel) {
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

talus::field::SampledSdf boxShape(const glm::dvec3& half) {
  return boxFieldAt(half, std::min({half.x, half.y, half.z}) / 5.0);
}

talus::PhysicalMaterial rubber() {
  talus::PhysicalMaterial material;

  material.density = 1000.0f;
  material.youngsModulus = 1.2854547e6f;
  material.poissonsRatio = 0.45f;

  return material;
}

std::shared_ptr<const talus::PreparedMesh> latticeBlock(
  const glm::dvec3& half,
  double voxel,
  std::span<const talus::PhysicalMaterial> lanes,
  talus::field::SampledSdf& field
) {
  field = boxFieldAt(half, voxel);

  return talus::deformable::buildMesh(
    field,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
}

}

TEST_CASE("a compound body sits at its centre of mass in its principal frame", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> children = lShape();
  const glm::dvec3 position(1.0, 2.0, 3.0);
  const glm::dquat rotation = glm::angleAxis(0.6, glm::normalize(glm::dvec3(1.0, 1.0, 0.0)));
  talus::dynamics::BodySet bodies;
  const uint32_t index = talus::dynamics::addCompoundBody(
    bodies, children, position, rotation, glm::dvec3(0.5, 0.0, 0.0), 0.4, 0.1, false
  );
  const talus::dynamics::RigidBody& body = bodies.rigids[index];
  double mass = 0.0;
  glm::dvec3 moment(0.0);

  for (const talus::dynamics::CompoundChild& child : children) {
    mass += talus::dynamics::childMass(child);
    moment += talus::dynamics::childMass(child) * child.centre;
  }

  REQUIRE(body.shape == talus::dynamics::RigidShape::Compound);
  CHECK(std::abs(body.mass - mass) <= 1.0e-12 * mass);
  CHECK(glm::length(body.position - (position + glm::mat3_cast(rotation) * (moment / mass))) <= 1.0e-12);
  CHECK(body.velocity == glm::dvec3(0.5, 0.0, 0.0));
  CHECK(body.friction == 0.4);
  CHECK(body.restitution == 0.1);

  const talus::dynamics::CompoundMass local = talus::dynamics::compoundMass(body.compound->children);
  const double trace = body.moment.x + body.moment.y + body.moment.z;

  CHECK(glm::length(local.centre) <= 1.0e-12);

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      const double expected = row == column ? body.moment[row] : 0.0;

      CHECK(std::abs(local.inertia[column][row] - expected) <= 1.0e-12 * trace);
    }
  }

  for (const talus::dynamics::CompoundChild& child : children) {
    const std::optional<uint32_t> found = talus::dynamics::findChild(*body.compound, child.key);

    REQUIRE(found.has_value());

    const std::array<glm::dvec3, 8> given = cornersOf(position, rotation, child);
    const std::array<glm::dvec3, 8> held =
      cornersOf(body.position, body.rotation, body.compound->children[*found]);

    for (size_t corner = 0u; corner != given.size(); ++corner) {
      CHECK(glm::length(held[corner] - given[corner]) <= 1.0e-12);
    }
  }
}

TEST_CASE("an anchored compound keeps its pose and has no mass", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> children = lShape();
  const glm::dquat rotation = glm::normalize(glm::dquat(0.9, 0.1, -0.2, 0.3));
  talus::dynamics::BodySet bodies;
  const uint32_t index = talus::dynamics::addCompoundBody(
    bodies, children, glm::dvec3(4.0, 0.5, -1.0), rotation, glm::dvec3(3.0, 0.0, 0.0), 0.6, 0.0, true
  );
  const talus::dynamics::RigidBody& body = bodies.rigids[index];

  CHECK(body.mass == 0.0);
  CHECK(body.moment == glm::dvec3(0.0));
  CHECK(body.position == glm::dvec3(4.0, 0.5, -1.0));
  CHECK(body.rotation == glm::normalize(rotation));
  CHECK(body.velocity == glm::dvec3(0.0));
  REQUIRE(body.compound->children.size() == children.size());

  for (const talus::dynamics::CompoundChild& child : children) {
    const std::optional<uint32_t> found = talus::dynamics::findChild(*body.compound, child.key);

    REQUIRE(found.has_value());
    CHECK(body.compound->children[*found].centre == child.centre);
    CHECK(body.compound->children[*found].density == child.density);
  }
}

TEST_CASE("a compound's bounds hold every child's world box", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> children = lShape();
  const glm::dvec3 position(0.2, 1.0, -0.5);
  const glm::dquat rotation = glm::angleAxis(1.1, glm::normalize(glm::dvec3(0.3, 1.0, -0.4)));
  talus::dynamics::BodySet bodies;

  talus::dynamics::addCompoundBody(bodies, children, position, rotation, glm::dvec3(0.0), 0.5, 0.0, false);
  talus::dynamics::addCompoundBody(bodies, children, position, noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const talus::dynamics::Bounds turned =
    talus::dynamics::boundsOf(bodies, {talus::dynamics::Side::Rigid, 0u}, 0.0);
  const talus::dynamics::Bounds square =
    talus::dynamics::boundsOf(bodies, {talus::dynamics::Side::Rigid, 1u}, 0.02);
  talus::dynamics::Bounds expected{glm::dvec3(1.0e30), glm::dvec3(-1.0e30)};

  for (const talus::dynamics::CompoundChild& child : children) {
    for (const glm::dvec3& corner : cornersOf(position, rotation, child)) {
      CHECK(glm::all(glm::lessThanEqual(turned.low, corner + glm::dvec3(1.0e-12))));
      CHECK(glm::all(glm::greaterThanEqual(turned.high, corner - glm::dvec3(1.0e-12))));
    }

    expected = talus::dynamics::unite(
      expected,
      talus::dynamics::boxBoundsIn(child.centre, glm::mat3_cast(child.rotation), child.half)
    );
  }

  CHECK(glm::length(square.low - (position + expected.low - glm::dvec3(0.02))) <= 1.0e-12);
  CHECK(glm::length(square.high - (position + expected.high + glm::dvec3(0.02))) <= 1.0e-12);
}

TEST_CASE("a one-child compound has the bounds and reach of the same box", "[dynamics][compound]") {
  const talus::field::SampledSdf field{};
  const glm::dvec3 half(0.05, 0.07, 0.03);
  const glm::dvec3 position(0.3, 0.2, -0.1);
  const glm::dquat rotation = glm::angleAxis(0.5, glm::normalize(glm::dvec3(1.0, 2.0, 3.0)));
  const std::vector<talus::dynamics::CompoundChild> one{part(glm::dvec3(0.0), half, 1000.0, 0u)};
  talus::dynamics::BodySet bodies;

  talus::dynamics::addRigidBox(bodies, field, half, 1.0, position, glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addCompoundBody(bodies, one, position, noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  bodies.rigids[0].rotation = rotation;
  bodies.rigids[1].rotation = rotation;

  const talus::dynamics::Bounds box =
    talus::dynamics::boundsOf(bodies, {talus::dynamics::Side::Rigid, 0u}, 0.02);
  const talus::dynamics::Bounds compound =
    talus::dynamics::boundsOf(bodies, {talus::dynamics::Side::Rigid, 1u}, 0.02);

  CHECK(bodies.rigids[1].position == position);
  CHECK(bodies.rigids[1].compound->children[0].centre == glm::dvec3(0.0));
  CHECK(compound.low == box.low);
  CHECK(compound.high == box.high);
  CHECK(talus::dynamics::reachOf(bodies.rigids[1]) == talus::dynamics::reachOf(bodies.rigids[0]));
}

TEST_CASE("a moving compound without mass is refused", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> weightless{part(glm::dvec3(0.0), glm::dvec3(0.1), 0.0, 0u)};
  talus::dynamics::BodySet bodies;

  CHECK_THROWS_AS(
    talus::dynamics::addCompoundBody(bodies, weightless, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false),
    std::runtime_error
  );
  CHECK(bodies.rigids.empty());
  CHECK(bodies.nextId == 0u);

  talus::dynamics::addCompoundBody(bodies, weightless, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  CHECK(bodies.rigids.size() == 1u);
}

TEST_CASE("a compound next to a box, a sampled body and a lattice steps without reading a missing field", "[dynamics][compound]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.05, 0.025, 0.05), 0.025, lanes, blockField);
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t heap = talus::dynamics::addCompoundBody(
    world.bodies, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false
  );

  talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.12, 0.0, 0.0), glm::dvec3(-0.5, 0.0, 0.0), 0.5, 3u);

  const uint32_t sampled = talus::dynamics::addRigidBox(
    world.bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(-0.12, 0.0, 0.0), glm::dvec3(0.5, 0.0, 0.0), 0.5, 3u
  );

  world.bodies.rigids[sampled].shape = talus::dynamics::RigidShape::Sampled;
  talus::dynamics::addLattice(world.bodies, mesh, lanes, glm::dvec3(0.0, 0.1, 0.0), glm::dvec3(0.0, -0.5, 0.0), 0.5, &blockField);

  for (uint32_t frame = 0u; frame != 20u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (const talus::dynamics::Contact& contact : world.contacts) {
      CHECK(!(contact.a.side == talus::dynamics::Side::Rigid && contact.a.body == heap));
      CHECK(!(contact.b.side == talus::dynamics::Side::Rigid && contact.b.body == heap));
    }
  }
}

namespace {

talus::dynamics::BodySet twoCubes() {
  talus::dynamics::BodySet bodies;

  talus::dynamics::addCompoundBody(bodies, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  talus::dynamics::addCompoundBody(bodies, cubeOfEight(0.025, 1000.0, 100u), glm::dvec3(0.0, 0.15, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  return bodies;
}

talus::dynamics::Contact childRecord(
  uint32_t bodyA,
  uint32_t childA,
  uint32_t bodyB,
  uint32_t childB,
  uint64_t key,
  uint32_t manifold,
  double normal
) {
  talus::dynamics::Contact contact;

  contact.a.side = talus::dynamics::Side::Rigid;
  contact.a.body = bodyA;
  contact.a.child = childA;
  contact.a.local = glm::dvec3(0.01 * static_cast<double>(key), -0.05, 0.0);
  contact.b.side = talus::dynamics::Side::Rigid;
  contact.b.body = bodyB;
  contact.b.child = childB;
  contact.b.local = glm::dvec3(0.0, 0.05, 0.02 * static_cast<double>(key));
  contact.friction = 0.5;
  contact.key = key;
  contact.manifold = manifold;
  contact.memory.normal = normal;

  return contact;
}

uint32_t pairSlot(
  talus::dynamics::ManifoldCache& cache,
  const talus::dynamics::BodySet& bodies,
  uint32_t first,
  uint32_t second
) {
  const talus::dynamics::BodyRef a{talus::dynamics::Side::Rigid, first};
  const talus::dynamics::BodyRef b{talus::dynamics::Side::Rigid, second};

  cache.sync(bodies, std::vector<talus::dynamics::BodyPair>{{a, b}});

  return cache.find(talus::dynamics::pairKey(bodies, a, b));
}

}

TEST_CASE("a rebuilt point carries memory from the old point with the same child keys", "[dynamics][compound]") {
  const talus::dynamics::BodySet bodies = twoCubes();
  talus::dynamics::ManifoldCache cache;
  const uint32_t slot = pairSlot(cache, bodies, 0u, 1u);

  REQUIRE(slot != talus::dynamics::noSlot);

  cache.replaceManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{
    childRecord(0u, 2u, 1u, 100u, 5u, 0u, 3.0),
    childRecord(0u, 3u, 1u, 101u, 5u, 0u, 4.0)
  });
  cache.rebuildManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{
    childRecord(0u, 3u, 1u, 101u, 5u, 0u, 0.0),
    childRecord(0u, 2u, 1u, 100u, 5u, 0u, 0.0),
    childRecord(0u, 6u, 1u, 104u, 5u, 0u, 0.0)
  });

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);

  REQUIRE(pair.manifolds.size() == 1u);
  REQUIRE(pair.points.size() == 3u);
  CHECK(pair.manifoldPoints[0].firstChild == 3u);
  CHECK(pair.manifoldPoints[0].secondChild == 101u);
  CHECK(pair.points[0].memory.normal == 4.0);
  CHECK(pair.manifoldPoints[1].firstChild == 2u);
  CHECK(pair.points[1].memory.normal == 3.0);
  CHECK(pair.points[2].memory.normal == 0.0);
}

TEST_CASE("a rebuilt manifold keeps its twist when its normal stays within the shared cone, and drops it for a flip or a wider turn", "[dynamics][compound]") {
  const talus::dynamics::BodySet bodies = twoCubes();
  const auto turned = [](talus::dynamics::Contact contact, double degrees) {
    const glm::dmat3 turn = glm::mat3_cast(glm::angleAxis(glm::radians(degrees), glm::dvec3(0.0, 0.0, 1.0)));

    for (glm::dvec3& axis : contact.basis) {
      axis = turn * axis;
    }

    return contact;
  };
  const auto carried = [&](const talus::dynamics::Contact& rebuilt) {
    talus::dynamics::ManifoldCache cache;
    const uint32_t slot = pairSlot(cache, bodies, 0u, 1u);

    REQUIRE(slot != talus::dynamics::noSlot);

    cache.replaceManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{childRecord(0u, 2u, 1u, 100u, 5u, 0u, 3.0)});
    REQUIRE(cache.at(slot).manifolds.size() == 1u);
    cache.at(slot).manifolds[0].twist = 2.5;
    cache.rebuildManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{rebuilt});
    REQUIRE(cache.at(slot).manifolds.size() == 1u);

    return cache.at(slot).manifolds[0].twist;
  };
  const talus::dynamics::Contact flipped = childRecord(1u, 100u, 0u, 2u, 5u, 0u, 0.0);
  talus::dynamics::Contact flippedAndReversed = flipped;

  for (glm::dvec3& axis : flippedAndReversed.basis) {
    axis = -axis;
  }

  CHECK(carried(childRecord(0u, 2u, 1u, 100u, 5u, 0u, 0.0)) == 2.5);
  CHECK(carried(turned(childRecord(0u, 2u, 1u, 100u, 5u, 0u, 0.0), 5.0)) == 2.5);
  CHECK(carried(turned(childRecord(0u, 2u, 1u, 100u, 5u, 0u, 0.0), 10.0)) == 0.0);
  CHECK(carried(flipped) == 0.0);
  CHECK(carried(flippedAndReversed) == 0.0);
}

TEST_CASE("records keep the child keys of their points, flipped or not", "[dynamics][compound]") {
  const talus::dynamics::BodySet bodies = twoCubes();
  talus::dynamics::ManifoldCache cache;
  const uint32_t slot = pairSlot(cache, bodies, 0u, 1u);

  REQUIRE(slot != talus::dynamics::noSlot);

  cache.replaceManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{
    childRecord(0u, 2u, 1u, 100u, 5u, 0u, 3.0),
    childRecord(1u, 105u, 0u, 7u, 6u, 1u, 2.0)
  });

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);

  REQUIRE(pair.manifolds.size() == 2u);
  CHECK(!pair.manifolds[0].flipped);
  CHECK(pair.manifolds[1].flipped);
  CHECK(pair.manifoldPoints[1].firstChild == 7u);
  CHECK(pair.manifoldPoints[1].secondChild == 105u);

  const std::vector<talus::dynamics::Contact> records = cache.records(slot);

  REQUIRE(records.size() == 2u);
  CHECK(records[0].a.child == 2u);
  CHECK(records[0].b.child == 100u);
  CHECK(records[1].a.body == 1u);
  CHECK(records[1].a.child == 105u);
  CHECK(records[1].b.child == 7u);
}

namespace {

constexpr double coulombSlide = 4.0 / (2.0 * 0.5 * 9.81);

struct RowResult {
  double drift = 0.0;
  double peak = 0.0;
  double fastest = 0.0;
  double sink = 0.0;
  double topDrop = 0.0;
  double energyRatio = 1.0;
};

const talus::field::SampledSdf& groundShape() {
  static const talus::field::SampledSdf ground = [] {
    const glm::vec3 centre(0.0f, -0.5f, 0.0f);
    const glm::vec3 half(20.0f, 0.5f, 20.0f);
    const glm::vec3 low = centre - half - glm::vec3(0.2f);
    const glm::vec3 high = centre + half + glm::vec3(0.2f);

    return talus::field::bake(
      std::vector<talus::field::Primitive>{fixture::box(centre, half)},
      low,
      high,
      glm::uvec3(glm::round((high - low) / 0.1f)) + glm::uvec3(1u)
    );
  }();

  return ground;
}

const talus::field::SampledSdf& floorShape() {
  static const talus::field::SampledSdf floor = [] {
    const glm::vec3 low(-1.04f, -0.14f, -1.04f);
    const glm::vec3 high(1.04f, 0.34f, 1.04f);

    return talus::field::bake(
      std::vector<talus::field::Primitive>{
        fixture::box(glm::vec3(0.0f), glm::vec3(1.0f, 0.1f, 1.0f))
      },
      low,
      high,
      glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
    );
  }();

  return floor;
}

RowResult measure(talus::dynamics::World& world, double seconds, double judgedFrom) {
  RowResult result;
  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  const uint32_t frames = static_cast<uint32_t>(std::lround(seconds * 60.0));
  const uint32_t judged = static_cast<uint32_t>(std::lround(judgedFrom * 60.0));
  const size_t count = world.bodies.rigids.size();
  std::vector<glm::dvec3> initial(count);
  std::vector<glm::dvec3> settled(count);
  size_t top = 0u;

  for (size_t index = 0u; index != count; ++index) {
    initial[index] = world.bodies.rigids[index].position;

    if (initial[index].y > initial[top].y) {
      top = index;
    }
  }

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (size_t index = 0u; index != count; ++index) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[index];

      result.fastest = std::max(result.fastest, glm::length(body.velocity));
      result.sink = std::max(result.sink, initial[index].y - body.position.y);

      if (frame + 1u == judged) {
        settled[index] = body.position;
      } else if (frame + 1u > judged) {
        result.drift =
          std::max(result.drift, glm::length(body.position - settled[index]));
        result.peak = std::max(result.peak, glm::length(body.velocity));
      }
    }

    result.energyRatio = std::max(
      result.energyRatio,
      talus::dynamics::energyOf(world.bodies, g).total() / start
    );
  }

  result.topDrop = initial[top].y - world.bodies.rigids[top].position.y;

  return result;
}

void restRule(const RowResult& result) {
  CHECK(result.drift < 0.01);
  CHECK(result.peak < 0.05);
  CHECK(result.energyRatio <= 1.02);
}

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

bool sameState(const talus::dynamics::World& first, const talus::dynamics::World& second) {
  if (first.bodies.rigids.size() != second.bodies.rigids.size()) {
    return false;
  }

  for (size_t index = 0u; index != first.bodies.rigids.size(); ++index) {
    const talus::dynamics::RigidBody& a = first.bodies.rigids[index];
    const talus::dynamics::RigidBody& b = second.bodies.rigids[index];

    if (a.position != b.position || a.velocity != b.velocity
        || a.angularVelocity != b.angularVelocity || a.rotation != b.rotation) {
      return false;
    }
  }

  return true;
}

uint32_t addOneCube(
  talus::dynamics::World& world,
  double half,
  double density,
  const glm::dvec3& position,
  const glm::dvec3& velocity
) {
  const std::vector<talus::dynamics::CompoundChild> one{part(glm::dvec3(0.0), glm::dvec3(half), density, 0u)};

  return talus::dynamics::addCompoundBody(world.bodies, one, position, noTurn, velocity, 0.5, 0.0, false);
}

uint32_t addEightCube(
  talus::dynamics::World& world,
  double half,
  double density,
  const glm::dvec3& position,
  const glm::dvec3& velocity
) {
  return talus::dynamics::addCompoundBody(
    world.bodies, cubeOfEight(0.5 * half, density, 0u), position, noTurn, velocity, 0.5, 0.0, false
  );
}

}

TEST_CASE("a compound on a static floor gets one reduced manifold on its touching children", "[dynamics][compound]") {
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floorShape(), 0.5);
  talus::dynamics::addCompoundBody(bodies, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0, 0.1505, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  REQUIRE(cache.size() == 1u);

  const talus::dynamics::CachedBodyPair& pair = cache.at(0u);
  std::vector<uint32_t> touched;

  REQUIRE(pair.manifolds.size() == 1u);
  REQUIRE(pair.points.size() == 4u);

  for (const talus::dynamics::CachedManifoldPoint& point : pair.manifoldPoints) {
    CHECK(point.secondChild == talus::dynamics::noChild);
    touched.push_back(point.firstChild);
  }

  std::sort(touched.begin(), touched.end());
  CHECK(touched == std::vector<uint32_t>{0u, 1u, 4u, 5u});
}

TEST_CASE("compound row: a one-child compound rests as the resting box", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addOneCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.05, 0.0), glm::dvec3(0.0));

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow rest-one | drift %.6f peak %.6f sink %.6f energy %.4f\n", result.drift, result.peak, result.sink, result.energyRatio);
  restRule(result);
}

TEST_CASE("compound row: a one-child 1 m compound rests bit for bit like the 1 m box", "[.][compoundrows]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.5));
  talus::dynamics::World boxes;
  talus::dynamics::World compounds;

  talus::dynamics::addStatic(boxes.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addStatic(compounds.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addRigidBox(boxes.bodies, crate, glm::dvec3(0.5), 1000.0, glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.0), 0.5, 3u);
  addOneCube(compounds, 0.5, 1000.0, glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.0));

  REQUIRE(compounds.bodies.rigids[0].mass == boxes.bodies.rigids[0].mass);
  REQUIRE(compounds.bodies.rigids[0].moment == boxes.bodies.rigids[0].moment);

  uint32_t diverged = 360u;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(boxes).nonFinite);
    REQUIRE(!talus::dynamics::step(compounds).nonFinite);

    if (diverged == 360u && !sameState(boxes, compounds)) {
      diverged = frame;
    }
  }

  std::printf("compoundrow rest-one-1m | first differing frame %u of 360 | hashes %016llx %016llx\n", diverged, static_cast<unsigned long long>(stateHash(boxes)), static_cast<unsigned long long>(stateHash(compounds)));
  CHECK(diverged == 360u);
}

TEST_CASE("compound row: a one-child compound slides as the sliding box", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addOneCube(world, 0.05, 1500.0, glm::dvec3(-0.3, 0.05, 0.0), glm::dvec3(2.0, 0.0, 0.0));

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
  const double distance = box.position.x + 0.3;

  std::printf("compoundrow slide-one | distance %.5f final speed %.6f | coulomb %.5f\n", distance, glm::length(box.velocity), coulombSlide);
  CHECK(distance >= 0.97 * coulombSlide);
  CHECK(distance <= 1.03 * coulombSlide);
  CHECK(glm::length(box.velocity) < 0.05);
}

TEST_CASE("compound row: an eight-child box rests on the ground", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.05, 0.0), glm::dvec3(0.0));

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow rest-eight | drift %.6f peak %.6f sink %.6f energy %.4f\n", result.drift, result.peak, result.sink, result.energyRatio);
  restRule(result);
}

TEST_CASE("compound row: an eight-child box slides as Coulomb's law says", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addEightCube(world, 0.05, 1500.0, glm::dvec3(-0.3, 0.05, 0.0), glm::dvec3(2.0, 0.0, 0.0));

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
  const double distance = box.position.x + 0.3;

  std::printf("compoundrow slide-eight | distance %.5f final speed %.6f | coulomb %.5f\n", distance, glm::length(box.velocity), coulombSlide);
  CHECK(distance >= 0.97 * coulombSlide);
  CHECK(distance <= 1.03 * coulombSlide);
  CHECK(glm::length(box.velocity) < 0.05);
}

namespace {

talus::field::SampledSdf sphereShape(double radius) {
  const double voxel = radius / 5.0;
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

uint32_t addSlab(
  talus::dynamics::World& world,
  uint32_t across,
  uint32_t deep,
  const glm::dvec3& half,
  const glm::dvec3& position
) {
  std::vector<talus::dynamics::CompoundChild> children;

  for (uint32_t z = 0u; z != deep; ++z) {
    for (uint32_t x = 0u; x != across; ++x) {
      const glm::dvec3 centre(
        (static_cast<double>(x) - 0.5 * static_cast<double>(across - 1u)) * 2.0 * half.x,
        0.0,
        (static_cast<double>(z) - 0.5 * static_cast<double>(deep - 1u)) * 2.0 * half.z
      );

      children.push_back(part(centre, half, 1000.0, x + across * z));
    }
  }

  return talus::dynamics::addCompoundBody(world.bodies, children, position, noTurn, glm::dvec3(0.0), 0.5, 0.0, true);
}

std::vector<talus::dynamics::CompoundChild> lowerL() {
  return {
    part(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 10u),
    part(glm::dvec3(0.15, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 11u),
    part(glm::dvec3(0.3, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 12u),
    part(glm::dvec3(0.0, 0.15, 0.0), glm::dvec3(0.05), 1000.0, 13u)
  };
}

std::vector<talus::dynamics::CompoundChild> upperL() {
  return {
    part(glm::dvec3(0.15, 0.1, 0.0), glm::dvec3(0.05), 1000.0, 20u),
    part(glm::dvec3(0.3, 0.1, 0.0), glm::dvec3(0.05), 1000.0, 21u),
    part(glm::dvec3(0.3, 0.25, 0.0), glm::dvec3(0.05), 1000.0, 22u)
  };
}

}

TEST_CASE("two L-shaped compounds touch only through their touching children", "[dynamics][compound]") {
  talus::dynamics::BodySet bodies;

  talus::dynamics::addCompoundBody(bodies, lowerL(), glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  talus::dynamics::addCompoundBody(bodies, upperL(), glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<double> still(2u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  const uint32_t slot = cache.find(talus::dynamics::pairKey(
    bodies,
    {talus::dynamics::Side::Rigid, 0u},
    {talus::dynamics::Side::Rigid, 1u}
  ));

  REQUIRE(slot != talus::dynamics::noSlot);

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);
  std::vector<std::pair<uint32_t, uint32_t>> touched;

  REQUIRE(pair.manifolds.size() == 1u);
  REQUIRE(!pair.points.empty());

  for (const talus::dynamics::CachedManifoldPoint& point : pair.manifoldPoints) {
    touched.emplace_back(point.firstChild, point.secondChild);
  }

  std::sort(touched.begin(), touched.end());
  touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
  CHECK(touched == std::vector<std::pair<uint32_t, uint32_t>>{{11u, 20u}, {12u, 21u}});
}

TEST_CASE("a box and a ball on a compound touch only the child under them", "[dynamics][compound]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.025));
  const talus::field::SampledSdf ball = sphereShape(0.03);
  talus::dynamics::World world;

  (void)addSlab(world, 3u, 3u, glm::dvec3(0.05), glm::dvec3(0.0));
  talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.025), 0.1, glm::dvec3(-0.1, 0.0755, -0.1), glm::dvec3(0.0), 0.5, 3u);
  talus::dynamics::addRigidSphere(world.bodies, ball, 0.03, 0.1, glm::dvec3(0.1, 0.0805, 0.1), glm::dvec3(0.0), 0.5, 0.0);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(world.bodies, 0.02);
  const std::vector<double> still(3u, 0.0);

  world.manifolds.sync(world.bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0);

  const talus::dynamics::BodyRef slab{talus::dynamics::Side::Rigid, 0u};
  const uint32_t onBox = world.manifolds.find(
    talus::dynamics::pairKey(world.bodies, slab, {talus::dynamics::Side::Rigid, 1u})
  );
  const uint32_t onBall = world.manifolds.find(
    talus::dynamics::pairKey(world.bodies, slab, {talus::dynamics::Side::Rigid, 2u})
  );

  REQUIRE(onBox != talus::dynamics::noSlot);
  REQUIRE(onBall != talus::dynamics::noSlot);
  REQUIRE(world.manifolds.at(onBox).manifolds.size() == 1u);
  REQUIRE(!world.manifolds.at(onBox).points.empty());

  for (const talus::dynamics::CachedManifoldPoint& point : world.manifolds.at(onBox).manifoldPoints) {
    CHECK(point.firstChild == 0u);
    CHECK(point.secondChild == talus::dynamics::noChild);
  }

  REQUIRE(world.manifolds.at(onBall).points.size() == 1u);
  CHECK(world.manifolds.at(onBall).manifoldPoints[0].firstChild == 8u);
}

TEST_CASE("a compound on a sampled body gets one reduced manifold on its touching children", "[dynamics][compound]") {
  const talus::field::SampledSdf slabField = boxShape(glm::dvec3(0.3, 0.05, 0.3));
  talus::dynamics::BodySet bodies;
  const uint32_t slab = talus::dynamics::addRigidBox(
    bodies, slabField, glm::dvec3(0.3, 0.05, 0.3), 0.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.5, 3u
  );

  bodies.rigids[slab].shape = talus::dynamics::RigidShape::Sampled;
  talus::dynamics::addCompoundBody(bodies, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0, 0.1005, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<double> still(2u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  REQUIRE(cache.size() == 1u);

  const talus::dynamics::CachedBodyPair& pair = cache.at(0u);
  std::vector<uint32_t> touched;

  REQUIRE(pair.manifolds.size() == 1u);
  REQUIRE(pair.points.size() == 4u);
  CHECK(pair.manifolds[0].flipped);
  CHECK(pair.pose.valid);

  for (const talus::dynamics::CachedManifoldPoint& point : pair.manifoldPoints) {
    CHECK(point.firstChild == talus::dynamics::noChild);
    touched.push_back(point.secondChild);
  }

  std::sort(touched.begin(), touched.end());
  CHECK(touched == std::vector<uint32_t>{0u, 1u, 4u, 5u});
}

TEST_CASE("a flipped manifold with a stored pose anchors each point on its own body", "[dynamics][compound]") {
  talus::dynamics::BodySet bodies = twoCubes();
  talus::dynamics::ManifoldCache cache;

  bodies.rigids[0].rotation = glm::angleAxis(0.4, glm::dvec3(0.0, 0.0, 1.0));
  bodies.rigids[1].rotation = glm::angleAxis(-0.9, glm::dvec3(1.0, 0.0, 0.0));

  const uint32_t slot = pairSlot(cache, bodies, 0u, 1u);
  talus::dynamics::Contact record = childRecord(1u, 105u, 0u, 7u, 9u, 0u, 0.0);

  REQUIRE(slot != talus::dynamics::noSlot);

  record.gap = 0.004;
  record.cached.valid = true;
  record.cached.rotationA = bodies.rigids[0].rotation;
  record.cached.rotationB = bodies.rigids[1].rotation;
  record.cached.separation = 0.004;
  cache.replaceManifolds(bodies, slot, std::vector<talus::dynamics::Contact>{record});

  const talus::dynamics::CachedBodyPair& pair = cache.at(slot);
  const glm::dvec3 shift = 0.5 * 0.004 * record.basis[0];

  REQUIRE(pair.manifolds.size() == 1u);
  CHECK(pair.manifolds[0].flipped);
  CHECK(pair.points[0].anchorB == glm::vec3(glm::mat3_cast(bodies.rigids[1].rotation) * record.a.local - shift));
  CHECK(pair.points[0].anchorA == glm::vec3(glm::mat3_cast(bodies.rigids[0].rotation) * record.b.local + shift));
}

TEST_CASE("compound row: ten one-child compounds stack as the ten-high box stack", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t level = 0u; level != 10u; ++level) {
    addOneCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.05 * (1.0 + 2.0 * level), 0.0), glm::dvec3(0.0));
  }

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow stack-one | drift %.6f peak %.6f top sink %.6f energy %.4f\n", result.drift, result.peak, result.topDrop, result.energyRatio);
  restRule(result);
  CHECK(result.topDrop < 0.1);
}

TEST_CASE("compound row: ten one-child 1 m compounds stack bit for bit like the 1 m boxes", "[.][compoundrows]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.5));
  talus::dynamics::World boxes;
  talus::dynamics::World compounds;

  talus::dynamics::addStatic(boxes.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addStatic(compounds.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t level = 0u; level != 10u; ++level) {
    const glm::dvec3 place(0.0, 0.5 * (1.0 + 2.0 * level), 0.0);

    talus::dynamics::addRigidBox(boxes.bodies, crate, glm::dvec3(0.5), 1000.0, place, glm::dvec3(0.0), 0.5, 3u);
    addOneCube(compounds, 0.5, 1000.0, place, glm::dvec3(0.0));
  }

  uint32_t diverged = 360u;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(boxes).nonFinite);
    REQUIRE(!talus::dynamics::step(compounds).nonFinite);

    if (diverged == 360u && !sameState(boxes, compounds)) {
      diverged = frame;
    }
  }

  std::printf("compoundrow stack-one-1m | first differing frame %u of 360 | hashes %016llx %016llx\n", diverged, static_cast<unsigned long long>(stateHash(boxes)), static_cast<unsigned long long>(stateHash(compounds)));
  CHECK(diverged == 360u);
}

TEST_CASE("compound row: ten eight-child boxes stack", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);

  for (uint32_t level = 0u; level != 10u; ++level) {
    addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.05 * (1.0 + 2.0 * level), 0.0), glm::dvec3(0.0));
  }

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow stack-eight | drift %.6f peak %.6f top sink %.6f energy %.4f\n", result.drift, result.peak, result.topDrop, result.energyRatio);
  restRule(result);
  CHECK(result.topDrop < 0.1);
}

TEST_CASE("compound row: two L-shaped compounds rest on each other", "[.][compoundrows]") {
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  talus::dynamics::addCompoundBody(world.bodies, lowerL(), glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  talus::dynamics::addCompoundBody(world.bodies, upperL(), glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  const RowResult result = measure(world, 6.0, 1.0);
  const uint32_t slot = world.manifolds.find(talus::dynamics::pairKey(
    world.bodies,
    {talus::dynamics::Side::Rigid, 0u},
    {talus::dynamics::Side::Rigid, 1u}
  ));

  REQUIRE(slot != talus::dynamics::noSlot);

  std::vector<std::pair<uint32_t, uint32_t>> touched;

  for (const talus::dynamics::CachedManifoldPoint& point : world.manifolds.at(slot).manifoldPoints) {
    touched.emplace_back(point.firstChild, point.secondChild);
  }

  std::sort(touched.begin(), touched.end());
  touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
  std::printf("compoundrow l-shapes | drift %.6f peak %.6f sink %.6f energy %.4f touching child pairs %zu\n", result.drift, result.peak, result.sink, result.energyRatio, touched.size());
  restRule(result);
  CHECK(touched == std::vector<std::pair<uint32_t, uint32_t>>{{11u, 20u}, {12u, 21u}});
}

TEST_CASE("compound row: a ball rests on an eight-child box", "[.][compoundrows]") {
  const talus::field::SampledSdf ball = sphereShape(0.05);
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, groundShape(), 0.5, 0.0);
  addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.05, 0.0), glm::dvec3(0.0));
  talus::dynamics::addRigidSphere(world.bodies, ball, 0.05, 1.0, glm::dvec3(0.012, 0.1505, 0.012), glm::dvec3(0.0), 0.5, 0.0);

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow ball-on-eight | drift %.6f peak %.6f sink %.6f energy %.4f\n", result.drift, result.peak, result.sink, result.energyRatio);
  restRule(result);
  CHECK(result.sink < 0.05);
}

TEST_CASE("compound row: an eight-child box rests on a sampled slab", "[.][compoundrows]") {
  const talus::field::SampledSdf slabField = boxShape(glm::dvec3(0.3, 0.05, 0.3));
  talus::dynamics::World world;
  const uint32_t slab = talus::dynamics::addRigidBox(
    world.bodies, slabField, glm::dvec3(0.3, 0.05, 0.3), 0.0, glm::dvec3(0.0, 0.05, 0.0), glm::dvec3(0.0), 0.5, 3u
  );

  world.bodies.rigids[slab].shape = talus::dynamics::RigidShape::Sampled;
  addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.1505, 0.0), glm::dvec3(0.0));

  const RowResult result = measure(world, 6.0, 1.0);

  std::printf("compoundrow eight-on-sampled | drift %.6f peak %.6f sink %.6f energy %.4f\n", result.drift, result.peak, result.sink, result.energyRatio);
  restRule(result);
}

TEST_CASE("promoting a switchable body that rests on a compound reads no missing field", "[dynamics][compound]") {
  const std::vector<talus::PhysicalMaterial> lanes{rubber()};
  talus::field::SampledSdf blockField;
  const auto mesh = latticeBlock(glm::dvec3(0.05, 0.025, 0.05), 0.025, lanes, blockField);
  talus::dynamics::World world;

  (void)addSlab(world, 3u, 3u, glm::dvec3(0.05), glm::dvec3(0.0));

  const uint32_t id = talus::dynamics::addSwitchable(
    world.bodies, mesh, lanes, blockField, glm::dvec3(0.0, 0.0755, 0.0), glm::dvec3(0.0), 0.5, 0.0
  );

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  talus::dynamics::promote(world, id);

  const std::optional<talus::dynamics::BodyRef> soft = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(soft.has_value());
  CHECK(soft->side == talus::dynamics::Side::Lattice);
}

namespace {

talus::field::SampledSdf plateShape() {
  const glm::vec3 low(-2.1f, -0.2f, -2.1f);
  const glm::vec3 high(2.1f, 0.4f, 2.1f);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{
      fixture::box(glm::vec3(0.0f, -0.02f, 0.0f), glm::vec3(2.0f, 0.02f, 2.0f))
    },
    low,
    high,
    glm::uvec3(glm::round((high - low) / 0.02f)) + glm::uvec3(1u)
  );
}

}

TEST_CASE("a thin compound tile thrown at the floor does not pass through it", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> tile{part(glm::dvec3(0.0), glm::dvec3(0.1, 0.003, 0.1), 2500.0, 0u)};
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);
  talus::dynamics::addCompoundBody(world.bodies, tile, glm::dvec3(0.0, 0.3, 0.0), noTurn, glm::dvec3(0.0, -15.0, 0.0), 0.5, 0.0, false);

  double lowest = 1.0;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    lowest = std::min(lowest, world.bodies.rigids[0].position.y);
  }

  CHECK(lowest >= 0.09);
}

TEST_CASE("compound row: a fast eight-child box onto a thin plate", "[.][compoundrows]") {
  const talus::field::SampledSdf plate = plateShape();
  talus::dynamics::World world;

  talus::dynamics::addStatic(world.bodies, plate, 0.5, 0.0);
  addEightCube(world, 0.05, 1500.0, glm::dvec3(0.0, 0.2, 0.0), glm::dvec3(0.0, -20.0, 0.0));

  double lowestTop = 1.0;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& box = world.bodies.rigids[0];
    const glm::dmat3 turn = glm::mat3_cast(box.rotation);
    const double reach = 0.05 * (std::abs(turn[0][1]) + std::abs(turn[1][1]) + std::abs(turn[2][1]));

    lowestTop = std::min(lowestTop, box.position.y + reach);
  }

  std::printf("compoundrow plate-eight | final y %.5f lowest top %.5f\n", world.bodies.rigids[0].position.y, lowestTop);
  CHECK(lowestTop >= -0.04);
}

TEST_CASE("a box slides across the seams of a compound floor without snagging", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> one{part(glm::dvec3(0.0), glm::dvec3(0.04), 1000.0, 0u)};
  talus::dynamics::World world;

  (void)addSlab(world, 12u, 1u, glm::dvec3(0.05), glm::dvec3(0.0, -0.05, 0.0));

  const uint32_t slider = talus::dynamics::addCompoundBody(
    world.bodies, one, glm::dvec3(-0.45, 0.04, 0.0), noTurn, glm::dvec3(2.0, 0.0, 0.0), 0.0, 0.0, false
  );
  double slowest = 2.0;
  double bounce = 0.0;

  for (uint32_t frame = 0u; frame != 18u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& body = world.bodies.rigids[slider];

    slowest = std::min(slowest, body.velocity.x);
    bounce = std::max(bounce, std::abs(body.velocity.y));
  }

  CHECK(slowest >= 1.98);
  CHECK(bounce <= 0.05);
}

TEST_CASE("a compound shape marks the faces its children cover", "[dynamics][compound]") {
  const std::shared_ptr<const talus::dynamics::CompoundShape> shape = talus::dynamics::makeCompoundShape({
    part(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 0u),
    part(glm::dvec3(0.1, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 1u),
    part(glm::dvec3(0.2, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 2u),
    part(glm::dvec3(0.1, 0.1, 0.0), glm::dvec3(0.05), 1000.0, 3u),
    part(glm::dvec3(0.301, 0.0, 0.0), glm::dvec3(0.05), 1000.0, 4u),
    part(glm::dvec3(0.0, 0.0, 0.125), glm::dvec3(0.1, 0.05, 0.075), 1000.0, 5u)
  });

  REQUIRE(shape->exposed.size() == 6u);
  CHECK(shape->exposed[0] == 0x1Du);
  CHECK(shape->exposed[1] == 0x34u);
  CHECK(shape->exposed[2] == 0x3Eu);
  CHECK(shape->exposed[3] == 0x3Bu);
  CHECK(shape->exposed[4] == talus::dynamics::allFaces);
  CHECK(shape->exposed[5] == talus::dynamics::allFaces);
}

TEST_CASE("a box thrown at a compound wall still stops at its outer face", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> one{part(glm::dvec3(0.0), glm::dvec3(0.04), 1000.0, 0u)};
  std::vector<talus::dynamics::CompoundChild> wall;
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  for (uint32_t level = 0u; level != 3u; ++level) {
    for (uint32_t row = 0u; row != 3u; ++row) {
      wall.push_back(part(glm::dvec3(0.0, 0.1 * level - 0.1, 0.1 * row - 0.1), glm::dvec3(0.05), 1000.0, 3u * level + row));
    }
  }

  talus::dynamics::addCompoundBody(world.bodies, wall, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t thrown = talus::dynamics::addCompoundBody(
    world.bodies, one, glm::dvec3(0.3, 0.0, 0.0), noTurn, glm::dvec3(-2.0, 0.0, 0.0), 0.5, 0.0, false
  );
  double nearest = 1.0;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    nearest = std::min(nearest, world.bodies.rigids[thrown].position.x - 0.04);
  }

  CHECK(nearest >= 0.05 - 0.01);
}

TEST_CASE("bodies resting on a zero-mass compound keep islands of their own", "[dynamics][compound]") {
  talus::dynamics::World world;

  (void)addSlab(world, 3u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  (void)addEightCube(world, 0.05, 1000.0, glm::dvec3(-0.2, 0.0505, 0.0), glm::dvec3(0.0));
  (void)addEightCube(world, 0.05, 1000.0, glm::dvec3(0.2, 0.0505, 0.0), glm::dvec3(0.0));

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(world.bodies, 0.02);
  const std::vector<double> still(3u, 0.0);

  world.manifolds.sync(world.bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0);

  const std::vector<talus::dynamics::Island> islands =
    talus::dynamics::islandsOf(world.bodies, world.manifolds, {});

  REQUIRE(islands.size() == 3u);

  for (const talus::dynamics::Island& island : islands) {
    CHECK(island.bodies.size() == 1u);
    CHECK(island.external);
  }
}

TEST_CASE("debris resting on a zero-mass compound falls asleep while another piece on it still moves", "[dynamics][compound]") {
  const talus::field::SampledSdf ball = sphereShape(0.05);
  talus::dynamics::World world;

  (void)addSlab(world, 4u, 4u, glm::dvec3(0.25, 0.05, 0.25), glm::dvec3(0.0, -0.05, 0.0));

  const uint32_t crate = addEightCube(world, 0.05, 1000.0, glm::dvec3(-0.75, 0.0505, -0.75), glm::dvec3(0.0));
  const uint32_t roller = talus::dynamics::addRigidSphere(
    world.bodies, ball, 0.05, 1.0, glm::dvec3(0.6, 0.0505, 0.3), glm::dvec3(-0.4, 0.0, 0.0), 0.5, 0.0
  );
  double lowest = 1.0;

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    lowest = std::min(lowest, world.bodies.rigids[roller].position.y);
  }

  CHECK(world.bodies.rigids[crate].asleep);
  CHECK(!world.bodies.rigids[roller].asleep);
  CHECK(glm::length(world.bodies.rigids[roller].velocity) > 0.05);
  CHECK(lowest >= 0.04);
}

TEST_CASE("a crate dropped onto a zero-mass compound that has fallen asleep lands on it", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 3u, 3u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(world.bodies.rigids[slab].asleep);

  const uint32_t crate = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.25, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(std::abs(world.bodies.rigids[crate].position.y - 0.05) <= 0.01);
}

TEST_CASE("an anchored compound on a static floor gets no contact points", "[dynamics][compound]") {
  talus::dynamics::BodySet bodies;

  talus::dynamics::addStatic(bodies, floorShape(), 0.5);
  talus::dynamics::addCompoundBody(bodies, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0, 0.1505, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const std::vector<talus::dynamics::BodyPair> pairs = talus::dynamics::broadphase(bodies, 0.02);
  const std::vector<double> still(1u, 0.0);
  talus::dynamics::ManifoldCache cache;

  cache.sync(bodies, pairs);
  (void)talus::dynamics::updateCachedPairs(bodies, cache, 0.02, still, 0.0);

  REQUIRE(cache.size() == 1u);
  CHECK(cache.at(0u).points.empty());
}

TEST_CASE("a glass pane under a steel block on a sloped zero-mass ledge does not turn under it", "[dynamics][compound]") {
  talus::dynamics::World world;
  const glm::dquat tilt = glm::angleAxis(25.0 * std::numbers::pi / 180.0, glm::dvec3(0.0, 0.0, 1.0));
  const glm::dmat3 slope = glm::mat3_cast(tilt);
  const glm::dvec3 normal = slope * glm::dvec3(0.0, 1.0, 0.0);
  const glm::dvec3 ledgeHalf(0.6, 0.05, 0.15);
  const glm::dvec3 paneHalf(0.35, 0.005, 0.3);
  const glm::dvec3 blockHalf(0.25, 0.25, 0.2);
  const std::vector<talus::dynamics::CompoundChild> ledge{part(glm::dvec3(0.0), ledgeHalf, 2450.0, 0u)};
  const std::vector<talus::dynamics::CompoundChild> pane{part(glm::dvec3(0.0), paneHalf, 2500.0, 1u)};
  const std::vector<talus::dynamics::CompoundChild> block{part(glm::dvec3(0.0), blockHalf, 7850.0, 2u)};

  (void)talus::dynamics::addCompoundBody(world.bodies, ledge, glm::dvec3(0.0), tilt, glm::dvec3(0.0), 0.6, 0.0, true);

  const uint32_t glass = talus::dynamics::addCompoundBody(
    world.bodies, pane, slope * glm::dvec3(0.0, ledgeHalf.y + paneHalf.y, 0.0), tilt, glm::dvec3(0.0), 0.6, 0.0, false
  );
  const uint32_t steel = talus::dynamics::addCompoundBody(
    world.bodies,
    block,
    slope * glm::dvec3(0.0, ledgeHalf.y + 2.0 * paneHalf.y + blockHalf.y, 0.08),
    tilt,
    glm::dvec3(0.0),
    0.6,
    0.0,
    false
  );
  const double reach = world.bodies.rigids[glass].compound->reach;
  const glm::dquat placed = world.bodies.rigids[glass].rotation;
  double turned = 0.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const glm::dquat turn = world.bodies.rigids[glass].rotation * glm::conjugate(placed);
    const double twist = 2.0 * std::asin(std::clamp(glm::dot(glm::dvec3(turn.x, turn.y, turn.z), normal), -1.0, 1.0));

    turned = std::max(turned, std::abs(twist) * reach);
  }

  CHECK(turned < 0.01);
  CHECK(world.bodies.rigids[glass].asleep);
  CHECK(world.bodies.rigids[steel].asleep);
}

TEST_CASE("a box on a wide zero-mass compound keeps its contact manifold through a small turn, as on a static floor", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> box{part(glm::dvec3(0.0), glm::dvec3(0.05), 1000.0, 0u)};
  const glm::dquat turned = glm::angleAxis(0.05, glm::dvec3(0.0, 1.0, 0.0));
  talus::dynamics::BodySet onFloor;
  talus::dynamics::BodySet onSlab;
  std::vector<talus::dynamics::CompoundChild> slab;

  for (uint32_t z = 0u; z != 3u; ++z) {
    for (uint32_t x = 0u; x != 3u; ++x) {
      slab.push_back(part(glm::dvec3(static_cast<double>(x) - 1.0, 0.0, static_cast<double>(z) - 1.0), glm::dvec3(0.5, 0.1, 0.5), 1000.0, x + 3u * z));
    }
  }

  talus::dynamics::addStatic(onFloor, floorShape(), 0.5);
  (void)talus::dynamics::addCompoundBody(onFloor, box, glm::dvec3(0.0, 0.15, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  (void)talus::dynamics::addCompoundBody(onSlab, slab, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);
  (void)talus::dynamics::addCompoundBody(onSlab, box, glm::dvec3(0.0, 0.15, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  for (talus::dynamics::BodySet* bodies : {&onFloor, &onSlab}) {
    talus::dynamics::RigidBody& resting = bodies->rigids.back();
    const std::vector<double> still(bodies->rigids.size(), 0.0);
    talus::dynamics::ManifoldCache cache;

    cache.sync(*bodies, talus::dynamics::broadphase(*bodies, 0.02));
    (void)talus::dynamics::updateCachedPairs(*bodies, cache, 0.02, still, 0.0);

    REQUIRE(cache.size() == 1u);
    REQUIRE(cache.at(0u).points.size() == 4u);

    const talus::dynamics::CachedPose built = cache.at(0u).pose;

    resting.rotation = turned;
    (void)talus::dynamics::updateCachedPairs(*bodies, cache, 0.02, still, 0.0);

    CHECK(cache.at(0u).pose.rotationA == built.rotationA);
    CHECK(cache.at(0u).pose.rotationB == built.rotationB);
    CHECK(cache.at(0u).points.size() == 4u);
  }
}

TEST_CASE("removing children keeps every other contact point's warm start and wakes only the bodies they held", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 6u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  const uint32_t dropped = addEightCube(world, 0.05, 1000.0, glm::dvec3(-0.3, 0.0505, 0.0), glm::dvec3(0.0));
  const uint32_t kept = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.3, 0.0505, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(world.bodies.rigids[dropped].asleep);
  REQUIRE(world.bodies.rigids[kept].asleep);

  const talus::dynamics::BodyRef slabRef{talus::dynamics::Side::Rigid, slab};
  const uint32_t heldSlot = world.manifolds.find(
    talus::dynamics::pairKey(world.bodies, slabRef, {talus::dynamics::Side::Rigid, kept})
  );
  const uint32_t lostSlot = world.manifolds.find(
    talus::dynamics::pairKey(world.bodies, slabRef, {talus::dynamics::Side::Rigid, dropped})
  );

  REQUIRE(heldSlot != talus::dynamics::noSlot);
  REQUIRE(lostSlot != talus::dynamics::noSlot);

  const std::vector<talus::dynamics::CachedContactPoint> before = world.manifolds.at(heldSlot).points;
  const glm::dvec3 restingAt = world.bodies.rigids[kept].position;
  const double droppedAt = world.bodies.rigids[dropped].position.y;

  REQUIRE(!before.empty());
  REQUIRE(talus::dynamics::removeCompoundChildren(world, world.bodies.rigids[slab].id, std::vector<uint32_t>{1u}));
  CHECK(!world.bodies.rigids[dropped].asleep);
  CHECK(world.bodies.rigids[kept].asleep);
  REQUIRE(world.manifolds.at(heldSlot).points.size() == before.size());

  for (size_t index = 0u; index != before.size(); ++index) {
    CHECK(world.manifolds.at(heldSlot).points[index].memory.normal == before[index].memory.normal);
    CHECK(world.manifolds.at(heldSlot).points[index].memory.friction == before[index].memory.friction);
  }

  for (const talus::dynamics::CachedManifoldPoint& point : world.manifolds.at(lostSlot).manifoldPoints) {
    CHECK(point.firstChild != 1u);
  }

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(world.bodies.rigids[dropped].position.y < droppedAt - 0.05);
  CHECK(world.bodies.rigids[kept].position == restingAt);
}

TEST_CASE("removing the child under a stack of two crates drops both crates", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 3u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  const uint32_t lower = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.0505, 0.0), glm::dvec3(0.0));
  const uint32_t upper = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.1515, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(world.bodies.rigids[lower].asleep);
  REQUIRE(world.bodies.rigids[upper].asleep);

  const double lowerAt = world.bodies.rigids[lower].position.y;
  const double upperAt = world.bodies.rigids[upper].position.y;

  REQUIRE(talus::dynamics::removeCompoundChildren(world, world.bodies.rigids[slab].id, std::vector<uint32_t>{1u}));

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(world.bodies.rigids[lower].position.y < lowerAt - 0.05);
  CHECK(world.bodies.rigids[upper].position.y < upperAt - 0.05);
}

TEST_CASE("removing a child under one edge of a crate leaves it resting on the children still under it", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 3u, 1u, glm::dvec3(0.05, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  const uint32_t crate = addEightCube(world, 0.1, 1000.0, glm::dvec3(0.0, 0.1005, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(world.bodies.rigids[crate].asleep);

  const glm::dvec3 restingAt = world.bodies.rigids[crate].position;
  double fastest = 0.0;
  double drift = 0.0;

  REQUIRE(talus::dynamics::removeCompoundChildren(world, world.bodies.rigids[slab].id, std::vector<uint32_t>{0u}));
  CHECK(!world.bodies.rigids[crate].asleep);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    fastest = std::max(fastest, glm::length(world.bodies.rigids[crate].velocity));
    drift = std::max(drift, glm::length(world.bodies.rigids[crate].position - restingAt));
  }

  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
}

TEST_CASE("removing children from a moving compound re-centres it and keeps every remaining child's motion", "[dynamics][compound]") {
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    cubeOfEight(0.05, 1000.0, 0u),
    glm::dvec3(0.2, 0.5, -0.1),
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, -1.0, 0.5))),
    glm::dvec3(0.5, -0.2, 0.1),
    0.5,
    0.0,
    false
  );

  world.bodies.rigids[index].angularVelocity = glm::dvec3(0.4, 0.9, -0.3);

  const talus::dynamics::RigidBody start = world.bodies.rigids[index];

  REQUIRE(talus::dynamics::removeCompoundChildren(world, start.id, std::vector<uint32_t>{2u, 3u, 6u, 7u}));

  const talus::dynamics::RigidBody& after = world.bodies.rigids[index];

  CHECK(std::abs(after.mass - 0.5 * start.mass) <= 1.0e-12 * start.mass);
  CHECK(after.angularVelocity == start.angularVelocity);
  REQUIRE(after.compound->children.size() == 4u);

  for (const talus::dynamics::CompoundChild& child : after.compound->children) {
    const std::optional<uint32_t> was = talus::dynamics::findChild(*start.compound, child.key);

    REQUIRE(was.has_value());

    const talus::dynamics::CompoundChild& old = start.compound->children[*was];
    const std::array<glm::dvec3, 8> then = cornersOf(start.position, start.rotation, old);
    const std::array<glm::dvec3, 8> now = cornersOf(after.position, after.rotation, child);
    const glm::dvec3 middleThen = start.position + glm::mat3_cast(start.rotation) * old.centre;
    const glm::dvec3 middleNow = after.position + glm::mat3_cast(after.rotation) * child.centre;
    const glm::dvec3 speedThen =
      start.velocity + glm::cross(start.angularVelocity, middleThen - start.position);
    const glm::dvec3 speedNow =
      after.velocity + glm::cross(after.angularVelocity, middleNow - after.position);

    for (size_t corner = 0u; corner != then.size(); ++corner) {
      CHECK(glm::length(now[corner] - then[corner]) <= 1.0e-12);
    }

    CHECK(glm::length(speedNow - speedThen) <= 1.0e-12);
  }
}

TEST_CASE("a removal that names an unknown child, repeats one or removes them all is refused", "[dynamics][compound]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;
  const uint32_t index = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.0));
  const uint32_t box = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(1.0, 0.5, 0.0), glm::dvec3(0.0), 0.5, 3u);
  const talus::dynamics::RigidBody start = world.bodies.rigids[index];

  CHECK(!talus::dynamics::removeCompoundChildren(world, start.id, std::vector<uint32_t>{9u}));
  CHECK(!talus::dynamics::removeCompoundChildren(world, start.id, std::vector<uint32_t>{1u, 1u}));
  CHECK(!talus::dynamics::removeCompoundChildren(world, start.id, std::vector<uint32_t>{0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u}));
  CHECK(!talus::dynamics::removeCompoundChildren(world, start.id, std::vector<uint32_t>{}));
  CHECK(!talus::dynamics::removeCompoundChildren(world, world.bodies.rigids[box].id, std::vector<uint32_t>{0u}));
  CHECK(!talus::dynamics::removeCompoundChildren(world, 999u, std::vector<uint32_t>{0u}));
  CHECK(world.bodies.rigids[index].compound == start.compound);
  CHECK(world.bodies.rigids[index].position == start.position);
  CHECK(world.bodies.rigids[index].mass == start.mass);
}

namespace {

struct Motion {
  glm::dvec3 momentum{0.0};
  glm::dvec3 angular{0.0};
  double energy = 0.0;
};

Motion motionOf(const talus::dynamics::BodySet& bodies) {
  Motion motion;

  for (const talus::dynamics::RigidBody& body : bodies.rigids) {
    if (!(body.mass > 0.0)) {
      continue;
    }

    const glm::dmat3 turn = glm::mat3_cast(body.rotation);
    const glm::dmat3 inertia = turn * diagonal(body.moment) * glm::transpose(turn);

    motion.momentum += body.mass * body.velocity;
    motion.angular += body.mass * glm::cross(body.position, body.velocity)
                      + inertia * body.angularVelocity;
    motion.energy += 0.5 * body.mass * glm::dot(body.velocity, body.velocity)
                     + 0.5 * glm::dot(body.angularVelocity, inertia * body.angularVelocity);
  }

  return motion;
}

std::vector<uint32_t> keysOf(const talus::dynamics::RigidBody& body) {
  std::vector<uint32_t> keys;

  for (const talus::dynamics::CompoundChild& child : body.compound->children) {
    keys.push_back(child.key);
  }

  return keys;
}

std::vector<talus::dynamics::RigidBody> splitBodies(const std::vector<std::vector<uint32_t>>& groups) {
  talus::dynamics::World world;
  const uint32_t index = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.1, 0.4, 0.2), glm::dvec3(0.3, 0.0, -0.2));

  world.bodies.rigids[index].angularVelocity = glm::dvec3(1.0, 0.5, -0.5);
  (void)talus::dynamics::splitCompound(world, world.bodies.rigids[index].id, groups);

  return world.bodies.rigids;
}

}

TEST_CASE("a split of a moving, spinning compound keeps momentum, angular momentum and energy", "[dynamics][compound]") {
  std::vector<talus::dynamics::CompoundChild> children = cubeOfEight(0.05, 1000.0, 0u);
  talus::dynamics::World world;

  for (size_t index = 0u; index != children.size(); ++index) {
    children[index].density = 1000.0 + 300.0 * static_cast<double>(index);
  }

  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    children,
    glm::dvec3(0.3, 1.2, -0.4),
    glm::angleAxis(0.8, glm::normalize(glm::dvec3(1.0, -2.0, 0.5))),
    glm::dvec3(1.0, 2.0, -0.5),
    0.5,
    0.0,
    false
  );

  world.bodies.rigids[index].angularVelocity = glm::dvec3(0.3, -1.1, 0.7);

  const talus::dynamics::RigidBody parent = world.bodies.rigids[index];
  const Motion before = motionOf(world.bodies);
  const std::vector<std::vector<uint32_t>> groups{{7u}, {4u, 6u, 5u}, {3u, 0u, 2u, 1u}};
  const std::vector<uint32_t> made = talus::dynamics::splitCompound(world, parent.id, groups);

  REQUIRE(made.size() == 2u);
  REQUIRE(world.bodies.rigids.size() == 3u);
  CHECK(world.bodies.rigids[0].id == parent.id);
  CHECK(made[0] == world.bodies.rigids[1].id);
  CHECK(made[1] == world.bodies.rigids[2].id);
  CHECK(keysOf(world.bodies.rigids[0]) == std::vector<uint32_t>{0u, 1u, 2u, 3u});
  CHECK(keysOf(world.bodies.rigids[1]) == std::vector<uint32_t>{4u, 5u, 6u});
  CHECK(keysOf(world.bodies.rigids[2]) == std::vector<uint32_t>{7u});

  const Motion after = motionOf(world.bodies);

  CHECK(glm::length(after.momentum - before.momentum) <= 1.0e-12 * glm::length(before.momentum));
  CHECK(glm::length(after.angular - before.angular) <= 1.0e-12 * glm::length(before.angular));
  CHECK(std::abs(after.energy - before.energy) <= 1.0e-12 * before.energy);

  for (const talus::dynamics::RigidBody& piece : world.bodies.rigids) {
    for (const talus::dynamics::CompoundChild& child : piece.compound->children) {
      const std::optional<uint32_t> was = talus::dynamics::findChild(*parent.compound, child.key);

      REQUIRE(was.has_value());

      const std::array<glm::dvec3, 8> then =
        cornersOf(parent.position, parent.rotation, parent.compound->children[*was]);
      const std::array<glm::dvec3, 8> now = cornersOf(piece.position, piece.rotation, child);

      for (size_t corner = 0u; corner != then.size(); ++corner) {
        CHECK(glm::length(now[corner] - then[corner]) <= 1.0e-12);
      }
    }
  }
}

TEST_CASE("a zero-mass compound's split pieces start at rest with their own mass", "[dynamics][compound]") {
  std::vector<talus::dynamics::CompoundChild> column;
  talus::dynamics::World world;

  for (uint32_t level = 0u; level != 4u; ++level) {
    column.push_back(part(glm::dvec3(0.0, 0.1 * level, 0.0), glm::dvec3(0.05), 2000.0, level));
  }

  const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, column, glm::dvec3(0.0, 0.05, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);
  const talus::dynamics::RigidBody parent = world.bodies.rigids[index];
  const std::vector<std::vector<uint32_t>> groups{{0u, 1u, 2u}, {3u}};

  REQUIRE(talus::dynamics::splitCompound(world, parent.id, groups).size() == 1u);

  const talus::dynamics::RigidBody& anchor = world.bodies.rigids[index];
  const talus::dynamics::RigidBody& piece = world.bodies.rigids[1];

  CHECK(anchor.mass == 0.0);
  CHECK(anchor.position == parent.position);
  CHECK(keysOf(anchor) == std::vector<uint32_t>{0u, 1u, 2u});
  CHECK(std::abs(piece.mass - 2.0) <= 1.0e-12);
  CHECK(piece.velocity == glm::dvec3(0.0));
  CHECK(piece.angularVelocity == glm::dvec3(0.0));
  CHECK(!piece.asleep);
  CHECK(glm::length(piece.position - glm::dvec3(0.0, 0.35, 0.0)) <= 1.0e-12);
}

TEST_CASE("a split keeps the warm start of contact points on children that stay in one body", "[dynamics][compound]") {
  struct Held {
    uint32_t child = 0u;
    uint64_t key = 0u;
    talus::dynamics::ContactMemory memory;
  };

  std::vector<talus::dynamics::CompoundChild> bar;
  talus::dynamics::World world;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);

  for (uint32_t index = 0u; index != 3u; ++index) {
    bar.push_back(part(glm::dvec3(0.1 * index - 0.1, 0.0, 0.0), glm::dvec3(0.05), 1000.0, index));
  }

  const uint32_t body = talus::dynamics::addCompoundBody(world.bodies, bar, glm::dvec3(0.0, 0.1505, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const uint32_t slot = world.manifolds.find(talus::dynamics::pairKey(
    world.bodies,
    {talus::dynamics::Side::Rigid, body},
    {talus::dynamics::Side::Static, 0u}
  ));

  REQUIRE(slot != talus::dynamics::noSlot);

  std::vector<Held> held;

  for (size_t at = 0u; at != world.manifolds.at(slot).points.size(); ++at) {
    const talus::dynamics::CachedManifoldPoint& point = world.manifolds.at(slot).manifoldPoints[at];

    if (point.firstChild != 2u) {
      held.push_back({point.firstChild, point.key, world.manifolds.at(slot).points[at].memory});
    }
  }

  REQUIRE(!held.empty());

  const std::vector<std::vector<uint32_t>> groups{{0u, 1u}, {2u}};

  REQUIRE(talus::dynamics::splitCompound(world, world.bodies.rigids[body].id, groups).size() == 1u);

  const auto memoryOf = [&](uint32_t child, uint64_t key, uint32_t& matches) {
    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);
    talus::dynamics::ContactMemory found;

    matches = 0u;

    for (size_t at = 0u; at != pair.points.size(); ++at) {
      if (pair.manifoldPoints[at].firstChild == child && pair.manifoldPoints[at].key == key) {
        found = pair.points[at].memory;
        matches += 1u;
      }
    }

    return found;
  };

  for (const Held& point : held) {
    uint32_t matches = 0u;
    const talus::dynamics::ContactMemory memory = memoryOf(point.child, point.key, matches);

    REQUIRE(matches == 1u);
    CHECK(memory.normal == point.memory.normal);
    CHECK(memory.friction == point.memory.friction);
  }

  for (const talus::dynamics::CachedManifoldPoint& point : world.manifolds.at(slot).manifoldPoints) {
    CHECK(point.firstChild != 2u);
  }

  const std::vector<double> still(world.bodies.lattices.size() + world.bodies.rigids.size(), 0.0);

  (void)talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0);

  uint32_t carried = 0u;

  for (const Held& point : held) {
    uint32_t matches = 0u;
    const talus::dynamics::ContactMemory memory = memoryOf(point.child, point.key, matches);

    if (matches == 1u) {
      CHECK(memory.normal == point.memory.normal);
      carried += 1u;
    }
  }

  CHECK(carried >= 2u);
}

TEST_CASE("splitting a compound at rest on the floor leaves its pieces at rest", "[dynamics][compound]") {
  talus::dynamics::World world;

  world.parameters.sleep = false;
  talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);

  const uint32_t index = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.1505, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  const std::vector<std::vector<uint32_t>> halves{{0u, 2u, 4u, 6u}, {1u, 3u, 5u, 7u}};

  REQUIRE(talus::dynamics::splitCompound(world, world.bodies.rigids[index].id, halves).size() == 1u);

  std::vector<glm::dvec3> places;
  double fastest = 0.0;
  double drift = 0.0;
  double energy = 1.0;

  for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
    places.push_back(body.position);
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (size_t at = 0u; at != world.bodies.rigids.size(); ++at) {
      fastest = std::max(fastest, glm::length(world.bodies.rigids[at].velocity));
      drift = std::max(drift, glm::length(world.bodies.rigids[at].position - places[at]));
    }

    energy = std::max(energy, talus::dynamics::energyOf(world.bodies, g).total() / start);
  }

  CHECK(fastest < 0.05);
  CHECK(drift < 0.01);
  CHECK(energy <= 1.02);
}

TEST_CASE("a split gives the same bodies whatever order its groups come in", "[dynamics][compound]") {
  const std::vector<talus::dynamics::RigidBody> first = splitBodies({{0u, 1u, 2u}, {3u, 4u}, {5u, 6u, 7u}});
  const std::vector<talus::dynamics::RigidBody> second = splitBodies({{7u, 6u, 5u}, {2u, 0u, 1u}, {4u, 3u}});

  REQUIRE(first.size() == 3u);
  REQUIRE(second.size() == first.size());

  for (size_t index = 0u; index != first.size(); ++index) {
    CHECK(first[index].id == second[index].id);
    CHECK(first[index].position == second[index].position);
    CHECK(first[index].rotation == second[index].rotation);
    CHECK(first[index].velocity == second[index].velocity);
    CHECK(first[index].mass == second[index].mass);
    CHECK(keysOf(first[index]) == keysOf(second[index]));
  }

  CHECK(keysOf(first[0]) == std::vector<uint32_t>{0u, 1u, 2u});
  CHECK(keysOf(first[1]) == std::vector<uint32_t>{3u, 4u});
  CHECK(keysOf(first[2]) == std::vector<uint32_t>{5u, 6u, 7u});
}

TEST_CASE("a split whose groups overlap, miss a child or name an unknown one is refused", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.0));
  const talus::dynamics::RigidBody start = world.bodies.rigids[index];
  const auto refused = [&](const std::vector<std::vector<uint32_t>>& groups) {
    return talus::dynamics::splitCompound(world, start.id, groups).empty();
  };

  CHECK(refused({{0u, 1u, 2u, 3u}, {3u, 4u, 5u, 6u, 7u}}));
  CHECK(refused({{0u, 1u, 2u}, {4u, 5u, 6u, 7u}}));
  CHECK(refused({{0u, 1u, 2u, 3u}, {4u, 5u, 6u, 9u}}));
  CHECK(refused({{0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u}}));
  CHECK(refused({{0u, 1u, 2u, 3u}, {}, {4u, 5u, 6u, 7u}}));
  CHECK(world.bodies.rigids.size() == 1u);
  CHECK(world.bodies.rigids[index].compound == start.compound);
}

namespace {

uint64_t compoundSceneHash(uint32_t workers, uint32_t width, double lift) {
  std::vector<talus::dynamics::CompoundChild> shelf;
  talus::dynamics::World world;

  world.workerCount = workers;
  world.simdWidth = width;
  talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);

  for (uint32_t index = 0u; index != 3u; ++index) {
    shelf.push_back(part(glm::dvec3(0.2 * index, 0.0, 0.0), glm::dvec3(0.1, 0.02, 0.1), 600.0, index));
  }

  talus::dynamics::addCompoundBody(world.bodies, shelf, glm::dvec3(0.3, 0.42, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  for (uint32_t level = 0u; level != 3u; ++level) {
    for (uint32_t column = 0u; column != 2u; ++column) {
      const glm::dquat turn = glm::angleAxis(0.05 * (level + 2.0 * column), glm::dvec3(0.0, 1.0, 0.0));

      talus::dynamics::addCompoundBody(
        world.bodies,
        cubeOfEight(0.025, 1000.0, 10u * level + column),
        glm::dvec3(-0.3 + 0.12 * column, 0.1505 + 0.101 * level + lift, 0.0),
        turn,
        glm::dvec3(0.0),
        0.5,
        0.0,
        false
      );
    }
  }

  const uint32_t thrown = talus::dynamics::addCompoundBody(
    world.bodies, lShape(), glm::dvec3(0.5, 0.8, 0.0), glm::angleAxis(0.3, glm::dvec3(0.0, 0.0, 1.0)), glm::dvec3(-0.6, 0.0, 0.0), 0.5, 0.0, false
  );
  const uint32_t thrownId = world.bodies.rigids[thrown].id;
  const uint32_t topId = world.bodies.rigids[5].id;

  world.bodies.rigids[thrown].angularVelocity = glm::dvec3(0.0, 2.0, 1.0);

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    if (frame == 30u) {
      const std::vector<std::vector<uint32_t>> pieces{{1u}, {2u}, {3u}};

      if (talus::dynamics::splitCompound(world, thrownId, pieces).size() != 2u) {
        return 0u;
      }
    }

    if (frame == 60u && !talus::dynamics::removeCompoundChildren(world, topId, std::vector<uint32_t>{22u, 23u})) {
      return 0u;
    }

    if (talus::dynamics::step(world).nonFinite) {
      return 0u;
    }
  }

  return stateHash(world);
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

TEST_CASE("compounds that collide, stack, split and lose children step the same on 1 and 8 workers", "[dynamics][compound]") {
  const uint64_t serial = compoundSceneHash(1u, 1u, 0.0);

  REQUIRE(serial != 0u);
  CHECK(compoundSceneHash(8u, 0u, 0.0) == serial);
  CHECK(compoundSceneHash(1u, 1u, 1.0e-9) != serial);
}

TEST_CASE("compound state hash", "[.][compoundhash]") {
  uint64_t first = 0u;

  for (const auto& [width, workers] : widthRuns()) {
    const uint64_t hash = compoundSceneHash(workers, width, 0.0);

    std::printf("compoundhash width %u workers %u hash %016llx\n", width, workers, static_cast<unsigned long long>(hash));

    if (first == 0u) {
      first = hash;
    } else {
      CHECK(hash == first);
    }
  }

  CHECK(first != 0u);
}

TEST_CASE("compound pile cost against the box pile", "[.][compoundbench]") {
  const talus::field::SampledSdf brick = boxShape(glm::dvec3(0.03));
  const double density = 0.5 / (0.06 * 0.06 * 0.06);

  for (const bool compounds : {false, true}) {
    for (const uint32_t workers : {1u, 8u}) {
      talus::dynamics::World world;

      world.workerCount = workers;
      world.parameters.sleep = false;
      talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);

      for (int row = 0; row != 10; ++row) {
        for (int column = 0; column != 10; ++column) {
          for (int level = 0; level != 10; ++level) {
            const glm::dvec3 place(-0.3 + 0.0605 * row, 0.13 + 0.0605 * level, -0.3 + 0.0605 * column);

            if (compounds) {
              talus::dynamics::addCompoundBody(world.bodies, cubeOfEight(0.015, density, 0u), place, noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
            } else {
              talus::dynamics::addRigidBox(world.bodies, brick, glm::dvec3(0.03), 0.5, place, glm::dvec3(0.0), 0.5, 3u);
            }
          }
        }
      }

      for (uint32_t frame = 0u; frame != 30u; ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
      }

      double elapsed = 0.0;

      for (uint32_t frame = 0u; frame != 60u; ++frame) {
        const auto before = std::chrono::steady_clock::now();

        REQUIRE(!talus::dynamics::step(world).nonFinite);
        elapsed += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
      }

      uint32_t pairs = 0u;
      uint32_t manifolds = 0u;

      for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
        if (world.manifolds.live(slot) && !world.manifolds.at(slot).points.empty()) {
          pairs += 1u;
          manifolds += static_cast<uint32_t>(world.manifolds.at(slot).manifolds.size());
        }
      }

      std::printf(
        "compoundbench shape %s workers %u ms %.3f pairs %u manifolds %u points %u\n",
        compounds ? "compound" : "box",
        workers,
        elapsed / 60.0,
        pairs,
        manifolds,
        world.manifolds.pointCount()
      );
    }
  }
}

namespace {

constexpr uint32_t RUBBLE_PIECES = 5000u;
constexpr uint32_t RUBBLE_STEPS = 600u;
constexpr uint32_t RUBBLE_FALL = 150u;

struct RubbleRandom {
  uint64_t state = 0x9E3779B97F4A7C15ull;

  double next() {
    state = state * 6364136223846793005ull + 1442695040888963407ull;

    return static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
  }

  double between(double low, double high) {
    return low + (high - low) * next();
  }
};

struct ChunkKind {
  glm::dvec3 half{0.0};
  double density = 0.0;
};

std::vector<talus::dynamics::CompoundChild> rubblePiece(uint32_t chunks, const ChunkKind& kind) {
  const bool flat = kind.half.y < 0.05;
  uint32_t across = 1u;

  while ((flat ? across * across : across * across * across) < chunks) {
    across += 1u;
  }

  std::vector<talus::dynamics::CompoundChild> children;

  for (uint32_t index = 0u; index != chunks; ++index) {
    const uint32_t x = index % across;
    const uint32_t z = (index / across) % across;
    const uint32_t y = index / (across * across);

    children.push_back(part(
      2.0 * kind.half * glm::dvec3(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)),
      kind.half,
      kind.density,
      index
    ));
  }

  return children;
}

double pieceRadius(const std::vector<talus::dynamics::CompoundChild>& children) {
  glm::dvec3 low(1.0e9);
  glm::dvec3 high(-1.0e9);

  for (const talus::dynamics::CompoundChild& child : children) {
    low = glm::min(low, child.centre - child.half);
    high = glm::max(high, child.centre + child.half);
  }

  return 0.5 * glm::length(high - low);
}

uint32_t rubbleChunks(uint32_t order) {
  if (order < 4000u) {
    return 1u;
  }

  RubbleRandom pick{0xC0FFEEull + order};

  if (order < 4650u) {
    return 2u + static_cast<uint32_t>(pick.next() * 4.0);
  }

  if (order < 4850u) {
    return 6u + static_cast<uint32_t>(pick.next() * 5.0);
  }

  if (order < 4950u) {
    return 11u + static_cast<uint32_t>(pick.next() * 10.0);
  }

  return 21u + static_cast<uint32_t>(pick.next() * 40.0);
}

void addRubble(talus::dynamics::World& world) {
  const std::array<ChunkKind, 4> kinds{{
    {glm::dvec3(0.25, 0.15, 0.125), 2400.0},
    {glm::dvec3(0.25, 0.25, 0.15), 2400.0},
    {glm::dvec3(0.125), 1800.0},
    {glm::dvec3(0.25, 0.01, 0.25), 2500.0}
  }};
  std::vector<talus::dynamics::CompoundChild> tiles;
  RubbleRandom random;
  std::vector<uint32_t> order(RUBBLE_PIECES);
  std::vector<glm::dvec4> placed;
  std::unordered_map<uint64_t, std::vector<uint32_t>> cells;
  const double cell = 3.0;
  const auto cellKey = [](int64_t x, int64_t y, int64_t z) {
    return (static_cast<uint64_t>(x + 1024) << 42) | (static_cast<uint64_t>(y + 1024) << 21) | static_cast<uint64_t>(z + 1024);
  };

  for (uint32_t row = 0u; row != 4u; ++row) {
    for (uint32_t column = 0u; column != 4u; ++column) {
      tiles.push_back(part(glm::dvec3(16.0 * column - 24.0, 0.0, 16.0 * row - 24.0), glm::dvec3(8.0, 0.5, 8.0), 2400.0, 4u * row + column));
    }
  }

  (void)talus::dynamics::addCompoundBody(world.bodies, tiles, glm::dvec3(0.0, -0.5, 0.0), noTurn, glm::dvec3(0.0), 0.6, 0.0, true);

  for (uint32_t index = 0u; index != RUBBLE_PIECES; ++index) {
    order[index] = index;
  }

  for (uint32_t index = RUBBLE_PIECES - 1u; index != 0u; --index) {
    std::swap(order[index], order[static_cast<uint32_t>(random.next() * static_cast<double>(index + 1u))]);
  }

  for (const uint32_t piece : order) {
    const double roll = random.next();
    const ChunkKind& kind = kinds[roll < 0.6 ? 0u : roll < 0.75 ? 1u : roll < 0.9 ? 2u : 3u];
    const std::vector<talus::dynamics::CompoundChild> children = rubblePiece(rubbleChunks(piece), kind);
    const double radius = pieceRadius(children);
    glm::dvec3 centre(0.0);
    double ceiling = 23.0;

    for (uint32_t attempt = 0u;; ++attempt) {
      centre = glm::dvec3(random.between(-11.0, 11.0), random.between(1.0 + radius, ceiling), random.between(-11.0, 11.0));

      const int64_t cx = static_cast<int64_t>(std::floor(centre.x / cell));
      const int64_t cy = static_cast<int64_t>(std::floor(centre.y / cell));
      const int64_t cz = static_cast<int64_t>(std::floor(centre.z / cell));
      bool clear = true;

      for (int64_t dx = -1; dx <= 1 && clear; ++dx) {
        for (int64_t dy = -1; dy <= 1 && clear; ++dy) {
          for (int64_t dz = -1; dz <= 1 && clear; ++dz) {
            const auto found = cells.find(cellKey(cx + dx, cy + dy, cz + dz));

            if (found == cells.end()) {
              continue;
            }

            for (const uint32_t other : found->second) {
              const glm::dvec4& sphere = placed[other];

              if (glm::length(glm::dvec3(sphere) - centre) < sphere.w + radius + 0.05) {
                clear = false;
                break;
              }
            }
          }
        }
      }

      if (clear) {
        cells[cellKey(cx, cy, cz)].push_back(static_cast<uint32_t>(placed.size()));
        placed.emplace_back(centre, radius);
        break;
      }

      if (attempt % 64u == 63u) {
        ceiling += 1.0;
      }
    }

    const glm::dquat turn = glm::normalize(glm::dquat(random.between(-1.0, 1.0) + 1.0e-3, random.between(-1.0, 1.0), random.between(-1.0, 1.0), random.between(-1.0, 1.0)));
    const glm::dvec3 velocity(random.between(-1.0, 1.0), random.between(-1.0, 0.0), random.between(-1.0, 1.0));
    const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, children, centre, turn, velocity, 0.6, 0.0, false);

    world.bodies.rigids[index].angularVelocity = glm::dvec3(random.between(-1.0, 1.0), random.between(-1.0, 1.0), random.between(-1.0, 1.0));
  }
}

struct RubbleStep {
  talus::dynamics::StepProfile profile;
  uint32_t awake = 0u;
  uint32_t pairs = 0u;
  uint32_t touching = 0u;
  uint32_t manifolds = 0u;
  uint32_t points = 0u;
  uint32_t islands = 0u;
  uint32_t awakeIslands = 0u;
};

RubbleStep rubbleCounts(const talus::dynamics::World& world, const talus::dynamics::StepProfile& profile) {
  RubbleStep counted;

  counted.profile = profile;

  for (const talus::dynamics::RigidBody& body : world.bodies.rigids) {
    counted.awake += body.mass > 0.0 && !body.asleep ? 1u : 0u;
  }

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot)) {
      continue;
    }

    const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

    counted.pairs += 1u;
    counted.touching += pair.points.empty() ? 0u : 1u;
    counted.manifolds += static_cast<uint32_t>(pair.manifolds.size());
  }

  counted.points = world.manifolds.pointCount();

  for (const talus::dynamics::Island& island : world.islands) {
    counted.islands += 1u;
    counted.awakeIslands += island.asleep ? 0u : 1u;
  }

  return counted;
}

struct RubblePhase {
  const char* name = "";
  double (*read)(const talus::dynamics::StepProfile&) = nullptr;
};

const std::array<RubblePhase, 20> RUBBLE_PHASES{{
  {"total", [](const talus::dynamics::StepProfile& p) { return p.total; }},
  {"travel", [](const talus::dynamics::StepProfile& p) { return p.travel; }},
  {"broadphase", [](const talus::dynamics::StepProfile& p) { return p.broadphase; }},
  {"pair update", [](const talus::dynamics::StepProfile& p) { return p.storeUpdate; }},
  {"  recycle", [](const talus::dynamics::StepProfile& p) { return p.pairs.recycle; }},
  {"  midphase", [](const talus::dynamics::StepProfile& p) { return p.pairs.midphase; }},
  {"  narrowphase", [](const talus::dynamics::StepProfile& p) { return p.pairs.narrowphase; }},
  {"  reduce", [](const talus::dynamics::StepProfile& p) { return p.pairs.reduce; }},
  {"  manifold build", [](const talus::dynamics::StepProfile& p) { return p.pairs.build; }},
  {"lattice", [](const talus::dynamics::StepProfile& p) { return p.lattice; }},
  {"islands", [](const talus::dynamics::StepProfile& p) { return p.islands; }},
  {"solver setup", [](const talus::dynamics::StepProfile& p) { return p.solverSetup; }},
  {"gather", [](const talus::dynamics::StepProfile& p) { return p.gather; }},
  {"prepare", [](const talus::dynamics::StepProfile& p) { return p.prepare; }},
  {"integrate", [](const talus::dynamics::StepProfile& p) { return p.integrate; }},
  {"solve", [](const talus::dynamics::StepProfile& p) { return p.substeps - p.integrate - p.restitute; }},
  {"restitute", [](const talus::dynamics::StepProfile& p) { return p.restitute; }},
  {"store back", [](const talus::dynamics::StepProfile& p) { return p.storeBack; }},
  {"continuous", [](const talus::dynamics::StepProfile& p) { return p.continuous; }},
  {"events and sleep", [](const talus::dynamics::StepProfile& p) { return p.events + p.sleep; }}
}};

void printRubbleWindow(const char* run, const char* window, std::span<const RubbleStep> steps) {
  for (const RubblePhase& phase : RUBBLE_PHASES) {
    std::vector<double> values;
    double sum = 0.0;

    for (const RubbleStep& step : steps) {
      values.push_back(phase.read(step.profile));
      sum += values.back();
    }

    std::sort(values.begin(), values.end());

    const size_t at = std::min(values.size() - 1u, static_cast<size_t>(std::ceil(0.99 * static_cast<double>(values.size()))) - 1u);

    std::printf("rigidbench %s %s | %-16s mean %8.3f p99 %8.3f worst %8.3f ms\n", run, window, phase.name, sum / static_cast<double>(values.size()), values[at], values.back());
  }

  double awake = 0.0;
  double pairs = 0.0;
  double touching = 0.0;
  double manifolds = 0.0;
  double points = 0.0;
  double islands = 0.0;
  double awakeIslands = 0.0;
  double tested = 0.0;
  double recycled = 0.0;
  double childPairs = 0.0;
  double redone = 0.0;

  for (const RubbleStep& step : steps) {
    awake += step.awake;
    pairs += step.pairs;
    touching += step.touching;
    manifolds += step.manifolds;
    points += step.points;
    islands += step.islands;
    awakeIslands += step.awakeIslands;
    tested += step.profile.pairs.tested;
    recycled += step.profile.pairs.recycled;
    childPairs += step.profile.pairs.childPairs;
    redone += step.profile.pairs.redone;
  }

  const double count = static_cast<double>(steps.size());

  std::printf(
    "rigidbench %s %s | means: awake %.0f pairs %.0f touching %.0f manifolds %.0f points %.0f islands %.0f awake islands %.0f | pairs tested %.0f recycled %.0f child pairs %.0f redone %.0f\n",
    run, window, awake / count, pairs / count, touching / count, manifolds / count, points / count, islands / count, awakeIslands / count,
    tested / count, recycled / count, childPairs / count, redone / count
  );
}

uint64_t runRubble(uint32_t workers, bool sleep, bool timed) {
  talus::dynamics::World world;
  std::vector<RubbleStep> steps;
  char run[96];

  world.workerCount = workers;
  world.parameters.sleep = sleep;
  world.parameters.timePairs = timed;
  addRubble(world);
  std::snprintf(run, sizeof(run), "workers %u sleep %s pair timers %s", workers, sleep ? "on" : "off", timed ? "on" : "off");

  for (uint32_t frame = 1u; frame <= RUBBLE_STEPS; ++frame) {
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    REQUIRE(!report.nonFinite);
    steps.push_back(rubbleCounts(world, report.profile));

    {
      const RubbleStep& last = steps.back();
      const talus::dynamics::StepProfile& p = last.profile;

      std::printf(
        "rigidstep %s step %u | awake %u pairs %u touching %u manifolds %u points %u islands %u awake %u | total %.3f travel %.3f broadphase %.3f update %.3f [recycle %.3f mid %.3f narrow %.3f reduce %.3f build %.3f tested %u recycled %u childpairs %u] islands %.3f setup %.3f gather %.3f prepare %.3f integrate %.3f solve %.3f restitute %.3f store %.3f continuous %.3f events %.3f sleep %.3f\n",
        run, frame, last.awake, last.pairs, last.touching, last.manifolds, last.points, last.islands, last.awakeIslands,
        p.total, p.travel, p.broadphase, p.storeUpdate, p.pairs.recycle, p.pairs.midphase, p.pairs.narrowphase, p.pairs.reduce, p.pairs.build,
        p.pairs.tested, p.pairs.recycled, p.pairs.childPairs, p.islands, p.solverSetup, p.gather, p.prepare, p.integrate,
        p.substeps - p.integrate - p.restitute, p.restitute, p.storeBack, p.continuous, p.events, p.sleep
      );
    }
  }

  printRubbleWindow(run, "falling", std::span<const RubbleStep>(steps).subspan(0u, RUBBLE_FALL));
  printRubbleWindow(run, "settling", std::span<const RubbleStep>(steps).subspan(RUBBLE_FALL));

  const uint64_t hash = stateHash(world);

  std::printf("rigidbench %s | hash %016llx bodies %zu\n", run, static_cast<unsigned long long>(hash), world.bodies.rigids.size());

  return hash;
}

}

TEST_CASE("rigid cost of the rubble pile on 4, 8 and 16 workers", "[.][rigidworkers]") {
  const uint64_t four = runRubble(4u, true, false);

  CHECK(runRubble(8u, true, false) == four);
  CHECK(runRubble(16u, true, false) == four);
}

TEST_CASE("rigid cost of 5,000 awake rubble pieces falling, colliding and settling", "[.][rigidbench]") {
  for (const bool sleep : {true, false}) {
    const uint64_t serial = runRubble(1u, sleep, false);
    const uint64_t parallel = runRubble(8u, sleep, false);
    const uint64_t serialTimed = runRubble(1u, sleep, true);
    const uint64_t parallelTimed = runRubble(8u, sleep, true);

    CHECK(serial == parallel);
    CHECK(serial == serialTimed);
    CHECK(serial == parallelTimed);
  }
}

namespace {

constexpr uint32_t STALL_STEPS = 120u;
constexpr uint32_t STALL_WORKERS = 8u;
constexpr double STALL_OVERHEAD = 2.0;
constexpr double STALL_LATENCY_MS = 8.0 * 15.625;

std::vector<double> rubbleStepTimes(uint32_t workers, uint64_t& hash) {
  talus::dynamics::World world;
  std::vector<double> times;

  world.workerCount = workers;
  addRubble(world);

  for (uint32_t frame = 0u; frame != STALL_STEPS; ++frame) {
    const auto begin = std::chrono::steady_clock::now();

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count());
  }

  hash = stateHash(world);

  return times;
}

}

TEST_CASE("rubble steps on 8 workers stay within a fair share of the CPU while other threads load every core", "[.][stallstress]") {
  const uint32_t cores = std::max(1u, std::thread::hardware_concurrency());
  uint64_t serialHash = 0u;
  uint64_t loadedHash = 0u;
  const std::vector<double> serial = rubbleStepTimes(1u, serialHash);
  std::vector<double> loaded;

  {
    std::vector<std::jthread> burners;

    for (uint32_t core = 0u; core != cores; ++core) {
      burners.emplace_back([](std::stop_token stop) {
        while (!stop.stop_requested()) {
        }
      });
      REQUIRE(testthreads::pinToCore(burners.back(), core));
    }

    loaded = rubbleStepTimes(STALL_WORKERS, loadedHash);
  }

  const double share = static_cast<double>(cores + STALL_WORKERS) / static_cast<double>(cores);
  uint32_t over = 0u;
  double worst = 0.0;
  uint32_t worstStep = 0u;

  for (uint32_t at = 0u; at != STALL_STEPS; ++at) {
    const double bound = STALL_OVERHEAD * serial[at] * share + STALL_LATENCY_MS;

    std::printf("stallstress step %u serial %.2f loaded %.2f bound %.2f ms\n", at + 1u, serial[at], loaded[at], bound);
    over += loaded[at] > bound ? 1u : 0u;

    if (loaded[at] / bound > worst) {
      worst = loaded[at] / bound;
      worstStep = at + 1u;
    }
  }

  std::printf("stallstress cores %u workers %u burners %u steps over the bound %u, worst step %u at %.2f of its bound\n", cores, STALL_WORKERS, cores, over, worstStep, worst);

  CHECK(loadedHash == serialHash);
  CHECK(over == 0u);
}

TEST_CASE("replacing a body's shape with a compound gives it that compound's children", "[dynamics][compound]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.05));
  talus::dynamics::World world;
  talus::dynamics::BodySet scratch;

  talus::dynamics::addStatic(world.bodies, floorShape(), 0.5);

  const uint32_t box = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.05), 1.0, glm::dvec3(0.0, 0.1505, 0.0), glm::dvec3(0.0), 0.5, 3u);

  (void)talus::dynamics::addCompoundBody(scratch, cubeOfEight(0.025, 1000.0, 0u), glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  REQUIRE(talus::dynamics::replaceRigidShape(world, world.bodies.rigids[box].id, scratch.rigids.front()));
  REQUIRE(world.bodies.rigids[box].compound == scratch.rigids.front().compound);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(std::abs(world.bodies.rigids[box].position.y - 0.15) <= 0.01);
}

TEST_CASE("a box pressed deeper than its half-width into a chunk seam keeps its contact", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> floor{
    part(glm::dvec3(-0.1, 0.0, 0.0), glm::dvec3(0.1, 0.05, 0.1), 1000.0, 0u),
    part(glm::dvec3(0.1, 0.0, 0.0), glm::dvec3(0.1, 0.05, 0.1), 1000.0, 1u)
  };

  for (const double across : {0.0, 0.005}) {
    talus::dynamics::World world;
    const uint32_t slab = talus::dynamics::addCompoundBody(world.bodies, floor, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);
    const uint32_t box = addOneCube(world, 0.02, 1000.0, glm::dvec3(across, 0.04, 0.0), glm::dvec3(0.0));
    double lowest = 0.04;
    size_t touching = 0u;

    for (uint32_t frame = 0u; frame != 60u; ++frame) {
      REQUIRE(!talus::dynamics::step(world).nonFinite);

      if (frame == 0u) {
        const uint32_t slot = world.manifolds.find(talus::dynamics::pairKey(
          world.bodies,
          {talus::dynamics::Side::Rigid, slab},
          {talus::dynamics::Side::Rigid, box}
        ));

        touching = slot == talus::dynamics::noSlot ? 0u : world.manifolds.at(slot).points.size();
      }

      lowest = std::min(lowest, world.bodies.rigids[box].position.y);
    }

    CHECK(touching > 0u);
    CHECK(lowest >= 0.04 - 0.01);
  }
}

namespace {

std::vector<talus::dynamics::CompoundChild> brickWall(uint32_t across, uint32_t high, uint32_t deep) {
  std::vector<talus::dynamics::CompoundChild> children;
  uint32_t index = 0u;

  for (uint32_t x = 0u; x != across; ++x) {
    for (uint32_t y = 0u; y != high; ++y) {
      for (uint32_t z = 0u; z != deep; ++z) {
        const double turn = index % 7u == 3u ? 0.5 * std::numbers::pi : index % 11u == 5u ? 0.3 : 0.0;

        children.push_back(part(
          glm::dvec3(0.2 * x, 0.1 * y, 0.2 * z),
          glm::dvec3(0.1, 0.05, 0.1),
          1000.0 + static_cast<double>(index % 5u),
          (index * 37u) % 1009u,
          glm::angleAxis(turn, glm::dvec3(0.0, 1.0, 0.0))
        ));
        index += 1u;
      }
    }
  }

  return children;
}

uint32_t shapeMismatches(
  const talus::dynamics::CompoundShape& edited,
  const talus::dynamics::CompoundShape& rebuilt
) {
  if (edited.children.size() != rebuilt.children.size()
      || edited.childBounds.size() != rebuilt.childBounds.size()
      || edited.exposed.size() != rebuilt.exposed.size()) {
    return 1u;
  }

  uint32_t mismatches = 0u;

  for (size_t index = 0u; index != rebuilt.children.size(); ++index) {
    const talus::dynamics::CompoundChild& a = edited.children[index];
    const talus::dynamics::CompoundChild& b = rebuilt.children[index];

    mismatches += a.key != b.key || a.centre != b.centre || a.rotation != b.rotation
                    || a.half != b.half || a.density != b.density
                    ? 1u
                    : 0u;
    mismatches += edited.childBounds[index].low != rebuilt.childBounds[index].low
                    || edited.childBounds[index].high != rebuilt.childBounds[index].high
                    ? 1u
                    : 0u;
    mismatches += edited.exposed[index] != rebuilt.exposed[index] ? 1u : 0u;
  }

  mismatches += edited.bounds.low != rebuilt.bounds.low || edited.bounds.high != rebuilt.bounds.high ? 1u : 0u;
  mismatches += edited.reach != rebuilt.reach || edited.thinnest != rebuilt.thinnest ? 1u : 0u;
  mismatches += edited.faceCovers != rebuilt.faceCovers || edited.childReach != rebuilt.childReach ? 1u : 0u;

  return mismatches;
}

uint32_t queryMismatches(
  const talus::dynamics::CompoundShape& edited,
  const talus::dynamics::CompoundShape& rebuilt
) {
  std::vector<uint32_t> fromEdited;
  std::vector<uint32_t> fromRebuilt;
  std::vector<uint32_t> expected;
  uint64_t state = 99991u;
  const auto next = [&state] {
    state = state * 6364136223846793005ull + 1442695040888963407ull;

    return static_cast<double>(state >> 11u) / 9007199254740992.0;
  };
  const glm::dvec3 low = rebuilt.bounds.low;
  const glm::dvec3 size = rebuilt.bounds.high - rebuilt.bounds.low;
  uint32_t mismatches = 0u;

  for (int probe = 0; probe != 96; ++probe) {
    const glm::dvec3 centre = low + glm::dvec3(next() * size.x, next() * size.y, next() * size.z);
    const glm::dvec3 reach = glm::dvec3(0.02 + 0.3 * next(), 0.02 + 0.3 * next(), 0.02 + 0.3 * next());
    const talus::dynamics::Bounds box{centre - reach, centre + reach};

    expected.clear();

    for (uint32_t index = 0u; index != rebuilt.childBounds.size(); ++index) {
      if (talus::dynamics::overlaps(rebuilt.childBounds[index], box)) {
        expected.push_back(index);
      }
    }

    talus::dynamics::overlappingChildren(edited, box, fromEdited);
    talus::dynamics::overlappingChildren(rebuilt, box, fromRebuilt);
    mismatches += fromEdited != expected || fromRebuilt != expected ? 1u : 0u;
  }

  return mismatches;
}

std::vector<talus::dynamics::CompoundChild> childrenWithout(
  const std::vector<talus::dynamics::CompoundChild>& children,
  const std::vector<uint32_t>& keys
) {
  std::vector<talus::dynamics::CompoundChild> kept;

  for (const talus::dynamics::CompoundChild& child : children) {
    if (std::find(keys.begin(), keys.end(), child.key) == keys.end()) {
      kept.push_back(child);
    }
  }

  return kept;
}

}

TEST_CASE("an edited anchored compound has exactly the shape a full rebuild gives", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    brickWall(8u, 6u, 5u),
    glm::dvec3(1.0, 0.5, -2.0),
    glm::angleAxis(0.2, glm::dvec3(0.0, 1.0, 0.0)),
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );
  const uint32_t id = world.bodies.rigids[index].id;
  std::vector<talus::dynamics::CompoundChild> kept = world.bodies.rigids[index].compound->children;
  std::vector<uint32_t> block;
  std::vector<uint32_t> scattered;

  for (uint32_t at = 0u; at != 240u; ++at) {
    const uint32_t x = at / 30u;
    const uint32_t y = (at / 5u) % 6u;
    const uint32_t z = at % 5u;

    if (x >= 2u && x <= 4u && y >= 1u && y <= 3u && z >= 1u && z <= 3u) {
      block.push_back((at * 37u) % 1009u);
    } else if (at % 9u == 4u) {
      scattered.push_back((at * 37u) % 1009u);
    }
  }

  REQUIRE(talus::dynamics::removeCompoundChildren(world, id, block));
  kept = childrenWithout(kept, block);
  CHECK(shapeMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(kept)) == 0u);
  CHECK(queryMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(kept)) == 0u);

  REQUIRE(talus::dynamics::removeCompoundChildren(world, id, scattered));
  kept = childrenWithout(kept, scattered);
  CHECK(shapeMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(kept)) == 0u);
  CHECK(queryMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(kept)) == 0u);

  std::vector<uint32_t> near;
  std::vector<uint32_t> far;

  for (const talus::dynamics::CompoundChild& child : kept) {
    (child.centre.x < 0.5 ? near : far).push_back(child.key);
  }

  const std::vector<uint32_t>& stays = near.size() > far.size() || (near.size() == far.size() && near.front() < far.front()) ? near : far;
  const std::vector<std::vector<uint32_t>> groups{near, far};

  REQUIRE(talus::dynamics::splitCompound(world, id, groups).size() == 1u);

  const std::vector<talus::dynamics::CompoundChild> parent = childrenWithout(kept, stays == near ? far : near);

  CHECK(shapeMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(parent)) == 0u);
  CHECK(queryMismatches(*world.bodies.rigids[index].compound, *talus::dynamics::makeCompoundShape(parent)) == 0u);
}

TEST_CASE("removing no children leaves a compound shape as it is", "[dynamics][compound]") {
  const std::vector<talus::dynamics::CompoundChild> wall = brickWall(3u, 2u, 2u);
  std::shared_ptr<const talus::dynamics::CompoundShape> shape = talus::dynamics::makeCompoundShape(wall);
  const std::shared_ptr<const talus::dynamics::CompoundShape> kept = shape;

  talus::dynamics::removeChildren(shape, std::vector<uint32_t>{});

  CHECK(shape == kept);
  CHECK(shapeMismatches(*shape, *talus::dynamics::makeCompoundShape(wall)) == 0u);
}

TEST_CASE("editing a compound leaves a copy of its body as it was", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies, brickWall(4u, 3u, 3u), glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true
  );
  const talus::dynamics::RigidBody copy = world.bodies.rigids[index];
  const std::vector<talus::dynamics::CompoundChild> all = copy.compound->children;
  const std::vector<uint32_t> gone{all[4].key, all[13].key, all[17].key};

  REQUIRE(talus::dynamics::removeCompoundChildren(world, copy.id, gone));
  CHECK(world.bodies.rigids[index].compound->children.size() == all.size() - gone.size());
  CHECK(shapeMismatches(*copy.compound, *talus::dynamics::makeCompoundShape(all)) == 0u);
  CHECK(queryMismatches(*copy.compound, *talus::dynamics::makeCompoundShape(all)) == 0u);
}

namespace {

uint32_t addGridBuilding(talus::dynamics::World& world, uint32_t across, uint32_t high, uint32_t deep) {
  std::vector<talus::dynamics::CompoundChild> children;

  children.reserve(static_cast<size_t>(across) * high * deep);

  for (uint32_t x = 0u; x != across; ++x) {
    for (uint32_t y = 0u; y != high; ++y) {
      for (uint32_t z = 0u; z != deep; ++z) {
        children.push_back(part(
          glm::dvec3(0.25 * x, 0.25 * y, 0.25 * z),
          glm::dvec3(0.125),
          2000.0,
          (x * high + y) * deep + z
        ));
      }
    }
  }

  return talus::dynamics::addCompoundBody(world.bodies, children, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.6, 0.0, true);
}

std::vector<uint32_t> gridBlock(uint32_t high, uint32_t deep, const glm::uvec3& from, const glm::uvec3& size) {
  std::vector<uint32_t> keys;

  for (uint32_t x = from.x; x != from.x + size.x; ++x) {
    for (uint32_t y = from.y; y != from.y + size.y; ++y) {
      for (uint32_t z = from.z; z != from.z + size.z; ++z) {
        keys.push_back((x * high + y) * deep + z);
      }
    }
  }

  return keys;
}

}

TEST_CASE("compound edit cost on large anchored bodies", "[.][compoundeditbench]") {
  struct Size {
    glm::uvec3 grid;
    double limit = 0.0;
  };

  const std::array<Size, 2> sizes{Size{glm::uvec3(20u, 20u, 10u), 0.2}, Size{glm::uvec3(25u, 20u, 20u), 0.5}};

  for (const Size& size : sizes) {
    const uint32_t count = size.grid.x * size.grid.y * size.grid.z;
    const glm::uvec3 middle = size.grid / 2u;
    const std::array<std::vector<uint32_t>, 2> removals{
      gridBlock(size.grid.y, size.grid.z, middle, glm::uvec3(1u)),
      gridBlock(size.grid.y, size.grid.z, middle - glm::uvec3(2u, 2u, 2u), glm::uvec3(5u, 5u, 4u))
    };

    for (const std::vector<uint32_t>& keys : removals) {
      double best = std::numeric_limits<double>::infinity();

      for (uint32_t run = 0u; run != 5u; ++run) {
        talus::dynamics::World world;
        const uint32_t index = addGridBuilding(world, size.grid.x, size.grid.y, size.grid.z);
        const uint32_t id = world.bodies.rigids[index].id;
        const auto before = std::chrono::steady_clock::now();

        REQUIRE(talus::dynamics::removeCompoundChildren(world, id, keys));
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
        REQUIRE(world.bodies.rigids[index].compound->children.size() == count - keys.size());
      }

      std::printf("compoundeditbench children %u removed %zu ms %.4f limit %.1f\n", count, keys.size(), best, size.limit);
      CHECK(best < size.limit);
    }

    double best = std::numeric_limits<double>::infinity();

    for (uint32_t run = 0u; run != 5u; ++run) {
      talus::dynamics::World world;
      const uint32_t index = addGridBuilding(world, size.grid.x, size.grid.y, size.grid.z);
      const uint32_t id = world.bodies.rigids[index].id;
      const std::vector<uint32_t>& piece = removals[1];
      std::vector<uint32_t> rest;

      for (const talus::dynamics::CompoundChild& child : world.bodies.rigids[index].compound->children) {
        if (!std::binary_search(piece.begin(), piece.end(), child.key)) {
          rest.push_back(child.key);
        }
      }

      const std::vector<std::vector<uint32_t>> groups{rest, piece};
      const auto before = std::chrono::steady_clock::now();

      REQUIRE(talus::dynamics::splitCompound(world, id, groups).size() == 1u);
      best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
    }

    std::printf("compoundeditbench children %u split off 100 ms %.4f (no limit)\n", count, best);
  }
}

namespace {

std::vector<talus::dynamics::CompoundChild> filling(
  const talus::dynamics::CompoundChild& child,
  const glm::uvec3& cells,
  uint32_t firstKey
) {
  std::vector<talus::dynamics::CompoundChild> parts;
  const glm::dvec3 half = child.half / glm::dvec3(cells);
  const glm::dmat3 turn = glm::mat3_cast(child.rotation);

  for (uint32_t k = 0u; k != cells.z; ++k) {
    for (uint32_t j = 0u; j != cells.y; ++j) {
      for (uint32_t i = 0u; i != cells.x; ++i) {
        const glm::dvec3 offset = half * (2.0 * glm::dvec3(i, j, k) + glm::dvec3(1.0)) - child.half;

        parts.push_back(part(
          child.centre + turn * offset,
          half,
          child.density,
          firstKey + i + cells.x * (j + cells.y * k),
          child.rotation
        ));
      }
    }
  }

  return parts;
}

std::vector<talus::dynamics::CompoundChild> withParts(
  const std::vector<talus::dynamics::CompoundChild>& children,
  uint32_t key,
  const std::vector<talus::dynamics::CompoundChild>& parts
) {
  std::vector<talus::dynamics::CompoundChild> result;

  for (const talus::dynamics::CompoundChild& child : children) {
    if (child.key != key) {
      result.push_back(child);
    }
  }

  result.insert(result.end(), parts.begin(), parts.end());

  return result;
}

talus::dynamics::CompoundChild childWithKey(const talus::dynamics::RigidBody& body, uint32_t key) {
  const std::optional<uint32_t> index = talus::dynamics::findChild(*body.compound, key);

  REQUIRE(index.has_value());

  return body.compound->children[*index];
}

}

TEST_CASE("replacing anchored children by boxes that fill them edits the shape in place into exactly what a rebuild gives", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    brickWall(8u, 6u, 5u),
    glm::dvec3(1.0, 0.5, -2.0),
    glm::angleAxis(0.2, glm::dvec3(0.0, 1.0, 0.0)),
    glm::dvec3(0.0),
    0.5,
    0.0,
    true
  );
  const uint32_t id = world.bodies.rigids[index].id;
  const std::vector<talus::dynamics::CompoundChild> children = world.bodies.rigids[index].compound->children;
  const talus::dynamics::CompoundShape* shape = world.bodies.rigids[index].compound.get();
  const uint64_t revision = shape->revision;
  const talus::dynamics::CompoundChild first = children[5];
  const talus::dynamics::CompoundChild second = children[100];
  const std::vector<talus::dynamics::CompoundChild> firstParts = filling(first, glm::uvec3(2u), 2000u);
  const std::vector<talus::dynamics::CompoundChild> secondParts = filling(second, glm::uvec3(1u, 3u, 3u), 2100u);
  const std::vector<talus::dynamics::ChildReplacement> swaps{
    talus::dynamics::ChildReplacement{first.key, firstParts},
    talus::dynamics::ChildReplacement{second.key, secondParts}
  };

  REQUIRE(talus::dynamics::replaceCompoundChildren(world, id, swaps));

  const std::vector<talus::dynamics::CompoundChild> expected =
    withParts(withParts(children, first.key, firstParts), second.key, secondParts);
  const talus::dynamics::CompoundShape& edited = *world.bodies.rigids[index].compound;

  CHECK(world.bodies.rigids[index].compound.get() == shape);
  CHECK(edited.revision == revision + 1u);
  CHECK(edited.children.size() == children.size() + 15u);
  CHECK(shapeMismatches(edited, *talus::dynamics::makeCompoundShape(expected)) == 0u);
  CHECK(queryMismatches(edited, *talus::dynamics::makeCompoundShape(expected)) == 0u);
}

TEST_CASE("replacing a child of a moving, spinning compound keeps its mass, centre and inertia to 1e-12 and its frame, momentum and kinetic energy exactly", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    lShape(),
    glm::dvec3(0.5, 2.0, -0.3),
    glm::angleAxis(0.3, glm::dvec3(0.0, 0.0, 1.0)),
    glm::dvec3(1.5, -0.5, 0.25),
    0.5,
    0.0,
    false
  );

  world.bodies.rigids[index].angularVelocity = glm::dvec3(0.4, -1.2, 2.0);

  const talus::dynamics::RigidBody before = world.bodies.rigids[index];
  const talus::dynamics::CompoundMass massBefore = talus::dynamics::compoundMass(before.compound->children);
  const talus::dynamics::CompoundChild bar = childWithKey(before, 1u);
  const std::vector<talus::dynamics::CompoundChild> parts = filling(bar, glm::uvec3(2u, 1u, 2u), 50u);
  const std::vector<talus::dynamics::ChildReplacement> swap{talus::dynamics::ChildReplacement{1u, parts}};

  REQUIRE(talus::dynamics::replaceCompoundChildren(world, before.id, swap));

  const talus::dynamics::RigidBody& after = world.bodies.rigids[index];
  const talus::dynamics::CompoundMass massAfter = talus::dynamics::compoundMass(after.compound->children);
  const double trace = massBefore.inertia[0][0] + massBefore.inertia[1][1] + massBefore.inertia[2][2];

  CHECK(after.mass == before.mass);
  CHECK(after.moment == before.moment);
  CHECK(after.position == before.position);
  CHECK(after.rotation == before.rotation);
  CHECK(after.velocity == before.velocity);
  CHECK(after.angularVelocity == before.angularVelocity);
  CHECK(std::abs(massAfter.mass - massBefore.mass) <= 1.0e-12 * massBefore.mass);
  CHECK(glm::length(massAfter.centre - massBefore.centre) <= 1.0e-12 * after.compound->reach);
  CHECK(largestEntry(massAfter.inertia - massBefore.inertia) <= 1.0e-12 * trace);
  CHECK(after.compound->children.size() == before.compound->children.size() + 3u);
  CHECK(!talus::dynamics::findChild(*after.compound, 1u).has_value());

  for (const talus::dynamics::CompoundChild& piece : parts) {
    CHECK(talus::dynamics::findChild(*after.compound, piece.key).has_value());
  }
}

TEST_CASE("a replacement whose boxes do not fill their child, change its density or rotation, or reuse a key is refused and changes nothing", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 3u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0));
  const uint32_t id = world.bodies.rigids[slab].id;
  const talus::dynamics::CompoundShape* shape = world.bodies.rigids[slab].compound.get();
  const talus::dynamics::CompoundChild middle = childWithKey(world.bodies.rigids[slab], 1u);
  const std::vector<talus::dynamics::CompoundChild> whole = filling(middle, glm::uvec3(2u), 10u);
  std::vector<std::vector<talus::dynamics::CompoundChild>> wrong;

  wrong.emplace_back(whole.begin(), whole.end() - 1);
  wrong.push_back(whole);
  wrong.back()[0].centre.x += 0.01;
  wrong.push_back(whole);
  wrong.back()[3].density *= 2.0;
  wrong.push_back(whole);
  wrong.back()[2].rotation = glm::angleAxis(0.1, glm::dvec3(0.0, 1.0, 0.0));
  wrong.push_back(whole);
  wrong.back()[4].key = 0u;
  wrong.push_back(whole);
  wrong.back()[5].key = wrong.back()[6].key;
  wrong.push_back(whole);
  wrong.back()[1].half.y = 0.0;
  wrong.emplace_back();

  for (size_t index = 0u; index != wrong.size(); ++index) {
    INFO("case " << index);

    const std::vector<talus::dynamics::ChildReplacement> swap{talus::dynamics::ChildReplacement{1u, wrong[index]}};

    CHECK(!talus::dynamics::replaceCompoundChildren(world, id, swap));
    CHECK(world.bodies.rigids[slab].compound.get() == shape);
    CHECK(world.bodies.rigids[slab].compound->revision == 0u);
    CHECK(world.bodies.rigids[slab].compound->children.size() == 3u);
  }

  const std::vector<talus::dynamics::ChildReplacement> unknown{talus::dynamics::ChildReplacement{7u, whole}};
  const std::vector<talus::dynamics::ChildReplacement> twice{
    talus::dynamics::ChildReplacement{1u, whole},
    talus::dynamics::ChildReplacement{1u, filling(middle, glm::uvec3(2u), 30u)}
  };
  const std::vector<talus::dynamics::ChildReplacement> none;
  const std::vector<talus::dynamics::ChildReplacement> elsewhere{talus::dynamics::ChildReplacement{1u, whole}};

  CHECK(!talus::dynamics::replaceCompoundChildren(world, id, unknown));
  CHECK(!talus::dynamics::replaceCompoundChildren(world, id, twice));
  CHECK(!talus::dynamics::replaceCompoundChildren(world, id, none));
  CHECK(!talus::dynamics::replaceCompoundChildren(world, 999u, elsewhere));
  CHECK(world.bodies.rigids[slab].compound->revision == 0u);
}

TEST_CASE("every edit of a compound shape raises its revision by one, and a shape rebuilt for a moving body carries it on", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t still = addSlab(world, 4u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  const uint32_t moving = addEightCube(world, 0.1, 1000.0, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0));
  const uint32_t stillId = world.bodies.rigids[still].id;
  const uint32_t movingId = world.bodies.rigids[moving].id;
  const auto revisionOf = [&world](uint32_t index) {
    return world.bodies.rigids[index].compound->revision;
  };

  CHECK(revisionOf(still) == 0u);
  CHECK(revisionOf(moving) == 0u);
  REQUIRE(talus::dynamics::removeCompoundChildren(world, stillId, std::vector<uint32_t>{3u}));
  CHECK(revisionOf(still) == 1u);
  REQUIRE(talus::dynamics::splitCompound(world, stillId, std::vector<std::vector<uint32_t>>{{0u, 1u}, {2u}}).size() == 1u);
  CHECK(revisionOf(still) == 2u);
  CHECK(world.bodies.rigids.back().compound->revision == 0u);
  REQUIRE(talus::dynamics::removeCompoundChildren(world, movingId, std::vector<uint32_t>{7u}));
  CHECK(revisionOf(moving) == 1u);
  REQUIRE(talus::dynamics::splitCompound(world, movingId, std::vector<std::vector<uint32_t>>{{0u, 1u, 2u, 3u}, {4u, 5u, 6u}}).size() == 1u);
  CHECK(revisionOf(moving) == 2u);
  CHECK(world.bodies.rigids.back().compound->revision == 0u);

  const talus::dynamics::CompoundChild cell = childWithKey(world.bodies.rigids[still], 0u);
  const std::vector<talus::dynamics::ChildReplacement> swap{
    talus::dynamics::ChildReplacement{0u, filling(cell, glm::uvec3(2u), 100u)}
  };

  REQUIRE(talus::dynamics::replaceCompoundChildren(world, stillId, swap));
  CHECK(revisionOf(still) == 3u);
}

TEST_CASE("a crate resting on a child that is replaced keeps resting on the boxes that replace it", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 3u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  const uint32_t crate = addEightCube(world, 0.05, 1000.0, glm::dvec3(0.0, 0.0505, 0.0), glm::dvec3(0.0));

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(world.bodies.rigids[crate].asleep);

  const glm::dvec3 restingAt = world.bodies.rigids[crate].position;
  const talus::dynamics::CompoundChild middle = childWithKey(world.bodies.rigids[slab], 1u);
  const std::vector<talus::dynamics::ChildReplacement> swap{
    talus::dynamics::ChildReplacement{1u, filling(middle, glm::uvec3(2u), 10u)}
  };

  REQUIRE(talus::dynamics::replaceCompoundChildren(world, world.bodies.rigids[slab].id, swap));

  double drift = 0.0;
  double peak = 0.0;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    drift = std::max(drift, glm::length(world.bodies.rigids[crate].position - restingAt));
    peak = std::max(peak, glm::length(world.bodies.rigids[crate].velocity));
  }

  const uint32_t slot = world.manifolds.find(talus::dynamics::pairKey(
    world.bodies,
    {talus::dynamics::Side::Rigid, slab},
    {talus::dynamics::Side::Rigid, crate}
  ));

  CHECK(drift < 0.01);
  CHECK(peak < 0.05);
  REQUIRE(slot != talus::dynamics::noSlot);
  REQUIRE(!world.manifolds.at(slot).manifoldPoints.empty());

  for (const talus::dynamics::CachedManifoldPoint& point : world.manifolds.at(slot).manifoldPoints) {
    CHECK(point.firstChild >= 10u);
  }
}

TEST_CASE("a sphere at 300 m/s does not pass through a zero-mass compound wall 0.2 m thick", "[dynamics][compound]") {
  const talus::field::SampledSdf ball = sphereShape(0.05);
  talus::dynamics::World world;
  const std::array<talus::dynamics::CompoundChild, 1> chunk{part(glm::dvec3(0.0), glm::dvec3(0.1, 0.25, 0.25), 2450.0, 0u)};

  world.parameters.gravity = glm::dvec3(0.0);
  (void)talus::dynamics::addCompoundBody(world.bodies, chunk, glm::dvec3(0.0, 1.0, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t shot = talus::dynamics::addRigidSphere(
    world.bodies, ball, 0.05, 4.1, glm::dvec3(-3.0, 1.0, 0.0), glm::dvec3(300.0, 0.0, 0.0), 0.5, 0.0
  );
  double farthest = -3.0;

  for (uint32_t frame = 0u; frame != 30u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    farthest = std::max(farthest, world.bodies.rigids[shot].position.x);
  }

  CHECK(farthest < 0.1);
  CHECK(world.bodies.rigids[shot].position.x < -0.1);
}

TEST_CASE("a fast box dropped onto a thin zero-mass compound plate lands and comes to rest", "[dynamics][compound]") {
  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf crate = boxShape(half);
  const std::array<talus::dynamics::CompoundChild, 1> plate{part(glm::dvec3(0.0), glm::dvec3(0.5, 0.02, 0.5), 2450.0, 0u)};
  talus::dynamics::World world;

  (void)talus::dynamics::addCompoundBody(world.bodies, plate, glm::dvec3(0.0, -0.02, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t box = talus::dynamics::addRigidBox(
    world.bodies, crate, half, 1.5, glm::dvec3(0.0, 0.22, 0.0), glm::dvec3(0.0, -20.0, 0.0), 0.5, 3u
  );
  double lowestTop = 1.0;
  glm::dvec3 settled(0.0);
  double drift = 0.0;
  double peak = 0.0;

  for (uint32_t frame = 0u; frame != 360u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& body = world.bodies.rigids[box];
    const glm::dmat3 turn = glm::mat3_cast(body.rotation);
    const double reach = half.y * (std::abs(turn[0][1]) + std::abs(turn[1][1]) + std::abs(turn[2][1]));

    lowestTop = std::min(lowestTop, body.position.y + reach);

    if (frame + 1u == 60u) {
      settled = body.position;
    } else if (frame + 1u > 60u) {
      drift = std::max(drift, glm::length(body.position - settled));
      peak = std::max(peak, glm::length(body.velocity));
    }
  }

  CHECK(lowestTop >= -0.02);
  CHECK(drift < 0.01);
  CHECK(peak < 0.05);
}

TEST_CASE("a fast compound piece dropped onto a thin zero-mass compound plate stays on it", "[dynamics][compound]") {
  const std::array<talus::dynamics::CompoundChild, 1> plate{part(glm::dvec3(0.0), glm::dvec3(0.5, 0.02, 0.5), 2450.0, 0u)};
  talus::dynamics::World world;

  (void)talus::dynamics::addCompoundBody(world.bodies, plate, glm::dvec3(0.0, -0.02, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t piece = talus::dynamics::addCompoundBody(
    world.bodies, cubeOfEight(0.025, 1000.0, 10u), glm::dvec3(0.0, 0.22, 0.0), noTurn, glm::dvec3(0.0, -20.0, 0.0), 0.5, 0.0, false
  );
  double lowest = 1.0;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    lowest = std::min(lowest, world.bodies.rigids[piece].position.y);
  }

  CHECK(lowest >= 0.05 - 0.02);
  CHECK(world.bodies.rigids[piece].position.y >= 0.05 - 0.01);
}

TEST_CASE("a fast spinning bar does not stick through a thin chunked zero-mass wall", "[dynamics][compound]") {
  const glm::dvec3 barHalf(0.25, 0.025, 0.025);
  const talus::field::SampledSdf barField = boxShape(barHalf);
  const double thickness = 0.02;
  const glm::dvec3 start(-1.2, 0.013, 0.021);
  const glm::dvec3 velocity(10.0, 0.0, 0.0);
  talus::dynamics::World world;
  std::vector<talus::dynamics::CompoundChild> wall;

  world.parameters.gravity = glm::dvec3(0.0);

  for (uint32_t j = 0u; j != 30u; ++j) {
    for (uint32_t k = 0u; k != 30u; ++k) {
      wall.push_back(part(glm::dvec3(0.0, -2.9 + 0.2 * j, -2.9 + 0.2 * k), glm::dvec3(0.5 * thickness, 0.1, 0.1), 2450.0, 30u * j + k));
    }
  }

  (void)talus::dynamics::addCompoundBody(world.bodies, wall, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t bar = talus::dynamics::addRigidBox(world.bodies, barField, barHalf, 2.0, start, velocity, 0.5, 3u);

  REQUIRE(talus::dynamics::setRigidPose(
    world, world.bodies.rigids[bar].id, start, glm::angleAxis(150.0 * std::numbers::pi / 180.0, glm::dvec3(0.0, 1.0, 0.0))
  ));
  world.bodies.rigids[bar].velocity = velocity;
  world.bodies.rigids[bar].previousVelocity = velocity;
  world.bodies.rigids[bar].angularVelocity = glm::dvec3(0.0, 47.0, 0.0);

  double farthest = -1.0e9;

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::dynamics::RigidBody& body = world.bodies.rigids[bar];

    for (const glm::dvec3& corner : cornersOf(body.position, body.rotation, part(glm::dvec3(0.0), barHalf, 0.0, 0u))) {
      farthest = std::max(farthest, corner.x);
    }
  }

  CHECK(farthest <= 0.5 * thickness + 0.02);
  CHECK(world.bodies.rigids[bar].position.x < -0.5 * thickness);
}

namespace {

constexpr double SHOT_RADIUS = 0.05;
constexpr double SHOT_MASS = 4.11;
constexpr double BLOCK_MASS = 231.25;

double angleFrom(const glm::dvec3& velocity, const glm::dvec3& direction) {
  return std::atan2(glm::length(glm::cross(velocity, direction)), glm::dot(velocity, direction));
}

uint32_t framesToCross(double distance, double speed) {
  return static_cast<uint32_t>(std::ceil(60.0 * distance / speed)) + 2u;
}

}

TEST_CASE("a sphere at 300 or 30 m/s that passes 5 to 80 cm above a resting box keeps its direction and leaves the box unmoved", "[dynamics][compound][ghostcontacts]") {
  const glm::dvec3 half(0.25);
  const talus::field::SampledSdf ball = sphereShape(SHOT_RADIUS);
  const talus::field::SampledSdf crate = boxShape(half);

  for (const double speed : {300.0, 30.0}) {
    for (const double clearance : {0.8, 0.2, 0.05}) {
      talus::dynamics::World world;

      world.parameters.gravity = glm::dvec3(0.0);

      const uint32_t box = talus::dynamics::addRigidBox(world.bodies, crate, half, BLOCK_MASS, glm::dvec3(0.0), glm::dvec3(0.0), 0.6, 3u);
      const uint32_t shot = talus::dynamics::addRigidSphere(
        world.bodies, ball, SHOT_RADIUS, SHOT_MASS, glm::dvec3(-3.0, half.y + SHOT_RADIUS + clearance, 0.0), glm::dvec3(speed, 0.0, 0.0), 0.5, 0.0
      );

      for (uint32_t frame = 0u; frame != framesToCross(6.0, speed); ++frame) {
        REQUIRE(!talus::dynamics::step(world).nonFinite);
      }

      const talus::dynamics::RigidBody& sphere = world.bodies.rigids[shot];
      const talus::dynamics::RigidBody& block = world.bodies.rigids[box];

      INFO("speed " << speed << " m/s, clearance " << clearance << " m");
      CHECK(sphere.position.x > 3.0);
      CHECK(angleFrom(sphere.velocity, glm::dvec3(1.0, 0.0, 0.0)) <= 1.0e-6);
      CHECK(glm::length(block.position) <= 1.0e-9);
      CHECK(glm::length(block.velocity) <= 1.0e-9);
      CHECK(glm::length(block.angularVelocity) <= 1.0e-9);
    }
  }
}

TEST_CASE("a sphere at 300 or 30 m/s aimed at a resting box hits it and the two move on with the shot's momentum", "[dynamics][compound][ghostcontacts]") {
  const glm::dvec3 half(0.25);
  const talus::field::SampledSdf ball = sphereShape(SHOT_RADIUS);
  const talus::field::SampledSdf crate = boxShape(half);

  for (const double speed : {300.0, 30.0}) {
    talus::dynamics::World world;

    world.parameters.gravity = glm::dvec3(0.0);

    const uint32_t box = talus::dynamics::addRigidBox(world.bodies, crate, half, BLOCK_MASS, glm::dvec3(0.0), glm::dvec3(0.0), 0.6, 3u);
    const uint32_t shot = talus::dynamics::addRigidSphere(
      world.bodies, ball, SHOT_RADIUS, SHOT_MASS, glm::dvec3(-3.0, 0.0, 0.0), glm::dvec3(speed, 0.0, 0.0), 0.5, 0.0
    );
    double ahead = -1.0e9;

    for (uint32_t frame = 0u; frame != framesToCross(6.0, speed); ++frame) {
      REQUIRE(!talus::dynamics::step(world).nonFinite);
      ahead = std::max(ahead, world.bodies.rigids[shot].position.x - world.bodies.rigids[box].position.x);
    }

    INFO("speed " << speed << " m/s");
    CHECK(ahead < 0.0);
    CHECK(world.bodies.rigids[box].velocity.x >= 0.99 * SHOT_MASS * speed / (SHOT_MASS + BLOCK_MASS));
  }
}

TEST_CASE("a sphere at 300 or 30 m/s that flies through the doorway of a zero-mass compound keeps its direction", "[dynamics][compound][ghostcontacts]") {
  const talus::field::SampledSdf ball = sphereShape(SHOT_RADIUS);
  const std::array<talus::dynamics::CompoundChild, 3> gate{
    part(glm::dvec3(0.0, 0.0, -0.75), glm::dvec3(0.25, 1.0, 0.25), 2400.0, 0u),
    part(glm::dvec3(0.0, 0.0, 0.75), glm::dvec3(0.25, 1.0, 0.25), 2400.0, 1u),
    part(glm::dvec3(0.0, 1.25, 0.0), glm::dvec3(0.25, 0.25, 1.0), 2400.0, 2u)
  };

  for (const double speed : {300.0, 30.0}) {
    talus::dynamics::World world;

    world.parameters.gravity = glm::dvec3(0.0);
    (void)talus::dynamics::addCompoundBody(world.bodies, gate, glm::dvec3(0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

    const uint32_t shot = talus::dynamics::addRigidSphere(
      world.bodies, ball, SHOT_RADIUS, SHOT_MASS, glm::dvec3(-3.0, 0.0, 0.0), glm::dvec3(speed, 0.0, 0.0), 0.5, 0.0
    );

    for (uint32_t frame = 0u; frame != framesToCross(6.0, speed); ++frame) {
      REQUIRE(!talus::dynamics::step(world).nonFinite);
    }

    const talus::dynamics::RigidBody& sphere = world.bodies.rigids[shot];

    INFO("speed " << speed << " m/s");
    CHECK(sphere.position.x > 3.0);
    CHECK(angleFrom(sphere.velocity, glm::dvec3(1.0, 0.0, 0.0)) <= 1.0e-6);
  }
}

TEST_CASE("a thin tile dropped at 10 m/s onto a glass pane lying on a zero-mass floor stays on the pane", "[dynamics][compound][ghostcontacts]") {
  const std::array<talus::dynamics::CompoundChild, 1> ground{part(glm::dvec3(0.0), glm::dvec3(2.0, 0.1, 2.0), 2400.0, 0u)};
  const std::array<talus::dynamics::CompoundChild, 1> pane{part(glm::dvec3(0.0), glm::dvec3(0.5, 0.02, 0.5), 2500.0, 0u)};
  const std::array<talus::dynamics::CompoundChild, 1> tile{part(glm::dvec3(0.0), glm::dvec3(0.08, 0.01, 0.08), 2500.0, 0u)};
  talus::dynamics::World world;

  (void)talus::dynamics::addCompoundBody(world.bodies, ground, glm::dvec3(0.0, -0.1, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);

  const uint32_t glass = talus::dynamics::addCompoundBody(world.bodies, pane, glm::dvec3(0.0, 0.02, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false);
  const uint32_t piece = talus::dynamics::addCompoundBody(world.bodies, tile, glm::dvec3(0.0, 0.34, 0.0), noTurn, glm::dvec3(0.0, -10.0, 0.0), 0.5, 0.0, false);
  double lowest = 1.0;

  for (uint32_t frame = 0u; frame != 90u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    lowest = std::min(lowest, world.bodies.rigids[piece].position.y - (world.bodies.rigids[glass].position.y + 0.02));
  }

  CHECK(lowest >= 0.0);
}


TEST_CASE("pieces asleep on a zero-mass slab, and a frozen one, leave the pair update until one of them moves again", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 4u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  std::vector<uint32_t> cubes;

  for (uint32_t index = 0u; index != 4u; ++index) {
    cubes.push_back(addEightCube(world, 0.05, 1000.0, glm::dvec3(-0.3 + 0.2 * index, 0.0505, 0.0), glm::dvec3(0.0)));
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  for (const uint32_t cube : cubes) {
    REQUIRE(world.bodies.rigids[cube].asleep);
  }

  talus::dynamics::RigidBody& frozen = world.bodies.rigids[cubes[3]];

  frozen.mass = 0.0;
  frozen.moment = glm::dvec3(0.0);
  REQUIRE(talus::dynamics::setVelocity(world, frozen.id, glm::dvec3(0.0)));
  REQUIRE(talus::dynamics::setVelocity(world, world.bodies.rigids[slab].id, glm::dvec3(0.0)));
  world.parameters.timePairs = true;

  for (uint32_t frame = 0u; frame != 3u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::StepReport resting = talus::dynamics::step(world);

  REQUIRE(!world.bodies.rigids[slab].asleep);
  REQUIRE(!world.bodies.rigids[cubes[3]].asleep);
  CHECK(resting.profile.pairs.tested == 0u);
  REQUIRE(talus::dynamics::setVelocity(world, world.bodies.rigids[cubes[0]].id, glm::dvec3(0.0, 0.5, 0.0)));

  const talus::dynamics::StepReport woken = talus::dynamics::step(world);

  CHECK(woken.profile.pairs.tested >= 1u);
  CHECK(!world.bodies.rigids[cubes[0]].asleep);
  CHECK(world.bodies.rigids[slab].mass == 0.0);
}

TEST_CASE("a pose written directly on an awake zero-mass slab refreshes its contacts with the pieces asleep on it in the same step", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t slab = addSlab(world, 4u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -0.05, 0.0));
  std::vector<uint32_t> cubes;

  for (uint32_t index = 0u; index != 4u; ++index) {
    cubes.push_back(addEightCube(world, 0.05, 1000.0, glm::dvec3(-0.3 + 0.2 * index, 0.0505, 0.0), glm::dvec3(0.0)));
  }

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(talus::dynamics::setVelocity(world, world.bodies.rigids[slab].id, glm::dvec3(0.0)));

  for (uint32_t frame = 0u; frame != 3u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const auto nearest = [&](uint32_t cube) {
    const uint32_t slabId = world.bodies.rigids[slab].id;
    double least = std::numeric_limits<double>::infinity();

    for (const uint32_t slot : world.manifolds.pairsOf(world.bodies.rigids[cube].id)) {
      const talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

      if (pair.firstId != slabId && pair.secondId != slabId) {
        continue;
      }

      for (const talus::dynamics::Contact& contact : world.manifolds.records(slot)) {
        least = std::min(least, contact.gap);
      }
    }

    return least;
  };
  std::vector<double> before;

  for (const uint32_t cube : cubes) {
    REQUIRE(world.bodies.rigids[cube].asleep);
    before.push_back(nearest(cube));
    REQUIRE(std::isfinite(before.back()));
  }

  world.bodies.rigids[slab].position.y -= 0.003;
  REQUIRE(!talus::dynamics::step(world).nonFinite);

  for (uint32_t index = 0u; index != cubes.size(); ++index) {
    REQUIRE(world.bodies.rigids[cubes[index]].asleep);
    CHECK(std::abs(nearest(cubes[index]) - before[index] - 0.003) <= 0.0005);
  }
}

TEST_CASE("a box frozen to zero mass and given its mass back just before a ball reaches it moves on with the ball's momentum", "[dynamics][compound]") {
  const glm::dvec3 half(0.25);
  const talus::field::SampledSdf ball = sphereShape(0.05);
  const talus::field::SampledSdf crate = boxShape(half);
  talus::dynamics::World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t box = talus::dynamics::addRigidBox(world.bodies, crate, half, 10.0, glm::dvec3(0.0), glm::dvec3(0.0), 0.6, 3u);
  const uint32_t shot = talus::dynamics::addRigidSphere(world.bodies, ball, 0.05, 1.0, glm::dvec3(-2.0, 0.0, 0.0), glm::dvec3(5.0, 0.0, 0.0), 0.5, 0.0);
  const double mass = world.bodies.rigids[box].mass;
  const glm::dvec3 moment = world.bodies.rigids[box].moment;
  bool thawed = false;

  world.bodies.rigids[box].mass = 0.0;
  world.bodies.rigids[box].moment = glm::dvec3(0.0);

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    const double gap = (world.bodies.rigids[box].position.x - half.x) - (world.bodies.rigids[shot].position.x + 0.05);

    if (!thawed && gap < 2.0 * 5.0 / 60.0) {
      REQUIRE(gap > 5.0 / 60.0);
      world.bodies.rigids[box].mass = mass;
      world.bodies.rigids[box].moment = moment;
      thawed = true;
    }

    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const talus::dynamics::RigidBody& moved = world.bodies.rigids[box];
  const talus::dynamics::RigidBody& hit = world.bodies.rigids[shot];
  const glm::dvec3 momentum = moved.mass * moved.velocity + hit.mass * hit.velocity;

  REQUIRE(thawed);
  CHECK(std::abs(moved.velocity.x - 5.0 / 11.0) <= 0.01 * 5.0 / 11.0);
  CHECK(std::abs(hit.velocity.x - 5.0 / 11.0) <= 0.01 * 5.0 / 11.0);
  CHECK(glm::length(momentum - glm::dvec3(5.0, 0.0, 0.0)) <= 1.0e-6 * 5.0);
}

namespace {

struct TrainRun {
  double deepest = 0.0;
  bool through = false;
  std::vector<double> trace;
};

TrainRun flyTrain(bool boxes, uint32_t count, double speed, double gap, double above, uint32_t ballast = 0u, uint32_t workers = 1u) {
  const glm::dvec3 half(0.25, 0.15, 0.125);
  const talus::field::SampledSdf crate = boxShape(half);
  const std::array<talus::dynamics::CompoundChild, 1> ground{part(glm::dvec3(0.0), glm::dvec3(4.0, 0.5, 4.0), 2400.0, 0u)};
  const std::array<talus::dynamics::CompoundChild, 1> slab{part(glm::dvec3(0.0), half, 2400.0, 0u)};
  talus::dynamics::World world;
  std::vector<uint32_t> train;
  TrainRun run;

  world.parameters.gravity = glm::dvec3(0.0);
  world.workerCount = workers;
  (void)talus::dynamics::addCompoundBody(world.bodies, ground, glm::dvec3(0.0, -0.5, 0.0), noTurn, glm::dvec3(0.0), 0.6, 0.0, true);

  for (uint32_t index = 0u; index != count; ++index) {
    const glm::dvec3 position(0.0, half.y + above + static_cast<double>(index) * (2.0 * half.y + gap), 0.0);
    const glm::dvec3 velocity(0.0, -speed, 0.0);

    train.push_back(
      boxes ? talus::dynamics::addRigidBox(world.bodies, crate, half, 2400.0 * 8.0 * half.x * half.y * half.z, position, velocity, 0.6, 3u)
            : talus::dynamics::addCompoundBody(world.bodies, slab, position, noTurn, velocity, 0.6, 0.0, false)
    );
  }

  for (uint32_t index = 0u; index != ballast; ++index) {
    (void)talus::dynamics::addCompoundBody(world.bodies, slab, glm::dvec3(100.0 + 2.0 * index, 0.0, 0.0), noTurn, glm::dvec3(0.0), 0.6, 0.0, true);
  }

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    for (const uint32_t slot : train) {
      const talus::dynamics::RigidBody& body = world.bodies.rigids[slot];

      for (const glm::dvec3 value : {body.position, body.velocity, body.angularVelocity}) {
        run.trace.insert(run.trace.end(), {value.x, value.y, value.z});
      }

      run.trace.insert(run.trace.end(), {body.rotation.w, body.rotation.x, body.rotation.y, body.rotation.z});
    }

    for (uint32_t index = 1u; index != count; ++index) {
      const double lower = world.bodies.rigids[train[index - 1u]].position.y;
      const double upper = world.bodies.rigids[train[index]].position.y;

      run.deepest = std::max(run.deepest, (lower + half.y) - (upper - half.y));
      run.through = run.through || upper < lower;
    }
  }

  return run;
}

}

TEST_CASE("two pieces flying together at 5 to 20 m/s, or three at 5 to 10 m/s, onto a zero-mass floor stop on each other when the front one stops", "[dynamics][compound][ghostcontacts]") {
  for (const bool boxes : {false, true}) {
    for (const uint32_t count : {2u, 3u}) {
      for (const double speed : {5.0, 10.0, 20.0}) {
        for (const double gap : {0.03, 0.1, 0.2}) {
          for (const double above : {0.01, 0.1}) {
            if (count == 3u && speed > 10.0) {
              continue;
            }

            const TrainRun run = flyTrain(boxes, count, speed, gap, above);

            INFO((boxes ? "boxes" : "compounds") << ", " << count << " pieces, " << speed << " m/s, gap " << gap << " m, front " << above << " m above the floor");
            CHECK(!run.through);
            CHECK(run.deepest <= 0.02);
          }
        }
      }
    }
  }
}

TEST_CASE("landing trains keep identical trajectories with a sparse active-pair walk and multiple workers", "[dynamics][compound][ghostcontacts]") {
  for (const bool boxes : {false, true}) {
    for (const uint32_t count : {2u, 3u}) {
      const TrainRun full = flyTrain(boxes, count, 10.0, 0.1, 0.1);

      for (const uint32_t workers : {1u, 8u}) {
        const TrainRun sparse = flyTrain(boxes, count, 10.0, 0.1, 0.1, 64u, workers);

        INFO("boxes " << boxes << ", pieces " << count << ", workers " << workers);
        CHECK(sparse.trace == full.trace);
        CHECK(!sparse.through);
        CHECK(sparse.deepest <= 0.02);
      }
    }
  }
}

namespace {

uint32_t addRayWall(talus::dynamics::World& world) {
  const std::array<talus::dynamics::CompoundChild, 3> children{
    part(glm::dvec3(0.0, 0.0, -0.25), glm::dvec3(0.125), 1000.0, 0u),
    part(glm::dvec3(0.0, 0.0, 0.0), glm::dvec3(0.125), 1000.0, 1u),
    part(glm::dvec3(0.0, 0.0, 0.25), glm::dvec3(0.125), 1000.0, 2u)
  };

  return talus::dynamics::addCompoundBody(world.bodies, children, glm::dvec3(1.0, 0.0, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, true);
}

}

TEST_CASE("a ray stops at the nearest compound child or zero-mass box, by distance, then body id, then child key", "[dynamics][compound]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.125));
  talus::dynamics::World world;
  const uint32_t wallIndex = addRayWall(world);
  const uint32_t boxIndex = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.125), 0.0, glm::dvec3(2.0, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);
  const uint32_t movingIndex = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.125), 1.0, glm::dvec3(0.5, 0.0, 0.0), glm::dvec3(0.0), 0.5, 3u);
  const uint32_t wall = world.bodies.rigids[wallIndex].id;
  const uint32_t fixedBox = world.bodies.rigids[boxIndex].id;
  const talus::dynamics::Bounds everywhere{glm::dvec3(-10.0), glm::dvec3(10.0)};
  const std::vector<uint32_t> blockers = talus::dynamics::rayBlockers(world.bodies, everywhere);
  const talus::dynamics::RaySkip nothing{};

  CHECK(blockers == std::vector<uint32_t>{wallIndex, boxIndex});
  CHECK(talus::dynamics::rayBlockers(world.bodies, talus::dynamics::Bounds{glm::dvec3(5.0), glm::dvec3(6.0)}).empty());

  const std::optional<talus::dynamics::RayHit> straight =
    talus::dynamics::firstHit(world.bodies, blockers, glm::dvec3(0.0), glm::dvec3(3.0, 0.0, 0.0), nothing);

  REQUIRE(straight.has_value());
  CHECK(straight->body == wall);
  CHECK(straight->child == 1u);
  CHECK(std::abs(straight->fraction - 0.875 / 3.0) <= 1.0e-15);

  const std::optional<talus::dynamics::RayHit> past =
    talus::dynamics::firstHit(world.bodies, blockers, glm::dvec3(0.0), glm::dvec3(3.0, 0.0, 0.0), talus::dynamics::RaySkip{wall, 1u});

  REQUIRE(past.has_value());
  CHECK(past->body == fixedBox);
  CHECK(past->child == talus::dynamics::noChild);
  CHECK(std::abs(past->fraction - 1.875 / 3.0) <= 1.0e-15);

  const std::optional<talus::dynamics::RayHit> seam = talus::dynamics::firstHit(
    world.bodies, blockers, glm::dvec3(0.0, 0.0, 0.125), glm::dvec3(3.0, 0.0, 0.125), nothing
  );

  REQUIRE(seam.has_value());
  CHECK(seam->child == 1u);

  const std::optional<talus::dynamics::RayHit> inside =
    talus::dynamics::firstHit(world.bodies, blockers, glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(3.0, 0.0, 0.0), nothing);

  REQUIRE(inside.has_value());
  CHECK(inside->body == fixedBox);
  CHECK(std::abs(inside->fraction - 0.875 / 2.0) <= 1.0e-15);

  CHECK(!talus::dynamics::firstHit(world.bodies, blockers, glm::dvec3(0.0), glm::dvec3(0.5, 0.0, 0.0), nothing).has_value());
  CHECK(!talus::dynamics::firstHit(world.bodies, blockers, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(3.0, 1.0, 0.0), nothing).has_value());
  CHECK(world.bodies.rigids[movingIndex].mass > 0.0);
}

TEST_CASE("a moving compound's children block rays too", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t wallIndex = addRayWall(world);
  const uint32_t pieceIndex = talus::dynamics::addCompoundBody(
    world.bodies, cubeOfEight(0.0625, 1000.0, 20u), glm::dvec3(0.5, 0.0, 0.0), noTurn, glm::dvec3(0.0), 0.5, 0.0, false
  );
  const talus::dynamics::Bounds everywhere{glm::dvec3(-10.0), glm::dvec3(10.0)};
  const std::vector<uint32_t> blockers = talus::dynamics::rayBlockers(world.bodies, everywhere);

  CHECK(blockers == std::vector<uint32_t>{wallIndex, pieceIndex});

  const std::optional<talus::dynamics::RayHit> hit = talus::dynamics::firstHit(
    world.bodies, blockers, glm::dvec3(0.0, 0.01, 0.01), glm::dvec3(3.0, 0.01, 0.01), talus::dynamics::RaySkip{}
  );

  REQUIRE(hit.has_value());
  CHECK(hit->body == world.bodies.rigids[pieceIndex].id);
  CHECK(hit->child >= 20u);
  CHECK(std::abs(hit->fraction - 0.375 / 3.0) <= 1.0e-12);
}

TEST_CASE("a ray meets a turned child of a turned compound and a turned zero-mass box where their turns put them", "[dynamics][compound]") {
  const talus::field::SampledSdf crate = boxShape(glm::dvec3(0.2, 0.05, 0.1));
  const glm::dvec3 half(0.3, 0.1, 0.05);
  const std::array<talus::dynamics::CompoundChild, 1> children{
    part(glm::dvec3(0.4, -0.2, 0.3), half, 1000.0, 5u, glm::angleAxis(0.9, glm::normalize(glm::dvec3(-0.3, 1.0, 0.4))))
  };
  talus::dynamics::World world;
  const uint32_t bodyIndex = talus::dynamics::addCompoundBody(
    world.bodies, children, glm::dvec3(1.0, 2.0, -1.5), glm::angleAxis(0.6, glm::normalize(glm::dvec3(1.0, 2.0, 0.5))), glm::dvec3(0.0), 0.5, 0.0, true
  );
  const uint32_t boxIndex = talus::dynamics::addRigidBox(world.bodies, crate, glm::dvec3(0.2, 0.05, 0.1), 0.0, glm::dvec3(-3.0, 0.5, 2.0), glm::dvec3(0.0), 0.5, 3u);

  REQUIRE(talus::dynamics::setRigidPose(
    world, world.bodies.rigids[boxIndex].id, glm::dvec3(-3.0, 0.5, 2.0), glm::angleAxis(0.7, glm::normalize(glm::dvec3(0.2, -1.0, 0.6)))
  ));

  const talus::dynamics::RigidBody& body = world.bodies.rigids[bodyIndex];
  const talus::dynamics::RigidBody& box = world.bodies.rigids[boxIndex];
  const talus::dynamics::CompoundChild child = childWithKey(body, 5u);
  const glm::dvec3 centre = body.position + glm::mat3_cast(body.rotation) * child.centre;
  const glm::dmat3 childTurn = glm::mat3_cast(body.rotation * child.rotation);
  const glm::dmat3 boxTurn = glm::mat3_cast(box.rotation);
  const std::vector<uint32_t> blockers =
    talus::dynamics::rayBlockers(world.bodies, talus::dynamics::Bounds{glm::dvec3(-10.0), glm::dvec3(10.0)});

  REQUIRE(blockers == std::vector<uint32_t>{bodyIndex, boxIndex});

  for (int axis = 0; axis != 3; ++axis) {
    INFO("axis " << axis);

    const std::optional<talus::dynamics::RayHit> onChild = talus::dynamics::firstHit(
      world.bodies, blockers, centre - childTurn[axis], centre + childTurn[axis], talus::dynamics::RaySkip{}
    );
    const std::optional<talus::dynamics::RayHit> onBox = talus::dynamics::firstHit(
      world.bodies, blockers, box.position - boxTurn[axis], box.position + boxTurn[axis], talus::dynamics::RaySkip{}
    );

    REQUIRE(onChild.has_value());
    CHECK(onChild->body == body.id);
    CHECK(onChild->child == 5u);
    CHECK(std::abs(onChild->fraction - 0.5 * (1.0 - half[axis])) <= 1.0e-12);
    REQUIRE(onBox.has_value());
    CHECK(onBox->body == box.id);
    CHECK(onBox->child == talus::dynamics::noChild);
    CHECK(std::abs(onBox->fraction - 0.5 * (1.0 - box.halfExtents[axis])) <= 1.0e-12);
  }
}

TEST_CASE("an impulse at a point changes a body's momentum by the impulse and its angular momentum by the impulse's moment, and wakes it", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    lShape(),
    glm::dvec3(0.0, 1.0, 0.0),
    glm::angleAxis(0.4, glm::normalize(glm::dvec3(1.0, 1.0, 0.0))),
    glm::dvec3(0.5, 0.0, 0.0),
    0.5,
    0.0,
    false
  );
  const uint32_t slab = addSlab(world, 2u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -2.0, 0.0));

  world.bodies.rigids[index].angularVelocity = glm::dvec3(0.2, -0.1, 0.3);
  world.bodies.rigids[index].asleep = true;

  const talus::dynamics::RigidBody before = world.bodies.rigids[index];
  const glm::dvec3 impulse(3.0, -1.0, 2.0);
  const glm::dvec3 point = before.position + glm::dvec3(0.2, 0.1, -0.15);

  REQUIRE(talus::dynamics::applyImpulse(world, before.id, impulse, point));

  const talus::dynamics::RigidBody& after = world.bodies.rigids[index];
  const glm::dvec3 momentum = after.mass * (after.velocity - before.velocity);
  const glm::dvec3 angular = talus::dynamics::worldInertia(after) * (after.angularVelocity - before.angularVelocity);
  const glm::dvec3 moment = glm::cross(point - before.position, impulse);

  CHECK(glm::length(momentum - impulse) <= 1.0e-12 * glm::length(impulse));
  CHECK(glm::length(angular - moment) <= 1.0e-12 * glm::length(moment));
  CHECK(!after.asleep);
  CHECK(after.previousVelocity == after.velocity);
  CHECK(after.position == before.position);
  CHECK(!talus::dynamics::applyImpulse(world, world.bodies.rigids[slab].id, impulse, point));
  CHECK(!talus::dynamics::applyImpulse(world, 999u, impulse, point));
}

TEST_CASE("setting a body's spin wakes it and refuses a body without mass", "[dynamics][compound]") {
  talus::dynamics::World world;
  const uint32_t index = addEightCube(world, 0.1, 1000.0, glm::dvec3(0.0, 1.0, 0.0), glm::dvec3(0.0));
  const uint32_t slab = addSlab(world, 2u, 1u, glm::dvec3(0.1, 0.05, 0.1), glm::dvec3(0.0, -2.0, 0.0));

  world.bodies.rigids[index].asleep = true;

  REQUIRE(talus::dynamics::setAngularVelocity(world, world.bodies.rigids[index].id, glm::dvec3(0.0, 2.0, 0.0)));
  CHECK(world.bodies.rigids[index].angularVelocity == glm::dvec3(0.0, 2.0, 0.0));
  CHECK(!world.bodies.rigids[index].asleep);
  CHECK(!talus::dynamics::setAngularVelocity(world, world.bodies.rigids[slab].id, glm::dvec3(1.0)));
  CHECK(!talus::dynamics::setAngularVelocity(world, 999u, glm::dvec3(1.0)));
}

TEST_CASE("in-place replacement cost on large anchored bodies", "[.][compoundreplacebench]") {
  for (const glm::uvec3 grid : {glm::uvec3(20u, 20u, 10u), glm::uvec3(25u, 20u, 20u)}) {
    const uint32_t count = grid.x * grid.y * grid.z;

    for (const uint32_t replaced : {1u, 10u, 100u}) {
      double best = std::numeric_limits<double>::infinity();
      double rebuild = std::numeric_limits<double>::infinity();

      for (uint32_t run = 0u; run != 5u; ++run) {
        talus::dynamics::World world;
        const uint32_t index = addGridBuilding(world, grid.x, grid.y, grid.z);
        const uint32_t id = world.bodies.rigids[index].id;
        std::vector<talus::dynamics::ChildReplacement> swaps;

        for (uint32_t at = 0u; at != replaced; ++at) {
          const talus::dynamics::CompoundChild& child = world.bodies.rigids[index].compound->children[(at * 37u + count / 2u) % count];

          swaps.push_back(talus::dynamics::ChildReplacement{child.key, filling(child, glm::uvec3(2u), 100000u + 8u * at)});
        }

        const auto before = std::chrono::steady_clock::now();

        REQUIRE(talus::dynamics::replaceCompoundChildren(world, id, swaps));
        best = std::min(best, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());

        const std::vector<talus::dynamics::CompoundChild> all = world.bodies.rigids[index].compound->children;
        const auto start = std::chrono::steady_clock::now();
        const std::shared_ptr<const talus::dynamics::CompoundShape> rebuilt = talus::dynamics::makeCompoundShape(all);

        rebuild = std::min(rebuild, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
        REQUIRE(rebuilt->children.size() == count + 7u * replaced);
      }

      std::printf("compoundreplacebench children %u replaced %u in one call ms %.4f | full rebuild ms %.3f (no limit)\n", count, replaced, best, rebuild);
    }
  }
}

TEST_CASE("swept-test cost with fast debris and glass shards over a large anchored building", "[.][sweepbench]") {
  const std::vector<talus::dynamics::CompoundChild> shard{part(glm::dvec3(0.0), glm::dvec3(0.08, 0.01, 0.08), 2500.0, 0u)};
  const std::vector<talus::dynamics::CompoundChild> cube = cubeOfEight(0.0625, 2000.0, 0u);

  for (const uint32_t workers : {1u, 8u}) {
    for (const bool shards : {false, true}) {
      for (const double speed : {2.0, 10.0}) {
        for (const bool continuous : {false, true}) {
          double bestMean = std::numeric_limits<double>::infinity();
          double bestSweep = std::numeric_limits<double>::infinity();
          double worst = 0.0;
          uint32_t sunk = 0u;

          for (uint32_t run = 0u; run != 3u; ++run) {
            talus::dynamics::World world;

            world.workerCount = workers;
            world.parameters.continuous = continuous;
            (void)addGridBuilding(world, 20u, 20u, 10u);

            for (uint32_t i = 0u; i != 10u; ++i) {
              for (uint32_t k = 0u; k != 5u; ++k) {
                for (uint32_t level = 0u; level != 4u; ++level) {
                  (void)talus::dynamics::addCompoundBody(
                    world.bodies,
                    shards ? shard : cube,
                    glm::dvec3(0.3 + 0.5 * i, 5.6 + 0.6 * level, 0.3 + 0.5 * k),
                    glm::angleAxis(0.3 * (i + k + level), glm::normalize(glm::dvec3(1.0, 0.5, 0.2))),
                    glm::dvec3(0.0, -speed, 0.0),
                    0.5,
                    0.0,
                    false
                  );
                }
              }
            }

            double total = 0.0;
            double sweep = 0.0;

            worst = 0.0;

            for (uint32_t frame = 0u; frame != 90u; ++frame) {
              const auto before = std::chrono::steady_clock::now();
              const talus::dynamics::StepReport report = talus::dynamics::step(world);
              const double took = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();

              REQUIRE(!report.nonFinite);
              total += took;
              sweep += report.profile.continuous;
              worst = std::max(worst, took);
            }

            sunk = 0u;

            for (size_t index = 1u; index != world.bodies.rigids.size(); ++index) {
              const glm::dvec3& centre = world.bodies.rigids[index].position;
              const bool inside = centre.x > -0.125 && centre.x < 4.875 && centre.y > -0.125 && centre.y < 4.875
                                  && centre.z > -0.125 && centre.z < 2.375;

              sunk += inside ? 1u : 0u;
            }

            bestMean = std::min(bestMean, total / 90.0);
            bestSweep = std::min(bestSweep, sweep / 90.0);
          }

          std::printf(
            "sweepbench workers %u %s at %.0f m/s continuous %d | best mean step ms %.3f, continuous phase ms %.3f | last run worst step ms %.3f, centres inside the building %u\n",
            workers,
            shards ? "200 glass shards" : "200 debris cubes",
            speed,
            continuous ? 1 : 0,
            bestMean,
            bestSweep,
            worst,
            sunk
          );
        }
      }
    }
  }
}
