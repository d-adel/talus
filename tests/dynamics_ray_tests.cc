#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/rays/worldray.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <vector>

namespace {

using talus::dynamics::BodyRef;
using talus::dynamics::CompoundChild;
using talus::dynamics::noChild;
using talus::dynamics::RaySkip;
using talus::dynamics::RigidBody;
using talus::dynamics::Side;
using talus::dynamics::World;

const talus::field::SampledSdf& emptyField() {
  static const talus::field::SampledSdf field{};

  return field;
}

uint32_t addBox(
  World& world,
  const glm::dvec3& half,
  const glm::dvec3& position,
  double mass = 1.0
) {
  talus::dynamics::addRigidBox(
    world.bodies,
    emptyField(),
    half,
    mass,
    position,
    glm::dvec3(0.0),
    0.5,
    3u
  );

  return world.bodies.rigids.back().id;
}

uint32_t addSphere(World& world, double radius, const glm::dvec3& position) {
  talus::dynamics::addRigidSphere(
    world.bodies,
    emptyField(),
    radius,
    1.0,
    position,
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  return world.bodies.rigids.back().id;
}

CompoundChild childBox(
  const glm::dvec3& centre,
  const glm::dquat& rotation,
  const glm::dvec3& half,
  uint32_t key
) {
  CompoundChild child;

  child.centre = centre;
  child.rotation = rotation;
  child.half = half;
  child.density = 1000.0;
  child.key = key;

  return child;
}

uint32_t addCompound(
  World& world,
  const std::vector<CompoundChild>& children,
  const glm::dvec3& position,
  const glm::dquat& rotation
) {
  talus::dynamics::addCompoundBody(
    world.bodies,
    children,
    position,
    rotation,
    glm::dvec3(0.0),
    0.5,
    0.0,
    false
  );

  return world.bodies.rigids.back().id;
}

const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);

talus::field::SampledSdf boxField(
  const glm::dvec3& half,
  double voxel,
  double outset = 0.0
) {
  const glm::vec3 reach =
    glm::vec3(half) + glm::vec3(static_cast<float>(2.0 * voxel));

  return talus::field::bake(
    std::vector<talus::field::Primitive>{fixture::box(
      glm::vec3(0.0f),
      glm::vec3(half) + glm::vec3(static_cast<float>(outset))
    )},
    -reach,
    reach,
    glm::uvec3(glm::round(2.0f * reach / static_cast<float>(voxel)))
      + glm::uvec3(1u)
  );
}

double angleBetween(const glm::dvec3& a, const glm::dvec3& b) {
  return std::acos(
    std::clamp(glm::dot(glm::normalize(a), glm::normalize(b)), -1.0, 1.0)
  );
}

std::vector<glm::dvec3> boxSamples(const glm::dvec3& half) {
  std::vector<glm::dvec3> samples;

  for (int x = -1; x != 2; ++x) {
    for (int y = -1; y != 2; ++y) {
      for (int z = -1; z != 2; ++z) {
        if (x != 0 || y != 0 || z != 0) {
          samples.push_back(glm::dvec3(x, y, z) * half);
        }
      }
    }
  }

  return samples;
}

uint32_t addSampledBox(
  World& world,
  const talus::field::SampledSdf& field,
  const glm::dvec3& half,
  const glm::dvec3& position,
  const glm::dquat& rotation
) {
  const std::vector<glm::dvec3> samples = boxSamples(half);

  talus::dynamics::addRigidBody(
    world.bodies,
    field,
    samples,
    1.0,
    glm::dvec3(1.0e-2),
    position,
    glm::dvec3(0.0),
    0.5,
    0.0
  );
  world.bodies.rigids.back().rotation = rotation;

  return world.bodies.rigids.back().id;
}

talus::PhysicalMaterial jelly() {
  talus::PhysicalMaterial material;

  material.density = 500.0f;
  material.friction = 0.7f;
  material.youngsModulus = 5.0e4f;
  material.poissonsRatio = 0.4f;

  return material;
}

struct SoftBlock {
  talus::field::SampledSdf field;
  std::vector<talus::PhysicalMaterial> lanes;
  std::shared_ptr<const talus::PreparedMesh> mesh;
};

SoftBlock softBlock(const glm::dvec3& half, double voxel) {
  SoftBlock block;

  block.field = boxField(half, voxel, 1.0e-4);
  block.lanes.assign(2u, jelly());
  block.mesh = talus::deformable::buildMesh(
    block.field,
    block.lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  return block;
}

uint32_t addSoftBlock(
  World& world,
  const SoftBlock& block,
  const glm::dvec3& at
) {
  (void)talus::dynamics::addLattice(
    world.bodies,
    block.mesh,
    block.lanes,
    at,
    glm::dvec3(0.0),
    block.lanes.front().friction,
    &block.field
  );

  return world.bodies.lattices.back().id;
}

void expectSameBody(const World& world, uint32_t id) {
  const std::optional<BodyRef> located = talus::dynamics::locateBody(world, id);
  const std::optional<BodyRef> searched =
    talus::dynamics::findBody(world.bodies, id);

  REQUIRE(located.has_value() == searched.has_value());

  if (located) {
    CHECK(located->side == searched->side);
    CHECK(located->index == searched->index);
  }
}

}

TEST_CASE(
  "the world ray cast hits spheres, boxes and compound children at the exact "
  "point and normal",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const glm::dvec3 centre(0.0, 1.0, 0.0);
  const uint32_t sphere = addSphere(world, 0.25, centre);

  const glm::dquat boxTurn =
    glm::angleAxis(glm::radians(30.0), glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t box =
    addBox(world, glm::dvec3(0.2, 0.1, 0.3), glm::dvec3(2.0, 1.0, 0.0));

  world.bodies.rigids.back().rotation = boxTurn;

  const uint32_t fixedBox =
    addBox(world, glm::dvec3(0.5), glm::dvec3(8.0, 0.5, 0.0), 0.0);

  const glm::dquat bodyTurn =
    glm::angleAxis(glm::radians(20.0), glm::dvec3(0.0, 0.0, 1.0));
  const glm::dquat childTurn =
    glm::angleAxis(glm::radians(15.0), glm::dvec3(1.0, 0.0, 0.0));
  const std::vector<CompoundChild> children{
    childBox(
      glm::dvec3(-0.4, 0.0, 0.0),
      childTurn,
      glm::dvec3(0.2, 0.1, 0.2),
      7u
    ),
    childBox(glm::dvec3(0.4, 0.0, 0.0), noTurn, glm::dvec3(0.2, 0.1, 0.2), 8u)
  };
  const glm::dvec3 compoundAt(5.0, 1.0, 0.0);
  const uint32_t compound = addCompound(world, children, compoundAt, bodyTurn);

  for (int pass = 0; pass != 2; ++pass) {
    if (pass == 1) {
      (void)talus::dynamics::step(world);
    }

    {
      const glm::dvec3 from(0.1, 3.0, 0.0);
      const glm::dvec3 to(0.1, -1.0, 0.0);
      const auto hit = talus::dynamics::castRay(world, from, to);
      const glm::dvec3 expected(0.1, 1.0 + std::sqrt(0.0625 - 0.01), 0.0);
      const glm::dvec3 normal = (expected - centre) / 0.25;

      REQUIRE(hit.has_value());
      CHECK(hit->side == Side::Rigid);
      CHECK(hit->body == sphere);
      CHECK(hit->child == noChild);
      CHECK(glm::length(hit->point - expected) < 1.0e-12);
      CHECK(glm::length(hit->normal - normal) < 1.0e-12);
      CHECK(std::abs(hit->fraction - (3.0 - expected.y) / 4.0) < 1.0e-12);
    }

    {
      const auto hit = talus::dynamics::castRay(
        world,
        glm::dvec3(2.05, 3.0, 0.04),
        glm::dvec3(2.05, -1.0, 0.04)
      );

      REQUIRE(hit.has_value());
      CHECK(hit->body == box);
      CHECK(std::abs(hit->point.y - 1.1) < 1.0e-12);
      CHECK(glm::length(hit->normal - glm::dvec3(0.0, 1.0, 0.0)) < 1.0e-12);
    }

    {
      const glm::dvec3 from(3.0, 1.0, 0.0);
      const glm::dvec3 to(1.0, 1.0, 0.0);
      const auto hit = talus::dynamics::castRay(world, from, to);
      const glm::dvec3 expected = boxTurn * glm::dvec3(1.0, 0.0, 0.0);

      REQUIRE(hit.has_value());
      CHECK(hit->body == box);
      CHECK(glm::length(hit->normal - expected) < 1.0e-12);
      CHECK(
        glm::length(hit->point - (from + hit->fraction * (to - from))) < 1.0e-12
      );
    }

    {
      const glm::dvec3 above = compoundAt
                               + bodyTurn * glm::dvec3(-0.4, 0.0, 0.0)
                               + glm::dvec3(0.0, 2.0, 0.0);
      const auto hit = talus::dynamics::castRay(
        world,
        above,
        above - glm::dvec3(0.0, 4.0, 0.0)
      );
      const glm::dvec3 expected =
        bodyTurn * (childTurn * glm::dvec3(0.0, 1.0, 0.0));

      REQUIRE(hit.has_value());
      CHECK(hit->body == compound);
      CHECK(hit->child == 7u);
      CHECK(glm::length(hit->normal - expected) < 1.0e-12);
    }

    {
      const auto hit = talus::dynamics::castRay(
        world,
        glm::dvec3(8.1, 3.0, 0.05),
        glm::dvec3(8.1, -1.0, 0.05)
      );

      REQUIRE(hit.has_value());
      CHECK(hit->body == fixedBox);
      CHECK(std::abs(hit->point.y - 1.0) < 1.0e-12);
      CHECK(glm::length(hit->normal - glm::dvec3(0.0, 1.0, 0.0)) < 1.0e-12);
    }

    CHECK_FALSE(
      talus::dynamics::castRay(
        world,
        glm::dvec3(20.0, 3.0, 0.0),
        glm::dvec3(20.0, -1.0, 0.0)
      )
        .has_value()
    );
  }
}

TEST_CASE(
  "the world ray cast returns the nearest hit and breaks ties by side, body "
  "and child",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t sphere = addSphere(world, 0.25, glm::dvec3(0.0, 0.0, 0.0));
  const uint32_t front =
    addBox(world, glm::dvec3(0.5), glm::dvec3(0.0, 2.0, 0.0));
  const uint32_t twinA =
    addBox(world, glm::dvec3(0.2), glm::dvec3(5.0, 0.0, 0.0));
  const uint32_t twinB =
    addBox(world, glm::dvec3(0.2), glm::dvec3(5.0, 0.0, 0.0));

  REQUIRE(twinA < twinB);

  for (int pass = 0; pass != 2; ++pass) {
    if (pass == 1) {
      (void)talus::dynamics::step(world);
    }

    const auto nearest = talus::dynamics::castRay(
      world,
      glm::dvec3(0.0, 4.0, 0.0),
      glm::dvec3(0.0, -4.0, 0.0)
    );

    REQUIRE(nearest.has_value());
    CHECK(nearest->body == front);
    CHECK(nearest->body != sphere);

    const auto tied = talus::dynamics::castRay(
      world,
      glm::dvec3(5.0, 3.0, 0.0),
      glm::dvec3(5.0, -3.0, 0.0)
    );

    REQUIRE(tied.has_value());
    CHECK(tied->body == twinA);
  }
}

TEST_CASE(
  "the world ray cast skips every named body and compound child",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t top =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 3.0, 0.0), 0.0);
  const uint32_t middle =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 2.0, 0.0), 0.0);
  const uint32_t bottom =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 1.0, 0.0), 0.0);
  const std::vector<CompoundChild> children{
    childBox(glm::dvec3(0.0, 0.0, 0.0), noTurn, glm::dvec3(0.2), 7u),
    childBox(glm::dvec3(0.0, -0.5, 0.0), noTurn, glm::dvec3(0.2), 8u)
  };
  const uint32_t compound =
    addCompound(world, children, glm::dvec3(10.0, 3.0, 0.0), noTurn);
  const uint32_t behind =
    addBox(world, glm::dvec3(0.3), glm::dvec3(10.0, 1.0, 0.0), 0.0);
  const glm::dvec3 from(0.0, 5.0, 0.0);
  const glm::dvec3 to(0.0, -1.0, 0.0);
  const glm::dvec3 compoundFrom(10.0, 5.0, 0.0);
  const glm::dvec3 compoundTo(10.0, -1.0, 0.0);

  for (int pass = 0; pass != 2; ++pass) {
    if (pass == 1) {
      (void)talus::dynamics::step(world);
    }

    CHECK(talus::dynamics::castRay(world, from, to)->body == top);

    const std::array<RaySkip, 1> one{RaySkip{top, noChild}};
    const std::array<RaySkip, 2> two{
      RaySkip{top, noChild},
      RaySkip{middle, noChild}
    };

    CHECK(talus::dynamics::castRay(world, from, to, one)->body == middle);
    CHECK(talus::dynamics::castRay(world, from, to, two)->body == bottom);

    const auto first =
      talus::dynamics::castRay(world, compoundFrom, compoundTo);

    REQUIRE(first.has_value());
    CHECK(first->body == compound);
    CHECK(first->child == 7u);

    const std::array<RaySkip, 1> child{RaySkip{compound, 7u}};
    const auto second =
      talus::dynamics::castRay(world, compoundFrom, compoundTo, child);

    REQUIRE(second.has_value());
    CHECK(second->body == compound);
    CHECK(second->child == 8u);

    const std::array<RaySkip, 1> whole{RaySkip{compound, noChild}};
    const auto beyond =
      talus::dynamics::castRay(world, compoundFrom, compoundTo, whole);

    REQUIRE(beyond.has_value());
    CHECK(beyond->body == behind);
  }
}

TEST_CASE(
  "the world ray cast ignores a shape that contains its start",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t sphere = addSphere(world, 0.5, glm::dvec3(0.0, 2.0, 0.0));
  const uint32_t box =
    addBox(world, glm::dvec3(0.5), glm::dvec3(0.0, 0.0, 0.0), 0.0);
  const std::vector<CompoundChild> children{
    childBox(glm::dvec3(0.0, 0.0, 0.0), noTurn, glm::dvec3(0.2), 7u),
    childBox(glm::dvec3(0.0, -0.6, 0.0), noTurn, glm::dvec3(0.2), 8u)
  };
  const uint32_t compound =
    addCompound(world, children, glm::dvec3(10.0, 2.0, 0.0), noTurn);

  for (int pass = 0; pass != 2; ++pass) {
    if (pass == 1) {
      (void)talus::dynamics::step(world);
    }

    const auto fromSphere = talus::dynamics::castRay(
      world,
      glm::dvec3(0.0, 2.0, 0.0),
      glm::dvec3(0.0, -2.0, 0.0)
    );

    REQUIRE(fromSphere.has_value());
    CHECK(fromSphere->body == box);
    CHECK(fromSphere->body != sphere);

    const auto fromChild = talus::dynamics::castRay(
      world,
      glm::dvec3(10.0, 2.0, 0.0),
      glm::dvec3(10.0, 0.0, 0.0)
    );

    REQUIRE(fromChild.has_value());
    CHECK(fromChild->body == compound);
    CHECK(fromChild->child == 8u);
  }
}

TEST_CASE(
  "the world ray cast finds bodies added, and drops bodies removed, since the "
  "last step",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t first =
    addBox(world, glm::dvec3(0.3), glm::dvec3(5.0, 0.0, 0.0));
  const glm::dvec3 from(0.0, 3.0, 0.0);
  const glm::dvec3 to(0.0, -3.0, 0.0);

  (void)talus::dynamics::step(world);

  CHECK_FALSE(talus::dynamics::castRay(world, from, to).has_value());

  const uint32_t added =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 0.0, 0.0));
  const auto found = talus::dynamics::castRay(world, from, to);

  REQUIRE(found.has_value());
  CHECK(found->body == added);

  CHECK(talus::dynamics::removeBody(world, added));
  CHECK_FALSE(talus::dynamics::castRay(world, from, to).has_value());

  const uint32_t again =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 0.0, 0.0));

  (void)talus::dynamics::step(world);

  const auto afterStep = talus::dynamics::castRay(world, from, to);

  REQUIRE(afterStep.has_value());
  CHECK(afterStep->body == again);

  CHECK(talus::dynamics::removeBody(world, again));
  CHECK_FALSE(talus::dynamics::castRay(world, from, to).has_value());

  (void)talus::dynamics::step(world);

  CHECK_FALSE(talus::dynamics::castRay(world, from, to).has_value());

  const auto remaining = talus::dynamics::castRay(
    world,
    glm::dvec3(5.0, 3.0, 0.0),
    glm::dvec3(5.0, -3.0, 0.0)
  );

  REQUIRE(remaining.has_value());
  CHECK(remaining->body == first);
}

TEST_CASE(
  "the world ray cast finds a new body when the body list is not in id order",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  addBox(world, glm::dvec3(0.3), glm::dvec3(5.0, 0.0, 0.0));
  const uint32_t old =
    addBox(world, glm::dvec3(0.3), glm::dvec3(8.0, 0.0, 0.0));

  (void)talus::dynamics::step(world);

  const uint32_t added =
    addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 0.0, 0.0));

  REQUIRE(added >= world.broadPhase.seenBodies());
  std::swap(world.bodies.rigids[1], world.bodies.rigids[2]);
  REQUIRE(world.bodies.rigids.back().id == old);

  const auto hit = talus::dynamics::castRay(
    world,
    glm::dvec3(0.0, 3.0, 0.0),
    glm::dvec3(0.0, -3.0, 0.0)
  );

  REQUIRE(hit.has_value());
  CHECK(hit->body == added);

  const auto oldHit = talus::dynamics::castRay(
    world,
    glm::dvec3(8.0, 3.0, 0.0),
    glm::dvec3(8.0, -3.0, 0.0)
  );

  REQUIRE(oldHit.has_value());
  CHECK(oldHit->body == old);
}

TEST_CASE(
  "the world ray cast agrees with a full scan on random rays",
  "[dynamics][rays]"
) {
  std::mt19937_64 random(20261031u);
  const auto uniform = [&random](double low, double high) {
    return std::uniform_real_distribution<double>(low, high)(random);
  };
  const auto point = [&](double extent) {
    return glm::dvec3(
      uniform(-extent, extent),
      uniform(0.0, extent),
      uniform(-extent, extent)
    );
  };
  const auto turn = [&]() {
    const double angle = uniform(0.0, 6.2);
    const glm::dvec3 axis = glm::normalize(
      glm::dvec3(uniform(-1.0, 1.0), uniform(0.1, 1.0), uniform(-1.0, 1.0))
    );

    return glm::angleAxis(angle, axis);
  };

  World a;

  for (uint32_t index = 0u; index != 300u; ++index) {
    const glm::dvec3 at = point(8.0);

    switch (index % 4u) {
      case 0u:
        addSphere(a, uniform(0.1, 0.4), at);
        break;
      case 1u: {
        const glm::dvec3 half(
          uniform(0.1, 0.4),
          uniform(0.1, 0.4),
          uniform(0.1, 0.4)
        );

        addBox(a, half, at);
        a.bodies.rigids.back().rotation = turn();
        break;
      }
      case 2u:
        addBox(a, glm::dvec3(uniform(0.1, 0.4)), at, 0.0);
        a.bodies.rigids.back().rotation = turn();
        break;
      default: {
        const std::vector<CompoundChild> children{
          childBox(
            glm::dvec3(-0.3, 0.0, 0.0),
            turn(),
            glm::dvec3(0.15, 0.1, 0.2),
            4u * index
          ),
          childBox(
            glm::dvec3(0.3, 0.1, 0.0),
            turn(),
            glm::dvec3(0.2, 0.15, 0.1),
            4u * index + 1u
          ),
          childBox(
            glm::dvec3(0.0, 0.4, 0.2),
            turn(),
            glm::dvec3(0.1, 0.1, 0.1),
            4u * index + 2u
          )
        };

        addCompound(a, children, at, turn());
        break;
      }
    }
  }

  for (int frame = 0; frame != 30; ++frame) {
    (void)talus::dynamics::step(a);
  }

  World b;

  b.bodies = a.bodies;

  REQUIRE(a.broadPhase.seenBodies() >= a.bodies.nextId);
  REQUIRE(b.broadPhase.seenBodies() == 0u);

  uint32_t hits = 0u;

  for (uint32_t ray = 0u; ray != 2000u; ++ray) {
    const glm::dvec3 from = point(9.0);
    const glm::dvec3 to =
      from
      + glm::dvec3(uniform(-6.0, 6.0), uniform(-6.0, 6.0), uniform(-6.0, 6.0));
    const auto left = talus::dynamics::castRay(a, from, to);
    const auto right = talus::dynamics::castRay(b, from, to);

    REQUIRE(left.has_value() == right.has_value());

    if (!left) {
      continue;
    }

    hits += 1u;

    CHECK(left->side == right->side);
    CHECK(left->body == right->body);
    CHECK(left->child == right->child);
    CHECK(std::abs(left->fraction - right->fraction) <= 1.0e-12);
    CHECK(glm::length(left->point - right->point) <= 1.0e-12);
    CHECK(glm::length(left->normal - right->normal) <= 1.0e-12);
  }

  CHECK(hits > 100u);
}

TEST_CASE(
  "locating a body by id agrees with a full search after additions and "
  "removals",
  "[dynamics]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  for (uint32_t index = 0u; index != 50u; ++index) {
    addBox(
      world,
      glm::dvec3(0.2),
      glm::dvec3(static_cast<double>(index), 0.0, 0.0)
    );
  }

  (void)talus::dynamics::step(world);

  std::mt19937 random(7u);
  std::vector<uint32_t> doomed;

  while (doomed.size() != 10u) {
    const uint32_t id = random() % 50u;

    if (std::find(doomed.begin(), doomed.end(), id) == doomed.end()) {
      doomed.push_back(id);
    }
  }

  CHECK(talus::dynamics::removeBodies(world, doomed) == 10u);

  for (uint32_t index = 0u; index != 5u; ++index) {
    addBox(
      world,
      glm::dvec3(0.2),
      glm::dvec3(0.0, 2.0 + static_cast<double>(index), 0.0)
    );
  }

  for (uint32_t id = 0u; id != world.bodies.nextId; ++id) {
    expectSameBody(world, id);
  }

  (void)talus::dynamics::step(world);

  for (uint32_t id = 0u; id != world.bodies.nextId; ++id) {
    expectSameBody(world, id);
  }
}

TEST_CASE(
  "the world ray cast hits static fields at the field's surface",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const talus::field::SampledSdf floor =
    boxField(glm::dvec3(2.0, 0.1, 2.0), 0.02);
  const uint32_t flat = talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const std::optional<talus::dynamics::WorldRayHit> level =
    talus::dynamics::castRay(
      world,
      glm::dvec3(0.31, 1.0, 0.17),
      glm::dvec3(0.31, -1.0, 0.17)
    );

  REQUIRE(level.has_value());
  CHECK(level->side == Side::Static);
  CHECK(level->body == flat);
  CHECK(std::abs(level->point.y - 0.1) < 1.0e-5);
  CHECK(angleBetween(level->normal, glm::dvec3(0.0, 1.0, 0.0)) < 1.0e-4);

  const glm::dquat tilt =
    glm::angleAxis(glm::radians(30.0), glm::dvec3(0.0, 0.0, 1.0));
  const glm::dvec3 place(5.0, 0.0, 0.0);
  const uint32_t tilted =
    talus::dynamics::addStatic(world.bodies, floor, 0.5, 0.0, place, tilt);
  const glm::dvec3 up = tilt * glm::dvec3(0.0, 1.0, 0.0);

  const std::optional<talus::dynamics::WorldRayHit> slanted =
    talus::dynamics::castRay(
      world,
      place + glm::dvec3(0.3, 1.0, 0.2),
      place + glm::dvec3(0.3, -1.0, 0.2)
    );

  REQUIRE(slanted.has_value());
  CHECK(slanted->side == Side::Static);
  CHECK(slanted->body == tilted);
  CHECK(
    std::abs(std::abs(glm::dot(slanted->point - place, up)) - 0.1) < 1.0e-5
  );
  CHECK(angleBetween(slanted->normal, up) < 1.0e-4);

  (void)talus::dynamics::step(world);

  const std::optional<talus::dynamics::WorldRayHit> stepped =
    talus::dynamics::castRay(
      world,
      glm::dvec3(0.31, 1.0, 0.17),
      glm::dvec3(0.31, -1.0, 0.17)
    );

  REQUIRE(stepped.has_value());
  CHECK(stepped->body == flat);
  CHECK(std::abs(stepped->point.y - 0.1) < 1.0e-5);
}

TEST_CASE(
  "the world ray cast hits sampled rigid bodies at the field's surface",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const glm::dvec3 half(0.2);
  const talus::field::SampledSdf field = boxField(half, 0.01);
  const glm::dquat turn =
    glm::angleAxis(glm::radians(30.0), glm::dvec3(0.0, 1.0, 0.0));
  const uint32_t id =
    addSampledBox(world, field, half, glm::dvec3(0.0, 1.0, 0.0), turn);

  const auto cast = [&](const glm::dvec3& from, const glm::dvec3& to) {
    return talus::dynamics::castRay(world, from, to);
  };

  for (uint32_t pass = 0u; pass != 2u; ++pass) {
    const glm::dvec3 centre = world.bodies.rigids.back().position;
    const std::optional<talus::dynamics::WorldRayHit> top =
      cast(centre + glm::dvec3(0.05, 2.0, 0.03), centre - glm::dvec3(0, 2, 0));

    REQUIRE(top.has_value());
    CHECK(top->side == Side::Rigid);
    CHECK(top->body == id);
    CHECK(std::abs(top->point.y - (centre.y + 0.2)) < 1.0e-5);
    CHECK(angleBetween(top->normal, glm::dvec3(0.0, 1.0, 0.0)) < 1.0e-4);

    const glm::dvec3 face = turn * glm::dvec3(1.0, 0.0, 0.0);
    const std::optional<talus::dynamics::WorldRayHit> side =
      cast(centre + 2.0 * face + glm::dvec3(0.0, 0.04, 0.0), centre);

    REQUIRE(side.has_value());
    CHECK(side->body == id);
    CHECK(angleBetween(side->normal, face) < 1.0e-4);
    CHECK(std::abs(glm::dot(side->point - centre, face) - 0.2) < 1.0e-5);

    (void)talus::dynamics::step(world);
  }

  RigidBody& body = world.bodies.rigids.back();

  body.fieldOrigin = glm::dvec3(0.0, 0.05, 0.0);

  const std::optional<talus::dynamics::WorldRayHit> shifted = cast(
    body.position + glm::dvec3(0.05, 2.0, 0.03),
    body.position - glm::dvec3(0.0, 2.0, 0.0)
  );

  REQUIRE(shifted.has_value());
  CHECK(std::abs(shifted->point.y - (body.position.y + 0.15)) < 1.0e-5);
}

TEST_CASE(
  "the world ray cast hits lattice bodies at their current surface",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const SoftBlock block = softBlock(glm::dvec3(0.2, 0.05, 0.2), 0.05);
  const uint32_t id = addSoftBlock(world, block, glm::dvec3(0.0, 0.5, 0.0));
  const glm::dvec3 from(0.03, 2.0, -0.02);
  const glm::dvec3 to(0.03, -1.0, -0.02);

  const std::optional<talus::dynamics::WorldRayHit> rest =
    talus::dynamics::castRay(world, from, to);

  REQUIRE(rest.has_value());
  CHECK(rest->side == Side::Lattice);
  CHECK(rest->body == id);
  CHECK(std::abs(rest->point.y - (0.5 + 0.05 + 1.0e-4)) < 1.0e-5);
  CHECK(angleBetween(rest->normal, glm::dvec3(0.0, 1.0, 0.0)) < 1.0e-4);

  for (glm::dvec3& node : world.bodies.lattices.back().position) {
    node += glm::dvec3(0.0, 0.1, 0.0);
  }

  const std::optional<talus::dynamics::WorldRayHit> lifted =
    talus::dynamics::castRay(world, from, to);

  REQUIRE(lifted.has_value());
  CHECK(std::abs(lifted->point.y - rest->point.y - 0.1) < 1.0e-5);
}

TEST_CASE(
  "the world ray cast never hits a lattice body without a field",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const SoftBlock block = softBlock(glm::dvec3(0.2, 0.05, 0.2), 0.05);

  (void)talus::dynamics::addLattice(
    world.bodies,
    block.mesh,
    block.lanes,
    glm::dvec3(0.0, 0.5, 0.0),
    glm::dvec3(0.0),
    0.5,
    nullptr
  );

  CHECK(!talus::dynamics::castRay(
    world,
    glm::dvec3(0.03, 2.0, -0.02),
    glm::dvec3(0.03, -1.0, -0.02)
  ));
}

TEST_CASE(
  "the world ray cast ignores a field that contains its start",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const talus::field::SampledSdf floor =
    boxField(glm::dvec3(2.0, 0.1, 2.0), 0.02);

  (void)talus::dynamics::addStatic(world.bodies, floor, 0.5);

  const uint32_t above =
    addBox(world, glm::dvec3(0.2), glm::dvec3(0.0, 1.0, 0.0));

  const std::optional<talus::dynamics::WorldRayHit> hit =
    talus::dynamics::castRay(
      world,
      glm::dvec3(0.0, 0.05, 0.0),
      glm::dvec3(0.0, 3.0, 0.0)
    );

  REQUIRE(hit.has_value());
  CHECK(hit->side == Side::Rigid);
  CHECK(hit->body == above);
  CHECK(std::abs(hit->point.y - 0.8) < 1.0e-9);
}

TEST_CASE(
  "the world ray cast finds statics and lattices added after a step, in any "
  "list order",
  "[dynamics][rays]"
) {
  World world;

  world.parameters.gravity = glm::dvec3(0.0);

  const talus::field::SampledSdf floor =
    boxField(glm::dvec3(2.0, 0.1, 2.0), 0.02);
  const SoftBlock block = softBlock(glm::dvec3(0.2, 0.05, 0.2), 0.05);

  (void)talus::dynamics::addStatic(world.bodies, floor, 0.5);
  (void)addSoftBlock(world, block, glm::dvec3(9.0, 0.5, 0.0));
  (void)addBox(world, glm::dvec3(0.2), glm::dvec3(5.0, 1.0, 0.0));
  (void)talus::dynamics::step(world);

  const uint32_t later = talus::dynamics::addStatic(
    world.bodies,
    floor,
    0.5,
    0.0,
    glm::dvec3(20.0, 0.0, 0.0)
  );
  const uint32_t soft = addSoftBlock(world, block, glm::dvec3(30.0, 0.5, 0.0));

  std::reverse(world.bodies.lattices.begin(), world.bodies.lattices.end());

  const std::optional<talus::dynamics::WorldRayHit> onStatic =
    talus::dynamics::castRay(
      world,
      glm::dvec3(20.2, 1.0, 0.1),
      glm::dvec3(20.2, -1.0, 0.1)
    );
  const std::optional<talus::dynamics::WorldRayHit> onLattice =
    talus::dynamics::castRay(
      world,
      glm::dvec3(30.03, 2.0, -0.02),
      glm::dvec3(30.03, -1.0, -0.02)
    );

  REQUIRE(onStatic.has_value());
  CHECK(onStatic->body == later);
  CHECK(std::abs(onStatic->point.y - 0.1) < 1.0e-5);
  REQUIRE(onLattice.has_value());
  CHECK(onLattice->side == Side::Lattice);
  CHECK(onLattice->body == soft);
}

TEST_CASE("world ray cast cost against many bodies", "[.][raycost]") {
  using Clock = std::chrono::steady_clock;

  const talus::field::SampledSdf floor =
    boxField(glm::dvec3(75.0, 1.0, 75.0), 0.5);
  const std::array<uint32_t, 2> counts{1000u, 20000u};
  std::array<double, 2> treeUs{0.0, 0.0};

  for (uint32_t run = 0u; run != 2u; ++run) {
    const uint32_t count = counts[run];
    World world;
    World scan;

    world.parameters.gravity = glm::dvec3(0.0);
    scan.parameters.gravity = glm::dvec3(0.0);
    (void)talus::dynamics::addStatic(
      world.bodies,
      floor,
      0.5,
      0.0,
      glm::dvec3(0.0, -1.0, 0.0)
    );
    (void)talus::dynamics::addStatic(
      scan.bodies,
      floor,
      0.5,
      0.0,
      glm::dvec3(0.0, -1.0, 0.0)
    );

    std::mt19937 random(1234u);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(count)));
    const double span = static_cast<double>(side);

    std::vector<glm::dvec3> centres;

    for (uint32_t index = 0u; index != count; ++index) {
      const glm::dvec3 at(
        static_cast<double>(index % side) - 0.5 * span + jitter(random),
        0.1,
        static_cast<double>(index / side) - 0.5 * span + jitter(random)
      );

      centres.push_back(at);

      (void)addBox(world, glm::dvec3(0.1), at, 0.0);
      (void)addBox(scan, glm::dvec3(0.1), at, 0.0);
    }

    (void)talus::dynamics::step(world);

    std::uniform_real_distribution<double> spread(-0.5 * span, 0.5 * span);
    std::vector<std::array<glm::dvec3, 2>> rays;

    for (uint32_t index = 0u; index != 10000u; ++index) {
      const glm::dvec3& over = centres[random() % count];
      const double x = index % 2u == 0u ? spread(random)
                                        : over.x + 0.09 * jitter(random) / 0.3;
      const double z = index % 2u == 0u ? spread(random)
                                        : over.z + 0.09 * jitter(random) / 0.3;

      rays.push_back({glm::dvec3(x, 1.0, z), glm::dvec3(x, -0.8, z)});
    }

    std::vector<std::optional<talus::dynamics::WorldRayHit>> hits;

    hits.reserve(rays.size());

    const Clock::time_point start = Clock::now();

    for (const auto& ray : rays) {
      hits.push_back(talus::dynamics::castRay(world, ray[0], ray[1]));
    }

    const double tree =
      std::chrono::duration<double, std::micro>(Clock::now() - start).count()
      / static_cast<double>(rays.size());
    const Clock::time_point scanStart = Clock::now();
    uint32_t bodyHits = 0u;

    for (uint32_t index = 0u; index != 200u; ++index) {
      const std::optional<talus::dynamics::WorldRayHit> flat =
        talus::dynamics::castRay(scan, rays[index][0], rays[index][1]);

      REQUIRE(flat.has_value() == hits[index].has_value());

      if (flat) {
        CHECK(flat->side == hits[index]->side);
        CHECK(flat->body == hits[index]->body);
        CHECK(flat->fraction == hits[index]->fraction);
        bodyHits += flat->side == Side::Rigid ? 1u : 0u;
      }
    }

    const double scanned =
      std::chrono::duration<double, std::micro>(Clock::now() - scanStart)
        .count()
      / 200.0;

    CHECK(bodyHits > 50u);
    std::printf(
      "raycost bodies %u tree_us %.3f scan_us %.3f\n",
      count,
      tree,
      scanned
    );
    treeUs[run] = tree;
  }

  const SoftBlock block = softBlock(glm::dvec3(0.2, 0.05, 0.2), 0.05);
  World world;

  world.parameters.gravity = glm::dvec3(0.0);
  (void)addSoftBlock(world, block, glm::dvec3(0.0, 0.5, 0.0));

  const Clock::time_point start = Clock::now();
  uint32_t found = 0u;

  for (uint32_t index = 0u; index != 1000u; ++index) {
    const double x = 0.001 * static_cast<double>(index % 100u);

    if (talus::dynamics::castRay(
          world,
          glm::dvec3(x, 2.0, 0.0),
          glm::dvec3(x, -1.0, 0.0)
        )) {
      found += 1u;
    }
  }

  const double lattice =
    std::chrono::duration<double, std::micro>(Clock::now() - start).count()
    / 1000.0;

  CHECK(found == 1000u);
  std::printf(
    "raycost lattice cells %zu us %.3f\n",
    block.mesh->hexes.size(),
    lattice
  );
  CHECK(treeUs[1] < 5.0 * treeUs[0]);
}
