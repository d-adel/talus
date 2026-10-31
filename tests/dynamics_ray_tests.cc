#include <catch2/catch_test_macros.hpp>

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/rays/worldray.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

namespace {

using talus::dynamics::BodyRef;
using talus::dynamics::CompoundChild;
using talus::dynamics::noChild;
using talus::dynamics::RaySkip;
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
  const uint32_t old = addBox(world, glm::dvec3(0.3), glm::dvec3(8.0, 0.0, 0.0));

  (void)talus::dynamics::step(world);

  const uint32_t added = addBox(world, glm::dvec3(0.3), glm::dvec3(0.0, 0.0, 0.0));

  REQUIRE(added >= world.broadPhase.seenBodies());
  std::swap(world.bodies.rigids[1], world.bodies.rigids[2]);
  REQUIRE(world.bodies.rigids.back().id == old);

  const auto hit = talus::dynamics::castRay(world, glm::dvec3(0.0, 3.0, 0.0), glm::dvec3(0.0, -3.0, 0.0));

  REQUIRE(hit.has_value());
  CHECK(hit->body == added);

  const auto oldHit = talus::dynamics::castRay(world, glm::dvec3(8.0, 3.0, 0.0), glm::dvec3(8.0, -3.0, 0.0));

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
