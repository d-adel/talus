#include <catch2/catch_test_macros.hpp>

#include "meshfixtures.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/meshbody/meshbody.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/dynamics/tiers/tiers.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <vector>

namespace {

using namespace talus::dynamics;
using meshfixture::Mesh;

const glm::dquat noTurn(1.0, 0.0, 0.0, 0.0);

struct Threads {
  std::shared_ptr<Scheduler> scheduler;
  Workers workers = resolveWorkers(8u, nullptr, nullptr, nullptr, scheduler);
};

struct Costs {
  double step = 0.0;
  double fill = 0.0;
  double worstFill = 0.0;
  double narrowphase = 0.0;
  double solver = 0.0;
  double contacts = 0.0;
};

struct Timings {
  Costs sum;
  uint32_t steps = 0u;

  void add(const StepReport& report) {
    sum.step += report.profile.total;
    sum.fill += report.profile.pairs.build;
    sum.worstFill = std::max(sum.worstFill, report.profile.pairs.build);
    sum.narrowphase += report.profile.pairs.narrowphase;
    sum.solver += report.profile.prepare + report.profile.substeps;
    sum.contacts += static_cast<double>(report.contacts);
    steps += 1u;
  }

  [[nodiscard]] Costs mean() const {
    const double count = static_cast<double>(std::max(steps, 1u));

    return {
      sum.step / count,
      sum.fill / count,
      sum.worstFill,
      sum.narrowphase / count,
      sum.solver / count,
      sum.contacts / count
    };
  }
};

const RigidBody& bodyOf(const World& world, uint32_t id) {
  const std::optional<BodyRef> ref = findBody(world.bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == Side::Rigid);

  return world.bodies.rigids[ref->index];
}

uint32_t addBoxFloor(World& world, const glm::dvec3& half) {
  CompoundChild slab;

  slab.half = half;
  slab.density = 1000.0;

  return world.bodies
    .rigids[addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{slab},
      glm::dvec3(0.0, -half.y, 0.0),
      noTurn,
      glm::dvec3(0.0),
      0.5,
      0.0,
      true
    )]
    .id;
}

uint32_t addBoxBody(
  World& world,
  const glm::dvec3& half,
  double mass,
  const glm::dvec3& at
) {
  CompoundChild box;

  box.half = half;
  box.density = mass / (8.0 * half.x * half.y * half.z);

  return world.bodies
    .rigids[addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{box},
      at,
      noTurn,
      glm::dvec3(0.0),
      0.5,
      0.0,
      false
    )]
    .id;
}

enum Part : uint32_t {
  body,
  head,
  thigh,
  shin,
  foot,
  arm,
  hand,
  partKinds
};

struct PartShape {
  glm::dvec3 half{0.0};
  double mass = 0.0;
};

constexpr std::array<PartShape, partKinds> partShapes{
  PartShape{{0.27, 0.26, 0.252}, 22.0},
  PartShape{{0.21, 0.15, 0.195}, 8.0},
  PartShape{{0.09, 0.045, 0.09}, 2.0},
  PartShape{{0.09, 0.035, 0.095}, 1.5},
  PartShape{{0.105, 0.065, 0.13}, 1.25},
  PartShape{{0.07, 0.14, 0.075}, 1.75},
  PartShape{{0.045, 0.11, 0.09}, 0.75}
};

constexpr double partGap = 0.01;

struct PlacedPart {
  Part kind = body;
  glm::dvec3 centre{0.0};
};

std::array<PlacedPart, 12> characterLayout() {
  const auto halfOf = [](Part kind) {
    return partShapes[kind].half;
  };
  const double footY = halfOf(foot).y;
  const double shinY = 2.0 * halfOf(foot).y + partGap + halfOf(shin).y;
  const double thighY = shinY + halfOf(shin).y + partGap + halfOf(thigh).y;
  const double bodyY = thighY + halfOf(thigh).y + partGap + halfOf(body).y;
  const double headY = bodyY + halfOf(body).y + partGap + halfOf(head).y;
  const double armY = bodyY + halfOf(body).y - halfOf(arm).y;
  const double handY = armY - halfOf(arm).y - partGap - halfOf(hand).y;
  const double legX = 0.165;
  const double armX = halfOf(body).x + partGap + halfOf(arm).x;

  return {
    PlacedPart{body, {0.0, bodyY, 0.0}},
    PlacedPart{head, {0.0, headY, 0.0}},
    PlacedPart{thigh, {-legX, thighY, 0.0}},
    PlacedPart{shin, {-legX, shinY, 0.0}},
    PlacedPart{foot, {-legX, footY, 0.0}},
    PlacedPart{thigh, {legX, thighY, 0.0}},
    PlacedPart{shin, {legX, shinY, 0.0}},
    PlacedPart{foot, {legX, footY, 0.0}},
    PlacedPart{arm, {-armX, armY, 0.0}},
    PlacedPart{hand, {-armX, handY, 0.0}},
    PlacedPart{arm, {armX, armY, 0.0}},
    PlacedPart{hand, {armX, handY, 0.0}}
  };
}

constexpr std::array<std::array<uint32_t, 2>, 11> characterFilters{
  std::array<uint32_t, 2>{0u, 9u},
  std::array<uint32_t, 2>{0u, 11u},
  std::array<uint32_t, 2>{9u, 11u},
  std::array<uint32_t, 2>{0u, 3u},
  std::array<uint32_t, 2>{0u, 6u},
  std::array<uint32_t, 2>{0u, 4u},
  std::array<uint32_t, 2>{0u, 7u},
  std::array<uint32_t, 2>{4u, 7u},
  std::array<uint32_t, 2>{3u, 6u},
  std::array<uint32_t, 2>{4u, 6u},
  std::array<uint32_t, 2>{7u, 3u}
};

constexpr std::array<std::array<uint32_t, 2>, 11> characterJoints{
  std::array<uint32_t, 2>{0u, 1u},
  std::array<uint32_t, 2>{0u, 2u},
  std::array<uint32_t, 2>{2u, 3u},
  std::array<uint32_t, 2>{3u, 4u},
  std::array<uint32_t, 2>{0u, 5u},
  std::array<uint32_t, 2>{5u, 6u},
  std::array<uint32_t, 2>{6u, 7u},
  std::array<uint32_t, 2>{0u, 8u},
  std::array<uint32_t, 2>{8u, 9u},
  std::array<uint32_t, 2>{0u, 10u},
  std::array<uint32_t, 2>{10u, 11u}
};

const std::array<MeshShape, partKinds>& partMeshes() {
  static const std::array<MeshShape, partKinds> shapes = [] {
    const Threads threads;
    std::array<MeshShape, partKinds> made;

    for (uint32_t kind = 0u; kind != partKinds; ++kind) {
      const glm::vec3 half(partShapes[kind].half);
      const Mesh mesh = meshfixture::box(-half, half);
      MeshShapeSettings settings;

      settings.mass = partShapes[kind].mass;
      made[kind] =
        meshShape(mesh.vertices, mesh.indices, settings, threads.workers);
    }

    return made;
  }();

  return shapes;
}

glm::dvec3 jointPoint(const PlacedPart& child) {
  const double edge = partShapes[child.kind].half.y + 0.5 * partGap;

  return child.centre + glm::dvec3(0.0, child.kind == head ? -edge : edge, 0.0);
}

struct Character {
  std::array<uint32_t, 12> parts{};
  uint32_t holder = 0u;
  glm::dvec3 rest{0.0};
  glm::dquat turn{1.0, 0.0, 0.0, 0.0};
};

Character addCharacter(World& world, const glm::dvec3& at, bool meshParts) {
  const std::array<PlacedPart, 12> layout = characterLayout();
  Character character;

  for (size_t index = 0u; index != layout.size(); ++index) {
    const PlacedPart& part = layout[index];

    character.parts[index] = meshParts ? addMeshBody(
                                           world.bodies,
                                           partMeshes()[part.kind],
                                           at + part.centre,
                                           noTurn,
                                           glm::dvec3(0.0)
                                         )
                                       : addBoxBody(
                                           world,
                                           partShapes[part.kind].half,
                                           partShapes[part.kind].mass,
                                           at + part.centre
                                         );
  }

  for (const auto& [parent, child] : characterJoints) {
    const glm::dvec3 point = at + jointPoint(layout[child]);
    const RigidBody& a = bodyOf(world, character.parts[parent]);
    const RigidBody& b = bodyOf(world, character.parts[child]);
    const glm::dquat back = glm::conjugate(a.rotation);
    JointDef def;

    def.bodyA = a.id;
    def.bodyB = b.id;

    if (layout[child].kind == head) {
      def.kind = JointKind::Spring;
      def.frameA = {back * (b.position - a.position), back * b.rotation};
      def.spring.linearStiffness = 1.0e5;
      def.spring.linearDamping = 1.0e3;
      def.spring.angularStiffness = 5.0e3;
      def.spring.angularDamping = 100.0;
    } else {
      def.kind = JointKind::Ball;
      def.frameA.position = back * (point - a.position);
      def.frameB.position = glm::conjugate(b.rotation) * (point - b.position);
    }

    REQUIRE(addJoint(world, def) != 0u);
  }

  for (const auto& [first, second] : characterFilters) {
    JointDef filter;

    filter.kind = JointKind::Spring;
    filter.bodyA = character.parts[first];
    filter.bodyB = character.parts[second];
    filter.spring.maxForce = 0.0;
    filter.spring.maxTorque = 0.0;
    REQUIRE(addJoint(world, filter) != 0u);
  }

  const RigidBody& torso = bodyOf(world, character.parts[0]);
  JointDef holder;

  character.rest = torso.position;
  character.turn = torso.rotation;
  holder.kind = JointKind::Spring;
  holder.bodyB = torso.id;
  holder.spring.target = {character.rest, character.turn};
  holder.spring.linearStiffness = 2.0e5;
  holder.spring.linearDamping = 4.0e3;
  holder.spring.angularStiffness = 2.0e4;
  holder.spring.angularDamping = 1.25e3;
  character.holder = addJoint(world, holder);
  REQUIRE(character.holder != 0u);

  return character;
}

double lowestPoint(const World& world, uint32_t id, const glm::dvec3& half) {
  const RigidBody& part = bodyOf(world, id);
  const glm::dquat turn = part.shape == RigidShape::Sampled
                            ? meshFrame(part).rotation
                            : part.rotation;
  const glm::dvec3 centre = part.shape == RigidShape::Sampled
                              ? meshFrame(part).position
                              : part.position;
  double lowest = std::numeric_limits<double>::infinity();

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::dvec3 sign(
      (corner & 1u) != 0u ? 1.0 : -1.0,
      (corner & 2u) != 0u ? 1.0 : -1.0,
      (corner & 4u) != 0u ? 1.0 : -1.0
    );

    lowest = std::min(lowest, (centre + turn * (sign * half)).y);
  }

  return lowest;
}

std::array<uint32_t, 2> pairCensus(const World& world) {
  std::array<uint32_t, 2> census{0u, 0u};

  const auto awake = [&](const BodyRef& ref) {
    return ref.side == Side::Rigid && !world.bodies.rigids[ref.index].asleep
           && world.bodies.rigids[ref.index].mass > 0.0;
  };

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot)) {
      continue;
    }

    const CachedBodyPair& pair = world.manifolds.at(slot);

    if (awake(pair.first) || awake(pair.second)) {
      census[0] += 1u;
      census[1] += pair.points.empty() ? 0u : 1u;
    }
  }

  return census;
}

uint64_t stateHash(const World& world) {
  uint64_t hash = 0xCBF29CE484222325ull;
  const auto mix = [&](double value) {
    hash = (hash ^ std::bit_cast<uint64_t>(value)) * 0x100000001B3ull;
  };

  for (const RigidBody& rigid : world.bodies.rigids) {
    for (uint32_t axis = 0u; axis != 3u; ++axis) {
      mix(rigid.position[axis]);
      mix(rigid.velocity[axis]);
      mix(rigid.angularVelocity[axis]);
    }

    mix(rigid.rotation.x);
    mix(rigid.rotation.y);
    mix(rigid.rotation.z);
    mix(rigid.rotation.w);
  }

  return hash;
}

bool allAsleep(const World& world) {
  return std::ranges::all_of(world.bodies.rigids, [](const RigidBody& rigid) {
    return rigid.asleep || rigid.mass == 0.0;
  });
}

}

TEST_CASE("fill cost of a mesh slab sliding on a box", "[.][fillcost]") {
  for (const double side : {0.25, 0.5, 1.0}) {
    const glm::vec3 half(
      static_cast<float>(0.5 * side),
      0.05f,
      static_cast<float>(0.5 * side)
    );
    const Mesh slab = meshfixture::box(-half, half);
    const Threads threads;
    MeshShapeSettings settings;

    settings.mass = 500.0 * side * side;
    settings.friction = 0.0;

    const MeshShape shape =
      meshShape(slab.vertices, slab.indices, settings, threads.workers);
    World world;

    world.parameters.timePairs = true;
    (void)addBoxFloor(world, glm::dvec3(4.0, 0.5, 4.0));
    (void)addMeshBody(
      world.bodies,
      shape,
      glm::dvec3(-1.0, 0.05, 0.0),
      noTurn,
      glm::dvec3(1.0, 0.0, 0.0)
    );

    Timings timings;

    for (uint32_t frame = 0u; frame != 90u; ++frame) {
      const StepReport report = step(world);

      REQUIRE_FALSE(report.nonFinite);

      if (frame >= 10u) {
        timings.add(report);
      }
    }

    const Costs result = timings.mean();

    std::printf(
      "fillcost side %.2f m contacts %.0f step_ms %.3f fill_ms %.4f "
      "worst_fill_ms %.4f narrowphase_ms %.4f state %016llx\n",
      side,
      result.contacts,
      result.step,
      result.fill,
      result.worstFill,
      result.narrowphase,
      static_cast<unsigned long long>(stateHash(world))
    );
  }
}

TEST_CASE(
  "step cost of characters built from mesh parts",
  "[.][meshbodycost]"
) {
  const glm::dvec3 floorHalf(25.0, 0.5, 25.0);
  constexpr double crateHalf = 0.05;
  constexpr double crateMass = 1.5;
  constexpr uint32_t warmupSteps = 30u;
  constexpr uint32_t measuredSteps = 90u;
  constexpr double swayMetres = 0.03;
  constexpr double swayHertz = 0.5;

  (void)partMeshes();

  for (const uint32_t crates : {0u, 5000u}) {
    for (const bool meshParts : {false, true}) {
      World world;

      world.parameters.timePairs = true;
      (void)addBoxFloor(world, floorHalf);

      const uint32_t side = static_cast<uint32_t>(
        std::ceil(std::sqrt(static_cast<double>(crates)))
      );

      for (uint32_t index = 0u; index != crates; ++index) {
        (void)addBoxBody(
          world,
          glm::dvec3(crateHalf),
          crateMass,
          glm::dvec3(
            0.3 * static_cast<double>(index % side),
            crateHalf,
            0.3
              * (static_cast<double>(index / side) - 0.5 * static_cast<double>(side))
          )
        );
      }

      for (uint32_t settle = 0u; settle != 900u && !allAsleep(world);
           ++settle) {
        REQUIRE_FALSE(step(world).nonFinite);
      }

      REQUIRE(allAsleep(world));

      std::vector<Character> characters;

      for (uint32_t index = 0u; index != 4u; ++index) {
        characters.push_back(addCharacter(
          world,
          glm::dvec3(-3.0, 0.0, -4.5 + 3.0 * static_cast<double>(index)),
          meshParts
        ));
      }

      Timings timings;
      double awake = 0.0;

      for (uint32_t frame = 0u; frame != warmupSteps + measuredSteps; ++frame) {
        const double time = static_cast<double>(frame + 1u) / 60.0;
        const glm::dvec3 sway(
          swayMetres * std::sin(2.0 * std::numbers::pi * swayHertz * time),
          0.0,
          0.0
        );

        for (const Character& character : characters) {
          REQUIRE(setSpringTarget(
            world,
            character.holder,
            {character.rest + sway, character.turn}
          ));
        }

        const StepReport report = step(world);

        REQUIRE_FALSE(report.nonFinite);

        if (frame >= warmupSteps) {
          timings.add(report);
          awake += static_cast<double>(awakeRigids(world).size());
        }
      }

      double lowest = std::numeric_limits<double>::infinity();
      const std::array<PlacedPart, 12> layout = characterLayout();

      for (const Character& character : characters) {
        for (size_t index = 0u; index != layout.size(); ++index) {
          lowest = std::min(
            lowest,
            lowestPoint(
              world,
              character.parts[index],
              partShapes[layout[index].kind].half
            )
          );
        }
      }

      const Costs result = timings.mean();
      const std::array<uint32_t, 2> census = pairCensus(world);

      std::printf(
        "meshbodycost parts %s crates %u awake %.1f pairs %u touching %u "
        "contacts %.0f step_ms %.3f fill_ms %.4f worst_fill_ms %.4f "
        "narrowphase_ms %.4f solver_ms %.4f lowest_mm %.2f state %016llx\n",
        meshParts ? "mesh" : "box",
        crates,
        awake / static_cast<double>(measuredSteps),
        census[0],
        census[1],
        result.contacts,
        result.step,
        result.fill,
        result.worstFill,
        result.narrowphase,
        result.solver,
        1000.0 * lowest,
        static_cast<unsigned long long>(stateHash(world))
      );
      CHECK(lowest >= -0.01);
    }
  }
}
