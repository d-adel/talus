#include <catch2/catch_test_macros.hpp>

#include "bowlfixture.h"
#include "meshfixtures.h"

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/meshbody/meshbody.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/scheduler/scheduler.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <tuple>
#include <utility>
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

uint32_t addAnchoredBox(
  World& world,
  const glm::dvec3& half,
  const glm::dvec3& centre
) {
  CompoundChild slab;

  slab.half = half;
  slab.density = 1000.0;

  return world.bodies
    .rigids[addCompoundBody(
      world.bodies,
      std::vector<CompoundChild>{slab},
      centre,
      noTurn,
      glm::dvec3(0.0),
      0.5,
      0.0,
      true
    )]
    .id;
}

uint32_t addBoxFloor(World& world, const glm::dvec3& half) {
  return addAnchoredBox(world, half, glm::dvec3(0.0, -half.y, 0.0));
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

std::array<PlacedPart, 12> handsByLegsLayout() {
  const auto halfOf = [](Part kind) {
    return partShapes[kind].half;
  };
  const double footY = halfOf(foot).y;
  const double shinY = 2.0 * halfOf(foot).y + partGap + halfOf(shin).y;
  const double thighY = shinY + halfOf(shin).y + partGap + halfOf(thigh).y;
  const double bodyY = thighY + halfOf(thigh).y + partGap + halfOf(body).y;
  const double headY = bodyY + halfOf(body).y + partGap + halfOf(head).y;
  const double handY = thighY;
  const double armY = handY + halfOf(hand).y + partGap + halfOf(arm).y;
  const double armX = halfOf(body).x + partGap + halfOf(arm).x;
  const double legX = armX - halfOf(hand).x - partGap - halfOf(thigh).x;

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

using PartSpacing = std::array<double, partKinds>;

Mesh roundedMesh(const glm::vec3& half) {
  Mesh mesh = meshfixture::icosphere(1.0f, 3u);

  for (glm::vec3& vertex : mesh.vertices) {
    vertex *= half;
  }

  return mesh;
}

const MeshShape& partMesh(Part kind, double spacing, bool rounded = false) {
  static std::map<std::tuple<uint32_t, double, bool>, MeshShape> shapes;
  const std::tuple<uint32_t, double, bool> key{kind, spacing, rounded};
  const auto found = shapes.find(key);

  if (found != shapes.end()) {
    return found->second;
  }

  const Threads threads;
  const glm::vec3 half(partShapes[kind].half);
  const Mesh mesh = rounded ? roundedMesh(half) : meshfixture::box(-half, half);
  MeshShapeSettings settings;

  settings.mass = partShapes[kind].mass;
  settings.spacing = spacing;

  return shapes
    .emplace(
      key,
      meshShape(mesh.vertices, mesh.indices, settings, threads.workers)
    )
    .first->second;
}

std::vector<std::array<uint32_t, 2>> gameFilters() {
  const std::array<std::array<uint32_t, 2>, 4> touching{
    std::array<uint32_t, 2>{1u, 8u},
    std::array<uint32_t, 2>{1u, 9u},
    std::array<uint32_t, 2>{1u, 10u},
    std::array<uint32_t, 2>{1u, 11u}
  };
  std::vector<std::array<uint32_t, 2>> filters;

  for (uint32_t first = 0u; first != 12u; ++first) {
    for (uint32_t second = first + 1u; second != 12u; ++second) {
      const std::array<uint32_t, 2> pair{first, second};

      if (std::ranges::find(characterJoints, pair) == characterJoints.end()
          && std::ranges::find(touching, pair) == touching.end()) {
        filters.push_back(pair);
      }
    }
  }

  return filters;
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

Character addCharacter(
  World& world,
  const glm::dvec3& at,
  const PartSpacing* spacing,
  std::span<const std::array<uint32_t, 2>> filters,
  const std::array<PlacedPart, 12>& layout = characterLayout(),
  bool rounded = false
) {
  Character character;

  for (size_t index = 0u; index != layout.size(); ++index) {
    const PlacedPart& part = layout[index];

    character.parts[index] =
      spacing != nullptr
        ? addMeshBody(
            world.bodies,
            partMesh(part.kind, (*spacing)[part.kind], rounded),
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

  for (const auto& [first, second] : filters) {
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

struct StoredCensus {
  uint32_t stored = 0u;
  uint32_t beyondBand = 0u;
  uint32_t betweenParts = 0u;
};

StoredCensus storedCensus(const World& world) {
  const double band =
    world.parameters.speculativeDistance + world.parameters.margin;
  StoredCensus census;

  for (uint32_t slot = 0u; slot != world.manifolds.size(); ++slot) {
    if (!world.manifolds.live(slot)) {
      continue;
    }

    const CachedBodyPair& pair = world.manifolds.at(slot);
    const bool parts =
      pair.second.side == Side::Rigid
      && world.bodies.rigids[pair.first.index].shape == RigidShape::Sampled
      && world.bodies.rigids[pair.second.index].shape == RigidShape::Sampled;

    for (const CachedContactPoint& point : pair.points) {
      census.stored += 1u;
      census.beyondBand += point.separation >= band ? 1u : 0u;
      census.betweenParts += parts ? 1u : 0u;
    }
  }

  return census;
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
  "fill cost of a mesh ball spinning in a field bowl",
  "[.][fillcost]"
) {
  const Threads threads;
  const Mesh sphere = meshfixture::icosphere(0.25f, 4u);
  MeshShapeSettings settings;

  settings.mass = 50.0;
  settings.friction = 0.0;

  const MeshShape ball =
    meshShape(sphere.vertices, sphere.indices, settings, threads.workers);

  for (const float radius : {0.30f, 0.26f}) {
    const talus::field::SampledSdf bowl = bowlfixture::field(radius);
    World world;

    world.parameters.timePairs = true;
    (void)addBoxFloor(world, glm::dvec3(4.0, 0.5, 4.0));

    const glm::dvec3 bowlAt(0.0, static_cast<double>(bowlfixture::half.y), 0.0);

    (void)bowlfixture::addBowl(world.bodies, bowl, bowlAt, 500.0, 0.0);

    const double drop = static_cast<double>(radius) - 0.25 + 0.0005;
    const uint32_t ballId = addMeshBody(
      world.bodies,
      ball,
      bowlAt + glm::dvec3(bowlfixture::cavity(radius))
        - glm::dvec3(0.0, drop, 0.0),
      noTurn,
      glm::dvec3(0.0)
    );
    const std::optional<BodyRef> spun = findBody(world.bodies, ballId);

    REQUIRE(spun.has_value());
    world.bodies.rigids[spun->index].angularVelocity =
      glm::dvec3(0.0, 1.0, 3.0);

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
      "fillcost ball in bowl %.2f m contacts %.0f step_ms %.3f fill_ms %.4f "
      "worst_fill_ms %.4f narrowphase_ms %.4f state %016llx\n",
      static_cast<double>(radius),
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

  struct Run {
    const char* parts = "box";
    std::optional<PartSpacing> spacing;
    bool game = false;
    uint32_t crates = 0u;
  };

  const PartSpacing two{0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02};
  const PartSpacing three{0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03};
  const PartSpacing four{0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04};
  const PartSpacing game{0.03, 0.03, 0.02, 0.02, 0.04, 0.02, 0.02};
  const std::vector<std::array<uint32_t, 2>> original(
    characterFilters.begin(),
    characterFilters.end()
  );
  const std::vector<std::array<uint32_t, 2>> all = gameFilters();
  const std::vector<Run> runs{
    {"box", std::nullopt, false, 0u},
    {"box", std::nullopt, false, 5000u},
    {"mesh2cm", two, false, 0u},
    {"mesh2cm", two, false, 5000u},
    {"box", std::nullopt, true, 0u},
    {"mesh2cm", two, true, 0u},
    {"meshgame", game, false, 0u},
    {"meshgame", game, true, 0u},
    {"mesh3cm", three, false, 0u},
    {"mesh3cm", three, true, 0u},
    {"mesh4cm", four, false, 0u},
    {"mesh4cm", four, true, 0u}
  };

  for (const Run& configuration : runs) {
    const uint32_t crates = configuration.crates;
    const std::span<const std::array<uint32_t, 2>> filters =
      configuration.game ? std::span(all) : std::span(original);
    World world;

    world.parameters.timePairs = true;
    (void)addBoxFloor(world, floorHalf);

    const uint32_t side =
      static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(crates))));

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

    for (uint32_t settle = 0u; settle != 900u && !allAsleep(world); ++settle) {
      REQUIRE_FALSE(step(world).nonFinite);
    }

    REQUIRE(allAsleep(world));

    std::vector<Character> characters;

    for (uint32_t index = 0u; index != 4u; ++index) {
      characters.push_back(addCharacter(
        world,
        glm::dvec3(-3.0, 0.0, -4.5 + 3.0 * static_cast<double>(index)),
        configuration.spacing ? &*configuration.spacing : nullptr,
        filters
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
      "meshbodycost parts %s filters %zu crates %u awake %.1f pairs %u "
      "touching %u "
      "contacts %.0f step_ms %.3f fill_ms %.4f worst_fill_ms %.4f "
      "narrowphase_ms %.4f solver_ms %.4f lowest_mm %.2f state %016llx\n",
      configuration.parts,
      filters.size(),
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

TEST_CASE("step cost of a mesh character turning on a kerb", "[.][turncost]") {
  constexpr double kerbHeight = 0.15;
  constexpr double turnRate = 3.0;
  constexpr double swingRate = 8.0;
  const PartSpacing game{0.03, 0.03, 0.02, 0.02, 0.04, 0.02, 0.02};
  const std::vector<std::array<uint32_t, 2>> armsTouchLegs(
    characterFilters.begin(),
    characterFilters.end()
  );
  const std::vector<std::array<uint32_t, 2>> onlyHead = gameFilters();
  const std::array<PlacedPart, 12> layout = handsByLegsLayout();

  struct Phase {
    const char* name = "";
    uint32_t steps = 0u;
    double rate = 0.0;
    double swing = 0.0;
    double step = 0.0;
    double worstStep = 0.0;
    double narrowphase = 0.0;
    double fill = 0.0;
    double solver = 0.0;
    double stored = 0.0;
    double active = 0.0;
    double beyondBand = 0.0;
    double betweenParts = 0.0;
    uint32_t worstStored = 0u;
    uint32_t activeAtWorst = 0u;
    StepReport slowest{};
  };

  for (const std::vector<std::array<uint32_t, 2>>* filters :
       {&onlyHead, &armsTouchLegs}) {
    for (const uint32_t workers : {1u, 8u}) {
      World world;

      world.workerCount = workers;
      world.parameters.timePairs = true;
      (void)addBoxFloor(world, glm::dvec3(4.0, 0.5, 4.0));
      (void)addAnchoredBox(
        world,
        glm::dvec3(0.6, 0.5 * kerbHeight, 0.4),
        glm::dvec3(0.0, 0.5 * kerbHeight, 0.0)
      );

      const Character character = addCharacter(
        world,
        glm::dvec3(0.0, kerbHeight, 0.15),
        &game,
        *filters,
        layout,
        true
      );

      std::array<Phase, 4> phases{
        Phase{"stand", 60u, 0.0, 0.0},
        Phase{"about", 60u, 0.0, std::numbers::pi},
        Phase{"spin", 120u, turnRate, 0.0},
        Phase{"stop", 60u, 0.0, 0.0}
      };
      double yaw = 0.0;

      for (Phase& phase : phases) {
        const double aim = yaw + phase.swing;

        for (uint32_t frame = 0u; frame != phase.steps; ++frame) {
          const double ease =
            phase.swing != 0.0
              ? 1.0 - std::exp(-swingRate * world.parameters.dt)
              : 0.0;

          yaw += phase.rate * world.parameters.dt + (aim - yaw) * ease;
          REQUIRE(setSpringTarget(
            world,
            character.holder,
            {character.rest,
             glm::angleAxis(yaw, glm::dvec3(0.0, 1.0, 0.0)) * character.turn}
          ));

          const StepReport report = step(world);
          const StoredCensus census = storedCensus(world);

          REQUIRE_FALSE(report.nonFinite);
          phase.step += report.profile.total;

          if (report.profile.total > phase.worstStep) {
            phase.worstStep = report.profile.total;
            phase.slowest = report;
          }

          phase.narrowphase += report.profile.pairs.narrowphase;
          phase.fill += report.profile.pairs.build;
          phase.solver += report.profile.prepare + report.profile.substeps;
          phase.stored += static_cast<double>(report.contacts);
          phase.active += static_cast<double>(report.activeContacts);
          phase.beyondBand += static_cast<double>(census.beyondBand);
          phase.betweenParts += static_cast<double>(census.betweenParts);

          if (report.contacts > phase.worstStored) {
            phase.worstStored = report.contacts;
            phase.activeAtWorst = report.activeContacts;
          }
        }
      }

      for (const Phase& phase : phases) {
        const double count = static_cast<double>(phase.steps);

        std::printf(
          "turncost filters %zu workers %u phase %s stored %.0f max_stored %u "
          "active %.1f active_at_max %u beyond_band %.0f between_parts %.0f "
          "step_ms %.3f narrowphase_ms %.4f fill_ms %.4f solver_ms %.4f "
          "slowest_ms %.3f slowest_stored %u slowest_active %u "
          "slowest_narrowphase_ms %.4f slowest_fill_ms %.4f "
          "slowest_solver_ms %.4f\n",
          filters->size(),
          workers,
          phase.name,
          phase.stored / count,
          phase.worstStored,
          phase.active / count,
          phase.activeAtWorst,
          phase.beyondBand / count,
          phase.betweenParts / count,
          phase.step / count,
          phase.narrowphase / count,
          phase.fill / count,
          phase.solver / count,
          phase.worstStep,
          phase.slowest.contacts,
          phase.slowest.activeContacts,
          phase.slowest.profile.pairs.narrowphase,
          phase.slowest.profile.pairs.build,
          phase.slowest.profile.prepare + phase.slowest.profile.substeps
        );
      }

      const MeshFrame torso = meshFrame(bodyOf(world, character.parts[0]));

      std::printf(
        "turncost filters %zu workers %u yaw %.3f torso_y %.4f state %016llx\n",
        filters->size(),
        workers,
        2.0 * std::atan2(torso.rotation.y, torso.rotation.w),
        torso.position.y,
        static_cast<unsigned long long>(stateHash(world))
      );
    }
  }
}
