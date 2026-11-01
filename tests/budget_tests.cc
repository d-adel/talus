#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "fieldprimitives.h"
#include "destructionfixtures.h"

#include <talus/destruction/budget/budget.h>
#include <talus/destruction/building/building.h>
#include <talus/destruction/chunker/chunker.h>
#include <talus/destruction/destructionworld/destructionworld.h>
#include <talus/destruction/destructionworld/loadqueue.h>
#include <talus/destruction/materials/materials.h>
#include <talus/destruction/stress/stress.h>
#include <talus/destruction/structure/structure.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace {

using talus::destruction::CostModel;
using talus::destruction::StepCost;
using talus::destruction::StepCounts;

constexpr double PREDICTION_TOLERANCE_MS = 1.0e-12;
constexpr uint32_t BLOCK_KEY = 9000u;

const glm::dquat LEVEL(1.0, 0.0, 0.0, 0.0);

uint32_t addSlider(talus::dynamics::World& world, const glm::dvec3& centre, const glm::dvec3& velocity) {
  const std::array<talus::dynamics::CompoundChild, 1> block{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(0.25), 1000.0, BLOCK_KEY}
  };
  const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, block, centre, LEVEL, velocity, 0.6, 0.0, false);

  return world.bodies.rigids[index].id;
}

}

TEST_CASE("the cost model predicts each group as the sum of its coefficients times its counts", "[destruction][budget]") {
  const CostModel& model = talus::destruction::costModel;
  const std::array<StepCounts, 3> sets{{
    {0u, 0u, 1u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u},
    {4000u, 9000u, 15795u, 900u, 700u, 30000u, 3u, 120u, 2500000u, 0u, 40000u, 300u},
    {12u, 30u, 3000u, 5u, 4u, 20u, 0u, 0u, 1000u, 7000000u, 9u, 2u},
  }};

  for (const StepCounts& counts : sets) {
    const double rigid =
      model.rigid[0] * counts.moving + model.rigid[1] * counts.awakeChunks + model.rigid[2] * counts.bodies;
    const double update = model.update[0] * counts.visited + model.update[1] * counts.gathered
                          + model.update[2] * counts.awakeManifolds + model.update[3] * counts.fractures
                          + model.update[4] * counts.broken + model.update[5] * counts.bodies;
    const double stress = model.stress[0] * static_cast<double>(counts.factorWork)
                          + model.stress[1] * static_cast<double>(counts.wideWork)
                          + model.stress[2] * counts.solvedChunks + model.stress[3] * counts.updates;
    const StepCost cost = talus::destruction::predictCost(counts);

    CHECK(std::abs(cost.rigid - rigid) <= PREDICTION_TOLERANCE_MS);
    CHECK(std::abs(cost.update - update) <= PREDICTION_TOLERANCE_MS);
    CHECK(std::abs(cost.stress - stress) <= PREDICTION_TOLERANCE_MS);
    CHECK(std::abs(cost.total - (rigid + update + stress)) <= PREDICTION_TOLERANCE_MS);
  }
}

TEST_CASE("a body whose fastest point stays under 0.3 m/s for half a second rests, and only the others count as moving", "[destruction][budget]") {
  talus::dynamics::World world;
  talus::destruction::DestructionWorld destruction;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t slow = addSlider(world, glm::dvec3(0.0, 5.0, 0.0), glm::dvec3(0.2, 0.0, 0.0));
  const uint32_t fast = addSlider(world, glm::dvec3(0.0, 5.0, 10.0), glm::dvec3(0.4, 0.0, 0.0));

  for (uint32_t step = 1u; step <= 31u; ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const talus::destruction::DestructionReport report = destruction.update(world);

    INFO("step " << step);
    REQUIRE(report.bodies == 2u);

    if (step == 29u) {
      CHECK(report.moving == 2u);
      CHECK(!destruction.rest().resting(slow));
    }

    if (step == 31u) {
      CHECK(report.moving == 1u);
      CHECK(destruction.rest().resting(slow));
      CHECK(!destruction.rest().resting(fast));
      CHECK(destruction.rest().restSteps(slow) == 31u);
    }
  }
}

namespace {

using talus::destruction::ChunkLoad;
using talus::destruction::StressState;
using talus::destruction::StructureState;

constexpr uint64_t SOLVED_REVISION = 7u;
const glm::dvec3 DOWN(0.0, -9.81, 0.0);

struct Solved {
  StressState stress;
  StructureState structure;
};

Solved solvedColumn(double utilization) {
  Solved solved;

  solved.stress.loads = {
    ChunkLoad{1u, glm::dvec3(30.0, -1000.0, 0.0), glm::dvec3(0.0, 0.0, 40.0)},
    ChunkLoad{3u, glm::dvec3(0.0, -2500.0, 10.0), glm::dvec3(5.0, 0.0, 0.0)},
  };
  solved.stress.gravity = DOWN;
  solved.stress.revision = SOLVED_REVISION;
  solved.stress.solved = true;
  solved.stress.utilization = utilization;
  solved.structure.revision = SOLVED_REVISION;

  return solved;
}

std::vector<ChunkLoad> grown(const std::vector<ChunkLoad>& loads, double change) {
  std::vector<ChunkLoad> changed = loads;

  for (ChunkLoad& load : changed) {
    load.force /= 1.0 - change;
    load.torque /= 1.0 - change;
  }

  return changed;
}

bool skips(const Solved& solved, double change) {
  return talus::destruction::tolerates(solved.stress, solved.structure, grown(solved.stress.loads, change), DOWN);
}

}

TEST_CASE("a solved piece skips an update whose load change is under its tier: 10 % below a utilization of 0.5, 3 % below 0.75, 1 % above", "[destruction][budget]") {
  const Solved low = solvedColumn(0.4);
  const Solved middle = solvedColumn(0.6);
  const Solved high = solvedColumn(0.8);

  CHECK(std::abs(talus::destruction::loadChange(low.stress, grown(low.stress.loads, 0.08), DOWN) - 0.08) <= 1.0e-12);
  CHECK(skips(low, 0.005));
  CHECK(skips(low, 0.02));
  CHECK(skips(low, 0.08));
  CHECK(!skips(low, 0.12));
  CHECK(skips(middle, 0.02));
  CHECK(!skips(middle, 0.04));
  CHECK(skips(high, 0.005));
  CHECK(!skips(high, 0.015));
}

TEST_CASE("the load change is measured from the last solve, so a drift of 0.4 % per step at a utilization of 0.8 is solved on its third step", "[destruction][budget]") {
  const Solved high = solvedColumn(0.8);
  std::vector<ChunkLoad> loads = high.stress.loads;
  std::vector<bool> skipped;

  for (uint32_t step = 1u; step <= 3u; ++step) {
    for (ChunkLoad& load : loads) {
      load.force *= 1.004;
      load.torque *= 1.004;
    }

    skipped.push_back(talus::destruction::tolerates(high.stress, high.structure, loads, DOWN));
  }

  CHECK(skipped == std::vector<bool>{true, true, false});
}

TEST_CASE("a wearing piece, a piece whose last update changed a bond and a piece never solved are solved at a 0.1 % change", "[destruction][budget]") {
  Solved wearing = solvedColumn(0.1);
  Solved cracked = solvedColumn(0.1);
  Solved fresh = solvedColumn(0.1);

  wearing.stress.eroding = true;
  cracked.structure.revision = SOLVED_REVISION + 1u;
  fresh.stress.solved = false;

  CHECK(skips(solvedColumn(0.1), 0.001));
  CHECK(!skips(wearing, 0.001));
  CHECK(!skips(cracked, 0.001));
  CHECK(!skips(fresh, 0.001));
}

TEST_CASE("gravity turned by 2 % in a piece's frame is solved at a utilization of 0.8 and skipped at 0.4", "[destruction][budget]") {
  const double angle = 2.0 * std::asin(0.01);
  const glm::dvec3 turned = glm::angleAxis(angle, glm::dvec3(0.0, 0.0, 1.0)) * DOWN;
  const Solved high = solvedColumn(0.8);
  const Solved low = solvedColumn(0.4);

  CHECK(std::abs(talus::destruction::loadChange(high.stress, high.stress.loads, turned) - 0.02) <= 1.0e-12);
  CHECK(!talus::destruction::tolerates(high.stress, high.structure, high.stress.loads, turned));
  CHECK(talus::destruction::tolerates(low.stress, low.structure, low.stress.loads, turned));
}

TEST_CASE("a load-case queue keeps at most 8 cases: eleven cases that each differ from the last leave 8 queued after 3 merges of the two adjacent cases that differ least, into the newer", "[destruction][budget]") {
  std::vector<double> cases;
  const auto differ = [](double a, double b) { return std::abs(a - b); };
  const std::array<double, 11> arriving{0.0, 10.0, 11.0, 30.0, 50.0, 51.5, 80.0, 120.0, 200.0, 202.0, 300.0};
  uint32_t merged = 0u;

  for (const double next : arriving) {
    merged += talus::destruction::enqueueCase(cases, next, false, talus::destruction::queuedCases, differ)
                  == talus::destruction::Enqueued::Merged
                ? 1u
                : 0u;
  }

  CHECK(merged == 3u);
  CHECK(cases == std::vector<double>{0.0, 11.0, 30.0, 51.5, 80.0, 120.0, 202.0, 300.0});
}

TEST_CASE("a load case near the newest replaces the last queued case, and one near the case being solved is dropped", "[destruction][budget]") {
  std::vector<double> cases;
  const auto differ = [](double a, double b) { return std::abs(a - b); };

  CHECK(talus::destruction::enqueueCase(cases, 1.0, true, talus::destruction::queuedCases, differ)
        == talus::destruction::Enqueued::Dropped);
  CHECK(cases.empty());
  CHECK(talus::destruction::enqueueCase(cases, 5.0, false, talus::destruction::queuedCases, differ)
        == talus::destruction::Enqueued::Joined);
  CHECK(talus::destruction::enqueueCase(cases, 5.1, true, talus::destruction::queuedCases, differ)
        == talus::destruction::Enqueued::Replaced);
  CHECK(cases == std::vector<double>{5.1});
}

TEST_CASE("queue compression preserves irreversible payloads and never replaces the case whose fracture preparation has started", "[destruction][budget][budgetcompletion]") {
  struct Case {
    double load = 0.0;
    uint32_t impulses = 0u;
  };
  std::vector<Case> cases{{-1.0, 1u}};
  const auto difference = [](const Case& a, const Case& b) { return std::abs(a.load - b.load); };
  const auto merge = [](const Case& older, Case& newer) { newer.impulses += older.impulses; };

  CHECK(talus::destruction::enqueueCase(cases, Case{-1.0, 0u}, true, 8u, difference, merge, 1u)
        == talus::destruction::Enqueued::Dropped);

  for (uint32_t index = 0u; index != 11u; ++index) {
    static_cast<void>(talus::destruction::enqueueCase(cases, Case{double(index), 1u}, false, 8u, difference, merge, 1u));
  }

  REQUIRE(cases.size() == 9u);
  CHECK(cases.front().load == -1.0);
  CHECK(cases.front().impulses == 1u);
  uint32_t impulses = 0u;

  for (const Case& kept : cases) {
    impulses += kept.impulses;
  }

  CHECK(impulses == 12u);
  CHECK(talus::destruction::enqueueCase(cases, Case{10.0, 0u}, true, 8u, difference, merge, 1u)
        == talus::destruction::Enqueued::Replaced);
  CHECK(cases.back().impulses >= 1u);
}

namespace {

using talus::destruction::Building;
using talus::destruction::BuildingReport;
using talus::destruction::DestructionSettings;
using talus::destruction::DestructionWorld;
using talus::destruction::Member;
using talus::destruction::Role;
using talus::destruction::StructuralMaterial;

constexpr double BOX_HALF = 0.25;
constexpr double BOX_PITCH = 0.502;
constexpr uint32_t PILE_SIDE = 10u;
constexpr uint32_t ROCKER_KEY = 9100u;

DestructionSettings pressed() {
  DestructionSettings settings;

  settings.target = 0.0;
  settings.ceiling = 0.0;

  return settings;
}

DestructionSettings relaxed() {
  DestructionSettings settings;

  settings.target = std::numeric_limits<double>::infinity();
  settings.ceiling = std::numeric_limits<double>::infinity();

  return settings;
}

talus::destruction::ChunkerSettings exactChunks() {
  talus::destruction::ChunkerSettings settings;

  settings.variety = false;

  return settings;
}

StructuralMaterial flatSoil() {
  StructuralMaterial material = talus::destruction::concrete();

  material.chunkSize = 64.0;
  material.fracture = talus::destruction::Fracture::None;
  material.pieces = 1u;

  return material;
}

void addFloor(talus::dynamics::World& world, DestructionWorld& destruction) {
  Building floor;

  floor.members.push_back(Member{glm::dvec3(0.0, -0.5, 0.0), glm::dvec3(20.0, 0.5, 20.0), 0u, Role::Foundation});

  const std::vector<StructuralMaterial> materials{flatSoil()};

  static_cast<void>(destruction.addBuilding(world, floor, materials));
}

uint32_t addBox(talus::dynamics::World& world, DestructionWorld& destruction, const glm::dvec3& centre, double half = BOX_HALF) {
  Building box;

  box.members.push_back(Member{centre, glm::dvec3(half), 0u, Role::Beam});

  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  const BuildingReport built = destruction.addBuilding(world, box, materials, exactChunks());

  REQUIRE(destruction.chunked(built.building).chunks.size() == 1u);

  return built.building;
}

uint32_t bodyOfBox(const DestructionWorld& destruction, uint32_t building) {
  const std::optional<uint32_t> body = destruction.bodyOf(building, 0u);

  REQUIRE(body.has_value());

  return *body;
}

const talus::dynamics::RigidBody& rigid(const talus::dynamics::World& world, uint32_t id) {
  const std::optional<talus::dynamics::BodyRef> ref = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(ref.has_value());

  return world.bodies.rigids[ref->index];
}

struct Pile {
  talus::dynamics::World world;
  DestructionWorld destruction;
  std::vector<uint32_t> boxes;
  uint32_t rocker = 0u;
};

std::unique_ptr<Pile> pileOf(const DestructionSettings& settings) {
  auto pile = std::make_unique<Pile>();

  pile->destruction.setSettings(settings);
  addFloor(pile->world, pile->destruction);

  for (uint32_t row = 0u; row != PILE_SIDE; ++row) {
    for (uint32_t column = 0u; column != PILE_SIDE; ++column) {
      const glm::dvec3 centre(BOX_PITCH * column, BOX_HALF + 0.001, BOX_PITCH * row);

      pile->boxes.push_back(addBox(pile->world, pile->destruction, centre));
    }
  }

  pile->rocker = addSlider(pile->world, glm::dvec3(BOX_PITCH * 5.0, 2.0 * BOX_HALF + 0.25 + 0.002, BOX_PITCH * 5.0), glm::dvec3(0.0));

  return pile;
}

void rock(Pile& pile, uint32_t step) {
  const double speed = (step / 20u) % 2u == 0u ? 0.1 : -0.1;

  REQUIRE(talus::dynamics::setVelocity(pile.world, pile.rocker, glm::dvec3(speed, 0.0, 0.0)));
}

talus::destruction::DestructionReport advanceOnce(talus::dynamics::World& world, DestructionWorld& destruction) {
  destruction.prepare(world);
  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const talus::destruction::DestructionReport report = destruction.update(world);
  CHECK(report.bodies == world.bodies.rigids.size());

  return report;
}

uint32_t frozenAmong(const talus::dynamics::World& world, const DestructionWorld& destruction, const std::vector<uint32_t>& boxes) {
  uint32_t frozen = 0u;

  for (const uint32_t box : boxes) {
    frozen += talus::dynamics::isFrozen(rigid(world, bodyOfBox(destruction, box))) ? 1u : 0u;
  }

  return frozen;
}

}

TEST_CASE("a pile of 10 by 10 boxes that a rocking crate keeps awake has at least 95 boxes frozen a second after they come to rest, under a target it exceeds, and none under a target it meets", "[destruction][budget]") {
  const std::unique_ptr<Pile> pile = pileOf(pressed());
  const std::unique_ptr<Pile> calm = pileOf(relaxed());

  for (uint32_t step = 0u; step != 60u; ++step) {
    rock(*pile, step);
    rock(*calm, step);

    static_cast<void>(advanceOnce(pile->world, pile->destruction));
    static_cast<void>(advanceOnce(calm->world, calm->destruction));
  }

  CHECK(!rigid(pile->world, pile->rocker).asleep);
  CHECK(frozenAmong(pile->world, pile->destruction, pile->boxes) >= 95u);
  CHECK(frozenAmong(calm->world, calm->destruction, calm->boxes) == 0u);
}

namespace {

uint32_t addCube(talus::dynamics::World& world, const glm::dvec3& centre, double half, double density, const glm::dvec3& velocity, uint32_t key) {
  const std::array<talus::dynamics::CompoundChild, 1> cube{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(half), density, key}
  };
  const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, cube, centre, LEVEL, velocity, 0.6, 0.0, false);

  return world.bodies.rigids[index].id;
}

std::vector<glm::dvec3> positionsOf(const Pile& pile) {
  std::vector<glm::dvec3> positions;

  for (const uint32_t box : pile.boxes) {
    positions.push_back(rigid(pile.world, bodyOfBox(pile.destruction, box)).position);
  }

  return positions;
}

}

TEST_CASE("a crate dropped on a frozen pile comes to rest on it, and no frozen box moves more than 1 mm in 2 s", "[destruction][budget]") {
  constexpr double CRATE_HALF = 0.1;
  constexpr double MOVE_LIMIT = 0.001;
  const std::unique_ptr<Pile> pile = pileOf(pressed());

  for (uint32_t step = 0u; step != 60u; ++step) {
    rock(*pile, step);
    static_cast<void>(advanceOnce(pile->world, pile->destruction));
  }

  REQUIRE(frozenAmong(pile->world, pile->destruction, pile->boxes) >= 95u);

  std::vector<uint8_t> frozen;

  for (const uint32_t box : pile->boxes) {
    frozen.push_back(talus::dynamics::isFrozen(rigid(pile->world, bodyOfBox(pile->destruction, box))) ? 1u : 0u);
  }

  const std::vector<glm::dvec3> before = positionsOf(*pile);
  const uint32_t crate = addCube(
    pile->world,
    glm::dvec3(BOX_PITCH * 2.0, 2.0 * BOX_HALF + CRATE_HALF + 0.3, BOX_PITCH * 2.0),
    CRATE_HALF,
    1000.0,
    glm::dvec3(0.0),
    ROCKER_KEY + 1u
  );

  for (uint32_t step = 60u; step != 180u; ++step) {
    rock(*pile, step);
    static_cast<void>(advanceOnce(pile->world, pile->destruction));
  }

  const std::vector<glm::dvec3> after = positionsOf(*pile);
  const talus::dynamics::RigidBody& rested = rigid(pile->world, crate);

  CHECK(glm::length(rested.velocity) < 0.05);
  CHECK(rested.position.y > 2.0 * BOX_HALF);

  for (size_t at = 0u; at != pile->boxes.size(); ++at) {
    if (frozen[at] != 0u) {
      INFO("box " << at);
      CHECK(glm::length(after[at] - before[at]) <= MOVE_LIMIT);
    }
  }
}

namespace {

struct BallHit {
  bool frozenBefore = false;
  bool thawedWhenStruck = false;
  bool frozenAfter = true;
  double boxSpeed = 0.0;
  double boxMass = 0.0;
  glm::dvec3 before{0.0};
  glm::dvec3 after{0.0};
};

BallHit hitBox(const DestructionSettings& settings) {
  constexpr double BOX_SIDE = 0.16;
  constexpr double BALL_HALF = 0.05;
  talus::dynamics::World world;
  DestructionWorld destruction(settings);
  BallHit hit;

  world.parameters.gravity = glm::dvec3(0.0);

  const uint32_t box = addBox(world, destruction, glm::dvec3(0.0, 2.0, 0.0), 0.5 * BOX_SIDE);

  for (uint32_t step = 0u; step != 40u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }

  const uint32_t held = bodyOfBox(destruction, box);

  hit.frozenBefore = talus::dynamics::isFrozen(rigid(world, held));
  hit.boxMass = hit.frozenBefore ? rigid(world, held).frozenMass : rigid(world, held).mass;

  const uint32_t ball = addCube(
    world,
    glm::dvec3(-0.5 * BOX_SIDE - BALL_HALF - 0.02, 2.0, 0.0),
    BALL_HALF,
    1.0 / (8.0 * BALL_HALF * BALL_HALF * BALL_HALF),
    glm::dvec3(5.0, 0.0, 0.0),
    ROCKER_KEY + 2u
  );

  hit.before = rigid(world, ball).mass * rigid(world, ball).velocity;

  for (uint32_t step = 0u; step != 10u; ++step) {
    destruction.prepare(world);

    const talus::dynamics::StepReport stepped = talus::dynamics::step(world);

    REQUIRE(!stepped.nonFinite);
    hit.thawedWhenStruck = hit.thawedWhenStruck || (stepped.thawed != 0u && !talus::dynamics::isFrozen(rigid(world, held)));
    static_cast<void>(destruction.update(world));
  }

  const talus::dynamics::RigidBody& ballAfter = rigid(world, ball);
  const talus::dynamics::RigidBody& boxAfter = rigid(world, held);

  hit.frozenAfter = talus::dynamics::isFrozen(boxAfter);
  hit.boxSpeed = glm::length(boxAfter.velocity);
  hit.after = ballAfter.mass * ballAfter.velocity + boxAfter.mass * boxAfter.velocity;

  return hit;
}

}

TEST_CASE("a 1 kg ball at 5 m/s hitting a frozen 10 kg box thaws it in that step, and the two keep the momentum the same hit keeps on a box never frozen", "[destruction][budget]") {
  constexpr double SAME_SHARE = 1.0e-12;
  const BallHit thawed = hitBox(pressed());
  const BallHit free = hitBox(relaxed());

  REQUIRE(thawed.frozenBefore);
  REQUIRE(!free.frozenBefore);
  REQUIRE(thawed.boxMass > 9.0);
  REQUIRE(thawed.boxMass < 11.0);
  CHECK(thawed.thawedWhenStruck);
  CHECK(!thawed.frozenAfter);
  CHECK(thawed.boxSpeed > 0.1);
  INFO("momentum lost by the hit on the thawed box " << glm::length(thawed.after - thawed.before) << ", on the free box "
       << glm::length(free.after - free.before));
  CHECK(glm::length(thawed.after - free.after) <= SAME_SHARE * glm::length(free.before));
}

TEST_CASE("removing the box under a frozen box thaws it within one step, and it falls at least 0.1 m within 0.25 s", "[destruction][budget]") {
  talus::dynamics::World world;
  DestructionWorld destruction(pressed());

  addFloor(world, destruction);

  const uint32_t lower = addBox(world, destruction, glm::dvec3(0.0, BOX_HALF + 0.001, 0.0));
  const uint32_t upper = addBox(world, destruction, glm::dvec3(0.0, 3.0 * BOX_HALF + 0.003, 0.0));

  for (uint32_t step = 0u; step != 60u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }

  REQUIRE(talus::dynamics::isFrozen(rigid(world, bodyOfBox(destruction, upper))));

  const double height = rigid(world, bodyOfBox(destruction, upper)).position.y;
  const std::array<uint32_t, 1> chunk{0u};

  REQUIRE(destruction.removeChunks(world, lower, chunk) == 1u);
  static_cast<void>(advanceOnce(world, destruction));
  CHECK(!talus::dynamics::isFrozen(rigid(world, bodyOfBox(destruction, upper))));

  for (uint32_t step = 1u; step != 15u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }

  CHECK(height - rigid(world, bodyOfBox(destruction, upper)).position.y >= 0.1);
}

TEST_CASE("support removed just after controller freeze lets the thawed body fall instead of freezing again immediately", "[destruction][budget][budgetcompletion][freezethaw]") {
  talus::dynamics::World world;
  DestructionWorld destruction(pressed());
  world.parameters.sleep = false;
  addFloor(world, destruction);
  const uint32_t lower = addBox(world, destruction, glm::dvec3(0.0, BOX_HALF + 0.001, 0.0));
  const uint32_t upper = addBox(world, destruction, glm::dvec3(0.0, 3.0 * BOX_HALF + 0.003, 0.0));

  for (uint32_t step = 0u; step != 120u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));

    if (talus::dynamics::isFrozen(rigid(world, bodyOfBox(destruction, upper)))) {
      break;
    }
  }

  const uint32_t upperBody = bodyOfBox(destruction, upper);
  REQUIRE(talus::dynamics::isFrozen(rigid(world, upperBody)));
  const double height = rigid(world, upperBody).position.y;
  const std::array<uint32_t, 1> chunk{0u};
  REQUIRE(destruction.removeChunks(world, lower, chunk) == 1u);
  REQUIRE(!talus::dynamics::isFrozen(rigid(world, upperBody)));

  for (uint32_t step = 0u; step != 15u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
    CHECK(!talus::dynamics::isFrozen(rigid(world, upperBody)));
  }

  CHECK(height - rigid(world, upperBody).position.y >= 0.1);
}

TEST_CASE("freezing a body changes no other body's contacts or sleep state", "[destruction][budget]") {
  talus::dynamics::World world;
  DestructionWorld destruction(relaxed());

  addFloor(world, destruction);

  const uint32_t lower = addBox(world, destruction, glm::dvec3(0.0, BOX_HALF + 0.001, 0.0));
  const uint32_t upper = addBox(world, destruction, glm::dvec3(0.0, 3.0 * BOX_HALF + 0.003, 0.0));
  const uint32_t side = addBox(world, destruction, glm::dvec3(2.0, BOX_HALF + 0.001, 0.0));

  for (uint32_t step = 0u; step != 20u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }

  const auto pairs = [&](uint32_t id) {
    std::vector<uint64_t> found;

    for (const uint32_t slot : world.manifolds.pairsOf(id)) {
      if (world.manifolds.live(slot)) {
        found.push_back(world.manifolds.at(slot).pair);
      }
    }

    std::sort(found.begin(), found.end());

    return found;
  };
  const uint32_t upperBody = bodyOfBox(destruction, upper);
  const uint32_t sideBody = bodyOfBox(destruction, side);
  const std::vector<uint64_t> upperPairs = pairs(upperBody);
  const std::vector<uint64_t> sidePairs = pairs(sideBody);
  const bool upperAsleep = rigid(world, upperBody).asleep;
  const bool sideAsleep = rigid(world, sideBody).asleep;

  REQUIRE(talus::dynamics::freezeBody(world, bodyOfBox(destruction, lower)));
  CHECK(pairs(upperBody) == upperPairs);
  CHECK(pairs(sideBody) == sidePairs);
  CHECK(rigid(world, upperBody).asleep == upperAsleep);
  CHECK(rigid(world, sideBody).asleep == sideAsleep);
  CHECK(!upperPairs.empty());
}

TEST_CASE("a box a blast pushes, a box a shot hits and a piece split from a blasted bar are not frozen within 60 steps of the touch, and are frozen after", "[destruction][budget]") {
  constexpr uint64_t TOUCH = 2u;
  constexpr uint64_t GUARD = 60u;
  talus::dynamics::World world;
  DestructionWorld destruction(pressed());

  world.parameters.sleep = false;
  addFloor(world, destruction);

  const uint32_t blasted = addBox(world, destruction, glm::dvec3(0.0, BOX_HALF + 0.001, 0.0));
  const uint32_t shotBox = addBox(world, destruction, glm::dvec3(6.0, BOX_HALF + 0.001, 0.0));
  Building bar;

  bar.members.push_back(Member{glm::dvec3(-6.0, BOX_HALF + 0.001, 0.0), glm::dvec3(2.0 * BOX_HALF, BOX_HALF, BOX_HALF), 0u, Role::Beam});

  const std::vector<StructuralMaterial> materials{talus::destruction::concrete()};
  const BuildingReport built = destruction.addBuilding(world, bar, materials, exactChunks());

  REQUIRE(destruction.chunked(built.building).chunks.size() == 2u);

  const uint32_t bullet = addCube(world, glm::dvec3(6.0, 3.0, 5.0), 0.025, 7800.0, glm::dvec3(0.0), ROCKER_KEY + 3u);

  destruction.blastAt(talus::destruction::Blast{glm::dvec3(0.0, BOX_HALF, -1.0), 200.0, 3.0, TOUCH});
  destruction.blastAt(talus::destruction::Blast{glm::dvec3(-6.0, BOX_HALF, -1.0), 200.0, 3.0, TOUCH});
  destruction.fire(talus::destruction::Shot{bullet, TOUCH, glm::dvec3(6.0, BOX_HALF, -0.5), glm::dvec3(0.0, 0.0, 20.0)});

  bool frozenEarly = false;

  for (uint64_t step = 1u; step <= TOUCH + 150u; ++step) {
    if (step == TOUCH + 10u) {
      const std::array<uint32_t, 1> half{1u};

      REQUIRE(destruction.removeChunks(world, built.building, half) == 1u);
    }

    static_cast<void>(advanceOnce(world, destruction));

    if (step <= TOUCH + GUARD) {
      for (const uint32_t building : {blasted, shotBox, built.building}) {
        const std::optional<uint32_t> body = destruction.bodyOf(building, 0u);

        if (body.has_value()) {
          frozenEarly = frozenEarly || talus::dynamics::isFrozen(rigid(world, *body));
        }
      }
    }
  }

  CHECK(!frozenEarly);

  for (const uint32_t building : {blasted, shotBox, built.building}) {
    INFO("building " << building);
    CHECK(talus::dynamics::isFrozen(rigid(world, bodyOfBox(destruction, building))));
  }
}

TEST_CASE("editing a frozen body's shape or motion restores its physical mass before the edit", "[destruction][budget][budgetcompletion]") {
  talus::dynamics::World world;
  const uint32_t body = addSlider(world, glm::dvec3(0.0, 2.0, 0.0), glm::dvec3(0.0));
  const talus::dynamics::RigidBody before = rigid(world, body);

  REQUIRE(talus::dynamics::freezeBody(world, body));

  SECTION("shape") {
    talus::dynamics::RigidBody replacement = before;

    replacement.mass *= 2.0;
    replacement.moment *= 2.0;
    REQUIRE(talus::dynamics::replaceRigidShape(world, body, replacement));
    CHECK(rigid(world, body).mass == replacement.mass);
    CHECK(rigid(world, body).moment == replacement.moment);
  }
  SECTION("pose") {
    REQUIRE(talus::dynamics::setRigidPose(world, body, glm::dvec3(1.0, 2.0, 0.0), LEVEL));
    CHECK(rigid(world, body).position == glm::dvec3(1.0, 2.0, 0.0));
  }
  SECTION("velocity") {
    REQUIRE(talus::dynamics::setVelocity(world, body, glm::dvec3(0.2, 0.0, 0.0)));
    CHECK(rigid(world, body).velocity == glm::dvec3(0.2, 0.0, 0.0));
  }
  SECTION("angular velocity") {
    REQUIRE(talus::dynamics::setAngularVelocity(world, body, glm::dvec3(0.0, 0.2, 0.0)));
    CHECK(rigid(world, body).angularVelocity == glm::dvec3(0.0, 0.2, 0.0));
  }
  SECTION("mass") {
    REQUIRE(talus::dynamics::setMass(world, body, 2.0 * before.mass));
    CHECK(rigid(world, body).mass == 2.0 * before.mass);
  }
  SECTION("impulse") {
    REQUIRE(talus::dynamics::applyImpulse(world, body, glm::dvec3(1.0, 0.0, 0.0), before.position));
    CHECK(rigid(world, body).mass * rigid(world, body).velocity == glm::dvec3(1.0, 0.0, 0.0));
  }

  CHECK(!talus::dynamics::isFrozen(rigid(world, body)));
}

TEST_CASE("removing or moving a static field thaws frozen rubble resting on it before its contacts are cleared", "[destruction][budget][budgetcompletion]") {
  const std::array<talus::field::Primitive, 1> ground{
    fixture::box(glm::vec3(0.0f, -0.25f, 0.0f), glm::vec3(1.0f, 0.25f, 1.0f))
  };
  const talus::field::SampledSdf floor = talus::field::bake(
    ground,
    glm::vec3(-1.05f, -0.55f, -1.05f), glm::vec3(1.05f, 0.05f, 1.05f), glm::uvec3(43u, 13u, 43u)
  );
  talus::dynamics::World world;
  const uint32_t floorIndex = talus::dynamics::addStatic(world.bodies, floor, 0.6);
  const uint32_t floorId = world.bodies.statics[floorIndex].id;
  const uint32_t body = addSlider(world, glm::dvec3(0.0, 0.251, 0.0), glm::dvec3(0.0));

  for (uint32_t step = 0u; step != 20u; ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(!world.manifolds.pairsOf(body).empty());
  REQUIRE(talus::dynamics::freezeBody(world, body));
  const double height = rigid(world, body).position.y;

  SECTION("remove") {
    REQUIRE(talus::dynamics::removeStatic(world, floorId));
  }
  SECTION("move") {
    REQUIRE(talus::dynamics::setStaticPose(world, floorId, glm::dvec3(0.0, -2.0, 0.0), LEVEL));
  }

  CHECK(!talus::dynamics::isFrozen(rigid(world, body)));

  for (uint32_t step = 0u; step != 15u; ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  CHECK(height - rigid(world, body).position.y >= 0.1);
}

TEST_CASE("a scripted shot below the generic thaw threshold thaws a frozen target before the contact solve", "[destruction][budget][budgetcompletion]") {
  talus::dynamics::World world;
  DestructionWorld destruction(pressed());

  world.parameters.gravity = glm::dvec3(0.0);
  const uint32_t box = addBox(world, destruction, glm::dvec3(0.0, 2.0, 0.0), 0.08);

  for (uint32_t step = 0u; step != 40u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }

  const uint32_t held = bodyOfBox(destruction, box);
  REQUIRE(talus::dynamics::isFrozen(rigid(world, held)));
  const uint32_t shot = addCube(world, glm::dvec3(-2.0, 2.0, 0.0), 0.05, 1000.0, glm::dvec3(0.0), ROCKER_KEY + 10u);

  destruction.fire(talus::destruction::Shot{shot, destruction.steps() + 1u, glm::dvec3(-0.131, 2.0, 0.0), glm::dvec3(0.1, 0.0, 0.0)});
  destruction.prepare(world);
  const talus::dynamics::StepReport report = talus::dynamics::step(world);

  REQUIRE(!report.nonFinite);
  CHECK(report.thawed >= 1u);
  CHECK(!talus::dynamics::isFrozen(rigid(world, held)));
  CHECK(rigid(world, held).velocity.x > 0.0);
  static_cast<void>(destruction.update(world));
  CHECK(destruction.guardedBody(held));
}

TEST_CASE("tool contact guards only the current owner of a chunk after it detaches from its parent", "[destruction][budget][budgetcompletion][guardownership]") {
  talus::dynamics::World world;
  DestructionWorld destruction(relaxed());
  world.parameters.gravity = glm::dvec3(0.0);
  Building building;
  building.members.push_back(Member{glm::dvec3(0.0), glm::dvec3(0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(3.0, 2.0, 0.0), glm::dvec3(0.25), 0u, Role::Beam});
  const std::array<StructuralMaterial, 1> materials{flatSoil()};
  const BuildingReport built = destruction.addBuilding(world, building, materials, exactChunks());
  REQUIRE(destruction.chunkCount(built.building) == 2u);
  REQUIRE(advanceOnce(world, destruction).created.size() == 1u);
  const auto parent = destruction.bodyOf(built.building, 0u);
  const auto detached = destruction.bodyOf(built.building, 1u);
  REQUIRE(parent.has_value());
  REQUIRE(detached.has_value());
  REQUIRE(*parent != *detached);

  for (uint32_t step = 0u; step != 65u; ++step) {
    static_cast<void>(advanceOnce(world, destruction));
  }
  REQUIRE(!destruction.guardedBody(*parent));
  REQUIRE(!destruction.guardedBody(*detached));
  uint32_t touched = *parent;
  uint32_t untouched = *detached;
  SECTION("the surviving parent is touched") {}
  SECTION("the detached body is touched") {
    std::swap(touched, untouched);
  }
  const glm::dvec3 centre = rigid(world, touched).position + glm::dvec3(0.0, 0.5, 0.0);
  const uint32_t tool = addCube(world, centre, 0.25, 1000.0, glm::dvec3(0.0), ROCKER_KEY + 11u);
  destruction.fire(talus::destruction::Shot{tool, destruction.steps() + 1u, centre, glm::dvec3(0.0)});
  static_cast<void>(advanceOnce(world, destruction));
  CHECK(destruction.guardedBody(touched));
  CHECK(!destruction.guardedBody(untouched));
}

TEST_CASE("the controller removes at most 256 frozen single chunks, smallest first, and keeps an unfrozen mover", "[destruction][budget][budgetcompletion]") {
  talus::dynamics::World world;
  DestructionSettings settings = pressed();

  settings.removalFloor = 0.0;
  DestructionWorld destruction(settings);
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;
  std::vector<uint32_t> boxes;

  for (uint32_t index = 0u; index != 300u; ++index) {
    boxes.push_back(addBox(world, destruction, glm::dvec3(2.0 * (index % 20u), 2.0, 2.0 * (index / 20u)), 0.125 + 0.0625 * (index / 100u)));
  }

  const uint32_t mover = addSlider(world, glm::dvec3(-20.0, 2.0, 0.0), glm::dvec3(1.0, 0.0, 0.0));
  uint32_t removed = 0u;

  REQUIRE(advanceOnce(world, destruction).created.size() == boxes.size());

  for (uint32_t step = 0u; step != 30u; ++step) {
    removed += advanceOnce(world, destruction).removedFrozen;
  }

  REQUIRE(removed == 256u);

  for (uint32_t index = 0u; index != boxes.size(); ++index) {
    INFO("box " << index);
    CHECK(destruction.bodyOf(boxes[index], 0u).has_value() == (index >= 256u));
  }

  CHECK(rigid(world, mover).mass > 0.0);
  CHECK(!talus::dynamics::isFrozen(rigid(world, mover)));
}

TEST_CASE("six overloaded chunks fracture four this step and two next under budget pressure, preserving the unlimited fracture set", "[destruction][budget][budgetcompletion]") {
  const auto fractures = [](const DestructionSettings& settings) {
    talus::dynamics::World world;
    DestructionWorld destruction(settings);
    world.parameters.gravity = glm::dvec3(0.0);

    for (uint32_t index = 0u; index != 6u; ++index) {
      Building block;
      block.members.push_back(Member{glm::dvec3(2.0 * index, 0.125, 0.0), glm::dvec3(0.125), 0u, Role::Column});
      const std::vector<StructuralMaterial> materials{talus::destruction::brickMasonry()};
      static_cast<void>(destruction.addBuilding(world, block, materials, exactChunks()));
    }

    static_cast<void>(addBox(world, destruction, glm::dvec3(100.0, 2.0, 0.0)));
    static_cast<void>(advanceOnce(world, destruction));
    destruction.blastAt(talus::destruction::Blast{glm::dvec3(5.0, 0.125, -1.0), 1.0e8, 20.0, destruction.steps() + 1u});
    const talus::destruction::DestructionReport first = advanceOnce(world, destruction);
    const talus::destruction::DestructionReport second = advanceOnce(world, destruction);

    REQUIRE(first.created.size() + second.created.size() > 0u);

    for (const auto& report : {first, second}) {
      for (const uint32_t body : report.created) {
        CHECK(destruction.guardedBody(body));
      }
    }

    return std::array<std::vector<talus::destruction::FracturedChunk>, 2>{first.fractured, second.fractured};
  };
  DestructionSettings boundedSettings = pressed();
  boundedSettings.ceiling = std::numeric_limits<double>::infinity();
  const auto bounded = fractures(boundedSettings);
  const auto unlimited = fractures(relaxed());

  CHECK(bounded[0].size() == 4u);
  CHECK(bounded[1].size() == 2u);
  std::vector<talus::destruction::FracturedChunk> both = bounded[0];

  both.insert(both.end(), bounded[1].begin(), bounded[1].end());
  CHECK(both == unlimited[0]);
  CHECK(unlimited[0].size() == 6u);
}

TEST_CASE("removing a thousand rigid bodies in one batch preserves an unrelated sleeping pair and remaps its cached body slots", "[destruction][budget][budgetcompletion]") {
  talus::dynamics::World world;
  DestructionWorld destruction(relaxed());
  addFloor(world, destruction);
  std::vector<uint32_t> removed;

  for (uint32_t index = 0u; index != 1000u; ++index) {
    removed.push_back(addCube(world, glm::dvec3(-12.0 + 0.6 * (index % 40u), 0.251, -8.0 + 0.6 * (index / 40u)), 0.25, 1000.0, glm::dvec3(0.0), BLOCK_KEY + index));
  }

  const uint32_t lower = addCube(world, glm::dvec3(16.0, 0.251, 0.0), 0.25, 1000.0, glm::dvec3(0.0), BLOCK_KEY + 1000u);
  const uint32_t upper = addCube(world, glm::dvec3(16.0, 0.753, 0.0), 0.25, 1000.0, glm::dvec3(0.0), BLOCK_KEY + 1001u);

  for (uint32_t step = 0u; step != 80u; ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(rigid(world, lower).asleep);
  REQUIRE(rigid(world, upper).asleep);
  const std::vector<uint32_t> slots(world.manifolds.pairsOf(upper).begin(), world.manifolds.pairsOf(upper).end());
  REQUIRE(!slots.empty());
  std::vector<talus::dynamics::Contact> before;

  for (const uint32_t slot : slots) {
    const auto points = world.manifolds.records(slot);
    before.insert(before.end(), points.begin(), points.end());
  }

  removed.push_back(removed.front());
  REQUIRE(talus::dynamics::removeBodies(world, removed) == 1000u);
  CHECK(rigid(world, lower).asleep);
  CHECK(rigid(world, upper).asleep);
  size_t points = 0u;

  for (const uint32_t slot : slots) {
    REQUIRE(world.manifolds.live(slot));
    const auto after = world.manifolds.records(slot);

    for (const auto& point : after) {
      REQUIRE(points < before.size());
      CHECK(point.key == before[points].key);
      CHECK(point.gap == before[points].gap);
      CHECK(point.memory.impulse == before[points].memory.impulse);
      CHECK(point.basis == before[points].basis);
      ++points;
    }
  }

  CHECK(points == before.size());
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  CHECK(rigid(world, lower).asleep);
  CHECK(rigid(world, upper).asleep);
}

TEST_CASE("freezing stops when its prediction reaches the target, so cleanup does not delete debris after freezing has covered the gap", "[destruction][budget][budgetcompletion][budgetstop]") {
  talus::dynamics::World world;
  DestructionSettings settings = relaxed();
  DestructionWorld destruction(settings);
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;

  for (uint32_t index = 0u; index != 100u; ++index) {
    static_cast<void>(addBox(world, destruction, glm::dvec3(2.0 * index, 2.0, 0.0)));
  }

  talus::destruction::DestructionReport last;

  for (uint32_t step = 0u; step != 32u; ++step) {
    last = advanceOnce(world, destruction);
  }

  REQUIRE(last.awakeChunks == 100u);
  REQUIRE(last.moving == 0u);
  settings.target = last.cost.total - 50.0 * talus::destruction::costModel.rigid[1];
  settings.removalFloor = 0.0;
  REQUIRE(settings.target > 0.0);
  destruction.setSettings(settings);
  const talus::destruction::DestructionReport report = advanceOnce(world, destruction);

  CHECK(report.frozen > 0u);
  CHECK(report.frozen < 100u);
  CHECK(report.removedFrozen == 0u);
  CHECK(report.awakeChunks == 100u - report.frozen);
  CHECK(report.cost.total <= settings.target);
  CHECK(report.cost.total == talus::destruction::predictCost(talus::destruction::stepCounts(report)).total);
}

TEST_CASE("an optional lower boundary discards only wholly escaped destructible bodies and leaves other bodies intact", "[destruction][budgetcompletion][debrisboundary]") {
  DestructionSettings settings = relaxed();
  bool bounded = false;

  SECTION("worlds remain unbounded by default") {}
  SECTION("the scene chooses a lower boundary") {
    settings.discardBelow = -2.0;
    bounded = true;
  }

  settings.activeSet = false;
  talus::dynamics::World world;
  world.parameters.gravity = glm::dvec3(0.0);
  world.parameters.sleep = false;
  DestructionWorld destruction(settings);
  const uint32_t above = addBox(world, destruction, glm::dvec3(0.0, 0.0, 0.0));
  const uint32_t crossing = addBox(world, destruction, glm::dvec3(2.0, -2.2, 0.0));
  const uint32_t below = addBox(world, destruction, glm::dvec3(4.0, -3.0, 0.0));
  Building pair;
  pair.members.push_back(Member{glm::dvec3(10.0, -3.0, 0.0), glm::dvec3(BOX_HALF), 0u, Role::Beam});
  pair.members.push_back(Member{glm::dvec3(10.5, -3.0, 0.0), glm::dvec3(BOX_HALF), 0u, Role::Beam});
  const std::array<StructuralMaterial, 1> materials{talus::destruction::concrete()};
  const uint32_t compound = destruction.addBuilding(world, pair, materials, exactChunks()).building;
  REQUIRE(destruction.chunked(compound).chunks.size() == 2u);
  const uint32_t unrelated = addSlider(world, glm::dvec3(6.0, -5.0, 0.0), glm::dvec3(0.0));
  const auto report = advanceOnce(world, destruction);

  CHECK(report.discarded == (bounded ? 2u : 0u));
  CHECK(destruction.bodyOf(above, 0u).has_value());
  CHECK(destruction.bodyOf(crossing, 0u).has_value());
  CHECK(destruction.bodyOf(below, 0u).has_value() == !bounded);
  CHECK(destruction.bodyOf(compound, 0u).has_value() == !bounded);
  CHECK(destruction.bodyOf(compound, 1u).has_value() == !bounded);
  CHECK(talus::dynamics::findBody(world.bodies, unrelated).has_value());
  CHECK(destructionfixture::mismatch(destruction, world).empty());
}

TEST_CASE("a rotating body's actual contact velocity decides whether it strikes frozen rubble", "[destruction][budgetcompletion][thawcontact]") {
  const bool striking = GENERATE(false, true);
  const bool reversed = GENERATE(false, true);
  const bool turned = GENERATE(false, true);
  CAPTURE(striking, reversed, turned);
  talus::dynamics::World world;
  const glm::dquat rotation = turned ? glm::angleAxis(0.7, glm::dvec3(0.0, 1.0, 0.0)) : LEVEL;
  const std::array<talus::dynamics::CompoundChild, 1> beam{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(1.0, 0.25, 0.25), 1000.0, BLOCK_KEY + 1u}
  };
  const auto addBeam = [&] {
    return talus::dynamics::addCompoundBody(world.bodies, beam, glm::dvec3(0.0, 0.501, 0.0), rotation,
      glm::dvec3(0.0), 0.6, 0.0, false);
  };
  const uint32_t firstBeam = reversed ? addBeam() : 0u;
  const uint32_t target = addSlider(world, rotation * glm::dvec3(1.0, 0.0, 0.0), glm::dvec3(0.0));
  world.bodies.rigids.back().rotation = rotation;
  const uint32_t incoming = reversed ? firstBeam : addBeam();
  const std::vector<double> still(world.bodies.rigids.size(), 0.0);
  world.manifolds.sync(world.bodies, talus::dynamics::broadphase(world.bodies, 0.02));
  static_cast<void>(talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0));
  REQUIRE(talus::dynamics::freezeBody(world, target));
  auto& coming = world.bodies.rigids[incoming];
  coming.velocity = striking ? glm::dvec3(0.0) : glm::dvec3(0.0, -1.0, 0.0);
  coming.angularVelocity = rotation * glm::dvec3(0.0, 0.0, striking ? -4.0 : 4.0);
  uint32_t points = 0u;

  for (const uint32_t slot : world.manifolds.pairsOf(target)) {
    world.manifolds.visitPoints(slot, [&](const talus::dynamics::CachedPointView& view) {
      const bool targetFirst = world.bodies.rigids[view.a.index].id == target;
      const glm::dvec3 offset = targetFirst ? view.point.localB : view.point.localA;
      REQUIRE(offset.x >= 0.74);
      REQUIRE(std::abs(view.manifold.basis[0].y) == 1.0);
      // Independently evaluate this level beam's vertical point velocity: vy + omega_z * x.
      const double vertical = coming.velocity.y + (striking ? -4.0 : 4.0) * offset.x;
      CHECK((striking ? -vertical : vertical) > 1.0);
      ++points;
    });
  }

  REQUIRE(points > 0u);
  CHECK(talus::dynamics::thawStruck(world, 0.3) == (striking ? 1u : 0u));
  CHECK(talus::dynamics::isFrozen(rigid(world, target)) == !striking);
}

TEST_CASE("thaw does not cross a cached broadphase pair with no contact points", "[destruction][budgetcompletion][thawcontact]") {
  talus::dynamics::World world;
  const std::array<talus::dynamics::CompoundChild, 2> ends{
    talus::dynamics::CompoundChild{glm::dvec3(-1.0, 0.0, 0.0), LEVEL, glm::dvec3(0.1), 1000.0, BLOCK_KEY},
    talus::dynamics::CompoundChild{glm::dvec3(1.0, 0.0, 0.0), LEVEL, glm::dvec3(0.1), 1000.0, BLOCK_KEY + 1u}
  };
  const uint32_t outer = talus::dynamics::addCompoundBody(world.bodies, ends, glm::dvec3(0.0), LEVEL, glm::dvec3(0.0), 0.6, 0.0, false);
  const uint32_t inner = addSlider(world, glm::dvec3(0.0), glm::dvec3(0.0));
  const uint32_t outerId = world.bodies.rigids[outer].id;
  const std::vector<double> still(world.bodies.rigids.size(), 0.0);
  world.manifolds.sync(world.bodies, talus::dynamics::broadphase(world.bodies, 0.02));
  static_cast<void>(talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0));
  REQUIRE(world.manifolds.pairsOf(outerId).size() == 1u);
  const uint32_t slot = world.manifolds.pairsOf(outerId).front();
  REQUIRE(world.manifolds.live(slot));
  REQUIRE(world.manifolds.at(slot).points.empty());
  REQUIRE(talus::dynamics::freezeBody(world, outerId));
  REQUIRE(talus::dynamics::freezeBody(world, inner));
  SECTION("direct thaw") { REQUIRE(talus::dynamics::thawBody(world, outerId)); }
  SECTION("motion edit") { REQUIRE(talus::dynamics::setVelocity(world, outerId, glm::dvec3(1.0, 0.0, 0.0))); }
  SECTION("single removal") { REQUIRE(talus::dynamics::removeBody(world, outerId)); }
  SECTION("batch removal") {
    const std::array<uint32_t, 1> removed{outerId};
    REQUIRE(talus::dynamics::removeBodies(world, removed) == 1u);
  }
  CHECK(talus::dynamics::isFrozen(rigid(world, inner)));
}

TEST_CASE("a heavy body's slow drift does not count as an impact that could move frozen rubble at the rest-speed limit", "[destruction][budgetcompletion][thawimpact]") {
  const double speed = GENERATE(0.01, 5.0);
  talus::dynamics::World world;
  const std::array<talus::dynamics::CompoundChild, 1> light{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(0.25), 80.0, BLOCK_KEY}
  };
  const std::array<talus::dynamics::CompoundChild, 1> heavy{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(0.25), 8000.0, BLOCK_KEY + 1u}
  };
  const uint32_t targetIndex = talus::dynamics::addCompoundBody(world.bodies, light, glm::dvec3(0.0), LEVEL,
    glm::dvec3(0.0), 0.6, 0.0, false);
  const uint32_t coming = talus::dynamics::addCompoundBody(world.bodies, heavy, glm::dvec3(0.0, 0.501, 0.0), LEVEL,
    glm::dvec3(0.0, -speed, 0.0), 0.6, 0.0, false);
  const uint32_t target = world.bodies.rigids[targetIndex].id;
  REQUIRE(world.bodies.rigids[targetIndex].mass == 10.0);
  REQUIRE(world.bodies.rigids[coming].mass == 1000.0);
  // The inelastic two-body result follows from momentum conservation, before freezing either body.
  const double followingSpeed = 1000.0 * speed / 1010.0;
  const bool appreciable = followingSpeed > talus::destruction::restSpeed;
  const std::vector<double> still(world.bodies.rigids.size(), 0.0);
  world.manifolds.sync(world.bodies, talus::dynamics::broadphase(world.bodies, 0.02));
  static_cast<void>(talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0));
  REQUIRE(world.manifolds.pointCount() > 0u);
  REQUIRE(talus::dynamics::freezeBody(world, target));
  CHECK(talus::dynamics::thawStruck(world, talus::destruction::restSpeed) == (appreciable ? 1u : 0u));
  CHECK(talus::dynamics::isFrozen(rigid(world, target)) == !appreciable);
}

TEST_CASE("a lever hit thaws a frozen beam when rotation can move its tip above the rest-speed limit", "[destruction][budgetcompletion][thawimpact]") {
  talus::dynamics::World world;
  const std::array<talus::dynamics::CompoundChild, 1> beam{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(1.0, 0.125, 0.125), 80.0, BLOCK_KEY}
  };
  const uint32_t targetIndex = talus::dynamics::addCompoundBody(world.bodies, beam, glm::dvec3(0.0), LEVEL,
    glm::dvec3(0.0), 0.6, 0.0, false);
  const uint32_t target = world.bodies.rigids[targetIndex].id;
  const std::array<talus::field::Primitive, 1> shape{fixture::sphere(glm::vec3(0.0f), 0.1f)};
  const auto field = talus::field::bake(shape, glm::vec3(-0.14f), glm::vec3(0.14f), glm::uvec3(15u));
  static_cast<void>(talus::dynamics::addRigidSphere(world.bodies, field, 0.1, 10.0, glm::dvec3(0.9, 0.226, 0.0),
    glm::dvec3(0.0, -0.5, 0.0), 0.6, 0.0));
  const auto& body = world.bodies.rigids[targetIndex];
  REQUIRE(body.mass == 10.0);
  const double impulse = 0.5 / (0.1 + 0.1 + 0.9 * 0.9 / body.moment.z);
  const double tipSpeed = impulse / 10.0 + impulse * 0.9 / body.moment.z;
  REQUIRE(tipSpeed > talus::destruction::restSpeed);
  REQUIRE(0.5 * 10.0 / 20.0 < talus::destruction::restSpeed);
  const std::vector<double> still(world.bodies.rigids.size(), 0.0);
  world.manifolds.sync(world.bodies, talus::dynamics::broadphase(world.bodies, 0.02));
  static_cast<void>(talus::dynamics::updateCachedPairs(world.bodies, world.manifolds, 0.02, still, 0.0));
  REQUIRE(world.manifolds.pointCount() == 1u);
  REQUIRE(talus::dynamics::freezeBody(world, target));
  CHECK(talus::dynamics::thawStruck(world, talus::destruction::restSpeed) == 1u);
  CHECK(!talus::dynamics::isFrozen(rigid(world, target)));
}

TEST_CASE(
  "a rest tracker fed the awake list counts as one that scans every body, "
  "through a removal that shifts indices and a body that falls asleep between "
  "two updates",
  "[destruction][budget][awake]"
) {
  talus::dynamics::World world;
  DestructionWorld owner;

  addFloor(world, owner);

  const uint32_t lead =
    addSlider(world, glm::dvec3(-5.0, 0.251, 0.0), glm::dvec3(0.0));
  const uint32_t settler =
    addSlider(world, glm::dvec3(0.0, 0.27, 0.0), glm::dvec3(0.0));
  const talus::dynamics::Workers workers;
  talus::destruction::RestTracker listed;
  talus::destruction::RestTracker scanned;
  std::vector<uint32_t> awake;
  const auto track = [&] {
    if (talus::dynamics::awakeSuperset(world, awake)) {
      listed.update(world, workers, awake);
    } else {
      listed.update(world, workers);
    }

    scanned.update(world, workers);

    for (uint32_t id = 0u; id != world.bodies.nextId; ++id) {
      INFO("body " << id);
      REQUIRE(listed.restSteps(id) == scanned.restSteps(id));
    }
  };
  const auto aboutToSleep = [&] {
    const talus::dynamics::RigidBody& body = rigid(world, settler);

    return !body.asleep
           && body.sleepTime
                >= world.parameters.timeToSleep - 3.0 * world.parameters.dt;
  };

  for (uint32_t step = 0u; step != 600u && !aboutToSleep(); ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
    track();
  }

  REQUIRE(aboutToSleep());
  REQUIRE(scanned.restSteps(settler) > 0u);
  REQUIRE(talus::dynamics::removeBody(world, lead));
  static_cast<void>(
    addSlider(world, glm::dvec3(5.0, 0.251, 0.0), glm::dvec3(0.0))
  );

  for (uint32_t step = 0u; step != 10u && !rigid(world, settler).asleep;
       ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  REQUIRE(rigid(world, settler).asleep);
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  track();
  CHECK(listed.restSteps(settler) == 0u);

  REQUIRE(
    talus::dynamics::setVelocity(world, settler, glm::dvec3(0.0, 0.0, 0.05))
  );
  REQUIRE(!talus::dynamics::step(world).nonFinite);
  track();
  CHECK(listed.restSteps(settler) == 1u);
}

TEST_CASE(
  "the destruction update finds a piece whose body index shifted after a "
  "removal and an append between updates",
  "[destruction][budget][awake]"
) {
  talus::dynamics::World world;
  DestructionWorld destruction;

  addFloor(world, destruction);

  const uint32_t spacer =
    addSlider(world, glm::dvec3(-5.0, 0.251, 0.0), glm::dvec3(0.0));
  const uint32_t box =
    addBox(world, destruction, glm::dvec3(0.0, BOX_HALF + 0.001, 0.0));
  const uint32_t rocker = addSlider(
    world,
    glm::dvec3(0.0, 2.0 * BOX_HALF + 0.25 + 0.002, 0.0),
    glm::dvec3(0.0)
  );
  const auto rockOnce = [&](uint32_t step) {
    const double speed = (step / 20u) % 2u == 0u ? 0.1 : -0.1;

    REQUIRE(
      talus::dynamics::setVelocity(world, rocker, glm::dvec3(speed, 0.0, 0.0))
    );
  };

  for (uint32_t step = 0u; step != 30u; ++step) {
    rockOnce(step);
    static_cast<void>(advanceOnce(world, destruction));
  }

  REQUIRE(talus::dynamics::removeBody(world, spacer));
  static_cast<void>(
    addSlider(world, glm::dvec3(5.0, 0.251, 0.0), glm::dvec3(0.0))
  );
  rockOnce(30u);

  talus::destruction::DestructionReport report;

  REQUIRE_NOTHROW(report = advanceOnce(world, destruction));
  CHECK(report.visited >= 1u);
  CHECK(report.gathered >= 1u);
  CHECK(!destruction.chunkLoads(box).empty());
}
