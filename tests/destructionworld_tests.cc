#include <catch2/catch_test_macros.hpp>

#include "destructionfixtures.h"

#include <talus/destruction/building/building.h>
#include <talus/destruction/chunker/chunker.h>
#include <talus/destruction/destructionworld/destructionworld.h>
#include <talus/destruction/materials/materials.h>
#include <talus/destruction/stress/stress.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/tiers/tiers.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using talus::destruction::BondState;
using talus::destruction::Building;
using talus::destruction::BuildingReport;
using talus::destruction::ChunkedBuilding;
using talus::destruction::ChunkerSettings;
using talus::destruction::DestructionSettings;
using talus::destruction::DestructionWorld;
using talus::destruction::Member;
using talus::destruction::Role;
using talus::destruction::StructuralMaterial;
using talus::dynamics::RigidBody;

constexpr uint32_t NO_BOND = 0xFFFFFFFFu;
constexpr double PI = 3.14159265358979323846;

const glm::dquat LEVEL(1.0, 0.0, 0.0, 0.0);

ChunkerSettings exact() {
  ChunkerSettings settings;

  settings.variety = false;

  return settings;
}

DestructionSettings exactLoads() {
  DestructionSettings settings;

  settings.tolerance = false;

  return settings;
}

DestructionSettings exceeded() {
  DestructionSettings settings;

  settings.target = 0.0;
  settings.ceiling = std::numeric_limits<double>::infinity();
  settings.activeSet = false;

  return settings;
}

DestructionSettings unlimited() {
  DestructionSettings settings;

  settings.target = std::numeric_limits<double>::infinity();
  settings.ceiling = std::numeric_limits<double>::infinity();

  return settings;
}

std::vector<StructuralMaterial> concreteOnly() {
  return {talus::destruction::concrete()};
}

glm::dquat aboutY(double angle) {
  return glm::angleAxis(angle, glm::dvec3(0.0, 1.0, 0.0));
}

Building cantilever(bool supported, const glm::dvec3& position, const glm::dquat& rotation) {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(2.0, 0.865, 0.0), glm::dvec3(1.75, 0.135, 0.25), 0u, Role::Slab});

  if (supported) {
    building.members.push_back(Member{glm::dvec3(2.0, 0.365, 0.0), glm::dvec3(1.75, 0.365, 0.25), 0u, Role::Wall});
  }

  building.position = position;
  building.rotation = rotation;

  return building;
}

Building column(const glm::dvec3& at) {
  Building building;

  building.members.push_back(Member{at + glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.25, 1.5, 0.25), 0u, Role::Column});

  return building;
}

uint32_t bondJoining(const ChunkedBuilding& chunked, uint32_t first, uint32_t second) {
  for (uint32_t index = 0u; index != chunked.bonds.size(); ++index) {
    if (chunked.bonds[index].first == first && chunked.bonds[index].second == second) {
      return index;
    }
  }

  return NO_BOND;
}

const RigidBody& rigidOf(const talus::dynamics::World& world, uint32_t id) {
  const std::optional<talus::dynamics::BodyRef> ref = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == talus::dynamics::Side::Rigid);

  return world.bodies.rigids[ref->index];
}

}

TEST_CASE("a building becomes one zero-mass body holding every chunk under a key unique across the world", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  Building turned = column(glm::dvec3(0.0));

  turned.position = glm::dvec3(2.0, 0.0, -1.0);
  turned.rotation = aboutY(0.5);

  const std::array<Building, 2> sources{cantilever(true, glm::dvec3(0.0), LEVEL), turned};
  std::array<BuildingReport, 2> reports{};

  for (size_t index = 0u; index != sources.size(); ++index) {
    reports[index] = destruction.addBuilding(world, sources[index], concreteOnly(), exact());
  }

  REQUIRE(destruction.buildings() == 2u);
  CHECK(destruction.chunked(0u).chunks.size() == 16u);
  CHECK(destruction.chunked(1u).chunks.size() == 6u);
  CHECK(reports[0].building == 0u);
  CHECK(reports[1].building == 1u);
  CHECK(reports[0].firstKey == 0u);
  CHECK(reports[1].firstKey == 144u);
  CHECK(destruction.firstKey(1u) == 144u);
  CHECK(destruction.keys() == 198u);
  CHECK(destruction.bodies() == std::vector<uint32_t>{reports[0].body, reports[1].body});

  for (uint32_t building = 0u; building != 2u; ++building) {
    INFO("building " << building);

    const BuildingReport& report = reports[building];
    const RigidBody& body = rigidOf(world, report.body);
    const ChunkedBuilding& chunked = destruction.chunked(building);

    CHECK(body.mass == 0.0);
    CHECK(body.shape == talus::dynamics::RigidShape::Compound);
    CHECK(body.position == sources[building].position);
    CHECK(std::abs(glm::dot(body.rotation, glm::normalize(sources[building].rotation))) >= 1.0 - 1.0e-15);
    REQUIRE(body.compound->children.size() == chunked.chunks.size());

    for (uint32_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
      const std::optional<uint32_t> child = talus::dynamics::findChild(*body.compound, report.firstKey + chunk);

      REQUIRE(child.has_value());

      const talus::dynamics::CompoundChild& held = body.compound->children[*child];

      CHECK(held.centre == chunked.chunks[chunk].centre);
      CHECK(held.half == chunked.chunks[chunk].halfExtents);
      CHECK(held.density == chunked.chunks[chunk].mass / chunked.chunks[chunk].volume);
      CHECK(destruction.bodyOf(building, chunk) == report.body);
    }
  }
}

TEST_CASE("adding a building settles it under its own weight, and a slab past its root's strength breaks there at load", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport held = destruction.addBuilding(world, cantilever(true, glm::dvec3(0.0), LEVEL), concreteOnly(), exact());
  const BuildingReport loose = destruction.addBuilding(world, cantilever(false, glm::dvec3(0.0, 0.0, 5.0), LEVEL), concreteOnly(), exact());
  const ChunkedBuilding& chunked = destruction.chunked(1u);
  const uint32_t root = bondJoining(chunked, 1u, 2u);

  CHECK(held.settled);
  CHECK(held.updates >= 1u);
  CHECK(held.rounds >= 1u);
  CHECK(held.factorizations <= held.rounds);
  CHECK(held.broken.empty());
  CHECK(loose.settled);
  REQUIRE(root != NO_BOND);
  CHECK(loose.broken == std::vector<uint32_t>{root});
  CHECK(destruction.bondState(1u, root) == BondState::Broken);
  CHECK(destruction.bondHealth(1u, root) == 0.0);

  for (uint32_t bond = 0u; bond != chunked.bonds.size(); ++bond) {
    if (bond != root) {
      INFO("bond " << bond);
      CHECK(destruction.bondState(1u, bond) != BondState::Broken);
    }
  }
}

TEST_CASE("a building turned in the world settles in its own frame", "[destruction][destructionworld]") {
  for (const double angle : {PI, PI / 6.0}) {
    INFO("turned by " << angle);

    talus::dynamics::World world;
    DestructionWorld destruction;
    const glm::dvec3 place(3.0, 0.0, -2.0);
    const glm::dquat turn = aboutY(angle);
    const BuildingReport report = destruction.addBuilding(world, cantilever(false, place, turn), concreteOnly(), exact());
    const ChunkedBuilding& chunked = destruction.chunked(0u);
    const RigidBody& body = rigidOf(world, report.body);

    CHECK(report.broken == std::vector<uint32_t>{bondJoining(chunked, 1u, 2u)});

    for (uint32_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
      const std::optional<uint32_t> child = talus::dynamics::findChild(*body.compound, report.firstKey + chunk);

      REQUIRE(child.has_value());

      const glm::dvec3 drawn = body.position + body.rotation * body.compound->children[*child].centre;

      CHECK(glm::length(drawn - (place + turn * chunked.chunks[chunk].centre)) <= 1.0e-12);
    }
  }
}

TEST_CASE("the destruction world refuses settings, buildings and indices it cannot use", "[destruction][destructionworld]") {
  DestructionSettings undefined;
  DestructionSettings sticky;
  DestructionSettings bouncy;

  undefined.friction = std::numeric_limits<double>::quiet_NaN();
  sticky.friction = -0.1;
  bouncy.restitution = 1.5;

  for (const DestructionSettings& settings : {undefined, sticky, bouncy}) {
    CHECK_THROWS_AS(DestructionWorld{settings}, std::invalid_argument);
  }

  talus::dynamics::World world;
  DestructionWorld destruction;

  CHECK_THROWS_AS(destruction.setSettings(sticky), std::invalid_argument);
  CHECK(destruction.settings().friction == DestructionSettings{}.friction);

  Building spun = column(glm::dvec3(0.0));

  spun.rotation = glm::dquat(0.0, 0.0, 0.0, 0.0);

  CHECK_THROWS_AS(destruction.addBuilding(world, Building{}, concreteOnly(), exact()), std::invalid_argument);
  CHECK_THROWS_AS(destruction.addBuilding(world, spun, concreteOnly(), exact()), std::invalid_argument);
  CHECK(destruction.buildings() == 0u);
  CHECK(destruction.keys() == 0u);
  CHECK(world.bodies.rigids.empty());

  const BuildingReport built = destruction.addBuilding(world, column(glm::dvec3(0.0)), concreteOnly(), exact());

  CHECK_THROWS_AS(destruction.chunked(1u), std::out_of_range);
  CHECK_THROWS_AS(destruction.firstKey(1u), std::out_of_range);
  CHECK_THROWS_AS(destruction.bodyOf(1u, 0u), std::out_of_range);
  CHECK_THROWS_AS(destruction.bodyOf(built.building, 54u), std::out_of_range);
  CHECK_THROWS_AS(destruction.bondState(built.building, 5u), std::out_of_range);
  CHECK_THROWS_AS(destruction.bondHealth(built.building, 5u), std::out_of_range);
}

namespace {

using talus::destruction::DestructionReport;
using destructionfixture::Motion;

constexpr double CONCRETE_DENSITY = 2450.0;
constexpr double CUBE = 0.125;

StructuralMaterial soil() {
  StructuralMaterial material = talus::destruction::concrete();

  material.chunkSize = 64.0;
  material.fracture = talus::destruction::Fracture::None;
  material.pieces = 1u;

  return material;
}

StructuralMaterial wholeBrick() {
  StructuralMaterial material = talus::destruction::brickMasonry();

  material.fracture = talus::destruction::Fracture::None;
  material.pieces = 1u;

  return material;
}

Building ground() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, -0.5, 0.0), glm::dvec3(20.0, 0.5, 20.0), 0u, Role::Foundation});

  return building;
}

uint32_t addGround(talus::dynamics::World& world, DestructionWorld& destruction) {
  const std::vector<StructuralMaterial> materials{soil()};

  return destruction.addBuilding(world, ground(), materials).body;
}

Building twoColumns() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(0.25, 1.5, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(3.0, 1.5, 0.0), glm::dvec3(0.25, 1.5, 0.25), 0u, Role::Column});

  return building;
}

Building bar(uint32_t chunks, const glm::dvec3& position, const glm::dquat& rotation) {
  Building building;
  const double half = 0.25 * static_cast<double>(chunks);

  building.members.push_back(Member{glm::dvec3(half, 1.25, 0.0), glm::dvec3(half, 0.25, 0.25), 0u, Role::Beam});
  building.position = position;
  building.rotation = rotation;

  return building;
}

RigidBody& movable(talus::dynamics::World& world, uint32_t id) {
  const std::optional<talus::dynamics::BodyRef> ref = talus::dynamics::findBody(world.bodies, id);

  REQUIRE(ref.has_value());
  REQUIRE(ref->side == talus::dynamics::Side::Rigid);

  return world.bodies.rigids[ref->index];
}

std::vector<uint32_t> keysOf(const RigidBody& body) {
  std::vector<uint32_t> keys;

  for (const talus::dynamics::CompoundChild& child : body.compound->children) {
    keys.push_back(child.key);
  }

  return keys;
}

DestructionReport advance(talus::dynamics::World& world, DestructionWorld& destruction) {
  REQUIRE(!talus::dynamics::step(world).nonFinite);

  return destruction.update(world);
}

void checkKept(const Motion& before, const Motion& after) {
  CHECK(glm::length(after.momentum - before.momentum) <= 1.0e-12 * destructionfixture::linearScale(before));
  CHECK(glm::length(after.angular - before.angular) <= 1.0e-12 * destructionfixture::angularScale(before));
  CHECK(std::abs(after.energy - before.energy) <= 1.0e-12 * destructionfixture::energyScale(before));
}

}

TEST_CASE("a column whose base chunk is removed leaves its building as one moving body at rest, and nothing anchored moves", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, twoColumns(), concreteOnly(), exact());
  const RigidBody before = rigidOf(world, built.body);
  const std::array<uint32_t, 1> base{0u};

  CHECK(destruction.removeChunks(world, built.building, base) == 1u);
  CHECK(!destruction.bodyOf(built.building, 0u).has_value());

  const DestructionReport report = destruction.update(world);

  REQUIRE(report.created.size() == 1u);

  const uint32_t falling = report.created[0];

  for (uint32_t chunk = 1u; chunk != 6u; ++chunk) {
    CHECK(destruction.bodyOf(built.building, chunk) == falling);
  }

  for (uint32_t chunk = 6u; chunk != 12u; ++chunk) {
    CHECK(destruction.bodyOf(built.building, chunk) == built.body);
  }

  const RigidBody& piece = rigidOf(world, falling);

  CHECK(std::abs(piece.mass - 5.0 * CONCRETE_DENSITY * CUBE) <= 1.0e-12 * piece.mass);
  CHECK(piece.velocity == glm::dvec3(0.0));
  CHECK(piece.angularVelocity == glm::dvec3(0.0));

  const RigidBody& still = rigidOf(world, built.body);

  CHECK(still.mass == 0.0);
  CHECK(still.position == before.position);
  CHECK(still.rotation == before.rotation);
  CHECK(keysOf(still) == std::vector<uint32_t>{built.firstKey + 6u, built.firstKey + 7u, built.firstKey + 8u, built.firstKey + 9u, built.firstKey + 10u, built.firstKey + 11u});
  CHECK(destructionfixture::mismatch(destruction, world).empty());

  const double top = piece.position.y;

  for (uint32_t step = 0u; step != 60u; ++step) {
    static_cast<void>(advance(world, destruction));
  }

  CHECK(rigidOf(world, falling).position.y < top - 0.4);
  CHECK(rigidOf(world, built.body).position == before.position);
  CHECK(rigidOf(world, built.body).rotation == before.rotation);
  CHECK(destructionfixture::mismatch(destruction, world).empty());
}

TEST_CASE("removing every chunk a body holds removes the body, and removing them again removes nothing", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const uint32_t floor = addGround(world, destruction);
  const BuildingReport built = destruction.addBuilding(world, column(glm::dvec3(0.0)), concreteOnly(), exact());
  const std::vector<uint32_t> all{0u, 1u, 2u, 3u, 4u, 5u};

  CHECK(destruction.removeChunks(world, built.building, all) == 6u);
  CHECK(!talus::dynamics::findBody(world.bodies, built.body).has_value());

  for (uint32_t chunk = 0u; chunk != 6u; ++chunk) {
    CHECK(!destruction.bodyOf(built.building, chunk).has_value());
  }

  CHECK(destruction.bodies() == std::vector<uint32_t>{floor});
  CHECK(destruction.removeChunks(world, built.building, all) == 0u);
  CHECK(destruction.update(world).created.empty());
  CHECK(destructionfixture::mismatch(destruction, world).empty());
}

TEST_CASE("a step's report counts the chunks its stress solves and factorizations covered: a column of six cut below its top solves and factorizes its four free chunks", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport built = destruction.addBuilding(world, column(glm::dvec3(0.0)), concreteOnly(), exact());
  const std::vector<uint32_t> top{5u};

  REQUIRE(destruction.chunked(built.building).chunks.size() == 6u);
  REQUIRE(destruction.removeChunks(world, built.building, top) == 1u);

  const DestructionReport cut = advance(world, destruction);

  CHECK(cut.solved == 1u);
  CHECK(cut.factorizations == 1u);
  CHECK(cut.factorizedChunks == 4u);
  CHECK(cut.solvedChunks == 4u * cut.rounds);
}

TEST_CASE("a step's report counts the factor work of its factorizations: the four free chunks of a cut column of six, a chain, factorize with work 2² + 2² + 2² + 1²", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport built = destruction.addBuilding(world, column(glm::dvec3(0.0)), concreteOnly(), exact());
  const std::vector<uint32_t> top{5u};

  REQUIRE(destruction.removeChunks(world, built.building, top) == 1u);

  const DestructionReport cut = advance(world, destruction);

  REQUIRE(cut.factorizations == 1u);
  CHECK(cut.factorWork == 13u);
  CHECK(cut.wideWork == 0u);
}

TEST_CASE("a removal scheduled for a step happens at the start of that step's update, and its split follows in that update", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, twoColumns(), concreteOnly(), exact());

  destruction.removeAt(3u, built.building, std::vector<uint32_t>{0u});

  for (uint64_t step = 1u; step != 3u; ++step) {
    const DestructionReport early = advance(world, destruction);

    CHECK(early.step == step);
    CHECK(early.removed == 0u);
    CHECK(early.created.empty());
    CHECK(destruction.bodyOf(built.building, 0u) == built.body);
  }

  const DestructionReport due = advance(world, destruction);

  CHECK(due.step == 3u);
  CHECK(due.removed == 1u);
  REQUIRE(due.created.size() == 1u);
  CHECK(!destruction.bodyOf(built.building, 0u).has_value());
  CHECK(destruction.bodyOf(built.building, 1u) == due.created[0]);
  CHECK(advance(world, destruction).removed == 0u);
  CHECK(destruction.steps() == 4u);
}

TEST_CASE("a removal naming a building or chunk the world does not have is refused before anything changes", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport built = destruction.addBuilding(world, twoColumns(), concreteOnly(), exact());
  const std::vector<uint32_t> first{0u};
  const std::vector<uint32_t> partly{0u, 999u};

  CHECK_THROWS_AS(destruction.removeChunks(world, 7u, first), std::out_of_range);
  CHECK_THROWS_AS(destruction.removeChunks(world, built.building, partly), std::out_of_range);
  CHECK(destruction.bodyOf(built.building, 0u) == built.body);
  CHECK_THROWS_AS(destruction.removeAt(1u, 7u, first), std::out_of_range);
  CHECK_THROWS_AS(destruction.removeAt(1u, built.building, partly), std::out_of_range);

  const DestructionReport report = destruction.update(world);

  CHECK(report.removed == 0u);
  CHECK(report.created.empty());
  CHECK(rigidOf(world, built.body).compound->children.size() == 12u);
}

TEST_CASE("chunks that break off while a building settles leave it as one moving body in the first update", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport built = destruction.addBuilding(world, cantilever(false, glm::dvec3(0.0), LEVEL), concreteOnly(), exact());

  REQUIRE(built.broken.size() == 1u);

  for (uint32_t chunk = 0u; chunk != 9u; ++chunk) {
    CHECK(destruction.bodyOf(built.building, chunk) == built.body);
  }

  const DestructionReport report = destruction.update(world);

  REQUIRE(report.created.size() == 1u);
  CHECK(destruction.bodyOf(built.building, 0u) == built.body);
  CHECK(destruction.bodyOf(built.building, 1u) == built.body);

  for (uint32_t chunk = 2u; chunk != 9u; ++chunk) {
    CHECK(destruction.bodyOf(built.building, chunk) == report.created[0]);
  }

  CHECK(destructionfixture::mismatch(destruction, world).empty());
}

TEST_CASE("a moving piece that loses two chunks splits three ways in its next update and keeps its momentum", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport built = destruction.addBuilding(world, bar(5u, glm::dvec3(0.0, 10.0, 0.0), LEVEL), concreteOnly(), exact());

  REQUIRE(built.settled);

  const DestructionReport departed = destruction.update(world);

  REQUIRE(departed.created.size() == 1u);
  CHECK(!talus::dynamics::findBody(world.bodies, built.body).has_value());

  const uint32_t piece = departed.created[0];

  movable(world, piece).velocity = glm::dvec3(2.0, 1.0, 0.0);
  movable(world, piece).angularVelocity = glm::dvec3(0.0, 3.0, 1.0);
  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const std::vector<uint32_t> middle{1u, 3u};

  CHECK(destruction.removeChunks(world, built.building, middle) == 2u);

  const Motion before = destructionfixture::motionOf(world);
  const DestructionReport split = destruction.update(world);
  const Motion after = destructionfixture::motionOf(world);

  REQUIRE(split.created.size() == 2u);
  CHECK(destruction.bodyOf(built.building, 0u) == piece);
  CHECK(destruction.bodyOf(built.building, 2u) == split.created[0]);
  CHECK(destruction.bodyOf(built.building, 4u) == split.created[1]);
  CHECK(keysOf(rigidOf(world, piece)) == std::vector<uint32_t>{built.firstKey});
  checkKept(before, after);
  CHECK(destructionfixture::mismatch(destruction, world).empty());
}

namespace {

using talus::destruction::BrokenBond;
using talus::destruction::ChunkLoad;

constexpr uint32_t CRATE_KEY = 1000000u;
constexpr double CRATE_HALF = 0.1;
constexpr double CRATE_DENSITY = 12500.0;
constexpr double G = 9.81;
constexpr double BLOCK_DENSITY = 125000.0;
constexpr double GAP = 0.01;
constexpr double PIER_OFFSET = 0.12;
constexpr double PIER_SPEED = 5.0;
constexpr double POST_SPEED = 1.0;
constexpr uint32_t HIT_STEPS = 300u;

bool near(double value, double expected, double tolerance) {
  return std::abs(value - expected) <= tolerance;
}

Building roof(const glm::dvec3& position, const glm::dquat& rotation) {
  Building building;

  building.members.push_back(Member{glm::dvec3(-0.5, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(0.5, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Foundation});
  building.members.push_back(Member{glm::dvec3(0.0, 1.25, 0.0), glm::dvec3(0.75, 0.25, 0.25), 0u, Role::Slab});
  building.position = position;
  building.rotation = rotation;

  return building;
}

Building stack() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.1, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(0.0, 2.5, 0.0), glm::dvec3(0.25, 0.25, 0.25), 0u, Role::Column});

  return building;
}

Building brickPier(const glm::dvec3& at) {
  Building building;

  building.members.push_back(Member{at + glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});

  return building;
}

uint32_t addBlock(talus::dynamics::World& world, const glm::dvec3& centre, const glm::dvec3& velocity, uint32_t key) {
  const std::array<talus::dynamics::CompoundChild, 1> block{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(CRATE_HALF), BLOCK_DENSITY, key}
  };
  const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, block, centre, LEVEL, velocity, 0.6, 0.0, false);

  return world.bodies.rigids[index].id;
}

std::vector<uint64_t> bondsIn(const DestructionWorld& destruction, BondState state) {
  std::vector<uint64_t> found;

  for (uint32_t building = 0u; building != destruction.buildings(); ++building) {
    const auto bonds = static_cast<uint32_t>(destruction.chunked(building).bonds.size());

    for (uint32_t bond = 0u; bond != bonds; ++bond) {
      if (destruction.bondState(building, bond) == state) {
        found.push_back((uint64_t{building} << 32u) | bond);
      }
    }
  }

  return found;
}

std::vector<std::vector<uint64_t>> groupsOf(const DestructionWorld& destruction) {
  std::vector<std::vector<uint64_t>> groups;

  for (const uint32_t body : destruction.bodies()) {
    std::vector<uint64_t> held;

    for (uint32_t building = 0u; building != destruction.buildings(); ++building) {
      const auto chunks = static_cast<uint32_t>(destruction.chunked(building).chunks.size());

      for (uint32_t chunk = 0u; chunk != chunks; ++chunk) {
        if (destruction.bodyOf(building, chunk) == body) {
          held.push_back((uint64_t{building} << 32u) | chunk);
        }
      }
    }

    groups.push_back(std::move(held));
  }

  std::sort(groups.begin(), groups.end());

  return groups;
}

struct HitOutcome {
  uint32_t firstLoads = 0u;
  uint32_t pendingSteps = 0u;
  uint32_t extraRounds = 0u;
  uint32_t pendingAtEnd = 0u;
  size_t created = 0u;
  std::vector<uint64_t> broken;
  std::vector<uint64_t> cracked;
  std::vector<std::vector<uint64_t>> groups;
};

HitOutcome hitPierAndColumn(const DestructionSettings& settings) {
  talus::dynamics::World world;
  DestructionWorld destruction(settings);
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  const BuildingReport pier = destruction.addBuilding(world, brickPier(glm::dvec3(-3.0, 0.0, 0.0)), brick, exact());
  const BuildingReport post = destruction.addBuilding(world, column(glm::dvec3(3.0, 0.0, 0.0)), concreteOnly(), exact());

  REQUIRE(pier.settled);
  REQUIRE(post.settled);
  REQUIRE(pier.cracked.empty());
  REQUIRE(post.cracked.empty());

  const std::array<uint32_t, 2> blocks{
    addBlock(world, glm::dvec3(-3.0 + PIER_OFFSET, 1.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY),
    addBlock(world, glm::dvec3(3.0 - 0.25 - CRATE_HALF - GAP, 2.75, 0.0), glm::dvec3(POST_SPEED, 0.0, 0.0), CRATE_KEY + 1u)
  };
  HitOutcome outcome;

  for (uint32_t step = 0u; step != HIT_STEPS; ++step) {
    const DestructionReport report = advance(world, destruction);

    if (step == 0u) {
      outcome.firstLoads = report.loads;

      for (const uint32_t block : blocks) {
        REQUIRE(talus::dynamics::removeBody(world, block));
      }
    }

    outcome.pendingSteps += report.pending != 0u ? 1u : 0u;
    outcome.extraRounds += report.rounds > report.solved ? report.rounds - report.solved : 0u;
    outcome.pendingAtEnd = report.pending;
    outcome.created += report.created.size();
  }

  outcome.broken = bondsIn(destruction, BondState::Broken);
  outcome.cracked = bondsIn(destruction, BondState::Cracked);
  outcome.groups = groupsOf(destruction);

  return outcome;
}

uint32_t addCrate(talus::dynamics::World& world, const glm::dvec3& centre, const glm::dquat& turn) {
  const std::array<talus::dynamics::CompoundChild, 1> crate{
    talus::dynamics::CompoundChild{glm::dvec3(0.0), LEVEL, glm::dvec3(CRATE_HALF), CRATE_DENSITY, CRATE_KEY}
  };
  const uint32_t index = talus::dynamics::addCompoundBody(world.bodies, crate, centre, turn, glm::dvec3(0.0), 0.6, 0.0, false);

  return world.bodies.rigids[index].id;
}

std::optional<ChunkLoad> loadOn(const DestructionWorld& destruction, uint32_t building, uint32_t chunk) {
  for (const ChunkLoad& load : destruction.chunkLoads(building)) {
    if (load.chunk == chunk) {
      return load;
    }
  }

  return std::nullopt;
}

}

TEST_CASE("a crate resting on a roof chunk loads that chunk with its weight, from a point on the chunk's top face", "[destruction][destructionworld]") {
  for (const double angle : {0.0, PI / 6.0}) {
    INFO("turned by " << angle);

    talus::dynamics::World world;
    DestructionWorld destruction(exactLoads());

    static_cast<void>(addGround(world, destruction));

    const glm::dvec3 place(2.0, 0.0, 1.0);
    const glm::dquat turn = aboutY(angle);
    const BuildingReport built = destruction.addBuilding(world, roof(place, turn), concreteOnly(), exact());
    const glm::dvec3 seat = destruction.chunked(built.building).chunks[5].centre + glm::dvec3(0.1, 0.25 + CRATE_HALF, 0.0);
    const uint32_t crate = addCrate(world, place + turn * seat, turn);
    const double weight = rigidOf(world, crate).mass * G;

    for (uint32_t step = 0u; step != 180u; ++step) {
      static_cast<void>(advance(world, destruction));
    }

    const std::optional<ChunkLoad> load = loadOn(destruction, built.building, 5u);

    REQUIRE(load.has_value());
    CHECK(near(load->force.y, -weight, 0.01 * weight));
    CHECK(near(load->force.x, 0.0, 0.01 * weight));
    CHECK(near(load->force.z, 0.0, 0.01 * weight));
    CHECK(near(-load->torque.z / weight, 0.1, 0.01));
    CHECK(near(load->torque.x / weight, 0.0, 0.01));

    for (const ChunkLoad& other : destruction.chunkLoads(built.building)) {
      if (other.chunk != 5u) {
        INFO("chunk " << other.chunk);
        CHECK(glm::length(other.force) <= 0.01 * weight);
      }
    }
  }
}

TEST_CASE("a crate settling on a roof asks for updates whose small load changes the tolerance skips and counts, step for step where the same scene without it solves", "[destruction][destructionworld]") {
  const auto settle = [](bool tolerance) {
    talus::dynamics::World world;
    DestructionSettings settings;

    settings.tolerance = tolerance;

    DestructionWorld destruction(settings);

    static_cast<void>(addGround(world, destruction));

    const glm::dvec3 place(2.0, 0.0, 1.0);
    const BuildingReport built = destruction.addBuilding(world, roof(place, LEVEL), concreteOnly(), exact());
    const glm::dvec3 seat = destruction.chunked(built.building).chunks[5].centre + glm::dvec3(0.1, 0.25 + CRATE_HALF, 0.0);

    static_cast<void>(addCrate(world, place + seat, LEVEL));

    std::vector<std::pair<uint32_t, uint32_t>> steps;

    for (uint32_t step = 0u; step != 180u; ++step) {
      const DestructionReport report = advance(world, destruction);

      REQUIRE(report.broken.empty());
      steps.emplace_back(report.solved, report.tolerated);
    }

    return steps;
  };
  const std::vector<std::pair<uint32_t, uint32_t>> with = settle(true);
  const std::vector<std::pair<uint32_t, uint32_t>> without = settle(false);
  uint32_t tolerated = 0u;
  uint32_t solved = 0u;

  for (size_t step = 0u; step != with.size(); ++step) {
    INFO("step " << step);
    CHECK(with[step].first + with[step].second == without[step].first);
    CHECK(without[step].second == 0u);
    tolerated += with[step].second;
    solved += with[step].first;
  }

  CHECK(tolerated > 0u);
  CHECK(solved > 0u);
  CHECK(tolerated > solved);
}

TEST_CASE("a piece standing on end carries its top chunk's weight through its bond as compression", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction(exactLoads());

  static_cast<void>(addGround(world, destruction));

  const glm::dquat upright = glm::angleAxis(PI / 2.0, glm::dvec3(0.0, 0.0, 1.0));
  const BuildingReport built = destruction.addBuilding(world, bar(2u, glm::dvec3(0.0, 0.1, 0.0), upright), concreteOnly(), exact());

  for (uint32_t step = 0u; step != 240u; ++step) {
    static_cast<void>(advance(world, destruction));
  }

  const talus::destruction::Bond& bond = destruction.chunked(built.building).bonds[0];
  const glm::dvec3 force = destruction.bondForce(built.building, 0u);
  const double push = glm::dot(force, bond.normal);
  const double weight = CONCRETE_DENSITY * CUBE * G;

  CHECK(near(push, weight, 0.01 * weight));
  CHECK(glm::length(force - push * bond.normal) <= 0.01 * weight);
}

TEST_CASE("pieces of one building resting on each other load each other, equal and opposite", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction(exactLoads());

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, stack(), concreteOnly(), exact());

  CHECK(advance(world, destruction).created.size() == 2u);

  for (uint32_t step = 0u; step != 300u; ++step) {
    static_cast<void>(advance(world, destruction));
  }

  const double weight = CONCRETE_DENSITY * CUBE * G;
  const std::optional<ChunkLoad> lower = loadOn(destruction, built.building, 0u);
  const std::optional<ChunkLoad> middle = loadOn(destruction, built.building, 1u);
  const std::optional<ChunkLoad> upper = loadOn(destruction, built.building, 2u);

  REQUIRE(lower.has_value());
  REQUIRE(middle.has_value());
  REQUIRE(upper.has_value());
  CHECK(near(upper->force.y, weight, 0.01 * weight));
  CHECK(near(middle->force.y, -weight, 0.01 * weight));
  CHECK(near(lower->force.y, 3.0 * weight, 0.03 * weight));
  CHECK(glm::length(upper->force + middle->force) <= 0.01 * weight);
}

TEST_CASE("a body falling freely stresses none of its bonds, even while it spins", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, bar(4u, glm::dvec3(0.0, 8.0, 0.0), LEVEL), concreteOnly(), exact());
  const DestructionReport departed = advance(world, destruction);

  REQUIRE(departed.created.size() == 1u);
  CHECK(!talus::dynamics::findBody(world.bodies, built.body).has_value());

  movable(world, departed.created[0]).angularVelocity = glm::dvec3(0.4, 1.0, 0.7);

  for (uint32_t step = 0u; step != 40u; ++step) {
    const DestructionReport report = advance(world, destruction);

    CHECK(report.solved == 0u);
    CHECK(report.pending == 0u);

    for (uint32_t bond = 0u; bond != 3u; ++bond) {
      CHECK(destruction.bondForce(built.building, bond) == glm::dvec3(0.0));
      CHECK(destruction.bondMoment(built.building, bond) == glm::dvec3(0.0));
    }
  }
}

TEST_CASE("a cantilever slab whose support is removed breaks first at its root, the bond beam theory names, however the building is turned", "[destruction][destructionworld]") {
  for (const DestructionSettings& settings : {exceeded(), unlimited()}) {
    for (const double angle : {0.0, PI, PI / 6.0}) {
      INFO("turned by " << angle << ", target " << settings.target << " ms");

      talus::dynamics::World world;
      DestructionWorld destruction(settings);
      const BuildingReport built = destruction.addBuilding(world, cantilever(true, glm::dvec3(1.0, 0.0, -2.0), aboutY(angle)), concreteOnly(), exact());

      REQUIRE(built.settled);
      CHECK(built.broken.empty());

      const uint32_t root = bondJoining(destruction.chunked(built.building), 1u, 2u);
      const std::vector<uint32_t> support{9u, 10u, 11u, 12u, 13u, 14u, 15u};

      REQUIRE(root != NO_BOND);
      CHECK(destruction.removeChunks(world, built.building, support) == 7u);

      DestructionReport first;
      uint32_t updates = 0u;

      while (first.broken.empty() && updates != 60u) {
        first = advance(world, destruction);
        ++updates;
      }

      CHECK(first.broken == std::vector<BrokenBond>{BrokenBond{built.building, root}});
      REQUIRE(first.created.size() == 1u);

      for (uint32_t chunk = 2u; chunk != 9u; ++chunk) {
        CHECK(destruction.bodyOf(built.building, chunk) == first.created[0]);
      }

      CHECK(destruction.bodyOf(built.building, 0u) == built.body);
      CHECK(destruction.bodyOf(built.building, 1u) == built.body);
    }
  }
}

TEST_CASE("a one-step hit served one crack round per update per step ends with the cracks, breaks and splits it has with no round limit", "[destruction][destructionworld]") {
  const HitOutcome budget = hitPierAndColumn(exceeded());
  const HitOutcome whole = hitPierAndColumn(unlimited());

  INFO("one round per step: " << budget.broken.size() << " broken, " << budget.cracked.size() << " cracked, "
       << budget.groups.size() << " bodies, pending in " << budget.pendingSteps << " steps");
  INFO("no limit: " << whole.broken.size() << " broken, " << whole.cracked.size() << " cracked, "
       << whole.groups.size() << " bodies, pending in " << whole.pendingSteps << " steps");

  REQUIRE(budget.firstLoads == 2u);
  REQUIRE(whole.firstLoads == 2u);
  REQUIRE(!whole.broken.empty());
  REQUIRE(!whole.cracked.empty());
  REQUIRE(whole.created != 0u);
  REQUIRE(budget.pendingSteps != 0u);
  CHECK(whole.pendingSteps == 0u);
  CHECK(budget.extraRounds == 0u);
  CHECK(budget.pendingAtEnd == 0u);
  CHECK(budget.broken == whole.broken);
  CHECK(budget.cracked == whole.cracked);
  CHECK(budget.groups == whole.groups);
}

namespace {

constexpr double WALL_SPEED = 2.0;

struct WallHits {
  std::vector<uint32_t> rounds;
  std::vector<uint32_t> solved;
  std::vector<uint32_t> pending;
  std::vector<uint64_t> broken;
  std::vector<uint64_t> cracked;
  std::vector<std::vector<uint64_t>> groups;
  std::vector<std::vector<ChunkLoad>> queuedBefore;
  std::vector<std::vector<std::vector<ChunkLoad>>> queuedAfter;
  std::vector<std::vector<uint32_t>> heldAfter;
  std::vector<uint32_t> cutChunks;
  uint32_t pendingBeforeCut = 0u;
  uint32_t casesStarted = 0u;
  uint32_t startedAt = NO_BOND;
  std::vector<std::vector<ChunkLoad>> queuedAfterSecond;
  std::vector<ChunkLoad> loadsAfterSecond;
};

Building brickWall() {
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 1.5, 0.0), glm::dvec3(1.0, 1.5, 0.125), 0u, Role::Wall});

  return building;
}

WallHits hitWall(const DestructionSettings& settings, bool pierToo, bool secondHit, bool cut, bool firstHit = true) {
  talus::dynamics::World world;
  DestructionWorld destruction(settings);
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  const BuildingReport wall = destruction.addBuilding(world, brickWall(), brick, exact());
  const BuildingReport pier = destruction.addBuilding(world, brickPier(glm::dvec3(-3.0, 0.0, 0.0)), brick, exact());
  WallHits outcome;

  for (uint32_t step = 0u; step != HIT_STEPS; ++step) {
    std::vector<uint32_t> blocks;

    if (step == 0u) {
      blocks.push_back(
        firstHit ? addBlock(world, glm::dvec3(0.0, 2.5, -0.125 - CRATE_HALF - GAP), glm::dvec3(0.0, 0.0, WALL_SPEED), CRATE_KEY + 1u)
                 : addBlock(world, glm::dvec3(50.0, 2.5, 0.0), glm::dvec3(0.0), CRATE_KEY + 1u)
      );

      if (pierToo) {
        blocks.push_back(
          addBlock(world, glm::dvec3(-3.0 + PIER_OFFSET, 1.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY)
        );
      }
    }

    if (step == 1u && secondHit) {
      blocks.push_back(
        addBlock(world, glm::dvec3(0.6, 1.25, 0.125 + CRATE_HALF + GAP), glm::dvec3(0.0, 0.0, -WALL_SPEED), CRATE_KEY + 2u)
      );
    }

    if (step == 2u && cut) {
      outcome.queuedBefore = destruction.queuedLoads(wall.body);

      std::vector<double> heights;

      for (uint32_t chunk = 0u; chunk != destruction.chunkCount(wall.building); ++chunk) {
        heights.push_back(destruction.chunkOf(wall.building, chunk).centre.y);
      }

      std::vector<double> rows = heights;

      std::sort(rows.begin(), rows.end());
      rows.erase(std::unique(rows.begin(), rows.end()), rows.end());
      REQUIRE(rows.size() >= 3u);

      for (uint32_t chunk = 0u; chunk != heights.size(); ++chunk) {
        if (heights[chunk] == rows[1]) {
          outcome.cutChunks.push_back(chunk);
        }
      }

      REQUIRE(destruction.removeChunks(world, wall.building, outcome.cutChunks) == outcome.cutChunks.size());
    }

    const DestructionReport report = advance(world, destruction);

    for (const uint32_t block : blocks) {
      REQUIRE(talus::dynamics::removeBody(world, block));
    }

    outcome.rounds.push_back(report.rounds);
    outcome.solved.push_back(report.solved);
    outcome.pending.push_back(report.pending);
    outcome.casesStarted += report.casesStarted;

    if (report.casesStarted != 0u && outcome.startedAt == NO_BOND) {
      outcome.startedAt = step;
    }

    if (step == 1u) {
      outcome.pendingBeforeCut = report.pending;
      outcome.queuedAfterSecond = destruction.queuedLoads(wall.body);
      outcome.loadsAfterSecond = destruction.chunkLoads(wall.building);
    }

    if (step == 2u && cut) {
      for (const uint32_t body : destruction.bodies()) {
        std::vector<uint32_t> held;

        for (uint32_t chunk = 0u; chunk != destruction.chunkCount(wall.building); ++chunk) {
          if (destruction.bodyOf(wall.building, chunk) == body) {
            held.push_back(chunk);
          }
        }

        if (!held.empty()) {
          outcome.heldAfter.push_back(held);
          outcome.queuedAfter.push_back(destruction.queuedLoads(body));
        }
      }
    }
  }

  static_cast<void>(pier);
  outcome.broken = bondsIn(destruction, BondState::Broken);
  outcome.cracked = bondsIn(destruction, BondState::Cracked);
  outcome.groups = groupsOf(destruction);

  return outcome;
}

}

TEST_CASE("under a target every step exceeds, each pending update runs one round per step: a wall's update of 5 rounds ends in 5 steps while a pier's runs beside it", "[destruction][destructionworld]") {
  const WallHits wallAlone = hitWall(unlimited(), false, false, false);
  const WallHits both = hitWall(unlimited(), true, false, false);
  const WallHits wallBudget = hitWall(exceeded(), false, false, false);
  const WallHits budget = hitWall(exceeded(), true, false, false);
  const uint32_t wallRounds = wallAlone.rounds[0];
  const uint32_t pierRounds = both.rounds[0] - wallRounds;

  INFO("the wall's update takes " << wallRounds << " rounds, the pier's " << pierRounds);
  REQUIRE(wallRounds == 5u);
  REQUIRE(pierRounds >= 2u);

  for (uint32_t step = 0u; step != wallRounds; ++step) {
    INFO("step " << step);
    CHECK(wallBudget.rounds[step] == 1u);
    CHECK(wallBudget.pending[step] == (step + 1u < wallRounds ? 1u : 0u));
  }

  for (uint32_t step = 0u; step != std::min(wallRounds, pierRounds); ++step) {
    INFO("step " << step);
    CHECK(budget.rounds[step] == 2u);
  }

  CHECK(wallBudget.groups == wallAlone.groups);
  CHECK(budget.groups == both.groups);
}

TEST_CASE("a hit on a wall whose update is pending, in its second round, is kept as a load case holding exactly the loads the hit put on the wall, and starts when the pending update ends", "[destruction][destructionworld]") {
  const WallHits budget = hitWall(exceeded(), false, true, false);
  const WallHits second = hitWall(unlimited(), false, true, false, false);
  const WallHits single = hitWall(unlimited(), false, false, false);
  const uint32_t firstRounds = single.rounds[0];

  REQUIRE(firstRounds >= 3u);
  REQUIRE(budget.rounds[1] == 1u);
  REQUIRE(budget.pendingBeforeCut == 1u);
  REQUIRE(budget.queuedAfterSecond.size() == 1u);
  REQUIRE(!second.loadsAfterSecond.empty());

  const std::vector<ChunkLoad>& queued = budget.queuedAfterSecond.front();

  REQUIRE(queued.size() == second.loadsAfterSecond.size());

  for (size_t at = 0u; at != queued.size(); ++at) {
    INFO("load " << at);
    CHECK(queued[at].chunk == second.loadsAfterSecond[at].chunk);
    CHECK(queued[at].force == second.loadsAfterSecond[at].force);
    CHECK(queued[at].torque == second.loadsAfterSecond[at].torque);
  }

  CHECK(budget.startedAt == firstRounds);
}

TEST_CASE("a removal that splits a wall with queued load cases gives each new piece exactly its own chunks' queued loads", "[destruction][destructionworld]") {
  const WallHits cut = hitWall(exceeded(), false, true, true);
  std::vector<std::pair<uint32_t, glm::dvec3>> before;
  std::vector<std::pair<uint32_t, glm::dvec3>> after;
  const auto order = [](const std::pair<uint32_t, glm::dvec3>& a, const std::pair<uint32_t, glm::dvec3>& b) {
    return std::tie(a.first, a.second.x, a.second.y, a.second.z) < std::tie(b.first, b.second.x, b.second.y, b.second.z);
  };

  REQUIRE(cut.pendingBeforeCut != 0u);
  REQUIRE(!cut.queuedBefore.empty());
  REQUIRE(cut.heldAfter.size() >= 2u);

  for (const std::vector<ChunkLoad>& queued : cut.queuedBefore) {
    for (const ChunkLoad& load : queued) {
      if (!std::binary_search(cut.cutChunks.begin(), cut.cutChunks.end(), load.chunk)) {
        before.emplace_back(load.chunk, load.force);
      }
    }
  }

  for (size_t piece = 0u; piece != cut.heldAfter.size(); ++piece) {
    for (const std::vector<ChunkLoad>& queued : cut.queuedAfter[piece]) {
      for (const ChunkLoad& load : queued) {
        INFO("chunk " << load.chunk);
        CHECK(std::binary_search(cut.heldAfter[piece].begin(), cut.heldAfter[piece].end(), load.chunk));

        if (load.force != glm::dvec3(0.0)) {
          after.emplace_back(load.chunk, load.force);
        }
      }
    }
  }

  std::sort(before.begin(), before.end(), order);
  std::sort(after.begin(), after.end(), order);
  REQUIRE(!before.empty());
  CHECK(after == before);
}

TEST_CASE("fracturing a pending wall preserves its active hit and remaps later queued hits onto the finer chunks", "[destruction][destructionworld][budgetcompletion]") {
  talus::dynamics::World world;
  DestructionWorld destruction(exceeded());
  const std::vector<StructuralMaterial> brick{talus::destruction::brickMasonry()};
  const BuildingReport wall = destruction.addBuilding(world, brickWall(), brick, exact());
  const uint32_t first = addBlock(world, glm::dvec3(0.0, 2.5, -0.125 - CRATE_HALF - GAP), glm::dvec3(0.0, 0.0, WALL_SPEED), CRATE_KEY + 1u);

  REQUIRE(advance(world, destruction).pending > 0u);
  REQUIRE(talus::dynamics::removeBody(world, first));
  const uint32_t second = addBlock(world, glm::dvec3(0.6, 1.25, 0.125 + CRATE_HALF + GAP), glm::dvec3(0.0, 0.0, -WALL_SPEED), CRATE_KEY + 2u);

  REQUIRE(advance(world, destruction).pending > 0u);
  REQUIRE(talus::dynamics::removeBody(world, second));
  const std::vector<std::vector<ChunkLoad>> before = destruction.queuedLoads(wall.body);
  REQUIRE(!before.empty());
  REQUIRE(!before.back().empty());
  const uint32_t chunk = before.back().front().chunk;
  const std::array<uint32_t, 1> breaking{chunk};
  glm::dvec3 force(0.0);
  glm::dvec3 moment(0.0);

  for (const ChunkLoad& load : before.back()) {
    force += load.force;
    moment += load.torque + glm::cross(destruction.chunkOf(wall.building, load.chunk).centre, load.force);
  }

  REQUIRE(destruction.fractureChunks(world, wall.building, breaking) == 1u);
  const std::vector<std::vector<ChunkLoad>> after = destruction.queuedLoads(wall.body);
  REQUIRE(after.size() == before.size());
  glm::dvec3 remappedForce(0.0);
  glm::dvec3 remappedMoment(0.0);

  for (const ChunkLoad& load : after.back()) {
    CHECK(load.chunk != chunk);
    remappedForce += load.force;
    remappedMoment += load.torque + glm::cross(destruction.chunkOf(wall.building, load.chunk).centre, load.force);
  }

  CHECK(glm::length(force - remappedForce) <= 1.0e-9 * std::max(1.0, glm::length(force)));
  CHECK(glm::length(moment - remappedMoment) <= 1.0e-9 * std::max(1.0, glm::length(moment)));
  const DestructionReport restarted = advance(world, destruction);
  CHECK(restarted.casesStarted == 0u);
  CHECK(restarted.rounds > 0u);
}

struct ThrowOutcome {
  bool threw = false;
  size_t postLoadsNext = 0u;
  std::vector<uint64_t> postBroken;
  std::vector<uint64_t> postCracked;
  std::vector<std::vector<uint64_t>> postGroups;
  std::string mismatch;
};

std::vector<uint64_t> inBuilding(const std::vector<uint64_t>& keys, uint32_t building) {
  std::vector<uint64_t> kept;

  for (const uint64_t key : keys) {
    if ((key >> 32u) == building) {
      kept.push_back(key);
    }
  }

  return kept;
}

ThrowOutcome hitPostBesideAPier(bool pierThrows) {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  const BuildingReport pier = destruction.addBuilding(world, brickPier(glm::dvec3(-3.0, 0.0, 0.0)), brick, exact());
  const BuildingReport post = destruction.addBuilding(world, column(glm::dvec3(3.0, 0.0, 0.0)), concreteOnly(), exact());
  const std::array<uint32_t, 2> blocks{
    addBlock(world, glm::dvec3(-3.0 + PIER_OFFSET, 1.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY),
    addBlock(world, glm::dvec3(3.0 - 0.25 - CRATE_HALF - GAP, 2.75, 0.0), glm::dvec3(POST_SPEED, 0.0, 0.0), CRATE_KEY + 1u)
  };
  ThrowOutcome outcome;

  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const glm::dquat upright = movable(world, pier.body).rotation;

  if (pierThrows) {
    movable(world, pier.body).rotation = glm::dquat(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
  }

  try {
    static_cast<void>(destruction.update(world));
  } catch (const std::exception&) {
    outcome.threw = true;
  }

  movable(world, pier.body).rotation = upright;

  for (const uint32_t block : blocks) {
    REQUIRE(talus::dynamics::removeBody(world, block));
  }

  static_cast<void>(advance(world, destruction));
  outcome.postLoadsNext = destruction.chunkLoads(post.building).size();

  for (uint32_t step = 1u; step != HIT_STEPS; ++step) {
    static_cast<void>(advance(world, destruction));
  }

  outcome.postBroken = inBuilding(bondsIn(destruction, BondState::Broken), post.building);
  outcome.postCracked = inBuilding(bondsIn(destruction, BondState::Cracked), post.building);

  for (const std::vector<uint64_t>& group : groupsOf(destruction)) {
    if (!group.empty() && (group.front() >> 32u) == post.building) {
      outcome.postGroups.push_back(group);
    }
  }

  outcome.mismatch = destructionfixture::mismatch(destruction, world);

  return outcome;
}

TEST_CASE("a solve that throws leaves the other solves of its step applied: a post hit beside a pier whose solve throws solves its own new loads next, not its hit again, and ends as it does when nothing throws", "[destruction][destructionworld]") {
  const ThrowOutcome thrown = hitPostBesideAPier(true);
  const ThrowOutcome clean = hitPostBesideAPier(false);

  REQUIRE(thrown.threw);
  REQUIRE(!clean.threw);
  REQUIRE(!clean.postBroken.empty());
  CHECK(thrown.postLoadsNext == 0u);
  CHECK(thrown.postBroken == clean.postBroken);
  CHECK(thrown.postCracked == clean.postCracked);
  CHECK(thrown.postGroups == clean.postGroups);
  CHECK(thrown.mismatch.empty());
}

std::pair<uint32_t, size_t> visitsAfterACut(bool cutThrows) {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const BuildingReport cut = destruction.addBuilding(world, column(glm::dvec3(0.0)), concreteOnly(), exact());
  const std::vector<uint32_t> top{5u};

  static_cast<void>(destruction.addBuilding(world, column(glm::dvec3(10.0, 0.0, 0.0)), concreteOnly(), exact()));

  for (uint32_t step = 0u; step != 5u; ++step) {
    static_cast<void>(advance(world, destruction));
  }

  REQUIRE(destruction.removeChunks(world, cut.building, top) == 1u);
  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const glm::dquat upright = movable(world, cut.body).rotation;

  if (cutThrows) {
    movable(world, cut.body).rotation = glm::dquat(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
    CHECK_THROWS(destruction.update(world));
  } else {
    CHECK_NOTHROW(destruction.update(world));
  }

  movable(world, cut.body).rotation = upright;

  const DestructionReport next = advance(world, destruction);

  return {next.visited, destruction.bodies().size()};
}

TEST_CASE("after an update throws, the next update visits every piece", "[destruction][destructionworld]") {
  const auto [thrownVisits, thrownPieces] = visitsAfterACut(true);
  const auto [cleanVisits, cleanPieces] = visitsAfterACut(false);

  CHECK(thrownVisits == thrownPieces);
  CHECK(cleanVisits < cleanPieces);
}

TEST_CASE("a contact force that is not finite is refused and counted when loads are gathered, and the updates after it run", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  const BuildingReport pier = destruction.addBuilding(world, brickPier(glm::dvec3(0.0)), brick, exact());
  const uint32_t block =
    addBlock(world, glm::dvec3(PIER_OFFSET, 1.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY);

  REQUIRE(!talus::dynamics::step(world).nonFinite);

  const auto touching = [&]() -> talus::dynamics::CachedContactPoint* {
    for (const uint32_t slot : world.manifolds.pairsOf(pier.body)) {
      talus::dynamics::CachedBodyPair& pair = world.manifolds.at(slot);

      if (world.manifolds.live(slot) && !pair.points.empty() && (pair.firstId == block || pair.secondId == block)) {
        return &pair.points.front();
      }
    }

    return nullptr;
  };

  REQUIRE(touching() != nullptr);

  const double kept = touching()->memory.impulse;

  touching()->memory.impulse = std::numeric_limits<double>::quiet_NaN();

  DestructionReport report;

  REQUIRE_NOTHROW(report = destruction.update(world));
  CHECK(report.refused == 1u);
  REQUIRE(touching() != nullptr);
  touching()->memory.impulse = kept;

  for (uint32_t step = 0u; step != 30u; ++step) {
    REQUIRE_NOTHROW(report = advance(world, destruction));
    CHECK(report.refused == 0u);
  }

  for (uint32_t bond = 0u; bond != destruction.chunked(pier.building).bonds.size(); ++bond) {
    const glm::dvec3 force = destruction.bondForce(pier.building, bond);

    CHECK(std::isfinite(force.x));
    CHECK(std::isfinite(force.y));
    CHECK(std::isfinite(force.z));
  }
}

TEST_CASE("a settled building is not solved again while nothing touches it", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));
  static_cast<void>(destruction.addBuilding(world, cantilever(true, glm::dvec3(0.0), LEVEL), concreteOnly(), exact()));

  for (uint32_t step = 0u; step != 60u; ++step) {
    const DestructionReport report = advance(world, destruction);

    CHECK(report.solved == 0u);
    CHECK(report.created.empty());
    CHECK(report.broken.empty());
  }
}

TEST_CASE("a dropped bar that breaks on landing splits into bodies that keep its momentum, angular momentum and kinetic energy", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const std::vector<StructuralMaterial> brick{talus::destruction::brickMasonry()};

  static_cast<void>(addGround(world, destruction));

  static_cast<void>(destruction.addBuilding(world, bar(4u, glm::dvec3(0.0, 2.0, 0.0), LEVEL), brick, exact()));

  const DestructionReport departed = advance(world, destruction);

  REQUIRE(departed.created.size() == 1u);

  movable(world, departed.created[0]).velocity = glm::dvec3(1.0, 0.0, 0.0);
  movable(world, departed.created[0]).angularVelocity = glm::dvec3(0.5, 0.0, 1.5);

  uint32_t splits = 0u;

  for (uint32_t step = 0u; step != 300u; ++step) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);

    const Motion before = destructionfixture::motionOf(world);
    const DestructionReport report = destruction.update(world);

    if (!report.created.empty()) {
      ++splits;
      checkKept(before, destructionfixture::motionOf(world));
    }

    const std::string wrong = destructionfixture::mismatch(destruction, world);

    INFO("step " << step << ": " << wrong);
    CHECK(wrong.empty());
  }

  CHECK(splits >= 1u);
}

namespace {

bool allAsleep(const talus::dynamics::World& world) {
  for (const RigidBody& body : world.bodies.rigids) {
    if (body.mass > 0.0 && !body.asleep) {
      return false;
    }
  }

  return true;
}

uint32_t sleepAll(talus::dynamics::World& world, DestructionWorld& destruction, uint32_t limit) {
  uint32_t steps = 0u;

  while (steps != limit && !allAsleep(world)) {
    static_cast<void>(advance(world, destruction));
    ++steps;
  }

  return steps;
}

uint32_t quietWithin(talus::dynamics::World& world, DestructionWorld& destruction, uint32_t limit) {
  for (uint32_t step = 1u; step <= limit; ++step) {
    const DestructionReport report = advance(world, destruction);

    if (report.loads == 0u && report.solved == 0u) {
      return step;
    }
  }

  return 0u;
}

}

TEST_CASE("a pile whose bodies all sleep gathers no loads and solves nothing, and gathers again when a crate lands on it", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, stack(), concreteOnly(), exact());

  REQUIRE(advance(world, destruction).created.size() == 2u);
  REQUIRE(sleepAll(world, destruction, 600u) != 600u);
  REQUIRE(quietWithin(world, destruction, 5u) != 0u);

  for (uint32_t step = 0u; step != 30u; ++step) {
    const DestructionReport resting = advance(world, destruction);

    CHECK(resting.loads == 0u);
    CHECK(resting.solved == 0u);
  }

  const std::optional<uint32_t> top = destruction.bodyOf(built.building, 2u);

  REQUIRE(top.has_value());

  static_cast<void>(addCrate(world, rigidOf(world, *top).position + glm::dvec3(0.0, 0.25 + CRATE_HALF + 0.4, 0.0), LEVEL));

  bool gathered = false;
  bool solved = false;

  for (uint32_t step = 0u; step != 60u; ++step) {
    const DestructionReport report = advance(world, destruction);

    gathered = gathered || report.loads != 0u;
    solved = solved || report.solved != 0u;
  }

  CHECK(gathered);
  CHECK(solved);
}

TEST_CASE("a building whose debris sleeps on it gathers no loads while the debris sleeps, and gathers again when more debris lands on it", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, roof(glm::dvec3(0.0), LEVEL), concreteOnly(), exact());
  const glm::dvec3 seat = destruction.chunked(built.building).chunks[5].centre + glm::dvec3(0.0, 0.25 + CRATE_HALF, 0.0);

  static_cast<void>(addCrate(world, seat, LEVEL));

  REQUIRE(sleepAll(world, destruction, 600u) != 600u);
  REQUIRE(quietWithin(world, destruction, 5u) != 0u);

  const std::optional<ChunkLoad> load = loadOn(destruction, built.building, 5u);

  REQUIRE(load.has_value());
  CHECK(load->force.y < 0.0);

  const glm::dvec3 drop = destruction.chunked(built.building).chunks[4].centre + glm::dvec3(0.0, 0.25 + CRATE_HALF + 0.4, 0.0);

  static_cast<void>(addCrate(world, drop, LEVEL));

  bool gathered = false;
  bool solved = false;

  for (uint32_t step = 0u; step != 60u; ++step) {
    const DestructionReport report = advance(world, destruction);

    gathered = gathered || report.loads != 0u;
    solved = solved || report.solved != 0u;
  }

  CHECK(gathered);
  CHECK(solved);
}

TEST_CASE("a crate held by friction on a tilted roof loads its roof chunk with its whole weight, turned into the building's frame", "[destruction][destructionworld]") {
  for (const double tilt : {0.2, -0.2}) {
    for (const double yaw : {0.0, 0.5}) {
      INFO("tilted by " << tilt << ", turned by " << yaw);

      talus::dynamics::World world;
      DestructionWorld destruction(exactLoads());
      const glm::dvec3 place(1.0, 0.5, -2.0);
      const glm::dquat turn = aboutY(yaw) * glm::angleAxis(tilt, glm::dvec3(0.0, 0.0, 1.0));
      const BuildingReport built = destruction.addBuilding(world, roof(place, turn), concreteOnly(), exact());
      const glm::dvec3 offset(0.1, 0.25 + CRATE_HALF, 0.05);
      const glm::dvec3 seat = place + turn * (destruction.chunked(built.building).chunks[5].centre + offset);
      const uint32_t crate = addCrate(world, seat, turn);
      const double weight = rigidOf(world, crate).mass * G;

      for (uint32_t step = 0u; step != 240u; ++step) {
        static_cast<void>(advance(world, destruction));
      }

      const glm::dvec3 expected = glm::conjugate(turn) * glm::dvec3(0.0, -weight, 0.0);
      const std::optional<ChunkLoad> load = loadOn(destruction, built.building, 5u);

      CHECK(glm::length(rigidOf(world, crate).position - seat) <= 0.01);
      REQUIRE(load.has_value());
      CHECK(glm::length(load->force - expected) <= 0.01 * weight);
      CHECK(glm::length(load->torque - glm::cross(offset, expected)) <= 0.001 * weight);
    }
  }
}

TEST_CASE("an L-shaped piece at rest on the ground, its body turned from its building's frame, carries its weight in the building's frame", "[destruction][destructionworld]") {
  const std::array<std::pair<double, glm::dvec3>, 2> poses{
    std::pair(0.0, glm::dvec3(0.0, -0.95, 0.0)),
    std::pair(PI / 2.0, glm::dvec3(0.0, 0.05, 0.0))
  };

  for (const auto& [roll, position] : poses) {
    INFO("rolled by " << roll);

    talus::dynamics::World world;
    DestructionWorld destruction(exactLoads());
    Building shape;

    static_cast<void>(addGround(world, destruction));
    shape.members.push_back(Member{glm::dvec3(0.5, 1.25, 0.0), glm::dvec3(0.5, 0.25, 0.25), 0u, Role::Beam});
    shape.members.push_back(Member{glm::dvec3(0.75, 1.75, 0.0), glm::dvec3(0.25, 0.25, 0.25), 0u, Role::Column});
    shape.position = position;
    shape.rotation = glm::angleAxis(roll, glm::dvec3(0.0, 0.0, 1.0));

    const BuildingReport built = destruction.addBuilding(world, shape, concreteOnly(), exact());
    const ChunkedBuilding& chunked = destruction.chunked(built.building);

    for (uint32_t step = 0u; step != 360u; ++step) {
      static_cast<void>(advance(world, destruction));
    }

    const std::optional<uint32_t> held = destruction.bodyOf(built.building, 0u);

    REQUIRE(held.has_value());

    const RigidBody& piece = rigidOf(world, *held);
    const double weight = piece.mass * G;
    const glm::dvec3 up(0.0, weight, 0.0);
    const glm::dvec3 support = glm::conjugate(piece.rotation * piece.compound->children.front().rotation) * up;

    REQUIRE(glm::length(glm::conjugate(piece.rotation) * up - support) >= 0.1 * weight);

    glm::dvec3 centre(0.0);
    glm::dvec3 force(0.0);
    glm::dvec3 moment(0.0);

    for (uint32_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
      REQUIRE(destruction.bodyOf(built.building, chunk) == held);
      centre += (chunked.chunks[chunk].mass / piece.mass) * chunked.chunks[chunk].centre;
    }

    for (const ChunkLoad& load : destruction.chunkLoads(built.building)) {
      force += load.force;
      moment += load.torque + glm::cross(chunked.chunks[load.chunk].centre - centre, load.force);
    }

    CHECK(glm::length(force - support) <= 0.01 * weight);
    CHECK(glm::length(moment) <= 0.005 * weight);
  }
}

TEST_CASE("a group that leaves its building keeps the cracks of the bonds it takes", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  Building pier;

  pier.members.push_back(Member{glm::dvec3(0.0, 0.75, 0.0), glm::dvec3(0.25, 0.75, 0.25), 0u, Role::Column});

  const BuildingReport built = destruction.addBuilding(world, pier, brick, exact());
  const uint32_t upper = bondJoining(destruction.chunked(built.building), 1u, 2u);
  const uint32_t block = addBlock(world, glm::dvec3(PIER_OFFSET, 1.5 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY);

  static_cast<void>(advance(world, destruction));
  REQUIRE(talus::dynamics::removeBody(world, block));
  REQUIRE(upper != NO_BOND);
  REQUIRE(destruction.bondState(built.building, upper) == BondState::Cracked);

  const std::array<uint32_t, 1> base{0u};

  REQUIRE(destruction.removeChunks(world, built.building, base) == 1u);

  const DestructionReport split = destruction.update(world);

  REQUIRE(split.created.size() == 1u);
  CHECK(destruction.bodyOf(built.building, 1u) == split.created[0]);
  CHECK(destruction.bodyOf(built.building, 2u) == split.created[0]);
  CHECK(destruction.bondState(built.building, upper) == BondState::Cracked);
}

TEST_CASE("a group that leaves its building keeps the worn health of the bonds it takes", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const std::vector<StructuralMaterial> reinforced{talus::destruction::reinforcedConcrete()};
  Building building;

  building.members.push_back(Member{glm::dvec3(0.0, 0.5, 0.0), glm::dvec3(0.25, 0.5, 0.25), 0u, Role::Column});
  building.members.push_back(Member{glm::dvec3(2.25, 0.865, 0.0), glm::dvec3(2.0, 0.135, 0.25), 0u, Role::Slab});
  building.members.push_back(Member{glm::dvec3(2.25, 0.365, 0.0), glm::dvec3(2.0, 0.365, 0.25), 0u, Role::Wall});

  const BuildingReport built = destruction.addBuilding(world, building, reinforced, exact());
  const ChunkedBuilding& chunked = destruction.chunked(built.building);
  const uint32_t root = bondJoining(chunked, 1u, 2u);
  std::vector<uint32_t> wall;

  for (uint32_t chunk = 0u; chunk != chunked.chunks.size(); ++chunk) {
    if (building.members[chunked.chunks[chunk].member].role == Role::Wall) {
      wall.push_back(chunk);
    }
  }

  REQUIRE(built.broken.empty());
  REQUIRE(destruction.removeChunks(world, built.building, wall) == wall.size());
  static_cast<void>(destruction.update(world));

  const double worn = destruction.bondHealth(built.building, root);

  REQUIRE(root != NO_BOND);
  REQUIRE(destruction.bondState(built.building, root) == BondState::Intact);
  REQUIRE(worn > 0.0);
  REQUIRE(worn < 1.0);

  const std::array<uint32_t, 1> base{0u};

  REQUIRE(destruction.removeChunks(world, built.building, base) == 1u);

  const DestructionReport split = destruction.update(world);

  REQUIRE(split.created.size() == 1u);
  CHECK(destruction.bodyOf(built.building, 1u) == split.created[0]);
  CHECK(destruction.bodyOf(built.building, 2u) == split.created[0]);
  CHECK(destruction.bondHealth(built.building, root) == worn);
}

namespace {

constexpr uint32_t ORDER_STEPS = 6u;

struct StaggeredHits {
  uint32_t columnBroken = 0u;
  uint32_t pierCracked = 0u;
  uint32_t deferredSteps = 0u;
  uint32_t lastDeferred = 0u;
  uint32_t firstSolved = 0u;
  uint32_t firstRounds = 0u;
  uint32_t firstDeferred = 0u;
};

StaggeredHits hitColumnThenPier(bool pierToo, bool large = true) {
  DestructionSettings settings = exceeded();

  settings.ceiling = 0.0;
  settings.wideChunks = large ? 1u : DestructionSettings{}.wideChunks;

  talus::dynamics::World world;
  DestructionWorld destruction(settings);
  const std::vector<StructuralMaterial> brick{wholeBrick()};
  const BuildingReport post = destruction.addBuilding(world, column(glm::dvec3(3.0, 0.0, 0.0)), concreteOnly(), exact());
  const BuildingReport pier = destruction.addBuilding(world, brickPier(glm::dvec3(-3.0, 0.0, 0.0)), brick, exact());
  StaggeredHits outcome;

  for (uint32_t step = 1u; step <= ORDER_STEPS; ++step) {
    std::vector<uint32_t> blocks;

    if (step == 1u) {
      blocks.push_back(
        addBlock(world, glm::dvec3(3.0 - 0.25 - CRATE_HALF - GAP, 2.75, 0.0), glm::dvec3(POST_SPEED, 0.0, 0.0), CRATE_KEY + 1u)
      );

      if (pierToo) {
        blocks.push_back(
          addBlock(world, glm::dvec3(-3.0 + PIER_OFFSET, 1.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0, -PIER_SPEED, 0.0), CRATE_KEY)
        );
      }
    }

    const DestructionReport report = advance(world, destruction);

    if (step == 1u) {
      outcome.firstSolved = report.solved;
      outcome.firstRounds = report.rounds;
      outcome.firstDeferred = report.deferred;
    }

    for (const uint32_t block : blocks) {
      REQUIRE(talus::dynamics::removeBody(world, block));
    }

    for (const BrokenBond& bond : report.broken) {
      if (bond.building == post.building && outcome.columnBroken == 0u) {
        outcome.columnBroken = step;
      }
    }

    if (outcome.pierCracked == 0u && destruction.bondState(pier.building, 0u) == BondState::Cracked) {
      outcome.pierCracked = step;
    }

    if (report.deferred != 0u) {
      outcome.deferredSteps += 1u;
      outcome.lastDeferred = step;
    }
  }

  return outcome;
}

}

TEST_CASE("above the collapse ceiling only the oldest pending update of a large piece gets its round: a column and a pier counted as large, hit in the same step, the column's update older, the column keeps its timing while the pier's waits", "[destruction][destructionworld]") {
  const StaggeredHits alone = hitColumnThenPier(false);
  const StaggeredHits both = hitColumnThenPier(true);

  INFO("column alone breaks in step " << alone.columnBroken << "; with the pier hit in the same step it breaks in step "
       << both.columnBroken << " and the pier cracks in step " << both.pierCracked << "; pending updates waited in "
       << both.deferredSteps << " steps, the last " << both.lastDeferred);

  REQUIRE(alone.columnBroken >= 2u);
  REQUIRE(alone.pierCracked == 0u);
  REQUIRE(both.pierCracked != 0u);
  CHECK(alone.deferredSteps == 0u);
  CHECK(both.columnBroken == alone.columnBroken);
  CHECK(both.deferredSteps != 0u);
  CHECK(both.pierCracked > both.columnBroken);
}

TEST_CASE("above the collapse ceiling a small piece's pending update is not deferred: its rounds cost too little to matter to the frame", "[destruction][destructionworld]") {
  const StaggeredHits small = hitColumnThenPier(true, false);
  const StaggeredHits large = hitColumnThenPier(true, true);

  REQUIRE(large.deferredSteps != 0u);
  CHECK(small.deferredSteps == 0u);
  CHECK(small.columnBroken == large.columnBroken);
}

TEST_CASE("above the ceiling two newly hit large pieces admit only the oldest first round and retain the younger hit until it can run", "[destruction][destructionworld][budgetcompletion]") {
  const StaggeredHits both = hitColumnThenPier(true);

  CHECK(both.firstSolved == 1u);
  CHECK(both.firstRounds == 1u);
  CHECK(both.firstDeferred == 1u);
  REQUIRE(both.columnBroken > 0u);
  CHECK(both.pierCracked > both.columnBroken);
}

TEST_CASE("a one-chunk piece resting under contact loads runs no stress update and takes no round, and its loads are still gathered", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction(exceeded());

  static_cast<void>(addGround(world, destruction));

  const BuildingReport built = destruction.addBuilding(world, bar(1u, glm::dvec3(0.0, -0.9, 0.0), LEVEL), concreteOnly(), exact());
  uint32_t gathered = 0u;
  uint32_t solved = 0u;
  uint32_t rounds = 0u;

  for (uint32_t step = 0u; step != 240u; ++step) {
    const DestructionReport report = advance(world, destruction);

    gathered += report.loads;
    solved += report.solved;
    rounds += report.rounds;
  }

  const double weight = CONCRETE_DENSITY * CUBE * G;
  const std::optional<ChunkLoad> load = loadOn(destruction, built.building, 0u);

  CHECK(gathered != 0u);
  CHECK(solved == 0u);
  CHECK(rounds == 0u);
  REQUIRE(load.has_value());
  CHECK(near(load->force.y, weight, 0.01 * weight));
  CHECK(glm::length(glm::dvec3(load->force.x, 0.0, load->force.z)) <= 0.01 * weight);
}

TEST_CASE("a building whose chunks are all anchored reads none of its contacts while awake bodies touch it, and a moving piece on it reads its own", "[destruction][destructionworld]") {
  talus::dynamics::World world;
  DestructionWorld destruction;
  const uint32_t floor = addGround(world, destruction);
  const uint32_t crate = addCrate(world, glm::dvec3(0.0, CRATE_HALF + GAP, 0.0), LEVEL);
  uint32_t touched = 0u;
  uint32_t gathered = 0u;

  for (uint32_t step = 0u; step != 30u; ++step) {
    const DestructionReport report = advance(world, destruction);

    touched += !rigidOf(world, crate).asleep && !talus::dynamics::contactsOf(world, *talus::dynamics::findBody(world.bodies, floor)).empty() ? 1u : 0u;
    gathered += report.gathered;
  }

  REQUIRE(touched != 0u);
  CHECK(gathered == 0u);
  CHECK(destruction.chunkLoads(0u).empty());

  const BuildingReport built = destruction.addBuilding(world, bar(1u, glm::dvec3(3.0, -0.9, 0.0), LEVEL), concreteOnly(), exact());
  uint32_t read = 0u;

  for (uint32_t step = 0u; step != 30u; ++step) {
    read += advance(world, destruction).gathered;
  }

  CHECK(read != 0u);
  CHECK(loadOn(destruction, built.building, 0u).has_value());
}

namespace {

struct Twin {
  talus::dynamics::World world;
  DestructionWorld destruction;
  BuildingReport built;
};

std::unique_ptr<Twin> visitTwin(bool visitAll, uint32_t workers = 1u, uint32_t wideChunks = DestructionSettings{}.wideChunks) {
  auto twin = std::make_unique<Twin>();
  DestructionSettings settings;

  settings.visitAll = visitAll;
  settings.wideChunks = wideChunks;
  twin->world.workerCount = workers;
  twin->destruction.setSettings(settings);
  static_cast<void>(addGround(twin->world, twin->destruction));
  twin->built = twin->destruction.addBuilding(twin->world, twoColumns(), concreteOnly(), exact());
  twin->destruction.removeAt(5u, twin->built.building, std::vector<uint32_t>{0u});
  twin->destruction.blastAt(talus::destruction::Blast{glm::dvec3(4.0, 1.25, 0.0), 30000.0, 3.0, 250u});

  return twin;
}

void visitEvents(Twin& twin, uint64_t step) {
  const uint32_t building = twin.built.building;

  if (step == 150u) {
    const RigidBody& fallen = rigidOf(twin.world, *twin.destruction.bodyOf(building, 3u));

    static_cast<void>(addCrate(twin.world, fallen.position + glm::dvec3(0.0, 2.0, 0.0), LEVEL));
  }

  if (step == 200u) {
    static_cast<void>(addBlock(twin.world, glm::dvec3(3.0, 3.0 + CRATE_HALF + GAP, 0.0), glm::dvec3(0.0), CRATE_KEY + 1u));
  }

  if (step == 300u) {
    const std::array<uint32_t, 1> middle{3u};

    static_cast<void>(twin.destruction.removeChunks(twin.world, building, middle));
  }

  if (step == 330u) {
    const std::array<uint32_t, 1> shaft{8u};

    static_cast<void>(twin.destruction.fractureChunks(twin.world, building, shaft));
  }

  if (step == 360u) {
    twin.world.parameters.gravity = glm::dvec3(0.0, -9.0, 0.0);
  }

  if (step == 380u) {
    const RigidBody& standing = rigidOf(twin.world, twin.built.body);

    REQUIRE(talus::dynamics::setRigidPose(
      twin.world,
      standing.id,
      standing.position,
      glm::normalize(standing.rotation * glm::angleAxis(1.0e-6, glm::dvec3(0.0, 0.0, 1.0)))
    ));
  }

  if (step == 400u) {
    const RigidBody& lying = rigidOf(twin.world, *twin.destruction.bodyOf(building, 1u));

    REQUIRE(talus::dynamics::setRigidPose(twin.world, lying.id, lying.position + glm::dvec3(0.0, 0.01, 0.0), lying.rotation));
  }
}

DestructionReport visitStep(Twin& twin) {
  twin.destruction.prepare(twin.world);
  REQUIRE(!talus::dynamics::step(twin.world).nonFinite);

  return twin.destruction.update(twin.world);
}

}

TEST_CASE("visiting only the pieces near motion steps exactly as visiting every piece, through sleep, wake-ups, a blast, edits, a gravity change and a turned building", "[destruction][destructionworld]") {
  const std::unique_ptr<Twin> every = visitTwin(true);
  const std::unique_ptr<Twin> near = visitTwin(false);
  uint32_t fewer = 0u;
  uint32_t blasted = 0u;
  uint32_t asleepSteps = 0u;

  for (uint64_t step = 1u; step <= 450u; ++step) {
    visitEvents(*every, step);
    visitEvents(*near, step);

    const DestructionReport all = visitStep(*every);
    const DestructionReport some = visitStep(*near);

    INFO("step " << step);
    REQUIRE(destructionfixture::sameOutcome(all, some));
    REQUIRE(destructionfixture::stateHash(every->destruction, every->world)
            == destructionfixture::stateHash(near->destruction, near->world));
    CHECK(some.visited <= all.visited);
    fewer += some.visited < all.visited ? 1u : 0u;
    blasted += some.blasted;
    asleepSteps += rigidOf(near->world, *near->destruction.bodyOf(near->built.building, 1u)).asleep ? 1u : 0u;
  }

  CHECK(fewer > 100u);
  CHECK(blasted > 0u);
  CHECK(asleepSteps > 50u);
  CHECK(destructionfixture::mismatch(near->destruction, near->world).empty());
}

TEST_CASE("the destruction update steps the same on 1 and 8 workers, report for report and state for state", "[destruction][destructionworld]") {
  const std::unique_ptr<Twin> serial = visitTwin(false, 1u);
  const std::unique_ptr<Twin> parallel = visitTwin(false, 8u);
  uint32_t solved = 0u;

  for (uint64_t step = 1u; step <= 450u; ++step) {
    visitEvents(*serial, step);
    visitEvents(*parallel, step);

    const DestructionReport one = visitStep(*serial);
    const DestructionReport eight = visitStep(*parallel);

    INFO("step " << step);
    REQUIRE(destructionfixture::sameOutcome(one, eight));
    REQUIRE(one.visited == eight.visited);
    REQUIRE(destructionfixture::stateHash(serial->destruction, serial->world)
            == destructionfixture::stateHash(parallel->destruction, parallel->world));
    solved += eight.solved;
  }

  CHECK(solved > 0u);
}

TEST_CASE("pieces of many chunks factorize after the parallel loop, each with all the workers, and the world steps exactly as when every piece factorizes inside the loop", "[destruction][destructionworld]") {
  const std::unique_ptr<Twin> inside = visitTwin(false, 8u, 0u);
  const std::unique_ptr<Twin> after = visitTwin(false, 8u, 2u);
  uint32_t wide = 0u;

  for (uint64_t step = 1u; step <= 450u; ++step) {
    visitEvents(*inside, step);
    visitEvents(*after, step);

    const DestructionReport looped = visitStep(*inside);
    const DestructionReport widened = visitStep(*after);

    INFO("step " << step);
    REQUIRE(destructionfixture::sameOutcome(looped, widened));
    REQUIRE(destructionfixture::stateHash(inside->destruction, inside->world)
            == destructionfixture::stateHash(after->destruction, after->world));
    REQUIRE(looped.wide == 0u);
    wide += widened.wide;
  }

  CHECK(wide > 0u);
}
