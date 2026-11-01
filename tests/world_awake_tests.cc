#include <catch2/catch_test_macros.hpp>

#include "destructionfixtures.h"
#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/destruction/blast/blast.h>
#include <talus/destruction/building/building.h>
#include <talus/destruction/chunker/chunker.h>
#include <talus/destruction/destructionworld/destructionworld.h>
#include <talus/destruction/materials/materials.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/compound/compound.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/joints/jointedits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/tiers/tiers.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>
#include <talus/mesh.h>
#include <talus/world.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace {

using talus::destruction::Building;
using talus::destruction::ChunkerSettings;
using talus::destruction::DestructionReport;
using talus::destruction::DestructionSettings;
using talus::destruction::Member;
using talus::destruction::Role;
using talus::destruction::StructuralMaterial;
using talus::dynamics::RigidBody;

constexpr double crateHalf = 0.1;
constexpr double crateDensity = 1250.0;
constexpr double blockDensity = 125000.0;
constexpr uint32_t crateKey = 1000000u;
constexpr uint32_t crateRows = 25u;
constexpr size_t serialLimit = 512u;
constexpr uint64_t twinSteps = 450u;
const glm::dquat level(1.0, 0.0, 0.0, 0.0);

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

const talus::field::SampledSdf& blockField() {
  static const talus::field::SampledSdf field =
    boxField(glm::dvec3(0.05, 0.025, 0.05), 0.025);

  return field;
}

const std::vector<talus::PhysicalMaterial>& rubber() {
  static const std::vector<talus::PhysicalMaterial> lanes = [] {
    talus::PhysicalMaterial material;

    material.density = 1000.0f;
    material.youngsModulus = 1.2854547e6f;
    material.poissonsRatio = 0.45f;

    return std::vector<talus::PhysicalMaterial>{material};
  }();

  return lanes;
}

const std::shared_ptr<const talus::PreparedMesh>& blockMesh() {
  static const std::shared_ptr<const talus::PreparedMesh> mesh =
    talus::deformable::buildMesh(
      blockField(),
      rubber(),
      glm::mat4(1.0f),
      std::span<const talus::deformable::SupportVolume>{},
      talus::VolumeElementType::Hex
    );

  return mesh;
}

StructuralMaterial soil() {
  StructuralMaterial material = talus::destruction::concrete();

  material.chunkSize = 64.0;
  material.fracture = talus::destruction::Fracture::None;
  material.pieces = 1u;

  return material;
}

Building ground() {
  Building building;

  building.members.push_back(
    Member{
      glm::dvec3(0.0, -0.5, 0.0),
      glm::dvec3(20.0, 0.5, 20.0),
      0u,
      Role::Foundation
    }
  );

  return building;
}

Building portal() {
  Building building;

  building.members.push_back(
    Member{
      glm::dvec3(0.0, 1.5, 0.0),
      glm::dvec3(0.25, 1.5, 0.25),
      0u,
      Role::Column
    }
  );
  building.members.push_back(
    Member{
      glm::dvec3(3.0, 1.5, 0.0),
      glm::dvec3(0.25, 1.5, 0.25),
      0u,
      Role::Column
    }
  );
  building.members.push_back(
    Member{
      glm::dvec3(1.5, 3.25, 0.0),
      glm::dvec3(1.75, 0.25, 0.25),
      0u,
      Role::Beam
    }
  );

  return building;
}

Building plinth() {
  Building building;

  building.members.push_back(
    Member{
      glm::dvec3(-4.0, 0.25, 3.0),
      glm::dvec3(1.0, 0.25, 0.25),
      0u,
      Role::Foundation
    }
  );

  return building;
}

Building looseBar() {
  Building building;

  building.members.push_back(
    Member{
      glm::dvec3(-2.0, 0.27, -1.5),
      glm::dvec3(1.0, 0.25, 0.25),
      0u,
      Role::Beam
    }
  );

  return building;
}

uint32_t addCrate(
  talus::dynamics::World& world,
  const glm::dvec3& centre,
  double density = crateDensity
) {
  const std::array<talus::dynamics::CompoundChild, 1> crate{
    talus::dynamics::CompoundChild{
      glm::dvec3(0.0),
      level,
      glm::dvec3(crateHalf),
      density,
      crateKey
    }
  };
  const uint32_t index = talus::dynamics::addCompoundBody(
    world.bodies,
    crate,
    centre,
    level,
    glm::dvec3(0.0),
    0.6,
    0.0,
    false
  );

  return world.bodies.rigids[index].id;
}

const RigidBody* rigidOf(const talus::dynamics::World& world, uint32_t id) {
  const std::optional<talus::dynamics::BodyRef> ref =
    talus::dynamics::findBody(world.bodies, id);

  if (!ref || ref->side != talus::dynamics::Side::Rigid) {
    return nullptr;
  }

  return &world.bodies.rigids[ref->index];
}

struct Perch {
  uint32_t crate = 0u;
  uint32_t chunk = 0u;
};

struct Yard {
  talus::World world;
  std::vector<uint32_t> buildings;
  std::vector<uint32_t> crates;
  uint32_t plinth = 0u;
  std::vector<Perch> perches;
  uint32_t block = 0u;
  uint32_t joint = 0u;
  uint32_t sleepingCuts = 0u;
  uint32_t poseWakes = 0u;
};

std::unique_ptr<Yard> makeYard(uint32_t workers, uint32_t width) {
  auto yard = std::make_unique<Yard>();
  talus::World& world = yard->world;
  DestructionSettings settings;
  const ChunkerSettings exact{false};

  settings.target = 0.05;
  settings.removalFloor = 0.0;
  world.destruction.setSettings(settings);
  world.dynamics.workerCount = workers;
  world.dynamics.simdWidth = width;

  const std::vector<StructuralMaterial> soilOnly{soil()};
  const std::vector<StructuralMaterial> concrete{
    talus::destruction::concrete()
  };

  static_cast<void>(
    world.destruction.addBuilding(world.dynamics, ground(), soilOnly)
  );

  for (const Building& building : {portal(), looseBar()}) {
    yard->buildings.push_back(
      world.destruction.addBuilding(world.dynamics, building, concrete, exact)
        .building
    );
  }

  yard->plinth =
    world.destruction.addBuilding(world.dynamics, plinth(), concrete, exact)
      .building;

  const talus::destruction::ChunkedBuilding& stones =
    world.destruction.chunked(yard->plinth);

  for (const double x : {-4.75, -3.25}) {
    const auto chunk = static_cast<uint32_t>(
      std::find_if(
        stones.chunks.begin(),
        stones.chunks.end(),
        [x](const talus::destruction::Chunk& stone) {
          return std::abs(stone.centre.x - x) < 0.01;
        }
      )
      - stones.chunks.begin()
    );

    REQUIRE(chunk != stones.chunks.size());
    yard->perches.push_back(
      Perch{
        addCrate(world.dynamics, glm::dvec3(x, 0.5 + crateHalf + 0.001, 3.0)),
        chunk
      }
    );
  }

  world.destruction.removeAt(
    5u,
    yard->buildings.front(),
    std::vector<uint32_t>{0u}
  );
  world.destruction.blastAt(
    talus::destruction::Blast{glm::dvec3(3.0, 1.0, 0.0), 30000.0, 3.0, 40u}
  );

  for (uint32_t row = 0u; row != crateRows; ++row) {
    for (uint32_t column = 0u; column != crateRows; ++column) {
      yard->crates.push_back(addCrate(
        world.dynamics,
        glm::dvec3(
          -1.0 + 0.3 * static_cast<double>(column),
          crateHalf + 0.001,
          0.8 + 0.3 * static_cast<double>(row)
        )
      ));
    }
  }

  static_cast<void>(
    addCrate(world.dynamics, glm::dvec3(-1.0, 3.0 * crateHalf + 0.002, 0.8))
  );

  talus::dynamics::JointDef pair;

  pair.bodyA = yard->crates[1];
  pair.bodyB = yard->crates[2];
  pair.frameA.position = glm::dvec3(0.15, 0.0, 0.0);
  pair.frameB.position = glm::dvec3(-0.15, 0.0, 0.0);
  yard->joint = talus::dynamics::addJoint(world.dynamics, pair);
  REQUIRE(yard->joint != 0u);

  yard->block = talus::dynamics::addSwitchable(
    world.dynamics.bodies,
    blockMesh(),
    rubber(),
    blockField(),
    glm::dvec3(-3.0, 0.026, 0.0),
    glm::dvec3(0.0),
    0.5,
    0.0
  );

  return yard;
}

struct ChunkAt {
  uint32_t building = 0u;
  uint32_t chunk = 0u;
};

const RigidBody* pieceOf(const Yard& yard, const ChunkAt& at) {
  const std::optional<uint32_t> id =
    yard.world.destruction.bodyOf(at.building, at.chunk);

  return id ? rigidOf(yard.world.dynamics, *id) : nullptr;
}

template<typename Wanted>
std::optional<ChunkAt> findChunk(const Yard& yard, Wanted wanted) {
  for (const uint32_t building : yard.buildings) {
    const uint32_t chunks = yard.world.destruction.chunkCount(building);

    for (uint32_t chunk = 0u; chunk != chunks; ++chunk) {
      const ChunkAt at{building, chunk};
      const RigidBody* body = pieceOf(yard, at);

      if (body != nullptr && wanted(*body)) {
        return at;
      }
    }
  }

  return std::nullopt;
}

std::optional<ChunkAt> sleepingChunk(const Yard& yard, size_t children) {
  return findChunk(yard, [children](const RigidBody& body) {
    return body.asleep && body.mass > 0.0 && body.compound != nullptr
           && body.compound->children.size() >= children;
  });
}

std::optional<ChunkAt> frozenChunk(const Yard& yard) {
  return findChunk(yard, [](const RigidBody& body) {
    return talus::dynamics::isFrozen(body);
  });
}

uint64_t perchStep(size_t perch) {
  return 280u + 50u * perch;
}

void edit(Yard& yard, uint64_t step) {
  talus::World& world = yard.world;
  talus::dynamics::World& dynamics = world.dynamics;

  if (step == 120u) {
    for (const uint32_t index : {1u, 37u, 64u}) {
      const RigidBody* crate = rigidOf(dynamics, yard.crates[index]);

      REQUIRE(crate != nullptr);
      REQUIRE(
        talus::dynamics::applyImpulse(
          dynamics,
          crate->id,
          glm::dvec3(0.0, 40.0, 15.0),
          crate->position + glm::dvec3(0.05, 0.0, 0.0)
        )
      );
    }
  }

  if (step == 150u) {
    const RigidBody* below = rigidOf(dynamics, yard.crates[44]);

    REQUIRE(below != nullptr);
    static_cast<void>(
      addCrate(dynamics, below->position + glm::dvec3(0.02, 0.6, 0.0))
    );
  }

  if (step == 170u) {
    REQUIRE(talus::dynamics::promote(dynamics, yard.block));
    REQUIRE(
      talus::dynamics::findBody(dynamics.bodies, yard.block)->side
      == talus::dynamics::Side::Lattice
    );
  }

  if (step == 200u) {
    REQUIRE(
      talus::dynamics::removeBodies(
        dynamics,
        std::vector<uint32_t>{yard.crates[50], yard.crates[51], yard.crates[73]}
      )
      == 3u
    );
  }

  if (step == 230u) {
    talus::dynamics::demote(dynamics, yard.block);
    REQUIRE(
      talus::dynamics::findBody(dynamics.bodies, yard.block)->side
      == talus::dynamics::Side::Rigid
    );
  }

  if (step == 240u) {
    for (const uint32_t id : yard.crates) {
      const RigidBody* crate = rigidOf(dynamics, id);

      if (crate != nullptr) {
        REQUIRE(
          talus::dynamics::applyImpulse(
            dynamics,
            id,
            glm::dvec3(0.0, 2.0, 0.0),
            crate->position
          )
        );
      }
    }
  }

  for (size_t perch = 0u; perch != yard.perches.size(); ++perch) {
    if (step == perchStep(perch)) {
      world.destruction.removeAt(
        world.destruction.steps() + 1u,
        yard.plinth,
        std::vector<uint32_t>{yard.perches[perch].chunk}
      );
    }
  }

  if (step == 260u || step == 300u || step == 340u || step == 380u) {
    const std::optional<ChunkAt> frozen = frozenChunk(yard);

    if (frozen) {
      static_cast<void>(addCrate(
        dynamics,
        pieceOf(yard, *frozen)->position + glm::dvec3(0.0, 1.5, 0.0),
        blockDensity
      ));
    }
  }

  if (step == 300u) {
    REQUIRE(talus::dynamics::removeJoint(dynamics, yard.joint));
  }

  if (step == 320u) {
    dynamics.parameters.gravity = glm::dvec3(0.0, -9.5, 0.0);
  }

  if (step == 360u || step == 420u) {
    const std::optional<ChunkAt> sleeping = sleepingChunk(yard, 2u);

    if (sleeping) {
      world.destruction.removeAt(
        world.destruction.steps() + 1u,
        sleeping->building,
        std::vector<uint32_t>{sleeping->chunk}
      );
      ++yard.sleepingCuts;
    }
  }

  if (step == 400u) {
    const std::optional<ChunkAt> sleeping = sleepingChunk(yard, 1u);

    if (sleeping) {
      const RigidBody& body = *pieceOf(yard, *sleeping);

      ++yard.poseWakes;

      REQUIRE(
        talus::dynamics::setRigidPose(
          dynamics,
          body.id,
          body.position + glm::dvec3(0.0, 0.01, 0.0),
          body.rotation
        )
      );
    }
  }
}

bool extraDynamicsStep(uint64_t step) {
  return step >= 100u && step <= 430u && step % 7u == 0u;
}

talus::StepReport fullScanStep(talus::World& world) {
  world.destruction.prepare(world.dynamics);
  talus::dynamics::StepReport motion = talus::dynamics::step(world.dynamics);

  world.dynamics.bodies.layout += 1u;

  DestructionReport damage = world.destruction.update(world.dynamics);

  return {std::move(motion), std::move(damage)};
}

bool sameMotion(
  const talus::dynamics::StepReport& a,
  const talus::dynamics::StepReport& b
) {
  return a.nonFinite == b.nonFinite && a.contacts == b.contacts
         && a.activeContacts == b.activeContacts && a.thawed == b.thawed
         && a.promotions == b.promotions && a.demotions == b.demotions
         && a.hits.size() == b.hits.size();
}

bool sameRest(const talus::World& a, const talus::World& b) {
  const uint32_t ids =
    std::max(a.dynamics.bodies.nextId, b.dynamics.bodies.nextId);

  for (uint32_t id = 0u; id != ids; ++id) {
    if (a.destruction.rest().restSteps(id)
        != b.destruction.rest().restSteps(id)) {
      return false;
    }
  }

  return true;
}

struct TwinRun {
  uint64_t digest = 0u;
  uint32_t sparseReady = 0u;
  uint32_t wideAwake = 0u;
  uint32_t jointWakes = 0u;
  uint32_t contactWakes = 0u;
  uint32_t updateWakes = 0u;
  uint32_t poseWakes = 0u;
  uint32_t frozen = 0u;
  uint32_t thawed = 0u;
  uint32_t removedFrozen = 0u;
  uint32_t created = 0u;
  uint32_t sleepingCuts = 0u;
};

TwinRun runTwins(uint32_t workers, uint32_t width) {
  const std::unique_ptr<Yard> full = makeYard(workers, width);
  const std::unique_ptr<Yard> stepped = makeYard(workers, width);
  TwinRun run;
  const auto asleep = [&stepped](uint32_t id) {
    const RigidBody* body = rigidOf(stepped->world.dynamics, id);

    return body != nullptr && body->asleep;
  };
  const uint32_t jointed = stepped->crates[2];
  const uint32_t struck = stepped->crates[44];
  bool waitingForContact = false;

  REQUIRE(stepped->world.dynamics.bodies.rigids.size() > serialLimit);

  for (uint64_t step = 1u; step <= twinSteps; ++step) {
    const bool jointedAsleep = step == 120u && asleep(jointed);
    std::vector<uint32_t> perchedAsleep;

    for (size_t perch = 0u; perch != stepped->perches.size(); ++perch) {
      if (step == perchStep(perch) && asleep(stepped->perches[perch].crate)) {
        perchedAsleep.push_back(stepped->perches[perch].crate);
      }
    }

    waitingForContact = waitingForContact || (step == 150u && asleep(struck));
    edit(*full, step);
    edit(*stepped, step);

    if (extraDynamicsStep(step)) {
      REQUIRE(!talus::dynamics::step(full->world.dynamics).nonFinite);
      REQUIRE(!talus::dynamics::step(stepped->world.dynamics).nonFinite);
    }

    const talus::StepReport all = fullScanStep(full->world);
    const talus::StepReport some = stepped->world.step();
    const talus::dynamics::World& dynamics = stepped->world.dynamics;

    INFO("workers " << workers << " width " << width << " step " << step);
    REQUIRE(!some.dynamics.nonFinite);
    REQUIRE(sameMotion(all.dynamics, some.dynamics));
    REQUIRE(destructionfixture::sameReport(all.destruction, some.destruction));
    REQUIRE(
      destructionfixture::stateHash(
        full->world.destruction,
        full->world.dynamics
      )
      == destructionfixture::stateHash(stepped->world.destruction, dynamics)
    );
    REQUIRE(sameRest(full->world, stepped->world));

    run.digest =
      (run.digest * 1099511628211ull)
      ^ destructionfixture::stateHash(stepped->world.destruction, dynamics);
    run.jointWakes += jointedAsleep && !asleep(jointed) ? 1u : 0u;

    if (waitingForContact && step < 230u && !asleep(struck)) {
      ++run.contactWakes;
      waitingForContact = false;
    }

    for (const uint32_t crate : perchedAsleep) {
      run.updateWakes += asleep(crate) ? 0u : 1u;
    }

    run.wideAwake += dynamics.awake.indices.size() > serialLimit ? 1u : 0u;
    run.sparseReady +=
      dynamics.awake.layout == dynamics.bodies.layout
          && 2u * dynamics.awake.indices.size() < dynamics.bodies.rigids.size()
        ? 1u
        : 0u;
    run.frozen += some.destruction.frozen;
    run.thawed += some.destruction.thawed;
    run.removedFrozen += some.destruction.removedFrozen;
    run.created += static_cast<uint32_t>(some.destruction.created.size());
  }

  run.sleepingCuts = stepped->sleepingCuts;
  run.poseWakes = stepped->poseWakes;
  REQUIRE(stepped->world.dynamics.manifolds.size() > serialLimit);

  return run;
}

}

TEST_CASE(
  "World::step steps bit for bit as a twin whose destruction update is forced "
  "to scan every body, through a blast, freezing, thawing, debris removal, "
  "joints, tier switches, appends, removals and extra dynamics steps",
  "[destruction][destructionworld][awake]"
) {
  const std::array<std::pair<uint32_t, uint32_t>, 4> settings{
    {{1u, 1u}, {4u, 8u}, {8u, 4u}, {8u, 8u}}
  };
  std::optional<uint64_t> digest;

  for (const auto& [workers, width] : settings) {
    const TwinRun run = runTwins(workers, width);

    INFO("workers " << workers << " width " << width);
    CHECK(run.sparseReady > twinSteps / 2u);
    CHECK(run.frozen > 0u);
    CHECK(run.thawed > 0u);
    CHECK(run.removedFrozen > 0u);
    CHECK(run.created > 0u);
    CHECK(run.sleepingCuts > 0u);
    CHECK(run.wideAwake > 10u);
    CHECK(run.jointWakes == 1u);
    CHECK(run.contactWakes == 1u);
    CHECK(run.updateWakes == 2u);
    CHECK(run.poseWakes == 1u);

    if (digest) {
      CHECK(run.digest == *digest);
    }

    digest = run.digest;
  }
}
