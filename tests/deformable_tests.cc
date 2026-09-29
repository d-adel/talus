#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"
#include "landingbar.h"
#include "windowmonitor.h"

#include <talus/deformable/contact/contact.h>
#include <talus/deformable/factor/factor.h>
#include <talus/deformable/material/material.h>
#include <talus/deformable/solver/solver.h>
#include <talus/field/analytic/analytic.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/support/support.h>
#include <talus/objectmodel/component/bodyid/bodyid.h>
#include <talus/objectmodel/component/pose/pose.h>
#include <talus/objectmodel/component/sdfgrid/sdfgrid.h>
#include <talus/objectmodel/component/sharedfield/sharedfield.h>
#include <talus/objectmodel/world/world.h>
#include <talus/physics/fieldshape/fieldshape.h>
#include <talus/physics/joltlibrary/joltlibrary.h>
#include <talus/physics/layers/layers.h>
#include <talus/physics/physicsworld/physicsworld.h>
#include <talus/runtime/runtime/runtime.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/component_wise.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using talus::objectmodel::DeformableState;
using talus::objectmodel::Entity;
using talus::objectmodel::PhysicalMaterial;
using talus::objectmodel::PreparedMesh;
using talus::objectmodel::World;
using talus::physics::JoltLibrary;
using talus::physics::PhysicsWorld;
using talus::runtime::Runtime;

namespace {

constexpr float HALF_LENGTH = 0.5f;
constexpr float HALF_THICKNESS = 0.05f;
constexpr float ANCHOR_END = -0.45f;
constexpr float ANCHOR_TOLERANCE = 1.0e-4f;
constexpr float YOUNGS = 1.0e7f;
constexpr float POISSONS = 0.3f;
constexpr float DENSITY = 500.0f;

constexpr uint32_t BULK_LANE = 1u;

constexpr float BASE_VOXEL = 0.025f;
constexpr float FINE_VOXEL = 0.0125f;
constexpr float COARSE_VOXEL = 0.05f;

constexpr uint32_t DYNAMIC_STEPS = 40u;

TEST_CASE("a deformable preconditioner follows tet rotation", "[deformable]") {
  talus::objectmodel::PreparedTet tet{};

  tet.volume = 1.0f / 6.0f;
  tet.restInverse = glm::mat3(1.0f);

  const talus::deformable::TetStiffness stiffness =
    talus::deformable::tetStiffness(
      tet,
      talus::deformable::ElasticParams{60.0f, 0.25f, 1.0f}
    );

  const glm::mat3 rotation = glm::mat3_cast(
    glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)))
  );
  const std::array<glm::mat3, 4> diagonal =
    talus::deformable::stiffnessDiagonal(stiffness, rotation);

  for (uint32_t node = 0u; node != 4u; ++node) {
    for (int axis = 0; axis != 3; ++axis) {
      std::array<glm::vec3, 4> direction{};
      std::array<glm::vec3, 4> response{};
      direction[node][axis] = 1.0f;

      talus::deformable::applyStiffness(
        direction,
        rotation,
        stiffness,
        response
      );

      const glm::vec3 expected = diagonal[node] * direction[node];

      REQUIRE(glm::length(response[node] - expected) < 1.0e-5f);
    }
  }
}

class Clock {
public:
  [[nodiscard]] double seconds() const {
    return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - m_began
    )
      .count();
  }

private:
  std::chrono::steady_clock::time_point m_began =
    std::chrono::steady_clock::now();
};

glm::vec3 pull() {
  return glm::vec3(0.0f, -PhysicsWorld::GRAVITY, 0.0f);
}

float analyticDeflection() {
  const float length = HALF_LENGTH - ANCHOR_END;
  const float area = 4.0f * HALF_THICKNESS * HALF_THICKNESS;
  const float second =
    2.0f * HALF_THICKNESS * std::pow(2.0f * HALF_THICKNESS, 3.0f) / 12.0f;

  const float load = DENSITY * area * PhysicsWorld::GRAVITY;

  return load * std::pow(length, 4.0f) / (8.0f * YOUNGS * second);
}

std::vector<PhysicalMaterial> beamMaterials() {
  std::vector<PhysicalMaterial> materials(2);

  materials[BULK_LANE].density = DENSITY;
  materials[BULK_LANE].youngsModulus = YOUNGS;
  materials[BULK_LANE].poissonsRatio = POISSONS;

  return materials;
}

Entity beamSupport(World& world) {
  const glm::vec3 low(-2.0f, -2.0f, -2.0f);
  const glm::vec3 high(ANCHOR_END + ANCHOR_TOLERANCE, 2.0f, 2.0f);

  const Entity entity = world.createAuthored();

  talus::objectmodel::Pose pose{};
  pose.position = 0.5f * (low + high);

  world.emplaceAuthored<talus::objectmodel::Pose>(entity, pose);

  world.emplaceAuthored<talus::objectmodel::Support>(
    entity,
    talus::objectmodel::Support{0.5f * (high - low)}
  );

  return entity;
}

talus::field::Primitive spanning(
  const glm::vec3& low,
  const glm::vec3& high,
  uint32_t lane
) {
  talus::field::Primitive primitive =
    fixture::box(0.5f * (low + high), 0.5f * (high - low));

  primitive.shapeConfig.y = lane;

  return primitive;
}

Entity beamEntity(
  World& world,
  float voxel,
  const glm::vec3& at = glm::vec3(0.0f)
) {
  const float skin = 0.2f * voxel;

  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(
    static_cast<uint32_t>(std::lround(1.0f / voxel)) + 3u,
    static_cast<uint32_t>(std::lround(2.0f * HALF_THICKNESS / voxel)) + 3u,
    static_cast<uint32_t>(std::lround(2.0f * HALF_THICKNESS / voxel)) + 3u
  );

  talus::objectmodel::Pose placed{};
  placed.position = at;

  const Entity entity = world.createSdfGrid(placed, grid);

  const glm::vec3 thick(0.0f, HALF_THICKNESS + skin, HALF_THICKNESS + skin);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(
      glm::vec3(-HALF_LENGTH - skin, 0.0f, 0.0f) - thick,
      glm::vec3(HALF_LENGTH + skin, 0.0f, 0.0f) + thick,
      BULK_LANE
    )
  ));

  const glm::vec3 minimum(
    -HALF_LENGTH - voxel,
    -HALF_THICKNESS - voxel,
    -HALF_THICKNESS - voxel
  );

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      minimum,
      -minimum,
      grid.resolution
    )
  );

  world.setMaterials(beamMaterials());

  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{}
  );

  return entity;
}

constexpr float BLOCKER_HALF = 0.06f;

Entity fieldBlocker(World& world, const glm::vec3& at) {
  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(9u);

  talus::objectmodel::Pose pose{};
  pose.position = at;

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(glm::vec3(-BLOCKER_HALF), glm::vec3(BLOCKER_HALF), 0u)
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      glm::vec3(-0.09f),
      glm::vec3(0.09f),
      grid.resolution
    )
  );

  world.addStaticBody(entity);

  return entity;
}

struct Dropped {
  Entity entity = talus::objectmodel::INVALID_ENTITY;
  JPH::BodyID body;
};

Dropped dropBody(
  Runtime& runtime,
  const glm::vec3& at,
  const talus::field::Primitive& primitive,
  float reach,
  float mass
) {
  World& world = runtime.world();

  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(25u);

  talus::objectmodel::Pose pose{};
  pose.position = at;

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(entity, primitive));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      glm::vec3(-reach),
      glm::vec3(reach),
      grid.resolution
    )
  );

  world.addRigidBody(entity);

  runtime.refreshBody(entity);

  const auto* id = world.registry().try_get<talus::objectmodel::BodyId>(entity);

  REQUIRE(id != nullptr);

  runtime.setMass(entity, mass);

  runtime.physics().bodies().SetRestitution(JPH::BodyID(id->id), 0.0f);
  runtime.physics().optimizeBroadPhase();

  return Dropped{entity, JPH::BodyID(id->id)};
}

Dropped dropCrate(
  Runtime& runtime,
  const glm::vec3& at,
  float half,
  float mass
) {
  return dropBody(
    runtime,
    at,
    spanning(glm::vec3(-half), glm::vec3(half), BULK_LANE),
    1.5f * half,
    mass
  );
}

Dropped dropBall(
  Runtime& runtime,
  const glm::vec3& at,
  float radius,
  float mass
) {
  talus::field::Primitive primitive = fixture::sphere(glm::vec3(0.0f), radius);

  primitive.shapeConfig.y = BULK_LANE;

  return dropBody(runtime, at, primitive, 1.5f * radius, mass);
}

void prepare(Runtime& runtime, Entity beam) {
  const Entity support = beamSupport(runtime.world());

  REQUIRE(support != talus::objectmodel::INVALID_ENTITY);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  REQUIRE(runtime.world().deformablePrepared(beam) != nullptr);
  REQUIRE(runtime.world().deformableState(beam) != nullptr);
  REQUIRE_FALSE(
    runtime.world().registry().all_of<talus::objectmodel::BodyId>(beam)
  );
}

talus::deformable::Settings staticSolve() {
  talus::deformable::Settings settings;

  settings.cgIterations = 1200u;
  settings.tolerance = 1.0e-5f;

  return settings;
}

talus::deformable::Settings dynamicSolve(uint32_t substeps) {
  talus::deformable::Settings settings;

  settings.substeps = substeps;

  return settings;
}

TEST_CASE("a deformable report exposes CG cap pressure", "[deformable]") {
  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  talus::deformable::Settings settings = dynamicSolve(1u);

  settings.cgIterations = 1u;
  settings.tolerance = 1.0e-6f;

  runtime.setDeformableSettings(settings);
  runtime.step();

  const talus::deformable::StepReport& report = runtime.deformableReport();

  REQUIRE(report.cgIterations == 3u);
  REQUIRE(report.cgCappedSolves == 3u);
  REQUIRE(report.cgWorstRelativeResidual > settings.tolerance);
  REQUIRE(report.setupSeconds > 0.0);
  REQUIRE(report.elasticSeconds > 0.0);
  REQUIRE(report.primaryCgSeconds > 0.0);
  REQUIRE(report.primaryCgOperatorApplications == 6u);
  REQUIRE(report.primaryCgTetSeconds > 0.0);
  REQUIRE(
    report.primaryCgSeconds >= report.primaryCgMassSeconds
                                 + report.primaryCgTetSeconds
                                 + report.primaryCgAnchorSeconds
  );
  REQUIRE(report.basisSeconds == 0.0);
  REQUIRE(
    report.cpuSeconds >= report.setupSeconds + report.elasticSeconds
                           + report.primaryCgSeconds + report.contactSeconds
  );
}

float tipDrop(const PreparedMesh& mesh, const DeformableState& state) {
  float furthest = -HALF_LENGTH;

  for (const glm::vec3& rest : mesh.restPositions) {
    furthest = glm::max(furthest, rest.x);
  }

  float drop = 0.0f;
  uint32_t counted = 0u;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.restPositions[node].x < furthest - 1.0e-4f) {
      continue;
    }

    drop += mesh.restPositions[node].y - state.position[node].y;
    ++counted;
  }

  REQUIRE(counted > 0u);

  return drop / static_cast<float>(counted);
}

float beamTip(const Runtime& runtime, Entity beam) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);

  REQUIRE(mesh != nullptr);
  REQUIRE(state != nullptr);

  return tipDrop(*mesh, *state);
}

float surfaceAt(const Runtime& runtime, Entity beam, float along) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);

  REQUIRE(mesh != nullptr);
  REQUIRE(state != nullptr);

  float top = std::numeric_limits<float>::lowest();

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    if (std::abs(mesh->restPositions[node].x - along) > 0.06f) {
      continue;
    }

    top = glm::max(top, state->position[node].y);
  }

  REQUIRE(top > std::numeric_limits<float>::lowest());

  return top;
}

void settleAtEquilibrium(Runtime& runtime, Entity beam) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);

  REQUIRE(mesh != nullptr);

  runtime.world().setDeformableState(
    beam,
    talus::deformable::solveEquilibrium(
      *mesh,
      runtime.world().materials(),
      runtime.world().registry().get<talus::objectmodel::Pose>(beam),
      staticSolve(),
      pull()
    )
  );
}

float beamMomentum(const Runtime& runtime, Entity beam) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);

  float momentum = 0.0f;

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    momentum += mesh->nodeMass[node] * state->velocity[node].y;
  }

  return momentum;
}

float beamEnergy(const Runtime& runtime, Entity beam) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);

  float energy = runtime.deformableReport().elasticEnergy;

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    energy += 0.5f * mesh->nodeMass[node]
              * glm::dot(state->velocity[node], state->velocity[node]);
    energy +=
      mesh->nodeMass[node] * PhysicsWorld::GRAVITY * state->position[node].y;
  }

  return energy;
}

}

TEST_CASE(
  "a deformable beam under gravity deflects like Euler-Bernoulli",
  "[deformable]"
) {
  const Clock clock;

  const float expected = analyticDeflection();

  std::vector<float> errors;

  for (const float voxel : {BASE_VOXEL, FINE_VOXEL}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, voxel);

    REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

    Runtime runtime(source, library);

    prepare(runtime, beam);

    const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);

    const DeformableState settled = talus::deformable::solveEquilibrium(
      mesh,
      runtime.world().materials(),
      runtime.world().registry().get<talus::objectmodel::Pose>(beam),
      staticSolve(),
      pull()
    );

    const float measured = tipDrop(mesh, settled);

    errors.push_back(std::abs(measured - expected) / expected);

    std::cout << "deformable: voxel " << voxel << " elements "
              << mesh.tets.size() << " tip " << measured << " expected "
              << expected
              << " (axis-aligned beam only; the solid is the union of lattice"
                 " cells whose eight corners are inside, so the surface"
                 " stair-steps at the voxel size)"
              << '\n';
  }

  std::cout << "deformable: deflection took " << clock.seconds()
            << " wall seconds in this tree" << '\n';

  REQUIRE(errors.size() == 2u);
  REQUIRE(errors[0] < 0.30f);
  REQUIRE(errors[1] < errors[0]);
}

TEST_CASE("a deformable beam holds its equilibrium", "[deformable]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, BASE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const float before = beamTip(runtime, beam);

  for (uint32_t taken = 0u; taken != 10u; ++taken) {
    runtime.step();
  }

  const float after = beamTip(runtime, beam);

  std::cout << "deformable: equilibrium tip " << before << " after ten steps "
            << after << ", " << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(before > 0.0f);
  REQUIRE(std::abs(after - before) < 1.0e-4f);
}

TEST_CASE("a deformable beam takes a sphere strike", "[deformable]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  constexpr float BALL_MASS = 0.5f;

  const float resting = surfaceAt(runtime, beam, 0.3f);

  const JPH::BodyID ball =
    dropBall(runtime, glm::vec3(0.3f, resting + 0.11f, 0.0f), 0.05f, BALL_MASS)
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  bodies.SetLinearVelocity(ball, JPH::Vec3(0.0f, -2.0f, 0.0f));

  const float mass = BALL_MASS;

  runtime.step();

  const float before =
    beamEnergy(runtime, beam)
    + 0.5f * mass * bodies.GetLinearVelocity(ball).LengthSq()
    + mass * PhysicsWorld::GRAVITY * bodies.GetPosition(ball).GetY();

  const float ballBefore = mass * bodies.GetLinearVelocity(ball).GetY();
  const float beamBefore = beamMomentum(runtime, beam);

  float exchanged = 0.0f;

  glm::vec3 onBall(0.0f);
  glm::vec3 onNodes(0.0f);

  for (uint32_t taken = 1u; taken != DYNAMIC_STEPS; ++taken) {
    runtime.step();

    for (const talus::deformable::RigidExchange& held :
         runtime.deformableReport().exchange) {
      if (held.body == ball.GetIndexAndSequenceNumber()) {
        exchanged += held.impulse.y;

        onBall += held.impulse;
        onNodes += held.nodeImpulse;
      }
    }
  }

  const float fall = mass * PhysicsWorld::GRAVITY * PhysicsWorld::TIME_STEP
                     * float(DYNAMIC_STEPS - 1u);

  const float ballLost =
    ballBefore - (mass * bodies.GetLinearVelocity(ball).GetY() + fall);

  const float beamGained = beamMomentum(runtime, beam) - beamBefore;

  const float after =
    beamEnergy(runtime, beam)
    + 0.5f * mass * bodies.GetLinearVelocity(ball).LengthSq()
    + mass * PhysicsWorld::GRAVITY * bodies.GetPosition(ball).GetY();

  std::cout << "deformable: ball lost " << ballLost << " beam gained "
            << beamGained << " exchanged " << exchanged << " energy " << before
            << " -> " << after
            << " (the only dissipation is implicit Euler and the contact"
               " projection, so the check claims no more than that kinetic"
               " plus elastic energy does not rise), "
            << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(std::abs(exchanged) > 0.0f);
  REQUIRE(std::abs(ballLost + exchanged) <= 0.05f * std::abs(exchanged));
  REQUIRE(glm::length(onBall + onNodes) <= 1.0e-4f * glm::length(onBall));
  REQUIRE(after <= before);
}

TEST_CASE(
  "a deformable beam converges as the internal timestep halves",
  "[deformable]"
) {
  const Clock clock;

  std::vector<float> tips;

  for (const uint32_t substeps : {4u, 8u}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

    Runtime runtime(source, library);

    prepare(runtime, beam);

    runtime.setDeformableSettings(dynamicSolve(substeps));

    for (uint32_t taken = 0u; taken != DYNAMIC_STEPS; ++taken) {
      runtime.step();
    }

    tips.push_back(beamTip(runtime, beam));
  }

  REQUIRE(tips.size() == 2u);
  REQUIRE(tips[0] > 0.0f);

  std::cout << "deformable: tip after " << DYNAMIC_STEPS
            << " steps at four substeps " << tips[0] << " at eight " << tips[1]
            << ", " << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(std::abs(tips[1] - tips[0]) / tips[0] < 0.05f);
}

Entity blockBody(World& world, float half, float voxel) {
  const float skin = 0.2f * voxel;

  talus::objectmodel::SdfGrid grid{};

  const uint32_t side =
    static_cast<uint32_t>(std::lround(2.0f * half / voxel)) + 3u;

  grid.resolution = glm::uvec3(side);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, 0.4f, 0.0f);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(glm::vec3(-half - skin), glm::vec3(half + skin), BULK_LANE)
  ));

  const glm::vec3 minimum(-half - voxel);

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      minimum,
      -minimum,
      grid.resolution
    )
  );

  world.setMaterials(beamMaterials());

  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{}
  );

  return entity;
}

TEST_CASE(
  "a hex element reproduces its constitutive response",
  "[deformable]"
) {
  constexpr float SIDE = 0.05f;
  constexpr float STRAIN = 1.0e-3f;

  talus::objectmodel::PreparedHex hex;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    hex.nodes[corner] = corner;
  }

  hex.halfExtents = glm::vec3(0.5f * SIDE);
  hex.volume = SIDE * SIDE * SIDE;

  const talus::deformable::HexStiffness stiffness =
    talus::deformable::hexStiffness(
      hex,
      talus::deformable::ElasticParams{YOUNGS, POISSONS, DENSITY}
    );

  std::array<glm::vec3, 8> corner{};

  for (uint32_t index = 0u; index != 8u; ++index) {
    corner[index] = hex.halfExtents
                    * glm::vec3(
                      (index & 1u) != 0u ? 1.0f : -1.0f,
                      (index & 2u) != 0u ? 1.0f : -1.0f,
                      (index & 4u) != 0u ? 1.0f : -1.0f
                    );
  }

  const auto energyOf = [&](const std::array<glm::vec3, 8>& offset) {
    float total = 0.0f;

    for (uint32_t row = 0u; row != 8u; ++row) {
      glm::vec3 sum(0.0f);

      for (uint32_t column = 0u; column != 8u; ++column) {
        sum += stiffness.block[row * 8u + column] * offset[column];
      }

      total += 0.5f * glm::dot(offset[row], sum);
    }

    return total;
  };

  const auto forceOf = [&](const std::array<glm::vec3, 8>& offset) {
    float worst = 0.0f;

    for (uint32_t row = 0u; row != 8u; ++row) {
      glm::vec3 sum(0.0f);

      for (uint32_t column = 0u; column != 8u; ++column) {
        sum += stiffness.block[row * 8u + column] * offset[column];
      }

      worst = glm::max(worst, glm::length(sum));
    }

    return worst;
  };

  const float shear = YOUNGS / (2.0f * (1.0f + POISSONS));
  const float bulk = YOUNGS / (3.0f * (1.0f - 2.0f * POISSONS));

  std::array<glm::vec3, 8> moved{};

  for (uint32_t index = 0u; index != 8u; ++index) {
    moved[index] = glm::vec3(0.013f, -0.021f, 0.007f);
  }

  REQUIRE(forceOf(moved) < 1.0e-3f * YOUNGS * hex.volume * STRAIN);

  for (uint32_t index = 0u; index != 8u; ++index) {
    moved[index] = STRAIN * glm::vec3(-corner[index].y, corner[index].x, 0.0f);
  }

  REQUIRE(forceOf(moved) < 1.0e-3f * YOUNGS * hex.volume * STRAIN);

  for (uint32_t index = 0u; index != 8u; ++index) {
    moved[index] = STRAIN
                   * glm::vec3(
                     corner[index].x,
                     -POISSONS * corner[index].y,
                     -POISSONS * corner[index].z
                   );
  }

  const float uniaxial = 0.5f * YOUNGS * STRAIN * STRAIN * hex.volume;

  std::cout << "hexlaw: uniaxial " << energyOf(moved) << " expected "
            << uniaxial << std::endl;

  REQUIRE(std::abs(energyOf(moved) - uniaxial) < 1.0e-3f * uniaxial);

  for (uint32_t index = 0u; index != 8u; ++index) {
    moved[index] = glm::vec3(STRAIN * corner[index].y, 0.0f, 0.0f);
  }

  const float simple = 0.5f * shear * STRAIN * STRAIN * hex.volume;

  std::cout << "hexlaw: shear " << energyOf(moved) << " expected " << simple
            << std::endl;

  REQUIRE(std::abs(energyOf(moved) - simple) < 1.0e-3f * simple);

  for (uint32_t index = 0u; index != 8u; ++index) {
    moved[index] = STRAIN * corner[index];
  }

  const float dilation = 0.5f * 9.0f * bulk * STRAIN * STRAIN * hex.volume;

  std::cout << "hexlaw: dilation " << energyOf(moved) << " expected "
            << dilation << std::endl;

  REQUIRE(std::abs(energyOf(moved) - dilation) < 1.0e-3f * dilation);
}

TEST_CASE(
  "a tet element reproduces its constitutive response",
  "[deformable]"
) {
  constexpr float SIDE = 0.05f;
  constexpr float STRAIN = 1.0e-3f;

  const std::array<glm::vec3, 4> rest{
    glm::vec3(0.0f, 0.0f, 0.0f),
    glm::vec3(SIDE, 0.0f, 0.0f),
    glm::vec3(0.0f, SIDE, 0.0f),
    glm::vec3(0.0f, 0.0f, SIDE)
  };

  talus::objectmodel::PreparedTet tet;

  for (uint32_t corner = 0u; corner != 4u; ++corner) {
    tet.nodes[corner] = corner;
  }

  tet.restInverse = glm::inverse(
    glm::mat3(rest[1] - rest[0], rest[2] - rest[0], rest[3] - rest[0])
  );
  tet.volume = SIDE * SIDE * SIDE / 6.0f;

  const talus::deformable::TetStiffness stiffness =
    talus::deformable::tetStiffness(
      tet,
      talus::deformable::ElasticParams{YOUNGS, POISSONS, DENSITY}
    );

  const auto forceOf = [&](const std::array<glm::vec3, 4>& offset) {
    float worst = 0.0f;

    for (uint32_t row = 0u; row != 4u; ++row) {
      glm::vec3 sum(0.0f);

      for (uint32_t column = 0u; column != 4u; ++column) {
        sum += stiffness.block[row * 4u + column] * offset[column];
      }

      worst = glm::max(worst, glm::length(sum));
    }

    return worst;
  };

  const auto energyOf = [&](const std::array<glm::vec3, 4>& offset) {
    float sum = 0.0f;

    for (uint32_t row = 0u; row != 4u; ++row) {
      for (uint32_t column = 0u; column != 4u; ++column) {
        sum += glm::dot(
          offset[row],
          stiffness.block[row * 4u + column] * offset[column]
        );
      }
    }

    return 0.5f * sum;
  };

  float reference = 0.0f;

  for (uint32_t at = 0u; at != 16u; ++at) {
    for (int column = 0; column != 3; ++column) {
      reference = glm::max(reference, glm::length(stiffness.block[at][column]));
    }
  }

  std::array<glm::vec3, 4> moved{};

  for (uint32_t index = 0u; index != 4u; ++index) {
    moved[index] = glm::vec3(0.013f, -0.021f, 0.007f);
  }

  const float shifted = forceOf(moved) / (reference * glm::length(moved[0]));

  for (uint32_t index = 0u; index != 4u; ++index) {
    moved[index] = STRAIN * glm::vec3(-rest[index].y, rest[index].x, 0.0f);
  }

  const float turned = forceOf(moved) / (reference * STRAIN * SIDE);

  for (uint32_t index = 0u; index != 4u; ++index) {
    moved[index] = STRAIN
                   * glm::vec3(
                     rest[index].x,
                     -POISSONS * rest[index].y,
                     -POISSONS * rest[index].z
                   );
  }

  const float uniaxial = 0.5f * YOUNGS * STRAIN * STRAIN * tet.volume;
  const float measured = energyOf(moved);

  std::cout << "tetlaw: translation " << shifted << " rotation " << turned
            << " uniaxial " << measured << " expected " << uniaxial << " ratio "
            << measured / uniaxial << std::endl;

  REQUIRE(shifted < 1.0e-5f);
  REQUIRE(turned < 1.0e-4f);

  REQUIRE(std::abs(measured - uniaxial) < 1.0e-3f * uniaxial);
}

TEST_CASE("a hex beam bends where a tet beam locks", "[deformable]") {
  const float expected = analyticDeflection();

  std::vector<float> errors;

  for (const bool hex : {false, true}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    Runtime runtime(source, library);

    talus::deformable::Settings settings = dynamicSolve(4u);

    settings.element = hex ? talus::deformable::VolumeElementType::Hex
                           : talus::deformable::VolumeElementType::Tet;

    runtime.setDeformableSettings(settings);

    prepare(runtime, beam);

    const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);

    REQUIRE(mesh.restPositions.size() == 189u);
    REQUIRE((hex ? mesh.hexes.size() : mesh.tets.size()) == (hex ? 80u : 480u));

    const DeformableState settled = talus::deformable::solveEquilibrium(
      mesh,
      runtime.world().materials(),
      runtime.world().registry().get<talus::objectmodel::Pose>(beam),
      staticSolve(),
      pull()
    );

    errors.push_back((tipDrop(mesh, settled) - expected) / expected);
  }

  std::cout << "hexbend: 189 nodes, tet " << 100.0f * errors[0] << "% hex "
            << 100.0f * errors[1] << "% against the Euler-Bernoulli drop "
            << expected << std::endl;

  REQUIRE(errors[0] < -0.4f);
  REQUIRE(std::abs(errors[1]) < 0.05f);
}

TEST_CASE("a deformable beam meets a field body", "[deformable]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  const Entity blocker = fieldBlocker(source, glm::vec3(0.3f, 0.0f, 0.0f));

  REQUIRE(beam != blocker);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  const auto* body =
    runtime.world().registry().try_get<talus::objectmodel::BodyId>(blocker);

  REQUIRE(body != nullptr);

  runtime.setDeformableSettings(dynamicSolve(4u));

  runtime.physics().optimizeBroadPhase();

  std::ostringstream captured;

  std::streambuf* before = std::cout.rdbuf(captured.rdbuf());

  uint32_t counted = 0u;
  uint32_t touched = 0u;

  constexpr uint32_t TAKEN = 20u;

  for (uint32_t taken = 0u; taken != TAKEN; ++taken) {
    runtime.step();

    const talus::deformable::StepReport& report = runtime.deformableReport();

    counted += report.unsupportedPartners;

    if (report.maxPenetration > 0.0f) {
      ++touched;
    }
  }

  std::cout.rdbuf(before);

  const std::string logged = captured.str();

  const std::string wanted =
    "A deformable meets a body its contact path cannot read";

  uint32_t said = 0u;

  for (size_t at = logged.find(wanted); at != std::string::npos;
       at = logged.find(wanted, at + wanted.size())) {
    ++said;
  }

  std::cout << "deformable: field body touched on " << touched
            << " steps, unsupported partners " << counted << " logged " << said
            << ", " << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(counted == 0u);
  REQUIRE(said == 0u);
  REQUIRE(touched > 0u);
  REQUIRE(runtime.world().deformablePrepared(beam) != nullptr);
}

TEST_CASE("a deformable runtime contains non-finite state", "[deformable]") {
  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);
  const Entity blocker = fieldBlocker(source, glm::vec3(0.3f, 0.04f, 0.0f));

  REQUIRE(beam != blocker);

  Runtime runtime(source, library);

  prepare(runtime, beam);
  runtime.physics().optimizeBroadPhase();

  runtime.world().patchDeformableState(beam, [](DeformableState& state) {
    state.position.front().x = std::numeric_limits<float>::quiet_NaN();
  });

  runtime.step();

  REQUIRE(runtime.deformableReport().nonFinite);
  REQUIRE(runtime.deformableReport().contacts == 0u);

  runtime.step();

  REQUIRE(runtime.deformableReport().nonFinite);
  REQUIRE(runtime.deformableReport().contacts == 0u);
}

namespace {

struct CrateLoad {
  float along = 0.0f;
  float across = 0.0f;
  float half = 0.0f;
  float mass = 0.0f;
};

void settleUnderCrates(
  Runtime& runtime,
  Entity beam,
  std::span<const CrateLoad> crates
) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);

  REQUIRE(mesh != nullptr);

  std::vector<glm::vec3> load(mesh->restPositions.size(), glm::vec3(0.0f));

  for (const CrateLoad& crate : crates) {
    std::vector<uint32_t> patch;

    for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
      const glm::vec3& rest = mesh->restPositions[node];

      if (
        std::abs(rest.x - crate.along) <= crate.half + 0.001f
        && std::abs(rest.z - crate.across) <= crate.half + 0.001f
        && rest.y > HALF_THICKNESS - 0.001f
      ) {
        patch.push_back(static_cast<uint32_t>(node));
      }
    }

    REQUIRE(!patch.empty());

    for (const uint32_t node : patch) {
      load[node] += glm::vec3(
        0.0f,
        -crate.mass * PhysicsWorld::GRAVITY / static_cast<float>(patch.size()),
        0.0f
      );
    }
  }

  runtime.world().setDeformableState(
    beam,
    talus::deformable::solveEquilibrium(
      *mesh,
      runtime.world().materials(),
      runtime.world().registry().get<talus::objectmodel::Pose>(beam),
      staticSolve(),
      pull(),
      load
    )
  );
}

void settleUnderCrate(
  Runtime& runtime,
  Entity beam,
  float along,
  float half,
  float mass
) {
  const std::array<CrateLoad, 1> crates{{CrateLoad{along, 0.0f, half, mass}}};

  settleUnderCrates(runtime, beam, crates);
}

struct WakeReasons {
  uint32_t edit = 0u;
  uint32_t mover = 0u;
  uint32_t destroy = 0u;
  uint32_t activation = 0u;
  uint32_t global = 0u;
  uint32_t other = 0u;
  uint32_t editDisarms = 0u;

  void add(const talus::runtime::DeformableActivity& activity) {
    edit += activity.editWakes;
    mover += activity.moverWakes;
    destroy += activity.destroyWakes;
    activation += activity.activationWakes;
    global += activity.globalWakes;
    other += activity.otherWakes;
    editDisarms += activity.editDisarms;
  }
};

std::ostream& operator<<(std::ostream& out, const WakeReasons& reasons) {
  return out << " wakes edit " << reasons.edit << " mover " << reasons.mover
             << " destroy " << reasons.destroy << " activation "
             << reasons.activation << " global " << reasons.global << " other "
             << reasons.other << " editDisarms " << reasons.editDisarms;
}

struct FrozenCrate {
  Dropped crate;
  uint32_t frozenAt = 0u;
};

FrozenCrate freezeUnderCrate(
  Runtime& runtime,
  Entity beam,
  float along,
  float half,
  float mass,
  WakeReasons& reasons
) {
  constexpr uint32_t REACH = 300u;

  settleUnderCrate(runtime, beam, along, half, mass);

  const float resting = surfaceAt(runtime, beam, along);

  FrozenCrate frozen;

  frozen.crate = dropCrate(
    runtime,
    glm::vec3(along, resting + half + 0.001f, 0.0f),
    half,
    mass
  );

  for (uint32_t taken = 1u; taken != REACH + 1u && frozen.frozenAt == 0u;
       ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (runtime.deformableActivity().freezes != 0u) {
      frozen.frozenAt = taken;
    }
  }

  REQUIRE(frozen.frozenAt != 0u);
  REQUIRE(runtime.deformableQuiescent(beam));
  REQUIRE_FALSE(runtime.physics().bodies().IsActive(frozen.crate.body));

  return frozen;
}

}

TEST_CASE("a deformable beam carries a resting box", "[.][quiescent]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;

  const float resting = surfaceAt(runtime, beam, 0.3f);

  const JPH::BodyID box = dropCrate(
                            runtime,
                            glm::vec3(0.3f, resting + BOX_HALF + 0.001f, 0.0f),
                            BOX_HALF,
                            BOX_MASS
  )
                            .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  constexpr uint32_t REACH = 300u;
  constexpr uint32_t SETTLE = 1800u;
  constexpr uint32_t HOLD = 30u;

  uint32_t contacts = 0u;
  uint32_t sleptAt = 0u;
  uint32_t frozenAt = 0u;
  uint32_t steppedSteps = 0u;
  uint32_t contactSteps = 0u;
  float penetration = 0.0f;

  WakeReasons reasons;

  const auto observe = [&]() {
    const talus::deformable::StepReport& report = runtime.deformableReport();

    contacts += report.contacts;
    penetration = glm::max(penetration, report.maxPenetration);
    steppedSteps += report.nodes != 0u ? 1u : 0u;
    contactSteps += report.nodes != 0u && report.contacts > 0u ? 1u : 0u;

    reasons.add(runtime.deformableActivity());
  };

  for (uint32_t taken = 1u; taken != SETTLE + 1u && frozenAt == 0u; ++taken) {
    runtime.step();

    observe();

    if (!bodies.IsActive(box) && sleptAt == 0u) {
      sleptAt = taken;
    }

    if (runtime.deformableActivity().freezes != 0u) {
      frozenAt = taken;
    }
  }

  const float frozenHeight = bodies.GetPosition(box).GetY();

  uint32_t awakeSteps = 0u;
  float heightDrift = 0.0f;

  for (uint32_t taken = 0u; taken != HOLD; ++taken) {
    runtime.step();

    observe();

    awakeSteps += bodies.IsActive(box) ? 1u : 0u;
    heightDrift = glm::max(
      heightDrift,
      std::abs(bodies.GetPosition(box).GetY() - frozenHeight)
    );
  }

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);

  std::cout << "deformable: substeps " << runtime.deformableSettings().substeps
            << " spectralRadius " << runtime.deformableSettings().spectralRadius
            << " penetration " << penetration << " voxel " << voxel
            << " box slept at step " << sleptAt << " beam froze at step "
            << frozenAt << " held for " << HOLD << " height drift "
            << heightDrift << " awake steps " << awakeSteps << " contact steps "
            << contactSteps << " of " << steppedSteps << " stepped" << reasons
            << ", " << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(sleptAt != 0u);
  REQUIRE(sleptAt <= REACH);
  REQUIRE(frozenAt != 0u);
  REQUIRE(awakeSteps == 0u);
  REQUIRE(heightDrift <= 1.0e-4f);
  REQUIRE(contactSteps == steppedSteps);
  REQUIRE(penetration < voxel);
  REQUIRE(std::abs(bodies.GetPosition(box).GetX()) < HALF_LENGTH - BOX_HALF);
  REQUIRE(contacts > 0u);
}

TEST_CASE("a deformable beam freezes under a resting box", "[deformable]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;

  settleUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const float resting = surfaceAt(runtime, beam, BOX_AT);

  const JPH::BodyID box =
    dropCrate(
      runtime,
      glm::vec3(BOX_AT, resting + BOX_HALF + 0.001f, 0.0f),
      BOX_HALF,
      BOX_MASS
    )
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  constexpr uint32_t REACH = 300u;
  constexpr uint32_t HOLD = 30u;

  uint32_t contacts = 0u;
  uint32_t uncontactedSteps = 0u;
  uint32_t frozenAt = 0u;
  float penetration = 0.0f;

  WakeReasons reasons;

  for (uint32_t taken = 1u; taken != REACH + 1u && frozenAt == 0u; ++taken) {
    runtime.step();

    const talus::deformable::StepReport& report = runtime.deformableReport();

    contacts += report.contacts;
    uncontactedSteps += report.contacts == 0u ? 1u : 0u;
    penetration = glm::max(penetration, report.maxPenetration);

    reasons.add(runtime.deformableActivity());

    if (runtime.deformableActivity().freezes != 0u) {
      frozenAt = taken;
    }
  }

  const bool asleepAtFreeze = !bodies.IsActive(box);
  const float frozenHeight = bodies.GetPosition(box).GetY();

  const talus::deformable::DeformableSolverState* solver =
    runtime.deformableSolver(beam);

  REQUIRE(solver != nullptr);

  const bool evicted = solver->factor.empty() && solver->factor.stored() == 0u
                       && solver->basis.column.empty()
                       && solver->stiffness.tet.empty()
                       && solver->stiffness.hex.empty();

  uint32_t awakeSteps = 0u;
  uint32_t steppedNodes = 0u;
  uint32_t wakes = 0u;
  uint32_t quiescentSteps = 0u;
  float heightDrift = 0.0f;

  for (uint32_t taken = 0u; taken != HOLD; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    awakeSteps += bodies.IsActive(box) ? 1u : 0u;
    steppedNodes += runtime.deformableReport().nodes;
    wakes += runtime.deformableActivity().wakes;
    quiescentSteps += runtime.deformableQuiescent(beam) ? 1u : 0u;
    heightDrift = glm::max(
      heightDrift,
      std::abs(bodies.GetPosition(box).GetY() - frozenHeight)
    );
  }

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);

  std::cout << "deformable: substeps " << runtime.deformableSettings().substeps
            << " spectralRadius " << runtime.deformableSettings().spectralRadius
            << " penetration " << penetration << " voxel " << voxel
            << " beam froze at step " << frozenAt << " box asleep "
            << (asleepAtFreeze ? 1 : 0) << " evicted " << (evicted ? 1 : 0)
            << " held for " << HOLD << " height drift " << heightDrift
            << " awake steps " << awakeSteps << " quiescent steps "
            << quiescentSteps << " stepped nodes " << steppedNodes
            << " uncontacted steps " << uncontactedSteps << reasons << ", "
            << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(frozenAt != 0u);
  REQUIRE(asleepAtFreeze);
  REQUIRE(evicted);
  REQUIRE(uncontactedSteps == 0u);
  REQUIRE(awakeSteps == 0u);
  REQUIRE(wakes == 0u);
  REQUIRE(quiescentSteps == HOLD);
  REQUIRE(steppedNodes == 0u);
  REQUIRE(heightDrift <= 1.0e-4f);
  REQUIRE(penetration < voxel);
  REQUIRE(std::abs(bodies.GetPosition(box).GetX()) < HALF_LENGTH - BOX_HALF);
  REQUIRE(contacts > 0u);
}

namespace {

uint64_t mixBits(uint64_t hash, float value) {
  return (hash ^ std::bit_cast<uint32_t>(value)) * 1099511628211ull;
}

uint64_t mixBits(uint64_t hash, const glm::vec3& value) {
  return mixBits(mixBits(mixBits(hash, value.x), value.y), value.z);
}

uint64_t mixBits(uint64_t hash, const glm::mat3& value) {
  for (int column = 0; column != 3; ++column) {
    hash = mixBits(hash, value[column]);
  }

  return hash;
}

uint64_t mixBits(uint64_t hash, const JPH::Vec3& value) {
  return mixBits(hash, glm::vec3(value.GetX(), value.GetY(), value.GetZ()));
}

uint64_t sharedCrateHash(uint32_t threads) {
  constexpr float BEAM_OFFSET = 0.075f;
  constexpr float LONE_OFFSET = 0.5f;
  constexpr float SHARED_HALF = 0.1f;
  constexpr float SHARED_MASS = 0.3f;
  constexpr float LONE_HALF = 0.05f;
  constexpr float LONE_MASS = 0.15f;
  constexpr float CRATE_AT = 0.3f;
  constexpr uint32_t STEPS = 24u;

  JoltLibrary library;

  World source;
  const Entity nearBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, -BEAM_OFFSET));
  const Entity farBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, BEAM_OFFSET));
  const Entity loneBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, LONE_OFFSET));

  Runtime runtime(source, library);

  prepare(runtime, nearBeam);

  REQUIRE(runtime.world().deformablePrepared(farBeam) != nullptr);
  REQUIRE(runtime.world().deformablePrepared(loneBeam) != nullptr);

  talus::deformable::Settings settings = dynamicSolve(4u);

  settings.stepThreads = threads;
  settings.reportContactDiagnostics = true;

  runtime.setDeformableSettings(settings);

  const Dropped shared = dropCrate(
    runtime,
    glm::vec3(
      CRATE_AT,
      glm::max(
        surfaceAt(runtime, nearBeam, CRATE_AT),
        surfaceAt(runtime, farBeam, CRATE_AT)
      ) + SHARED_HALF
        + 0.001f,
      0.0f
    ),
    SHARED_HALF,
    SHARED_MASS
  );

  const Dropped lone = dropCrate(
    runtime,
    glm::vec3(
      CRATE_AT,
      surfaceAt(runtime, loneBeam, CRATE_AT) + LONE_HALF + 0.001f,
      LONE_OFFSET
    ),
    LONE_HALF,
    LONE_MASS
  );

  uint32_t contacts = 0u;
  std::set<std::pair<uint32_t, uint32_t>> touches;

  for (uint32_t step = 0u; step != STEPS; ++step) {
    runtime.step();

    contacts += runtime.deformableReport().contacts;

    for (const talus::deformable::ContactSolve& solve :
         runtime.deformableReport().contactSolves) {
      for (const talus::deformable::ContactSample& sample : solve.samples) {
        touches.emplace(solve.deformable, sample.partner);
      }
    }
  }

  const auto touched = [&touches](Entity beam, const Dropped& crate) {
    return touches.contains(
      {static_cast<uint32_t>(beam), crate.body.GetIndexAndSequenceNumber()}
    );
  };

  REQUIRE(contacts > 0u);
  REQUIRE_FALSE(runtime.deformableReport().nonFinite);
  REQUIRE(touched(nearBeam, shared));
  REQUIRE(touched(farBeam, shared));
  REQUIRE(touched(loneBeam, lone));
  REQUIRE_FALSE(touched(loneBeam, shared));
  REQUIRE_FALSE(touched(nearBeam, lone));
  REQUIRE_FALSE(touched(farBeam, lone));

  uint64_t hash = 14695981039346656037ull;

  for (const Entity beam : {nearBeam, farBeam, loneBeam}) {
    const DeformableState* state = runtime.world().deformableState(beam);

    REQUIRE(state != nullptr);

    for (const glm::vec3& position : state->position) {
      hash = mixBits(hash, position);
    }

    for (const glm::vec3& velocity : state->velocity) {
      hash = mixBits(hash, velocity);
    }

    for (const talus::objectmodel::PlasticPoint& point : state->plastic) {
      hash = mixBits(mixBits(hash, point.strain), point.equivalent);
    }

    for (const talus::objectmodel::CommittedTet& tet : state->committed) {
      hash = mixBits(hash, tet.rotation);
    }
  }

  const JPH::BodyInterface& bodies = runtime.physics().bodies();

  for (const JPH::BodyID& body : {shared.body, lone.body}) {
    hash = mixBits(hash, JPH::Vec3(bodies.GetPosition(body)));
    hash = mixBits(hash, bodies.GetLinearVelocity(body));
    hash = mixBits(hash, bodies.GetAngularVelocity(body));
  }

  return hash;
}

}

TEST_CASE(
  "two deformables resting on one dynamic crate step bitwise alike on one "
  "and several threads",
  "[deformable]"
) {
  const uint64_t serial = sharedCrateHash(1u);
  const uint64_t threaded = sharedCrateHash(4u);

  std::cout << "shared crate: serial hash " << serial << " threaded hash "
            << threaded << '\n';

  REQUIRE(serial == threaded);
}

TEST_CASE("a deformable co-sleep probe", "[.][cosleep]") {
  constexpr float CRATE_MASS = 0.15f;
  constexpr float CRATE_HALF = 0.03f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const float resting = surfaceAt(runtime, beam, -0.25f);

  const JPH::BodyID crate =
    dropCrate(
      runtime,
      glm::vec3(-0.25f, resting + CRATE_HALF + 0.001f, 0.0f),
      CRATE_HALF,
      CRATE_MASS
    )
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  std::ofstream evidence("deformable-cosleep.txt", std::ios::trunc);

  REQUIRE(evidence);

  constexpr uint32_t STEPS = 900u;

  uint32_t sleptAt = 0u;
  uint32_t contactSteps = 0u;
  uint32_t awakeSteps = 0u;
  uint32_t wakeEvents = 0u;
  uint32_t attempts = 0u;
  uint32_t aborts = 0u;
  uint32_t frozenAt = 0u;
  uint32_t quiescentSteps = 0u;
  uint32_t beamWakes = 0u;
  float sleptHeight = 0.0f;
  float heightDrift = 0.0f;
  bool previous = true;

  WakeReasons reasons;

  for (uint32_t step = 1u; step != STEPS + 1u; ++step) {
    runtime.step();

    attempts += runtime.deformableActivity().armAttempts;
    aborts += runtime.deformableActivity().armAborts;
    beamWakes += runtime.deformableActivity().wakes;

    reasons.add(runtime.deformableActivity());

    if (runtime.deformableActivity().freezes != 0u && frozenAt == 0u) {
      frozenAt = step;
    }

    quiescentSteps += runtime.deformableQuiescent(beam) ? 1u : 0u;

    const bool awake = bodies.IsActive(crate);

    awakeSteps += awake ? 1u : 0u;
    wakeEvents += (awake && !previous) ? 1u : 0u;
    previous = awake;

    if (runtime.deformableReport().contacts > 0u) {
      ++contactSteps;
    }

    if (!awake && sleptAt == 0u) {
      sleptAt = step;
      sleptHeight = bodies.GetPosition(crate).GetY();
    }

    if (sleptAt != 0u) {
      heightDrift = glm::max(
        heightDrift,
        std::abs(bodies.GetPosition(crate).GetY() - sleptHeight)
      );
    }

    if (step % 120u == 0u) {
      const JPH::Vec3 at = bodies.GetPosition(crate);
      const JPH::Vec3 moving = bodies.GetLinearVelocity(crate);

      evidence << "cosleep.step " << step << " crateAwake " << (awake ? 1u : 0u)
               << " crateY " << at.GetY() << " crateSpeed " << moving.Length()
               << " contacts " << runtime.deformableReport().contacts << '\n';
    }
  }

  const JPH::Vec3 settled = bodies.GetPosition(crate);
  const bool awake = bodies.IsActive(crate);

  evidence << "cosleep.result sleptAtStep " << sleptAt << " frozenAtStep "
           << frozenAt << " contactSteps " << contactSteps << " quiescentSteps "
           << quiescentSteps << " beamWakes " << beamWakes << " finalY "
           << settled.GetY() << " finalAwake " << (awake ? 1u : 0u)
           << " armAttempts " << attempts << " armAborts " << aborts
           << " heightDrift " << heightDrift << reasons << '\n';

  std::cout << "cosleep: sleptAtStep " << sleptAt << " of " << STEPS
            << " frozenAtStep " << frozenAt << " contactSteps " << contactSteps
            << " quiescentSteps " << quiescentSteps << " beamWakes "
            << beamWakes << " awakeSteps " << awakeSteps << " wakeEvents "
            << wakeEvents << " finalAwake " << (awake ? 1u : 0u)
            << " armAttempts " << attempts << " armAborts " << aborts
            << " heightDrift " << heightDrift << reasons << '\n';

  REQUIRE(sleptAt != 0u);
  REQUIRE_FALSE(awake);
  REQUIRE(wakeEvents == 0u);
  REQUIRE(awakeSteps == sleptAt - 1u);
  REQUIRE(frozenAt != 0u);
  REQUIRE(contactSteps == frozenAt);
  REQUIRE(quiescentSteps == STEPS - frozenAt + 1u);
  REQUIRE(beamWakes == 0u);
  REQUIRE(heightDrift <= 1.0e-4f);
  REQUIRE(aborts <= 1u);
}

TEST_CASE("a deformable arming holds its crate awake", "[.][cosleep]") {
  constexpr float CRATE_MASS = 0.15f;
  constexpr float CRATE_HALF = 0.03f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  const float resting = surfaceAt(runtime, beam, -0.25f);

  const JPH::BodyID crate =
    dropCrate(
      runtime,
      glm::vec3(-0.25f, resting + CRATE_HALF + 0.001f, 0.0f),
      CRATE_HALF,
      CRATE_MASS
    )
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const JPH::PhysicsSettings& sleeping =
    runtime.physics().system().GetPhysicsSettings();

  const float allowance =
    sleeping.mPointVelocitySleepThreshold * sleeping.mTimeBeforeSleep;
  const uint32_t window = static_cast<uint32_t>(
    std::lround(sleeping.mTimeBeforeSleep / PhysicsWorld::TIME_STEP)
  );

  constexpr uint32_t WINDOWS = 8u;

  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);

  REQUIRE(mesh != nullptr);

  uint32_t attempts = 0u;
  uint32_t asleepSteps = 0u;
  uint32_t sleptAt = 0u;
  uint32_t step = 0u;

  float quietest = std::numeric_limits<float>::max();

  std::vector<glm::vec3> anchor =
    runtime.world().deformableState(beam)->position;

  for (uint32_t taken = 0u; taken != WINDOWS; ++taken) {
    float widest = 0.0f;

    for (uint32_t inside = 0u; inside != window; ++inside) {
      runtime.step();
      ++step;

      attempts += runtime.deformableActivity().armAttempts;

      if (!bodies.IsActive(crate)) {
        ++asleepSteps;

        if (sleptAt == 0u) {
          sleptAt = step;
        }
      }

      const DeformableState* state = runtime.world().deformableState(beam);

      for (size_t node = 0u; node != mesh->anchored.size(); ++node) {
        if (mesh->anchored[node] != 0u) {
          continue;
        }

        widest =
          glm::max(widest, glm::distance(state->position[node], anchor[node]));
      }
    }

    quietest = glm::min(quietest, widest);

    anchor = runtime.world().deformableState(beam)->position;
  }

  std::cout << "arming: steps " << step << " allowance " << allowance
            << " quietestWindowDrift " << quietest << " armAttempts "
            << attempts << " asleepSteps " << asleepSteps << " sleptAtStep "
            << sleptAt << '\n';

  REQUIRE(asleepSteps == 0u);
  REQUIRE(attempts == 0u);
  REQUIRE(quietest > allowance);
}

TEST_CASE("a soft oscillating deformable never freezes", "[.][quiescent]") {
  constexpr float SOFT_YOUNGS = 1.0e4f;
  constexpr float TARGET_SAG = 0.05f;
  constexpr uint32_t STEPS = 1500u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  std::vector<PhysicalMaterial> materials = beamMaterials();

  materials[BULK_LANE].youngsModulus = SOFT_YOUNGS;

  source.setMaterials(materials);

  Runtime runtime(source, library);

  runtime.setDeformableSettings(dynamicSolve(1u));

  prepare(runtime, beam);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);

  std::vector<glm::vec3> load(mesh.restPositions.size(), glm::vec3(0.0f));
  std::vector<uint32_t> tip;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.restPositions[node].x > HALF_LENGTH - 0.06f) {
      tip.push_back(static_cast<uint32_t>(node));
    }
  }

  REQUIRE(!tip.empty());

  const float second = 0.1f * std::pow(0.1f, 3.0f) / 12.0f;
  const float reach = HALF_LENGTH - ANCHOR_END;
  const float total =
    3.0f * SOFT_YOUNGS * second * TARGET_SAG / (reach * reach * reach);

  for (const uint32_t node : tip) {
    load[node] = glm::vec3(0.0f, -total / static_cast<float>(tip.size()), 0.0f);
  }

  runtime.world().setDeformableState(
    beam,
    talus::deformable::solveEquilibrium(
      mesh,
      runtime.world().materials(),
      runtime.world().registry().get<talus::objectmodel::Pose>(beam),
      staticSolve(),
      glm::vec3(0.0f),
      load
    )
  );

  float amplitude = 0.0f;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    amplitude = glm::max(
      amplitude,
      glm::length(
        runtime.world().deformableState(beam)->position[node]
        - mesh.restPositions[node]
      )
    );
  }

  uint32_t attempts = 0u;
  uint32_t confirms = 0u;
  uint32_t refusals = 0u;
  uint32_t freezes = 0u;
  uint32_t quiescentSteps = 0u;
  float closest = std::numeric_limits<float>::max();

  WakeReasons reasons;

  for (uint32_t step = 0u; step != STEPS; ++step) {
    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();

    attempts += activity.armAttempts;
    confirms += activity.confirms;
    refusals += activity.refusals;
    freezes += activity.freezes;
    quiescentSteps += runtime.deformableQuiescent(beam) ? 1u : 0u;

    reasons.add(activity);

    if (activity.confirms != 0u) {
      closest = glm::min(closest, activity.worstCorrection);
    }
  }

  const talus::deformable::Settings& settings = runtime.deformableSettings();

  std::cout << "quiescent: soft beam youngs " << SOFT_YOUNGS << " amplitude "
            << amplitude << " substeps " << settings.substeps
            << " spectralRadius " << settings.spectralRadius << " steps "
            << STEPS << " armAttempts " << attempts << " confirms " << confirms
            << " refusals " << refusals << " freezes " << freezes
            << " closestCorrection " << closest << " threshold "
            << talus::runtime::QUIESCENT_CORRECTION << reasons << '\n';

  REQUIRE(confirms > 0u);
  REQUIRE(refusals == confirms);
  REQUIRE(freezes == 0u);
  REQUIRE(quiescentSteps == 0u);
}

TEST_CASE("a heavy crate needs its load to freeze", "[.][quiescent]") {
  constexpr float CRATE_MASS = 1.5f;
  constexpr float CRATE_HALF = 0.05f;
  constexpr float CRATE_AT = -0.25f;
  constexpr uint32_t REACH = 600u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleUnderCrate(runtime, beam, CRATE_AT, CRATE_HALF, CRATE_MASS);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const float resting = surfaceAt(runtime, beam, CRATE_AT);

  const JPH::BodyID crate =
    dropCrate(
      runtime,
      glm::vec3(CRATE_AT, resting + CRATE_HALF + 0.001f, 0.0f),
      CRATE_HALF,
      CRATE_MASS
    )
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);

  float damping = 0.0f;

  {
    const JPH::BodyLockRead lock(
      runtime.physics().system().GetBodyLockInterface(),
      crate
    );

    REQUIRE(lock.Succeeded());

    damping = lock.GetBody().GetMotionProperties()->GetLinearDamping();
  }

  const auto velocityOf = [&]() {
    const JPH::Vec3 moving = bodies.GetLinearVelocity(crate);

    return glm::vec3(moving.GetX(), moving.GetY(), moving.GetZ());
  };

  std::vector<glm::vec3> velocity{velocityOf()};
  std::vector<glm::vec3> stepLoad{glm::vec3(0.0f)};
  std::vector<uint32_t> awake{bodies.IsActive(crate) ? 1u : 0u};
  std::vector<glm::vec3> windowSum(mesh.restPositions.size(), glm::vec3(0.0f));

  uint32_t frozenAt = 0u;
  uint32_t armedAt = 0u;
  uint32_t windowSteps = 0u;
  uint32_t confirms = 0u;
  uint32_t confirmSteps = 0u;
  float accepted = 0.0f;
  glm::vec3 confirmed(0.0f);

  WakeReasons reasons;

  for (uint32_t step = 1u; step != REACH + 1u && frozenAt == 0u; ++step) {
    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();
    const std::vector<glm::vec3>& load = runtime.deformableReport().contactLoad;

    reasons.add(activity);
    confirms += activity.confirms;

    if (activity.armAttempts != 0u) {
      armedAt = step;
      windowSteps = 0u;

      std::fill(windowSum.begin(), windowSum.end(), glm::vec3(0.0f));
    }

    if (runtime.armedDeformable(beam) || activity.confirms != 0u) {
      for (size_t node = 0u; node != load.size(); ++node) {
        windowSum[node] += load[node];
      }

      ++windowSteps;
    }

    glm::vec3 total(0.0f);

    for (const glm::vec3& force : load) {
      total += force;
    }

    velocity.push_back(velocityOf());
    stepLoad.push_back(total);
    awake.push_back(bodies.IsActive(crate) ? 1u : 0u);

    if (activity.freezes != 0u) {
      frozenAt = step;
      accepted = activity.worstCorrection;
      confirmed = activity.confirmLoad;
      confirmSteps = activity.confirmSteps;
    }
  }

  REQUIRE(frozenAt != 0u);
  REQUIRE(armedAt != 0u);

  uint32_t awakeInWindow = 0u;

  for (uint32_t taken = armedAt - 1u; taken != frozenAt; ++taken) {
    awakeInWindow += awake[taken];
  }

  REQUIRE(frozenAt > armedAt);

  const float step = PhysicsWorld::TIME_STEP;
  const uint32_t window = frozenAt - armedAt + 1u;
  const uint32_t measured = window - 1u;
  const float span = static_cast<float>(measured) * step;
  const glm::vec3 weight = CRATE_MASS * pull();

  glm::vec3 damped(0.0f);

  for (uint32_t taken = armedAt; taken != frozenAt; ++taken) {
    damped += velocity[taken];
  }

  const glm::vec3 change = velocity[frozenAt - 1u] - velocity[armedAt - 1u];

  const glm::vec3 carried =
    confirmed * static_cast<float>(window) - stepLoad[frozenAt];

  const glm::vec3 momentum =
    static_cast<float>(measured) * weight - CRATE_MASS * change / step
    - CRATE_MASS * damping / (1.0f - damping * step) * damped;

  const float residual =
    glm::length(carried - momentum) / static_cast<float>(measured);

  const float bound = runtime.deformableSettings().contactTolerance * CRATE_MASS
                      * PhysicsWorld::GRAVITY;

  std::vector<glm::vec3> averaged(windowSum.size(), glm::vec3(0.0f));

  if (windowSteps != 0u) {
    for (size_t node = 0u; node != averaged.size(); ++node) {
      averaged[node] = windowSum[node] / static_cast<float>(windowSteps);
    }
  }

  const DeformableState& frozen = *runtime.world().deformableState(beam);

  const auto judge = [&](std::span<const glm::vec3> applied) {
    return talus::deformable::checkEquilibrium(
      mesh,
      frozen,
      runtime.world().materials(),
      runtime.deformableSettings(),
      pull(),
      applied
    );
  };

  const talus::deformable::EquilibriumCheck withLoad = judge(averaged);
  const talus::deformable::EquilibriumCheck withoutLoad = judge({});

  const talus::deformable::Settings& settings = runtime.deformableSettings();

  std::cout << "quiescent: heavy crate " << CRATE_MASS << " kg substeps "
            << settings.substeps << " spectralRadius "
            << settings.spectralRadius << " contactTolerance "
            << settings.contactTolerance << " linearDamping " << damping
            << " armed at step " << armedAt << " froze at step " << frozenAt
            << " after " << confirms << " confirms, window " << window
            << " steps, runtime window " << confirmSteps << ", crate awake "
            << awakeInWindow << " of " << window << ", confirm load "
            << -confirmed.y << " N weight "
            << CRATE_MASS * PhysicsWorld::GRAVITY << " N deviation "
            << glm::length(confirmed - weight) << " N, first " << measured
            << " steps carried " << -carried.y / static_cast<float>(measured)
            << " N momentum " << -momentum.y / static_cast<float>(measured)
            << " N residual " << residual << " N bound " << bound
            << " N (velocity change " << glm::length(change) << " m/s over "
            << span << " s), accepted " << accepted << " with load "
            << withLoad.maxCorrection << " without load "
            << withoutLoad.maxCorrection << " crate asleep "
            << (bodies.IsActive(crate) ? 0 : 1) << " threshold "
            << talus::runtime::QUIESCENT_CORRECTION << reasons << '\n';

  REQUIRE(awakeInWindow == window);
  REQUIRE(residual <= bound);
  REQUIRE(confirmSteps == window);
  REQUIRE(windowSteps == window);
  REQUIRE(runtime.deformableQuiescent(beam));
  REQUIRE_FALSE(bodies.IsActive(crate));
  REQUIRE(accepted <= talus::runtime::QUIESCENT_CORRECTION);
  REQUIRE(withLoad.maxCorrection == accepted);
  REQUIRE_FALSE(withLoad.cgCapped);
  REQUIRE_FALSE(withoutLoad.cgCapped);
  REQUIRE(withoutLoad.maxCorrection > talus::runtime::QUIESCENT_CORRECTION);
}

TEST_CASE("a deformable arming holds a sleeping partner", "[.][quiescent]") {
  constexpr float CRATE_MASS = 0.15f;
  constexpr float CRATE_HALF = 0.03f;
  constexpr float SLEEPER_AT = -0.25f;
  constexpr float WAKER_AT = -0.1f;
  constexpr uint32_t REACH = 300u;
  constexpr uint32_t HOLD = 30u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const auto dropAt = [&](float along) {
    const float resting = surfaceAt(runtime, beam, along);

    return dropCrate(
      runtime,
      glm::vec3(along, resting + CRATE_HALF + 0.001f, 0.0f),
      CRATE_HALF,
      CRATE_MASS
    );
  };

  const Dropped sleeper = dropAt(SLEEPER_AT);
  const Dropped waker = dropAt(WAKER_AT);

  {
    const JPH::BodyLockWrite lock(
      runtime.physics().system().GetBodyLockInterface(),
      waker.body
    );

    REQUIRE(lock.Succeeded());

    lock.GetBody().SetAllowSleeping(false);
  }

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);
  const float voxel = glm::compMax(mesh.voxelSize);

  std::vector<uint32_t> footprint;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::vec3& rest = mesh.restPositions[node];

    if (
      std::abs(rest.x - SLEEPER_AT) <= CRATE_HALF + voxel
      && rest.y > HALF_THICKNESS - 0.001f
    ) {
      footprint.push_back(static_cast<uint32_t>(node));
    }
  }

  REQUIRE(!footprint.empty());

  WakeReasons reasons;

  uint32_t reachedAt = 0u;

  for (uint32_t step = 1u; step != REACH + 1u && reachedAt == 0u; ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (runtime.armedDeformable(beam) && !bodies.IsActive(sleeper.body)) {
      reachedAt = step;
    }
  }

  REQUIRE(reachedAt != 0u);

  const float sleptHeight = bodies.GetPosition(sleeper.body).GetY();

  uint32_t armedSteps = 0u;
  uint32_t asleepSteps = 0u;
  uint32_t supportedSteps = 0u;
  uint32_t quiescentSteps = 0u;
  uint32_t wakerAwakeSteps = 0u;
  float heightDrift = 0.0f;

  for (uint32_t taken = 0u; taken != HOLD; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    const std::vector<glm::vec3>& load = runtime.deformableReport().contactLoad;

    glm::vec3 carried(0.0f);

    for (const uint32_t node : footprint) {
      if (node < load.size()) {
        carried += load[node];
      }
    }

    armedSteps += runtime.armedDeformable(beam) ? 1u : 0u;
    asleepSteps += bodies.IsActive(sleeper.body) ? 0u : 1u;
    supportedSteps += carried.y < 0.0f ? 1u : 0u;
    quiescentSteps += runtime.deformableQuiescent(beam) ? 1u : 0u;
    wakerAwakeSteps += bodies.IsActive(waker.body) ? 1u : 0u;
    heightDrift = glm::max(
      heightDrift,
      std::abs(bodies.GetPosition(sleeper.body).GetY() - sleptHeight)
    );
  }

  const talus::deformable::Settings& settings = runtime.deformableSettings();

  std::cout << "quiescent: arming with a sleeping partner substeps "
            << settings.substeps << " spectralRadius "
            << settings.spectralRadius << " reached at step " << reachedAt
            << " held " << HOLD << " armed " << armedSteps << " sleeper asleep "
            << asleepSteps << " supported " << supportedSteps << " quiescent "
            << quiescentSteps << " waker awake " << wakerAwakeSteps
            << " height drift " << heightDrift << reasons << '\n';

  REQUIRE(armedSteps == HOLD);
  REQUIRE(asleepSteps == HOLD);
  REQUIRE(supportedSteps == HOLD);
  REQUIRE(quiescentSteps == 0u);
  REQUIRE(wakerAwakeSteps == HOLD);
  REQUIRE(heightDrift <= 1.0e-4f);

  SECTION("an edit to the sleeper disarms the beam") {
    runtime.setMass(sleeper.entity, 2.0f * CRATE_MASS);
    runtime.step();

    const talus::runtime::DeformableActivity edited =
      runtime.deformableActivity();

    reasons.add(edited);

    const bool armedAfterEdit = runtime.armedDeformable(beam);
    const bool sleeperAwake = bodies.IsActive(sleeper.body);

    std::cout << "quiescent: after the edit armAborts " << edited.armAborts
              << " armed " << (armedAfterEdit ? 1 : 0) << " sleeper awake "
              << (sleeperAwake ? 1 : 0) << reasons << '\n';

    REQUIRE_FALSE(armedAfterEdit);
    REQUIRE(sleeperAwake);
    REQUIRE(edited.editDisarms == 1u);
    REQUIRE(edited.armAborts == 1u);
  }

  SECTION("destroying the armed beam wakes the sleeper") {
    REQUIRE(runtime.destroy(beam));

    runtime.step();

    const talus::runtime::DeformableActivity released =
      runtime.deformableActivity();

    reasons.add(released);

    const bool sleeperAwake = bodies.IsActive(sleeper.body);
    const float sinking = bodies.GetLinearVelocity(sleeper.body).GetY();

    std::cout << "quiescent: after destroying the armed beam sleeper awake "
              << (sleeperAwake ? 1 : 0) << " vertical velocity " << sinking
              << reasons << '\n';

    REQUIRE(sleeperAwake);
    REQUIRE(sinking < 0.0f);
    REQUIRE(released.destroyWakes == 1u);
  }
}

TEST_CASE("a destroyed frozen deformable drops its load", "[.][quiescent]") {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr uint32_t FALL = 10u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenCrate frozen =
    freezeUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS, reasons);

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const float frozenHeight = bodies.GetPosition(frozen.crate.body).GetY();

  REQUIRE(runtime.destroy(beam));

  uint32_t activeSteps = 0u;

  for (uint32_t taken = 0u; taken != FALL; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    activeSteps += bodies.IsActive(frozen.crate.body) ? 1u : 0u;
  }

  const float fell =
    frozenHeight - bodies.GetPosition(frozen.crate.body).GetY();
  const float sinking = bodies.GetLinearVelocity(frozen.crate.body).GetY();

  std::cout << "quiescent: destroyed frozen beam, froze at step "
            << frozen.frozenAt << " substeps "
            << runtime.deformableSettings().substeps << " box active "
            << activeSteps << " of " << FALL << " fell " << fell
            << " vertical velocity " << sinking << reasons << '\n';

  REQUIRE(activeSteps == FALL);
  REQUIRE(sinking < 0.0f);
  REQUIRE(fell > 1.0e-4f);
  REQUIRE(reasons.destroy == 1u);
}

TEST_CASE(
  "a destroyed deformable wakes the box its refusal left asleep",
  "[.][quiescent]"
) {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr uint32_t REACH = 300u;
  constexpr uint32_t FALL = 10u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  settleAtEquilibrium(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const float resting = surfaceAt(runtime, beam, BOX_AT);

  const Dropped box = dropCrate(
    runtime,
    glm::vec3(BOX_AT, resting + BOX_HALF + 0.001f, 0.0f),
    BOX_HALF,
    BOX_MASS
  );

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  WakeReasons reasons;

  uint32_t refusedAt = 0u;

  for (uint32_t step = 1u; step != REACH + 1u && refusedAt == 0u; ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (runtime.deformableActivity().refusals != 0u) {
      refusedAt = step;
    }
  }

  REQUIRE(refusedAt != 0u);
  REQUIRE_FALSE(runtime.armedDeformable(beam));
  REQUIRE_FALSE(runtime.deformableQuiescent(beam));
  REQUIRE_FALSE(bodies.IsActive(box.body));

  const float refusedHeight = bodies.GetPosition(box.body).GetY();

  REQUIRE(runtime.destroy(beam));

  uint32_t activeSteps = 0u;

  for (uint32_t taken = 0u; taken != FALL; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    activeSteps += bodies.IsActive(box.body) ? 1u : 0u;
  }

  const float fell = refusedHeight - bodies.GetPosition(box.body).GetY();
  const float sinking = bodies.GetLinearVelocity(box.body).GetY();

  std::cout << "quiescent: destroyed beam after a refusal, refused at step "
            << refusedAt << " substeps "
            << runtime.deformableSettings().substeps << " box active "
            << activeSteps << " of " << FALL << " fell " << fell
            << " vertical velocity " << sinking << reasons << '\n';

  REQUIRE(activeSteps == FALL);
  REQUIRE(sinking < 0.0f);
  REQUIRE(fell > 1.0e-4f);
  REQUIRE(reasons.destroy == 1u);
}

TEST_CASE(
  "a frozen deformable wakes on an edit and on a mover",
  "[.][quiescent]"
) {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr float MOVER_MASS = 0.15f;
  constexpr float MOVER_HALF = 0.03f;
  constexpr float MOVER_AT = -0.25f;
  constexpr float MOVER_LIFT = 0.25f;
  constexpr uint32_t REACH = 120u;
  constexpr uint32_t RESUME = 30u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const Dropped mover = dropCrate(
    runtime,
    glm::vec3(
      MOVER_AT,
      surfaceAt(runtime, beam, MOVER_AT) + MOVER_LIFT + MOVER_HALF,
      0.0f
    ),
    MOVER_HALF,
    MOVER_MASS
  );

  bodies.SetGravityFactor(mover.body, 0.0f);

  WakeReasons reasons;

  const FrozenCrate frozen =
    freezeUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS, reasons);

  REQUIRE_FALSE(bodies.IsActive(mover.body));

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);

  uint32_t steppedSteps = 0u;
  uint32_t contactSteps = 0u;
  bool nonFinite = false;
  float penetration = 0.0f;

  const auto resume = [&]() {
    for (uint32_t taken = 0u; taken != RESUME; ++taken) {
      runtime.step();

      reasons.add(runtime.deformableActivity());

      const talus::deformable::StepReport& report = runtime.deformableReport();

      steppedSteps += report.nodes != 0u ? 1u : 0u;
      contactSteps += report.nodes != 0u && report.contacts > 0u ? 1u : 0u;
      nonFinite = nonFinite || report.nonFinite;
      penetration = glm::max(penetration, report.maxPenetration);
    }
  };

  const talus::deformable::Settings& settings = runtime.deformableSettings();

  SECTION("an edit wakes it") {
    runtime.setMass(frozen.crate.entity, 2.0f * BOX_MASS);
    runtime.step();

    const talus::runtime::DeformableActivity woke =
      runtime.deformableActivity();

    reasons.add(woke);

    const uint32_t wakingNodes = runtime.deformableReport().nodes;
    const bool rebuilt = !runtime.deformableSolver(beam)->factor.empty();

    resume();

    std::cout << "quiescent: edit wake substeps " << settings.substeps
              << " froze at step " << frozen.frozenAt << " woke "
              << woke.editWakes << " waking nodes " << wakingNodes
              << " rebuilt " << (rebuilt ? 1 : 0) << " resumed " << steppedSteps
              << " of " << RESUME << " contact steps " << contactSteps
              << " penetration " << penetration << " non-finite "
              << (nonFinite ? 1 : 0) << reasons << '\n';

    REQUIRE(woke.editWakes == 1u);
    REQUIRE(woke.wakes == 1u);
    REQUIRE(wakingNodes != 0u);
    REQUIRE(rebuilt);
    REQUIRE(steppedSteps == RESUME);
    REQUIRE(contactSteps == steppedSteps);
    REQUIRE_FALSE(nonFinite);
    REQUIRE(penetration < voxel);
    REQUIRE(
      std::abs(bodies.GetPosition(frozen.crate.body).GetX())
      < HALF_LENGTH - BOX_HALF
    );
  }

  SECTION("a mover wakes it") {
    bodies.SetGravityFactor(mover.body, 1.0f);
    bodies.ActivateBody(mover.body);

    uint32_t wokeAt = 0u;
    uint32_t moverWakes = 0u;
    uint32_t editWakes = 0u;
    uint32_t wakingNodes = 0u;
    bool rebuilt = false;
    float clearance = 0.0f;

    for (uint32_t step = 1u; step != REACH + 1u && wokeAt == 0u; ++step) {
      const float bottom = bodies.GetPosition(mover.body).GetY() - MOVER_HALF;
      const float top = surfaceAt(runtime, beam, MOVER_AT);

      runtime.step();

      const talus::runtime::DeformableActivity& activity =
        runtime.deformableActivity();

      reasons.add(activity);

      if (activity.wakes != 0u) {
        wokeAt = step;
        moverWakes = activity.moverWakes;
        editWakes = activity.editWakes;
        wakingNodes = runtime.deformableReport().nodes;
        rebuilt = !runtime.deformableSolver(beam)->factor.empty();
        clearance = bottom - top;
      }
    }

    resume();

    const float landed = bodies.GetPosition(mover.body).GetY() - MOVER_HALF
                         - surfaceAt(runtime, beam, MOVER_AT);

    std::cout << "quiescent: mover wake substeps " << settings.substeps
              << " froze at step " << frozen.frozenAt << " woke at step "
              << wokeAt << " mover " << moverWakes << " edit " << editWakes
              << " clearance " << clearance << " waking nodes " << wakingNodes
              << " rebuilt " << (rebuilt ? 1 : 0) << " resumed " << steppedSteps
              << " of " << RESUME << " contact steps " << contactSteps
              << " penetration " << penetration << " landed gap " << landed
              << " non-finite " << (nonFinite ? 1 : 0) << reasons << '\n';

    REQUIRE(wokeAt != 0u);
    REQUIRE(moverWakes == 1u);
    REQUIRE(editWakes == 0u);
    REQUIRE(clearance > 0.0f);
    REQUIRE(wakingNodes != 0u);
    REQUIRE(rebuilt);
    REQUIRE(steppedSteps == RESUME);
    REQUIRE(contactSteps == steppedSteps);
    REQUIRE_FALSE(nonFinite);
    REQUIRE(penetration < voxel);
    REQUIRE(landed > -voxel);
    REQUIRE(std::abs(bodies.GetPosition(mover.body).GetX()) < HALF_LENGTH);
    REQUIRE(
      std::abs(bodies.GetPosition(frozen.crate.body).GetX())
      < HALF_LENGTH - BOX_HALF
    );
  }
}

namespace {

constexpr float STACK_HALF = 0.05f;
constexpr float STACK_MASS = 0.15f;
constexpr float STACK_GAP = 0.01f;
constexpr float STACK_AT = 0.3f;
constexpr float BESIDE_AT = 0.18f;
constexpr float CAP_GAP = 0.01f;
constexpr float CAP_SHIFT = 0.03f;

struct FrozenStack {
  Dropped lower;
  Dropped upper;
  Dropped beside;
  Entity cap = talus::objectmodel::INVALID_ENTITY;
  uint32_t frozenAt = 0u;
};

float heightOf(Runtime& runtime, const Dropped& crate) {
  return runtime.physics().bodies().GetPosition(crate.body).GetY();
}

FrozenStack freezeUnderStack(
  Runtime& runtime,
  Entity beam,
  WakeReasons& reasons
) {
  constexpr uint32_t SEAT = 30u;
  constexpr uint32_t REACH = 600u;

  const std::array<CrateLoad, 2> crates{
    {CrateLoad{STACK_AT, 0.0f, STACK_HALF, STACK_MASS},
     CrateLoad{BESIDE_AT, 0.0f, STACK_HALF, STACK_MASS}}
  };

  settleUnderCrates(runtime, beam, crates);

  const auto onBeam = [&](float along) {
    return dropCrate(
      runtime,
      glm::vec3(
        along,
        surfaceAt(runtime, beam, along) + STACK_HALF + 0.001f,
        0.0f
      ),
      STACK_HALF,
      STACK_MASS
    );
  };

  FrozenStack stack;

  stack.lower = onBeam(STACK_AT);
  stack.beside = onBeam(BESIDE_AT);

  stack.upper = dropCrate(
    runtime,
    glm::vec3(
      STACK_AT,
      heightOf(runtime, stack.lower) + 2.0f * STACK_HALF + STACK_GAP,
      0.0f
    ),
    STACK_HALF,
    STACK_MASS
  );

  runtime.physics().bodies().SetGravityFactor(stack.upper.body, 0.0f);

  stack.cap = fieldBlocker(
    runtime.world(),
    glm::vec3(
      BESIDE_AT - CAP_SHIFT,
      heightOf(runtime, stack.beside) + STACK_HALF + CAP_GAP + BLOCKER_HALF,
      0.0f
    )
  );

  runtime.refreshBody(stack.cap);

  REQUIRE(
    runtime.world().registry().all_of<talus::objectmodel::BodyId>(stack.cap)
  );

  uint32_t seatedFreezes = 0u;

  for (uint32_t taken = 0u; taken != SEAT; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    seatedFreezes += runtime.deformableActivity().freezes;
  }

  REQUIRE(seatedFreezes == 0u);

  for (uint32_t taken = 1u; taken != REACH + 1u && stack.frozenAt == 0u;
       ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (runtime.deformableActivity().freezes != 0u) {
      stack.frozenAt = taken;
    }
  }

  const JPH::BodyInterface& bodies = runtime.physics().bodies();

  const float stacked = heightOf(runtime, stack.upper)
                        - heightOf(runtime, stack.lower) - 2.0f * STACK_HALF;

  REQUIRE(stack.frozenAt != 0u);
  REQUIRE(runtime.deformableQuiescent(beam));
  REQUIRE_FALSE(bodies.IsActive(stack.lower.body));
  REQUIRE_FALSE(bodies.IsActive(stack.upper.body));
  REQUIRE_FALSE(bodies.IsActive(stack.beside.body));
  REQUIRE(stacked > 0.0f);
  REQUIRE(
    stacked < runtime.physics()
                .system()
                .GetPhysicsSettings()
                .mSpeculativeContactDistance
  );

  return stack;
}

float clearanceOver(
  Runtime& runtime,
  Entity beam,
  const Dropped& crate,
  float along
) {
  return heightOf(runtime, crate) - STACK_HALF
         - surfaceAt(runtime, beam, along);
}

}

TEST_CASE(
  "a frozen deformable wakes when a crate it holds is destroyed",
  "[.][quiescent]"
) {
  constexpr uint32_t FREE = 5u;
  constexpr uint32_t SETTLE = 60u;
  constexpr float REFERENCE_AT = 5.0f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenStack stack = freezeUnderStack(runtime, beam, reasons);

  const float upperFrom = heightOf(runtime, stack.upper);
  const float besideFrom = heightOf(runtime, stack.beside);

  const Dropped reference = dropCrate(
    runtime,
    glm::vec3(REFERENCE_AT, upperFrom, 0.0f),
    STACK_HALF,
    STACK_MASS
  );

  runtime.physics().bodies().SetGravityFactor(stack.upper.body, 1.0f);

  const bool upperStillAsleep =
    !runtime.physics().bodies().IsActive(stack.upper.body);

  REQUIRE(runtime.destroy(stack.lower.entity));

  runtime.step();

  const talus::runtime::DeformableActivity woke = runtime.deformableActivity();
  const bool frozenAfter = runtime.deformableQuiescent(beam);

  reasons.add(woke);

  float fallError = 0.0f;
  float upperDrop = 0.0f;
  float referenceDrop = 0.0f;
  float besideShift = 0.0f;
  float besideGapLow = std::numeric_limits<float>::max();
  float besideGapHigh = std::numeric_limits<float>::lowest();

  const auto observe = [&]() {
    besideShift = glm::max(
      besideShift,
      std::abs(heightOf(runtime, stack.beside) - besideFrom)
    );

    const float gap = clearanceOver(runtime, beam, stack.beside, BESIDE_AT);

    besideGapLow = glm::min(besideGapLow, gap);
    besideGapHigh = glm::max(besideGapHigh, gap);
  };

  const auto compareFall = [&]() {
    upperDrop = upperFrom - heightOf(runtime, stack.upper);
    referenceDrop = upperFrom - heightOf(runtime, reference);
    fallError = glm::max(fallError, std::abs(upperDrop - referenceDrop));
  };

  observe();
  compareFall();

  for (uint32_t taken = 1u; taken != FREE; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    observe();
    compareFall();
  }

  for (uint32_t taken = 0u; taken != SETTLE; ++taken) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    observe();
  }

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);
  const float landed = clearanceOver(runtime, beam, stack.upper, STACK_AT);

  std::cout << "quiescent: destroyed a held crate, froze at step "
            << stack.frozenAt << " woke " << woke.wakes << " destroy "
            << woke.destroyWakes << " frozen after " << (frozenAfter ? 1 : 0)
            << " upper drop " << upperDrop << " reference drop "
            << referenceDrop << " over " << FREE << " steps, fall error "
            << fallError << " upper landed gap " << landed << " beside shift "
            << besideShift << " beside gap " << besideGapLow << " to "
            << besideGapHigh << " voxel " << voxel << reasons << '\n';

  REQUIRE(upperStillAsleep);
  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.destroyWakes == 1u);
  REQUIRE_FALSE(frozenAfter);
  REQUIRE(referenceDrop > 0.0f);
  REQUIRE(fallError <= 1.0e-5f);
  REQUIRE(std::abs(landed) < voxel);
  REQUIRE(besideGapLow > 0.0f);
  REQUIRE(besideGapHigh < voxel);
  REQUIRE(
    std::abs(runtime.physics().bodies().GetPosition(stack.beside.body).GetX())
    < HALF_LENGTH - STACK_HALF
  );
}

TEST_CASE(
  "a frozen deformable wakes when the top of its stack is removed",
  "[.][quiescent]"
) {
  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenStack stack = freezeUnderStack(runtime, beam, reasons);

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);
  const float clearance = clearanceOver(runtime, beam, stack.upper, STACK_AT);

  REQUIRE(runtime.destroy(stack.upper.entity));

  runtime.step();

  const talus::runtime::DeformableActivity woke = runtime.deformableActivity();
  const bool frozenAfter = runtime.deformableQuiescent(beam);

  reasons.add(woke);

  std::cout << "quiescent: removed the top of the stack, froze at step "
            << stack.frozenAt << " top clearance over the beam " << clearance
            << " voxel " << voxel << " woke " << woke.wakes << " destroy "
            << woke.destroyWakes << " frozen after " << (frozenAfter ? 1 : 0)
            << reasons << '\n';

  REQUIRE(clearance > voxel);
  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.destroyWakes == 1u);
  REQUIRE_FALSE(frozenAfter);
}

TEST_CASE(
  "a frozen deformable wakes at the step after a paused teleport of its load",
  "[.][quiescent]"
) {
  constexpr uint32_t PAUSED = 3u;
  constexpr float AWAY = 5.0f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenStack stack = freezeUnderStack(runtime, beam, reasons);

  runtime.world().patch<talus::objectmodel::Pose>(
    stack.upper.entity,
    [](talus::objectmodel::Pose& pose) {
      pose.position = glm::vec3(AWAY, AWAY, 0.0f);
    }
  );

  for (uint32_t frame = 0u; frame != PAUSED; ++frame) {
    runtime.applyPoseEdits();
    runtime.syncPoses(1.0f);
  }

  const bool frozenWhilePaused = runtime.deformableQuiescent(beam);
  const float movedTo =
    runtime.physics().bodies().GetPosition(stack.upper.body).GetX();

  runtime.step();

  const talus::runtime::DeformableActivity woke = runtime.deformableActivity();
  const bool frozenAfter = runtime.deformableQuiescent(beam);

  reasons.add(woke);

  std::cout << "quiescent: teleported a held crate over " << PAUSED
            << " paused frames, froze at step " << stack.frozenAt
            << " frozen while paused " << (frozenWhilePaused ? 1 : 0)
            << " moved to x " << movedTo << " woke " << woke.wakes << " edit "
            << woke.editWakes << " frozen after " << (frozenAfter ? 1 : 0)
            << reasons << '\n';

  REQUIRE(frozenWhilePaused);
  REQUIRE(movedTo > AWAY - 1.0f);
  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.editWakes == 1u);
  REQUIRE_FALSE(frozenAfter);
}

TEST_CASE(
  "a frozen deformable wakes when a static touching its load is destroyed",
  "[.][quiescent]"
) {
  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenStack stack = freezeUnderStack(runtime, beam, reasons);

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);
  const float capBottom = runtime.world()
                            .registry()
                            .get<talus::objectmodel::Pose>(stack.cap)
                            .position.y
                          - BLOCKER_HALF;
  const float capClearance =
    capBottom - surfaceAt(runtime, beam, BESIDE_AT - CAP_SHIFT);
  const float capGap = capBottom - heightOf(runtime, stack.beside) - STACK_HALF;

  REQUIRE(runtime.destroy(stack.cap));

  runtime.step();

  const talus::runtime::DeformableActivity woke = runtime.deformableActivity();
  const bool frozenAfter = runtime.deformableQuiescent(beam);

  reasons.add(woke);

  std::cout << "quiescent: destroyed a static over the load, froze at step "
            << stack.frozenAt << " static clearance over the beam "
            << capClearance << " gap to the crate " << capGap << " voxel "
            << voxel << " woke " << woke.wakes << " destroy "
            << woke.destroyWakes << " frozen after " << (frozenAfter ? 1 : 0)
            << reasons << '\n';

  REQUIRE(capClearance > voxel);
  REQUIRE(capGap > 0.0f);
  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.destroyWakes == 1u);
  REQUIRE_FALSE(frozenAfter);
}

TEST_CASE(
  "a load activated inside a physics update wakes its deformable",
  "[.][quiescent]"
) {
  constexpr float STRIKER_HALF = 0.02f;
  constexpr float STRIKER_MASS = 0.05f;
  constexpr uint32_t REACH = 30u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenStack stack = freezeUnderStack(runtime, beam, reasons);

  const JPH::BodyInterface& bodies = runtime.physics().bodies();

  const Dropped striker = dropCrate(
    runtime,
    glm::vec3(
      STACK_AT,
      heightOf(runtime, stack.upper) + STACK_HALF + STRIKER_HALF + 0.002f,
      0.0f
    ),
    STRIKER_HALF,
    STRIKER_MASS
  );

  uint32_t activatedAt = 0u;
  uint32_t wokeAt = 0u;
  bool frozenThroughActivation = false;
  talus::runtime::DeformableActivity woke;

  for (uint32_t step = 1u; step != REACH + 1u && wokeAt == 0u; ++step) {
    const bool asleep = !bodies.IsActive(stack.upper.body);

    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();

    reasons.add(activity);

    if (activatedAt == 0u && asleep && bodies.IsActive(stack.upper.body)) {
      activatedAt = step;
      frozenThroughActivation = runtime.deformableQuiescent(beam);
    }

    if (activity.wakes != 0u) {
      wokeAt = step;
      woke = activity;
    }
  }

  std::cout << "quiescent: a striker on the stack, froze at step "
            << stack.frozenAt << " striker active "
            << (bodies.IsActive(striker.body) ? 1 : 0)
            << " upper activated in update " << activatedAt
            << " frozen through it " << (frozenThroughActivation ? 1 : 0)
            << " woke at step " << wokeAt << " wakes " << woke.wakes
            << " activation " << woke.activationWakes << " mover "
            << woke.moverWakes << reasons << '\n';

  REQUIRE(activatedAt != 0u);
  REQUIRE(frozenThroughActivation);
  REQUIRE(wokeAt == activatedAt + 1u);
  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.activationWakes == 1u);
}

TEST_CASE(
  "an edit outside every loading island wakes and disarms nothing",
  "[.][quiescent]"
) {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr float FAR = 3.0f;
  constexpr uint32_t REACH = 300u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  settleUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS);

  const Dropped crate = dropCrate(
    runtime,
    glm::vec3(
      BOX_AT,
      surfaceAt(runtime, beam, BOX_AT) + BOX_HALF + 0.001f,
      0.0f
    ),
    BOX_HALF,
    BOX_MASS
  );

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const auto hang = [&](float height) {
    const Dropped hung =
      dropCrate(runtime, glm::vec3(FAR, height, 0.0f), BOX_HALF, BOX_MASS);

    bodies.SetGravityFactor(hung.body, 0.0f);

    return hung;
  };

  const Dropped edited = hang(1.0f);
  const Dropped armedDoomed = hang(2.0f);
  const Dropped frozenDoomed = hang(3.0f);

  uint32_t created = 0u;

  const auto editElsewhere = [&](const Dropped& doomed) {
    runtime.setMass(edited.entity, 2.0f * BOX_MASS);
    runtime.setLinearVelocity(edited.entity, glm::vec3(0.0f, 0.0f, 0.1f));

    runtime.world().patch<talus::objectmodel::Pose>(
      edited.entity,
      [](talus::objectmodel::Pose& pose) {
        pose.position = glm::vec3(FAR, 1.5f, 0.0f);
      }
    );

    runtime.applyPoseEdits();
    runtime.refreshBody(edited.entity);

    static_cast<void>(dropCrate(
      runtime,
      glm::vec3(-FAR, 1.0f + static_cast<float>(created), 0.0f),
      BOX_HALF,
      BOX_MASS
    ));

    ++created;

    REQUIRE(runtime.destroy(doomed.entity));
  };

  WakeReasons reasons;

  uint32_t armedAt = 0u;

  for (uint32_t step = 1u; step != REACH + 1u && armedAt == 0u; ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (runtime.armedDeformable(beam)) {
      armedAt = step;
    }
  }

  REQUIRE(armedAt != 0u);

  editElsewhere(armedDoomed);

  runtime.step();

  const talus::runtime::DeformableActivity whileArmed =
    runtime.deformableActivity();
  const bool heldArming =
    runtime.armedDeformable(beam) || runtime.deformableQuiescent(beam);

  reasons.add(whileArmed);

  uint32_t frozenAt = 0u;

  for (uint32_t step = 1u;
       step != REACH + 1u && !runtime.deformableQuiescent(beam);
       ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    frozenAt = step;
  }

  REQUIRE(runtime.deformableQuiescent(beam));
  REQUIRE_FALSE(bodies.IsActive(crate.body));

  editElsewhere(frozenDoomed);

  runtime.step();

  const talus::runtime::DeformableActivity whileFrozen =
    runtime.deformableActivity();
  const bool heldFrozen = runtime.deformableQuiescent(beam);

  reasons.add(whileFrozen);

  std::cout << "quiescent: edits outside the island, armed at step " << armedAt
            << " disarms " << whileArmed.editDisarms << " aborts "
            << whileArmed.armAborts << " held arming " << (heldArming ? 1 : 0)
            << ", froze " << frozenAt << " steps later, wakes "
            << whileFrozen.wakes << " disarms " << whileFrozen.editDisarms
            << " held frozen " << (heldFrozen ? 1 : 0) << " edited body active "
            << (bodies.IsActive(edited.body) ? 1 : 0) << reasons << '\n';

  REQUIRE(whileArmed.editDisarms == 0u);
  REQUIRE(whileArmed.armAborts == 0u);
  REQUIRE(heldArming);
  REQUIRE(whileFrozen.wakes == 0u);
  REQUIRE(whileFrozen.editDisarms == 0u);
  REQUIRE(heldFrozen);
  REQUIRE_FALSE(bodies.IsActive(crate.body));
}

TEST_CASE("a gravity change wakes a frozen deformable", "[.][quiescent]") {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr float HEAVIER = 1.1f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  WakeReasons reasons;

  const FrozenCrate frozen =
    freezeUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS, reasons);

  runtime.physics().system().SetGravity(
    JPH::Vec3(0.0f, -HEAVIER * PhysicsWorld::GRAVITY, 0.0f)
  );

  runtime.step();

  const talus::runtime::DeformableActivity woke = runtime.deformableActivity();
  const bool frozenAfter = runtime.deformableQuiescent(beam);

  reasons.add(woke);

  std::cout << "quiescent: gravity scaled by " << HEAVIER << ", froze at step "
            << frozen.frozenAt << " woke " << woke.wakes << " global "
            << woke.globalWakes << " frozen after " << (frozenAfter ? 1 : 0)
            << reasons << '\n';

  REQUIRE(woke.wakes == 1u);
  REQUIRE(woke.globalWakes == 1u);
  REQUIRE_FALSE(frozenAfter);
}

TEST_CASE(
  "a crate shared by two frozen deformables wakes both",
  "[.][quiescent]"
) {
  constexpr float BEAM_OFFSET = 0.075f;
  constexpr float SHARED_HALF = 0.1f;
  constexpr float SHARED_MASS = 0.3f;
  constexpr float SHARED_AT = 0.3f;
  constexpr float OWN_HALF = 0.03f;
  constexpr float OWN_MASS = 0.15f;
  constexpr float OWN_AT = -0.2f;
  constexpr uint32_t REACH = 900u;

  JoltLibrary library;

  World source;
  const Entity nearBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, -BEAM_OFFSET));
  const Entity farBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, BEAM_OFFSET));

  Runtime runtime(source, library);

  prepare(runtime, nearBeam);

  REQUIRE(runtime.world().deformablePrepared(farBeam) != nullptr);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const std::array<CrateLoad, 2> nearLoads{
    {CrateLoad{SHARED_AT, 0.0f, SHARED_HALF, 0.5f * SHARED_MASS},
     CrateLoad{OWN_AT, -BEAM_OFFSET, OWN_HALF, OWN_MASS}}
  };
  const std::array<CrateLoad, 1> farLoads{
    {CrateLoad{SHARED_AT, 0.0f, SHARED_HALF, 0.5f * SHARED_MASS}}
  };

  settleUnderCrates(runtime, nearBeam, nearLoads);
  settleUnderCrates(runtime, farBeam, farLoads);

  const Dropped shared = dropCrate(
    runtime,
    glm::vec3(
      SHARED_AT,
      glm::max(
        surfaceAt(runtime, nearBeam, SHARED_AT),
        surfaceAt(runtime, farBeam, SHARED_AT)
      ) + SHARED_HALF
        + 0.001f,
      0.0f
    ),
    SHARED_HALF,
    SHARED_MASS
  );

  const Dropped own = dropCrate(
    runtime,
    glm::vec3(
      OWN_AT,
      surfaceAt(runtime, nearBeam, OWN_AT) + OWN_HALF + 0.001f,
      -BEAM_OFFSET
    ),
    OWN_HALF,
    OWN_MASS
  );

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  WakeReasons reasons;

  uint32_t frozenAt = 0u;

  for (uint32_t step = 1u; step != REACH + 1u && frozenAt == 0u; ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (
      runtime.deformableQuiescent(nearBeam)
      && runtime.deformableQuiescent(farBeam)
    ) {
      frozenAt = step;
    }
  }

  std::cout << "quiescent: two beams share a crate, both frozen at step "
            << frozenAt << reasons << '\n';

  REQUIRE(frozenAt != 0u);
  REQUIRE_FALSE(bodies.IsActive(shared.body));
  REQUIRE_FALSE(bodies.IsActive(own.body));

  const auto editAndStep = [&](const Dropped& crate, float mass) {
    runtime.setMass(crate.entity, mass);
    runtime.step();

    const talus::runtime::DeformableActivity woke =
      runtime.deformableActivity();

    reasons.add(woke);

    const bool nearFrozen = runtime.deformableQuiescent(nearBeam);
    const bool farFrozen = runtime.deformableQuiescent(farBeam);

    std::cout << "quiescent: after the edit wakes " << woke.wakes << " edit "
              << woke.editWakes << " activation " << woke.activationWakes
              << " nearBeam frozen " << (nearFrozen ? 1 : 0)
              << " farBeam frozen " << (farFrozen ? 1 : 0) << reasons << '\n';

    REQUIRE(woke.wakes == 2u);
    REQUIRE(woke.editWakes == 2u);
    REQUIRE_FALSE(nearFrozen);
    REQUIRE_FALSE(farFrozen);
  };

  SECTION("an edit to the shared crate wakes both beams") {
    editAndStep(shared, 2.0f * SHARED_MASS);
  }

  SECTION("an edit to one beam's own crate wakes the other through the share") {
    editAndStep(own, 2.0f * OWN_MASS);
  }
}

TEST_CASE(
  "a re-prepared then destroyed deformable leaves no box asleep",
  "[.][quiescent]"
) {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float BOX_AT = 0.3f;
  constexpr uint32_t REACH = 300u;
  constexpr uint32_t FALL = 10u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  WakeReasons reasons;

  const auto dropsAfterRepreparing = [&](const Dropped& box, const char* from) {
    const float held = bodies.GetPosition(box.body).GetY();

    runtime.refreshBody(beam);

    REQUIRE(runtime.destroy(beam));

    uint32_t activeSteps = 0u;

    for (uint32_t taken = 0u; taken != FALL; ++taken) {
      runtime.step();

      reasons.add(runtime.deformableActivity());

      activeSteps += bodies.IsActive(box.body) ? 1u : 0u;
    }

    const float fell = held - bodies.GetPosition(box.body).GetY();
    const float sinking = bodies.GetLinearVelocity(box.body).GetY();

    std::cout << "quiescent: re-prepared then destroyed " << from
              << ", box active " << activeSteps << " of " << FALL << " fell "
              << fell << " vertical velocity " << sinking << reasons << '\n';

    REQUIRE(activeSteps == FALL);
    REQUIRE(sinking < 0.0f);
    REQUIRE(fell > 1.0e-4f);
  };

  SECTION("from quiescent") {
    const FrozenCrate frozen =
      freezeUnderCrate(runtime, beam, BOX_AT, BOX_HALF, BOX_MASS, reasons);

    dropsAfterRepreparing(frozen.crate, "from quiescent");
  }

  SECTION("after a refusal") {
    settleAtEquilibrium(runtime, beam);

    const Dropped box = dropCrate(
      runtime,
      glm::vec3(
        BOX_AT,
        surfaceAt(runtime, beam, BOX_AT) + BOX_HALF + 0.001f,
        0.0f
      ),
      BOX_HALF,
      BOX_MASS
    );

    uint32_t refusedAt = 0u;

    for (uint32_t step = 1u; step != REACH + 1u && refusedAt == 0u; ++step) {
      runtime.step();

      reasons.add(runtime.deformableActivity());

      if (runtime.deformableActivity().refusals != 0u) {
        refusedAt = step;
      }
    }

    REQUIRE(refusedAt != 0u);
    REQUIRE_FALSE(runtime.armedDeformable(beam));
    REQUIRE_FALSE(runtime.deformableQuiescent(beam));
    REQUIRE_FALSE(bodies.IsActive(box.body));

    dropsAfterRepreparing(box, "after a refusal");
  }
}

TEST_CASE(
  "an arming deformable disarms on an edit above its sleeping partner",
  "[.][quiescent]"
) {
  constexpr float CRATE_MASS = 0.15f;
  constexpr float CRATE_HALF = 0.03f;
  constexpr float MIDDLE_HALF = 0.06f;
  constexpr float SLEEPER_AT = -0.25f;
  constexpr float WAKER_AT = -0.1f;
  constexpr float GAP = 0.01f;
  constexpr float AWAY = 5.0f;
  constexpr uint32_t REACH = 300u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  const std::array<CrateLoad, 2> crates{
    {CrateLoad{SLEEPER_AT, 0.0f, CRATE_HALF, CRATE_MASS},
     CrateLoad{WAKER_AT, 0.0f, CRATE_HALF, CRATE_MASS}}
  };

  settleUnderCrates(runtime, beam, crates);

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  const auto onBeam = [&](float along) {
    return dropCrate(
      runtime,
      glm::vec3(
        along,
        surfaceAt(runtime, beam, along) + CRATE_HALF + 0.001f,
        0.0f
      ),
      CRATE_HALF,
      CRATE_MASS
    );
  };

  const auto hover = [&](const Dropped& below, float belowHalf, float half) {
    const Dropped hung = dropCrate(
      runtime,
      glm::vec3(
        SLEEPER_AT,
        heightOf(runtime, below) + belowHalf + GAP + half,
        0.0f
      ),
      half,
      CRATE_MASS
    );

    bodies.SetGravityFactor(hung.body, 0.0f);

    return hung;
  };

  const auto allowSleeping = [&](const Dropped& crate, bool allow) {
    const JPH::BodyLockWrite lock(
      runtime.physics().system().GetBodyLockInterface(),
      crate.body
    );

    REQUIRE(lock.Succeeded());

    lock.GetBody().SetAllowSleeping(allow);
  };

  const Dropped sleeper = onBeam(SLEEPER_AT);
  const Dropped waker = onBeam(WAKER_AT);
  const Dropped middle = hover(sleeper, CRATE_HALF, MIDDLE_HALF);
  const Dropped top = hover(middle, MIDDLE_HALF, CRATE_HALF);

  allowSleeping(waker, false);

  const auto asleep = [&](const Dropped& crate) {
    return !bodies.IsActive(crate.body);
  };

  WakeReasons reasons;

  uint32_t reachedAt = 0u;

  for (uint32_t step = 1u; step != REACH + 1u && reachedAt == 0u; ++step) {
    runtime.step();

    reasons.add(runtime.deformableActivity());

    if (
      runtime.armedDeformable(beam) && asleep(sleeper) && asleep(middle)
      && asleep(top)
    ) {
      reachedAt = step;
    }
  }

  REQUIRE(reachedAt != 0u);

  const JPH::PhysicsSettings& settings =
    runtime.physics().system().GetPhysicsSettings();

  const float voxel =
    glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);
  const float middleGap = heightOf(runtime, middle) - MIDDLE_HALF
                          - heightOf(runtime, sleeper) - CRATE_HALF;
  const float topGap = heightOf(runtime, top) - CRATE_HALF
                       - heightOf(runtime, middle) - MIDDLE_HALF;
  const float middleClearance = heightOf(runtime, middle) - MIDDLE_HALF
                                - surfaceAt(runtime, beam, SLEEPER_AT);
  const auto boundsOf = [&](const Dropped& crate) {
    const JPH::BodyLockRead lock(
      runtime.physics().system().GetBodyLockInterface(),
      crate.body
    );

    REQUIRE(lock.Succeeded());

    return lock.GetBody().GetWorldSpaceBounds();
  };

  const float topOverSleeper =
    boundsOf(top).mMin.GetY() - boundsOf(sleeper).mMax.GetY();
  const float reach =
    settings.mSpeculativeContactDistance + settings.mPenetrationSlop + voxel;

  REQUIRE(middleGap > 0.0f);
  REQUIRE(middleGap < settings.mSpeculativeContactDistance);
  REQUIRE(topGap > 0.0f);
  REQUIRE(topGap < settings.mSpeculativeContactDistance);
  REQUIRE(middleClearance > voxel);
  REQUIRE(topOverSleeper > reach);
  REQUIRE_FALSE(asleep(waker));

  const auto disarmsAfter = [&](const char* edit) {
    runtime.step();

    const talus::runtime::DeformableActivity edited =
      runtime.deformableActivity();

    reasons.add(edited);

    const bool armedAfter = runtime.armedDeformable(beam);
    const bool frozenAfter = runtime.deformableQuiescent(beam);

    allowSleeping(waker, true);

    uint32_t frozenAt = 0u;
    uint32_t rearms = 0u;

    for (uint32_t step = 1u; step != REACH + 1u && frozenAt == 0u; ++step) {
      runtime.step();

      const talus::runtime::DeformableActivity& activity =
        runtime.deformableActivity();

      reasons.add(activity);

      rearms += activity.armAttempts;

      if (activity.freezes != 0u) {
        frozenAt = step;
      }
    }

    std::cout << "quiescent: " << edit << " over a sleeping partner, armed at "
              << reachedAt << " stacked gaps " << middleGap << " and " << topGap
              << " clearance " << middleClearance << " top over the sleeper "
              << topOverSleeper << " disarms " << edited.editDisarms
              << " aborts " << edited.armAborts << " armed after "
              << (armedAfter ? 1 : 0) << " frozen after "
              << (frozenAfter ? 1 : 0) << ", waker released, froze at "
              << frozenAt << " after " << rearms << " arm attempts" << reasons
              << '\n';

    REQUIRE(edited.editDisarms == 1u);
    REQUIRE(edited.armAborts == 1u);
    REQUIRE_FALSE(armedAfter);
    REQUIRE_FALSE(frozenAfter);
    REQUIRE((frozenAt == 0u || rearms != 0u));
  };

  SECTION("destroying the stacked crate") {
    REQUIRE(runtime.destroy(middle.entity));

    disarmsAfter("destroyed the stacked crate");
  }

  SECTION("teleporting the stacked crate away") {
    runtime.world().patch<talus::objectmodel::Pose>(
      middle.entity,
      [](talus::objectmodel::Pose& pose) {
        pose.position = glm::vec3(AWAY, AWAY, 0.0f);
      }
    );

    runtime.applyPoseEdits();

    REQUIRE(bodies.GetPosition(middle.body).GetX() > AWAY - 1.0f);

    disarmsAfter("teleported the stacked crate");
  }

  SECTION("changing the stacked crate's mass") {
    runtime.setMass(middle.entity, 2.0f * CRATE_MASS);

    disarmsAfter("changed the stacked crate's mass");
  }

  SECTION("destroying the crate two up the stack") {
    REQUIRE(runtime.destroy(top.entity));

    disarmsAfter("destroyed the crate two up");
  }
}

namespace {

constexpr float ELEMENT_HALF = 0.0125f;
constexpr double BALANCE_LIMIT = 1.0e-6;

constexpr std::array<std::array<uint32_t, 4>, 6> KUHN_TETS{
  {{0u, 1u, 3u, 7u},
   {0u, 3u, 2u, 7u},
   {0u, 2u, 6u, 7u},
   {0u, 6u, 4u, 7u},
   {0u, 4u, 5u, 7u},
   {0u, 5u, 1u, 7u}}
};

constexpr float SPIN_HALF_LENGTH = 0.075f;
constexpr float SPIN_HALF_THICKNESS = 0.025f;

struct ElementBalance {
  double force = 0.0;
  double torque = 0.0;
};

std::array<glm::vec3, 8> restCube() {
  std::array<glm::vec3, 8> corners{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    corners[corner] = ELEMENT_HALF
                      * glm::vec3(
                        (corner & 1u) != 0u ? 1.0f : -1.0f,
                        (corner & 2u) != 0u ? 1.0f : -1.0f,
                        (corner & 4u) != 0u ? 1.0f : -1.0f
                      );
  }

  return corners;
}

std::array<glm::vec3, 8> displacedCube(const std::array<glm::vec3, 8>& rest) {
  const glm::dmat3 rotation = glm::mat3_cast(
    glm::angleAxis(0.7, glm::normalize(glm::dvec3(1.0, 2.0, 3.0)))
  );
  const glm::dmat3 strain(1.0, 0.3, -0.2, 0.3, -0.5, 0.4, -0.2, 0.4, 0.7);
  const glm::dmat3 deformation = rotation * (glm::dmat3(1.0) + 1.0e-3 * strain);
  const glm::dvec3 shift = 0.18 * glm::normalize(glm::dvec3(1.0));

  std::array<glm::vec3, 8> placed{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    placed[corner] = glm::vec3(deformation * glm::dvec3(rest[corner]) + shift);
  }

  return placed;
}

talus::objectmodel::PreparedHex cubeHex() {
  const glm::vec3 voxel(2.0f * ELEMENT_HALF);

  talus::objectmodel::PreparedHex hex;

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    hex.nodes[corner] = corner;
  }

  hex.halfExtents = 0.5f * voxel;
  hex.volume = voxel.x * voxel.y * voxel.z;
  hex.material = BULK_LANE;

  return hex;
}

talus::objectmodel::PreparedTet kuhnTet(
  const std::array<uint32_t, 4>& pick,
  const std::array<glm::vec3, 8>& rest
) {
  talus::objectmodel::PreparedTet tet;

  tet.nodes = glm::uvec4(pick[0], pick[1], pick[2], pick[3]);
  tet.material = BULK_LANE;

  const glm::vec3& first = rest[tet.nodes[0]];

  glm::mat3 shape(
    rest[tet.nodes[1]] - first,
    rest[tet.nodes[2]] - first,
    rest[tet.nodes[3]] - first
  );

  if (glm::determinant(shape) < 0.0f) {
    std::swap(tet.nodes[1], tet.nodes[2]);

    shape = glm::mat3(
      rest[tet.nodes[1]] - first,
      rest[tet.nodes[2]] - first,
      rest[tet.nodes[3]] - first
    );
  }

  tet.volume = glm::determinant(shape) / 6.0f;
  tet.restInverse = glm::inverse(shape);

  return tet;
}

template<size_t Corners>
ElementBalance balanceOf(
  const std::array<glm::vec3, Corners>& at,
  const std::array<glm::vec3, Corners>& force
) {
  glm::dvec3 centre(0.0);

  for (const glm::vec3& corner : at) {
    centre += glm::dvec3(corner);
  }

  centre /= static_cast<double>(Corners);

  glm::dvec3 net(0.0);
  glm::dvec3 torque(0.0);
  double forceScale = 0.0;
  double torqueScale = 0.0;

  for (size_t corner = 0u; corner != Corners; ++corner) {
    const glm::dvec3 arm = glm::dvec3(at[corner]) - centre;
    const glm::dvec3 pushed(force[corner]);

    net += pushed;
    torque += glm::cross(arm, pushed);
    forceScale += glm::length(pushed);
    torqueScale += glm::length(arm) * glm::length(pushed);
  }

  return {glm::length(net) / forceScale, glm::length(torque) / torqueScale};
}

Entity spinningBar(World& world) {
  const float skin = 0.2f * BASE_VOXEL;

  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(
    static_cast<uint32_t>(std::lround(2.0f * SPIN_HALF_LENGTH / BASE_VOXEL))
      + 3u,
    static_cast<uint32_t>(std::lround(2.0f * SPIN_HALF_THICKNESS / BASE_VOXEL))
      + 3u,
    static_cast<uint32_t>(std::lround(2.0f * SPIN_HALF_THICKNESS / BASE_VOXEL))
      + 3u
  );

  const Entity entity = world.createSdfGrid(talus::objectmodel::Pose{}, grid);

  const glm::vec3 half(
    SPIN_HALF_LENGTH + skin,
    SPIN_HALF_THICKNESS + skin,
    SPIN_HALF_THICKNESS + skin
  );

  REQUIRE(world.addPrimitive(entity, spanning(-half, half, BULK_LANE)));

  const glm::vec3 minimum(
    -SPIN_HALF_LENGTH - BASE_VOXEL,
    -SPIN_HALF_THICKNESS - BASE_VOXEL,
    -SPIN_HALF_THICKNESS - BASE_VOXEL
  );

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      minimum,
      -minimum,
      grid.resolution
    )
  );

  world.setMaterials(beamMaterials());

  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{}
  );

  return entity;
}

bool finiteState(const DeformableState& state) {
  const auto finite = [](const glm::vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
           && std::isfinite(value.z);
  };

  return std::all_of(state.position.begin(), state.position.end(), finite)
         && std::all_of(state.velocity.begin(), state.velocity.end(), finite);
}

double kineticEnergy(const PreparedMesh& mesh, const DeformableState& state) {
  double kinetic = 0.0;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const glm::dvec3 moving(state.velocity[node]);

    kinetic +=
      0.5 * static_cast<double>(mesh.nodeMass[node]) * glm::dot(moving, moving);
  }

  return kinetic;
}

}

TEST_CASE(
  "a deformable element force balances away from its rest placement",
  "[deformable]"
) {
  const std::vector<PhysicalMaterial> materials = beamMaterials();
  const PhysicalMaterial& lane = materials[BULK_LANE];

  const talus::deformable::ElasticParams params{
    lane.youngsModulus,
    lane.poissonsRatio,
    lane.density
  };

  const std::array<glm::vec3, 8> rest = restCube();
  const std::array<glm::vec3, 8> placed = displacedCube(rest);

  std::vector<ElementBalance> balances;

  const talus::objectmodel::PreparedHex hex = cubeHex();

  const talus::deformable::HexTrial hexed = talus::deformable::hexTrial(
    placed,
    rest,
    hex,
    talus::deformable::hexStiffness(hex, params)
  );

  balances.push_back(balanceOf(placed, hexed.force));

  std::cout << "elementbalance: hex netForce " << balances.back().force
            << " netTorque " << balances.back().torque << '\n';

  for (size_t index = 0u; index != KUHN_TETS.size(); ++index) {
    const talus::objectmodel::PreparedTet tet = kuhnTet(KUHN_TETS[index], rest);

    REQUIRE(tet.volume > 0.0f);

    std::array<glm::vec3, 4> deformed{};
    std::array<glm::vec3, 4> resting{};

    for (int corner = 0; corner != 4; ++corner) {
      deformed[static_cast<size_t>(corner)] = placed[tet.nodes[corner]];
      resting[static_cast<size_t>(corner)] = rest[tet.nodes[corner]];
    }

    const talus::deformable::TrialState computed = talus::deformable::trial(
      deformed,
      resting,
      tet,
      talus::deformable::tetStiffness(tet, params)
    );

    balances.push_back(balanceOf(deformed, computed.force));

    std::cout << "elementbalance: tet " << index << " netForce "
              << balances.back().force << " netTorque "
              << balances.back().torque << '\n';
  }

  for (const ElementBalance& balance : balances) {
    REQUIRE(balance.force <= BALANCE_LIMIT);
    REQUIRE(balance.torque <= BALANCE_LIMIT);
  }
}

namespace {

constexpr uint32_t TANGENT_STATES = 8u;
constexpr float TANGENT_STRAIN = 0.1f;
constexpr float TANGENT_JITTER = 0.2f;
constexpr float TANGENT_ANGLE = 3.0f;
constexpr float TANGENT_STEP = 1.0e-2f;
constexpr double TANGENT_LIMIT = 1.0e-2;

struct TangentError {
  double full = 0.0;
  double rotation = 0.0;
  double symmetric = 0.0;
  double skew = 0.0;
};

float tangentNoise(uint32_t& seed) {
  seed = seed * 1664525u + 1013904223u;

  return static_cast<float>(seed >> 8u) / 8388608.0f - 1.0f;
}

std::array<glm::vec3, 8> tangentCube(
  const std::array<glm::vec3, 8>& rest,
  uint32_t& seed
) {
  const glm::vec3 axis(
    tangentNoise(seed),
    tangentNoise(seed),
    tangentNoise(seed) + 2.0f
  );
  const glm::mat3 rotation = glm::mat3_cast(
    glm::angleAxis(TANGENT_ANGLE * tangentNoise(seed), glm::normalize(axis))
  );

  glm::mat3 stretch(1.0f);

  for (int column = 0; column != 3; ++column) {
    for (int row = column; row != 3; ++row) {
      const float value = TANGENT_STRAIN * tangentNoise(seed);

      stretch[column][row] += value;

      if (row != column) {
        stretch[row][column] += value;
      }
    }
  }

  std::array<glm::vec3, 8> placed{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::vec3 jitter(
      tangentNoise(seed),
      tangentNoise(seed),
      tangentNoise(seed)
    );

    placed[corner] =
      rotation
      * (stretch * rest[corner] + TANGENT_JITTER * TANGENT_STRAIN * ELEMENT_HALF * jitter);
  }

  return placed;
}

template<size_t CORNERS, typename Forces>
TangentError tangentError(
  const std::array<glm::vec3, CORNERS>& at,
  const std::array<glm::mat3, CORNERS * CORNERS>& stiffness,
  const glm::mat3& rotation,
  const talus::deformable::InitialStress& stress,
  Forces forces
) {
  const double step = static_cast<double>(TANGENT_STEP * ELEMENT_HALF);
  const size_t size = 3u * CORNERS;

  std::vector<double> measured(size * size, 0.0);
  std::vector<double> material(size * size, 0.0);
  std::vector<double> exact(size * size, 0.0);
  std::vector<double> operated(size * size, 0.0);

  for (size_t column = 0u; column != CORNERS; ++column) {
    for (int axis = 0; axis != 3; ++axis) {
      std::array<glm::vec3, CORNERS> ahead = at;
      std::array<glm::vec3, CORNERS> behind = at;

      ahead[column][axis] += static_cast<float>(step);
      behind[column][axis] -= static_cast<float>(step);

      const double moved = static_cast<double>(ahead[column][axis])
                           - static_cast<double>(behind[column][axis]);

      const std::array<glm::vec3, CORNERS> pushed = forces(ahead);
      const std::array<glm::vec3, CORNERS> pulled = forces(behind);

      std::array<glm::vec3, CORNERS> unit{};
      std::array<glm::vec3, CORNERS> applied{};

      unit[column][axis] = 1.0f;

      talus::deformable::applyInitialStress(stress, unit, applied);

      for (size_t row = 0u; row != CORNERS; ++row) {
        for (int component = 0; component != 3; ++component) {
          const size_t entry =
            (3u * row + static_cast<size_t>(component)) * size + 3u * column
            + static_cast<size_t>(axis);

          measured[entry] = -(static_cast<double>(pushed[row][component])
                              - static_cast<double>(pulled[row][component]))
                            / moved;
          operated[entry] = static_cast<double>(applied[row][component]);
        }
      }
    }
  }

  const glm::mat3 inverse = glm::transpose(rotation);

  for (size_t row = 0u; row != CORNERS; ++row) {
    for (size_t column = 0u; column != CORNERS; ++column) {
      const glm::mat3 linear =
        rotation * stiffness[row * CORNERS + column] * inverse;
      const glm::mat3 turned = stress.twist[row] * stress.turn[column];

      for (int r = 0; r != 3; ++r) {
        for (int c = 0; c != 3; ++c) {
          const size_t entry = (3u * row + static_cast<size_t>(r)) * size
                               + 3u * column + static_cast<size_t>(c);

          material[entry] = static_cast<double>(linear[c][r]);
          exact[entry] = static_cast<double>(linear[c][r] + turned[c][r]);
          operated[entry] += static_cast<double>(linear[c][r]);
        }
      }
    }
  }

  double whole = 0.0;
  double term = 0.0;
  double missed = 0.0;
  double unsymmetric = 0.0;
  double antisymmetric = 0.0;

  for (size_t row = 0u; row != size; ++row) {
    for (size_t column = 0u; column != size; ++column) {
      const double held = measured[row * size + column];
      const double mirrored = 0.5 * (held + measured[column * size + row]);
      const double rotated = held - material[row * size + column];

      whole += held * held;
      term += rotated * rotated;
      antisymmetric += (held - mirrored) * (held - mirrored);
      missed += (held - exact[row * size + column])
                * (held - exact[row * size + column]);
      unsymmetric += (mirrored - operated[row * size + column])
                     * (mirrored - operated[row * size + column]);
    }
  }

  return TangentError{
    std::sqrt(missed / whole),
    std::sqrt(missed / term),
    std::sqrt(unsymmetric / term),
    std::sqrt(antisymmetric / term)
  };
}

}

TEST_CASE(
  "a deformable element tangent carries its rotation's derivative",
  "[deformable]"
) {
  const std::vector<PhysicalMaterial> materials = beamMaterials();
  const PhysicalMaterial& lane = materials[BULK_LANE];

  const talus::deformable::ElasticParams params{
    lane.youngsModulus,
    lane.poissonsRatio,
    lane.density
  };

  const std::array<glm::vec3, 8> rest = restCube();
  const talus::objectmodel::PreparedHex hex = cubeHex();
  const talus::deformable::HexStiffness hexed =
    talus::deformable::hexStiffness(hex, params);

  uint32_t seed = 20260923u;

  TangentError worst;
  double hexSkew = 0.0;
  double tetSkew = 0.0;

  const auto record = [&worst](const TangentError& error) {
    worst.full = std::max(worst.full, error.full);
    worst.rotation = std::max(worst.rotation, error.rotation);
    worst.symmetric = std::max(worst.symmetric, error.symmetric);
  };

  for (uint32_t state = 0u; state != TANGENT_STATES; ++state) {
    const std::array<glm::vec3, 8> placed = tangentCube(rest, seed);

    const talus::deformable::HexTrial trial =
      talus::deformable::hexTrial(placed, rest, hex, hexed);

    const TangentError hexError = tangentError<8u>(
      placed,
      hexed.block,
      trial.rotation,
      talus::deformable::initialStress(
        placed,
        rest,
        hex,
        hexed,
        trial.rotation,
        trial.force
      ),
      [&](const std::array<glm::vec3, 8>& at) {
        return talus::deformable::hexTrial(at, rest, hex, hexed).force;
      }
    );

    std::cout << "elementtangent: hex state " << state << " strain "
              << TANGENT_STRAIN << " full " << hexError.full << " rotation "
              << hexError.rotation << " symmetric " << hexError.symmetric
              << " skew " << hexError.skew << '\n';

    record(hexError);
    hexSkew = std::max(hexSkew, hexError.skew);

    for (const std::array<uint32_t, 4>& pick : KUHN_TETS) {
      const talus::objectmodel::PreparedTet tet = kuhnTet(pick, rest);
      const talus::deformable::TetStiffness stiffness =
        talus::deformable::tetStiffness(tet, params);

      std::array<glm::vec3, 4> deformed{};
      std::array<glm::vec3, 4> resting{};

      for (int corner = 0; corner != 4; ++corner) {
        deformed[static_cast<size_t>(corner)] = placed[tet.nodes[corner]];
        resting[static_cast<size_t>(corner)] = rest[tet.nodes[corner]];
      }

      const talus::deformable::TrialState computed =
        talus::deformable::trial(deformed, resting, tet, stiffness);

      const TangentError tetError = tangentError<4u>(
        deformed,
        stiffness.block,
        computed.rotation,
        talus::deformable::initialStress(
          deformed,
          resting,
          tet,
          stiffness,
          computed.rotation,
          computed.force
        ),
        [&](const std::array<glm::vec3, 4>& at) {
          return talus::deformable::trial(at, resting, tet, stiffness).force;
        }
      );

      record(tetError);
      tetSkew = std::max(tetSkew, tetError.skew);
    }
  }

  std::cout << "elementtangent: states " << TANGENT_STATES << " strain "
            << TANGENT_STRAIN << " step " << TANGENT_STEP * ELEMENT_HALF
            << " worst full " << worst.full << " rotation " << worst.rotation
            << " symmetric " << worst.symmetric << " skew hex " << hexSkew
            << " tet " << tetSkew << '\n';

  REQUIRE(worst.rotation <= TANGENT_LIMIT);
  REQUIRE(worst.symmetric <= TANGENT_LIMIT);
}

TEST_CASE(
  "a free spinning deformable keeps its energy bounded",
  "[.][spinbound]"
) {
  constexpr uint32_t FRAMES = 300u;
  constexpr double CEILING = 1.10;

  JoltLibrary library;

  World source;
  const Entity bar = spinningBar(source);

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  talus::deformable::Settings settings = dynamicSolve(1u);

  settings.element = talus::deformable::VolumeElementType::Hex;

  runtime.setDeformableSettings(settings);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);
  REQUIRE(runtime.world().deformableState(bar) != nullptr);
  REQUIRE(mesh->restPositions.size() == 63u);
  REQUIRE(mesh->hexes.size() == 24u);
  REQUIRE(
    std::none_of(
      mesh->anchored.begin(),
      mesh->anchored.end(),
      [](uint8_t held) { return held != 0u; }
    )
  );

  DeformableState state = *runtime.world().deformableState(bar);

  const glm::mat3 strain(
    glm::vec3(1.0f, 0.5f, -0.3f),
    glm::vec3(0.5f, -0.6f, 0.2f),
    glm::vec3(-0.3f, 0.2f, 0.4f)
  );
  const glm::vec3 spin(0.0f, 5.0f, 0.0f);
  const glm::vec3 drift(0.2f, -0.1f, 0.05f);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 rest = mesh->restPositions[node];
    const glm::vec3 placed = rest + 0.01f * (strain * rest);

    state.position[node] = placed;
    state.velocity[node] = drift + glm::cross(spin, placed);
    state.acceleration[node] = glm::vec3(0.0f);
  }

  runtime.world().setDeformableState(bar, state);

  const talus::deformable::Settings& effective = runtime.deformableSettings();

  std::cout << "spinbound: substeps " << effective.substeps
            << " spectralRadius " << effective.spectralRadius << " tolerance "
            << effective.tolerance << " cgIterations " << effective.cgIterations
            << " factorBudget " << effective.factorBudget << " element "
            << (effective.element == talus::deformable::VolumeElementType::Hex
                  ? "hex"
                  : "tet")
            << " nodes " << mesh->restPositions.size() << " hexes "
            << mesh->hexes.size() << '\n';

  double first = 0.0;
  double last = 0.0;
  double peak = 0.0;
  uint32_t peakFrame = 0u;
  uint32_t firstAbove = 0u;
  uint32_t firstNonFinite = 0u;

  for (uint32_t frame = 1u; frame != FRAMES + 1u; ++frame) {
    runtime.step();

    const PreparedMesh* current = runtime.world().deformablePrepared(bar);
    const DeformableState* moved = runtime.world().deformableState(bar);

    REQUIRE(current != nullptr);
    REQUIRE(moved != nullptr);

    if (firstNonFinite == 0u && !finiteState(*moved)) {
      firstNonFinite = frame;
    }

    last = kineticEnergy(*current, *moved);

    if (frame == 1u) {
      first = last;

      REQUIRE(first > 0.0);
    }

    const double ratio = last / first;

    if (ratio > peak) {
      peak = ratio;
      peakFrame = frame;
    }

    if (firstAbove == 0u && ratio > CEILING) {
      firstAbove = frame;
    }
  }

  std::cout << "spinbound: frame1Kinetic " << first << " peakRatio " << peak
            << " peakFrame " << peakFrame << " firstAboveCeiling " << firstAbove
            << " frame" << FRAMES << "Kinetic " << last << " frame" << FRAMES
            << "Ratio " << last / first << " firstNonFinite " << firstNonFinite
            << '\n';

  REQUIRE(firstNonFinite == 0u);
  REQUIRE(peak <= CEILING);
}

namespace {

constexpr uint32_t PLATE_LANE = 1u;
constexpr uint32_t PLATE_CELLS = 8u;
constexpr float PLATE_WIDTH = 0.2f;
constexpr float PLATE_YOUNGS = 7.0e10f;
constexpr float PLATE_POISSONS = 0.22f;
constexpr float PLATE_DENSITY = 2500.0f;
constexpr float PLATE_TENSION = 7.0e7f;

std::vector<PhysicalMaterial> plateMaterials() {
  std::vector<PhysicalMaterial> materials(2);

  materials[PLATE_LANE].density = PLATE_DENSITY;
  materials[PLATE_LANE].youngsModulus = PLATE_YOUNGS;
  materials[PLATE_LANE].poissonsRatio = PLATE_POISSONS;

  return materials;
}

float plateSpacing() {
  return PLATE_WIDTH / static_cast<float>(PLATE_CELLS);
}

glm::vec3 plateHalf(uint32_t layers) {
  return glm::vec3(
    0.5f * PLATE_WIDTH,
    2.0f * PLATE_WIDTH,
    0.5f * static_cast<float>(layers) * plateSpacing()
  );
}

std::shared_ptr<const PreparedMesh> plateMesh(
  std::span<const PhysicalMaterial> materials,
  uint32_t layers
) {
  const float spacing = plateSpacing();
  const glm::vec3 half = plateHalf(layers);

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * spacing));

  primitive.shapeConfig.y = PLATE_LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(PLATE_CELLS + 1u, 4u * PLATE_CELLS + 1u, layers + 1u)
  );

  const talus::deformable::SupportVolume clamp{
    glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, half.y, 0.0f)),
    glm::vec3(PLATE_WIDTH, 0.25f * spacing, PLATE_WIDTH)
  };

  const std::shared_ptr<const PreparedMesh> mesh = talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>(&clamp, 1u),
    talus::deformable::VolumeElementType::Hex
  );

  REQUIRE(mesh != nullptr);

  return mesh;
}

std::vector<glm::vec3> plateTraction(
  const PreparedMesh& mesh,
  uint32_t layers
) {
  const float spacing = plateSpacing();
  const glm::vec3 half = plateHalf(layers);

  std::vector<glm::vec3> load(mesh.restPositions.size(), glm::vec3(0.0f));

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::vec3 at = mesh.restPositions[node];

    if (std::abs(at.y - half.y) > 0.25f * spacing) {
      continue;
    }

    const bool edge = std::abs(std::abs(at.x) - half.x) < 0.25f * spacing;
    const bool face = std::abs(std::abs(at.z) - half.z) < 0.25f * spacing;
    const float tributary =
      (edge ? 0.5f * spacing : spacing) * (face ? 0.5f * spacing : spacing);

    load[node] = glm::vec3(0.0f, PLATE_TENSION * tributary, 0.0f);
  }

  return load;
}

}

TEST_CASE(
  "a one-cell hex plate in tension reaches its equilibrium",
  "[.][onecellplate]"
) {
  const Clock clock;

  const std::vector<PhysicalMaterial> materials = plateMaterials();
  const std::shared_ptr<const PreparedMesh> mesh = plateMesh(materials, 1u);
  const std::vector<glm::vec3> load = plateTraction(*mesh, 1u);
  const talus::deformable::Settings settings = staticSolve();

  const DeformableState settled = talus::deformable::solveEquilibrium(
    *mesh,
    materials,
    talus::objectmodel::Pose{},
    settings,
    glm::vec3(0.0f),
    load
  );

  const talus::deformable::EquilibriumCheck check =
    talus::deformable::checkEquilibrium(
      *mesh,
      settled,
      materials,
      settings,
      glm::vec3(0.0f),
      load
    );

  float stretch = 0.0f;
  float lateral = 0.0f;

  for (size_t node = 0u; node != mesh->restPositions.size(); ++node) {
    stretch =
      glm::max(stretch, settled.position[node].y - mesh->restPositions[node].y);
    lateral = glm::max(
      lateral,
      std::abs(settled.position[node].z - mesh->restPositions[node].z)
    );
  }

  std::cout << "oneCellPlate: cgIterations " << settings.cgIterations
            << " tolerance " << settings.tolerance << " nodes "
            << mesh->restPositions.size() << " hexes " << mesh->hexes.size()
            << " stretch " << stretch << " expected "
            << 4.0f * PLATE_WIDTH * PLATE_TENSION / PLATE_YOUNGS << " lateral "
            << lateral << " correction " << check.maxCorrection << " capped "
            << check.cgCapped << ", " << clock.seconds() << " wall seconds"
            << '\n';

  REQUIRE_FALSE(check.cgCapped);
  REQUIRE(check.maxCorrection <= talus::deformable::EQUILIBRIUM_MOTION);
}

TEST_CASE("a deformable holds a resting crate", "[.][sustainedrest]") {
  constexpr float CRATE_MASS = 0.15f;
  constexpr float CRATE_HALF = 0.03f;
  constexpr uint32_t WINDOW = 600u;
  constexpr uint32_t WINDOWS = 8u;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  const float rest = surfaceAt(runtime, beam, -0.25f);

  const JPH::BodyID crate =
    dropCrate(
      runtime,
      glm::vec3(-0.25f, rest + CRATE_HALF + 0.001f, 0.0f),
      CRATE_HALF,
      CRATE_MASS
    )
      .body;

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  std::ofstream evidence("deformable-sustained-rest.txt", std::ios::trunc);

  REQUIRE(evidence);

  for (uint32_t window = 0u; window != WINDOWS; ++window) {
    double heightSum = 0.0;
    float lowest = std::numeric_limits<float>::max();
    float highest = std::numeric_limits<float>::lowest();
    float fastest = 0.0f;
    double speedSum = 0.0;
    uint32_t contacts = 0u;

    for (uint32_t step = 0u; step != WINDOW; ++step) {
      runtime.step();

      const float height = bodies.GetPosition(crate).GetY();
      const float speed = bodies.GetLinearVelocity(crate).Length();

      heightSum += height;
      lowest = glm::min(lowest, height);
      highest = glm::max(highest, height);
      fastest = glm::max(fastest, speed);
      speedSum += speed;
      contacts += runtime.deformableReport().contacts;
    }

    const double frames = static_cast<double>(WINDOW);

    evidence << "sustained.window " << window << " meanY " << heightSum / frames
             << " minY " << lowest << " maxY " << highest << " bandY "
             << (highest - lowest) << " meanSpeed " << speedSum / frames
             << " maxSpeed " << fastest << " surfaceY "
             << surfaceAt(runtime, beam, -0.25f) << " meanContacts "
             << static_cast<double>(contacts) / frames << '\n';

    std::cout << "sustained: window " << window << " meanY "
              << heightSum / frames << " bandY " << (highest - lowest)
              << " meanSpeed " << speedSum / frames << " surfaceY "
              << surfaceAt(runtime, beam, -0.25f) << '\n';
  }
}

TEST_CASE("a deformable settles at every stiffness", "[.][stiffnessfloor]") {
  constexpr uint32_t SETTLE = 2400u;
  constexpr uint32_t WINDOW = 30u;

  std::ofstream evidence("deformable-stiffness-floor.txt", std::ios::trunc);

  REQUIRE(evidence);

  for (const float modulus : {1.0e5f, 1.0e6f, 1.0e7f, 1.0e8f, 1.0e9f}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    std::vector<PhysicalMaterial> materials(2);

    materials[BULK_LANE].density = DENSITY;
    materials[BULK_LANE].youngsModulus = modulus;
    materials[BULK_LANE].poissonsRatio = POISSONS;

    source.setMaterials(materials);

    Runtime runtime(source, library);

    prepare(runtime, beam);

    for (uint32_t step = 0u; step != SETTLE; ++step) {
      runtime.step();
    }

    const DeformableState* held = runtime.world().deformableState(beam);

    REQUIRE(held != nullptr);

    const std::vector<glm::vec3> anchor = held->position;

    float worstAcceleration = 0.0f;
    float worstSpeed = 0.0f;
    float worstDrift = 0.0f;
    bool finite = true;

    for (uint32_t step = 0u; step != WINDOW; ++step) {
      runtime.step();

      const DeformableState* now = runtime.world().deformableState(beam);

      REQUIRE(now != nullptr);

      for (size_t node = 0u; node != now->position.size(); ++node) {
        worstAcceleration =
          glm::max(worstAcceleration, glm::length(now->acceleration[node]));
        worstSpeed = glm::max(worstSpeed, glm::length(now->velocity[node]));
        worstDrift =
          glm::max(worstDrift, glm::length(now->position[node] - anchor[node]));

        finite = finite && std::isfinite(now->position[node].x)
                 && std::isfinite(now->acceleration[node].x);
      }
    }

    evidence << "stiffness.point youngs " << modulus << " settled " << SETTLE
             << " window " << WINDOW << " maxAccelG "
             << worstAcceleration / 9.81f << " maxAccelMs2 "
             << worstAcceleration << " maxSpeed " << worstSpeed
             << " maxDriftMm " << worstDrift * 1000.0f << " finite "
             << (finite ? 1u : 0u) << '\n';

    std::cout << "stiffness: youngs " << modulus << " maxAccelG "
              << worstAcceleration / 9.81f << " maxSpeed " << worstSpeed
              << " maxDriftMm " << worstDrift * 1000.0f << " finite "
              << (finite ? 1u : 0u) << '\n';
  }
}

TEST_CASE(
  "a quiet deformable is not always at equilibrium",
  "[.][quietnotrest]"
) {
  constexpr uint32_t WINDOW = 30u;
  constexpr uint32_t WINDOWS = 12u;
  constexpr float TARGET_SAG = 0.05f;

  std::ofstream evidence("deformable-quiet-not-rest.txt", std::ios::trunc);

  REQUIRE(evidence);

  for (const float modulus : {1.0e4f, 1.0e5f, 1.0e6f}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    std::vector<PhysicalMaterial> materials(2);

    materials[BULK_LANE].density = DENSITY;
    materials[BULK_LANE].youngsModulus = modulus;
    materials[BULK_LANE].poissonsRatio = POISSONS;

    source.setMaterials(materials);

    Runtime runtime(source, library);

    prepare(runtime, beam);

    const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);
    const talus::objectmodel::Pose& pose =
      runtime.world().registry().get<talus::objectmodel::Pose>(beam);

    std::vector<glm::vec3> load(mesh.restPositions.size(), glm::vec3(0.0f));

    std::vector<uint32_t> tip;

    for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
      if (mesh.restPositions[node].x > HALF_LENGTH - 0.06f) {
        tip.push_back(static_cast<uint32_t>(node));
      }
    }

    REQUIRE(!tip.empty());

    const float second = 0.1f * std::pow(0.1f, 3.0f) / 12.0f;
    const float reach = HALF_LENGTH - ANCHOR_END;
    const float total =
      3.0f * modulus * second * TARGET_SAG / (reach * reach * reach);

    for (const uint32_t node : tip) {
      load[node] =
        glm::vec3(0.0f, -total / static_cast<float>(tip.size()), 0.0f);
    }

    const talus::deformable::Settings settings = runtime.deformableSettings();

    DeformableState state = talus::deformable::solveEquilibrium(
      mesh,
      runtime.world().materials(),
      pose,
      settings,
      glm::vec3(0.0f),
      load
    );

    float amplitude = 0.0f;

    for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
      amplitude = glm::max(
        amplitude,
        glm::length(state.position[node] - mesh.restPositions[node])
      );
    }

    for (uint32_t window = 0u; window != WINDOWS; ++window) {
      const talus::deformable::EquilibriumCheck checked =
        talus::deformable::checkEquilibrium(
          mesh,
          state,
          runtime.world().materials(),
          settings,
          glm::vec3(0.0f)
        );

      const std::vector<glm::vec3> anchor = state.position;

      float drift = 0.0f;
      float fastest = 0.0f;

      for (uint32_t step = 0u; step != WINDOW; ++step) {
        talus::deformable::StepReport report;

        talus::deformable::step(
          mesh,
          state,
          runtime.world().materials(),
          settings,
          glm::vec3(0.0f),
          PhysicsWorld::TIME_STEP,
          {},
          {},
          nullptr,
          report
        );

        for (size_t node = 0u; node != state.position.size(); ++node) {
          if (mesh.anchored[node] != 0u) {
            continue;
          }

          drift =
            glm::max(drift, glm::length(state.position[node] - anchor[node]));
          fastest = glm::max(fastest, glm::length(state.velocity[node]));
        }
      }

      evidence << "quiet.point youngs " << modulus << " window " << window
               << " amplitudeMm " << amplitude * 1000.0f << " driftMm "
               << drift * 1000.0f << " maxSpeed " << fastest << " correctionMm "
               << checked.maxCorrection * 1000.0f << '\n';

      std::cout << "quiet: youngs " << modulus << " window " << window
                << " driftMm " << drift * 1000.0f << " correctionMm "
                << checked.maxCorrection * 1000.0f << " maxSpeed " << fastest
                << '\n';
    }
  }
}

TEST_CASE("a loaded equilibrium needs its load", "[.][loadedrest]") {
  constexpr float PATCH = 0.06f;
  constexpr float CRATE_MASS = 0.15f;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);
  const talus::objectmodel::Pose& pose =
    runtime.world().registry().get<talus::objectmodel::Pose>(beam);

  std::vector<uint32_t> patch;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const glm::vec3& rest = mesh.restPositions[node];

    if (std::abs(rest.x + 0.25f) <= PATCH && rest.y > HALF_THICKNESS - 0.001f) {
      patch.push_back(static_cast<uint32_t>(node));
    }
  }

  REQUIRE(!patch.empty());

  std::vector<glm::vec3> load(mesh.restPositions.size(), glm::vec3(0.0f));

  const float share =
    CRATE_MASS * PhysicsWorld::GRAVITY / static_cast<float>(patch.size());

  for (const uint32_t node : patch) {
    load[node] = glm::vec3(0.0f, -share, 0.0f);
  }

  const DeformableState bent = talus::deformable::solveEquilibrium(
    mesh,
    runtime.world().materials(),
    pose,
    runtime.deformableSettings(),
    pull(),
    load
  );

  const DeformableState rest = talus::deformable::restState(mesh, pose);

  const auto measure =
    [&](const DeformableState& state, std::span<const glm::vec3> applied) {
      return talus::deformable::checkEquilibrium(
        mesh,
        state,
        runtime.world().materials(),
        runtime.deformableSettings(),
        pull(),
        applied
      );
    };

  const talus::deformable::EquilibriumCheck withLoad = measure(bent, load);
  const talus::deformable::EquilibriumCheck withoutLoad = measure(bent, {});
  const talus::deformable::EquilibriumCheck atRest = measure(rest, load);

  float sag = 0.0f;

  for (const uint32_t node : patch) {
    sag = glm::max(sag, mesh.restPositions[node].y - bent.position[node].y);
  }

  std::ofstream evidence("deformable-loaded-rest.txt", std::ios::trunc);

  REQUIRE(evidence);

  evidence << "loaded.patch nodes " << patch.size() << " sagMm "
           << sag * 1000.0f << " bentWithLoadMm "
           << withLoad.maxCorrection * 1000.0f << " bentWithoutLoadMm "
           << withoutLoad.maxCorrection * 1000.0f << " restWithLoadMm "
           << atRest.maxCorrection * 1000.0f << '\n';

  std::cout << "loaded: sagMm " << sag * 1000.0f << " bentWithLoadMm "
            << withLoad.maxCorrection * 1000.0f << " bentWithoutLoadMm "
            << withoutLoad.maxCorrection * 1000.0f << " restWithLoadMm "
            << atRest.maxCorrection * 1000.0f << '\n';
}

TEST_CASE("a deformable beam reports its cost", "[deformable]") {
  const Clock clock;

  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

  Runtime runtime(source, library);

  prepare(runtime, beam);

  runtime.setDeformableSettings(dynamicSolve(4u));

  double spent = 0.0;

  for (uint32_t taken = 0u; taken != DYNAMIC_STEPS; ++taken) {
    runtime.step();

    spent += runtime.deformableReport().cpuSeconds;
  }

  const talus::deformable::StepReport& report = runtime.deformableReport();

  std::cout << "deformable: " << report.elements << " elements, "
            << report.nodes << " nodes, "
            << spent / static_cast<double>(DYNAMIC_STEPS)
            << " cpu seconds per outer step measured in this Debug tree, where"
               " no performance claim belongs; the only dissipation is the"
               " numerical damping of implicit Euler, never a material"
               " property, "
            << clock.seconds() << " wall seconds" << '\n';

  REQUIRE(runtime.world().deformablePrepared(beam) != nullptr);
  REQUIRE(report.nodes > 0u);
  REQUIRE(report.elements > 0u);
  REQUIRE(spent > 0.0);
}

namespace {

constexpr uint32_t WORLD_SCALE_WARMUP = 120u;
constexpr uint32_t WORLD_SCALE_STEADY_FRAMES = 1200u;
constexpr uint32_t WORLD_SCALE_ACTIVATION_TRIALS = 1000u;
constexpr uint32_t WORLD_SCALE_MAX_STEADY_FRAMES = 2400u;
constexpr uint32_t WORLD_SCALE_MAX_ACTIVATION_TRIALS = 2000u;
constexpr uint32_t WORLD_SCALE_STABILITY_WINDOW = 200u;

enum class SamplingStop : uint8_t {
  Stable,
  Limit,
  NonFinite
};

struct Distribution {
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double worst = 0.0;
};

struct CpuScaleMeasurement {
  Distribution physics;
  Distribution deformable;
  uint32_t frames = 0u;
  uint32_t nodes = 0u;
  uint32_t elements = 0u;
  SamplingStop stop = SamplingStop::Limit;
  double setupMilliseconds = 0.0;
  double elasticMilliseconds = 0.0;
  double primaryCgMilliseconds = 0.0;
  double contactMilliseconds = 0.0;
};

Distribution summarize(std::vector<double> values) {
  REQUIRE_FALSE(values.empty());

  std::sort(values.begin(), values.end());

  const auto at = [&values](double fraction) {
    const size_t index = std::min(
      values.size() - 1u,
      static_cast<size_t>(
        fraction * static_cast<double>(values.size() - 1u) + 0.5
      )
    );

    return values[index];
  };

  return Distribution{at(0.50), at(0.95), at(0.99), values.back()};
}

bool closeEnough(double before, double after) {
  const double scale = glm::max(glm::max(before, after), 1.0e-3);

  return std::abs(before - after) <= 0.05 * scale;
}

bool stableTail(const std::vector<double>& values) {
  const size_t window = WORLD_SCALE_STABILITY_WINDOW;

  if (values.size() < 2u * window) {
    return false;
  }

  const size_t middle = values.size() - window;
  const size_t first = middle - window;

  const Distribution before = summarize(
    std::vector<double>(values.begin() + first, values.begin() + middle)
  );
  const Distribution after =
    summarize(std::vector<double>(values.begin() + middle, values.end()));

  return closeEnough(before.p50, after.p50)
         && closeEnough(before.p95, after.p95)
         && closeEnough(before.p99, after.p99);
}

const char* stopName(SamplingStop stop) {
  switch (stop) {
    case SamplingStop::Stable:
      return "stable";
    case SamplingStop::Limit:
      return "sampleLimit";
    case SamplingStop::NonFinite:
      return "nonFinite";
  }

  return "unknown";
}

class ScaleScene {
public:
  ScaleScene(uint32_t bodies, float voxel) {
    m_bodies.reserve(bodies);

    for (uint32_t index = 0u; index != bodies; ++index) {
      const Entity beam = beamEntity(m_source, voxel);

      REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

      talus::objectmodel::Pose pose{};

      pose.position.z =
        0.3f
        * (static_cast<float>(index) - 0.5f * static_cast<float>(bodies - 1u));

      m_source.emplaceOrReplace<talus::objectmodel::Pose>(beam, pose);

      m_bodies.push_back(beam);
    }

    m_runtime = std::make_unique<Runtime>(m_source, m_library);
    m_runtime->setDeformableSettings(dynamicSolve(1u));

    prepare(*m_runtime, m_bodies.front());

    for (const Entity body : m_bodies) {
      settleAtEquilibrium(*m_runtime, body);

      const PreparedMesh* mesh = m_runtime->world().deformablePrepared(body);
      const DeformableState* state = m_runtime->world().deformableState(body);

      REQUIRE(mesh != nullptr);
      REQUIRE(state != nullptr);

      m_nodes += static_cast<uint32_t>(mesh->restPositions.size());
      m_elements +=
        static_cast<uint32_t>(mesh->tets.size() + mesh->hexes.size());
      m_equilibrium.push_back(*state);

      const auto found =
        std::find(mesh->anchored.begin(), mesh->anchored.end(), uint8_t{0u});

      REQUIRE(found != mesh->anchored.end());

      m_mobileNodes.push_back(
        static_cast<uint32_t>(std::distance(mesh->anchored.begin(), found))
      );
    }
  }

  [[nodiscard]] Runtime& runtime() noexcept {
    return *m_runtime;
  }

  [[nodiscard]] uint32_t nodes() const noexcept {
    return m_nodes;
  }

  [[nodiscard]] uint32_t elements() const noexcept {
    return m_elements;
  }

  [[nodiscard]] bool finite() const {
    for (const Entity body : m_bodies) {
      const DeformableState* state = m_runtime->world().deformableState(body);

      if (state == nullptr) {
        return false;
      }

      for (const glm::vec3& at : state->position) {
        if (
          !std::isfinite(at.x) || !std::isfinite(at.y) || !std::isfinite(at.z)
        ) {
          return false;
        }
      }
    }

    return true;
  }

  void resetAndActivate(uint32_t activeBodies) {
    REQUIRE(activeBodies <= m_bodies.size());

    for (size_t index = 0u; index != m_bodies.size(); ++index) {
      m_runtime->world().setDeformableState(
        m_bodies[index],
        m_equilibrium[index]
      );

      if (index >= activeBodies) {
        continue;
      }

      const uint32_t node = m_mobileNodes[index];

      m_runtime->world().patchDeformableState(
        m_bodies[index],
        [node](DeformableState& state) { state.velocity[node].y = -0.1f; }
      );
    }
  }

private:
  JoltLibrary m_library;
  World m_source;
  std::unique_ptr<Runtime> m_runtime;
  std::vector<Entity> m_bodies;
  std::vector<DeformableState> m_equilibrium;
  std::vector<uint32_t> m_mobileNodes;
  uint32_t m_nodes = 0u;
  uint32_t m_elements = 0u;
};

CpuScaleMeasurement measureCpuScale(
  ScaleScene& scene,
  uint32_t activeBodies,
  uint32_t defaultSamples,
  uint32_t maximumSamples
) {
  Runtime& runtime = scene.runtime();

  for (uint32_t frame = 0u; frame != WORLD_SCALE_WARMUP; ++frame) {
    if (activeBodies != 0u) {
      scene.resetAndActivate(activeBodies);
    }

    runtime.step();
  }

  std::vector<double> physics;
  std::vector<double> deformable;

  physics.reserve(maximumSamples);
  deformable.reserve(maximumSamples);

  double setup = 0.0;
  double elastic = 0.0;
  double primaryCg = 0.0;
  double contact = 0.0;
  uint32_t reportedNodes = 0u;
  uint32_t reportedElements = 0u;
  SamplingStop stop = SamplingStop::Limit;

  for (uint32_t frame = 0u; frame != maximumSamples; ++frame) {
    if (activeBodies != 0u) {
      scene.resetAndActivate(activeBodies);
    }

    const auto began = std::chrono::steady_clock::now();

    runtime.step();

    const auto done = std::chrono::steady_clock::now();
    const talus::deformable::StepReport& report = runtime.deformableReport();

    reportedNodes = report.nodes;
    reportedElements = report.elements;

    if (!std::isfinite(report.cpuSeconds) || !scene.finite()) {
      stop = SamplingStop::NonFinite;

      break;
    }

    physics.push_back(
      1000.0 * std::chrono::duration<double>(done - began).count()
    );
    deformable.push_back(1000.0 * report.cpuSeconds);
    setup += 1000.0 * report.setupSeconds;
    elastic += 1000.0 * report.elasticSeconds;
    primaryCg += 1000.0 * report.primaryCgSeconds;
    contact += 1000.0 * report.contactSeconds;

    if (
      physics.size() >= defaultSamples && stableTail(physics)
      && stableTail(deformable)
    ) {
      stop = SamplingStop::Stable;

      break;
    }
  }

  const double frames = static_cast<double>(physics.size());

  REQUIRE(reportedNodes == scene.nodes());
  REQUIRE(reportedElements == scene.elements());
  REQUIRE_FALSE(physics.empty());
  REQUIRE_FALSE(deformable.empty());

  return CpuScaleMeasurement{
    summarize(std::move(physics)),
    summarize(std::move(deformable)),
    static_cast<uint32_t>(frames),
    scene.nodes(),
    scene.elements(),
    stop,
    setup / frames,
    elastic / frames,
    primaryCg / frames,
    contact / frames
  };
}

void printCpuScale(
  std::ostream& stream,
  const char* curve,
  const char* sampleKind,
  uint32_t potentialBodies,
  uint32_t activeBodies,
  const CpuScaleMeasurement& measured
) {
  stream << "worldscale.cpu curve " << curve << " activeBodies " << activeBodies
         << " potentialBodies " << potentialBodies << " nodes "
         << measured.nodes << " elements " << measured.elements << " "
         << sampleKind << " " << measured.frames << " sampleStop "
         << stopName(measured.stop) << " cpuPhysicsP50Ms "
         << measured.physics.p50 << " cpuPhysicsP95Ms " << measured.physics.p95
         << " cpuPhysicsP99Ms " << measured.physics.p99 << " cpuPhysicsWorstMs "
         << measured.physics.worst << " cpuDeformableP50Ms "
         << measured.deformable.p50 << " cpuDeformableP95Ms "
         << measured.deformable.p95 << " cpuDeformableP99Ms "
         << measured.deformable.p99 << " cpuDeformableWorstMs "
         << measured.deformable.worst << " setupMeanMs "
         << measured.setupMilliseconds << " elasticMeanMs "
         << measured.elasticMilliseconds << " primaryCgMeanMs "
         << measured.primaryCgMilliseconds << " contactMeanMs "
         << measured.contactMilliseconds << '\n';
}

}

TEST_CASE(
  "a deformable world reports its CPU scaling envelope",
  "[.][worldscale]"
) {
  std::ofstream evidence("deformable-world-scale-cpu.txt", std::ios::trunc);

  REQUIRE(evidence);

  const auto record = [&evidence](
                        const char* curve,
                        const char* sampleKind,
                        uint32_t potentialBodies,
                        uint32_t activeBodies,
                        const CpuScaleMeasurement& measured
                      ) {
    printCpuScale(
      std::cout,
      curve,
      sampleKind,
      potentialBodies,
      activeBodies,
      measured
    );
    printCpuScale(
      evidence,
      curve,
      sampleKind,
      potentialBodies,
      activeBodies,
      measured
    );
  };

  struct PartitionPoint {
    uint32_t bodies = 0u;
    float voxel = 0.0f;
  };

  for (const uint32_t bodies : {1u, 2u, 4u, 8u}) {
    ScaleScene scene(bodies, COARSE_VOXEL);

    const CpuScaleMeasurement settled = measureCpuScale(
      scene,
      0u,
      WORLD_SCALE_STEADY_FRAMES,
      WORLD_SCALE_MAX_STEADY_FRAMES
    );
    const CpuScaleMeasurement inactive = measureCpuScale(
      scene,
      1u,
      WORLD_SCALE_STEADY_FRAMES,
      WORLD_SCALE_MAX_STEADY_FRAMES
    );
    const CpuScaleMeasurement active = measureCpuScale(
      scene,
      bodies,
      WORLD_SCALE_STEADY_FRAMES,
      WORLD_SCALE_MAX_STEADY_FRAMES
    );
    const CpuScaleMeasurement activation = measureCpuScale(
      scene,
      1u,
      WORLD_SCALE_ACTIVATION_TRIALS,
      WORLD_SCALE_MAX_ACTIVATION_TRIALS
    );

    record("settled", "steadyFrames", bodies, 0u, settled);
    record("inactive", "steadyFrames", bodies, 1u, inactive);
    record("active", "steadyFrames", bodies, bodies, active);
    record("activation", "activationTrials", bodies, 1u, activation);

    REQUIRE(settled.nodes > 0u);
    REQUIRE(settled.elements > 0u);
    REQUIRE(settled.physics.p95 > 0.0);
    REQUIRE(settled.deformable.p95 > 0.0);
    REQUIRE(inactive.nodes == settled.nodes);
    REQUIRE(inactive.elements == settled.elements);
    REQUIRE(inactive.physics.p95 > 0.0);
    REQUIRE(inactive.deformable.p95 > 0.0);
    REQUIRE(active.nodes == settled.nodes);
    REQUIRE(active.elements == settled.elements);
    REQUIRE(active.physics.p95 > 0.0);
    REQUIRE(active.deformable.p95 > 0.0);
    REQUIRE(activation.nodes == settled.nodes);
    REQUIRE(activation.elements == settled.elements);
    REQUIRE(activation.physics.p95 > 0.0);
    REQUIRE(activation.deformable.p95 > 0.0);
  }

  const std::array<PartitionPoint, 4> partition{
    PartitionPoint{1u, BASE_VOXEL},
    PartitionPoint{2u, 1.0f / 30.0f},
    PartitionPoint{4u, 1.0f / 24.0f},
    PartitionPoint{8u, 1.0f / 15.0f}
  };

  uint32_t partitionReferenceNodes = 0u;

  for (const PartitionPoint& point : partition) {
    ScaleScene scene(point.bodies, point.voxel);

    const CpuScaleMeasurement measured = measureCpuScale(
      scene,
      point.bodies,
      WORLD_SCALE_STEADY_FRAMES,
      WORLD_SCALE_MAX_STEADY_FRAMES
    );

    record("partition", "steadyFrames", point.bodies, point.bodies, measured);

    REQUIRE(measured.nodes > 0u);
    REQUIRE(measured.elements > 0u);
    REQUIRE(measured.physics.p95 > 0.0);
    REQUIRE(measured.deformable.p95 > 0.0);

    if (partitionReferenceNodes == 0u) {
      partitionReferenceNodes = measured.nodes;

      continue;
    }

    REQUIRE(measured.nodes >= 4u * partitionReferenceNodes / 5u);
    REQUIRE(measured.nodes <= 6u * partitionReferenceNodes / 5u);
  }

  evidence << "worldscale.finding equilibratedBodiesStepped 1 classification "
              "implementation-limited\n";
}

namespace {

struct ControlledContactMeasurement {
  double cpuPhysicsMilliseconds = 0.0;
  talus::deformable::ContactSolve solve;
  DeformableState state;
  std::vector<talus::deformable::RigidExchange> exchange;
  std::vector<glm::vec3> impulses;
  float residual = 0.0f;
  size_t contactOperatorEntries = 0u;
  uint64_t factorGeneration = 0u;
};

struct ContactDistribution {
  double p50 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double worst = 0.0;
};

struct ControlledContactSeries {
  ContactDistribution coldCpuPhysics;
  ContactDistribution coldResponseCache;
  ContactDistribution warmCpuPhysics;
  ContactDistribution warmDelassus;
  ContactDistribution warmPgs;
  ContactDistribution warmResponse;
  ContactDistribution warmContact;
  ContactDistribution warmPasses;
  uint32_t contacts = 0u;
  uint32_t rows = 0u;
  uint32_t uniqueNodes = 0u;
  uint32_t cacheColumns = 0u;
  uint32_t coldCacheMisses = 0u;
  uint32_t coldBasisSolves = 0u;
  uint32_t warmCacheMisses = 0u;
  uint32_t warmBasisSolves = 0u;
};

ContactDistribution summarizeContact(std::vector<double> values) {
  REQUIRE_FALSE(values.empty());

  std::sort(values.begin(), values.end());

  const auto at = [&values](double fraction) {
    const size_t index = std::min(
      values.size() - 1u,
      static_cast<size_t>(
        fraction * static_cast<double>(values.size() - 1u) + 0.5
      )
    );

    return values[index];
  };

  return ContactDistribution{at(0.50), at(0.95), at(0.99), values.back()};
}

class ControlledContactScene {
public:
  ControlledContactScene() {
    const Entity beam = beamEntity(m_source, COARSE_VOXEL);

    REQUIRE(beam != talus::objectmodel::INVALID_ENTITY);

    m_runtime = std::make_unique<Runtime>(m_source, m_library);

    talus::deformable::Settings settings = dynamicSolve(1u);

    settings.element = talus::deformable::VolumeElementType::Hex;
    settings.cgIterations = 1200u;
    settings.factorBudget = 0u;
    settings.reportContactDiagnostics = true;

    m_runtime->setDeformableSettings(settings);
    prepare(*m_runtime, beam);
    settleAtEquilibrium(*m_runtime, beam);

    m_mesh = m_runtime->world().deformablePrepared(beam);
    const DeformableState* state = m_runtime->world().deformableState(beam);

    REQUIRE(m_mesh != nullptr);
    REQUIRE(state != nullptr);

    m_equilibrium = *state;

    for (uint32_t triangle = 0u; triangle != m_mesh->surfaceTriangles.size();
         ++triangle) {
      const glm::uvec3 nodes = m_mesh->surfaceTriangles[triangle];

      if (
        m_mesh->anchored[nodes.x] != 0u || m_mesh->anchored[nodes.y] != 0u
        || m_mesh->anchored[nodes.z] != 0u
      ) {
        continue;
      }

      m_triangles.push_back(triangle);
    }

    REQUIRE(m_triangles.size() >= 4u);

    prime();
  }

  [[nodiscard]] std::vector<talus::deformable::SurfaceContact> contacts(
    uint32_t count,
    std::span<const uint32_t> triangles
  ) const {
    REQUIRE(count != 0u);
    REQUIRE_FALSE(triangles.empty());

    std::vector<talus::deformable::SurfaceContact> held;

    held.reserve(count);

    constexpr std::array<glm::vec3, 4> weights{
      glm::vec3(1.0f / 3.0f),
      glm::vec3(0.6f, 0.2f, 0.2f),
      glm::vec3(0.2f, 0.6f, 0.2f),
      glm::vec3(0.2f, 0.2f, 0.6f)
    };

    for (uint32_t index = 0u; index != count; ++index) {
      const uint32_t triangle = triangles[index % triangles.size()];
      const glm::uvec3 nodes = m_mesh->surfaceTriangles[triangle];
      const glm::vec3 picked = weights[index % weights.size()];

      glm::vec3 point(0.0f);

      for (int corner = 0; corner != 3; ++corner) {
        point += picked[corner] * m_equilibrium.position[nodes[corner]];
      }

      talus::deformable::SurfaceContact contact;

      contact.nodes = nodes;
      contact.weights = picked;
      contact.partner = 0;
      contact.normal = glm::vec3(0.0f, 1.0f, 0.0f);
      contact.localPoint = point;
      contact.friction = 0.5f;
      contact.triangle = triangle;

      held.push_back(contact);
    }

    return held;
  }

  [[nodiscard]] std::vector<talus::deformable::SurfaceContact> contacts(
    uint32_t count
  ) const {
    return contacts(count, std::span<const uint32_t>(m_triangles.data(), 4u));
  }

  [[nodiscard]] std::vector<uint32_t> trianglesForUniqueNodes(
    uint32_t target
  ) const {
    std::vector<uint32_t> selected;
    std::vector<uint32_t> nodes;

    for (const uint32_t triangle : m_triangles) {
      const glm::uvec3 held = m_mesh->surfaceTriangles[triangle];
      bool fresh = false;

      for (int corner = 0; corner != 3; ++corner) {
        fresh =
          fresh
          || std::find(nodes.begin(), nodes.end(), held[corner]) == nodes.end();
      }

      if (!fresh) {
        continue;
      }

      selected.push_back(triangle);

      for (int corner = 0; corner != 3; ++corner) {
        if (
          std::find(nodes.begin(), nodes.end(), held[corner]) == nodes.end()
        ) {
          nodes.push_back(held[corner]);
        }
      }

      if (nodes.size() >= target) {
        break;
      }
    }

    REQUIRE(nodes.size() >= target);

    return selected;
  }

  [[nodiscard]] ControlledContactMeasurement measure(
    std::span<const talus::deformable::SurfaceContact> contacts,
    bool cold,
    std::optional<talus::deformable::ContactOperator> contactOperator =
      std::nullopt,
    bool dynamicPartner = false,
    bool reportContactSystem = false
  ) {
    m_state = m_equilibrium;

    if (cold) {
      m_solver.basis.column.clear();
      m_solver.basis.generation = m_solver.factor.generation();
    }

    std::vector<talus::deformable::RigidPartner> partners(1);

    partners[0].body = 1u;
    partners[0].linearVelocity = glm::vec3(0.0f, 0.1f, 0.0f);
    partners[0].originalLinearVelocity = partners[0].linearVelocity;

    if (dynamicPartner) {
      partners[0].dynamic = true;
      partners[0].inverseMass = 0.25f;
      partners[0].inverseInertia = glm::mat3(0.5f);
    }

    talus::deformable::Settings settings = m_runtime->deformableSettings();

    if (contactOperator) {
      settings.contactOperator = *contactOperator;
    }

    settings.reportContactSystem = reportContactSystem;

    talus::deformable::StepReport report;

    talus::deformable::step(
      *m_mesh,
      m_state,
      m_runtime->world().materials(),
      settings,
      pull(),
      PhysicsWorld::TIME_STEP,
      contacts,
      partners,
      &m_solver,
      report
    );

    REQUIRE(report.refactors == 0u);
    REQUIRE(m_solver.factor.generation() == m_factorGeneration);
    REQUIRE(report.contactSolves.size() == 1u);

    return ControlledContactMeasurement{
      1000.0 * report.cpuSeconds,
      std::move(report.contactSolves.front()),
      m_state,
      std::move(report.exchange),
      std::move(report.contactImpulses),
      report.contactResidual,
      report.contactOperator.size(),
      m_solver.factor.generation()
    };
  }

private:
  void prime() {
    m_state = m_equilibrium;

    std::vector<talus::deformable::RigidPartner> partners;
    talus::deformable::StepReport report;

    talus::deformable::step(
      *m_mesh,
      m_state,
      m_runtime->world().materials(),
      m_runtime->deformableSettings(),
      pull(),
      PhysicsWorld::TIME_STEP,
      std::span<const talus::deformable::SurfaceContact>{},
      partners,
      &m_solver,
      report
    );

    REQUIRE_FALSE(m_solver.factor.empty());

    m_factorGeneration = m_solver.factor.generation();
    m_solver.basis.column.clear();
    m_solver.basis.generation = m_factorGeneration;
  }

  JoltLibrary m_library;
  World m_source;
  std::unique_ptr<Runtime> m_runtime;
  const PreparedMesh* m_mesh = nullptr;
  DeformableState m_equilibrium;
  DeformableState m_state;
  talus::deformable::DeformableSolverState m_solver;
  std::vector<uint32_t> m_triangles;
  uint64_t m_factorGeneration = 0u;
};

ControlledContactSeries measureControlledSeries(
  ControlledContactScene& scene,
  std::span<const talus::deformable::SurfaceContact> contacts
) {
  constexpr uint32_t COLD_SAMPLES = 8u;
  constexpr uint32_t WARM_SAMPLES = 32u;

  std::vector<double> coldCpuPhysics;
  std::vector<double> coldResponseCache;
  std::vector<double> warmCpuPhysics;
  std::vector<double> warmDelassus;
  std::vector<double> warmPgs;
  std::vector<double> warmResponse;
  std::vector<double> warmContact;
  std::vector<double> warmPasses;

  coldCpuPhysics.reserve(COLD_SAMPLES);
  coldResponseCache.reserve(COLD_SAMPLES);
  warmCpuPhysics.reserve(WARM_SAMPLES);
  warmDelassus.reserve(WARM_SAMPLES);
  warmPgs.reserve(WARM_SAMPLES);
  warmResponse.reserve(WARM_SAMPLES);
  warmContact.reserve(WARM_SAMPLES);
  warmPasses.reserve(WARM_SAMPLES);

  ControlledContactMeasurement cold;

  for (uint32_t sample = 0u; sample != COLD_SAMPLES; ++sample) {
    cold = scene.measure(contacts, true);

    REQUIRE(cold.solve.contacts == contacts.size());
    REQUIRE(cold.solve.newBasisSolves > 0u);

    coldCpuPhysics.push_back(cold.cpuPhysicsMilliseconds);
    coldResponseCache.push_back(1000.0 * cold.solve.responseCacheSeconds);
  }

  ControlledContactMeasurement warm;

  for (uint32_t sample = 0u; sample != WARM_SAMPLES; ++sample) {
    warm = scene.measure(contacts, false);

    REQUIRE(warm.solve.contacts == contacts.size());
    REQUIRE(warm.solve.cacheMisses == 0u);
    REQUIRE(warm.solve.newBasisSolves == 0u);

    warmCpuPhysics.push_back(warm.cpuPhysicsMilliseconds);
    warmDelassus.push_back(1000.0 * warm.solve.delassusSeconds);
    warmPgs.push_back(1000.0 * warm.solve.pgsSeconds);
    warmResponse.push_back(1000.0 * warm.solve.responseApplySeconds);
    warmContact.push_back(1000.0 * warm.solve.totalContactSeconds);
    warmPasses.push_back(static_cast<double>(warm.solve.pgsPasses));
  }

  return ControlledContactSeries{
    summarizeContact(std::move(coldCpuPhysics)),
    summarizeContact(std::move(coldResponseCache)),
    summarizeContact(std::move(warmCpuPhysics)),
    summarizeContact(std::move(warmDelassus)),
    summarizeContact(std::move(warmPgs)),
    summarizeContact(std::move(warmResponse)),
    summarizeContact(std::move(warmContact)),
    summarizeContact(std::move(warmPasses)),
    warm.solve.contacts,
    warm.solve.rows,
    warm.solve.uniqueNodes,
    warm.solve.cacheColumnsAfter,
    cold.solve.cacheMisses,
    cold.solve.newBasisSolves,
    warm.solve.cacheMisses,
    warm.solve.newBasisSolves
  };
}

void printControlledSeries(
  std::ostream& stream,
  const char* curve,
  const ControlledContactSeries& measured
) {
  const double contacts = static_cast<double>(measured.contacts);

  stream << "contactscale.control curve " << curve
         << " contactGenerationBypassed 1 contacts " << measured.contacts
         << " rows " << measured.rows << " uniqueContactNodes "
         << measured.uniqueNodes << " cacheColumns " << measured.cacheColumns
         << " coldCacheMisses " << measured.coldCacheMisses
         << " coldNewBasisSolves " << measured.coldBasisSolves
         << " warmCacheMisses " << measured.warmCacheMisses
         << " warmNewBasisSolves " << measured.warmBasisSolves
         << " coldCpuPhysicsP50Ms " << measured.coldCpuPhysics.p50
         << " coldCpuPhysicsP95Ms " << measured.coldCpuPhysics.p95
         << " coldResponseCacheP50Ms " << measured.coldResponseCache.p50
         << " coldResponseCacheP95Ms " << measured.coldResponseCache.p95
         << " cpuPhysicsP50Ms " << measured.warmCpuPhysics.p50
         << " cpuPhysicsP95Ms " << measured.warmCpuPhysics.p95
         << " GAssemblyP50Ms " << measured.warmDelassus.p50
         << " GAssemblyP95Ms " << measured.warmDelassus.p95
         << " GAssemblyP50MsPerC2 "
         << (contacts > 0.0 ? measured.warmDelassus.p50 / (contacts * contacts)
                            : 0.0)
         << " GAssemblyP95MsPerC2 "
         << (contacts > 0.0 ? measured.warmDelassus.p95 / (contacts * contacts)
                            : 0.0)
         << " PGSP50Ms " << measured.warmPgs.p50 << " PGSP95Ms "
         << measured.warmPgs.p95 << " PGSContactPassesP50 "
         << measured.warmPasses.p50 << " PGSContactPassesP95 "
         << measured.warmPasses.p95 << " responseP50Ms "
         << measured.warmResponse.p50 << " responseP95Ms "
         << measured.warmResponse.p95 << " totalContactP50Ms "
         << measured.warmContact.p50 << " totalContactP95Ms "
         << measured.warmContact.p95 << '\n';
}

double relativeL2(
  const std::vector<glm::vec3>& reference,
  const std::vector<glm::vec3>& measured
) {
  REQUIRE(reference.size() == measured.size());

  double difference = 0.0;
  double magnitude = 0.0;

  for (size_t index = 0u; index != reference.size(); ++index) {
    const glm::dvec3 expected(reference[index]);
    const glm::dvec3 actual(measured[index]);
    const glm::dvec3 delta = expected - actual;

    difference += glm::dot(delta, delta);
    magnitude += glm::dot(expected, expected);
  }

  return std::sqrt(difference) / std::max(std::sqrt(magnitude), 1.0e-30);
}

double relativeL2(
  const std::vector<talus::deformable::RigidExchange>& reference,
  const std::vector<talus::deformable::RigidExchange>& measured
) {
  REQUIRE(reference.size() == measured.size());

  double difference = 0.0;
  double magnitude = 0.0;

  const auto add =
    [&difference,
     &magnitude](const glm::vec3& expectedValue, const glm::vec3& actualValue) {
      const glm::dvec3 expected(expectedValue);
      const glm::dvec3 actual(actualValue);
      const glm::dvec3 delta = expected - actual;

      difference += glm::dot(delta, delta);
      magnitude += glm::dot(expected, expected);
    };

  for (size_t index = 0u; index != reference.size(); ++index) {
    REQUIRE(reference[index].body == measured[index].body);

    add(reference[index].impulse, measured[index].impulse);
    add(reference[index].nodeImpulse, measured[index].nodeImpulse);
  }

  return std::sqrt(difference) / std::max(std::sqrt(magnitude), 1.0e-30);
}

void requireCoulomb(
  std::span<const talus::deformable::SurfaceContact> contacts,
  const std::vector<glm::vec3>& impulses
) {
  REQUIRE(contacts.size() == impulses.size());

  for (size_t index = 0u; index != contacts.size(); ++index) {
    const glm::vec3& impulse = impulses[index];

    REQUIRE(impulse.x >= -1.0e-6f);
    REQUIRE(
      glm::length(glm::vec2(impulse.y, impulse.z))
      <= contacts[index].friction * impulse.x + 1.0e-5f
    );
  }
}

}

TEST_CASE(
  "a deformable contact reports its controlled scale",
  "[.][contactscale]"
) {
  ControlledContactScene scene;

  std::ofstream evidence(
    "deformable-contact-scale-controlled.txt",
    std::ios::trunc
  );

  REQUIRE(evidence);

  for (const uint32_t count : {8u, 16u, 32u, 64u, 128u, 192u, 256u}) {
    const std::vector<talus::deformable::SurfaceContact> contacts =
      scene.contacts(count);
    const ControlledContactSeries measured =
      measureControlledSeries(scene, contacts);

    printControlledSeries(std::cout, "contactCount", measured);
    printControlledSeries(evidence, "contactCount", measured);

    REQUIRE(measured.contacts == count);
    REQUIRE(measured.rows == 3u * count);
    REQUIRE(measured.coldCacheMisses > 0u);
    REQUIRE(measured.coldBasisSolves > 0u);
    REQUIRE(measured.warmCacheMisses == 0u);
    REQUIRE(measured.warmBasisSolves == 0u);
  }

  uint32_t previousNodes = 0u;

  for (const uint32_t target : {3u, 12u, 24u, 48u, 96u}) {
    const std::vector<uint32_t> triangles =
      scene.trianglesForUniqueNodes(target);
    const std::vector<talus::deformable::SurfaceContact> contacts =
      scene.contacts(128u, triangles);
    const ControlledContactSeries measured =
      measureControlledSeries(scene, contacts);

    printControlledSeries(std::cout, "uniqueNodes", measured);
    printControlledSeries(evidence, "uniqueNodes", measured);

    REQUIRE(measured.contacts == 128u);
    REQUIRE(measured.rows == 384u);
    REQUIRE(measured.uniqueNodes >= target);
    REQUIRE(measured.uniqueNodes > previousNodes);
    REQUIRE(measured.warmCacheMisses == 0u);
    REQUIRE(measured.warmBasisSolves == 0u);

    previousNodes = measured.uniqueNodes;
  }
}

TEST_CASE(
  "a deformable contact compares dense and unbuilt response",
  "[.][contactschur]"
) {
  constexpr uint32_t CONTACTS = 128u;

  const talus::deformable::Settings settings;

  REQUIRE(
    settings.contactOperator == talus::deformable::ContactOperator::Unbuilt
  );

  ControlledContactScene scene;
  const std::vector<talus::deformable::SurfaceContact> contacts =
    scene.contacts(CONTACTS);

  const ControlledContactMeasurement warmed = scene.measure(
    contacts,
    true,
    talus::deformable::ContactOperator::Dense,
    true,
    true
  );
  const ControlledContactMeasurement dense = scene.measure(
    contacts,
    false,
    talus::deformable::ContactOperator::Dense,
    true,
    true
  );
  const ControlledContactMeasurement unbuilt =
    scene.measure(contacts, false, std::nullopt, true, true);

  REQUIRE(warmed.solve.cacheMisses > 0u);
  REQUIRE(warmed.solve.newBasisSolves > 0u);
  REQUIRE(dense.factorGeneration == unbuilt.factorGeneration);
  REQUIRE(dense.solve.cacheMisses == 0u);
  REQUIRE(unbuilt.solve.cacheMisses == 0u);
  REQUIRE(dense.solve.newBasisSolves == 0u);
  REQUIRE(unbuilt.solve.newBasisSolves == 0u);
  REQUIRE(dense.solve.contacts == CONTACTS);
  REQUIRE(dense.solve.contacts == unbuilt.solve.contacts);
  REQUIRE(dense.solve.rows == 3u * CONTACTS);
  REQUIRE(dense.solve.rows == unbuilt.solve.rows);
  REQUIRE(dense.solve.pgsPasses == unbuilt.solve.pgsPasses);
  REQUIRE(
    dense.contactOperatorEntries
    == static_cast<size_t>(dense.solve.rows) * dense.solve.rows
  );
  REQUIRE(unbuilt.contactOperatorEntries == 0u);
  REQUIRE_FALSE(dense.exchange.empty());
  REQUIRE_FALSE(unbuilt.exchange.empty());
  REQUIRE(relativeL2(dense.impulses, unbuilt.impulses) <= 1.0e-5);
  REQUIRE(relativeL2(dense.state.position, unbuilt.state.position) <= 1.0e-5);
  REQUIRE(relativeL2(dense.state.velocity, unbuilt.state.velocity) <= 1.0e-5);
  REQUIRE(
    relativeL2(dense.state.acceleration, unbuilt.state.acceleration) <= 1.0e-5
  );
  REQUIRE(relativeL2(dense.exchange, unbuilt.exchange) <= 1.0e-5);
  REQUIRE(std::abs(dense.residual - unbuilt.residual) <= 1.0e-5f);

  requireCoulomb(contacts, dense.impulses);
  requireCoulomb(contacts, unbuilt.impulses);
}

TEST_CASE(
  "a deformable beam records its stair-stepped surface edges",
  "[deformable]"
) {
  constexpr uint32_t NODES = 12u;
  constexpr uint32_t EDGES = 30u;

  const std::vector<talus::field::Primitive> primitives{
    spanning(glm::vec3(-0.2f), glm::vec3(0.2f), BULK_LANE)
  };

  const talus::field::SampledSdf lattice = talus::field::bake(
    primitives,
    glm::vec3(-0.1f, -0.05f, -0.05f),
    glm::vec3(0.1f, 0.05f, 0.05f),
    glm::uvec3(3u, 2u, 2u)
  );

  const std::vector<PhysicalMaterial> materials = beamMaterials();

  const std::shared_ptr<const PreparedMesh> mesh = talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    {},
    talus::deformable::VolumeElementType::Tet
  );

  REQUIRE(mesh != nullptr);
  REQUIRE(mesh->restPositions.size() == NODES);
  REQUIRE(mesh->surfaceNodes.size() == NODES);
  REQUIRE(mesh->tets.size() == 12u);
  REQUIRE(mesh->surfaceEdges.size() == EDGES);

  for (size_t at = 0u; at != mesh->surfaceEdges.size(); ++at) {
    const glm::uvec2 edge = mesh->surfaceEdges[at];

    REQUIRE(edge.x < edge.y);
    REQUIRE(edge.y < mesh->restPositions.size());

    REQUIRE(
      std::find(mesh->surfaceNodes.begin(), mesh->surfaceNodes.end(), edge.x)
      != mesh->surfaceNodes.end()
    );
    REQUIRE(
      std::find(mesh->surfaceNodes.begin(), mesh->surfaceNodes.end(), edge.y)
      != mesh->surfaceNodes.end()
    );

    if (at == 0u) {
      continue;
    }

    const glm::uvec2 before = mesh->surfaceEdges[at - 1u];

    REQUIRE((before.x < edge.x || (before.x == edge.x && before.y < edge.y)));
  }

  std::cout << "deformable: a two cell beam has " << mesh->surfaceEdges.size()
            << " surface edges over " << mesh->surfaceNodes.size()
            << " surface nodes (20 lattice edges of a 3 x 2 x 2 node block plus"
               " one diagonal on each of its 10 boundary squares; the face the"
               " two cells share is owned twice and contributes none)"
            << '\n';
}

TEST_CASE("a deformable body drops its factor when remeshed", "[deformable]") {
  JoltLibrary library;

  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);

  Runtime runtime(source, library);

  runtime.setDeformableSettings(dynamicSolve(4u));
  prepare(runtime, beam);

  const size_t before =
    runtime.world().deformablePrepared(beam)->restPositions.size();

  for (uint32_t taken = 0u; taken != 5u; ++taken) {
    runtime.step();
  }

  const glm::vec3 minimum(
    -HALF_LENGTH - BASE_VOXEL,
    -HALF_THICKNESS - BASE_VOXEL,
    -HALF_THICKNESS - BASE_VOXEL
  );

  runtime.world().setSampledSdf(
    beam,
    talus::field::bake(
      runtime.world().primitives(beam),
      minimum,
      -minimum,
      glm::uvec3(
        static_cast<uint32_t>(std::lround(1.0f / BASE_VOXEL)) + 3u,
        static_cast<uint32_t>(std::lround(2.0f * HALF_THICKNESS / BASE_VOXEL))
          + 3u,
        static_cast<uint32_t>(std::lround(2.0f * HALF_THICKNESS / BASE_VOXEL))
          + 3u
      )
    )
  );

  runtime.refreshBody(beam);

  const size_t after =
    runtime.world().deformablePrepared(beam)->restPositions.size();

  REQUIRE(after != before);

  for (uint32_t taken = 0u; taken != 5u; ++taken) {
    runtime.step();
  }

  const DeformableState* state = runtime.world().deformableState(beam);

  REQUIRE(state != nullptr);

  float worst = 0.0f;

  for (const glm::vec3& moving : state->velocity) {
    worst = glm::max(worst, glm::length(moving));
  }

  std::cout << "deformable: a beam remeshed from " << before << " to " << after
            << " nodes keeps stepping, its fastest node at " << worst << '\n';

  REQUIRE(std::isfinite(worst));
  REQUIRE(worst < 1.0f);
}

namespace {

namespace rigidmodes {

constexpr uint32_t LANE = 1u;
constexpr float HALF_LENGTH = 0.2f;
constexpr float HALF_WIDTH = 0.025f;
constexpr float VOXEL = 0.025f;
constexpr float YOUNGS = 2.0e8f;
constexpr float POISSONS = 0.3f;
constexpr float DENSITY = 1000.0f;
constexpr double BAR = 1.0e-5;

std::vector<PhysicalMaterial> lanes() {
  std::vector<PhysicalMaterial> made(2);

  made[LANE].density = DENSITY;
  made[LANE].youngsModulus = YOUNGS;
  made[LANE].poissonsRatio = POISSONS;

  return made;
}

std::shared_ptr<const PreparedMesh> bar(
  std::span<const PhysicalMaterial> materials,
  talus::deformable::VolumeElementType element
) {
  const glm::vec3 half(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH);

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * VOXEL));

  primitive.shapeConfig.y = LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(glm::round(2.0f * half / VOXEL)) + glm::uvec3(1u)
  );

  std::shared_ptr<const PreparedMesh> mesh = talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    element
  );

  REQUIRE(mesh != nullptr);
  REQUIRE(std::ranges::none_of(mesh->anchored, [](uint8_t held) {
    return held != 0u;
  }));

  return mesh;
}

double residual(
  const talus::deformable::SystemFactor& factor,
  const PreparedMesh& mesh,
  const std::vector<glm::vec3>& mode
) {
  const float inertia = factor.reference().inertia;

  std::vector<glm::vec3> loaded(mode.size(), glm::vec3(0.0f));

  for (size_t node = 0u; node != mode.size(); ++node) {
    loaded[node] = inertia * mesh.nodeMass[node] * mode[node];
  }

  talus::deformable::applyFactor(factor, glm::mat3(1.0f), mesh, loaded);

  double apart = 0.0;
  double length = 0.0;

  for (size_t node = 0u; node != mode.size(); ++node) {
    const glm::dvec3 miss = glm::dvec3(loaded[node]) - glm::dvec3(mode[node]);

    apart += glm::dot(miss, miss);
    length += glm::dot(glm::dvec3(mode[node]), glm::dvec3(mode[node]));
  }

  return std::sqrt(apart / length);
}

std::array<double, 6> residuals(
  const talus::deformable::SystemFactor& factor,
  const PreparedMesh& mesh
) {
  const talus::deformable::FactorReference& reference = factor.reference();
  const size_t nodes = mesh.restPositions.size();

  std::array<double, 6> made{};

  for (int axis = 0; axis != 3; ++axis) {
    glm::vec3 about(0.0f);

    about[axis] = 1.0f;

    std::vector<glm::vec3> translation(nodes, about);
    std::vector<glm::vec3> rotation(nodes, glm::vec3(0.0f));

    for (size_t node = 0u; node != nodes; ++node) {
      rotation[node] =
        glm::cross(about, reference.positions[node] - reference.centroid);
    }

    made[static_cast<size_t>(axis)] = residual(factor, mesh, translation);
    made[static_cast<size_t>(axis) + 3u] = residual(factor, mesh, rotation);
  }

  return made;
}

}

}

TEST_CASE("a free deformable's factor maps its rigid motions", "[deformable]") {
  const std::vector<PhysicalMaterial> materials = rigidmodes::lanes();

  std::vector<double> measured;

  for (const talus::deformable::VolumeElementType element :
       {talus::deformable::VolumeElementType::Hex,
        talus::deformable::VolumeElementType::Tet}) {
    const std::shared_ptr<const PreparedMesh> mesh =
      rigidmodes::bar(materials, element);

    talus::deformable::Settings settings;

    settings.element = element;

    DeformableState state =
      talus::deformable::restState(*mesh, talus::objectmodel::Pose{});
    talus::deformable::DeformableSolverState solver;
    talus::deformable::StepReport report;

    talus::deformable::step(
      *mesh,
      state,
      materials,
      settings,
      glm::vec3(0.0f),
      PhysicsWorld::TIME_STEP,
      {},
      {},
      &solver,
      report
    );

    REQUIRE(solver.stats.refactorCount == 1u);
    REQUIRE(!solver.factor.empty());

    const std::array<double, 6> made =
      rigidmodes::residuals(solver.factor, *mesh);

    std::cout << "deformable: a free "
              << (element == talus::deformable::VolumeElementType::Hex ? "hex"
                                                                       : "tet")
              << " bar of " << mesh->restPositions.size() << " nodes, substeps "
              << settings.substeps << " spectralRadius "
              << settings.spectralRadius << ", its release factor at step "
              << solver.factor.reference().step << " inertia "
              << solver.factor.reference().inertia
              << " maps translations x y z to within " << made[0] << ' '
              << made[1] << ' ' << made[2] << " and rotations to within "
              << made[3] << ' ' << made[4] << ' ' << made[5] << " (bar "
              << rigidmodes::BAR << ")\n";

    measured.insert(measured.end(), made.begin(), made.end());
  }

  for (const double each : measured) {
    REQUIRE(each <= rigidmodes::BAR);
  }
}

TEST_CASE("a deformable beam catches a dropped crate", "[deformable]") {
  constexpr float BOX_MASS = 0.15f;
  constexpr float BOX_HALF = 0.05f;
  constexpr float DROP = 0.245f;
  constexpr uint32_t FRAMES = 40u;
  constexpr float HEADROOM = 1.25f;
  constexpr float SETTLED = 2.0e-3f;

  for (const bool hex : {false, true}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    Runtime runtime(source, library);

    talus::deformable::Settings settings = dynamicSolve(4u);

    settings.element = hex ? talus::deformable::VolumeElementType::Hex
                           : talus::deformable::VolumeElementType::Tet;

    runtime.setDeformableSettings(settings);

    prepare(runtime, beam);

    const float resting = surfaceAt(runtime, beam, 0.3f);

    const JPH::BodyID box = dropCrate(
                              runtime,
                              glm::vec3(0.3f, resting + BOX_HALF + DROP, 0.0f),
                              BOX_HALF,
                              BOX_MASS
    )
                              .body;

    JPH::BodyInterface& bodies = runtime.physics().bodies();

    float approaching = 0.0f;
    float carrying = 0.0f;
    float reach = 0.0f;
    float drift = 0.0f;
    uint32_t supported = 0u;
    bool touched = false;

    for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
      runtime.step();

      const talus::deformable::StepReport& report = runtime.deformableReport();

      drift = glm::max(drift, report.contactResidual);

      if (report.contacts != 0u) {
        touched = true;
        ++supported;
      }

      const DeformableState* state = runtime.world().deformableState(beam);

      REQUIRE(state != nullptr);

      float fastest = 0.0f;

      for (size_t node = 0u; node != state->position.size(); ++node) {
        REQUIRE(std::isfinite(glm::length(state->position[node])));

        fastest = glm::max(fastest, glm::length(state->velocity[node]));
      }

      if (touched) {
        carrying = glm::max(carrying, fastest);
      } else {
        approaching = glm::max(approaching, fastest);
      }

      reach = glm::max(reach, std::abs(bodies.GetPosition(box).GetX()));
    }

    std::cout << "deformable: a " << (hex ? "hex" : "tet") << " beam moves at "
              << approaching << " before the crate lands and " << carrying
              << " after, the"
                 " crate reaching "
              << reach << " along a beam that ends at " << HALF_LENGTH
              << ", contact residual " << drift << '\n';

    REQUIRE(touched);
    REQUIRE(approaching > 0.0f);
    REQUIRE(carrying < HEADROOM * approaching);
    REQUIRE(reach < HALF_LENGTH - BOX_HALF);
    REQUIRE(drift < SETTLED);
    REQUIRE(supported > FRAMES / 2u);
  }
}

TEST_CASE(
  "a deformable beam holds a heavy crate on its corner",
  "[deformable]"
) {
  constexpr float BOX_MASS = 5.0f;
  constexpr float BOX_HALF = 0.05f;
  constexpr uint32_t FRAMES = 120u;
  constexpr uint32_t TAIL = 20u;
  constexpr float SETTLED = 2.0e-3f;
  constexpr float REACH = 0.3f;

  for (const bool hex : {false, true}) {
    JoltLibrary library;

    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);

    Runtime runtime(source, library);

    talus::deformable::Settings settings = dynamicSolve(4u);

    settings.element = hex ? talus::deformable::VolumeElementType::Hex
                           : talus::deformable::VolumeElementType::Tet;

    runtime.setDeformableSettings(settings);

    prepare(runtime, beam);
    settleAtEquilibrium(runtime, beam);

    const float resting = surfaceAt(runtime, beam, 0.4f);

    const JPH::BodyID box =
      dropCrate(
        runtime,
        glm::vec3(0.4f, resting + BOX_HALF + 0.002f, HALF_THICKNESS),
        BOX_HALF,
        BOX_MASS
      )
        .body;

    JPH::BodyInterface& bodies = runtime.physics().bodies();

    float worstKinetic = 0.0f;
    float lateKinetic = 0.0f;
    float drift = 0.0f;
    float penetration = 0.0f;
    uint32_t supported = 0u;
    uint32_t lateSupported = 0u;

    for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
      runtime.step();

      const talus::deformable::StepReport& report = runtime.deformableReport();

      worstKinetic = glm::max(worstKinetic, report.kineticEnergy);
      drift = glm::max(drift, report.contactResidual);
      penetration = glm::max(penetration, report.maxPenetration);

      if (report.contacts != 0u) {
        ++supported;
      }

      if (frame >= FRAMES - TAIL) {
        lateKinetic = glm::max(lateKinetic, report.kineticEnergy);

        if (report.contacts != 0u) {
          ++lateSupported;
        }
      }

      const DeformableState* state = runtime.world().deformableState(beam);

      REQUIRE(state != nullptr);

      for (size_t node = 0u; node != state->position.size(); ++node) {
        REQUIRE(std::isfinite(glm::length(state->position[node])));
        REQUIRE(std::isfinite(glm::length(state->velocity[node])));
      }
    }

    const float voxel =
      glm::compMax(runtime.world().deformablePrepared(beam)->voxelSize);

    std::cout << "deformable: a " << (hex ? "hex" : "tet") << " beam under a "
              << BOX_MASS << " kg corner load peaks at " << worstKinetic
              << " J, ends under " << lateKinetic << " J, contact residual "
              << drift << ", penetration " << penetration << " against a "
              << voxel << " voxel, supported on " << supported << " of "
              << FRAMES << " steps, awake " << (bodies.IsActive(box) ? 1 : 0)
              << ", " << lateSupported << " of the last " << TAIL
              << ", crate ending at " << bodies.GetPosition(box).GetX() << " "
              << bodies.GetPosition(box).GetY() << " "
              << bodies.GetPosition(box).GetZ() << "\n";

    const bool carried = bodies.GetPosition(box).GetY() > resting - BOX_HALF;

    REQUIRE(worstKinetic < BOX_MASS * PhysicsWorld::GRAVITY * REACH);
    REQUIRE(lateKinetic < worstKinetic);
    REQUIRE(drift < SETTLED);
    REQUIRE(penetration < voxel);
    REQUIRE(supported != 0u);

    if (carried) {
      REQUIRE(lateSupported != 0u);
    } else {
      REQUIRE(lateSupported == 0u);
    }
  }
}

TEST_CASE("deformable surface topology", "[.][hexcontact]") {
  std::array<std::vector<uint64_t>, 2> shape;

  for (const bool hex : {false, true}) {
    JoltLibrary library;
    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);
    Runtime runtime(source, library);

    talus::deformable::Settings settings = dynamicSolve(4u);

    settings.element = hex ? talus::deformable::VolumeElementType::Hex
                           : talus::deformable::VolumeElementType::Tet;

    runtime.setDeformableSettings(settings);
    prepare(runtime, beam);

    const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);

    const uint64_t stride = static_cast<uint64_t>(mesh.restPositions.size());

    double volume = 0.0;
    double area = 0.0;
    uint32_t degenerate = 0u;

    std::vector<uint64_t> directed;

    directed.reserve(mesh.surfaceTriangles.size() * 3u);

    for (const glm::uvec3& triangle : mesh.surfaceTriangles) {
      const glm::vec3& a = mesh.restPositions[triangle.x];
      const glm::vec3& b = mesh.restPositions[triangle.y];
      const glm::vec3& c = mesh.restPositions[triangle.z];

      const glm::vec3 across = glm::cross(b - a, c - a);
      const float size = glm::length(across);

      if (size <= 0.0f) {
        ++degenerate;
      }

      volume += static_cast<double>(glm::dot(a, across)) / 6.0;
      area += 0.5 * static_cast<double>(size);

      for (uint32_t corner = 0u; corner != 3u; ++corner) {
        const uint64_t from = triangle[static_cast<int>(corner)];
        const uint64_t to = triangle[static_cast<int>((corner + 1u) % 3u)];

        directed.push_back(from * stride + to);
      }
    }

    std::sort(directed.begin(), directed.end());

    uint32_t repeated = 0u;
    uint32_t unpaired = 0u;

    for (size_t at = 0u; at != directed.size();) {
      size_t next = at + 1u;

      while (next != directed.size() && directed[next] == directed[at]) {
        ++next;
      }

      if (next - at != 1u) {
        ++repeated;
      }

      const uint64_t from = directed[at] / stride;
      const uint64_t to = directed[at] % stride;

      if (!std::binary_search(
            directed.begin(),
            directed.end(),
            to * stride + from
          )) {
        ++unpaired;
      }

      at = next;
    }

    for (const glm::uvec3& triangle : mesh.surfaceTriangles) {
      std::array<uint64_t, 3> triple{triangle.x, triangle.y, triangle.z};

      std::sort(triple.begin(), triple.end());

      shape[hex ? 1u : 0u].push_back(
        (triple[0] * stride + triple[1]) * stride + triple[2]
      );
    }

    std::sort(shape[hex ? 1u : 0u].begin(), shape[hex ? 1u : 0u].end());

    std::cout << "surface: " << (hex ? "hex" : "tet") << " nodes "
              << mesh.restPositions.size() << " elements "
              << (hex ? mesh.hexes.size() : mesh.tets.size()) << " triangles "
              << mesh.surfaceTriangles.size() << " edges "
              << mesh.surfaceEdges.size() << " volume " << volume << " area "
              << area << " degenerate " << degenerate << " repeatedDirected "
              << repeated << " unpairedDirected " << unpaired << std::endl;
  }

  std::cout << "surface: identical triangle sets "
            << (shape[0] == shape[1] ? "yes" : "no") << std::endl;
}

float apartFrom(
  const std::vector<glm::vec3>& given,
  const std::vector<glm::vec3>& reference
) {
  if (given.size() != reference.size() || reference.empty()) {
    return -1.0f;
  }

  float difference = 0.0f;
  float size = 0.0f;

  for (size_t at = 0u; at != reference.size(); ++at) {
    difference +=
      glm::dot(given[at] - reference[at], given[at] - reference[at]);
    size += glm::dot(reference[at], reference[at]);
  }

  return size > 0.0f ? std::sqrt(difference / size) : 0.0f;
}

struct Frozen {
  std::vector<double> matrix;
  std::vector<double> right;
  size_t rows = 0u;
};

struct Verdict {
  std::vector<double> lambda;
  double worstNegative = 0.0;
  double worstSlack = 0.0;
  double worstProduct = 0.0;
  uint32_t changes = 0u;
};

bool choleskySolve(
  const std::vector<double>& given,
  const std::vector<double>& right,
  std::vector<double>& answer,
  size_t order
) {
  std::vector<double> work(given);

  for (size_t i = 0u; i != order; ++i) {
    double pivot = work[i * order + i];

    for (size_t k = 0u; k != i; ++k) {
      pivot -= work[i * order + k] * work[i * order + k];
    }

    if (!(pivot > 1.0e-300)) {
      return false;
    }

    work[i * order + i] = std::sqrt(pivot);

    for (size_t j = i + 1u; j != order; ++j) {
      double sum = work[j * order + i];

      for (size_t k = 0u; k != i; ++k) {
        sum -= work[j * order + k] * work[i * order + k];
      }

      work[j * order + i] = sum / work[i * order + i];
    }
  }

  answer.assign(order, 0.0);

  for (size_t i = 0u; i != order; ++i) {
    double sum = right[i];

    for (size_t k = 0u; k != i; ++k) {
      sum -= work[i * order + k] * answer[k];
    }

    answer[i] = sum / work[i * order + i];
  }

  for (size_t step = 0u; step != order; ++step) {
    const size_t i = order - 1u - step;
    double sum = answer[i];

    for (size_t k = i + 1u; k != order; ++k) {
      sum -= work[k * order + i] * answer[k];
    }

    answer[i] = sum / work[i * order + i];
  }

  return true;
}

Verdict solveActiveSet(const Frozen& held) {
  Verdict found;

  const size_t rows = held.rows;

  found.lambda.assign(rows, 0.0);

  std::vector<bool> active(rows, false);

  for (uint32_t turn = 0u; turn != 4u * static_cast<uint32_t>(rows) + 64u;
       ++turn) {
    std::vector<size_t> chosen;

    for (size_t i = 0u; i != rows; ++i) {
      if (active[i]) {
        chosen.push_back(i);
      }
    }

    std::vector<double> block(chosen.size() * chosen.size(), 0.0);
    std::vector<double> piece(chosen.size(), 0.0);

    for (size_t a = 0u; a != chosen.size(); ++a) {
      piece[a] = held.right[chosen[a]];

      for (size_t b = 0u; b != chosen.size(); ++b) {
        block[a * chosen.size() + b] =
          held.matrix[chosen[a] * rows + chosen[b]];
      }
    }

    std::vector<double> solved;

    if (
      !chosen.empty() && !choleskySolve(block, piece, solved, chosen.size())
    ) {
      break;
    }

    found.lambda.assign(rows, 0.0);

    for (size_t a = 0u; a != chosen.size(); ++a) {
      found.lambda[chosen[a]] = solved[a];
    }

    size_t drop = rows;
    double worst = -1.0e-12;

    for (size_t a = 0u; a != chosen.size(); ++a) {
      if (found.lambda[chosen[a]] < worst) {
        worst = found.lambda[chosen[a]];
        drop = chosen[a];
      }
    }

    if (drop != rows) {
      active[drop] = false;
      ++found.changes;

      continue;
    }

    size_t add = rows;
    double lowest = -1.0e-10;

    for (size_t i = 0u; i != rows; ++i) {
      if (active[i]) {
        continue;
      }

      double slack = -held.right[i];

      for (size_t j = 0u; j != rows; ++j) {
        slack += held.matrix[i * rows + j] * found.lambda[j];
      }

      if (slack < lowest) {
        lowest = slack;
        add = i;
      }
    }

    if (add == rows) {
      break;
    }

    active[add] = true;
    ++found.changes;
  }

  for (size_t i = 0u; i != rows; ++i) {
    double slack = -held.right[i];

    for (size_t j = 0u; j != rows; ++j) {
      slack += held.matrix[i * rows + j] * found.lambda[j];
    }

    found.worstNegative = glm::min(found.worstNegative, found.lambda[i]);
    found.worstSlack = glm::min(found.worstSlack, slack);
    found.worstProduct =
      glm::max(found.worstProduct, std::abs(found.lambda[i] * slack));
  }

  return found;
}

std::vector<double> sweepPgs(const Frozen& held, uint32_t passes) {
  std::vector<double> lambda(held.rows, 0.0);
  std::vector<double> current(held.rows, 0.0);

  for (uint32_t pass = 0u; pass != passes; ++pass) {
    for (size_t i = 0u; i != held.rows; ++i) {
      const double diagonal = held.matrix[i * held.rows + i];

      if (!(diagonal > 0.0)) {
        continue;
      }

      const double violation = held.right[i] - current[i];

      double next = lambda[i] + 0.8 * violation / diagonal;

      next = glm::max(0.0, next);

      const double change = next - lambda[i];

      if (change == 0.0) {
        continue;
      }

      lambda[i] = next;

      for (size_t r = 0u; r != held.rows; ++r) {
        current[r] += held.matrix[r * held.rows + i] * change;
      }
    }
  }

  return lambda;
}

double lambdaApart(
  const std::vector<double>& given,
  const std::vector<double>& reference
) {
  double difference = 0.0;
  double size = 0.0;

  for (size_t i = 0u; i != reference.size(); ++i) {
    difference += (given[i] - reference[i]) * (given[i] - reference[i]);
    size += reference[i] * reference[i];
  }

  return size > 0.0 ? std::sqrt(difference / size) : 0.0;
}

TEST_CASE("deformable contact against an exact oracle", "[.][passes]") {
  constexpr float BOX_HALF = 0.05f;

  for (const bool hex : {false, true}) {
    JoltLibrary library;
    World source;
    const Entity beam = beamEntity(source, COARSE_VOXEL);
    Runtime runtime(source, library);

    talus::deformable::Settings settings = dynamicSolve(4u);

    settings.element = hex ? talus::deformable::VolumeElementType::Hex
                           : talus::deformable::VolumeElementType::Tet;
    settings.reportContactSystem = true;

    runtime.setDeformableSettings(settings);
    prepare(runtime, beam);
    settleAtEquilibrium(runtime, beam);

    const float resting = surfaceAt(runtime, beam, 0.4f);

    dropCrate(
      runtime,
      glm::vec3(0.4f, resting + BOX_HALF + 0.002f, HALF_THICKNESS),
      BOX_HALF,
      5.0f
    );

    Frozen held;

    for (uint32_t frame = 0u; frame != 60u && held.rows == 0u; ++frame) {
      runtime.step();

      const talus::deformable::StepReport& report = runtime.deformableReport();

      if (report.contactRight.empty()) {
        continue;
      }

      const size_t full = report.contactRight.size();
      const size_t count = full / 3u;

      held.rows = count;
      held.matrix.assign(count * count, 0.0);
      held.right.assign(count, 0.0);

      for (size_t a = 0u; a != count; ++a) {
        held.right[a] = static_cast<double>(report.contactRight[3u * a]);

        for (size_t b = 0u; b != count; ++b) {
          held.matrix[a * count + b] = static_cast<double>(
            report.contactOperator[(3u * a) * full + 3u * b]
          );
        }
      }
    }

    REQUIRE(held.rows > 0u);

    const Verdict exact = solveActiveSet(held);

    std::cout << "oracle: " << (hex ? "hex" : "tet") << " normals " << held.rows
              << " setChanges " << exact.changes << " worstNegativeLambda "
              << exact.worstNegative << " worstSlack " << exact.worstSlack
              << " worstProduct " << exact.worstProduct << "\n";

    for (const uint32_t passes : {8u, 16u, 32u, 64u, 128u, 1024u}) {
      const std::vector<double> approximate = sweepPgs(held, passes);

      std::cout << "oracle: " << (hex ? "hex" : "tet") << " pgs " << passes
                << " lambdaError " << lambdaApart(approximate, exact.lambda)
                << "\n";
    }
  }
}

TEST_CASE("a deformable body caches its element stiffness", "[deformable]") {
  constexpr uint32_t SETTLE = 12u;

  JoltLibrary library;
  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);
  Runtime runtime(source, library);

  runtime.setDeformableSettings(dynamicSolve(4u));
  prepare(runtime, beam);

  runtime.step();

  const uint32_t first = runtime.deformableReport().stiffnessRebuilds;

  REQUIRE(first == 1u);

  uint32_t later = 0u;

  for (uint32_t frame = 0u; frame != SETTLE; ++frame) {
    runtime.step();

    later += runtime.deformableReport().stiffnessRebuilds;
  }

  REQUIRE(later == 0u);

  const DeformableState* state = runtime.world().deformableState(beam);

  REQUIRE(state != nullptr);

  std::vector<glm::vec3> before = state->position;

  runtime.step();

  REQUIRE(runtime.deformableReport().stiffnessRebuilds == 0u);

  const float moved = apartFrom(state->position, before);

  std::cout << "stiffnesscache: firstRebuild " << first << " laterRebuilds "
            << later << " stillMoving " << moved << std::endl;

  REQUIRE(moved > 0.0f);
}

TEST_CASE(
  "a deformable body rebuilds stiffness for a new material",
  "[deformable]"
) {
  constexpr uint32_t SETTLE = 10u;
  constexpr uint32_t AFTER = 20u;

  JoltLibrary library;
  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);
  Runtime runtime(source, library);

  JoltLibrary steadyLibrary;
  World steadySource;
  const Entity steadyBeam = beamEntity(steadySource, COARSE_VOXEL);
  Runtime steady(steadySource, steadyLibrary);

  runtime.setDeformableSettings(dynamicSolve(4u));
  steady.setDeformableSettings(dynamicSolve(4u));
  prepare(runtime, beam);
  prepare(steady, steadyBeam);

  uint32_t settled = 0u;

  for (uint32_t frame = 0u; frame != SETTLE; ++frame) {
    runtime.step();
    steady.step();

    if (frame != 0u) {
      settled += runtime.deformableReport().stiffnessRebuilds;
    }
  }

  REQUIRE(settled == 0u);

  const std::span<const PhysicalMaterial> was = runtime.world().materials();

  std::vector<PhysicalMaterial> changed(was.begin(), was.end());

  REQUIRE(!changed.empty());

  for (PhysicalMaterial& material : changed) {
    material.youngsModulus *= 8.0f;
  }

  runtime.world().setMaterials(changed);
  runtime.physics().impacts().setMaterials(runtime.world().materials());

  runtime.step();
  steady.step();

  const uint32_t onChange = runtime.deformableReport().stiffnessRebuilds;

  REQUIRE(onChange == 1u);

  uint32_t again = 0u;

  for (uint32_t frame = 0u; frame != AFTER; ++frame) {
    runtime.step();
    steady.step();

    again += runtime.deformableReport().stiffnessRebuilds;
  }

  REQUIRE(again == 0u);

  const DeformableState* stiffened = runtime.world().deformableState(beam);
  const DeformableState* original = steady.world().deformableState(steadyBeam);

  REQUIRE(stiffened != nullptr);
  REQUIRE(original != nullptr);

  const float apart = apartFrom(stiffened->position, original->position);

  std::cout << "stiffnessmaterial: settledRebuilds " << settled << " onChange "
            << onChange << " afterwards " << again << " responseApart " << apart
            << std::endl;

  REQUIRE(apart > 1.0e-4f);
}

TEST_CASE(
  "a deformable contact response matches its own system",
  "[deformable]"
) {
  constexpr uint32_t FRAMES = 40u;

  JoltLibrary library;
  World source;

  const Entity block = blockBody(source, 0.05f, COARSE_VOXEL);

  fieldBlocker(source, glm::vec3(0.0f, 0.25f, 0.0f));

  Runtime runtime(source, library);

  talus::deformable::Settings settings = dynamicSolve(4u);

  settings.element = talus::deformable::VolumeElementType::Hex;
  settings.reportContactSystem = true;

  runtime.setDeformableSettings(settings);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  REQUIRE(runtime.world().deformablePrepared(block) != nullptr);

  float worst = 0.0f;
  float settled = 0.0f;
  uint32_t measured = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();

    const talus::deformable::StepReport& report = runtime.deformableReport();

    if (report.contacts == 0u) {
      continue;
    }

    ++measured;
    worst = glm::max(worst, report.contactResponseResidual);

    if (frame + 10u >= FRAMES) {
      settled = glm::max(settled, report.contactResponseResidual);
    }
  }

  std::cout << "alpharesponse: measured on " << measured << " of " << FRAMES
            << " frames, worst relative residual " << worst << ", settled "
            << settled << std::endl;

  REQUIRE(measured > FRAMES / 2u);
  REQUIRE(worst < 0.5f);
  REQUIRE(settled < 0.1f);
}

namespace {

constexpr uint32_t FRICTION_TAIL = 20u;

struct FrictionBalance {
  double dissipated = 0.0;
  double injected = 0.0;
  float worstSliding = -1.0e30f;
  float worstLeak = -1.0e30f;
  float worstSlipGap = -1.0e30f;
  float settledSlip = 0.0f;
  uint32_t settledFrames = 0u;
  uint32_t frozenAt = 0u;
  float worstCone = 0.0f;
  float penetration = 0.0f;
  uint32_t measured = 0u;
  uint32_t sliding = 0u;
  uint32_t checked = 0u;
  uint32_t cappedFrames = 0u;
  float worstClaim = -1.0e30f;
  uint32_t peakPasses = 0u;
  uint32_t restFrames = 0u;
  uint64_t restPasses = 0u;
  double restSweepSeconds = 0.0;
};

float resolvedAnchor(
  const talus::deformable::ContactCheck& at,
  float tolerance
) {
  return glm::max(tolerance * at.residualScale, at.residualNorm);
}

float resolvedSlip(const talus::deformable::ContactCheck& at, float tolerance) {
  return at.tangentCompliance * resolvedAnchor(at, tolerance) / at.area;
}

float slipGap(const talus::deformable::ContactCheck& at) {
  return glm::length(at.solverTangentSlip / at.area - at.tangentAdvance);
}

float frictionLeak(const talus::deformable::ContactCheck& at, float tolerance) {
  const float scale = 2.0f * resolvedAnchor(at, tolerance);
  const float slip = glm::length(at.tangentAdvance) * at.area;
  const float pull = at.tangentImpulse;
  const float compliance = at.tangentCompliance;

  const float leak = scale * (compliance * (pull + scale) + slip);
  const float rounding = 8.0f * std::numeric_limits<float>::epsilon() * pull
                         * (compliance * pull + slip);

  return (leak + rounding) / at.area;
}

FrictionBalance frictionBalance(
  bool hex,
  float tolerance,
  uint32_t frames,
  float speed,
  talus::deformable::ContactOperator contactOperator,
  float spectralRadius
) {
  FrictionBalance held;

  std::vector<float> contactSlip;

  JoltLibrary library;
  World source;

  const Entity block = blockBody(source, 0.05f, COARSE_VOXEL);

  fieldBlocker(source, glm::vec3(0.0f, 0.25f, 0.0f));

  Runtime runtime(source, library);

  talus::deformable::Settings settings;

  settings.element = hex ? talus::deformable::VolumeElementType::Hex
                         : talus::deformable::VolumeElementType::Tet;
  settings.reportContactSystem = true;
  settings.contactTolerance = tolerance;
  settings.contactIterations = tolerance < 1.0e-4f ? 4096u : 256u;
  settings.contactOperator = contactOperator;
  settings.spectralRadius = spectralRadius;

  runtime.setDeformableSettings(settings);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  const size_t nodes =
    runtime.world().deformablePrepared(block)->restPositions.size();

  runtime.world().patchDeformableState(block, [&](DeformableState& state) {
    for (size_t node = 0u; node != nodes; ++node) {
      state.velocity[node] = glm::vec3(speed, 0.0f, 0.0f);
    }
  });

  for (uint32_t frame = 0u; frame != frames; ++frame) {
    runtime.step();

    const talus::deformable::StepReport& report = runtime.deformableReport();

    const bool awake = held.frozenAt == 0u;

    if (awake && runtime.deformableQuiescent(block)) {
      held.frozenAt = frame + 1u;
    }

    held.penetration = glm::max(held.penetration, report.maxPenetration);
    held.peakPasses = glm::max(held.peakPasses, report.contactPasses);

    if (report.kineticEnergy <= 1.0e-7f) {
      ++held.restFrames;
      held.restPasses += report.contactTotalPasses;
      held.restSweepSeconds += report.contactSweepSeconds;
    }

    float biggest = 0.0f;

    const bool converged = report.contactPasses < settings.contactIterations;

    held.cappedFrames += converged ? 0u : 1u;

    for (const talus::deformable::ContactCheck& at : report.contactChecks) {
      biggest = glm::max(biggest, at.normalImpulse);

      if (!(at.area > 0.0f)) {
        continue;
      }

      ++held.checked;

      if (converged) {
        const float claim = at.residualNorm - tolerance * at.residualScale;

        held.worstClaim = std::isnan(claim)
                            ? std::numeric_limits<float>::infinity()
                            : glm::max(held.worstClaim, claim);
      }

      const bool finite = std::isfinite(resolvedAnchor(at, tolerance));
      const float gap = slipGap(at) - resolvedSlip(at, tolerance);
      const float leak = at.frictionWork - frictionLeak(at, tolerance);

      held.worstSlipGap = !finite || std::isnan(gap)
                            ? std::numeric_limits<float>::infinity()
                            : glm::max(held.worstSlipGap, gap);
      held.worstLeak = !finite || std::isnan(leak)
                         ? std::numeric_limits<float>::infinity()
                         : glm::max(held.worstLeak, leak);
    }

    float frameSlip = -1.0f;

    for (const talus::deformable::ContactCheck& at : report.contactChecks) {
      if (at.normalImpulse <= 0.05f * biggest) {
        continue;
      }

      ++held.measured;

      frameSlip = glm::max(frameSlip, at.advanceTangent);

      const float limit = at.friction * at.normalImpulse;

      if (limit > 0.0f) {
        held.worstCone =
          glm::max(held.worstCone, (at.tangentImpulse - limit) / limit);

        if (at.tangentImpulse >= 0.999f * limit) {
          ++held.sliding;
          held.worstSliding = glm::max(held.worstSliding, at.frictionWork);
        }
      }

      if (at.frictionWork < 0.0f) {
        held.dissipated -= at.frictionWork;
      } else {
        held.injected += at.frictionWork;
      }
    }

    if (awake && frameSlip >= 0.0f) {
      contactSlip.push_back(frameSlip);
    }
  }

  const size_t tail = std::min<size_t>(FRICTION_TAIL, contactSlip.size());

  held.settledFrames = static_cast<uint32_t>(tail);

  for (size_t row = contactSlip.size() - tail; row != contactSlip.size();
       ++row) {
    held.settledSlip = glm::max(held.settledSlip, contactSlip[row]);
  }

  return held;
}
}

TEST_CASE("a deformable contact dissipates through friction", "[deformable]") {
  constexpr uint32_t FRAMES = 100u;

  for (const bool hex : {false, true}) {
    const FrictionBalance shipped = frictionBalance(
      hex,
      1.0e-3f,
      FRAMES,
      0.5f,
      talus::deformable::Settings{}.contactOperator,
      talus::deformable::Settings{}.spectralRadius
    );

    std::cout << "frictionwork: " << (hex ? "hex" : "tet") << " shipped"
              << " measured " << shipped.measured << " sliding "
              << shipped.sliding << " worstSlidingWork " << shipped.worstSliding
              << " worstLeakExcess " << shipped.worstLeak << " slipGap "
              << shipped.worstSlipGap << " checked " << shipped.checked
              << " injectedFraction " << shipped.injected / shipped.dissipated
              << " cone " << shipped.worstCone << " penetration "
              << shipped.penetration << " settledSlip " << shipped.settledSlip
              << " settledFrames " << shipped.settledFrames << " frozenAt "
              << shipped.frozenAt << " cappedFrames " << shipped.cappedFrames
              << " worstClaim " << shipped.worstClaim << " peakPasses "
              << shipped.peakPasses << " restFrames " << shipped.restFrames
              << " restPasses " << shipped.restPasses << " restSweepSeconds "
              << shipped.restSweepSeconds << std::endl;

    REQUIRE(shipped.measured > 20u);
    REQUIRE(shipped.sliding > 10u);
    REQUIRE(shipped.dissipated > 1.0e-3);

    REQUIRE(shipped.checked > 20u);
    REQUIRE(shipped.cappedFrames * 10u <= FRAMES);
    REQUIRE(shipped.worstClaim <= 0.0f);
    REQUIRE(shipped.worstSlipGap <= 0.0f);
    REQUIRE(shipped.worstLeak <= 0.0f);
    REQUIRE(shipped.injected / shipped.dissipated < 5.0e-3);
    REQUIRE(shipped.worstCone < 1.0e-5f);
    REQUIRE(shipped.penetration == 0.0f);
    REQUIRE(shipped.settledFrames == FRICTION_TAIL);
    REQUIRE(shipped.settledSlip < 5.0e-3f);
  }
}

TEST_CASE(
  "a deformable contact dissipates through friction at a converged tolerance",
  "[.][frictionconverged]"
) {
  constexpr uint32_t FRAMES = 100u;
  constexpr int SPEEDS = 24;

  for (const float spectralRadius : {0.5f, 0.8f}) {
    for (const talus::deformable::ContactOperator contactOperator :
         {talus::deformable::ContactOperator::Dense,
          talus::deformable::ContactOperator::Unbuilt}) {
      for (const bool hex : {false, true}) {
        for (int k = 0; k != SPEEDS; ++k) {
          const FrictionBalance converged = frictionBalance(
            hex,
            1.0e-5f,
            FRAMES,
            0.5f * (1.0f + 2.0e-7f * static_cast<float>(k)),
            contactOperator,
            spectralRadius
          );

          std::cout
            << "frictionwork: " << (hex ? "hex" : "tet") << " converged "
            << (contactOperator == talus::deformable::ContactOperator::Dense
                  ? "dense"
                  : "unbuilt")
            << " spectralRadius " << spectralRadius << " k " << k
            << " measured " << converged.measured << " sliding "
            << converged.sliding << " worstSlidingWork "
            << converged.worstSliding << " worstLeakExcess "
            << converged.worstLeak << " slipGap " << converged.worstSlipGap
            << " checked " << converged.checked << " injectedFraction "
            << converged.injected / converged.dissipated << " cappedFrames "
            << converged.cappedFrames << " worstClaim " << converged.worstClaim
            << " peakPasses " << converged.peakPasses << " restFrames "
            << converged.restFrames << " restPasses " << converged.restPasses
            << " restSweepSeconds " << converged.restSweepSeconds << std::endl;

          REQUIRE(converged.measured > 20u);
          REQUIRE(converged.sliding > 10u);
          REQUIRE(converged.dissipated > 1.0e-3);

          REQUIRE(converged.checked > 20u);
          REQUIRE(converged.cappedFrames * 10u <= FRAMES);
          REQUIRE(converged.worstClaim <= 0.0f);
          REQUIRE(converged.worstSlipGap <= 0.0f);
          REQUIRE(converged.worstLeak <= 0.0f);
          REQUIRE(converged.injected / converged.dissipated < 1.0e-4);
        }
      }
    }
  }
}

namespace {

constexpr int CARRY_POLAR_PASSES = 64;

struct CarryGap {
  float difference = 0.0f;
  float unturned = 0.0f;
  float size = 0.0f;
  size_t contacts = 0u;
  uint32_t posed = 0u;
  double poseAngle = 0.0;
};

glm::dmat3 carryPolar(const glm::dmat3& gradient) {
  glm::dmat3 rotation = gradient;

  for (int pass = 0; pass != CARRY_POLAR_PASSES; ++pass) {
    rotation = 0.5 * (rotation + glm::transpose(glm::inverse(rotation)));
  }

  return rotation;
}

std::vector<glm::vec3> posedCarry(
  const PreparedMesh& mesh,
  const DeformableState& apart,
  const DeformableState& touched,
  double& poseAngle
) {
  const size_t nodes = apart.position.size();

  double total = 0.0;
  glm::dvec3 posedCentre(0.0);
  glm::dvec3 freeCentre(0.0);
  glm::dvec3 velocity(0.0);
  glm::dvec3 acceleration(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    total += mass;
    posedCentre += mass * glm::dvec3(touched.position[node]);
    freeCentre += mass * glm::dvec3(apart.position[node]);
    velocity += mass * glm::dvec3(apart.velocity[node]);
    acceleration += mass * glm::dvec3(apart.acceleration[node]);
  }

  posedCentre /= total;
  freeCentre /= total;
  velocity /= total;
  acceleration /= total;

  glm::dmat3 covariance(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    covariance += static_cast<double>(mesh.nodeMass[node])
                  * glm::outerProduct(
                    glm::dvec3(touched.position[node]) - posedCentre,
                    glm::dvec3(apart.position[node]) - freeCentre
                  );
  }

  const glm::dmat3 turn = carryPolar(covariance);
  const glm::dmat3 back = glm::transpose(turn);

  poseAngle = 0.5
              * glm::length(
                glm::dvec3(
                  turn[1][2] - turn[2][1],
                  turn[2][0] - turn[0][2],
                  turn[0][1] - turn[1][0]
                )
              );

  std::vector<glm::dvec3> arm(nodes);
  std::vector<glm::dvec3> drift(nodes);

  glm::dmat3 inertia(0.0);
  glm::dvec3 momentum(0.0);
  glm::dvec3 torque(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    arm[node] = back * (glm::dvec3(touched.position[node]) - posedCentre);
    inertia +=
      mass
      * (glm::dot(arm[node], arm[node]) * glm::dmat3(1.0) - glm::outerProduct(arm[node], arm[node]));
    momentum +=
      mass * glm::cross(arm[node], glm::dvec3(apart.velocity[node]) - velocity);
    torque += mass
              * glm::cross(
                arm[node],
                glm::dvec3(apart.acceleration[node]) - acceleration
              );
  }

  const glm::dmat3 inverse = glm::inverse(inertia);
  const glm::dvec3 spin = inverse * momentum;

  glm::dvec3 gyroscopic(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const double mass = static_cast<double>(mesh.nodeMass[node]);

    drift[node] =
      glm::dvec3(apart.velocity[node]) - velocity - glm::cross(spin, arm[node]);
    gyroscopic += mass
                  * glm::cross(
                    arm[node],
                    glm::cross(spin, glm::cross(spin, arm[node]))
                      + 2.0 * glm::cross(spin, drift[node])
                  );
  }

  const glm::dvec3 rate = inverse * (torque - gyroscopic);
  const glm::dmat3 turnedInverse = turn * inverse * back;
  const glm::dvec3 reposed = turnedInverse * momentum;

  glm::dvec3 reposedGyroscopic(0.0);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 placed = turn * arm[node];
    const glm::dvec3 moving = turn * drift[node];

    reposedGyroscopic += static_cast<double>(mesh.nodeMass[node])
                         * glm::cross(
                           placed,
                           glm::cross(reposed, glm::cross(reposed, placed))
                             + 2.0 * glm::cross(reposed, moving)
                         );
  }

  const glm::dvec3 reposedRate = turnedInverse * (torque - reposedGyroscopic);

  std::vector<glm::vec3> expected(nodes);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::dvec3 placed = turn * arm[node];
    const glm::dvec3 moving = turn * drift[node];
    const glm::dvec3 deforming = glm::dvec3(apart.acceleration[node])
                                 - acceleration - glm::cross(rate, arm[node])
                                 - glm::cross(spin, glm::cross(spin, arm[node]))
                                 - 2.0 * glm::cross(spin, drift[node]);

    expected[node] = glm::vec3(
      acceleration + glm::cross(reposedRate, placed)
      + glm::cross(reposed, glm::cross(reposed, placed))
      + 2.0 * glm::cross(reposed, moving) + turn * deforming
    );
  }

  return expected;
}

CarryGap carryGap(
  const PreparedMesh& mesh,
  const std::vector<PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  const DeformableState& from
) {
  const talus::field::SampledSdf slab = landingbar::landingSlab();

  DeformableState touched = from;
  DeformableState apart = from;

  const std::vector<talus::deformable::SurfaceContact> contacts =
    landingbar::landingContacts(mesh, touched, materials, slab);
  std::vector<talus::deformable::RigidPartner> partners(
    1u,
    landingbar::landingPartner(materials)
  );

  auto touchedSolver =
    std::make_unique<talus::deformable::DeformableSolverState>();
  auto apartSolver =
    std::make_unique<talus::deformable::DeformableSolverState>();
  talus::deformable::StepReport touchedReport;
  talus::deformable::StepReport apartReport;

  landingbar::landingStep(
    mesh,
    touched,
    materials,
    settings,
    contacts,
    partners,
    touchedSolver.get(),
    touchedReport
  );
  landingbar::landingStep(
    mesh,
    apart,
    materials,
    settings,
    {},
    {},
    apartSolver.get(),
    apartReport
  );

  CarryGap gap;

  gap.contacts = contacts.size();
  gap.posed = touchedReport.posedSubsteps;

  const std::vector<glm::vec3> expected =
    gap.posed != 0u ? posedCarry(mesh, apart, touched, gap.poseAngle)
                    : apart.acceleration;

  for (size_t node = 0u; node != apart.acceleration.size(); ++node) {
    gap.difference = glm::max(
      gap.difference,
      glm::length(touched.acceleration[node] - expected[node])
    );
    gap.unturned = glm::max(
      gap.unturned,
      glm::length(touched.acceleration[node] - apart.acceleration[node])
    );
    gap.size = glm::max(gap.size, glm::length(apart.acceleration[node]));
  }

  return gap;
}

}

TEST_CASE(
  "a landing deformable's acceleration holds only its smooth solve",
  "[.][contactsplit]"
) {
  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());

  for (const float spectralRadius : {0.5f, 0.8f}) {
    for (const talus::deformable::VolumeElementType element :
         plasticbar::ELEMENTS) {
      const std::shared_ptr<const PreparedMesh> mesh =
        landingbar::landingBar(materials, element);

      talus::deformable::Settings settings = plasticbar::barStep(element);

      settings.spectralRadius = spectralRadius;

      REQUIRE(settings.substeps == 1u);

      const landingbar::LandingRun run =
        landingbar::landBar(*mesh, materials, settings, 0.0f);

      REQUIRE(run.firstContact != 0u);

      for (const bool resting : {false, true}) {
        const CarryGap gap = carryGap(
          *mesh,
          materials,
          settings,
          resting ? run.endState : run.contactState
        );

        std::cout << "contactsplit carry: " << plasticbar::elementName(element)
                  << " spectralRadius " << settings.spectralRadius
                  << " substeps " << settings.substeps << " contactIterations "
                  << settings.contactIterations << " contactTolerance "
                  << settings.contactTolerance << " contactOperator "
                  << (settings.contactOperator
                          == talus::deformable::ContactOperator::Dense
                        ? "dense"
                        : "unbuilt")
                  << " state " << (resting ? "rest" : "impact") << " frame "
                  << (resting ? landingbar::LANDING_FRAMES : run.firstContact)
                  << " contacts " << gap.contacts << " posedSubsteps "
                  << gap.posed << " expected "
                  << (gap.posed != 0u ? "turnedByPose" : "identity")
                  << " poseAngleRad " << gap.poseAngle
                  << " largestNodeDifference " << gap.difference
                  << " accelerationSize " << gap.size << " ratio "
                  << gap.difference / gap.size << " unturnedDifference "
                  << gap.unturned << " unturnedRatio "
                  << gap.unturned / gap.size << std::endl;

        REQUIRE(gap.contacts > 0u);
        REQUIRE(gap.difference <= 1.0e-6f * gap.size);
      }
    }
  }
}

TEST_CASE(
  "a deformable bar dropped face down comes to rest on a field slab",
  "[.][contactsplit]"
) {
  constexpr double MAIN_REST_HEIGHT_MM = 7.5;
  constexpr double IMPACT_TOLERANCE = 1.0e-6;

  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());

  std::vector<landingbar::LandingRun> runs;

  for (const float spectralRadius : {0.5f, 0.8f}) {
    for (const talus::deformable::VolumeElementType element :
         plasticbar::ELEMENTS) {
      const std::shared_ptr<const PreparedMesh> mesh =
        landingbar::landingBar(materials, element);

      talus::deformable::Settings settings = plasticbar::barStep(element);

      settings.spectralRadius = spectralRadius;

      const landingbar::LandingRun run =
        landingbar::landBar(*mesh, materials, settings, 0.0f);

      std::cout
        << "contactsplit rest: " << plasticbar::elementName(element)
        << " spectralRadius " << settings.spectralRadius << " substeps "
        << settings.substeps << " contactIterations "
        << settings.contactIterations << " contactTolerance "
        << settings.contactTolerance << " contactOperator "
        << (settings.contactOperator
                == talus::deformable::ContactOperator::Dense
              ? "dense"
              : "unbuilt")
        << " frames " << landingbar::LANDING_FRAMES << " firstContactFrame "
        << run.firstContact << " restFrame " << run.restFrame << " atRest "
        << (run.atRest ? "yes" : "no") << " restWindowMaxSpeed "
        << run.restWindowSpeed << " tailWindowMaxSpeed " << run.tailWindowSpeed
        << " restSpeed " << landingbar::LANDING_REST_SPEED << " maxPenetration "
        << run.maxPenetration << " endCentreY " << run.endHeight
        << " endHeightMm "
        << 1000.0
             * (run.endHeight - static_cast<double>(plasticbar::HALF_WIDTH))
        << " mainEndHeightMm " << MAIN_REST_HEIGHT_MM
        << " peakAfterContactRatio " << run.peakAfterContact
        << " firstNonFinite " << run.firstNonFinite << std::endl;

      std::cout << "contactsplit energy: " << plasticbar::elementName(element)
                << " spectralRadius " << settings.spectralRadius << " releaseJ "
                << run.release << " impacts " << run.impacts
                << " worstImpactFrame " << run.worstImpactFrame
                << " worstImpactPlusNextFrameChangeJ " << run.worstImpactChange
                << " overRelease " << run.worstImpactChange / run.release
                << " tolerance " << IMPACT_TOLERANCE << " reboundSpeed "
                << run.reboundSpeed << std::endl;

      runs.push_back(run);
    }
  }

  for (const landingbar::LandingRun& run : runs) {
    REQUIRE(run.atRest);
    REQUIRE(run.maxPenetration == 0.0f);
    REQUIRE(run.tailWindowSpeed < landingbar::LANDING_REST_SPEED);
    REQUIRE(run.impacts > 0u);
    REQUIRE(run.worstImpactChange <= IMPACT_TOLERANCE * run.release);
  }
}

namespace {

constexpr float LANDING_LINE_RATE = 20.0f;
constexpr double LANDING_LINE_PEAK = 1.01;
constexpr double LANDING_LINE_IMPACT_TOLERANCE = 1.0e-6;

void landingLine(const char* family, const glm::vec3& axis) {
  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());

  std::vector<landingbar::LandingRun> runs;

  for (const float spectralRadius : {0.5f, 0.8f}) {
    for (const talus::deformable::VolumeElementType element :
         plasticbar::ELEMENTS) {
      const std::shared_ptr<const PreparedMesh> mesh =
        landingbar::landingBar(materials, element);

      talus::deformable::Settings settings = plasticbar::barStep(element);

      settings.spectralRadius = spectralRadius;

      const landingbar::LandingRun run = landingbar::landBar(
        *mesh,
        materials,
        settings,
        LANDING_LINE_RATE,
        axis,
        landingbar::LANDING_WIDE_SLAB
      );

      std::ostringstream impacts;

      for (size_t impact = 0u; impact != run.impactChanges.size(); ++impact) {
        impacts << " f" << run.impactFrames[impact] << ' '
                << run.impactChanges[impact] << ' '
                << run.impactChanges[impact] / run.release;
      }

      std::cout << "landingline: " << family << ' '
                << plasticbar::elementName(element) << " rate "
                << LANDING_LINE_RATE << " axis " << axis.x << ',' << axis.y
                << ',' << axis.z << " slabHalf "
                << landingbar::LANDING_WIDE_SLAB << " spectralRadius "
                << settings.spectralRadius << " substeps " << settings.substeps
                << " contactIterations " << settings.contactIterations
                << " contactTolerance " << settings.contactTolerance
                << " contactOperator "
                << (settings.contactOperator
                        == talus::deformable::ContactOperator::Dense
                      ? "dense"
                      : "unbuilt")
                << " frames " << landingbar::LANDING_FRAMES << " releaseJ "
                << run.release << " peakEnergyRatio " << run.peak
                << " peakFrame " << run.peakFrame << " peakBar "
                << LANDING_LINE_PEAK << " firstContactFrame "
                << run.firstContact << " atFirstContactKineticJ "
                << run.contactKinetic << " atFirstContactStrainJ "
                << run.contactStrain << " impacts " << run.impacts
                << " impactPending " << (run.impactPending ? "yes" : "no")
                << " worstImpactFrame " << run.worstImpactFrame
                << " worstImpactPlusNextFrameChangeJ " << run.worstImpactChange
                << " overRelease " << run.worstImpactChange / run.release
                << " impactToleranceOfRelease " << LANDING_LINE_IMPACT_TOLERANCE
                << " atRest " << (run.atRest ? "yes" : "no")
                << " restWindowMaxSpeed " << run.restWindowSpeed
                << " endCentreY " << run.endHeight << " aboveSlab "
                << (run.aboveSlab ? "yes" : "no") << " maxPenetration "
                << run.maxPenetration << " velocityContactPassesMax "
                << run.velocityPasses << " velocityContactTotalPasses "
                << run.velocityTotalPasses << " velocityContactResidualMax "
                << run.velocityResidual << " velocityContactCapped "
                << run.velocityCapped << " posedSubsteps " << run.posedSubsteps
                << " posedSolves " << run.posedSolves << " posedCapped "
                << run.posedCapped << " posedKineticChangeJ "
                << run.posedKineticChange << " firstNonFinite "
                << run.firstNonFinite << " impactPlusNextFrameChangeJ"
                << impacts.str() << std::endl;

      runs.push_back(run);
    }
  }

  for (const landingbar::LandingRun& run : runs) {
    REQUIRE(run.firstNonFinite == 0u);
    REQUIRE(run.peak <= LANDING_LINE_PEAK);
    REQUIRE(run.impacts > 0u);
    REQUIRE(!run.impactPending);

    for (const double change : run.impactChanges) {
      REQUIRE(change <= LANDING_LINE_IMPACT_TOLERANCE * run.release);
    }

    REQUIRE(run.atRest);
    REQUIRE(run.aboveSlab);
  }
}

}

TEST_CASE(
  "a free deformable bar spun about its long axis lands on a wide slab without "
  "gaining energy",
  "[.][landingline]"
) {
  std::cout << "landingline old write: arm CT hex long 20 peakEnergyRatio "
               "1.33, arm C hex 10 peakEnergyRatio 1.10"
            << std::endl;

  landingLine("long", glm::vec3(1.0f, 0.0f, 0.0f));
}

TEST_CASE(
  "a free deformable bar tumbling about a skew axis lands on a wide slab "
  "without gaining energy",
  "[.][landingline]"
) {
  landingLine("skew", glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)));
}

TEST_CASE(
  "a free deformable bar takes a dynamic partner's impulse as its momentum",
  "[deformable]"
) {
  constexpr float PARTNER_MASS = 2.0f;
  constexpr float PARTNER_SPEED = 1.0f;
  constexpr float PARTNER_CLEARANCE = 5.0e-3f;
  constexpr double MOMENTUM_TOLERANCE = 1.0e-5;

  const Clock clock;

  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());
  const talus::field::SampledSdf slab = landingbar::landingSlab();

  const glm::vec3 slabSize(
    1.2f,
    2.0f * landingbar::LANDING_SLAB_HALF_HEIGHT,
    1.2f
  );
  const glm::vec3 slabInertia =
    PARTNER_MASS / 12.0f
    * glm::vec3(
      slabSize.y * slabSize.y + slabSize.z * slabSize.z,
      slabSize.x * slabSize.x + slabSize.z * slabSize.z,
      slabSize.x * slabSize.x + slabSize.y * slabSize.y
    );

  std::vector<std::pair<double, double>> exchanges;
  std::vector<talus::deformable::StepReport> velocitySolves;
  std::vector<talus::deformable::StepReport> idleReports;
  std::vector<float> velocityTolerances;

  for (const float spectralRadius : {0.5f, 0.8f}) {
    for (const talus::deformable::VolumeElementType element :
         plasticbar::ELEMENTS) {
      const std::shared_ptr<const PreparedMesh> mesh =
        landingbar::landingBar(materials, element);

      talus::deformable::Settings settings = plasticbar::barStep(element);

      settings.spectralRadius = spectralRadius;

      DeformableState state =
        talus::deformable::restState(*mesh, talus::objectmodel::Pose{});

      const std::vector<glm::vec3> before = state.velocity;

      talus::deformable::ShapeProbe probe;

      probe.field = &slab;
      probe.centerOfMass = glm::vec3(
        0.0f,
        -plasticbar::HALF_WIDTH - landingbar::LANDING_SLAB_HALF_HEIGHT
          - PARTNER_CLEARANCE,
        0.0f
      );
      probe.linearVelocity = glm::vec3(0.0f, PARTNER_SPEED, 0.0f);

      const float voxel = std::max(
        mesh->voxelSize.x,
        std::max(mesh->voxelSize.y, mesh->voxelSize.z)
      );

      const std::vector<talus::deformable::SurfaceContact> contacts =
        talus::deformable::findContacts(
          *mesh,
          state,
          materials,
          glm::vec3(0.0f),
          std::span<const talus::deformable::ShapeProbe>(&probe, 1u),
          plasticbar::TIME_STEP,
          talus::deformable::ContactGaps{0.1f * voxel, 0.2f * voxel}
        );

      REQUIRE(!contacts.empty());

      std::vector<talus::deformable::RigidPartner> partners(1u);

      partners[0].centerOfMass = probe.centerOfMass;
      partners[0].linearVelocity = probe.linearVelocity;
      partners[0].originalLinearVelocity = probe.linearVelocity;
      partners[0].inverseMass = 1.0f / PARTNER_MASS;
      partners[0].inverseInertia = glm::mat3(
        glm::vec3(1.0f / slabInertia.x, 0.0f, 0.0f),
        glm::vec3(0.0f, 1.0f / slabInertia.y, 0.0f),
        glm::vec3(0.0f, 0.0f, 1.0f / slabInertia.z)
      );
      partners[0].friction = materials[plasticbar::LANE].friction;
      partners[0].dynamic = true;

      auto solver =
        std::make_unique<talus::deformable::DeformableSolverState>();
      talus::deformable::StepReport report;

      talus::deformable::step(
        *mesh,
        state,
        materials,
        settings,
        glm::vec3(0.0f),
        plasticbar::TIME_STEP,
        contacts,
        partners,
        solver.get(),
        report
      );

      glm::dvec3 gained(0.0);

      for (size_t node = 0u; node != state.velocity.size(); ++node) {
        gained += static_cast<double>(mesh->nodeMass[node])
                  * glm::dvec3(state.velocity[node] - before[node]);
      }

      const glm::dvec3 impulse(partners[0].impulse);
      const double mismatch = glm::length(gained + impulse);
      const double size = glm::length(impulse);

      std::cout << "deformable momentum: " << plasticbar::elementName(element)
                << " spectralRadius " << settings.spectralRadius << " substeps "
                << settings.substeps << " contactIterations "
                << settings.contactIterations << " contactTolerance "
                << settings.contactTolerance << " contactOperator "
                << (settings.contactOperator
                        == talus::deformable::ContactOperator::Dense
                      ? "dense"
                      : "unbuilt")
                << " contacts " << contacts.size() << " partnerImpulseY "
                << impulse.y << " barMomentumChangeY " << gained.y
                << " mismatch " << mismatch << " relative " << mismatch / size
                << " tolerance " << MOMENTUM_TOLERANCE << " partnerSpeedAfter "
                << partners[0].linearVelocity.y << " velocityContactPasses "
                << report.velocityContactPasses
                << " velocityContactTotalPasses "
                << report.velocityContactTotalPasses
                << " velocityContactResidual " << report.velocityContactResidual
                << " velocityContactCapped " << report.velocityContactCapped
                << " velocityResponseApplyMs "
                << 1000.0 * report.velocityResponseApplySeconds << std::endl;

      exchanges.emplace_back(mismatch, size);
      velocitySolves.push_back(report);
      velocityTolerances.push_back(settings.contactTolerance);

      DeformableState idle =
        talus::deformable::restState(*mesh, talus::objectmodel::Pose{});
      auto idleSolver =
        std::make_unique<talus::deformable::DeformableSolverState>();
      const std::vector<talus::deformable::SurfaceContact> noContacts;
      std::vector<talus::deformable::RigidPartner> noPartners;
      talus::deformable::StepReport idleReport;

      talus::deformable::step(
        *mesh,
        idle,
        materials,
        settings,
        glm::vec3(0.0f),
        plasticbar::TIME_STEP,
        noContacts,
        noPartners,
        idleSolver.get(),
        idleReport
      );

      idleReports.push_back(idleReport);
    }
  }

  std::cout << "deformable momentum: " << clock.seconds()
            << " wall seconds in this tree" << std::endl;

  for (const auto& [mismatch, size] : exchanges) {
    REQUIRE(size > 0.0);
    REQUIRE(mismatch <= MOMENTUM_TOLERANCE * size);
  }

  for (size_t run = 0u; run != velocitySolves.size(); ++run) {
    const talus::deformable::StepReport& pressed = velocitySolves[run];

    REQUIRE(pressed.velocityContactPasses >= 1u);
    REQUIRE(
      pressed.velocityContactTotalPasses >= pressed.velocityContactPasses
    );
    REQUIRE(pressed.velocityContactCapped == 0u);
    REQUIRE(pressed.velocityContactResidual <= velocityTolerances[run]);
  }

  for (const talus::deformable::StepReport& untouched : idleReports) {
    REQUIRE(untouched.contactRows == 0u);
    REQUIRE(untouched.velocityContactPasses == 0u);
    REQUIRE(untouched.velocityContactTotalPasses == 0u);
    REQUIRE(untouched.velocityContactResidual == 0.0f);
    REQUIRE(untouched.velocityContactCapped == 0u);
    REQUIRE(untouched.velocityResponseApplySeconds == 0.0);
  }
}

TEST_CASE("a deformable contact tracks its displacement rate", "[deformable]") {
  constexpr uint32_t SETTLE = 12u;
  constexpr uint32_t FRAMES = 24u;

  JoltLibrary library;
  World source;

  const Entity block = blockBody(source, 0.05f, COARSE_VOXEL);

  fieldBlocker(source, glm::vec3(0.0f, 0.25f, 0.0f));

  Runtime runtime(source, library);

  talus::deformable::Settings settings;

  settings.substeps = 1u;
  settings.element = talus::deformable::VolumeElementType::Hex;
  settings.reportContactSystem = true;

  runtime.setDeformableSettings(settings);

  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(block);
  const size_t nodes = mesh.restPositions.size();

  runtime.world().patchDeformableState(block, [&](DeformableState& state) {
    for (size_t node = 0u; node != nodes; ++node) {
      state.velocity[node] = glm::vec3(0.3f, 0.0f, 0.0f);
    }
  });

  for (uint32_t frame = 0u; frame != SETTLE; ++frame) {
    runtime.step();
  }

  const DeformableState* state = runtime.world().deformableState(block);

  float worstApart = 0.0f;
  float biggestRate = 0.0f;
  uint32_t compared = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    const std::vector<glm::vec3> before = state->position;

    runtime.step();

    const talus::deformable::StepReport& report = runtime.deformableReport();

    REQUIRE(report.contactChecks.size() == report.contactFrames.size());

    for (size_t i = 0u; i != report.contactFrames.size(); ++i) {
      const talus::deformable::ContactFrame& at = report.contactFrames[i];
      const talus::deformable::ContactCheck& held = report.contactChecks[i];

      glm::vec3 moved(0.0f);

      for (int corner = 0; corner != 3; ++corner) {
        const uint32_t node = at.nodes[corner];

        if (mesh.anchored[node] != 0u) {
          continue;
        }

        moved += at.weights[corner] * (state->position[node] - before[node])
                 / PhysicsWorld::TIME_STEP;
      }

      const float rate = glm::length(
        glm::vec2(glm::dot(at.tangent, moved), glm::dot(at.bitangent, moved))
      );

      ++compared;
      biggestRate = glm::max(biggestRate, rate);
      worstApart = glm::max(worstApart, std::abs(rate - held.advanceTangent));
    }
  }

  std::cout << "sliprate: compared " << compared << " biggestRate "
            << biggestRate << " worstApart " << worstApart << std::endl;

  REQUIRE(compared > FRAMES);
  REQUIRE(biggestRate > 1.0e-4f);
  REQUIRE(worstApart < 1.0e-5f);
}

struct MgLevel {
  std::vector<glm::ivec3> coord;
  std::vector<float> mass;
  std::vector<uint8_t> anchored;
  std::vector<std::array<uint32_t, 8>> element;
  std::vector<glm::mat3> rotation;
  std::vector<uint32_t> lane;
  std::vector<talus::deformable::HexStiffness> stiffness;
  std::vector<glm::mat3> inverseDiagonal;
  std::vector<int32_t> nodeAt;
  glm::ivec3 low{0};
  glm::ivec3 span{0};
  glm::vec3 halfExtents{0.0f};
  float scale = 0.0f;
  float inertia = 1.0f;
};

void buildDiagonal(MgLevel& level) {
  const size_t nodes = level.mass.size();

  std::vector<glm::mat3> block(nodes, glm::mat3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    block[node] = glm::mat3(level.inertia * level.mass[node]);
  }

  for (size_t at = 0u; at != level.element.size(); ++at) {
    const std::array<glm::mat3, 8> corner =
      hexStiffnessDiagonal(level.stiffness[at], level.rotation[at]);

    for (uint32_t which = 0u; which != 8u; ++which) {
      block[level.element[at][which]] += level.scale * corner[which];
    }
  }

  level.inverseDiagonal.assign(nodes, glm::mat3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    if (level.anchored[node] != 0u) {
      continue;
    }

    glm::mat3 held = block[node];

    for (int axis = 0; axis != 3; ++axis) {
      if (!(held[axis][axis] > 0.0f)) {
        held[axis][axis] = 1.0f;
      }
    }

    level.inverseDiagonal[node] = glm::inverse(held);
  }
}

void applyLevel(
  const MgLevel& level,
  const std::vector<glm::vec3>& input,
  std::vector<glm::vec3>& output
) {
  const size_t nodes = level.mass.size();

  for (size_t node = 0u; node != nodes; ++node) {
    output[node] = level.inertia * level.mass[node] * input[node];
  }

  std::array<glm::vec3, 8> gathered{};
  std::array<glm::vec3, 8> produced{};

  for (size_t at = 0u; at != level.element.size(); ++at) {
    const std::array<uint32_t, 8>& corners = level.element[at];

    for (uint32_t which = 0u; which != 8u; ++which) {
      gathered[which] = input[corners[which]];
    }

    applyHexStiffness(
      gathered,
      level.rotation[at],
      level.stiffness[at],
      produced
    );

    for (uint32_t which = 0u; which != 8u; ++which) {
      output[corners[which]] += level.scale * produced[which];
    }
  }

  for (size_t node = 0u; node != nodes; ++node) {
    if (level.anchored[node] != 0u) {
      output[node] = glm::vec3(0.0f);
    }
  }
}

MgLevel buildFineLevel(
  const PreparedMesh& mesh,
  const DeformableState& state,
  std::span<const PhysicalMaterial> materials,
  float inner
) {
  MgLevel level;

  const size_t nodes = mesh.restPositions.size();

  level.scale = inner * inner;
  level.inertia = 1.0f;
  level.halfExtents = 0.5f * mesh.voxelSize;

  glm::vec3 least(std::numeric_limits<float>::max());

  for (const glm::vec3& at : mesh.restPositions) {
    least = glm::min(least, at);
  }

  level.coord.assign(nodes, glm::ivec3(0));

  glm::ivec3 most(0);

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::vec3 offset =
      (mesh.restPositions[node] - least) / mesh.voxelSize;

    level.coord[node] = glm::ivec3(
      static_cast<int>(std::lround(offset.x)),
      static_cast<int>(std::lround(offset.y)),
      static_cast<int>(std::lround(offset.z))
    );

    most = glm::max(most, level.coord[node]);
  }

  level.low = glm::ivec3(0);
  level.span = most + glm::ivec3(1);
  level.nodeAt.assign(
    static_cast<size_t>(level.span.x) * level.span.y * level.span.z,
    -1
  );

  for (size_t node = 0u; node != nodes; ++node) {
    const glm::ivec3& at = level.coord[node];

    level.nodeAt
      [static_cast<size_t>(at.x)
       + static_cast<size_t>(level.span.x)
           * (static_cast<size_t>(at.y) + static_cast<size_t>(level.span.y) * static_cast<size_t>(at.z))] =
      static_cast<int32_t>(node);
  }

  level.mass.assign(mesh.nodeMass.begin(), mesh.nodeMass.end());
  level.anchored.assign(mesh.anchored.begin(), mesh.anchored.end());

  for (size_t at = 0u; at != mesh.hexes.size(); ++at) {
    level.element.push_back(mesh.hexes[at].nodes);
    level.rotation.push_back(state.committed[at].rotation);
    level.lane.push_back(mesh.hexes[at].material);

    const PhysicalMaterial& held = mesh.hexes[at].material < materials.size()
                                     ? materials[mesh.hexes[at].material]
                                     : materials[0];

    level.stiffness.push_back(hexStiffness(
      mesh.hexes[at],
      talus::deformable::ElasticParams{
        held.youngsModulus,
        held.poissonsRatio,
        held.density
      }
    ));
  }

  buildDiagonal(level);

  return level;
}

TEST_CASE("deformable multigrid fine operator matches", "[.][mgcheck]") {
  JoltLibrary library;
  World source;
  const Entity beam = beamEntity(source, BASE_VOXEL);
  Runtime runtime(source, library);

  talus::deformable::Settings settings = dynamicSolve(4u);

  settings.element = talus::deformable::VolumeElementType::Hex;

  runtime.setDeformableSettings(settings);
  prepare(runtime, beam);

  for (uint32_t frame = 0u; frame != 8u; ++frame) {
    runtime.step();
  }

  const PreparedMesh* mesh = runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);

  REQUIRE(mesh != nullptr);
  REQUIRE(state != nullptr);
  REQUIRE(!mesh->hexes.empty());

  const float inner = PhysicsWorld::TIME_STEP / 4.0f;

  const MgLevel fine =
    buildFineLevel(*mesh, *state, runtime.world().materials(), inner);

  talus::deformable::SystemFactor factor;

  analyzeSystem(factor, *mesh);
  refreshFactor(
    factor,
    *mesh,
    *state,
    runtime.world().materials(),
    inner,
    1.0f
  );

  REQUIRE(!factor.empty());

  const size_t nodes = mesh->restPositions.size();

  std::vector<glm::vec3> wanted(nodes, glm::vec3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh->anchored[node] != 0u) {
      continue;
    }

    const float spin = static_cast<float>(node % 71u);

    wanted[node] = glm::vec3(
      0.001f * (spin - 35.0f),
      0.002f * (spin - 10.0f),
      0.0015f * (spin - 50.0f)
    );
  }

  std::vector<glm::vec3> solved(wanted);

  applyFactor(factor, glm::mat3(1.0f), *mesh, solved);

  std::vector<glm::vec3> back(nodes, glm::vec3(0.0f));

  applyLevel(fine, solved, back);

  float apart = 0.0f;
  float size = 0.0f;

  for (size_t node = 0u; node != nodes; ++node) {
    apart += glm::dot(back[node] - wanted[node], back[node] - wanted[node]);
    size += glm::dot(wanted[node], wanted[node]);
  }

  std::cout << "mgcheck: nodes " << nodes << " elements " << mesh->hexes.size()
            << " span " << fine.span.x << "x" << fine.span.y << "x"
            << fine.span.z << " roundTripRelative "
            << (size > 0.0f ? std::sqrt(apart / size) : -1.0f) << std::endl;

  REQUIRE(size > 0.0f);
  REQUIRE(std::sqrt(apart / size) < 1.0e-3f);
}

struct AlphaRule {
  double alphaMass = 0.0;
  double alphaForce = 0.0;
  double beta = 0.25;
  double gamma = 0.5;
};

AlphaRule alphaRuleOf(double spectral) {
  AlphaRule made;

  made.alphaMass = (2.0 * spectral - 1.0) / (spectral + 1.0);
  made.alphaForce = spectral / (spectral + 1.0);
  made.gamma = 0.5 - made.alphaMass + made.alphaForce;
  made.beta = 0.25 * (1.0 - made.alphaMass + made.alphaForce)
              * (1.0 - made.alphaMass + made.alphaForce);

  return made;
}

struct Ringer {
  const PreparedMesh* mesh = nullptr;
  std::vector<talus::deformable::HexStiffness> stiffness;
  std::vector<glm::mat3> rotation;
  std::vector<glm::vec3> position;
  std::vector<glm::vec3> velocity;
  std::vector<glm::vec3> acceleration;
  std::vector<glm::mat3> inverseDiagonal;
  float scale = 0.0f;
};

float forceOf(
  Ringer& beam,
  const std::vector<glm::vec3>& position,
  std::vector<glm::vec3>& force
) {
  const PreparedMesh& mesh = *beam.mesh;

  std::fill(force.begin(), force.end(), glm::vec3(0.0f));

  float energy = 0.0f;

  std::array<glm::vec3, 8> deformed{};
  std::array<glm::vec3, 8> rest{};

  for (size_t at = 0u; at != mesh.hexes.size(); ++at) {
    const std::array<uint32_t, 8>& corners = mesh.hexes[at].nodes;

    for (uint32_t which = 0u; which != 8u; ++which) {
      deformed[which] = position[corners[which]];
      rest[which] = mesh.restPositions[corners[which]];
    }

    const talus::deformable::HexTrial made =
      hexTrial(deformed, rest, mesh.hexes[at], beam.stiffness[at]);

    beam.rotation[at] = made.rotation;
    energy += made.energy;

    for (uint32_t which = 0u; which != 8u; ++which) {
      force[corners[which]] += made.force[which];
    }
  }

  for (size_t node = 0u; node != force.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      force[node] = glm::vec3(0.0f);
    }
  }

  return energy;
}

void applyRinger(
  const Ringer& beam,
  float scale,
  const std::vector<glm::vec3>& input,
  std::vector<glm::vec3>& output
) {
  const PreparedMesh& mesh = *beam.mesh;

  for (size_t node = 0u; node != input.size(); ++node) {
    output[node] = mesh.nodeMass[node] * input[node];
  }

  std::array<glm::vec3, 8> gathered{};
  std::array<glm::vec3, 8> produced{};

  for (size_t at = 0u; at != mesh.hexes.size(); ++at) {
    const std::array<uint32_t, 8>& corners = mesh.hexes[at].nodes;

    for (uint32_t which = 0u; which != 8u; ++which) {
      gathered[which] = input[corners[which]];
    }

    applyHexStiffness(
      gathered,
      beam.rotation[at],
      beam.stiffness[at],
      produced
    );

    for (uint32_t which = 0u; which != 8u; ++which) {
      output[corners[which]] += scale * produced[which];
    }
  }

  for (size_t node = 0u; node != output.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      output[node] = glm::vec3(0.0f);
    }
  }
}

void alphaAdvance(
  const AlphaRule& rule,
  double step,
  double stiffness,
  double& position,
  double& velocity,
  double& acceleration
) {
  const double shifted =
    position + step * velocity + step * step * (0.5 - rule.beta) * acceleration;
  const double weighted =
    (1.0 - rule.alphaForce) * shifted + rule.alphaForce * position;
  const double couple = (1.0 - rule.alphaForce) * rule.beta * step * step;

  const double right = -stiffness * weighted - rule.alphaMass * acceleration;
  const double left = (1.0 - rule.alphaMass) + stiffness * couple;

  const double next = right / left;

  position = shifted + rule.beta * step * step * next;
  velocity += step * ((1.0 - rule.gamma) * acceleration + rule.gamma * next);
  acceleration = next;
}

double spectralRadiusOf(const AlphaRule& rule, double step, double stiffness) {
  std::array<std::array<double, 3>, 3> map{};

  for (size_t column = 0u; column != 3u; ++column) {
    double position = column == 0u ? 1.0 : 0.0;
    double velocity = column == 1u ? 1.0 : 0.0;
    double acceleration = column == 2u ? 1.0 : 0.0;

    alphaAdvance(rule, step, stiffness, position, velocity, acceleration);

    map[0][column] = position;
    map[1][column] = velocity;
    map[2][column] = acceleration;
  }

  std::array<double, 3> probe{1.0, 0.7, -0.3};

  constexpr uint32_t TURNS = 20000u;

  double carried = 0.0;
  uint32_t counted = 0u;

  for (uint32_t turn = 0u; turn != TURNS; ++turn) {
    std::array<double, 3> next{};

    for (size_t row = 0u; row != 3u; ++row) {
      next[row] = map[row][0] * probe[0] + map[row][1] * probe[1]
                  + map[row][2] * probe[2];
    }

    const double size =
      std::sqrt(next[0] * next[0] + next[1] * next[1] + next[2] * next[2]);

    if (!(size > 1.0e-300)) {
      return 0.0;
    }

    if (turn >= TURNS / 10u) {
      carried += std::log(size);
      ++counted;
    }

    for (size_t row = 0u; row != 3u; ++row) {
      probe[row] = next[row] / size;
    }
  }

  return counted != 0u ? std::exp(carried / static_cast<double>(counted)) : 0.0;
}

TEST_CASE("deformable generalized alpha spectrum", "[.][genalpha]") {
  for (const double spectral : {1.0, 0.8, 0.5, 0.2, 0.0}) {
    const AlphaRule rule = alphaRuleOf(spectral);

    std::cout << "genalpha: rhoInf " << spectral << " alphaMass "
              << rule.alphaMass << " alphaForce " << rule.alphaForce << " beta "
              << rule.beta << " gamma " << rule.gamma << " secondOrder "
              << std::abs(rule.gamma - (0.5 - rule.alphaMass + rule.alphaForce))
              << std::endl;

    std::cout << "genspectrum: rhoInf " << spectral;

    for (const double product : {0.01, 0.1, 1.0, 10.0, 100.0, 1.0e4, 1.0e8}) {
      const double stiffness = product * product;

      std::cout << " " << spectralRadiusOf(rule, 1.0, stiffness);
    }

    std::cout << std::endl;
  }
}

TEST_CASE(
  "deformable generalized alpha response definiteness",
  "[.][alpharesponse]"
) {
  JoltLibrary library;
  World source;
  const Entity beam = beamEntity(source, COARSE_VOXEL);
  Runtime runtime(source, library);

  talus::deformable::Settings settings = dynamicSolve(4u);

  settings.element = talus::deformable::VolumeElementType::Hex;

  runtime.setDeformableSettings(settings);
  prepare(runtime, beam);

  for (uint32_t frame = 0u; frame != 8u; ++frame) {
    runtime.step();
  }

  const PreparedMesh& mesh = *runtime.world().deformablePrepared(beam);
  const DeformableState* state = runtime.world().deformableState(beam);
  const size_t nodes = mesh.restPositions.size();

  Ringer beam189;

  beam189.mesh = &mesh;
  beam189.rotation.assign(mesh.hexes.size(), glm::mat3(1.0f));
  beam189.position = state->position;
  beam189.velocity.assign(nodes, glm::vec3(0.0f));
  beam189.acceleration.assign(nodes, glm::vec3(0.0f));

  for (const talus::objectmodel::PreparedHex& hex : mesh.hexes) {
    const PhysicalMaterial& held =
      hex.material < runtime.world().materials().size()
        ? runtime.world().materials()[hex.material]
        : runtime.world().materials()[0];

    beam189.stiffness.push_back(hexStiffness(
      hex,
      talus::deformable::ElasticParams{
        held.youngsModulus,
        held.poissonsRatio,
        held.density
      }
    ));
  }

  std::vector<glm::vec3> force(nodes, glm::vec3(0.0f));

  forceOf(beam189, beam189.position, force);

  std::vector<glm::vec3> probe(nodes, glm::vec3(0.0f));
  std::vector<glm::vec3> pushed(nodes, glm::vec3(0.0f));

  for (size_t node = 0u; node != nodes; ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    probe[node] = glm::vec3(
      static_cast<float>((node * 31u) % 17u) - 8.0f,
      static_cast<float>((node * 11u) % 13u) - 6.0f,
      static_cast<float>((node * 7u) % 19u) - 9.0f
    );
  }

  float largest = 0.0f;

  for (uint32_t turn = 0u; turn != 600u; ++turn) {
    applyRinger(beam189, 1.0f, probe, pushed);

    for (size_t node = 0u; node != nodes; ++node) {
      pushed[node] -= mesh.nodeMass[node] * probe[node];
      pushed[node] = mesh.anchored[node] != 0u
                       ? glm::vec3(0.0f)
                       : pushed[node] / mesh.nodeMass[node];
    }

    float size = 0.0f;

    for (const glm::vec3& at : pushed) {
      size += glm::dot(at, at);
    }

    size = std::sqrt(size);

    if (!(size > 0.0f)) {
      break;
    }

    largest = size;

    for (size_t node = 0u; node != nodes; ++node) {
      probe[node] = pushed[node] / size;
    }
  }

  std::cout << "alpharesponse: nodes " << nodes << " maxMassStiffnessRatio "
            << largest << std::endl;

  for (const uint32_t substeps : {1u, 2u, 4u}) {
    const float inner = PhysicsWorld::TIME_STEP / static_cast<float>(substeps);

    for (const double spectral : {1.0, 0.8, 0.5, 0.2, 0.0}) {
      const AlphaRule rule = alphaRuleOf(spectral);
      const double product = static_cast<double>(inner) * inner * largest;
      const double bracket =
        (1.0 - rule.alphaMass)
        + (1.0 - rule.alphaForce) * (rule.beta - rule.gamma) * product;

      std::cout << "alpharesponse: substeps " << substeps << " rhoInf "
                << spectral << " hSquaredLambda " << product
                << " startOfStepBracket " << bracket << " definite "
                << (bracket > 0.0 ? 1 : 0) << std::endl;
    }
  }
}

namespace {

namespace damping {

constexpr double SPECTRAL = 0.5;
constexpr double ALPHA_MASS = (2.0 * SPECTRAL - 1.0) / (SPECTRAL + 1.0);
constexpr double ALPHA_FORCE = SPECTRAL / (SPECTRAL + 1.0);
constexpr double GAMMA = 0.5 - ALPHA_MASS + ALPHA_FORCE;
constexpr double BETA =
  0.25 * (1.0 - ALPHA_MASS + ALPHA_FORCE) * (1.0 - ALPHA_MASS + ALPHA_FORCE);
constexpr float FRAME = 1.0f / 60.0f;

constexpr uint32_t PLAIN_LANE = 1u;
constexpr uint32_t DAMPED_LANE = 2u;

constexpr float BAR_VOXEL = 0.025f;
constexpr float BAR_HALF_LENGTH = 0.075f;
constexpr float BAR_HALF_THICKNESS = 0.025f;
constexpr float BAR_YOUNGS = 1.0e7f;
constexpr float BAR_POISSONS = 0.3f;
constexpr float BAR_DENSITY = 500.0f;
constexpr float HOLD_TOLERANCE = 1.0e-4f;
constexpr float PRE_STRAIN = 0.01f;

constexpr float OPERATOR_TAU = 1.265e-2f;
constexpr double OPERATOR_BAR = 1.0e-5;

enum class Hold : uint8_t {
  Free,
  Hinge,
  Clamp
};

double weightOf(double tau, double inner) {
  return (1.0 - ALPHA_FORCE) * GAMMA * inner * tau;
}

double reachOf(double inner) {
  return (1.0 - ALPHA_FORCE) * BETA * inner * inner;
}

std::vector<PhysicalMaterial> barLanes(float plain, float damped) {
  std::vector<PhysicalMaterial> made(3);

  for (const uint32_t lane : {PLAIN_LANE, DAMPED_LANE}) {
    made[lane].density = BAR_DENSITY;
    made[lane].youngsModulus = BAR_YOUNGS;
    made[lane].poissonsRatio = BAR_POISSONS;
  }

  made[PLAIN_LANE].stiffnessDamping = plain;
  made[DAMPED_LANE].stiffnessDamping = damped;

  return made;
}

std::shared_ptr<const PreparedMesh> bar(
  std::span<const PhysicalMaterial> lanes,
  talus::deformable::VolumeElementType element,
  Hold hold,
  bool split
) {
  const float skin = 0.2f * BAR_VOXEL;
  const glm::vec3 half(
    BAR_HALF_LENGTH + skin,
    BAR_HALF_THICKNESS + skin,
    BAR_HALF_THICKNESS + skin
  );

  talus::field::Primitive primitive = fixture::box(glm::vec3(0.0f), half);

  primitive.shapeConfig.y = split ? PLAIN_LANE : DAMPED_LANE;

  const glm::vec3 minimum(
    -BAR_HALF_LENGTH - BAR_VOXEL,
    -BAR_HALF_THICKNESS - BAR_VOXEL,
    -BAR_HALF_THICKNESS - BAR_VOXEL
  );
  const glm::uvec3 resolution(
    static_cast<uint32_t>(std::lround(2.0f * BAR_HALF_LENGTH / BAR_VOXEL)) + 3u,
    static_cast<uint32_t>(std::lround(2.0f * BAR_HALF_THICKNESS / BAR_VOXEL))
      + 3u,
    static_cast<uint32_t>(std::lround(2.0f * BAR_HALF_THICKNESS / BAR_VOXEL))
      + 3u
  );

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    minimum,
    -minimum,
    resolution
  );

  const glm::vec3 low = hold == Hold::Hinge
                          ? glm::vec3(-HOLD_TOLERANCE, -2.0f, -HOLD_TOLERANCE)
                          : glm::vec3(-2.0f, -2.0f, -2.0f);
  const glm::vec3 high =
    hold == Hold::Hinge
      ? glm::vec3(HOLD_TOLERANCE, 2.0f, HOLD_TOLERANCE)
      : glm::vec3(-BAR_HALF_LENGTH + HOLD_TOLERANCE, 2.0f, 2.0f);

  const talus::deformable::SupportVolume support{
    glm::translate(glm::mat4(1.0f), -0.5f * (low + high)),
    0.5f * (high - low)
  };

  const std::shared_ptr<const PreparedMesh> built =
    talus::deformable::buildMesh(
      lattice,
      lanes,
      glm::mat4(1.0f),
      hold == Hold::Free
        ? std::span<const talus::deformable::SupportVolume>{}
        : std::span<const talus::deformable::SupportVolume>(&support, 1u),
      element
    );

  REQUIRE(built != nullptr);
  REQUIRE(built->restPositions.size() == 63u);

  if (!split) {
    return built;
  }

  PreparedMesh made = *built;

  for (talus::objectmodel::PreparedHex& hex : made.hexes) {
    float along = 0.0f;

    for (const uint32_t node : hex.nodes) {
      along += made.restPositions[node].x;
    }

    hex.material = along > 0.0f ? DAMPED_LANE : PLAIN_LANE;
  }

  for (talus::objectmodel::PreparedTet& tet : made.tets) {
    float along = 0.0f;

    for (int corner = 0; corner != 4; ++corner) {
      along += made.restPositions[tet.nodes[corner]].x;
    }

    tet.material = along > 0.0f ? DAMPED_LANE : PLAIN_LANE;
  }

  return std::make_shared<const PreparedMesh>(std::move(made));
}

talus::deformable::Settings settingsFor(
  talus::deformable::VolumeElementType element
) {
  talus::deformable::Settings made;

  made.element = element;

  return made;
}

const char* nameOf(talus::deformable::VolumeElementType element) {
  return element == talus::deformable::VolumeElementType::Hex ? "hex" : "tet";
}

struct Dense {
  size_t rows = 0u;
  std::vector<double> values;

  explicit Dense(size_t count) : rows(count), values(count * count, 0.0) {}

  [[nodiscard]] double& at(size_t row, size_t column) {
    return values[row * rows + column];
  }

  [[nodiscard]] double at(size_t row, size_t column) const {
    return values[row * rows + column];
  }
};

std::vector<size_t> freeRows(const PreparedMesh& mesh) {
  std::vector<size_t> made;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    for (size_t axis = 0u; axis != 3u; ++axis) {
      made.push_back(3u * node + axis);
    }
  }

  return made;
}

glm::dmat3 skewOf(const glm::dvec3& arm) {
  return glm::dmat3(
    glm::dvec3(0.0, arm.z, -arm.y),
    glm::dvec3(-arm.z, 0.0, arm.x),
    glm::dvec3(arm.y, -arm.x, 0.0)
  );
}

void addElement(
  Dense& target,
  std::span<const uint32_t> corners,
  std::span<const glm::mat3> blocks,
  const glm::dmat3& rotation,
  std::span<const glm::vec3> positions,
  double stiffnessScale,
  double dampingScale
) {
  const size_t count = corners.size();
  const size_t size = 3u * count;

  std::vector<double> stiffness(size * size, 0.0);

  for (size_t row = 0u; row != count; ++row) {
    for (size_t column = 0u; column != count; ++column) {
      const glm::dmat3 turned = rotation
                                * glm::dmat3(blocks[row * count + column])
                                * glm::transpose(rotation);

      for (int r = 0; r != 3; ++r) {
        for (int c = 0; c != 3; ++c) {
          stiffness
            [(3u * row + static_cast<size_t>(r)) * size + 3u * column
             + static_cast<size_t>(c)] = turned[c][r];
        }
      }
    }
  }

  std::vector<double> element(size * size, 0.0);

  for (size_t at = 0u; at != element.size(); ++at) {
    element[at] = stiffnessScale * stiffness[at];
  }

  if (dampingScale != 0.0) {
    glm::dvec3 centre(0.0);

    for (const uint32_t node : corners) {
      centre += glm::dvec3(positions[node]);
    }

    centre /= static_cast<double>(count);

    std::vector<glm::dmat3> skews(count);
    glm::dmat3 inertia(0.0);

    for (size_t corner = 0u; corner != count; ++corner) {
      const glm::dvec3 arm = glm::dvec3(positions[corners[corner]]) - centre;

      skews[corner] = skewOf(arm);
      inertia +=
        glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm);
    }

    const glm::dmat3 inverse = glm::inverse(inertia);

    std::vector<double> complement(size * size, 0.0);

    for (size_t row = 0u; row != count; ++row) {
      for (size_t column = 0u; column != count; ++column) {
        const glm::dmat3 rigid = glm::dmat3(1.0 / static_cast<double>(count))
                                 - skews[row] * inverse * skews[column];

        for (int r = 0; r != 3; ++r) {
          for (int c = 0; c != 3; ++c) {
            complement
              [(3u * row + static_cast<size_t>(r)) * size + 3u * column
               + static_cast<size_t>(c)] =
                (row == column && r == c ? 1.0 : 0.0) - rigid[c][r];
          }
        }
      }
    }

    std::vector<double> half(size * size, 0.0);

    for (size_t row = 0u; row != size; ++row) {
      for (size_t middle = 0u; middle != size; ++middle) {
        const double left = stiffness[row * size + middle];

        for (size_t column = 0u; column != size; ++column) {
          half[row * size + column] +=
            left * complement[middle * size + column];
        }
      }
    }

    for (size_t row = 0u; row != size; ++row) {
      for (size_t middle = 0u; middle != size; ++middle) {
        const double left = complement[row * size + middle];

        for (size_t column = 0u; column != size; ++column) {
          element[row * size + column] +=
            dampingScale * left * half[middle * size + column];
        }
      }
    }
  }

  for (size_t row = 0u; row != count; ++row) {
    for (size_t column = 0u; column != count; ++column) {
      for (size_t r = 0u; r != 3u; ++r) {
        for (size_t c = 0u; c != 3u; ++c) {
          target.at(3u * corners[row] + r, 3u * corners[column] + c) +=
            element[(3u * row + r) * size + 3u * column + c];
        }
      }
    }
  }
}

double laneTau(std::span<const PhysicalMaterial> lanes, uint32_t lane) {
  if (lane >= lanes.size()) {
    return 0.0;
  }

  const float held = lanes[lane].stiffnessDamping;

  return std::isfinite(held) && held > 0.0f ? static_cast<double>(held) : 0.0;
}

Dense referenceSystem(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> lanes,
  std::span<const glm::vec3> positions,
  std::span<const glm::mat3> rotations,
  double inertia,
  double reach,
  double weight
) {
  Dense made(3u * mesh.restPositions.size());

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    for (size_t axis = 0u; axis != 3u; ++axis) {
      made.at(3u * node + axis, 3u * node + axis) +=
        inertia * static_cast<double>(mesh.nodeMass[node]);
    }
  }

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const talus::objectmodel::PreparedTet& tet = mesh.tets[element];
    const PhysicalMaterial& lane = lanes[tet.material];
    const talus::deformable::TetStiffness stiffness =
      talus::deformable::tetStiffness(
        tet,
        talus::deformable::ElasticParams{
          lane.youngsModulus,
          lane.poissonsRatio,
          lane.density
        }
      );
    const std::array<uint32_t, 4> corners{
      tet.nodes[0],
      tet.nodes[1],
      tet.nodes[2],
      tet.nodes[3]
    };

    addElement(
      made,
      corners,
      stiffness.block,
      glm::dmat3(rotations[element]),
      positions,
      reach,
      weight * laneTau(lanes, tet.material)
    );
  }

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];
    const PhysicalMaterial& lane = lanes[hex.material];
    const talus::deformable::HexStiffness stiffness =
      talus::deformable::hexStiffness(
        hex,
        talus::deformable::ElasticParams{
          lane.youngsModulus,
          lane.poissonsRatio,
          lane.density
        }
      );

    addElement(
      made,
      hex.nodes,
      stiffness.block,
      glm::dmat3(rotations[element]),
      positions,
      reach,
      weight * laneTau(lanes, hex.material)
    );
  }

  return made;
}

std::vector<glm::mat3> rotationsOf(const DeformableState& state) {
  std::vector<glm::mat3> made;

  for (const talus::objectmodel::CommittedTet& held : state.committed) {
    made.push_back(held.rotation);
  }

  return made;
}

template<typename Given>
double apart(
  const std::vector<size_t>& rows,
  const Given& given,
  const Dense& reference,
  const Dense& share
) {
  double difference = 0.0;
  double size = 0.0;

  for (const size_t row : rows) {
    for (const size_t column : rows) {
      const double gap = given(row, column) - reference.at(row, column);

      difference += gap * gap;
      size += share.at(row, column) * share.at(row, column);
    }
  }

  return size > 0.0 ? std::sqrt(difference / size) : std::sqrt(difference);
}

double diagonalApart(
  const PreparedMesh& mesh,
  const std::vector<glm::mat3>& given,
  const Dense& reference,
  const Dense& share
) {
  double difference = 0.0;
  double size = 0.0;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    for (size_t r = 0u; r != 3u; ++r) {
      for (size_t c = 0u; c != 3u; ++c) {
        const double gap =
          static_cast<double>(
            given[node][static_cast<int>(c)][static_cast<int>(r)]
          )
          - reference.at(3u * node + r, 3u * node + c);
        const double held = share.at(3u * node + r, 3u * node + c);

        difference += gap * gap;
        size += held * held;
      }
    }
  }

  return size > 0.0 ? std::sqrt(difference / size) : std::sqrt(difference);
}

double sweepApart(
  const std::vector<size_t>& rows,
  const std::vector<float>& sweep,
  const Dense& reference,
  const Dense& share
) {
  const size_t count = reference.rows;

  double difference = 0.0;
  double size = 0.0;

  for (const size_t column : rows) {
    for (const size_t row : rows) {
      double product = 0.0;
      double damped = 0.0;

      for (const size_t middle : rows) {
        if (middle / 3u > row / 3u) {
          continue;
        }

        const double held = static_cast<double>(sweep[middle * count + column]);

        product += reference.at(row, middle) * held;
        damped += share.at(row, middle) * held;
      }

      const double gap = product - (row == column ? 1.0 : 0.0);

      difference += gap * gap;
      size += damped * damped;
    }
  }

  return size > 0.0 ? std::sqrt(difference / size) : std::sqrt(difference);
}

std::vector<double> inverted(
  const Dense& matrix,
  const std::vector<size_t>& rows
) {
  const size_t count = rows.size();

  std::vector<double> work(count * 2u * count, 0.0);

  for (size_t row = 0u; row != count; ++row) {
    for (size_t column = 0u; column != count; ++column) {
      work[row * 2u * count + column] = matrix.at(rows[row], rows[column]);
    }

    work[row * 2u * count + count + row] = 1.0;
  }

  for (size_t pivot = 0u; pivot != count; ++pivot) {
    size_t best = pivot;

    for (size_t row = pivot + 1u; row != count; ++row) {
      if (
        std::abs(work[row * 2u * count + pivot])
        > std::abs(work[best * 2u * count + pivot])
      ) {
        best = row;
      }
    }

    for (size_t column = 0u; column != 2u * count; ++column) {
      std::swap(
        work[pivot * 2u * count + column],
        work[best * 2u * count + column]
      );
    }

    const double lead = work[pivot * 2u * count + pivot];

    for (size_t column = 0u; column != 2u * count; ++column) {
      work[pivot * 2u * count + column] /= lead;
    }

    for (size_t row = 0u; row != count; ++row) {
      if (row == pivot) {
        continue;
      }

      const double factor = work[row * 2u * count + pivot];

      if (factor == 0.0) {
        continue;
      }

      for (size_t column = 0u; column != 2u * count; ++column) {
        work[row * 2u * count + column] -=
          factor * work[pivot * 2u * count + column];
      }
    }
  }

  std::vector<double> made(count * count, 0.0);

  for (size_t row = 0u; row != count; ++row) {
    for (size_t column = 0u; column != count; ++column) {
      made[row * count + column] = work[row * 2u * count + count + column];
    }
  }

  return made;
}

double basisApart(
  const PreparedMesh& mesh,
  const talus::deformable::SystemFactor& factor,
  const glm::mat3& turn,
  const talus::deformable::StepReport& report
) {
  const std::vector<size_t> rows = freeRows(mesh);
  const size_t count = rows.size();
  const glm::dmat3 unturn = glm::transpose(glm::dmat3(turn));

  Dense engine(3u * mesh.restPositions.size());

  for (const size_t row : rows) {
    for (const size_t column : rows) {
      engine.at(row, column) =
        factor.entry(static_cast<uint32_t>(row), static_cast<uint32_t>(column));
    }
  }

  const std::vector<double> compliance = inverted(engine, rows);

  std::vector<size_t> slot(3u * mesh.restPositions.size(), count);

  for (size_t at = 0u; at != count; ++at) {
    slot[rows[at]] = at;
  }

  const size_t contacts = report.contactFrames.size();
  const size_t size = 3u * contacts;

  REQUIRE(report.contactOperator.size() == size * size);

  const auto rowOf = [&](size_t which) {
    const talus::deformable::ContactFrame& frame =
      report.contactFrames[which / 3u];
    const glm::vec3 direction = which % 3u == 0u   ? frame.normal
                                : which % 3u == 1u ? frame.tangent
                                                   : frame.bitangent;

    std::vector<double> made(count, 0.0);

    for (int corner = 0; corner != 3; ++corner) {
      const uint32_t node = frame.nodes[corner];

      if (mesh.anchored[node] != 0u) {
        continue;
      }

      const glm::dvec3 turned =
        unturn
        * (static_cast<double>(frame.weights[corner]) * glm::dvec3(direction));

      for (size_t axis = 0u; axis != 3u; ++axis) {
        made[slot[3u * node + axis]] += turned[static_cast<int>(axis)];
      }
    }

    return made;
  };

  double difference = 0.0;
  double size2 = 0.0;

  for (size_t r = 0u; r != size; ++r) {
    const std::vector<double> pusher = rowOf(r);
    std::vector<double> response(count, 0.0);

    for (size_t row = 0u; row != count; ++row) {
      for (size_t column = 0u; column != count; ++column) {
        response[row] += compliance[row * count + column] * pusher[column];
      }
    }

    for (size_t s = 0u; s != size; ++s) {
      const std::vector<double> felt = rowOf(s);

      double entry = 0.0;

      for (size_t at = 0u; at != count; ++at) {
        entry += felt[at] * response[at];
      }

      entry *= BETA;

      const double gap =
        static_cast<double>(report.contactOperator[s * size + r]) - entry;

      difference += gap * gap;
      size2 += entry * entry;
    }
  }

  return size2 > 0.0 ? std::sqrt(difference / size2) : std::sqrt(difference);
}

std::vector<talus::deformable::SurfaceContact> topContacts(
  const PreparedMesh& mesh,
  const DeformableState& state
) {
  std::vector<talus::deformable::SurfaceContact> made;

  for (uint32_t triangle = 0u; triangle != mesh.surfaceTriangles.size();
       ++triangle) {
    const glm::uvec3 nodes = mesh.surfaceTriangles[triangle];

    bool top = true;

    for (int corner = 0; corner != 3; ++corner) {
      const glm::vec3& rest = mesh.restPositions[nodes[corner]];

      top = top && rest.y > BAR_HALF_THICKNESS - 0.25f * BAR_VOXEL
            && rest.x > 0.25f * BAR_VOXEL;
    }

    if (!top || made.size() == 4u) {
      continue;
    }

    glm::vec3 point(0.0f);

    for (int corner = 0; corner != 3; ++corner) {
      point += state.position[nodes[corner]] / 3.0f;
    }

    talus::deformable::SurfaceContact contact;

    contact.nodes = nodes;
    contact.weights = glm::vec3(1.0f / 3.0f);
    contact.partner = 0;
    contact.normal = glm::vec3(0.0f, -1.0f, 0.0f);
    contact.localPoint = point - glm::vec3(0.0f, 1.0f, 0.0f);
    contact.friction = 0.5f;
    contact.triangle = triangle;

    made.push_back(contact);
  }

  REQUIRE(made.size() == 4u);

  return made;
}

DeformableState bentStart(const PreparedMesh& mesh) {
  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  for (size_t node = 0u; node != state.position.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    const glm::vec3 rest = mesh.restPositions[node];
    const float along = rest.x + BAR_HALF_LENGTH;

    state.velocity[node] =
      glm::vec3(0.0f, -10.0f * along * along, 3.0f * along * rest.y);
  }

  return state;
}

glm::mat3 strainPattern() {
  return glm::mat3(
    glm::vec3(1.0f, 0.5f, -0.3f),
    glm::vec3(0.5f, -0.6f, 0.2f),
    glm::vec3(-0.3f, 0.2f, 0.4f)
  );
}

glm::vec3 spinOf(bool tumble, float rate) {
  return tumble ? rate * glm::normalize(glm::vec3(0.3f, 0.4f, 1.0f))
                : glm::vec3(0.0f, rate, 0.0f);
}

DeformableState freeStart(
  const PreparedMesh& mesh,
  bool tumble,
  float rate,
  float strain
) {
  const glm::mat3 pattern = strainPattern();
  const glm::vec3 spin = spinOf(tumble, rate);
  const glm::vec3 drift(0.2f, -0.1f, 0.05f);

  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 rest = mesh.restPositions[node];
    const glm::vec3 placed = rest + strain * (pattern * rest);

    state.position[node] = placed;
    state.velocity[node] = drift + glm::cross(spin, placed);
  }

  return state;
}

struct SiteCheck {
  double factor = -1.0;
  double basis = -1.0;
  double applied = -1.0;
  double isolated = -1.0;
  double diagonal = -1.0;
  double sweep = -1.0;
};

SiteCheck checkSites(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> lanes,
  const DeformableState& start,
  talus::deformable::Settings settings,
  std::span<const talus::deformable::SurfaceContact> contacts,
  bool factored
) {
  SiteCheck made;

  settings.reportSystem = true;
  settings.reportContactSystem = !contacts.empty();
  settings.contactOperator = talus::deformable::ContactOperator::Dense;

  std::vector<talus::deformable::RigidPartner> partners(1);

  partners[0].centerOfMass = glm::vec3(0.0f, 1.0f, 0.0f);
  partners[0].dynamic = false;

  DeformableState state = start;
  talus::deformable::DeformableSolverState solver;
  talus::deformable::StepReport report;

  if (factored) {
    DeformableState primed = start;
    talus::deformable::StepReport priming;

    talus::deformable::step(
      mesh,
      primed,
      barLanes(0.0f, 0.0f),
      settings,
      glm::vec3(0.0f, -9.81f, 0.0f),
      FRAME,
      contacts,
      contacts.empty() ? std::span<talus::deformable::RigidPartner>{}
                       : std::span<talus::deformable::RigidPartner>(partners),
      &solver,
      priming
    );

    REQUIRE(solver.factor.reference().damping.empty());
    REQUIRE((contacts.empty() || !solver.basis.column.empty()));
  }

  talus::deformable::step(
    mesh,
    state,
    lanes,
    settings,
    glm::vec3(0.0f, -9.81f, 0.0f),
    FRAME,
    contacts,
    contacts.empty() ? std::span<talus::deformable::RigidPartner>{}
                     : std::span<talus::deformable::RigidPartner>(partners),
    factored ? &solver : nullptr,
    report
  );

  REQUIRE(report.cgPerSubstep.size() == 1u);
  REQUIRE(report.refactors == 0u);
  REQUIRE(report.systemPositions.size() == mesh.restPositions.size());

  const double inner = static_cast<double>(FRAME);
  const double weight = weightOf(1.0, inner);
  const double reach = reachOf(inner);
  const double inertia = 1.0 - ALPHA_MASS;
  const std::vector<size_t> rows = freeRows(mesh);
  const size_t count = 3u * mesh.restPositions.size();

  const Dense read = referenceSystem(
    mesh,
    lanes,
    report.systemPositions,
    report.systemRotations,
    inertia,
    reach,
    weight
  );
  const Dense readShare = referenceSystem(
    mesh,
    lanes,
    report.systemPositions,
    report.systemRotations,
    0.0,
    0.0,
    weight
  );

  made.applied = apart(
    rows,
    [&](size_t row, size_t column) {
      return static_cast<double>(report.systemOperator[row * count + column]);
    },
    read,
    readShare
  );
  made.isolated = apart(
    rows,
    [&](size_t row, size_t column) {
      return static_cast<double>(report.systemDamping[row * count + column]);
    },
    readShare,
    readShare
  );

  if (factored) {
    const std::vector<glm::mat3> rotations = rotationsOf(start);
    const Dense gathered = referenceSystem(
      mesh,
      lanes,
      start.position,
      rotations,
      inertia,
      reach,
      weight
    );
    const Dense gatheredShare =
      referenceSystem(mesh, lanes, start.position, rotations, 0.0, 0.0, weight);

    made.factor = apart(
      rows,
      [&](size_t row, size_t column) {
        return solver.factor.entry(
          static_cast<uint32_t>(row),
          static_cast<uint32_t>(column)
        );
      },
      gathered,
      gatheredShare
    );

    if (!contacts.empty()) {
      made.basis = basisApart(
        mesh,
        solver.factor,
        talus::deformable::bodyRotation(mesh, start, solver.factor.reference()),
        report
      );
    }
  } else {
    made.diagonal = diagonalApart(mesh, report.systemDiagonal, read, readShare);
    made.sweep = sweepApart(rows, report.systemSweep, read, readShare);
  }

  return made;
}

}

}

TEST_CASE(
  "every damped operator site assembles one damping",
  "[.][dampingoperator]"
) {
  constexpr uint32_t LEAD_FRAMES = 4u;

  const std::vector<PhysicalMaterial> lanes =
    damping::barLanes(0.0f, damping::OPERATOR_TAU);
  const std::vector<PhysicalMaterial> plain = damping::barLanes(0.0f, 0.0f);

  std::cout.precision(9);

  for (const talus::deformable::VolumeElementType element :
       {talus::deformable::VolumeElementType::Hex,
        talus::deformable::VolumeElementType::Tet}) {
    const talus::deformable::Settings settings = damping::settingsFor(element);

    const std::shared_ptr<const PreparedMesh> clamped =
      damping::bar(lanes, element, damping::Hold::Clamp, true);

    DeformableState lead = damping::bentStart(*clamped);
    talus::deformable::DeformableSolverState leadSolver;

    for (uint32_t frame = 0u; frame != LEAD_FRAMES; ++frame) {
      talus::deformable::StepReport report;

      talus::deformable::step(
        *clamped,
        lead,
        lanes,
        settings,
        glm::vec3(0.0f, -9.81f, 0.0f),
        damping::FRAME,
        {},
        {},
        &leadSolver,
        report
      );
    }

    const std::vector<talus::deformable::SurfaceContact> contacts =
      damping::topContacts(*clamped, lead);

    const damping::SiteCheck factored =
      damping::checkSites(*clamped, lanes, lead, settings, contacts, true);
    const damping::SiteCheck unfactored =
      damping::checkSites(*clamped, lanes, lead, settings, contacts, false);

    const std::shared_ptr<const PreparedMesh> loose =
      damping::bar(lanes, element, damping::Hold::Free, true);

    DeformableState spun =
      damping::freeStart(*loose, false, 5.0f, damping::PRE_STRAIN);
    talus::deformable::DeformableSolverState spunSolver;

    for (uint32_t frame = 0u; frame != LEAD_FRAMES; ++frame) {
      talus::deformable::StepReport report;

      talus::deformable::step(
        *loose,
        spun,
        lanes,
        settings,
        glm::vec3(0.0f, -9.81f, 0.0f),
        damping::FRAME,
        {},
        {},
        &spunSolver,
        report
      );
    }

    const damping::SiteCheck floating =
      damping::checkSites(*loose, lanes, spun, settings, {}, true);

    talus::deformable::Settings equilibrium = settings;

    equilibrium.cgIterations = 1200u;
    equilibrium.tolerance = 1.0e-5f;

    const DeformableState settledDamped = talus::deformable::solveEquilibrium(
      *clamped,
      lanes,
      talus::objectmodel::Pose{},
      equilibrium,
      glm::vec3(0.0f, -9.81f, 0.0f)
    );
    const DeformableState settledPlain = talus::deformable::solveEquilibrium(
      *clamped,
      plain,
      talus::objectmodel::Pose{},
      equilibrium,
      glm::vec3(0.0f, -9.81f, 0.0f)
    );
    const talus::deformable::EquilibriumCheck checkedDamped =
      talus::deformable::checkEquilibrium(
        *clamped,
        lead,
        lanes,
        equilibrium,
        glm::vec3(0.0f, -9.81f, 0.0f)
      );
    const talus::deformable::EquilibriumCheck checkedPlain =
      talus::deformable::checkEquilibrium(
        *clamped,
        lead,
        plain,
        equilibrium,
        glm::vec3(0.0f, -9.81f, 0.0f)
      );

    std::cout << "dampingoperator: " << damping::nameOf(element) << " tau "
              << damping::OPERATOR_TAU << " lanes tau 0 and tau h "
              << damping::FRAME << " substeps " << settings.substeps
              << " clamped factorGather " << factored.factor
              << " responseBasis " << factored.basis << " cgApply "
              << factored.applied << " cgApplyIsolated " << factored.isolated
              << " noFactor cgApply " << unfactored.applied
              << " cgApplyIsolated " << unfactored.isolated
              << " jacobiDiagonal " << unfactored.diagonal << " gaussSeidel "
              << unfactored.sweep << " floating factorGather "
              << floating.factor << " cgApply " << floating.applied
              << " cgApplyIsolated " << floating.isolated << " staticSame "
              << (settledDamped.position == settledPlain.position
                      && checkedDamped.maxCorrection
                           == checkedPlain.maxCorrection
                    ? 1
                    : 0)
              << " bar " << damping::OPERATOR_BAR << '\n';

    REQUIRE(factored.factor <= damping::OPERATOR_BAR);
    REQUIRE(factored.basis <= damping::OPERATOR_BAR);
    REQUIRE(factored.applied <= damping::OPERATOR_BAR);
    REQUIRE(factored.isolated <= damping::OPERATOR_BAR);
    REQUIRE(unfactored.applied <= damping::OPERATOR_BAR);
    REQUIRE(unfactored.isolated <= damping::OPERATOR_BAR);
    REQUIRE(unfactored.diagonal <= damping::OPERATOR_BAR);
    REQUIRE(unfactored.sweep <= damping::OPERATOR_BAR);
    REQUIRE(floating.factor <= damping::OPERATOR_BAR);
    REQUIRE(floating.applied <= damping::OPERATOR_BAR);
    REQUIRE(floating.isolated <= damping::OPERATOR_BAR);
    REQUIRE(settledDamped.position == settledPlain.position);
    REQUIRE(checkedDamped.maxCorrection == checkedPlain.maxCorrection);
  }
}

namespace {

namespace damping {

constexpr float BEAM_HALF_LENGTH = 0.5f;
constexpr float BEAM_HALF_THICKNESS = 0.05f;
constexpr float BEAM_ANCHOR_END = -0.45f;
constexpr float BEAM_VOXEL = 0.05f;
constexpr float BEAM_RELEASE = 0.05f;
constexpr uint32_t RING_FRAMES = 300u;
constexpr double RING_SKIP_PERIODS = 3.0;
constexpr double RECOVERY_BAR = 0.02;
constexpr double CONTINUITY_SCALE = 1.0e-3;
constexpr std::array<double, 2> CONTINUITY_SCALES{1.0e-1, 1.0e-2};
constexpr double CONTINUITY_BAR = 0.05;
constexpr float CONTINUITY_TOLERANCE = 1.0e-6f;
constexpr float CONTINUITY_AMPLITUDE = 10.0f;
constexpr uint32_t MODE_PASSES = 3u;
constexpr float MODE_OMEGA = 15.8f;

constexpr float FREE_TAU = 1.4095e-4f;
constexpr float HINGE_TAU = 2.152e-4f;
constexpr float LARGEST_MODE_TAU = 6.324e-3f;
constexpr uint32_t RIGID_FRAMES = 300u;
constexpr uint32_t ORBIT_FRAMES = 300u;
constexpr double RIGID_BAR = 1.0e-3;

std::shared_ptr<const PreparedMesh> cantilever(
  std::span<const PhysicalMaterial> lanes,
  talus::deformable::VolumeElementType element
) {
  const float skin = 0.2f * BEAM_VOXEL;
  const glm::vec3 half(
    BEAM_HALF_LENGTH + skin,
    BEAM_HALF_THICKNESS + skin,
    BEAM_HALF_THICKNESS + skin
  );

  talus::field::Primitive primitive = fixture::box(glm::vec3(0.0f), half);

  primitive.shapeConfig.y = DAMPED_LANE;

  const glm::vec3 minimum(
    -BEAM_HALF_LENGTH - BEAM_VOXEL,
    -BEAM_HALF_THICKNESS - BEAM_VOXEL,
    -BEAM_HALF_THICKNESS - BEAM_VOXEL
  );
  const glm::uvec3 resolution(
    static_cast<uint32_t>(std::lround(1.0f / BEAM_VOXEL)) + 3u,
    static_cast<uint32_t>(std::lround(2.0f * BEAM_HALF_THICKNESS / BEAM_VOXEL))
      + 3u,
    static_cast<uint32_t>(std::lround(2.0f * BEAM_HALF_THICKNESS / BEAM_VOXEL))
      + 3u
  );

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    minimum,
    -minimum,
    resolution
  );

  const glm::vec3 low(-2.0f, -2.0f, -2.0f);
  const glm::vec3 high(BEAM_ANCHOR_END + HOLD_TOLERANCE, 2.0f, 2.0f);

  const talus::deformable::SupportVolume support{
    glm::translate(glm::mat4(1.0f), -0.5f * (low + high)),
    0.5f * (high - low)
  };

  const std::shared_ptr<const PreparedMesh> made = talus::deformable::buildMesh(
    lattice,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>(&support, 1u),
    element
  );

  REQUIRE(made != nullptr);
  REQUIRE(made->restPositions.size() == 189u);

  return made;
}

struct Recurrence {
  double decay = 0.0;
  double angle = 0.0;
  double zeta = 0.0;
};

Recurrence recurrence(double omegaH, double zeta) {
  const double stiffness = omegaH * omegaH;
  const double viscous = 2.0 * zeta * omegaH;

  const double lead = (1.0 - ALPHA_MASS) + viscous * (1.0 - ALPHA_FORCE) * GAMMA
                      + stiffness * (1.0 - ALPHA_FORCE) * BETA;

  std::array<std::array<double, 3>, 3> map{};

  for (size_t column = 0u; column != 3u; ++column) {
    const double x = column == 0u ? 1.0 : 0.0;
    const double v = column == 1u ? 1.0 : 0.0;
    const double a = column == 2u ? 1.0 : 0.0;

    const double next =
      -(
        ALPHA_MASS * a + viscous * (v + (1.0 - ALPHA_FORCE) * (1.0 - GAMMA) * a)
        + stiffness
            * (x + (1.0 - ALPHA_FORCE) * v + (1.0 - ALPHA_FORCE) * (0.5 - BETA) * a)
      )
      / lead;

    map[0][column] = x + v + (0.5 - BETA) * a + BETA * next;
    map[1][column] = v + (1.0 - GAMMA) * a + GAMMA * next;
    map[2][column] = next;
  }

  const double trace = map[0][0] + map[1][1] + map[2][2];
  const double minors = map[0][0] * map[1][1] - map[0][1] * map[1][0]
                        + map[0][0] * map[2][2] - map[0][2] * map[2][0]
                        + map[1][1] * map[2][2] - map[1][2] * map[2][1];
  const double determinant =
    map[0][0] * (map[1][1] * map[2][2] - map[1][2] * map[2][1])
    - map[0][1] * (map[1][0] * map[2][2] - map[1][2] * map[2][0])
    + map[0][2] * (map[1][0] * map[2][1] - map[1][1] * map[2][0]);

  const auto cubic = [&](double at) {
    return ((at - trace) * at + minors) * at - determinant;
  };

  double low = -1.5;
  double high = 1.5;

  for (uint32_t pass = 0u; pass != 200u; ++pass) {
    const double middle = 0.5 * (low + high);

    if (cubic(middle) > 0.0) {
      high = middle;
    } else {
      low = middle;
    }
  }

  const double real = 0.5 * (low + high);
  const double sum = trace - real;
  const double product = minors - real * sum;
  const double imaginary =
    std::sqrt(std::max(0.0, 4.0 * product - sum * sum)) / 2.0;

  Recurrence made;

  made.decay = -0.5 * std::log(product);
  made.angle = std::atan2(imaginary, sum / 2.0);
  made.zeta =
    made.decay / std::sqrt(made.decay * made.decay + made.angle * made.angle);

  return made;
}

double continuousOmegaH(double discreteAngle) {
  double low = 0.5 * discreteAngle;
  double high = 2.0 * discreteAngle;

  for (uint32_t pass = 0u; pass != 200u; ++pass) {
    const double middle = 0.5 * (low + high);

    if (recurrence(middle, 0.0).angle > discreteAngle) {
      high = middle;
    } else {
      low = middle;
    }
  }

  return 0.5 * (low + high);
}

double tipOf(const PreparedMesh& mesh, const DeformableState& state) {
  double sum = 0.0;
  uint32_t count = 0u;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.restPositions[node].x > BEAM_HALF_LENGTH - 1.0e-4f) {
      sum += static_cast<double>(
        state.position[node].y - mesh.restPositions[node].y
      );
      ++count;
    }
  }

  REQUIRE(count != 0u);

  return sum / static_cast<double>(count);
}

DeformableState scaledRelease(
  const PreparedMesh& mesh,
  const DeformableState& shaped,
  float factor
) {
  DeformableState made = shaped;

  for (size_t node = 0u; node != made.position.size(); ++node) {
    made.position[node] =
      mesh.restPositions[node]
      + factor * (shaped.position[node] - mesh.restPositions[node]);
  }

  std::fill(made.velocity.begin(), made.velocity.end(), glm::vec3(0.0f));
  std::fill(
    made.acceleration.begin(),
    made.acceleration.end(),
    glm::vec3(0.0f)
  );

  return made;
}

struct Ring {
  double period = 0.0;
  double decay = 0.0;
  double zeta = 0.0;
  uint32_t peaks = 0u;
  uint32_t triggered = 0u;
  uint64_t cg = 0u;
  bool finite = true;
};

Ring ring(
  const PreparedMesh& mesh,
  std::span<const PhysicalMaterial> lanes,
  const DeformableState& released,
  const talus::deformable::Settings& settings
) {
  std::vector<size_t> tips;

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    if (mesh.restPositions[node].x > BEAM_HALF_LENGTH - 1.0e-4f) {
      tips.push_back(node);
    }
  }

  REQUIRE(!tips.empty());

  const auto tipOf = [&](const DeformableState& at) {
    double sum = 0.0;

    for (const size_t node : tips) {
      sum +=
        static_cast<double>(at.position[node].y - mesh.restPositions[node].y);
    }

    return sum / static_cast<double>(tips.size());
  };

  DeformableState state = released;
  talus::deformable::DeformableSolverState solver;

  Ring made;

  std::vector<double> signal{tipOf(state)};

  for (uint32_t frame = 0u; frame != RING_FRAMES; ++frame) {
    talus::deformable::StepReport report;

    talus::deformable::step(
      mesh,
      state,
      lanes,
      settings,
      glm::vec3(0.0f),
      FRAME,
      {},
      {},
      &solver,
      report
    );

    made.triggered +=
      report.cgPerSubstep.size() != std::max(settings.substeps, 1u) ? 1u : 0u;
    made.cg += report.cgIterations;

    const double tip = tipOf(state);

    made.finite = made.finite && std::isfinite(tip);
    signal.push_back(tip);
  }

  const double h = static_cast<double>(FRAME);

  std::vector<double> crossings;

  for (size_t at = 1u; at != signal.size(); ++at) {
    if (signal[at - 1u] < 0.0 && signal[at] >= 0.0) {
      const double share = -signal[at - 1u] / (signal[at] - signal[at - 1u]);

      crossings.push_back(h * (static_cast<double>(at - 1u) + share));
    }
  }

  REQUIRE(crossings.size() >= 4u);

  made.period = (crossings.back() - crossings.front())
                / static_cast<double>(crossings.size() - 1u);

  double sumT = 0.0;
  double sumY = 0.0;
  double sumTT = 0.0;
  double sumTY = 0.0;

  for (size_t at = 1u; at + 1u != signal.size(); ++at) {
    const double before = signal[at - 1u];
    const double here = signal[at];
    const double after = signal[at + 1u];
    const bool maximum = here > before && here >= after && here > 0.0;
    const bool minimum = here < before && here <= after && here < 0.0;

    if (!maximum && !minimum) {
      continue;
    }

    const double curvature = before - 2.0 * here + after;
    const double offset =
      curvature != 0.0 ? 0.5 * (before - after) / curvature : 0.0;
    const double peak = here - 0.25 * (before - after) * offset;
    const double time = h * (static_cast<double>(at) + offset);

    if (time < RING_SKIP_PERIODS * made.period) {
      continue;
    }

    const double logged = std::log(std::abs(peak));

    sumT += time;
    sumY += logged;
    sumTT += time * time;
    sumTY += time * logged;
    ++made.peaks;
  }

  REQUIRE(made.peaks >= 6u);

  const double count = static_cast<double>(made.peaks);
  const double slope =
    (count * sumTY - sumT * sumY) / (count * sumTT - sumT * sumT);
  const double frequency = 2.0 * 3.14159265358979323846 / made.period;

  made.decay = -slope;
  made.zeta =
    made.decay / std::sqrt(made.decay * made.decay + frequency * frequency);

  return made;
}

double angularOf(
  const PreparedMesh& mesh,
  const DeformableState& state,
  const std::vector<glm::vec3>& field,
  glm::dvec3& result
) {
  double mass = 0.0;
  glm::dvec3 centre(0.0);
  glm::dvec3 mean(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const double held = static_cast<double>(mesh.nodeMass[node]);

    mass += held;
    centre += held * glm::dvec3(state.position[node]);
    mean += held * glm::dvec3(field[node]);
  }

  centre /= mass;
  mean /= mass;
  result = glm::dvec3(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    result += static_cast<double>(mesh.nodeMass[node])
              * glm::cross(
                glm::dvec3(state.position[node]) - centre,
                glm::dvec3(field[node]) - mean
              );
  }

  return mass;
}

glm::dvec3 momentumOf(const PreparedMesh& mesh, const DeformableState& state) {
  glm::dvec3 made(0.0);

  angularOf(mesh, state, state.velocity, made);

  return made;
}

glm::dvec3 torqueOf(const PreparedMesh& mesh, const DeformableState& state) {
  glm::dvec3 made(0.0);

  angularOf(mesh, state, state.acceleration, made);

  return made;
}

double hingeMomentum(const PreparedMesh& mesh, const DeformableState& state) {
  double sum = 0.0;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::dvec3 at(state.position[node]);
    const glm::dvec3 moving(state.velocity[node]);

    sum += static_cast<double>(mesh.nodeMass[node])
           * (at.z * moving.x - at.x * moving.z);
  }

  return sum;
}

double hingeTorque(const PreparedMesh& mesh, const DeformableState& state) {
  double sum = 0.0;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::dvec3 at(state.position[node]);
    const glm::dvec3 turning(state.acceleration[node]);

    sum += static_cast<double>(mesh.nodeMass[node])
           * (at.z * turning.x - at.x * turning.z);
  }

  return sum;
}

DeformableState hingeStart(const PreparedMesh& mesh, float rate) {
  const glm::vec3 spin(0.0f, rate, 0.0f);

  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  for (size_t node = 0u; node != state.position.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    const glm::vec3 rest = mesh.restPositions[node];

    state.velocity[node] = glm::cross(spin, rest);
    state.acceleration[node] = glm::cross(spin, glm::cross(spin, rest));
  }

  return state;
}

struct Stepped {
  uint32_t substeps = 0u;
  uint32_t capped = 0u;
  uint32_t cg = 0u;
};

Stepped advance(
  const PreparedMesh& mesh,
  DeformableState& state,
  std::span<const PhysicalMaterial> lanes,
  const talus::deformable::Settings& settings,
  talus::deformable::DeformableSolverState* solver
) {
  talus::deformable::DeformableSolverState fresh;
  talus::deformable::StepReport report;

  talus::deformable::step(
    mesh,
    state,
    lanes,
    settings,
    glm::vec3(0.0f),
    FRAME,
    {},
    {},
    solver != nullptr ? solver : &fresh,
    report
  );

  return Stepped{
    static_cast<uint32_t>(report.cgPerSubstep.size()),
    report.cgCappedSolves,
    report.cgIterations
  };
}

double decrementOf(
  const PreparedMesh& mesh,
  const DeformableState& plain,
  const DeformableState& damped
) {
  double sum = 0.0;

  for (size_t node = 0u; node != plain.velocity.size(); ++node) {
    const glm::dvec3 before(plain.velocity[node]);

    sum += static_cast<double>(mesh.nodeMass[node])
           * glm::dot(before, glm::dvec3(damped.velocity[node]) - before);
  }

  return sum;
}

struct Decrement {
  double lowest = 0.0;
  double scaled = 0.0;
};

Decrement ringDecrement(
  const PreparedMesh& mesh,
  const DeformableState& released,
  const talus::deformable::Settings& settings,
  float tau,
  double scale
) {
  const std::vector<PhysicalMaterial> plain = barLanes(0.0f, 0.0f);
  const std::vector<PhysicalMaterial> lowest = barLanes(0.0f, tau);
  const std::vector<PhysicalMaterial> scaled =
    barLanes(0.0f, static_cast<float>(scale * static_cast<double>(tau)));

  DeformableState arm = released;
  talus::deformable::DeformableSolverState armSolver;

  Decrement made;

  for (uint32_t frame = 0u; frame != RING_FRAMES; ++frame) {
    DeformableState withLowest = arm;
    DeformableState withScaled = arm;
    DeformableState without = arm;

    advance(mesh, withLowest, lowest, settings, nullptr);
    advance(mesh, withScaled, scaled, settings, nullptr);
    advance(mesh, without, plain, settings, nullptr);
    advance(mesh, arm, plain, settings, &armSolver);

    made.lowest += decrementOf(mesh, without, withLowest);
    made.scaled += decrementOf(mesh, without, withScaled);
  }

  return made;
}

struct Paired {
  double sum = 0.0;
  double torqued = 0.0;
  double worst = 0.0;
  double scaledSum = 0.0;
  double scaledTorqued = 0.0;
  Decrement decrement;
  uint32_t capped = 0u;
  uint32_t decisions = 0u;
  bool finite = true;
};

Paired pairedFree(
  const PreparedMesh& mesh,
  float tau,
  bool tumble,
  const talus::deformable::Settings& settings
) {
  const std::vector<PhysicalMaterial> damped = barLanes(0.0f, tau);
  const std::vector<PhysicalMaterial> scaled = barLanes(
    0.0f,
    static_cast<float>(CONTINUITY_SCALE * static_cast<double>(tau))
  );
  const std::vector<PhysicalMaterial> plain = barLanes(0.0f, 0.0f);

  talus::deformable::Settings split = settings;

  split.rigidSplit = talus::deformable::RigidSplit::Unprojected;

  DeformableState arm = freeStart(mesh, tumble, 5.0f, PRE_STRAIN);
  talus::deformable::DeformableSolverState armSolver;

  const double start = glm::length(momentumOf(mesh, arm));

  glm::dvec3 summed(0.0);
  glm::dvec3 torqued(0.0);
  glm::dvec3 scaledSummed(0.0);
  glm::dvec3 scaledTorqued(0.0);
  Paired made;

  for (uint32_t frame = 0u; frame != RIGID_FRAMES; ++frame) {
    DeformableState withTau = arm;
    DeformableState withScaled = arm;
    DeformableState without = arm;

    const Stepped fromTau = advance(mesh, withTau, damped, split, nullptr);
    const Stepped fromScaled =
      advance(mesh, withScaled, scaled, split, nullptr);
    const Stepped fromZero = advance(mesh, without, plain, split, nullptr);

    advance(mesh, arm, damped, settings, &armSolver);

    const glm::dvec3 moved =
      momentumOf(mesh, withTau) - momentumOf(mesh, without);
    const glm::dvec3 turned = torqueOf(mesh, withTau) - torqueOf(mesh, without);
    const glm::dvec3 scaledMoved =
      momentumOf(mesh, withScaled) - momentumOf(mesh, without);
    const glm::dvec3 scaledTurned =
      torqueOf(mesh, withScaled) - torqueOf(mesh, without);
    const double inner = static_cast<double>(FRAME)
                         / static_cast<double>(std::max(fromTau.substeps, 1u));

    summed += moved;
    torqued += moved + (1.0 - GAMMA) * inner * turned;
    scaledSummed += scaledMoved;
    scaledTorqued += scaledMoved + (1.0 - GAMMA) * inner * scaledTurned;
    made.decrement.lowest += decrementOf(mesh, without, withTau);
    made.decrement.scaled += decrementOf(mesh, without, withScaled);
    made.worst = std::max(made.worst, glm::length(moved) / start);
    made.capped += fromTau.capped + fromScaled.capped + fromZero.capped;
    made.decisions += fromTau.substeps != fromZero.substeps ? 1u : 0u;
    made.finite = made.finite && std::isfinite(glm::length(moved));
  }

  made.sum = glm::length(summed) / start;
  made.torqued = glm::length(torqued) / start;
  made.scaledSum = glm::length(scaledSummed) / start;
  made.scaledTorqued = glm::length(scaledTorqued) / start;

  return made;
}

DeformableState settledOrbit(
  const PreparedMesh& mesh,
  float rate,
  const talus::deformable::Settings& settings
) {
  const std::vector<PhysicalMaterial> plain = barLanes(0.0f, 0.0f);

  DeformableState state = hingeStart(mesh, rate);
  talus::deformable::DeformableSolverState solver;

  for (uint32_t frame = 0u; frame != ORBIT_FRAMES; ++frame) {
    advance(mesh, state, plain, settings, &solver);
  }

  return state;
}

Paired pairedHinge(
  const PreparedMesh& mesh,
  const DeformableState& settled,
  float tau,
  const talus::deformable::Settings& settings
) {
  const std::vector<PhysicalMaterial> damped = barLanes(0.0f, tau);
  const std::vector<PhysicalMaterial> scaled = barLanes(
    0.0f,
    static_cast<float>(CONTINUITY_SCALE * static_cast<double>(tau))
  );
  const std::vector<PhysicalMaterial> plain = barLanes(0.0f, 0.0f);

  DeformableState arm = settled;
  talus::deformable::DeformableSolverState armSolver;

  const double start = std::abs(hingeMomentum(mesh, settled));

  double summed = 0.0;
  double torqued = 0.0;
  double scaledSummed = 0.0;
  double scaledTorqued = 0.0;
  Paired made;

  for (uint32_t frame = 0u; frame != RIGID_FRAMES; ++frame) {
    DeformableState withTau = arm;
    DeformableState withScaled = arm;
    DeformableState without = arm;

    const Stepped fromTau = advance(mesh, withTau, damped, settings, nullptr);
    const Stepped fromScaled =
      advance(mesh, withScaled, scaled, settings, nullptr);
    const Stepped fromZero = advance(mesh, without, plain, settings, nullptr);

    advance(mesh, arm, damped, settings, &armSolver);

    const double moved =
      hingeMomentum(mesh, withTau) - hingeMomentum(mesh, without);
    const double turned =
      hingeTorque(mesh, withTau) - hingeTorque(mesh, without);
    const double scaledMoved =
      hingeMomentum(mesh, withScaled) - hingeMomentum(mesh, without);
    const double scaledTurned =
      hingeTorque(mesh, withScaled) - hingeTorque(mesh, without);
    const double inner = static_cast<double>(FRAME)
                         / static_cast<double>(std::max(fromTau.substeps, 1u));

    summed += moved;
    torqued += moved + (1.0 - GAMMA) * inner * turned;
    scaledSummed += scaledMoved;
    scaledTorqued += scaledMoved + (1.0 - GAMMA) * inner * scaledTurned;
    made.decrement.lowest += decrementOf(mesh, without, withTau);
    made.decrement.scaled += decrementOf(mesh, without, withScaled);
    made.worst = std::max(made.worst, std::abs(moved) / start);
    made.capped += fromTau.capped + fromScaled.capped + fromZero.capped;
    made.decisions += fromTau.substeps != fromZero.substeps ? 1u : 0u;
    made.finite = made.finite && std::isfinite(moved);
  }

  made.sum = std::abs(summed) / start;
  made.torqued = std::abs(torqued) / start;
  made.scaledSum = std::abs(scaledSummed) / start;
  made.scaledTorqued = std::abs(scaledTorqued) / start;

  return made;
}

}

}

TEST_CASE(
  "a damped cantilever recovers its authored damping ratio",
  "[.][dampedmode]"
) {
  std::cout.precision(9);

  for (const talus::deformable::VolumeElementType element :
       {talus::deformable::VolumeElementType::Hex,
        talus::deformable::VolumeElementType::Tet}) {
    const std::vector<PhysicalMaterial> plain = damping::barLanes(0.0f, 0.0f);
    const std::shared_ptr<const PreparedMesh> mesh =
      damping::cantilever(plain, element);

    talus::deformable::Settings equilibrium = damping::settingsFor(element);

    equilibrium.cgIterations = 1200u;
    equilibrium.tolerance = 1.0e-5f;

    DeformableState released = talus::deformable::solveEquilibrium(
      *mesh,
      plain,
      talus::objectmodel::Pose{},
      equilibrium,
      glm::vec3(0.0f, -9.81f * damping::BEAM_RELEASE, 0.0f)
    );

    const double target = damping::tipOf(*mesh, released);

    for (uint32_t pass = 0u; pass != damping::MODE_PASSES; ++pass) {
      std::vector<glm::vec3> load(mesh->restPositions.size(), glm::vec3(0.0f));

      for (size_t node = 0u; node != load.size(); ++node) {
        load[node] = damping::MODE_OMEGA * damping::MODE_OMEGA
                     * mesh->nodeMass[node]
                     * (released.position[node] - mesh->restPositions[node]);
      }

      released = talus::deformable::solveEquilibrium(
        *mesh,
        plain,
        talus::objectmodel::Pose{},
        equilibrium,
        glm::vec3(0.0f),
        load
      );
    }

    released = damping::scaledRelease(
      *mesh,
      released,
      static_cast<float>(target / damping::tipOf(*mesh, released))
    );

    const DeformableState amplified =
      damping::scaledRelease(*mesh, released, damping::CONTINUITY_AMPLITUDE);

    for (const uint32_t substeps : {1u, 2u, 4u}) {
      talus::deformable::Settings settings = damping::settingsFor(element);

      settings.substeps = substeps;

      talus::deformable::Settings tight = settings;

      tight.tolerance = damping::CONTINUITY_TOLERANCE;

      const damping::Ring undamped =
        damping::ring(*mesh, plain, released, settings);
      const double omega = 2.0 * 3.14159265358979323846 / undamped.period;
      const double inner =
        static_cast<double>(damping::FRAME) / static_cast<double>(substeps);
      const double continuous =
        damping::continuousOmegaH(omega * inner) / inner;

      std::cout << "dampedmode: " << damping::nameOf(element) << " substeps "
                << substeps << " spectralRadius " << settings.spectralRadius
                << " cgIterations " << settings.cgIterations << " tolerance "
                << settings.tolerance << " frames " << damping::RING_FRAMES
                << " tau 0 periodFrames " << undamped.period * 60.0
                << " omega1 " << omega << " continuousOmega1 " << continuous
                << " omegaH " << continuous * inner << " numericalZeta "
                << undamped.zeta << " recurrenceZeta "
                << damping::recurrence(continuous * inner, 0.0).zeta
                << " peaks " << undamped.peaks << " triggered "
                << undamped.triggered << '\n';

      REQUIRE(undamped.finite);
      REQUIRE(undamped.triggered == 0u);

      for (const double zeta : {0.01, 0.05}) {
        const float tau = static_cast<float>(2.0 * zeta / omega);
        const std::vector<PhysicalMaterial> lanes =
          damping::barLanes(0.0f, tau);
        const damping::Ring damped =
          damping::ring(*mesh, lanes, released, settings);
        const double half = static_cast<double>(tau) * omega / 2.0;
        const double recovery = (damped.zeta - undamped.zeta) / half;
        const double expected =
          (damping::recurrence(
             continuous * inner,
             static_cast<double>(tau) * continuous / 2.0
           )
             .zeta
           - damping::recurrence(continuous * inner, 0.0).zeta)
          / half;
        const double continuousRecovery =
          (damped.zeta - undamped.zeta)
          / (static_cast<double>(tau) * continuous / 2.0);

        std::cout << "dampedmode: " << damping::nameOf(element) << " substeps "
                  << substeps << " zeta " << zeta << " tau " << tau
                  << " periodFrames " << damped.period * 60.0 << " zeta "
                  << damped.zeta << " recovery " << recovery
                  << " expectedRecovery " << expected
                  << " recoveryOverContinuous " << continuousRecovery
                  << " gapPp " << 100.0 * (recovery - expected) << " peaks "
                  << damped.peaks << " triggered " << damped.triggered
                  << " cgPerFrame "
                  << static_cast<double>(damped.cg) / damping::RING_FRAMES
                  << " bar 0.02" << '\n';

        REQUIRE(damped.finite);
        REQUIRE(damped.triggered == 0u);
        REQUIRE(std::abs(recovery - expected) <= damping::RECOVERY_BAR);
      }

      const float lowestTau = static_cast<float>(2.0 * 0.01 / omega);
      const damping::Ring tightPlain =
        damping::ring(*mesh, plain, released, tight);
      const damping::Ring tightLowest = damping::ring(
        *mesh,
        damping::barLanes(0.0f, lowestTau),
        released,
        tight
      );
      const double lowestChange = tightLowest.zeta - tightPlain.zeta;

      REQUIRE(tightPlain.finite);
      REQUIRE(tightPlain.triggered == 0u);
      REQUIRE(tightLowest.finite);
      REQUIRE(tightLowest.triggered == 0u);
      REQUIRE(lowestChange > 0.0);

      for (const double scale : damping::CONTINUITY_SCALES) {
        const float scaledTau =
          static_cast<float>(scale * static_cast<double>(lowestTau));
        const damping::Ring tightScaled = damping::ring(
          *mesh,
          damping::barLanes(0.0f, scaledTau),
          released,
          tight
        );
        const double linear =
          (tightScaled.zeta - tightPlain.zeta) / (scale * lowestChange);
        const damping::Decrement decrement =
          damping::ringDecrement(*mesh, amplified, tight, lowestTau, scale);
        const double pairedLinear =
          decrement.scaled / (scale * decrement.lowest);

        std::cout << "dampedmode: continuity " << damping::nameOf(element)
                  << " substeps " << substeps << " tolerance "
                  << tight.tolerance << " scale " << scale << " tau "
                  << lowestTau << " scaledTau " << scaledTau << " zeta0 "
                  << tightPlain.zeta << " zetaLowest " << tightLowest.zeta
                  << " zetaScaled " << tightScaled.zeta << " changeOverLinear "
                  << linear << " bar " << damping::CONTINUITY_BAR << " peaks "
                  << tightScaled.peaks << " cgPerFrame "
                  << static_cast<double>(tightScaled.cg) / damping::RING_FRAMES
                  << " recorded releaseScale " << damping::CONTINUITY_AMPLITUDE
                  << " pairedDecrement " << decrement.lowest
                  << " scaledPairedDecrement " << decrement.scaled
                  << " pairedOverLinear " << pairedLinear << '\n';

        REQUIRE(tightScaled.finite);
        REQUIRE(tightScaled.triggered == 0u);
        REQUIRE(decrement.lowest < 0.0);
        REQUIRE(std::abs(linear - 1.0) <= damping::CONTINUITY_BAR);
      }
    }
  }
}

TEST_CASE(
  "damping leaves the rigid motion of a spinning body to the step",
  "[.][dampedrigid]"
) {
  std::cout.precision(9);

  for (const talus::deformable::VolumeElementType element :
       {talus::deformable::VolumeElementType::Hex,
        talus::deformable::VolumeElementType::Tet}) {
    const talus::deformable::Settings settings = damping::settingsFor(element);
    const std::shared_ptr<const PreparedMesh> mesh = damping::bar(
      damping::barLanes(0.0f, 0.0f),
      element,
      damping::Hold::Free,
      false
    );

    for (const bool tumble : {false, true}) {
      const damping::Paired paired =
        damping::pairedFree(*mesh, damping::FREE_TAU, tumble, settings);
      const double linear =
        paired.decrement.scaled
        / (damping::CONTINUITY_SCALE * paired.decrement.lowest);

      std::cout << "dampedrigid: free " << damping::nameOf(element) << ' '
                << (tumble ? "tumble" : "spin") << " rate 5 preStrain "
                << damping::PRE_STRAIN << " tau " << damping::FREE_TAU
                << " frames " << damping::RIGID_FRAMES << " substeps "
                << settings.substeps << " tolerance " << settings.tolerance
                << " pairedSumOverL0 " << paired.sum
                << " pairedWithTorqueOverL0 " << paired.torqued
                << " worstFrameOverL0 " << paired.worst << " capped "
                << paired.capped << " decisionsDiffer " << paired.decisions
                << " scaledPairedSumOverL0 " << paired.scaledSum
                << " scaledWithTorque " << paired.scaledTorqued
                << " momentumScaledOverLinear "
                << paired.scaledSum / (damping::CONTINUITY_SCALE * paired.sum)
                << " pairedDecrement " << paired.decrement.lowest
                << " scaledPairedDecrement " << paired.decrement.scaled
                << " decrementScaledOverLinear " << linear << " bar "
                << damping::RIGID_BAR << '\n';

      REQUIRE(paired.finite);
      REQUIRE(paired.sum <= damping::RIGID_BAR);
      REQUIRE(paired.torqued <= damping::RIGID_BAR);
    }
  }

  const talus::deformable::Settings settings =
    damping::settingsFor(talus::deformable::VolumeElementType::Hex);
  const std::shared_ptr<const PreparedMesh> hinged = damping::bar(
    damping::barLanes(0.0f, 0.0f),
    talus::deformable::VolumeElementType::Hex,
    damping::Hold::Hinge,
    false
  );

  for (const float rate : {5.0f, 10.0f, 20.0f}) {
    const DeformableState settled =
      damping::settledOrbit(*hinged, rate, settings);
    const damping::Paired paired =
      damping::pairedHinge(*hinged, settled, damping::HINGE_TAU, settings);
    const double linear =
      paired.decrement.scaled
      / (damping::CONTINUITY_SCALE * paired.decrement.lowest);

    std::cout << "dampedrigid: hinge hex rate " << rate << " orbitFrames "
              << damping::ORBIT_FRAMES << " tau " << damping::HINGE_TAU
              << " frames " << damping::RIGID_FRAMES << " pairedSumOverL0 "
              << paired.sum << " pairedWithTorqueOverL0 " << paired.torqued
              << " worstFrameOverL0 " << paired.worst << " capped "
              << paired.capped << " decisionsDiffer " << paired.decisions
              << " scaledPairedSumOverL0 " << paired.scaledSum
              << " scaledWithTorque " << paired.scaledTorqued
              << " momentumScaledOverLinear "
              << paired.scaledSum / (damping::CONTINUITY_SCALE * paired.sum)
              << " pairedDecrement " << paired.decrement.lowest
              << " scaledPairedDecrement " << paired.decrement.scaled
              << " decrementScaledOverLinear " << linear << " bar "
              << damping::RIGID_BAR << '\n';

    REQUIRE(paired.finite);
    REQUIRE(paired.sum <= damping::RIGID_BAR);
    REQUIRE(paired.torqued <= damping::RIGID_BAR);

    if (rate == 20.0f) {
      const damping::Paired largest = damping::pairedHinge(
        *hinged,
        settled,
        damping::LARGEST_MODE_TAU,
        settings
      );

      std::cout << "dampedrigid: recorded hinge hex rate 20 tau "
                << damping::LARGEST_MODE_TAU << " pairedSumOverL0 "
                << largest.sum << " pairedWithTorqueOverL0 " << largest.torqued
                << " worstFrameOverL0 " << largest.worst << " capped "
                << largest.capped << " finite " << largest.finite << '\n';
    }
  }
}

namespace {

constexpr float CONFIRM_RATE = 10.0f;
constexpr float CONFIRM_SLAB_EXTENT = 1.5f;
constexpr float CONFIRM_SLAB_VOXEL = 0.05f;
constexpr uint32_t CONFIRM_FRAMES = 600u;
constexpr uint32_t CONFIRM_ARMS = 2u;
constexpr float CONFIRM_REST_BALANCE =
  0.2f * talus::deformable::EQUILIBRIUM_BALANCE;
constexpr float CONFIRM_SHARE = 5.0e-2f;
constexpr float CONFIRM_PINCH = 50.0f;
constexpr float HANG_HEIGHT = 1.0f;
constexpr float HANG_ANCHOR = 0.005f;
constexpr float WAKE_CRATE_MASS = 0.05f;
constexpr float WAKE_CRATE_HALF = 0.015f;
constexpr float WAKE_CRATE_LIFT = 0.05f;
constexpr float WAKE_CRATE_AT = 0.1f;
constexpr float TIP_HALF_HEIGHT = 0.075f;
constexpr float TIP_LEAN_FROM_BALANCE = -1.0e-3f;
constexpr float TIP_TIPPED = 0.25f * glm::pi<float>();

struct ConfirmRun {
  uint32_t frozenAt = 0u;
  uint32_t armAttempts = 0u;
  uint32_t confirms = 0u;
  uint32_t refusals = 0u;
  uint32_t mostIterations = 0u;
  float worstCorrection = 0.0f;
  float worstForce = 0.0f;
  float worstTorque = 0.0f;
  bool everyFactored = true;
  bool everyResolved = true;
  bool anyUnbalanced = false;
};

std::vector<PhysicalMaterial> confirmMaterials() {
  return plasticbar::barMaterials(std::numeric_limits<float>::infinity());
}

talus::deformable::Settings confirmSettings(
  talus::deformable::VolumeElementType element
) {
  const talus::deformable::Settings settings = plasticbar::barStep(element);

  REQUIRE(settings.cgIterations == 64u);
  REQUIRE(settings.tolerance == 1.0e-3f);

  return settings;
}

std::string confirmFields(const talus::deformable::EquilibriumCheck& check) {
  std::ostringstream fields;

  fields << " freeModes " << check.freeModes << " factored "
         << (check.factored ? 1 : 0) << " cgIterations " << check.cgIterations
         << " capped " << (check.cgCapped ? 1 : 0) << " correction "
         << check.maxCorrection << " forceBalance " << check.forceBalance
         << " torqueBalance " << check.torqueBalance << " unbalanced "
         << (check.unbalanced ? 1 : 0) << " shift " << check.shift;

  return fields.str();
}

std::string confirmConfiguration(const talus::deformable::Settings& settings) {
  std::ostringstream fields;

  fields << " cgIterations " << settings.cgIterations << " tolerance "
         << settings.tolerance << " quiescentCorrection "
         << talus::runtime::QUIESCENT_CORRECTION << " eps "
         << talus::deformable::EQUILIBRIUM_BALANCE << " muOverTraceRatio "
         << talus::deformable::EQUILIBRIUM_SHIFT << " restBalance "
         << CONFIRM_REST_BALANCE;

  return fields.str();
}

Entity confirmBar(
  World& source,
  const glm::vec3& half,
  const glm::vec3& at,
  talus::deformable::VolumeElementType element,
  const glm::quat& turn = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)
) {
  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round(2.0f * half / plasticbar::COARSE_VOXEL))
    + glm::uvec3(1u);

  talus::objectmodel::Pose placed{};

  placed.position = at;
  placed.rotation = turn;

  const Entity bar = source.createSdfGrid(placed, grid);

  talus::field::Primitive shape = fixture::box(
    glm::vec3(0.0f),
    half + glm::vec3(0.2f * plasticbar::COARSE_VOXEL)
  );

  shape.shapeConfig.y = plasticbar::LANE;

  REQUIRE(source.addPrimitive(bar, shape));

  source.setSampledSdf(
    bar,
    talus::field::bake(source.primitives(bar), -half, half, grid.resolution)
  );

  talus::objectmodel::DeformableBody authored{};

  authored.element = element;

  source.emplaceOrReplace<talus::objectmodel::DeformableBody>(bar, authored);

  return bar;
}

void confirmSlab(World& source) {
  const glm::vec3 half(
    CONFIRM_SLAB_EXTENT,
    landingbar::LANDING_SLAB_HALF_HEIGHT,
    CONFIRM_SLAB_EXTENT
  );
  const glm::vec3 reach = half + glm::vec3(2.0f * CONFIRM_SLAB_VOXEL);

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round(2.0f * reach / CONFIRM_SLAB_VOXEL)) + glm::uvec3(1u);

  talus::objectmodel::Pose placed{};

  placed.position = glm::vec3(0.0f, landingbar::LANDING_SLAB_CENTRE, 0.0f);

  const Entity slab = source.createSdfGrid(placed, grid);

  talus::field::Primitive shape = fixture::box(glm::vec3(0.0f), half);

  shape.shapeConfig.y = plasticbar::LANE;

  REQUIRE(source.addPrimitive(slab, shape));

  source.setSampledSdf(
    slab,
    talus::field::bake(source.primitives(slab), -reach, reach, grid.resolution)
  );

  source.addStaticBody(slab);
}

void confirmAnchor(World& world, const glm::vec3& at) {
  const Entity entity = world.createAuthored();

  talus::objectmodel::Pose pose{};

  pose.position = at;

  world.emplaceAuthored<talus::objectmodel::Pose>(entity, pose);
  world.emplaceAuthored<talus::objectmodel::Support>(
    entity,
    talus::objectmodel::Support{glm::vec3(HANG_ANCHOR)}
  );
}

void refreshAll(Runtime& runtime) {
  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  runtime.physics().optimizeBroadPhase();
}

ConfirmRun runToFreeze(
  Runtime& runtime,
  Entity body,
  uint32_t frames,
  const std::string& label
) {
  ConfirmRun run;

  for (uint32_t frame = 1u; frame <= frames && run.frozenAt == 0u; ++frame) {
    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();

    run.armAttempts += activity.armAttempts;
    run.confirms += activity.confirms;
    run.refusals += activity.refusals;

    if (activity.confirms != 0u) {
      const talus::deformable::EquilibriumCheck& check = activity.lastConfirm;

      run.mostIterations = std::max(run.mostIterations, check.cgIterations);
      run.worstCorrection =
        std::max(run.worstCorrection, activity.worstCorrection);
      run.worstForce = std::max(run.worstForce, check.forceBalance);
      run.worstTorque = std::max(run.worstTorque, check.torqueBalance);
      run.everyFactored = run.everyFactored && check.factored;
      run.everyResolved = run.everyResolved && !check.cgCapped;
      run.anyUnbalanced = run.anyUnbalanced || check.unbalanced;

      std::cout << label << " confirm frame " << frame << " armAttempts "
                << activity.armAttempts << " refusals " << activity.refusals
                << " freezes " << activity.freezes << " confirmSteps "
                << activity.confirmSteps << " worstCorrection "
                << activity.worstCorrection << confirmFields(check) << '\n';
    }

    if (runtime.deformableQuiescent(body)) {
      run.frozenAt = frame;
    }
  }

  std::cout << label << " run frozenAt " << run.frozenAt << " armAttempts "
            << run.armAttempts << " confirms " << run.confirms << " refusals "
            << run.refusals << " mostIterations " << run.mostIterations
            << " worstCorrection " << run.worstCorrection << " worstForce "
            << run.worstForce << " worstTorque " << run.worstTorque
            << " everyFactored " << (run.everyFactored ? 1 : 0)
            << " everyResolved " << (run.everyResolved ? 1 : 0) << '\n';

  return run;
}

double confirmArm(const PreparedMesh& mesh, const DeformableState& state) {
  const glm::dvec3 centre = landingbar::landingCentre(mesh, state);

  double arm = 0.0;

  for (const glm::vec3& position : state.position) {
    arm = std::max(arm, glm::length(glm::dvec3(position) - centre));
  }

  return arm;
}

double confirmScale(const PreparedMesh& mesh, std::span<const glm::vec3> load) {
  double scale = 0.0;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    if (mesh.anchored[node] != 0u) {
      continue;
    }

    scale += static_cast<double>(mesh.nodeMass[node])
             * static_cast<double>(PhysicsWorld::GRAVITY);

    if (node < load.size()) {
      scale += static_cast<double>(glm::length(load[node]));
    }
  }

  return scale;
}

size_t nodeAt(const DeformableState& state, const glm::vec3& at) {
  size_t nearest = 0u;
  float closest = std::numeric_limits<float>::infinity();

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const float distance = glm::length(state.position[node] - at);

    if (distance < closest) {
      closest = distance;
      nearest = node;
    }
  }

  REQUIRE(closest < 0.25f * plasticbar::COARSE_VOXEL);

  return nearest;
}

Entity freeBar(World& source, talus::deformable::VolumeElementType element) {
  const Entity bar = confirmBar(
    source,
    glm::vec3(
      plasticbar::HALF_LENGTH,
      plasticbar::HALF_WIDTH,
      plasticbar::HALF_WIDTH
    ),
    glm::vec3(0.0f, landingbar::LANDING_DROP + plasticbar::HALF_WIDTH, 0.0f),
    element
  );

  confirmSlab(source);

  return bar;
}

const PreparedMesh& spinFreeBar(Runtime& runtime, Entity bar) {
  refreshAll(runtime);

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);
  REQUIRE(runtime.world().deformableState(bar) != nullptr);
  REQUIRE(
    std::none_of(
      mesh->anchored.begin(),
      mesh->anchored.end(),
      [](uint8_t held) { return held != 0u; }
    )
  );

  runtime.world().patchDeformableState(bar, [&](DeformableState& held) {
    const glm::vec3 centre(landingbar::landingCentre(*mesh, held));
    const glm::vec3 spin(0.0f, 0.0f, CONFIRM_RATE);

    for (size_t node = 0u; node != held.position.size(); ++node) {
      held.velocity[node] = glm::cross(spin, held.position[node] - centre);
      held.acceleration[node] = glm::vec3(0.0f);
    }
  });

  return *mesh;
}

size_t restNodeAt(const PreparedMesh& mesh, const glm::vec3& at) {
  size_t nearest = 0u;
  float closest = std::numeric_limits<float>::infinity();

  for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
    const float distance = glm::length(mesh.restPositions[node] - at);

    if (distance < closest) {
      closest = distance;
      nearest = node;
    }
  }

  REQUIRE(closest < 0.25f * plasticbar::COARSE_VOXEL);

  return nearest;
}

struct HangingRow {
  ConfirmRun run;
  talus::deformable::EquilibriumCheck atRest;
  talus::deformable::EquilibriumCheck turned;
  talus::deformable::EquilibriumCheck reacted;
};

HangingRow hangingRow(uint32_t anchors) {
  const std::vector<PhysicalMaterial> materials = confirmMaterials();
  const talus::deformable::VolumeElementType element =
    talus::deformable::VolumeElementType::Hex;
  const glm::vec3 gravity(0.0f, -PhysicsWorld::GRAVITY, 0.0f);
  const float top = HANG_HEIGHT + plasticbar::HALF_LENGTH;

  const Clock clock;

  JoltLibrary library;
  World source;

  source.setMaterials(materials);

  const Entity bar = confirmBar(
    source,
    glm::vec3(
      plasticbar::HALF_WIDTH,
      plasticbar::HALF_LENGTH,
      plasticbar::HALF_WIDTH
    ),
    glm::vec3(0.0f, HANG_HEIGHT, 0.0f),
    element
  );

  Runtime runtime(source, library);

  const talus::deformable::Settings settings = confirmSettings(element);

  runtime.setDeformableSettings(settings);

  if (anchors == 1u) {
    confirmAnchor(runtime.world(), glm::vec3(0.0f, top, 0.0f));
  } else {
    confirmAnchor(
      runtime.world(),
      glm::vec3(-plasticbar::HALF_WIDTH, top, 0.0f)
    );
    confirmAnchor(
      runtime.world(),
      glm::vec3(plasticbar::HALF_WIDTH, top, 0.0f)
    );
  }

  refreshAll(runtime);

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);
  REQUIRE(runtime.world().deformableState(bar) != nullptr);
  REQUIRE(
    static_cast<uint32_t>(std::count_if(
      mesh->anchored.begin(),
      mesh->anchored.end(),
      [](uint8_t held) { return held != 0u; }
    ))
    == anchors
  );

  const std::string label =
    std::string("hangingconfirm: anchors ") + std::to_string(anchors);

  std::cout << label << " nodes " << mesh->restPositions.size()
            << " anchorSpanOverLength "
            << (anchors == 2u ? plasticbar::HALF_WIDTH / plasticbar::HALF_LENGTH
                              : 0.0f)
            << confirmConfiguration(settings) << '\n';

  HangingRow row;

  row.run = runToFreeze(runtime, bar, CONFIRM_FRAMES, label);

  std::cout << label << " wall seconds " << clock.seconds() << '\n';

  const DeformableState& state = *runtime.world().deformableState(bar);

  const size_t hung =
    nodeAt(state, glm::vec3(0.0f, HANG_HEIGHT - plasticbar::HALF_LENGTH, 0.0f));

  size_t pivot = 0u;

  while (mesh->anchored[pivot] == 0u) {
    ++pivot;
  }

  const double drop =
    static_cast<double>(state.position[pivot].y - state.position[hung].y);
  const double arm = confirmArm(*mesh, state);
  const double scale = confirmScale(*mesh, {});
  const double share = static_cast<double>(CONFIRM_SHARE);
  const double push = share * scale * arm / (drop - share * arm);

  std::vector<glm::vec3> across(state.position.size(), glm::vec3(0.0f));
  std::vector<glm::vec3> along(state.position.size(), glm::vec3(0.0f));

  across[hung].z = static_cast<float>(push);
  along[hung].x = static_cast<float>(push);

  row.atRest = talus::deformable::checkEquilibrium(
    *mesh,
    state,
    materials,
    settings,
    gravity
  );
  row.turned = talus::deformable::checkEquilibrium(
    *mesh,
    state,
    materials,
    settings,
    gravity,
    across
  );
  row.reacted = talus::deformable::checkEquilibrium(
    *mesh,
    state,
    materials,
    settings,
    gravity,
    along
  );

  std::cout << label << " drop " << drop << " arm " << arm << " push " << push
            << '\n';
  std::cout << label << " rest" << confirmFields(row.atRest) << '\n';
  std::cout << label << " freeTorque" << confirmFields(row.turned) << '\n';
  std::cout << label << " heldTorque" << confirmFields(row.reacted) << '\n';

  return row;
}

struct TipRun {
  uint32_t armAttempts = 0u;
  uint32_t confirms = 0u;
  uint32_t refusals = 0u;
  uint32_t refusedAt = 0u;
  uint32_t frozenAt = 0u;
  uint32_t tippedAt = 0u;
  float refusedTilt = 0.0f;
  float refusedMoved = 0.0f;
  float frozenTilt = 0.0f;
  float tilt = 0.0f;
  talus::deformable::EquilibriumCheck refused;
};

float columnTilt(const DeformableState& state, size_t base, size_t crown) {
  const glm::vec3 axis = state.position[crown] - state.position[base];

  return std::acos(glm::clamp(glm::normalize(axis).y, -1.0f, 1.0f));
}

TipRun tipOverEdge() {
  const std::vector<PhysicalMaterial> materials = confirmMaterials();
  const talus::deformable::VolumeElementType element =
    talus::deformable::VolumeElementType::Hex;
  const float balance = std::atan(plasticbar::HALF_WIDTH / TIP_HALF_HEIGHT);
  const float lean = balance + TIP_LEAN_FROM_BALANCE;
  const float lift =
    plasticbar::HALF_WIDTH * std::sin(lean) + TIP_HALF_HEIGHT * std::cos(lean);

  JoltLibrary library;
  World source;

  source.setMaterials(materials);

  const Entity bar = confirmBar(
    source,
    glm::vec3(plasticbar::HALF_WIDTH, TIP_HALF_HEIGHT, plasticbar::HALF_WIDTH),
    glm::vec3(0.0f, lift, 0.0f),
    element,
    glm::angleAxis(-lean, glm::vec3(0.0f, 0.0f, 1.0f))
  );

  confirmSlab(source);

  Runtime runtime(source, library);

  const talus::deformable::Settings settings = confirmSettings(element);

  runtime.setDeformableSettings(settings);

  refreshAll(runtime);

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);
  REQUIRE(runtime.world().deformableState(bar) != nullptr);

  const size_t base =
    restNodeAt(*mesh, glm::vec3(0.0f, -TIP_HALF_HEIGHT, 0.0f));
  const size_t crown =
    restNodeAt(*mesh, glm::vec3(0.0f, TIP_HALF_HEIGHT, 0.0f));
  const std::vector<glm::vec3> start =
    runtime.world().deformableState(bar)->position;

  const std::string label = "tippingconfirm:";

  std::cout << label << " nodes " << mesh->restPositions.size()
            << " halfHeight " << TIP_HALF_HEIGHT << " balance " << balance
            << " leanFromBalance " << TIP_LEAN_FROM_BALANCE << " tipped "
            << TIP_TIPPED << confirmConfiguration(settings) << '\n';

  TipRun run;

  for (uint32_t frame = 1u;
       frame <= CONFIRM_FRAMES && run.frozenAt == 0u && run.tippedAt == 0u;
       ++frame) {
    runtime.step();

    const talus::runtime::DeformableActivity& activity =
      runtime.deformableActivity();
    const DeformableState& state = *runtime.world().deformableState(bar);

    float moved = 0.0f;

    for (size_t node = 0u; node != start.size(); ++node) {
      moved = glm::max(moved, glm::distance(state.position[node], start[node]));
    }

    run.armAttempts += activity.armAttempts;
    run.confirms += activity.confirms;
    run.refusals += activity.refusals;
    run.tilt = columnTilt(state, base, crown);

    if (activity.confirms != 0u) {
      std::cout << label << " confirm frame " << frame << " armAttempts "
                << activity.armAttempts << " refusals " << activity.refusals
                << " freezes " << activity.freezes << " tilt " << run.tilt
                << " moved " << moved << confirmFields(activity.lastConfirm)
                << '\n';
    }

    if (
      activity.refusals != 0u && activity.lastConfirm.unbalanced
      && run.refusedAt == 0u
    ) {
      run.refusedAt = frame;
      run.refusedTilt = run.tilt;
      run.refusedMoved = moved;
      run.refused = activity.lastConfirm;
    }

    if (runtime.deformableQuiescent(bar)) {
      run.frozenAt = frame;
      run.frozenTilt = run.tilt;
    }

    if (run.tilt >= TIP_TIPPED) {
      run.tippedAt = frame;
    }
  }

  std::cout << label << " run armAttempts " << run.armAttempts << " confirms "
            << run.confirms << " refusals " << run.refusals << " refusedAt "
            << run.refusedAt << " refusedTilt " << run.refusedTilt
            << " refusedMoved " << run.refusedMoved << " frozenAt "
            << run.frozenAt << " frozenTilt " << run.frozenTilt << " tippedAt "
            << run.tippedAt << " tilt " << run.tilt << '\n';
  std::cout << label << " refused" << confirmFields(run.refused) << '\n';

  return run;
}

}

TEST_CASE(
  "a free deformable bar resting on a field slab freezes with its confirm "
  "resolved",
  "[.][freeconfirm]"
) {
  const std::vector<PhysicalMaterial> materials = confirmMaterials();

  for (const talus::deformable::VolumeElementType element :
       plasticbar::ELEMENTS) {
    const Clock clock;

    JoltLibrary library;
    World source;

    source.setMaterials(materials);

    const Entity bar = freeBar(source, element);

    Runtime runtime(source, library);

    const talus::deformable::Settings settings = confirmSettings(element);

    runtime.setDeformableSettings(settings);

    const PreparedMesh& mesh = spinFreeBar(runtime, bar);

    const std::string label =
      std::string("freeconfirm: ") + plasticbar::elementName(element);

    std::cout << label << " nodes " << mesh.restPositions.size() << " rate "
              << CONFIRM_RATE << confirmConfiguration(settings) << '\n';

    const ConfirmRun run = runToFreeze(runtime, bar, CONFIRM_FRAMES, label);

    std::cout << label << " wall seconds " << clock.seconds() << '\n';

    REQUIRE(run.frozenAt != 0u);
    REQUIRE(run.confirms <= CONFIRM_ARMS);
    REQUIRE(run.everyFactored);
    REQUIRE(run.everyResolved);
    REQUIRE_FALSE(run.anyUnbalanced);
    REQUIRE(run.worstCorrection <= talus::runtime::QUIESCENT_CORRECTION);
    REQUIRE(run.worstForce <= CONFIRM_REST_BALANCE);
    REQUIRE(run.worstTorque <= CONFIRM_REST_BALANCE);
  }
}

TEST_CASE(
  "a frozen free deformable bar wakes on an impulse and refreezes within two "
  "arms",
  "[.][freeconfirm]"
) {
  const std::vector<PhysicalMaterial> materials = confirmMaterials();

  for (const talus::deformable::VolumeElementType element :
       plasticbar::ELEMENTS) {
    const Clock clock;

    JoltLibrary library;
    World source;

    source.setMaterials(materials);

    const Entity bar = freeBar(source, element);

    Runtime runtime(source, library);

    const talus::deformable::Settings settings = confirmSettings(element);

    runtime.setDeformableSettings(settings);

    const PreparedMesh& mesh = spinFreeBar(runtime, bar);

    const std::string label =
      std::string("freewake: ") + plasticbar::elementName(element);

    std::cout << label << " nodes " << mesh.restPositions.size() << " rate "
              << CONFIRM_RATE << " crateMass " << WAKE_CRATE_MASS
              << " crateHalf " << WAKE_CRATE_HALF << " crateLift "
              << WAKE_CRATE_LIFT << " crateAlong " << WAKE_CRATE_AT
              << confirmConfiguration(settings) << '\n';

    const ConfirmRun rested =
      runToFreeze(runtime, bar, CONFIRM_FRAMES, label + " rest");

    REQUIRE(rested.frozenAt != 0u);

    const DeformableState& frozen = *runtime.world().deformableState(bar);
    const glm::vec3 centre(landingbar::landingCentre(mesh, frozen));
    const size_t right =
      restNodeAt(mesh, glm::vec3(plasticbar::HALF_LENGTH, 0.0f, 0.0f));
    const size_t left =
      restNodeAt(mesh, glm::vec3(-plasticbar::HALF_LENGTH, 0.0f, 0.0f));
    const glm::vec3 ends = frozen.position[right] - frozen.position[left];
    const glm::vec3 axis = glm::normalize(glm::vec3(ends.x, 0.0f, ends.z));

    float top = -std::numeric_limits<float>::infinity();

    for (const glm::vec3& position : frozen.position) {
      top = std::max(top, position.y);
    }

    const glm::vec3 above = centre + WAKE_CRATE_AT * axis;

    const Dropped crate = dropCrate(
      runtime,
      glm::vec3(above.x, top + WAKE_CRATE_LIFT + WAKE_CRATE_HALF, above.z),
      WAKE_CRATE_HALF,
      WAKE_CRATE_MASS
    );

    WakeReasons reasons;

    uint32_t wokeAt = 0u;

    for (uint32_t frame = 1u; frame <= CONFIRM_FRAMES && wokeAt == 0u;
         ++frame) {
      runtime.step();

      reasons.add(runtime.deformableActivity());

      if (!runtime.deformableQuiescent(bar)) {
        wokeAt = frame;
      }
    }

    REQUIRE(wokeAt != 0u);

    const ConfirmRun woken =
      runToFreeze(runtime, bar, CONFIRM_FRAMES, label + " woken");

    JPH::BodyInterface& bodies = runtime.physics().bodies();

    const float crateHeight = bodies.GetPosition(crate.body).GetY();
    const bool crateAsleep = !bodies.IsActive(crate.body);

    std::cout << label << " restFrozenAt " << rested.frozenAt << " top " << top
              << " wokeAt " << wokeAt << reasons << " refrozenAt "
              << woken.frozenAt << " confirms " << woken.confirms
              << " crateHeight " << crateHeight << " crateAsleep "
              << (crateAsleep ? 1 : 0) << " wall seconds " << clock.seconds()
              << '\n';

    REQUIRE(woken.frozenAt != 0u);
    REQUIRE(woken.confirms <= CONFIRM_ARMS);
    REQUIRE(woken.everyFactored);
    REQUIRE(woken.everyResolved);
    REQUIRE_FALSE(woken.anyUnbalanced);
    REQUIRE(woken.worstCorrection <= talus::runtime::QUIESCENT_CORRECTION);
    REQUIRE(woken.worstForce <= CONFIRM_REST_BALANCE);
    REQUIRE(woken.worstTorque <= CONFIRM_REST_BALANCE);
    REQUIRE(crateHeight > top);
    REQUIRE(crateAsleep);
  }
}

TEST_CASE(
  "a free deformable bar's confirm refuses an unbalanced load",
  "[deformable]"
) {
  const std::vector<PhysicalMaterial> materials = confirmMaterials();
  const glm::vec3 gravity(0.0f, -PhysicsWorld::GRAVITY, 0.0f);

  for (const talus::deformable::VolumeElementType element :
       plasticbar::ELEMENTS) {
    const std::shared_ptr<const PreparedMesh> mesh =
      landingbar::landingBar(materials, element);
    const DeformableState state =
      talus::deformable::restState(*mesh, talus::objectmodel::Pose{});
    const talus::deformable::Settings settings = confirmSettings(element);

    const std::vector<glm::vec3>& rest = mesh->restPositions;
    const float edge = 0.25f * plasticbar::COARSE_VOXEL;
    const double weight = landingbar::landingMass(*mesh)
                          * static_cast<double>(PhysicsWorld::GRAVITY);

    std::vector<glm::vec3> support(rest.size(), glm::vec3(0.0f));

    float shares = 0.0f;

    for (size_t node = 0u; node != rest.size(); ++node) {
      if (rest[node].y > -plasticbar::HALF_WIDTH + edge) {
        continue;
      }

      const float alongX =
        plasticbar::HALF_LENGTH - std::abs(rest[node].x) < edge ? 0.5f : 1.0f;
      const float alongZ =
        plasticbar::HALF_WIDTH - std::abs(rest[node].z) < edge ? 0.5f : 1.0f;

      support[node].y = alongX * alongZ;
      shares += alongX * alongZ;
    }

    REQUIRE(shares > 0.0f);

    for (glm::vec3& share : support) {
      share *= static_cast<float>(weight) / shares;
    }

    const double arm = confirmArm(*mesh, state);
    const double restScale = confirmScale(*mesh, support);

    const float kept = (1.0f - CONFIRM_SHARE) / (1.0f + CONFIRM_SHARE);

    std::vector<glm::vec3> lighter(support);

    for (glm::vec3& share : lighter) {
      share *= kept;
    }

    const size_t right = nodeAt(
      state,
      glm::vec3(plasticbar::HALF_LENGTH, plasticbar::HALF_WIDTH, 0.0f)
    );
    const size_t left = nodeAt(
      state,
      glm::vec3(-plasticbar::HALF_LENGTH, plasticbar::HALF_WIDTH, 0.0f)
    );
    const double span =
      static_cast<double>(state.position[right].x - state.position[left].x);
    const double share = static_cast<double>(CONFIRM_SHARE);
    const double couple = share * restScale * arm / (span - 2.0 * share * arm);

    std::vector<glm::vec3> twisted(support);

    twisted[right].y += static_cast<float>(couple);
    twisted[left].y -= static_cast<float>(couple);

    const float pinch = CONFIRM_PINCH * static_cast<float>(weight);

    std::vector<glm::vec3> pinched(support);

    for (const float across :
         {-plasticbar::HALF_WIDTH, 0.0f, plasticbar::HALF_WIDTH}) {
      const size_t upRight = nodeAt(
        state,
        glm::vec3(plasticbar::HALF_LENGTH, -plasticbar::HALF_WIDTH, across)
      );
      const size_t upLeft = nodeAt(
        state,
        glm::vec3(-plasticbar::HALF_LENGTH, -plasticbar::HALF_WIDTH, across)
      );
      const size_t down =
        nodeAt(state, glm::vec3(0.0f, plasticbar::HALF_WIDTH, across));

      pinched[upRight].y += pinch / 6.0f;
      pinched[upLeft].y += pinch / 6.0f;
      pinched[down].y -= pinch / 3.0f;
    }

    const auto confirm = [&](const std::vector<glm::vec3>& load) {
      return talus::deformable::checkEquilibrium(
        *mesh,
        state,
        materials,
        settings,
        gravity,
        load
      );
    };

    const talus::deformable::EquilibriumCheck atRest = confirm(support);
    const talus::deformable::EquilibriumCheck heavy = confirm(lighter);
    const talus::deformable::EquilibriumCheck turning = confirm(twisted);
    const talus::deformable::EquilibriumCheck redistributed = confirm(pinched);

    const std::string label =
      std::string("unbalancedconfirm: ") + plasticbar::elementName(element);

    std::cout << label << " weight " << weight << " arm " << arm
              << " restScale " << restScale << " kept " << kept << " couple "
              << couple << " pinch " << pinch << confirmConfiguration(settings)
              << '\n';
    std::cout << label << " rest" << confirmFields(atRest) << '\n';
    std::cout << label << " force" << confirmFields(heavy) << '\n';
    std::cout << label << " torque" << confirmFields(turning) << '\n';
    std::cout << label << " redistributed" << confirmFields(redistributed)
              << '\n';

    REQUIRE(atRest.freeModes == 6u);
    REQUIRE(atRest.factored);
    REQUIRE_FALSE(atRest.cgCapped);
    REQUIRE_FALSE(atRest.unbalanced);
    REQUIRE(atRest.maxCorrection <= talus::runtime::QUIESCENT_CORRECTION);
    REQUIRE(atRest.forceBalance <= CONFIRM_REST_BALANCE);
    REQUIRE(atRest.torqueBalance <= CONFIRM_REST_BALANCE);

    REQUIRE(heavy.unbalanced);
    REQUIRE(heavy.forceBalance > talus::deformable::EQUILIBRIUM_BALANCE);

    REQUIRE(turning.unbalanced);
    REQUIRE(turning.forceBalance <= CONFIRM_REST_BALANCE);
    REQUIRE(turning.torqueBalance > talus::deformable::EQUILIBRIUM_BALANCE);

    REQUIRE_FALSE(redistributed.cgCapped);
    REQUIRE_FALSE(redistributed.unbalanced);
    REQUIRE(redistributed.forceBalance <= CONFIRM_REST_BALANCE);
    REQUIRE(redistributed.torqueBalance <= CONFIRM_REST_BALANCE);
    REQUIRE(redistributed.maxCorrection > talus::runtime::QUIESCENT_CORRECTION);
  }
}

TEST_CASE(
  "a deformable bar hanging from one anchor freezes on its first confirm and "
  "refuses a torque about it",
  "[deformable]"
) {
  const HangingRow row = hangingRow(1u);

  REQUIRE(row.run.frozenAt != 0u);
  REQUIRE(row.run.refusals == 0u);
  REQUIRE(row.run.confirms == 1u);
  REQUIRE(row.run.everyFactored);
  REQUIRE(row.run.everyResolved);
  REQUIRE_FALSE(row.run.anyUnbalanced);
  REQUIRE(row.run.worstForce <= CONFIRM_REST_BALANCE);
  REQUIRE(row.run.worstTorque <= CONFIRM_REST_BALANCE);

  REQUIRE(row.atRest.freeModes == 3u);
  REQUIRE(row.atRest.factored);
  REQUIRE_FALSE(row.atRest.cgCapped);
  REQUIRE_FALSE(row.atRest.unbalanced);
  REQUIRE(row.atRest.maxCorrection <= talus::runtime::QUIESCENT_CORRECTION);
  REQUIRE(row.atRest.torqueBalance <= CONFIRM_REST_BALANCE);

  REQUIRE(row.turned.unbalanced);
  REQUIRE(row.turned.torqueBalance > talus::deformable::EQUILIBRIUM_BALANCE);
}

TEST_CASE(
  "a deformable bar hanging from two anchors freezes with one free mode and "
  "refuses a torque about them",
  "[deformable]"
) {
  const HangingRow row = hangingRow(2u);

  REQUIRE(row.run.frozenAt != 0u);
  REQUIRE(row.run.refusals == 0u);
  REQUIRE(row.run.confirms == 1u);
  REQUIRE(row.run.everyFactored);
  REQUIRE(row.run.everyResolved);
  REQUIRE_FALSE(row.run.anyUnbalanced);
  REQUIRE(row.run.worstForce <= CONFIRM_REST_BALANCE);
  REQUIRE(row.run.worstTorque <= CONFIRM_REST_BALANCE);

  REQUIRE(row.atRest.freeModes == 1u);
  REQUIRE(row.atRest.factored);
  REQUIRE_FALSE(row.atRest.cgCapped);
  REQUIRE_FALSE(row.atRest.unbalanced);
  REQUIRE(row.atRest.maxCorrection <= talus::runtime::QUIESCENT_CORRECTION);
  REQUIRE(row.atRest.torqueBalance <= CONFIRM_REST_BALANCE);

  REQUIRE(row.turned.unbalanced);
  REQUIRE(row.turned.torqueBalance > talus::deformable::EQUILIBRIUM_BALANCE);
  REQUIRE_FALSE(row.reacted.unbalanced);
}

TEST_CASE(
  "a free deformable bar tipping over its edge from rest refuses its confirm "
  "as unbalanced and does not freeze while it tips",
  "[deformable]"
) {
  const TipRun run = tipOverEdge();

  REQUIRE(run.frozenAt == 0u);
  REQUIRE(run.armAttempts >= 1u);
  REQUIRE(run.refusedAt != 0u);
  REQUIRE(run.refused.unbalanced);
  REQUIRE(run.refused.freeModes == 6u);
  REQUIRE(run.refused.factored);
  REQUIRE_FALSE(run.refused.cgCapped);
  REQUIRE(run.refused.maxCorrection <= talus::runtime::QUIESCENT_CORRECTION);
  REQUIRE(
    std::max(run.refused.forceBalance, run.refused.torqueBalance)
    > talus::deformable::EQUILIBRIUM_BALANCE
  );
  REQUIRE(run.tippedAt > run.refusedAt);
}

namespace {

constexpr float YIELDING_SKIN = 0.2f * plasticbar::COARSE_VOXEL;
constexpr float YIELDING_REST_HEIGHT = plasticbar::HALF_WIDTH + YIELDING_SKIN;
constexpr double YIELDING_MOMENTUM = 1.0e-6;
constexpr double YIELDING_ENERGY = 1.0e-6;
constexpr double YIELDING_MASS = 1.0e-6;

Entity yieldingSlab(World& world) {
  talus::objectmodel::SdfGrid grid{};

  grid.resolution = glm::uvec3(29u, 5u, 29u);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, -0.05f, 0.0f);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(glm::vec3(-0.6f, -0.05f, -0.6f), glm::vec3(0.6f, 0.05f, 0.6f), 0u)
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(
      world.primitives(entity),
      glm::vec3(-0.7f, -0.1f, -0.7f),
      glm::vec3(0.7f, 0.1f, 0.7f),
      grid.resolution
    )
  );

  world.addStaticBody(entity);

  return entity;
}

Entity wideYieldingSlab(World& world, float extent) {
  constexpr float SLAB_VOXEL = 0.05f;

  const glm::vec3 half(extent, 0.05f, extent);
  const glm::vec3 reach =
    half + glm::vec3(2.0f * SLAB_VOXEL, 0.05f, 2.0f * SLAB_VOXEL);

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round(2.0f * reach / SLAB_VOXEL)) + glm::uvec3(1u);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, -0.05f, 0.0f);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(entity, spanning(-half, half, 0u)));

  world.setSampledSdf(
    entity,
    talus::field::bake(world.primitives(entity), -reach, reach, grid.resolution)
  );

  world.addStaticBody(entity);

  return entity;
}

Entity yieldingBox(
  World& world,
  const talus::objectmodel::Pose& pose,
  uint32_t lane,
  const glm::vec3& half,
  bool block,
  float skin = YIELDING_SKIN
) {
  const float voxel = plasticbar::COARSE_VOXEL;
  const glm::vec3 low = -half - glm::vec3(voxel);
  const glm::vec3 high =
    half + glm::vec3(voxel)
    + (block ? glm::vec3(0.0f, 2.0f * plasticbar::HALF_WIDTH, 0.0f) : glm::vec3(0.0f));

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round((high - low) / voxel)) + glm::uvec3(1u);

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(-half - glm::vec3(skin), half + glm::vec3(skin), lane)
  ));

  if (block) {
    REQUIRE(world.addPrimitive(
      entity,
      spanning(
        glm::vec3(0.1f, plasticbar::HALF_WIDTH, -plasticbar::HALF_WIDTH)
          - glm::vec3(YIELDING_SKIN),
        glm::vec3(
          plasticbar::HALF_LENGTH,
          3.0f * plasticbar::HALF_WIDTH,
          plasticbar::HALF_WIDTH
        ) + glm::vec3(YIELDING_SKIN),
        lane
      )
    ));
  }

  world.setSampledSdf(
    entity,
    talus::field::bake(world.primitives(entity), low, high, grid.resolution)
  );

  world.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    entity,
    talus::objectmodel::DeformableBody{
      talus::objectmodel::VolumeElementType::Hex
    }
  );

  return entity;
}

Entity yieldingBar(
  World& world,
  const talus::objectmodel::Pose& pose,
  uint32_t lane,
  bool block
) {
  return yieldingBox(
    world,
    pose,
    lane,
    glm::vec3(
      plasticbar::HALF_LENGTH,
      plasticbar::HALF_WIDTH,
      plasticbar::HALF_WIDTH
    ),
    block
  );
}

void yieldingPrepare(Runtime& runtime) {
  const std::vector<Entity> grids(
    runtime.world().sdfGrids().begin(),
    runtime.world().sdfGrids().end()
  );

  for (const Entity entity : grids) {
    runtime.refreshBody(entity);
  }

  runtime.optimizeBroadPhase();
}

std::optional<JPH::BodyID> yieldingBody(Runtime& runtime, Entity entity) {
  const auto* id =
    runtime.world().registry().try_get<talus::objectmodel::BodyId>(entity);

  if (id == nullptr) {
    return std::nullopt;
  }

  return JPH::BodyID(id->id);
}

double yieldingApart(const glm::dvec3& before, const glm::dvec3& after) {
  const double scale = std::max(glm::length(before), glm::length(after));

  return scale > 0.0 ? glm::length(after - before) / scale : 0.0;
}

struct TransitionShare {
  double linear = 0.0;
  double angular = 0.0;
};

TransitionShare transitionShare(
  const talus::runtime::YieldTransition& transition,
  const windowmonitor::MonitoredBody& body
) {
  return {
    windowmonitor::momentumApart(
      transition.linearBefore,
      transition.linearAfter,
      windowmonitor::linearFloor(body)
    ),
    windowmonitor::momentumApart(
      transition.angularBefore,
      transition.angularAfter,
      windowmonitor::angularFloor(body)
    )
  };
}

float yieldingPlastic(const DeformableState& state) {
  float largest = 0.0f;

  for (const talus::objectmodel::PlasticPoint& point : state.plastic) {
    largest = std::max(largest, point.equivalent);
  }

  return largest;
}

glm::vec2 yieldingDamping(Runtime& runtime, JPH::BodyID id) {
  const JPH::BodyLockRead lock(
    runtime.physics().system().GetBodyLockInterface(),
    id
  );

  REQUIRE(lock.Succeeded());

  const JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();

  REQUIRE(motion != nullptr);

  return glm::vec2(motion->GetLinearDamping(), motion->GetAngularDamping());
}
}

TEST_CASE(
  "a yielding deformable dropped below its yield speed stays a Jolt body",
  "[deformable]"
) {
  constexpr float GAP = 0.003f;
  constexpr uint32_t FRAMES = 90u;

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + GAP, 0.0f);

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());
  REQUIRE(runtime.rigidUntilYield(bar));

  uint32_t promotions = 0u;
  uint32_t steppedNodes = 0u;
  bool kept = true;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    promotions += runtime.deformableActivity().promotions;
    steppedNodes += runtime.deformableReport().nodes;
    kept = kept && yieldingBody(runtime, bar) == id;
  }

  const float height =
    runtime.world().registry().get<talus::objectmodel::Pose>(bar).position.y;
  const bool asleep = !runtime.physics().bodies().IsActive(*id);
  const glm::vec2 damping = yieldingDamping(runtime, *id);

  std::cout << "rigidbelowyield: impact "
            << std::sqrt(2.0f * PhysicsWorld::GRAVITY * GAP) << " trigger "
            << plasticbar::YIELD
                 / std::sqrt(plasticbar::DENSITY * plasticbar::YOUNGS)
            << " promotions " << promotions << " steppedNodes " << steppedNodes
            << " height " << height << " asleep " << asleep << " linearDamping "
            << damping.x << " angularDamping " << damping.y << std::endl;

  REQUIRE(promotions == 0u);
  REQUIRE(steppedNodes == 0u);
  REQUIRE(kept);
  REQUIRE(asleep);
  REQUIRE(damping.x == 0.0f);
  REQUIRE(damping.y == 0.0f);
  REQUIRE(
    std::abs(height - YIELDING_REST_HEIGHT) < 0.5f * plasticbar::COARSE_VOXEL
  );
}

TEST_CASE(
  "a yielding deformable promotes on impact and returns to Jolt with its "
  "momentum",
  "[.][deformable]"
) {
  constexpr float DROP = 0.5f;
  constexpr uint32_t FRAMES = 90u;

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + DROP, 0.0f);
  pose.rotation = glm::angleAxis(0.3f, glm::vec3(0.0f, 0.0f, 1.0f));

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());

  runtime.physics().bodies().SetAngularVelocity(
    *id,
    JPH::Vec3(0.4f, 0.0f, 1.5f)
  );

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*mesh);

  uint32_t promotions = 0u;
  uint32_t demotions = 0u;
  double worstLinear = 0.0;
  double worstAngular = 0.0;
  double worstEnergy = -std::numeric_limits<double>::infinity();
  float worstPose = 0.0f;
  float permanent = 0.0f;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    for (const talus::runtime::YieldTransition& transition :
         runtime.deformableActivity().transitions) {
      const TransitionShare share = transitionShare(transition, monitored);
      const double linear = share.linear;
      const double angular = share.angular;
      const double energy = (transition.kineticAfter + transition.penaltyAfter
                             - transition.kineticBefore)
                            / transition.kineticBefore;

      std::cout << "promotereturn: frame " << frame << ' '
                << (transition.promotion ? "promote" : "demote") << " approach "
                << transition.approach << " linear " << linear << " angular "
                << angular << " |L| " << glm::length(transition.angularBefore)
                << " energy " << energy << " permanent " << transition.permanent
                << " penaltyAfter " << transition.penaltyAfter << '\n';

      promotions += transition.promotion ? 1u : 0u;
      demotions += transition.promotion ? 0u : 1u;
      worstLinear = std::max(worstLinear, linear);
      worstAngular = std::max(worstAngular, angular);
      worstEnergy = std::max(worstEnergy, energy);
      permanent = std::max(permanent, transition.permanent);
    }

    if (runtime.promotedDeformable(bar)) {
      const DeformableState* state = runtime.world().deformableState(bar);

      REQUIRE(state != nullptr);

      glm::vec3 centre(0.0f);
      glm::vec3 restCentre(0.0f);
      float mass = 0.0f;

      for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
        centre += mesh->nodeMass[node] * state->position[node];
        restCentre += mesh->nodeMass[node] * mesh->restPositions[node];
        mass += mesh->nodeMass[node];
      }

      const glm::vec3 posed = glm::vec3(
        talus::objectmodel::model(
          runtime.world().registry().get<talus::objectmodel::Pose>(bar)
        )
        * glm::vec4(restCentre / mass, 1.0f)
      );

      worstPose = std::max(worstPose, glm::distance(posed, centre / mass));
    }
  }

  const DeformableState* state = runtime.world().deformableState(bar);

  REQUIRE(state != nullptr);

  const glm::vec2 damping = yieldingDamping(runtime, *id);

  std::cout << "promotereturn: promotions " << promotions << " demotions "
            << demotions << " worstLinear " << worstLinear << " worstAngular "
            << worstAngular << " worstEnergy " << worstEnergy << " worstPose "
            << worstPose << " plastic " << yieldingPlastic(*state)
            << " permanent " << permanent << " linearDamping " << damping.x
            << " angularDamping " << damping.y << std::endl;

  REQUIRE(promotions >= 1u);
  REQUIRE(demotions >= 1u);
  REQUIRE_FALSE(runtime.promotedDeformable(bar));
  REQUIRE(yieldingBody(runtime, bar) == id);
  REQUIRE(runtime.physics().bodies().IsAdded(*id));
  REQUIRE(damping.x == 0.0f);
  REQUIRE(damping.y == 0.0f);
  REQUIRE(yieldingPlastic(*state) > 0.0f);
  REQUIRE(worstLinear <= windowmonitor::transitionBound(monitored));
  REQUIRE(worstAngular <= windowmonitor::transitionBound(monitored));
  REQUIRE(worstEnergy <= YIELDING_ENERGY);
  REQUIRE(worstPose < 1.0e-3f);
}

TEST_CASE(
  "a yielding deformable demoting faster than Jolt's default angular limit "
  "keeps its angular momentum",
  "[.][deformable]"
) {
  constexpr float SPIN = 60.0f;
  constexpr uint32_t FRAMES = 30u;

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.3f, 1.0f, -0.2f);
  pose.rotation =
    glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)));

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());

  const glm::vec3 spin = SPIN * glm::normalize(glm::vec3(0.3f, 1.0f, 0.6f));
  const glm::vec3 drift(0.2f, -0.1f, 0.3f);

  runtime.physics().bodies().SetLinearVelocity(
    *id,
    JPH::Vec3(drift.x, drift.y, drift.z)
  );
  runtime.physics().bodies().SetAngularVelocity(
    *id,
    JPH::Vec3(spin.x, spin.y, spin.z)
  );

  REQUIRE(
    runtime.physics().bodies().GetAngularVelocity(*id).Length() > 0.99f * SPIN
  );
  REQUIRE(runtime.forcePromotion(bar));

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*mesh);

  double mass = 0.0;
  glm::dvec3 restCentre(0.0);

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    mass += static_cast<double>(mesh->nodeMass[node]);
    restCentre += static_cast<double>(mesh->nodeMass[node])
                  * glm::dvec3(mesh->restPositions[node]);
  }

  restCentre /= mass;

  glm::dmat3 restInertia(0.0);

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh->restPositions[node]) - restCentre;

    restInertia +=
      static_cast<double>(mesh->nodeMass[node])
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const float limit = JPH::BodyCreationSettings().mMaxAngularVelocity;

  uint32_t promotions = 0u;
  uint32_t demotions = 0u;
  double linear = 0.0;
  double angular = 0.0;
  double energy = 0.0;
  double restEnergy = 0.0;
  float returned = 0.0f;

  for (uint32_t frame = 0u; frame != FRAMES && demotions == 0u; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    for (const talus::runtime::YieldTransition& transition :
         runtime.deformableActivity().transitions) {
      if (transition.promotion) {
        ++promotions;

        continue;
      }

      ++demotions;

      const TransitionShare share = transitionShare(transition, monitored);

      linear = share.linear;
      angular = share.angular;
      energy = (transition.kineticAfter - transition.kineticBefore)
               / transition.kineticBefore;
      returned = runtime.physics().bodies().GetAngularVelocity(*id).Length();

      const DeformableState* state = runtime.world().deformableState(bar);

      REQUIRE(state != nullptr);

      const talus::deformable::MeshFrame fit =
        talus::deformable::meshFrame(*mesh, state->position);
      const glm::dmat3 turned =
        fit.rotation * restInertia * glm::transpose(fit.rotation);
      const double restKinetic =
        0.5 * glm::dot(transition.linearBefore, transition.linearBefore) / mass
        + 0.5
            * glm::dot(
              transition.angularBefore,
              glm::inverse(turned) * transition.angularBefore
            );

      restEnergy =
        (restKinetic - transition.kineticBefore) / transition.kineticBefore;

      std::cout << "fastdemotion: frame " << frame << " linear " << linear
                << " angular " << angular << " |L| "
                << glm::length(transition.angularBefore) << " energy " << energy
                << " restInertiaEnergy " << restEnergy << " spinAfter "
                << returned << " limit " << limit << '\n';
    }
  }

  std::cout << "fastdemotion: promotions " << promotions << " demotions "
            << demotions << " linear " << linear << " angular " << angular
            << " energy " << energy << " restInertiaEnergy " << restEnergy
            << " spinAfter " << returned << std::endl;

  REQUIRE(promotions == 1u);
  REQUIRE(demotions == 1u);
  REQUIRE_FALSE(runtime.promotedDeformable(bar));
  REQUIRE(linear <= windowmonitor::transitionBound(monitored));
  REQUIRE(angular <= windowmonitor::transitionBound(monitored));
  REQUIRE(energy <= YIELDING_ENERGY);
  REQUIRE(restEnergy > YIELDING_ENERGY);
  REQUIRE(returned > limit);
}

TEST_CASE(
  "a yielding deformable's Jolt body carries its mesh's mass",
  "[deformable]"
) {
  JoltLibrary library;
  World source;

  std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(plasticbar::YIELD);

  materials[plasticbar::LANE].density = 500.0f;

  source.setMaterials(materials);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.3f, 1.0f, -0.2f);
  pose.rotation =
    glm::angleAxis(0.6f, glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f)));

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, true);

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);
  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(id.has_value());
  REQUIRE(mesh != nullptr);

  double mass = 0.0;
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    mass += static_cast<double>(mesh->nodeMass[node]);
    centre += static_cast<double>(mesh->nodeMass[node])
              * glm::dvec3(mesh->restPositions[node]);
  }

  centre /= mass;

  glm::dmat3 inertia(0.0);

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(mesh->restPositions[node]) - centre;

    inertia +=
      static_cast<double>(mesh->nodeMass[node])
      * (glm::dot(arm, arm) * glm::dmat3(1.0) - glm::outerProduct(arm, arm));
  }

  const glm::dmat3 turn = glm::mat3_cast(glm::dquat(pose.rotation));
  const glm::dvec3 placed = turn * centre + glm::dvec3(pose.position);
  const glm::dmat3 world = turn * inertia * glm::transpose(turn);

  double joltMass = 0.0;
  glm::dvec3 joltCentre(0.0);
  glm::dmat3 joltInertia(0.0);

  {
    const JPH::BodyLockRead lock(
      runtime.physics().system().GetBodyLockInterface(),
      *id
    );

    REQUIRE(lock.Succeeded());

    const JPH::Body& body = lock.GetBody();
    const JPH::Vec3 at = body.GetCenterOfMassPosition();
    const JPH::Mat44 inverse = body.GetInverseInertia();

    joltMass =
      1.0 / static_cast<double>(body.GetMotionProperties()->GetInverseMass());
    joltCentre = glm::dvec3(at.GetX(), at.GetY(), at.GetZ());

    for (int column = 0; column != 3; ++column) {
      const JPH::Vec3 held = inverse.GetColumn3(column);

      joltInertia[column] = glm::dvec3(held.GetX(), held.GetY(), held.GetZ());
    }

    joltInertia = glm::inverse(joltInertia);
  }

  double largest = 0.0;
  double inertiaApart = 0.0;

  for (int column = 0; column != 3; ++column) {
    for (int row = 0; row != 3; ++row) {
      largest = std::max(largest, std::abs(world[column][row]));
      inertiaApart = std::max(
        inertiaApart,
        std::abs(joltInertia[column][row] - world[column][row])
      );
    }
  }

  const double massApart = std::abs(joltMass - mass) / mass;
  const double centreApart =
    glm::length(joltCentre - placed) / glm::length(placed);

  inertiaApart /= largest;

  std::cout << "meshmass: mesh " << mass << " jolt " << joltMass
            << " massApart " << massApart << " centreApart " << centreApart
            << " inertiaApart " << inertiaApart << std::endl;

  REQUIRE(massApart <= YIELDING_MASS);
  REQUIRE(centreApart <= YIELDING_MASS);
  REQUIRE(inertiaApart <= YIELDING_MASS);
}

TEST_CASE(
  "an elastic deformable keeps the always-deformable path",
  "[deformable]"
) {
  constexpr uint32_t ELASTIC_LANE = 1u;
  constexpr uint32_t ANCHORED_LANE = 2u;
  constexpr uint32_t FRAMES = 10u;

  JoltLibrary library;
  World source;

  std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());

  materials.push_back(
    plasticbar::barMaterials(plasticbar::YIELD)[plasticbar::LANE]
  );

  source.setMaterials(materials);

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose falling{};
  falling.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + 0.2f, -0.2f);

  talus::objectmodel::Pose held{};
  held.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + 0.2f, 0.2f);

  const Entity elastic = yieldingBar(source, falling, ELASTIC_LANE, false);
  const Entity anchored = yieldingBar(source, held, ANCHORED_LANE, false);

  const Entity support = source.createAuthored();

  talus::objectmodel::Pose clamp{};
  clamp.position = glm::vec3(-0.25f, YIELDING_REST_HEIGHT + 0.2f, 0.2f);

  source.emplaceAuthored<talus::objectmodel::Pose>(support, clamp);
  source.emplaceAuthored<talus::objectmodel::Support>(
    support,
    talus::objectmodel::Support{glm::vec3(0.1f, 0.1f, 0.1f)}
  );

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const PreparedMesh* elasticMesh = runtime.world().deformablePrepared(elastic);
  const PreparedMesh* anchoredMesh =
    runtime.world().deformablePrepared(anchored);

  REQUIRE(elasticMesh != nullptr);
  REQUIRE(anchoredMesh != nullptr);
  REQUIRE(
    std::count(
      anchoredMesh->anchored.begin(),
      anchoredMesh->anchored.end(),
      uint8_t{1u}
    )
    > 0
  );

  const uint32_t nodes = static_cast<uint32_t>(
    elasticMesh->restPositions.size() + anchoredMesh->restPositions.size()
  );

  uint32_t everyFrame = 0u;
  uint32_t promotions = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    everyFrame += runtime.deformableReport().nodes == nodes ? 1u : 0u;
    promotions += runtime.deformableActivity().promotions;
  }

  const bool elasticBody = yieldingBody(runtime, elastic).has_value();
  const bool anchoredBody = yieldingBody(runtime, anchored).has_value();

  std::cout << "elasticunchanged: steppedFrames " << everyFrame << " of "
            << FRAMES << " elasticBody " << elasticBody << " anchoredBody "
            << anchoredBody << " promotions " << promotions << std::endl;

  REQUIRE_FALSE(runtime.rigidUntilYield(elastic));
  REQUIRE_FALSE(runtime.rigidUntilYield(anchored));
  REQUIRE_FALSE(elasticBody);
  REQUIRE_FALSE(anchoredBody);
  REQUIRE(everyFrame == FRAMES);
  REQUIRE(promotions == 0u);
}

namespace {

uint64_t sharedYieldHash(uint32_t threads) {
  constexpr float BEAM_OFFSET = 0.075f;
  constexpr float BAR_AT = 0.3f;
  constexpr uint32_t YIELD_LANE = 2u;
  constexpr float YIELD_DENSITY = 150.0f;
  constexpr uint32_t STEPS = 24u;

  JoltLibrary library;

  World source;
  const Entity nearBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, -BEAM_OFFSET));
  const Entity farBeam =
    beamEntity(source, COARSE_VOXEL, glm::vec3(0.0f, 0.0f, BEAM_OFFSET));

  std::vector<PhysicalMaterial> materials = beamMaterials();
  PhysicalMaterial yielding =
    plasticbar::barMaterials(plasticbar::YIELD)[plasticbar::LANE];

  yielding.density = YIELD_DENSITY;
  materials.push_back(yielding);

  REQUIRE(materials.size() == YIELD_LANE + 1u);

  source.setMaterials(materials);

  Runtime runtime(source, library);

  prepare(runtime, nearBeam);

  REQUIRE(runtime.world().deformablePrepared(farBeam) != nullptr);

  talus::deformable::Settings settings = dynamicSolve(4u);

  settings.stepThreads = threads;
  settings.reportContactDiagnostics = true;

  runtime.setDeformableSettings(settings);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(
    BAR_AT,
    glm::max(
      surfaceAt(runtime, nearBeam, BAR_AT),
      surfaceAt(runtime, farBeam, BAR_AT)
    ) + YIELDING_REST_HEIGHT
      + 0.001f,
    0.0f
  );
  pose.rotation =
    glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f));

  const Entity bar = yieldingBar(runtime.world(), pose, YIELD_LANE, false);

  runtime.refreshBody(bar);
  runtime.optimizeBroadPhase();

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);
  const JPH::BodyInterface& bodies = runtime.physics().bodies();

  REQUIRE(id.has_value());
  REQUIRE(runtime.rigidUntilYield(bar));
  REQUIRE(bodies.GetMotionType(*id) == JPH::EMotionType::Dynamic);
  REQUIRE(
    bodies.GetShape(*id)->GetSubType() == JPH::EShapeSubType::OffsetCenterOfMass
  );

  uint32_t contacts = 0u;
  uint32_t promotions = 0u;
  std::set<std::pair<uint32_t, uint32_t>> touches;

  for (uint32_t step = 0u; step != STEPS; ++step) {
    runtime.step();

    contacts += runtime.deformableReport().contacts;
    promotions += runtime.deformableActivity().promotions;

    for (const talus::deformable::ContactSolve& solve :
         runtime.deformableReport().contactSolves) {
      for (const talus::deformable::ContactSample& sample : solve.samples) {
        touches.emplace(solve.deformable, sample.partner);
      }
    }
  }

  const auto touched = [&touches, &id](Entity beam) {
    return touches.contains(
      {static_cast<uint32_t>(beam), id->GetIndexAndSequenceNumber()}
    );
  };

  REQUIRE(contacts > 0u);
  REQUIRE_FALSE(runtime.deformableReport().nonFinite);
  REQUIRE(touched(nearBeam));
  REQUIRE(touched(farBeam));
  REQUIRE(promotions == 0u);
  REQUIRE_FALSE(runtime.promotedDeformable(bar));
  REQUIRE(yieldingBody(runtime, bar) == id);

  uint64_t hash = 14695981039346656037ull;

  for (const Entity beam : {nearBeam, farBeam}) {
    const DeformableState* state = runtime.world().deformableState(beam);

    REQUIRE(state != nullptr);

    for (const glm::vec3& position : state->position) {
      hash = mixBits(hash, position);
    }

    for (const glm::vec3& velocity : state->velocity) {
      hash = mixBits(hash, velocity);
    }

    for (const talus::objectmodel::PlasticPoint& point : state->plastic) {
      hash = mixBits(mixBits(hash, point.strain), point.equivalent);
    }

    for (const talus::objectmodel::CommittedTet& tet : state->committed) {
      hash = mixBits(hash, tet.rotation);
    }
  }

  hash = mixBits(hash, JPH::Vec3(bodies.GetPosition(*id)));
  hash = mixBits(hash, bodies.GetLinearVelocity(*id));
  hash = mixBits(hash, bodies.GetAngularVelocity(*id));

  return hash;
}

}

TEST_CASE(
  "two deformables resting on one unpromoted yielding body step bitwise "
  "alike on one and several threads",
  "[deformable]"
) {
  const uint64_t serial = sharedYieldHash(1u);
  const uint64_t threaded = sharedYieldHash(4u);

  std::cout << "shared yielding body: serial hash " << serial
            << " threaded hash " << threaded << '\n';

  REQUIRE(serial == threaded);
}

namespace {

constexpr double STRUCK_MOMENTUM = 1.0e-3;
constexpr float STRUCK_BOX_HALF = 0.05f;
constexpr float STRUCK_VOXEL = 0.025f;

struct YieldMotion {
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
  double kinetic = 0.0;
};

YieldMotion joltMotion(Runtime& runtime, JPH::BodyID id) {
  const JPH::BodyLockRead lock(
    runtime.physics().system().GetBodyLockInterface(),
    id
  );

  REQUIRE(lock.Succeeded());

  const JPH::Body& body = lock.GetBody();
  const JPH::Vec3 moving = body.GetLinearVelocity();
  const JPH::Vec3 turning = body.GetAngularVelocity();
  const JPH::Vec3 at = body.GetCenterOfMassPosition();
  const JPH::Mat44 inverse = body.GetInverseInertia();

  glm::dmat3 inertia(0.0);

  for (int column = 0; column != 3; ++column) {
    const JPH::Vec3 held = inverse.GetColumn3(column);

    inertia[column] = glm::dvec3(held.GetX(), held.GetY(), held.GetZ());
  }

  inertia = glm::inverse(inertia);

  const double mass =
    1.0 / static_cast<double>(body.GetMotionProperties()->GetInverseMass());
  const glm::dvec3 linear(moving.GetX(), moving.GetY(), moving.GetZ());
  const glm::dvec3 spin(turning.GetX(), turning.GetY(), turning.GetZ());
  const glm::dvec3 centre(at.GetX(), at.GetY(), at.GetZ());

  YieldMotion motion;

  motion.linear = mass * linear;
  motion.angular = glm::cross(centre, motion.linear) + inertia * spin;
  motion.kinetic = 0.5 * mass * glm::dot(linear, linear)
                   + 0.5 * glm::dot(spin, inertia * spin);

  return motion;
}

YieldMotion stateMotion(
  const PreparedMesh& mesh,
  const DeformableState& state
) {
  YieldMotion motion;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);
    const glm::dvec3 moving(state.velocity[node]);

    motion.linear += weight * moving;
    motion.angular +=
      weight * glm::cross(glm::dvec3(state.position[node]), moving);
    motion.kinetic += 0.5 * weight * glm::dot(moving, moving);
  }

  return motion;
}

YieldMotion barMotion(Runtime& runtime, Entity bar, JPH::BodyID id) {
  if (!runtime.promotedDeformable(bar)) {
    return joltMotion(runtime, id);
  }

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);
  const DeformableState* state = runtime.world().deformableState(bar);

  REQUIRE(mesh != nullptr);
  REQUIRE(state != nullptr);

  return stateMotion(*mesh, *state);
}

Entity struckBox(World& world, const glm::vec3& at) {
  const glm::vec3 reach(STRUCK_BOX_HALF + 2.0f * STRUCK_VOXEL);

  talus::objectmodel::SdfGrid grid{};

  grid.resolution =
    glm::uvec3(glm::round(2.0f * reach / STRUCK_VOXEL)) + glm::uvec3(1u);

  talus::objectmodel::Pose pose{};
  pose.position = at;

  const Entity entity = world.createSdfGrid(pose, grid);

  REQUIRE(world.addPrimitive(
    entity,
    spanning(glm::vec3(-STRUCK_BOX_HALF), glm::vec3(STRUCK_BOX_HALF), 0u)
  ));

  world.setSampledSdf(
    entity,
    talus::field::bake(world.primitives(entity), -reach, reach, grid.resolution)
  );

  world.addRigidBody(entity);

  return entity;
}

struct TransitionWorst {
  uint32_t promotions = 0u;
  uint32_t demotions = 0u;
  double linear = 0.0;
  double angular = 0.0;
  double energy = -std::numeric_limits<double>::infinity();
};

void countTransitions(
  Runtime& runtime,
  TransitionWorst& worst,
  const char* label,
  uint32_t frame
) {
  for (const talus::runtime::YieldTransition& transition :
       runtime.deformableActivity().transitions) {
    const double linear =
      yieldingApart(transition.linearBefore, transition.linearAfter);
    const double angular =
      yieldingApart(transition.angularBefore, transition.angularAfter);
    const double added = transition.kineticAfter + transition.penaltyAfter
                         - transition.kineticBefore;
    const double energy =
      transition.kineticBefore > 0.0 ? added / transition.kineticBefore : added;

    std::cout << label << ": frame " << frame << ' '
              << (transition.promotion ? "promote" : "demote") << " approach "
              << transition.approach << " linear " << linear << " angular "
              << angular << " energy " << energy << " permanent "
              << transition.permanent << '\n';

    worst.promotions += transition.promotion ? 1u : 0u;
    worst.demotions += transition.promotion ? 0u : 1u;
    worst.linear = std::max(worst.linear, linear);
    worst.angular = std::max(worst.angular, angular);
    worst.energy = std::max(worst.energy, energy);
  }
}

struct StruckRun {
  TransitionWorst worst;
  double linear = 0.0;
  double angular = 0.0;
  double energy = 0.0;
};

StruckRun strikeYieldingBar(float speed) {
  constexpr uint32_t FRAMES = 60u;
  constexpr float GAP = 0.02f;
  constexpr float OFFSET = 0.1f;

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  const Entity bar =
    yieldingBar(source, talus::objectmodel::Pose{}, plasticbar::LANE, false);
  const Entity box = struckBox(
    source,
    glm::vec3(
      OFFSET,
      plasticbar::HALF_WIDTH + YIELDING_SKIN + STRUCK_BOX_HALF + GAP,
      0.0f
    )
  );

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> barId = yieldingBody(runtime, bar);
  const std::optional<JPH::BodyID> boxId = yieldingBody(runtime, box);

  REQUIRE(barId.has_value());
  REQUIRE(boxId.has_value());
  REQUIRE(runtime.rigidUntilYield(bar));

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  bodies.SetFriction(*barId, 0.0f);
  bodies.SetFriction(*boxId, 0.0f);
  bodies.SetLinearVelocity(*boxId, JPH::Vec3(0.0f, -speed, 0.0f));

  for (const JPH::BodyID held : {*barId, *boxId}) {
    const JPH::BodyLockWrite lock(
      runtime.physics().system().GetBodyLockInterface(),
      held
    );

    REQUIRE(lock.Succeeded());

    JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();

    motion->SetLinearDamping(0.0f);
    motion->SetAngularDamping(0.0f);
  }

  const auto system = [&]() {
    const YieldMotion held = barMotion(runtime, bar, *barId);
    const YieldMotion struck = joltMotion(runtime, *boxId);

    return YieldMotion{
      held.linear + struck.linear,
      held.angular + struck.angular,
      held.kinetic + struck.kinetic
    };
  };

  const YieldMotion start = system();

  StruckRun run;

  std::optional<YieldMotion> returned;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    const uint32_t demoted = run.worst.demotions;

    countTransitions(runtime, run.worst, "struckbar", frame);

    if (run.worst.demotions != demoted) {
      returned = system();
    }
  }

  const YieldMotion end = system();
  const YieldMotion scored = returned.value_or(end);

  run.linear = yieldingApart(start.linear, scored.linear);
  run.angular = yieldingApart(start.angular, scored.angular);
  run.energy = (scored.kinetic - start.kinetic) / start.kinetic;

  std::cout << "struckbar: end systemLinear "
            << yieldingApart(start.linear, end.linear) << " systemAngular "
            << yieldingApart(start.angular, end.angular) << " systemEnergy "
            << (end.kinetic - start.kinetic) / start.kinetic << '\n';

  std::cout << "struckbar: speed " << speed << " promotions "
            << run.worst.promotions << " demotions " << run.worst.demotions
            << " worstLinear " << run.worst.linear << " worstAngular "
            << run.worst.angular << " worstEnergy " << run.worst.energy
            << " systemLinear " << run.linear << " systemAngular "
            << run.angular << " systemEnergy " << run.energy
            << " promotedAtEnd " << runtime.promotedDeformable(bar)
            << std::endl;

  return run;
}

}

TEST_CASE(
  "a velocity edit on a promoted deformable moves its state's momentum by "
  "exactly the edit and leaves Jolt untouched",
  "[.][deformable]"
) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, 1.0f, 0.0f);

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());

  runtime.physics().bodies().SetLinearVelocity(
    *id,
    JPH::Vec3(0.2f, -0.1f, 0.3f)
  );
  runtime.physics().bodies().SetAngularVelocity(
    *id,
    JPH::Vec3(0.5f, 1.0f, 2.0f)
  );

  REQUIRE(runtime.forcePromotion(bar));

  runtime.step();
  runtime.syncPoses(1.0f);

  REQUIRE(runtime.promotedDeformable(bar));
  REQUIRE_FALSE(runtime.physics().bodies().IsAdded(*id));

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  double mass = 0.0;

  for (const float weight : mesh->nodeMass) {
    mass += static_cast<double>(weight);
  }

  const YieldMotion before = joltMotion(runtime, *id);
  const DeformableState held = *runtime.world().deformableState(bar);
  const YieldMotion stateBefore = stateMotion(*mesh, held);

  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != mesh->nodeMass.size(); ++node) {
    centre += static_cast<double>(mesh->nodeMass[node])
              * glm::dvec3(held.position[node]);
  }

  centre /= mass;

  const glm::vec3 edit(-0.4f, 0.7f, 0.1f);

  const talus::deformable::ImpactWindow* window = runtime.impactWindow(bar);

  REQUIRE(window != nullptr);

  const std::vector<glm::dvec3> windowBefore = window->velocity;
  const double injectedBefore = window->ledger.injected;

  glm::dvec3 windowMomentum(0.0);

  for (size_t node = 0u; node != windowBefore.size(); ++node) {
    windowMomentum +=
      static_cast<double>(mesh->nodeMass[node]) * windowBefore[node];
  }

  const glm::dvec3 shift = glm::dvec3(edit) - windowMomentum / mass;

  runtime.setLinearVelocity(bar, edit);

  double shiftApart = 0.0;
  double injected = 0.0;

  for (size_t node = 0u; node != windowBefore.size(); ++node) {
    const glm::dvec3& moved = window->velocity[node];

    shiftApart = std::max(
      shiftApart,
      glm::length(moved - windowBefore[node] - shift) / glm::length(shift)
    );
    injected +=
      0.5 * static_cast<double>(mesh->nodeMass[node])
      * (glm::dot(moved, moved) - glm::dot(windowBefore[node], windowBefore[node]));
  }

  const double bookedApart =
    std::abs(window->ledger.injected - injectedBefore - injected)
    / std::abs(injected);

  const YieldMotion stateAfter =
    stateMotion(*mesh, *runtime.world().deformableState(bar));
  const YieldMotion after = joltMotion(runtime, *id);

  const glm::dvec3 wanted = mass * glm::dvec3(edit);
  const double linearApart =
    glm::length(stateAfter.linear - wanted) / glm::length(wanted);
  const double spinApart = yieldingApart(
    stateBefore.angular - glm::cross(centre, stateBefore.linear),
    stateAfter.angular - glm::cross(centre, stateAfter.linear)
  );
  const bool joltKept = before.linear == after.linear
                        && before.angular == after.angular
                        && !runtime.physics().bodies().IsAdded(*id)
                        && !runtime.physics().bodies().IsActive(*id);

  runtime.setMass(bar, 5.0f);

  const YieldMotion massed = joltMotion(runtime, *id);

  talus::objectmodel::Pose moved = pose;
  moved.position += glm::vec3(1.0f, 0.0f, 0.0f);

  const DeformableState posedBefore = *runtime.world().deformableState(bar);

  runtime.world().replace<talus::objectmodel::Pose>(bar, moved);
  runtime.applyPoseEdits();

  const DeformableState* posedAfter = runtime.world().deformableState(bar);

  std::cout << "promotedvelocity: linearApart " << linearApart << " spinApart "
            << spinApart << " windowShiftApart " << shiftApart << " injected "
            << injected << " bookedApart " << bookedApart << " joltKept "
            << joltKept << " massKept " << (massed.linear == after.linear)
            << std::endl;

  REQUIRE(linearApart <= YIELDING_MOMENTUM);
  REQUIRE(spinApart <= YIELDING_MOMENTUM);
  REQUIRE(shiftApart <= YIELDING_MOMENTUM);
  REQUIRE(bookedApart <= YIELDING_MOMENTUM);
  REQUIRE(injected != 0.0);
  REQUIRE(joltKept);
  REQUIRE(massed.linear == after.linear);
  REQUIRE(massed.angular == after.angular);
  REQUIRE(posedAfter->position == posedBefore.position);
  REQUIRE(posedAfter->velocity == posedBefore.velocity);
  REQUIRE_FALSE(runtime.physics().bodies().IsAdded(*id));
}

TEST_CASE(
  "a yielding bar struck by a box and a spun bar forced through promotion "
  "keep momentum and add no energy",
  "[.][deformable]"
) {
  const float trigger =
    plasticbar::YIELD / std::sqrt(plasticbar::DENSITY * plasticbar::YOUNGS);

  const StruckRun slow = strikeYieldingBar(0.5f * trigger);
  const StruckRun fast = strikeYieldingBar(4.0f * trigger);

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.3f, 1.0f, -0.2f);
  pose.rotation =
    glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)));

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());

  runtime.physics().bodies().SetLinearVelocity(
    *id,
    JPH::Vec3(0.3f, -0.2f, 0.1f)
  );
  runtime.physics().bodies().SetAngularVelocity(
    *id,
    JPH::Vec3(0.5f, 1.0f, 3.0f)
  );

  runtime.step();
  runtime.syncPoses(1.0f);

  REQUIRE(runtime.forcePromotion(bar));

  TransitionWorst spun;

  for (uint32_t frame = 0u; frame != 8u; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    countTransitions(runtime, spun, "spunbar", frame);
  }

  std::cout << "spunbar: promotions " << spun.promotions << " demotions "
            << spun.demotions << " worstLinear " << spun.linear
            << " worstAngular " << spun.angular << " worstEnergy "
            << spun.energy << std::endl;

  REQUIRE(slow.worst.promotions == 0u);
  REQUIRE(slow.linear <= STRUCK_MOMENTUM);
  REQUIRE(slow.angular <= STRUCK_MOMENTUM);
  REQUIRE(slow.energy <= YIELDING_ENERGY);

  REQUIRE(fast.worst.promotions >= 1u);
  REQUIRE(fast.worst.demotions >= 1u);
  REQUIRE(fast.worst.linear <= STRUCK_MOMENTUM);
  REQUIRE(fast.worst.angular <= STRUCK_MOMENTUM);
  REQUIRE(fast.worst.energy <= YIELDING_ENERGY);
  REQUIRE(fast.linear <= STRUCK_MOMENTUM);
  REQUIRE(fast.energy <= YIELDING_ENERGY);

  REQUIRE(spun.promotions == 1u);
  REQUIRE(spun.demotions == 1u);
  REQUIRE(spun.linear <= STRUCK_MOMENTUM);
  REQUIRE(spun.angular <= STRUCK_MOMENTUM);
  REQUIRE(spun.energy <= YIELDING_ENERGY);
  REQUIRE_FALSE(runtime.promotedDeformable(bar));
  REQUIRE(runtime.physics().bodies().IsAdded(*id));
}

TEST_CASE(
  "a tet yield body and a damped yield body stay on the implicit path",
  "[deformable]"
) {
  constexpr uint32_t DAMPED_LANE = 2u;
  constexpr float DAMPING = 1.0e-3f;
  constexpr uint32_t FRAMES = 4u;

  JoltLibrary library;
  World source;

  std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(plasticbar::YIELD);
  PhysicalMaterial damped = materials[plasticbar::LANE];

  damped.stiffnessDamping = DAMPING;
  materials.push_back(damped);

  REQUIRE(materials.size() == DAMPED_LANE + 1u);

  source.setMaterials(materials);

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose tetPose{};
  tetPose.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + 0.2f, -0.2f);

  talus::objectmodel::Pose dampedPose{};
  dampedPose.position = glm::vec3(0.0f, YIELDING_REST_HEIGHT + 0.2f, 0.2f);

  const Entity tet = yieldingBar(source, tetPose, plasticbar::LANE, false);

  source.emplaceOrReplace<talus::objectmodel::DeformableBody>(
    tet,
    talus::objectmodel::DeformableBody{
      talus::objectmodel::VolumeElementType::Tet
    }
  );

  const Entity dampedBar = yieldingBar(source, dampedPose, DAMPED_LANE, false);

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const PreparedMesh* tetMesh = runtime.world().deformablePrepared(tet);
  const PreparedMesh* dampedMesh =
    runtime.world().deformablePrepared(dampedBar);

  REQUIRE(tetMesh != nullptr);
  REQUIRE(dampedMesh != nullptr);
  REQUIRE(!tetMesh->tets.empty());
  REQUIRE(tetMesh->hexes.empty());
  REQUIRE(!dampedMesh->hexes.empty());
  REQUIRE(talus::deformable::stiffnessDamping(materials[DAMPED_LANE]) > 0.0f);

  const uint32_t nodes = static_cast<uint32_t>(
    tetMesh->restPositions.size() + dampedMesh->restPositions.size()
  );

  uint32_t everyFrame = 0u;
  uint32_t promotions = 0u;
  uint32_t windows = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    everyFrame += runtime.deformableReport().nodes == nodes ? 1u : 0u;
    promotions += runtime.deformableActivity().promotions;
    windows += runtime.deformableActivity().windows;
  }

  std::cout << "implicitrouting: steppedFrames " << everyFrame << " of "
            << FRAMES << " promotions " << promotions << " windows " << windows
            << " tetBody " << yieldingBody(runtime, tet).has_value()
            << " dampedBody " << yieldingBody(runtime, dampedBar).has_value()
            << std::endl;

  REQUIRE_FALSE(runtime.rigidUntilYield(tet));
  REQUIRE_FALSE(runtime.rigidUntilYield(dampedBar));
  REQUIRE_FALSE(yieldingBody(runtime, tet).has_value());
  REQUIRE_FALSE(yieldingBody(runtime, dampedBar).has_value());
  REQUIRE(everyFrame == FRAMES);
  REQUIRE(promotions == 0u);
  REQUIRE(windows == 0u);
}

namespace {

constexpr double WINDOW_EPSILON = 0.035;
constexpr double TRANSITION_FLOOR = 1.0e-9;
constexpr double STORED_HULL_GAP = 1.0e-4;

double hullHeight(
  const JPH::ConvexHullShape& hull,
  const glm::dvec3& fieldPoint
) {
  const JPH::Vec3 centre = hull.GetCenterOfMass();
  const glm::dvec3 local =
    fieldPoint - glm::dvec3(centre.GetX(), centre.GetY(), centre.GetZ());

  double highest = -std::numeric_limits<double>::infinity();

  for (const JPH::Plane& plane : hull.GetPlanes()) {
    const JPH::Vec3 normal = plane.GetNormal();

    highest = std::max(
      highest,
      glm::dot(glm::dvec3(normal.GetX(), normal.GetY(), normal.GetZ()), local)
        + static_cast<double>(plane.GetConstant())
    );
  }

  return highest;
}

const JPH::ConvexHullShape* bodyHull(Runtime& runtime, JPH::BodyID id) {
  const JPH::ShapeRefC shape = runtime.physics().bodies().GetShape(id);

  REQUIRE(shape != nullptr);
  REQUIRE(shape->GetSubType() == JPH::EShapeSubType::OffsetCenterOfMass);

  const auto* field = dynamic_cast<const talus::physics::FieldShape*>(
    static_cast<const JPH::OffsetCenterOfMassShape&>(*shape).GetInnerShape()
  );

  REQUIRE(field != nullptr);

  return field->hull();
}

double storedHullGap(Runtime& runtime, Entity entity, JPH::BodyID id) {
  const PreparedMesh* mesh = runtime.world().deformablePrepared(entity);

  REQUIRE(mesh != nullptr);

  const JPH::ConvexHullShape* hull = bodyHull(runtime, id);

  REQUIRE(hull != nullptr);

  const std::span<const glm::vec3> stored = runtime.storedShape(entity);

  REQUIRE(stored.size() == mesh->restPositions.size());

  double widest = -std::numeric_limits<double>::infinity();

  for (const uint32_t node : mesh->surfaceNodes) {
    widest = std::max(widest, hullHeight(*hull, glm::dvec3(stored[node])));
  }

  return widest;
}

constexpr float BENT_DROP = 3.0f;
constexpr float BENT_TILT = 0.35f;
constexpr float BENT_EXTENT = 2.5f;
constexpr float BENT_SKIN = 0.05f * plasticbar::COARSE_VOXEL;
constexpr uint32_t BENT_FRAMES = 360u;
constexpr uint32_t BENT_SETTLE = 120u;
constexpr float BENT_INSIDE = 0.75e-3f;
constexpr float BENT_LOWERING = BENT_SKIN + BENT_INSIDE;

Entity bentBarScene(World& source) {
  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(wideYieldingSlab(source, BENT_EXTENT));

  talus::objectmodel::Pose pose{};
  pose.position =
    glm::vec3(0.0f, plasticbar::HALF_WIDTH + BENT_SKIN + BENT_DROP, 0.0f);
  pose.rotation =
    glm::angleAxis(BENT_TILT, glm::normalize(glm::vec3(1.0f, 0.0f, 0.5f)));

  return yieldingBox(
    source,
    pose,
    plasticbar::LANE,
    glm::vec3(
      plasticbar::HALF_LENGTH,
      plasticbar::HALF_WIDTH,
      plasticbar::HALF_WIDTH
    ),
    false,
    BENT_SKIN
  );
}

struct BentSettled {
  uint32_t ran = 0u;
  uint32_t demotions = 0u;
  float permanent = 0.0f;
};

constexpr int64_t HANDBACK_ULPS = 4;
constexpr double HANDBACK_VELOCITY = 1.0e-6;
constexpr double HANDBACK_MOMENTUM = 60.0 * 0x1.0p-24;
constexpr double HANDBACK_TURN = 1.0e-6;
constexpr uint32_t HANDBACK_FRAMES = 60u;
constexpr float HANDBACK_GAP = 0.01f;
constexpr float HANDBACK_STRIKE = 1.0f;
constexpr float HANDBACK_ALONG = 0.1f;
constexpr float HANDBACK_SPIN_LIMIT = 1000.0f;

int64_t orderedBits(float value) {
  const int32_t bits = std::bit_cast<int32_t>(value);

  return bits < 0 ? static_cast<int64_t>(INT32_MIN) - bits : bits;
}

int64_t ulpsApart(float value, double target) {
  return std::abs(orderedBits(value) - orderedBits(static_cast<float>(target)));
}

int64_t poseUlps(
  JPH::RVec3Arg centre,
  JPH::QuatArg rotation,
  const talus::runtime::PartnerHandback& handback
) {
  const glm::dquat
    held(rotation.GetW(), rotation.GetX(), rotation.GetY(), rotation.GetZ());
  const glm::dquat wanted = glm::dot(held, handback.rotation) < 0.0
                              ? -handback.rotation
                              : handback.rotation;

  return std::max(
    {ulpsApart(static_cast<float>(centre.GetX()), handback.centre.x),
     ulpsApart(static_cast<float>(centre.GetY()), handback.centre.y),
     ulpsApart(static_cast<float>(centre.GetZ()), handback.centre.z),
     ulpsApart(rotation.GetX(), wanted.x),
     ulpsApart(rotation.GetY(), wanted.y),
     ulpsApart(rotation.GetZ(), wanted.z),
     ulpsApart(rotation.GetW(), wanted.w)}
  );
}

uint32_t recordLines(
  const talus::runtime::DeformableActivity& activity,
  uint32_t frame,
  const windowmonitor::MonitoredBody& body,
  double restingMass
) {
  uint32_t lines = 0u;

  const auto print = [&lines](const std::vector<std::string>& printed) {
    for (const std::string& line : printed) {
      std::cout << line << '\n';
      ++lines;
    }
  };

  print(
    windowmonitor::frameViolations(
      activity.windowFrames,
      frame,
      WINDOW_EPSILON,
      [&body](Entity) { return body; },
      restingMass
    )
  );

  for (const talus::runtime::WindowRecord& record : activity.windowFrames) {
    for (const talus::runtime::PartnerHandback& handback : record.handbacks) {
      print(windowmonitor::handbackViolations(handback, frame, WINDOW_EPSILON));
    }
  }

  for (const talus::runtime::YieldTransition& transition :
       activity.transitions) {
    print(windowmonitor::transitionViolations(transition, frame, body));
  }

  return lines;
}

struct StruckWindowRun {
  uint32_t windows = 0u;
  uint32_t touched = 0u;
  uint32_t handbacks = 0u;
  uint32_t turned = 0u;
  int64_t worstUlps = 0;
  double linear = 0.0;
  double angular = 0.0;
  double linearSize = 0.0;
  double angularSize = 0.0;
  float peakSpin = 0.0f;
};

struct SystemMomentum {
  glm::dvec3 linear{0.0};
  glm::dvec3 angular{0.0};
};

SystemMomentum systemMomentum(
  const PreparedMesh& mesh,
  std::span<const glm::dvec3> position,
  std::span<const glm::dvec3> velocity,
  Runtime& runtime,
  JPH::BodyID box
) {
  const JPH::BodyLockRead lock(
    runtime.physics().system().GetBodyLockInterface(),
    box
  );

  REQUIRE(lock.Succeeded());

  const JPH::Body& body = lock.GetBody();
  const double boxMass =
    1.0 / static_cast<double>(body.GetMotionProperties()->GetInverseMass());
  const JPH::RVec3 at = body.GetCenterOfMassPosition();
  const JPH::Vec3 moving = body.GetLinearVelocity();
  const JPH::Vec3 turning = body.GetAngularVelocity();
  const JPH::Mat44 inverse = body.GetInverseInertia();

  glm::dmat3 inertia(0.0);

  for (int column = 0; column != 3; ++column) {
    const JPH::Vec3 entry = inverse.GetColumn3(column);

    inertia[column] = glm::dvec3(entry.GetX(), entry.GetY(), entry.GetZ());
  }

  inertia = glm::inverse(inertia);

  const glm::dvec3 boxCentre(at.GetX(), at.GetY(), at.GetZ());
  const glm::dvec3 boxVelocity(moving.GetX(), moving.GetY(), moving.GetZ());
  const glm::dvec3 boxSpin(turning.GetX(), turning.GetY(), turning.GetZ());

  double mass = boxMass;
  glm::dvec3 centre = boxMass * boxCentre;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    mass += static_cast<double>(mesh.nodeMass[node]);
    centre += static_cast<double>(mesh.nodeMass[node]) * position[node];
  }

  centre /= mass;

  SystemMomentum total;

  total.linear = boxMass * boxVelocity;
  total.angular =
    boxMass * glm::cross(boxCentre - centre, boxVelocity) + inertia * boxSpin;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const double weight = static_cast<double>(mesh.nodeMass[node]);

    total.linear += weight * velocity[node];
    total.angular +=
      weight * glm::cross(position[node] - centre, velocity[node]);
  }

  return total;
}

StruckWindowRun struckWindow(bool gravity) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, 1.0f, 0.0f);

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);
  const Entity box = struckBox(
    source,
    glm::vec3(
      HANDBACK_ALONG,
      1.0f + plasticbar::HALF_WIDTH + YIELDING_SKIN + STRUCK_BOX_HALF
        + HANDBACK_GAP,
      0.0f
    )
  );

  Runtime runtime(source, library);

  if (!gravity) {
    runtime.physics().system().SetGravity(JPH::Vec3::sZero());
  }

  talus::deformable::Settings settings = runtime.deformableSettings();

  settings.demoteAfterYield = false;

  runtime.setDeformableSettings(settings);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> barId = yieldingBody(runtime, bar);
  const std::optional<JPH::BodyID> boxId = yieldingBody(runtime, box);

  REQUIRE(barId.has_value());
  REQUIRE(boxId.has_value());

  if (!gravity) {
    JPH::BodyLockWrite lock(
      runtime.physics().system().GetBodyLockInterface(),
      *boxId
    );

    REQUIRE(lock.Succeeded());

    JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();

    motion->SetLinearDamping(0.0f);
    motion->SetAngularDamping(0.0f);
    motion->SetMaxAngularVelocity(HANDBACK_SPIN_LIMIT);
  }

  runtime.physics().bodies().SetLinearVelocity(
    *boxId,
    JPH::Vec3(0.0f, -HANDBACK_STRIKE, 0.0f)
  );

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  std::vector<glm::dvec3> restPosition(mesh->restPositions.size());
  std::vector<glm::dvec3> restVelocity(
    mesh->restPositions.size(),
    glm::dvec3(0.0)
  );

  {
    const glm::mat4 model = talus::objectmodel::model(pose);

    for (size_t node = 0u; node != restPosition.size(); ++node) {
      restPosition[node] = glm::dvec3(
        glm::vec3(model * glm::vec4(mesh->restPositions[node], 1.0f))
      );
    }
  }

  const SystemMomentum start =
    systemMomentum(*mesh, restPosition, restVelocity, runtime, *boxId);

  REQUIRE(runtime.forcePromotion(bar));

  StruckWindowRun run;

  run.linearSize = glm::length(start.linear);
  run.angularSize = glm::length(start.angular);

  for (uint32_t frame = 0u; frame != HANDBACK_FRAMES; ++frame) {
    const JPH::Quat before = runtime.physics().bodies().GetRotation(*boxId);

    runtime.step();

    const talus::deformable::ImpactWindow* window = runtime.impactWindow(bar);

    if (window == nullptr) {
      continue;
    }

    ++run.windows;

    std::vector<glm::dvec3> position(window->position.size());

    for (size_t node = 0u; node != position.size(); ++node) {
      position[node] = window->origin + window->position[node];
    }

    const SystemMomentum now =
      systemMomentum(*mesh, position, window->velocity, runtime, *boxId);

    run.linear = std::max(
      run.linear,
      glm::length(now.linear - start.linear) / run.linearSize
    );
    run.angular = std::max(
      run.angular,
      glm::length(now.angular - start.angular) / run.angularSize
    );
    run.peakSpin = std::max(
      run.peakSpin,
      runtime.physics().bodies().GetAngularVelocity(*boxId).Length()
    );

    for (const talus::runtime::WindowRecord& record :
         runtime.deformableActivity().windowFrames) {
      run.touched += record.step.dynamicPartners;

      for (const talus::runtime::PartnerHandback& handback : record.handbacks) {
        if (handback.body != boxId->GetIndexAndSequenceNumber()) {
          continue;
        }

        ++run.handbacks;

        const JPH::Quat after = runtime.physics().bodies().GetRotation(*boxId);
        const double turn =
          2.0
          * std::acos(
            std::min(1.0, std::abs(static_cast<double>(after.Dot(before))))
          );

        if (turn <= HANDBACK_TURN) {
          continue;
        }

        ++run.turned;
        run.worstUlps = std::max(
          run.worstUlps,
          poseUlps(
            runtime.physics().bodies().GetCenterOfMassPosition(*boxId),
            after,
            handback
          )
        );
      }
    }
  }

  return run;
}

BentSettled settleBentBar(Runtime& runtime, Entity bar, JPH::BodyID id) {
  BentSettled settled;
  uint32_t demotedAt = 0u;

  for (uint32_t frame = 1u; frame <= BENT_FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    settled.ran = frame;
    settled.demotions += runtime.deformableActivity().demotions;

    for (const talus::runtime::YieldTransition& transition :
         runtime.deformableActivity().transitions) {
      settled.permanent = std::max(settled.permanent, transition.permanent);
    }

    if (runtime.deformableActivity().demotions != 0u) {
      demotedAt = frame;
    }

    if (
      settled.demotions != 0u && !runtime.promotedDeformable(bar)
      && (!runtime.physics().bodies().IsActive(id) || frame >= demotedAt + BENT_SETTLE)
    ) {
      break;
    }
  }

  return settled;
}
}

TEST_CASE(
  "a field box resting on a permanent impact window bar stays on it",
  "[.][deformable]"
) {
  constexpr uint32_t FRAMES = 60u;
  constexpr float BOX_SPEED = 0.1f;
  constexpr float BOX_REACH = 0.01f;

  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, plasticbar::HALF_WIDTH, 0.0f);

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);
  const float boxRest = 2.0f * plasticbar::HALF_WIDTH + STRUCK_BOX_HALF;
  const Entity box = struckBox(source, glm::vec3(0.0f, boxRest, 0.0f));

  Runtime runtime(source, library);

  talus::deformable::Settings settings = runtime.deformableSettings();

  settings.demoteAfterYield = false;

  runtime.setDeformableSettings(settings);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> boxId = yieldingBody(runtime, box);

  REQUIRE(boxId.has_value());
  REQUIRE(runtime.forcePromotion(bar));

  const double boxMass = static_cast<double>(
    runtime.physics().bodies().GetShape(*boxId)->GetMassProperties().mMass
  );

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*mesh);
  const double restingMass = monitored.mass + boxMass;

  float fastest = 0.0f;
  double worstShare = 0.0;
  uint32_t touched = 0u;
  uint32_t records = 0u;
  uint32_t demotions = 0u;
  uint32_t lines = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    fastest = std::max(
      fastest,
      runtime.physics().bodies().GetLinearVelocity(*boxId).Length()
    );
    demotions += runtime.deformableActivity().demotions;

    for (const std::string& line : windowmonitor::frameViolations(
           runtime.deformableActivity().windowFrames,
           frame,
           WINDOW_EPSILON,
           [&monitored](Entity) { return monitored; },
           restingMass
         )) {
      std::cout << line << '\n';
      ++lines;
    }

    for (const talus::runtime::WindowRecord& record :
         runtime.deformableActivity().windowFrames) {
      if (record.entity != bar) {
        continue;
      }

      ++records;
      touched += record.step.dynamicPartners;
      worstShare = std::max(
        worstShare,
        record.step.stepCreation / std::max(record.step.input, 1.0e-300)
      );
    }
  }

  const JPH::RVec3 reached =
    runtime.physics().bodies().GetCenterOfMassPosition(*boxId);

  std::cout << "boxonwindow: boxMass " << boxMass << " fastest " << fastest
            << " boxHeight " << reached.GetY() << " of " << boxRest
            << " boxDrift " << reached.GetX() << ' ' << reached.GetZ()
            << " records " << records << " touchedFrames " << touched
            << " worstCreationShare " << worstShare << " restingMass "
            << restingMass << " lines " << lines << " demotions " << demotions
            << " promoted " << runtime.promotedDeformable(bar) << std::endl;

  REQUIRE(std::abs(boxMass - 1.0) < 1.0e-3);
  REQUIRE(runtime.promotedDeformable(bar));
  REQUIRE(demotions == 0u);
  REQUIRE(records == FRAMES);
  REQUIRE(touched > 0u);
  REQUIRE(fastest < BOX_SPEED);
  REQUIRE(std::abs(reached.GetY() - boxRest) < BOX_REACH);
  REQUIRE(std::abs(reached.GetX()) < BOX_REACH);
  REQUIRE(std::abs(reached.GetZ()) < BOX_REACH);
  REQUIRE(lines == 0u);
}

TEST_CASE(
  "a field box resting on a permanent impact window bar is handed back at "
  "the window's end velocity with no violation line",
  "[.][deformable]"
) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  static_cast<void>(yieldingSlab(source));

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(0.0f, plasticbar::HALF_WIDTH, 0.0f);

  const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);
  const Entity box = struckBox(
    source,
    glm::vec3(0.0f, 2.0f * plasticbar::HALF_WIDTH + STRUCK_BOX_HALF, 0.0f)
  );

  Runtime runtime(source, library);

  talus::deformable::Settings settings = runtime.deformableSettings();

  settings.demoteAfterYield = false;

  runtime.setDeformableSettings(settings);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> boxId = yieldingBody(runtime, box);

  REQUIRE(boxId.has_value());
  REQUIRE(runtime.forcePromotion(bar));

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*mesh);
  const double restingMass =
    monitored.mass
    + static_cast<double>(
      runtime.physics().bodies().GetShape(*boxId)->GetMassProperties().mMass
    );

  uint32_t handbacks = 0u;
  uint32_t changed = 0u;
  uint32_t lines = 0u;
  double worstVelocity = 0.0;

  for (uint32_t frame = 0u; frame != HANDBACK_FRAMES; ++frame) {
    runtime.step();
    runtime.syncPoses(1.0f);

    lines +=
      recordLines(runtime.deformableActivity(), frame, monitored, restingMass);

    for (const talus::runtime::WindowRecord& record :
         runtime.deformableActivity().windowFrames) {
      for (const talus::runtime::PartnerHandback& handback : record.handbacks) {
        if (handback.body != boxId->GetIndexAndSequenceNumber()) {
          continue;
        }

        ++handbacks;
        changed += handback.contactLinear != glm::dvec3(0.0)
                       || handback.contactAngular != glm::dvec3(0.0)
                     ? 1u
                     : 0u;

        const JPH::Vec3 linear =
          runtime.physics().bodies().GetLinearVelocity(*boxId);
        const JPH::Vec3 angular =
          runtime.physics().bodies().GetAngularVelocity(*boxId);

        worstVelocity = std::max(
          {worstVelocity,
           glm::length(
             glm::dvec3(linear.GetX(), linear.GetY(), linear.GetZ())
             - handback.linear
           ),
           glm::length(
             glm::dvec3(angular.GetX(), angular.GetY(), angular.GetZ())
             - handback.angular
           )}
        );
      }
    }
  }

  std::cout << "handbackbox: handbacks " << handbacks << " contactChanged "
            << changed << " worstVelocity " << worstVelocity << " lines "
            << lines << std::endl;

  REQUIRE(handbacks > 0u);
  CHECK(changed == 0u);
  CHECK(worstVelocity <= HANDBACK_VELOCITY);
  CHECK(lines == 0u);
}

TEST_CASE(
  "a struck field box handed back by a permanent impact window lands on the "
  "window's end pose",
  "[.][deformable]"
) {
  const StruckWindowRun run = struckWindow(true);

  std::cout << "handbackpose: windows " << run.windows << " touchedFrames "
            << run.touched << " handbacks " << run.handbacks << " turnedFrames "
            << run.turned << " worstPoseUlps " << run.worstUlps << std::endl;

  REQUIRE(run.windows == HANDBACK_FRAMES);
  REQUIRE(run.touched > 0u);
  REQUIRE(run.turned > 0u);
  CHECK(run.worstUlps <= HANDBACK_ULPS);
}

TEST_CASE(
  "a permanent impact window bar struck by a free field box keeps the "
  "system's momentum",
  "[.][deformable]"
) {
  const StruckWindowRun run = struckWindow(false);
  const double bound = std::max(1.0e-6, HANDBACK_MOMENTUM);

  std::cout << "handbackmomentum: windows " << run.windows << " touchedFrames "
            << run.touched << " handbacks " << run.handbacks << " |P| "
            << run.linearSize << " |L| " << run.angularSize << " worstLinear "
            << run.linear << " worstAngular " << run.angular << " peakSpin "
            << run.peakSpin << " bound " << bound << std::endl;

  REQUIRE(run.windows == HANDBACK_FRAMES);
  REQUIRE(run.touched > 0u);
  REQUIRE(run.peakSpin < HANDBACK_SPIN_LIMIT);
  CHECK(run.linear <= bound);
  CHECK(run.angular <= bound);
}

TEST_CASE(
  "a demoted bent bar resting on the slab promotes without stored penalty "
  "energy",
  "[.][deformable]"
) {
  JoltLibrary library;
  World source;

  const Entity bar = bentBarScene(source);

  Runtime runtime(source, library);

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

  REQUIRE(id.has_value());

  const BentSettled settled = settleBentBar(runtime, bar, *id);
  const uint32_t demotions = settled.demotions;
  const uint32_t ran = settled.ran;
  const float permanent = settled.permanent;

  const DeformableState* rested = runtime.world().deformableState(bar);

  REQUIRE(rested != nullptr);

  const float bent = yieldingPlastic(*rested);
  const float height =
    runtime.physics().bodies().GetCenterOfMassPosition(*id).GetY();
  std::cout << "bentrepromote: restHeight " << height << std::endl;

  REQUIRE(height > 0.0f);
  REQUIRE(demotions >= 1u);
  REQUIRE_FALSE(runtime.promotedDeformable(bar));
  REQUIRE(bent > 0.0f);

  {
    const JPH::RVec3 settledAt = runtime.physics().bodies().GetPosition(*id);

    runtime.physics().bodies().SetPosition(
      *id,
      settledAt - JPH::RVec3(0.0f, BENT_LOWERING, 0.0f),
      JPH::EActivation::DontActivate
    );
  }

  REQUIRE(runtime.forcePromotion(bar));

  runtime.step();
  runtime.syncPoses(1.0f);

  const PreparedMesh* mesh = runtime.world().deformablePrepared(bar);

  REQUIRE(mesh != nullptr);

  const windowmonitor::MonitoredBody monitored =
    windowmonitor::monitoredBody(*mesh);

  uint32_t promotions = 0u;
  double linear = 0.0;
  double angular = 0.0;
  double added = 0.0;
  double penalty = -1.0;
  double kinetic = 0.0;

  for (const talus::runtime::YieldTransition& transition :
       runtime.deformableActivity().transitions) {
    if (!transition.promotion || transition.entity != bar) {
      continue;
    }

    ++promotions;

    const TransitionShare share = transitionShare(transition, monitored);

    linear = share.linear;
    angular = share.angular;
    added = transition.kineticAfter + transition.penaltyAfter
            - transition.kineticBefore;
    penalty = transition.penaltyAfter;
    kinetic = transition.kineticBefore;
  }

  const talus::deformable::ImpactWindow* window = runtime.impactWindow(bar);

  REQUIRE(window != nullptr);

  std::cout << "bentrepromote: ran " << ran << " demotions " << demotions
            << " plastic " << bent << " permanent " << permanent << " skin "
            << BENT_SKIN << " promotions " << promotions << " openingInside "
            << window->openingInside << " penaltyAfter " << penalty
            << " linear " << linear << " angular " << angular
            << " kineticBefore " << kinetic << " added " << added << std::endl;

  REQUIRE(promotions == 1u);
  REQUIRE(window->openingInside > 0u);
  REQUIRE(penalty == 0.0);
  REQUIRE(linear <= windowmonitor::transitionBound(monitored));
  REQUIRE(angular <= windowmonitor::transitionBound(monitored));
  REQUIRE(added <= YIELDING_ENERGY * std::max(kinetic, TRANSITION_FLOOR));
}

namespace {

constexpr float DISPLACED_VOXEL = 0.0125f;
constexpr float DISPLACED_ARCH = 9.0e-3f;
constexpr float DISPLACED_DOME = 4.0e-3f;

struct DisplacedRun {
  double gap = 0.0;
  double reached = 0.0;
  double authoredReached = 0.0;
};

DisplacedRun displacedRun(
  const talus::field::SampledSdf& field,
  const PreparedMesh& mesh,
  const JPH::ConvexHullShape& authored,
  const glm::vec3& half,
  float rise,
  bool dome
) {
  std::vector<glm::vec3> shape(mesh.restPositions.size());

  for (size_t node = 0u; node != shape.size(); ++node) {
    const glm::vec3& rest = mesh.restPositions[node];
    const float along = rest.x / half.x;
    const float across = rest.z / half.z;
    const float lift =
      dome ? rise * (1.0f - along * along) * (1.0f - across * across)
           : rise * (1.0f - along * along);

    shape[node] = rest + glm::vec3(0.0f, lift, 0.0f);
  }

  const JPH::RefConst<JPH::ConvexHullShape> hull =
    talus::physics::displacedHull(field, mesh, shape);

  REQUIRE(hull != nullptr);

  DisplacedRun run;

  run.gap = -std::numeric_limits<double>::infinity();

  for (const uint32_t node : mesh.surfaceNodes) {
    run.gap = std::max(run.gap, hullHeight(*hull, glm::dvec3(shape[node])));
  }

  const glm::dvec3 crest(0.0, half.y + YIELDING_SKIN + rise, 0.0);

  run.reached = hullHeight(*hull, crest);
  run.authoredReached = hullHeight(authored, crest);

  return run;
}

}

TEST_CASE(
  "a displaced hull holds an arched and a domed bar's moved surface",
  "[deformable]"
) {
  JoltLibrary library;

  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );
  const glm::vec3 low = -half - glm::vec3(DISPLACED_VOXEL);
  const glm::vec3 high = half + glm::vec3(DISPLACED_VOXEL);
  const glm::uvec3 resolution =
    glm::uvec3(glm::round((high - low) / DISPLACED_VOXEL)) + glm::uvec3(1u);

  const talus::field::SampledSdf field = talus::field::bake(
    std::vector<talus::field::Primitive>{spanning(
      -half - glm::vec3(YIELDING_SKIN),
      half + glm::vec3(YIELDING_SKIN),
      plasticbar::LANE
    )},
    low,
    high,
    resolution
  );

  const std::vector<PhysicalMaterial> materials =
    plasticbar::barMaterials(plasticbar::YIELD);
  const std::shared_ptr<const PreparedMesh> mesh = talus::deformable::buildMesh(
    field,
    materials,
    glm::mat4(1.0f),
    {},
    talus::objectmodel::VolumeElementType::Hex
  );

  REQUIRE(mesh != nullptr);
  REQUIRE(!mesh->hexes.empty());

  const talus::physics::FieldHull authored =
    talus::physics::convexHullOf(field);

  REQUIRE(authored.shape != nullptr);

  const DisplacedRun arch =
    displacedRun(field, *mesh, *authored.shape, half, DISPLACED_ARCH, false);
  const DisplacedRun dome =
    displacedRun(field, *mesh, *authored.shape, half, DISPLACED_DOME, true);

  std::cout << "displacedhull: archGap " << arch.gap << " archReached "
            << arch.reached << " archAuthored " << arch.authoredReached
            << " domeGap " << dome.gap << " domeReached " << dome.reached
            << " domeAuthored " << dome.authoredReached << " tolerance "
            << STORED_HULL_GAP << std::endl;

  CHECK(arch.gap <= STORED_HULL_GAP);
  CHECK(dome.gap <= STORED_HULL_GAP);
  CHECK(arch.reached <= STORED_HULL_GAP);
  CHECK(dome.reached <= STORED_HULL_GAP);
  CHECK(arch.authoredReached > STORED_HULL_GAP);
  CHECK(dome.authoredReached > STORED_HULL_GAP);
}

TEST_CASE(
  "a demoted bent bar's hull holds its stored surface and a bar demoted "
  "without plastic strain keeps its authored hull",
  "[.][deformable]"
) {
  constexpr uint32_t FREE_FRAMES = 30u;

  double gap = 0.0;
  float bent = 0.0f;
  uint32_t bentPoints = 0u;
  uint32_t demotions = 0u;

  {
    JoltLibrary library;
    World source;

    const Entity bar = bentBarScene(source);

    Runtime runtime(source, library);

    yieldingPrepare(runtime);

    const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

    REQUIRE(id.has_value());

    demotions = settleBentBar(runtime, bar, *id).demotions;

    const DeformableState* rested = runtime.world().deformableState(bar);

    REQUIRE(rested != nullptr);
    REQUIRE_FALSE(runtime.promotedDeformable(bar));

    bent = yieldingPlastic(*rested);
    gap = storedHullGap(runtime, bar, *id);
    bentPoints = bodyHull(runtime, *id)->GetNumPoints();
  }

  bool same = false;
  uint32_t freePoints = 0u;
  uint32_t authoredPoints = 0u;
  uint32_t freeDemotions = 0u;
  float freePlastic = -1.0f;

  {
    JoltLibrary library;
    World source;

    source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

    talus::objectmodel::Pose pose{};
    pose.position = glm::vec3(0.3f, 1.0f, -0.2f);
    pose.rotation =
      glm::angleAxis(0.4f, glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f)));

    const Entity bar = yieldingBar(source, pose, plasticbar::LANE, false);

    Runtime runtime(source, library);

    runtime.physics().system().SetGravity(JPH::Vec3::sZero());

    yieldingPrepare(runtime);

    const std::optional<JPH::BodyID> id = yieldingBody(runtime, bar);

    REQUIRE(id.has_value());

    runtime.physics().bodies().SetAngularVelocity(
      *id,
      JPH::Vec3(0.5f, 1.0f, 2.0f)
    );

    REQUIRE(runtime.forcePromotion(bar));

    for (uint32_t frame = 0u; frame != FREE_FRAMES && freeDemotions == 0u;
         ++frame) {
      runtime.step();
      runtime.syncPoses(1.0f);

      freeDemotions += runtime.deformableActivity().demotions;
    }

    const DeformableState* rested = runtime.world().deformableState(bar);
    const auto& held =
      runtime.world().registry().get<talus::objectmodel::SharedSdf>(bar);

    REQUIRE(rested != nullptr);
    REQUIRE(held.field != nullptr);

    freePlastic = yieldingPlastic(*rested);

    const JPH::ConvexHullShape* hull = bodyHull(runtime, *id);
    const talus::physics::FieldHull authored =
      talus::physics::convexHullOf(*held.field);

    REQUIRE(hull != nullptr);
    REQUIRE(authored.shape != nullptr);

    freePoints = hull->GetNumPoints();
    authoredPoints = authored.shape->GetNumPoints();
    same = freePoints == authoredPoints
           && hull->GetCenterOfMass() == authored.shape->GetCenterOfMass();

    for (JPH::uint point = 0u; same && point != freePoints; ++point) {
      same = hull->GetPoint(point) == authored.shape->GetPoint(point);
    }
  }

  std::cout << "windowhull: bent demotions " << demotions << " plastic " << bent
            << " hullPoints " << bentPoints << " signedDistance " << gap
            << " free demotions " << freeDemotions << " plastic " << freePlastic
            << " hullPoints " << freePoints << " of " << authoredPoints
            << " same " << (same ? 1 : 0) << std::endl;

  REQUIRE(demotions >= 1u);
  REQUIRE(bent > 0.0f);
  CHECK(gap <= STORED_HULL_GAP);
  REQUIRE(freeDemotions == 1u);
  REQUIRE(freePlastic == 0.0f);
  CHECK(same);
}

namespace {

constexpr uint32_t FLIGHT_STEPS = 600u;
constexpr uint32_t FLIGHT_FINE_PARTS = 4000u;
constexpr uint32_t FLIGHT_COARSE_PARTS = 2000u;
constexpr double FLIGHT_FALL = 5.0e-3;
constexpr double FLIGHT_LINEAR = 1.0e-3;
constexpr double FLIGHT_ANGULAR = 1.0e-3;
constexpr double FLIGHT_ENERGY = 1.0e-2;
constexpr double FLIGHT_TURN = 5.0e-3;
constexpr double FLIGHT_REFERENCE = 1.0e-2;
constexpr float FLIGHT_DRIFT_SPEED = 0.01f;
constexpr float FLIGHT_DRIFT_SPIN = 0.02f;
constexpr uint32_t FLIGHT_DRIFT_STEPS = 120u;

struct FlightReference {
  std::vector<glm::dmat3> turn;
  double total = 0.0;
};

struct Flight {
  glm::vec3 half{0.0f};
  glm::vec3 spin{0.0f};
  glm::vec3 velocity{0.0f};
  bool falls = false;
  bool symmetric = false;
};

struct FlightRun {
  double fall = 0.0;
  double linear = 0.0;
  double angular = 0.0;
  double energy = 0.0;
  double turn = 0.0;
  double total = 0.0;
  double referenceGap = 0.0;
  uint32_t promotions = 0u;
  bool kept = true;
};

double flightStep() {
  return static_cast<double>(PhysicsWorld::TIME_STEP);
}

glm::dmat3 flightRotation(const glm::dvec3& axisAngle) {
  const double angle = glm::length(axisAngle);

  if (angle == 0.0) {
    return glm::dmat3(1.0);
  }

  return glm::mat3_cast(glm::angleAxis(angle, axisAngle / angle));
}

double flightAngle(const glm::dmat3& from, const glm::dmat3& to) {
  const glm::dmat3 apart = glm::transpose(from) * to;

  return std::acos(
    std::clamp(0.5 * (apart[0][0] + apart[1][1] + apart[2][2] - 1.0), -1.0, 1.0)
  );
}

FlightReference topFlight(const glm::dmat3& inertia, const glm::dvec3& spin) {
  const double axial = inertia[0][0];
  const double across = 0.5 * (inertia[1][1] + inertia[2][2]);
  const glm::dvec3 momentum = inertia * spin;
  const double precession = (across - axial) / across * spin.x;

  FlightReference made;

  made.turn.reserve(FLIGHT_STEPS + 1u);

  for (uint32_t tick = 0u; tick <= FLIGHT_STEPS; ++tick) {
    const double elapsed = flightStep() * static_cast<double>(tick);

    made.turn.push_back(
      flightRotation(momentum / across * elapsed)
      * flightRotation(glm::dvec3(precession * elapsed, 0.0, 0.0))
    );
  }

  made.total =
    glm::length(spin) * flightStep() * static_cast<double>(FLIGHT_STEPS);

  return made;
}

FlightReference eulerFlight(
  const glm::dmat3& inertia,
  const glm::dvec3& spin,
  uint32_t parts
) {
  const glm::dmat3 inverse = glm::inverse(inertia);
  const double part = flightStep() / static_cast<double>(parts);

  const auto rate = [&](const glm::dvec3& about) {
    return inverse * glm::cross(inertia * about, about);
  };

  const auto turning = [](const glm::dquat& from, const glm::dvec3& about) {
    return 0.5 * (from * glm::dquat(0.0, about.x, about.y, about.z));
  };

  FlightReference made;

  made.turn.reserve(FLIGHT_STEPS + 1u);
  made.turn.push_back(glm::dmat3(1.0));

  glm::dvec3 omega = spin;
  glm::dquat orientation(1.0, 0.0, 0.0, 0.0);

  for (uint32_t tick = 0u; tick != FLIGHT_STEPS; ++tick) {
    for (uint32_t at = 0u; at != parts; ++at) {
      const glm::dvec3 firstRate = rate(omega);
      const glm::dquat firstTurn = turning(orientation, omega);
      const glm::dvec3 secondSpin = omega + (0.5 * part) * firstRate;
      const glm::dvec3 secondRate = rate(secondSpin);
      const glm::dquat secondTurn =
        turning(orientation + (0.5 * part) * firstTurn, secondSpin);
      const glm::dvec3 thirdSpin = omega + (0.5 * part) * secondRate;
      const glm::dvec3 thirdRate = rate(thirdSpin);
      const glm::dquat thirdTurn =
        turning(orientation + (0.5 * part) * secondTurn, thirdSpin);
      const glm::dvec3 fourthSpin = omega + part * thirdRate;
      const glm::dvec3 fourthRate = rate(fourthSpin);
      const glm::dquat fourthTurn =
        turning(orientation + part * thirdTurn, fourthSpin);

      made.total += part * glm::length(omega);
      omega += (part / 6.0)
               * (firstRate + 2.0 * secondRate + 2.0 * thirdRate + fourthRate);
      orientation =
        orientation
        + (part / 6.0)
            * (firstTurn + 2.0 * secondTurn + 2.0 * thirdTurn + fourthTurn);
    }

    made.turn.push_back(glm::mat3_cast(glm::normalize(orientation)));
  }

  return made;
}

FlightRun flyYielding(const Flight& flight) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  const Entity flying = yieldingBox(
    source,
    talus::objectmodel::Pose{},
    plasticbar::LANE,
    flight.half,
    false
  );

  Runtime runtime(source, library);

  if (!flight.falls) {
    runtime.physics().system().SetGravity(JPH::Vec3::sZero());
  }

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, flying);

  REQUIRE(id.has_value());
  REQUIRE(runtime.rigidUntilYield(flying));

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  bodies.SetLinearVelocity(
    *id,
    JPH::Vec3(flight.velocity.x, flight.velocity.y, flight.velocity.z)
  );
  bodies.SetAngularVelocity(
    *id,
    JPH::Vec3(flight.spin.x, flight.spin.y, flight.spin.z)
  );

  const JPH::BodyLockInterface& locks =
    runtime.physics().system().GetBodyLockInterface();

  glm::dmat3 inertia(1.0);
  double mass = 0.0;
  glm::dvec3 start(0.0);

  {
    const JPH::BodyLockRead lock(locks, *id);

    REQUIRE(lock.Succeeded());

    const JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();
    const JPH::Mat44 local = motion->GetLocalSpaceInverseInertia();
    const JPH::RVec3 centre = lock.GetBody().GetCenterOfMassPosition();

    for (int column = 0; column != 3; ++column) {
      const JPH::Vec3 held = local.GetColumn3(column);

      inertia[column] = glm::dvec3(held.GetX(), held.GetY(), held.GetZ());
    }

    inertia = glm::inverse(inertia);
    mass = 1.0 / static_cast<double>(motion->GetInverseMass());
    start = glm::dvec3(centre.GetX(), centre.GetY(), centre.GetZ());
  }

  const JPH::Vec3 gravity = runtime.physics().system().GetGravity();
  const glm::dvec3 pull(gravity.GetX(), gravity.GetY(), gravity.GetZ());
  const glm::dvec3 spin(flight.spin);

  FlightRun run;
  FlightReference exact;

  if (flight.symmetric) {
    exact = topFlight(inertia, spin);
  } else {
    exact = eulerFlight(inertia, spin, FLIGHT_FINE_PARTS);

    const FlightReference coarse =
      eulerFlight(inertia, spin, FLIGHT_COARSE_PARTS);

    for (uint32_t tick = 0u; tick <= FLIGHT_STEPS; ++tick) {
      run.referenceGap = std::max(
        run.referenceGap,
        flightAngle(exact.turn[tick], coarse.turn[tick])
      );
    }
  }

  run.total = exact.total;

  const glm::dvec3 momentumStart = inertia * spin;
  const double energyStart = 0.5 * glm::dot(spin, momentumStart);
  const glm::dvec3 linearStart = mass * glm::dvec3(flight.velocity);

  for (uint32_t tick = 1u; tick <= FLIGHT_STEPS; ++tick) {
    runtime.step();

    run.promotions += runtime.deformableActivity().promotions;
    run.kept = run.kept && !runtime.promotedDeformable(flying)
               && yieldingBody(runtime, flying) == id;

    JPH::Quat rotation = JPH::Quat::sIdentity();
    JPH::Vec3 angular = JPH::Vec3::sZero();
    JPH::Vec3 linear = JPH::Vec3::sZero();
    JPH::RVec3 centre = JPH::RVec3::sZero();

    {
      const JPH::BodyLockRead lock(locks, *id);

      REQUIRE(lock.Succeeded());

      rotation = lock.GetBody().GetRotation();
      angular = lock.GetBody().GetAngularVelocity();
      linear = lock.GetBody().GetLinearVelocity();
      centre = lock.GetBody().GetCenterOfMassPosition();
    }

    const glm::dmat3 turn = glm::mat3_cast(
      glm::normalize(
        glm::dquat(
          rotation.GetW(),
          rotation.GetX(),
          rotation.GetY(),
          rotation.GetZ()
        )
      )
    );
    const glm::dvec3 omega(angular.GetX(), angular.GetY(), angular.GetZ());
    const glm::dvec3 momentum = turn * inertia * glm::transpose(turn) * omega;
    const double energy = 0.5 * glm::dot(omega, momentum);

    if (energyStart > 0.0) {
      run.angular = std::max(
        run.angular,
        glm::length(momentum - momentumStart) / glm::length(momentumStart)
      );
      run.energy =
        std::max(run.energy, std::abs(energy - energyStart) / energyStart);
      run.turn = std::max(run.turn, flightAngle(turn, exact.turn[tick]));
    }

    if (glm::length(linearStart) > 0.0) {
      const glm::dvec3 carried =
        mass * glm::dvec3(linear.GetX(), linear.GetY(), linear.GetZ());

      run.linear = std::max(
        run.linear,
        glm::length(carried - linearStart) / glm::length(linearStart)
      );
    }

    if (flight.falls && tick == FLIGHT_STEPS) {
      const double elapsed = flightStep() * static_cast<double>(tick);
      const glm::dvec3 moved =
        glm::dvec3(flight.velocity) * elapsed + 0.5 * pull * elapsed * elapsed;
      const glm::dvec3 reached(centre.GetX(), centre.GetY(), centre.GetZ());

      run.fall = glm::length(reached - start - moved) / glm::length(moved);
    }
  }

  return run;
}

}

TEST_CASE(
  "a yielding body in free flight follows exact rigid motion for 600 steps",
  "[deformable]"
) {
  const glm::vec3 bar(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );
  const glm::vec3 plate(
    plasticbar::HALF_LENGTH,
    2.0f * plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );
  const glm::vec3 about = glm::normalize(glm::vec3(1.0f));
  const glm::vec3 coast(1.0f, 2.0f, 3.0f);

  const FlightRun fall =
    flyYielding(Flight{bar, glm::vec3(0.0f), glm::vec3(0.0f), true, true});

  std::cout << "rigidflight: fall positionError " << fall.fall << " promotions "
            << fall.promotions << " kept " << fall.kept << std::endl;

  struct Spun {
    const char* name = "";
    float speed = 0.0f;
    FlightRun run;
  };

  std::vector<Spun> spins;

  for (const bool symmetric : {true, false}) {
    for (const float speed : {10.0f, 20.0f}) {
      const bool coasts = symmetric && speed == 10.0f;

      spins.push_back(
        Spun{
          symmetric ? "bar" : "plate",
          speed,
          flyYielding(
            Flight{
              symmetric ? bar : plate,
              speed * about,
              coasts ? coast : glm::vec3(0.0f),
              false,
              symmetric
            }
          )
        }
      );

      const FlightRun& run = spins.back().run;

      std::cout << "rigidflight: " << spins.back().name << ' ' << speed
                << " rad/s linearDrift " << run.linear << " angularDrift "
                << run.angular << " energyDrift " << run.energy << " turn "
                << run.turn << " of " << run.total << " share "
                << run.turn / run.total << " referenceGap " << run.referenceGap
                << " promotions " << run.promotions << " kept " << run.kept
                << std::endl;
    }
  }

  REQUIRE(fall.kept);
  REQUIRE(fall.promotions == 0u);
  REQUIRE(fall.fall < FLIGHT_FALL);

  for (const Spun& spun : spins) {
    INFO(spun.name << ' ' << spun.speed << " rad/s");

    REQUIRE(spun.run.kept);
    REQUIRE(spun.run.promotions == 0u);
    REQUIRE(
      spun.run.referenceGap <= FLIGHT_REFERENCE * FLIGHT_TURN * spun.run.total
    );
    REQUIRE(spun.run.linear < FLIGHT_LINEAR);
    REQUIRE(spun.run.angular < FLIGHT_ANGULAR);
    REQUIRE(spun.run.energy < FLIGHT_ENERGY);
    REQUIRE(spun.run.turn < FLIGHT_TURN * spun.run.total);
  }
}

TEST_CASE(
  "a yielding body drifting and spinning below Jolt's sleep thresholds falls "
  "asleep in free flight",
  "[deformable]"
) {
  JoltLibrary library;
  World source;

  source.setMaterials(plasticbar::barMaterials(plasticbar::YIELD));

  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );
  const Entity drifting = yieldingBox(
    source,
    talus::objectmodel::Pose{},
    plasticbar::LANE,
    half,
    false
  );

  Runtime runtime(source, library);

  runtime.physics().system().SetGravity(JPH::Vec3::sZero());

  yieldingPrepare(runtime);

  const std::optional<JPH::BodyID> id = yieldingBody(runtime, drifting);

  REQUIRE(id.has_value());
  REQUIRE(runtime.rigidUntilYield(drifting));

  const JPH::PhysicsSettings& settings =
    runtime.physics().system().GetPhysicsSettings();
  const glm::vec3 velocity =
    FLIGHT_DRIFT_SPEED * glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f));
  const glm::vec3 spin = FLIGHT_DRIFT_SPIN * glm::normalize(glm::vec3(1.0f));

  REQUIRE(
    FLIGHT_DRIFT_SPEED + FLIGHT_DRIFT_SPIN * glm::length(half)
    < settings.mPointVelocitySleepThreshold
  );

  JPH::BodyInterface& bodies = runtime.physics().bodies();

  bodies.SetLinearVelocity(*id, JPH::Vec3(velocity.x, velocity.y, velocity.z));
  bodies.SetAngularVelocity(*id, JPH::Vec3(spin.x, spin.y, spin.z));

  REQUIRE(bodies.IsActive(*id));

  uint32_t asleepAt = 0u;
  uint32_t promotions = 0u;

  for (uint32_t tick = 1u; tick <= FLIGHT_DRIFT_STEPS && asleepAt == 0u;
       ++tick) {
    runtime.step();

    promotions += runtime.deformableActivity().promotions;

    if (!bodies.IsActive(*id)) {
      asleepAt = tick;
    }
  }

  std::cout << "rigidflight: drift asleepAt " << asleepAt << " of "
            << FLIGHT_DRIFT_STEPS << " timeBeforeSleep "
            << settings.mTimeBeforeSleep << " promotions " << promotions
            << std::endl;

  REQUIRE(promotions == 0u);
  REQUIRE(!runtime.promotedDeformable(drifting));
  REQUIRE(asleepAt != 0u);
}

namespace {

namespace contactfree {

constexpr float VOXEL = 0.0125f;
constexpr float SPIN_RATE = 20.5f;
constexpr std::array<float, 4> VIBRATIONS{0.15f, 0.2f, 0.25f, 0.3f};
constexpr std::array<float, 2> SHAKES{0.0f, 12.0f};
constexpr uint32_t FRAMES = 3u;

struct Run {
  double worst = -std::numeric_limits<double>::infinity();
  uint32_t worstFrame = 0u;
  uint32_t capped = 0u;
  uint32_t evaluations = 0u;
};

std::shared_ptr<const PreparedMesh> freeBar(
  const std::vector<PhysicalMaterial>& materials
) {
  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * VOXEL));

  primitive.shapeConfig.y = plasticbar::LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(glm::round(2.0f * half / VOXEL)) + glm::uvec3(1u)
  );

  std::shared_ptr<const PreparedMesh> mesh = talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );

  REQUIRE(mesh != nullptr);

  return mesh;
}

float rough(size_t node, uint32_t lane) {
  uint32_t hashed =
    static_cast<uint32_t>(node) * 2654435761u + lane * 40503u + 12345u;

  hashed ^= hashed >> 15u;
  hashed *= 2246822519u;
  hashed ^= hashed >> 13u;
  hashed *= 3266489917u;
  hashed ^= hashed >> 16u;

  return static_cast<float>(hashed) / 4294967295.0f * 2.0f - 1.0f;
}

double elasticEnergy(
  const PreparedMesh& mesh,
  const DeformableState& state,
  const std::vector<PhysicalMaterial>& materials
) {
  const bool sized = state.plastic.size() == 8u * mesh.hexes.size();

  double energy = 0.0;

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];
    const PhysicalMaterial& lane = materials[hex.material];

    std::array<glm::vec3, 8> deformed{};
    std::array<glm::vec3, 8> rest{};
    std::array<talus::objectmodel::PlasticPoint, 8> committed{};

    for (size_t corner = 0u; corner != 8u; ++corner) {
      deformed[corner] = state.position[hex.nodes[corner]];
      rest[corner] = mesh.restPositions[hex.nodes[corner]];

      if (sized) {
        committed[corner] = state.plastic[8u * element + corner];
      }
    }

    const talus::deformable::HexPointTrial measured =
      talus::deformable::hexPointTrial(
        deformed,
        rest,
        hex,
        talus::deformable::ElasticParams{
          lane.youngsModulus,
          lane.poissonsRatio,
          lane.density
        },
        talus::deformable::PlasticParams{
          lane.yieldStress,
          lane.hardeningModulus
        },
        committed,
        0.0f,
        talus::deformable::ElementRotation::Polar,
        false,
        false
      );

    energy += static_cast<double>(measured.energy);
  }

  return energy;
}

double plasticWork(
  const PreparedMesh& mesh,
  const DeformableState& state,
  const std::vector<PhysicalMaterial>& materials
) {
  if (state.plastic.size() != 8u * mesh.hexes.size()) {
    return 0.0;
  }

  double work = 0.0;

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];
    const PhysicalMaterial& lane = materials[hex.material];
    const double weight = 0.125 * static_cast<double>(hex.volume);

    for (size_t point = 0u; point != 8u; ++point) {
      const double strain =
        static_cast<double>(state.plastic[8u * element + point].equivalent);

      if (strain > 0.0) {
        work += weight
                * (static_cast<double>(lane.yieldStress) * strain
                   + 0.5 * static_cast<double>(lane.hardeningModulus) * strain
                       * strain);
      }
    }
  }

  return work;
}

Run spin(
  const PreparedMesh& mesh,
  const std::vector<PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  float vibration,
  float shake
) {
  const glm::vec3 axis = glm::normalize(glm::vec3(0.3f, 0.4f, 1.0f));

  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  const glm::vec3 centre(landingbar::landingCentre(mesh, state));
  const glm::vec3 spun = SPIN_RATE * axis;
  const double mass = landingbar::landingMass(mesh);

  glm::dvec3 noiseMean(0.0);
  glm::dvec3 shakeMean(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 noise(rough(node, 0u), rough(node, 1u), rough(node, 2u));
    const glm::vec3 jolt(rough(node, 3u), rough(node, 4u), rough(node, 5u));

    noiseMean += static_cast<double>(mesh.nodeMass[node]) * glm::dvec3(noise);
    shakeMean += static_cast<double>(mesh.nodeMass[node]) * glm::dvec3(jolt);
  }

  noiseMean /= mass;
  shakeMean /= mass;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 arm = state.position[node] - centre;
    const glm::vec3 noise =
      glm::vec3(rough(node, 0u), rough(node, 1u), rough(node, 2u))
      - glm::vec3(noiseMean);
    const glm::vec3 jolt =
      glm::vec3(rough(node, 3u), rough(node, 4u), rough(node, 5u))
      - glm::vec3(shakeMean);

    state.velocity[node] = glm::cross(spun, arm) + vibration * noise;
    state.acceleration[node] =
      glm::cross(spun, glm::cross(spun, arm)) + shake * jolt;
  }

  auto solver = std::make_unique<talus::deformable::DeformableSolverState>();

  Run run;

  double kinetic = kineticEnergy(mesh, state);
  double elastic = elasticEnergy(mesh, state, materials);
  double work = plasticWork(mesh, state, materials);
  glm::dvec3 massCentre = landingbar::landingCentre(mesh, state);

  for (uint32_t frame = 1u; frame <= FRAMES; ++frame) {
    talus::deformable::StepReport report;

    talus::deformable::step(
      mesh,
      state,
      materials,
      settings,
      landingbar::landingGravity(),
      plasticbar::TIME_STEP,
      {},
      {},
      solver.get(),
      report
    );

    REQUIRE(finiteState(state));

    const double kineticAfter = kineticEnergy(mesh, state);
    const double elasticAfter = elasticEnergy(mesh, state, materials);
    const double workAfter = plasticWork(mesh, state, materials);
    const glm::dvec3 centreAfter = landingbar::landingCentre(mesh, state);
    const double external = -mass
                            * static_cast<double>(landingbar::LANDING_GRAVITY)
                            * (centreAfter.y - massCentre.y);
    const double created = (kineticAfter - kinetic) + (elasticAfter - elastic)
                           + (workAfter - work) - external;

    if (created > run.worst) {
      run.worst = created;
      run.worstFrame = frame;
    }

    run.capped += report.plasticCapped;
    run.evaluations += report.plasticEvaluations;

    kinetic = kineticAfter;
    elastic = elasticAfter;
    work = workAfter;
    massCentre = centreAfter;
  }

  return run;
}

}

}

TEST_CASE(
  "a free plastic bar spinning from a rough post-impact state creates no "
  "energy through its committed plasticity on a contact-free step",
  "[deformable]"
) {
  const std::vector<PhysicalMaterial> yielding =
    plasticbar::barMaterials(plasticbar::YIELD);
  const std::vector<PhysicalMaterial> unyielding =
    plasticbar::barMaterials(std::numeric_limits<float>::infinity());
  const std::shared_ptr<const PreparedMesh> yieldingBar =
    contactfree::freeBar(yielding);
  const std::shared_ptr<const PreparedMesh> unyieldingBar =
    contactfree::freeBar(unyielding);
  const talus::deformable::Settings settings =
    plasticbar::barStep(talus::deformable::VolumeElementType::Hex);

  std::cout << "contactfree: hex voxel " << contactfree::VOXEL << " nodes "
            << yieldingBar->restPositions.size() << " spin "
            << contactfree::SPIN_RATE << " frames " << contactfree::FRAMES
            << " substeps " << settings.substeps << " cg "
            << settings.cgIterations << " tolerance " << settings.tolerance
            << " spectral radius " << settings.spectralRadius
            << " plastic passes " << settings.plasticPasses << '\n';

  for (const float vibration : contactfree::VIBRATIONS) {
    for (const float shake : contactfree::SHAKES) {
      const contactfree::Run control = contactfree::spin(
        *unyieldingBar,
        unyielding,
        settings,
        vibration,
        shake
      );
      const double allowed = std::max(control.worst, 0.0);
      const contactfree::Run plastic =
        contactfree::spin(*yieldingBar, yielding, settings, vibration, shake);

      std::cout << "contactfree: vibration " << vibration << " shake " << shake
                << " yield-disabled rise " << control.worst << " tolerance "
                << allowed << " created " << plastic.worst << " at frame "
                << plastic.worstFrame << " capped " << plastic.capped
                << " evaluations " << plastic.evaluations << '\n';

      INFO(
        "vibration " << vibration << " shake " << shake << ": created "
                     << plastic.worst << " at frame " << plastic.worstFrame
                     << " against " << allowed
      );

      REQUIRE(plastic.evaluations != 0u);
      CHECK(plastic.worst <= allowed);
    }
  }
}
