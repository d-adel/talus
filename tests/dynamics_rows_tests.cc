#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/ledger/ledger.h>
#include <talus/dynamics/narrowphase/narrowphase.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/mesh.h>
#include <talus/material.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace {

talus::field::SampledSdf rowBox(const glm::dvec3& half, double voxel) {
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

}

TEST_CASE(
  "dynamics row resting crate: a 1.5 kg crate on a cantilever beam rests "
  "and loads it with its weight",
  "[.][dynamicsrows]"
) {
  std::vector<talus::PhysicalMaterial> lanes(1);

  lanes[0].density = 500.0f;
  lanes[0].youngsModulus = 1.0e7f;
  lanes[0].poissonsRatio = 0.3f;

  const glm::dvec3 beamHalf(0.5, 0.05, 0.05);
  const talus::field::SampledSdf beamField = rowBox(beamHalf, 0.05);
  const talus::deformable::SupportVolume anchor{
    glm::inverse(
      glm::translate(glm::mat4(1.0f), glm::vec3(-0.725f, 0.0f, 0.0f))
    ),
    glm::vec3(0.275f, 1.0f, 1.0f)
  };
  const auto mesh = talus::deformable::buildMesh(
    beamField,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>(&anchor, 1u),
    talus::deformable::VolumeElementType::Hex
  );
  const glm::dvec3 crateHalf(0.05);
  const talus::field::SampledSdf crate = rowBox(crateHalf, 0.01);
  talus::dynamics::World world;

  talus::dynamics::addLattice(
    world.bodies,
    mesh,
    lanes,
    glm::dvec3(0.0),
    glm::dvec3(0.0),
    0.5,
    &beamField
  );
  talus::dynamics::addRigidBox(
    world.bodies,
    crate,
    crateHalf,
    1.5,
    glm::dvec3(-0.25, 0.1, 0.0),
    glm::dvec3(0.0),
    0.5,
    4u
  );

  for (uint32_t frame = 0u; frame != 120u; ++frame) {
    REQUIRE(!talus::dynamics::step(world).nonFinite);
  }

  const glm::dvec3 settled = world.bodies.rigids.front().position;
  double drift = 0.0;
  double fastest = 0.0;
  double load = 0.0;
  double elapsed = 0.0;

  for (uint32_t frame = 0u; frame != 300u; ++frame) {
    const auto clock = std::chrono::steady_clock::now();

    REQUIRE(!talus::dynamics::step(world).nonFinite);
    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - clock
    )
                 .count();
    drift = std::max(
      drift,
      glm::length(world.bodies.rigids.front().position - settled)
    );
    fastest =
      std::max(fastest, glm::length(world.bodies.rigids.front().velocity));
    load += talus::dynamics::normalForceOn(
              world,
              {talus::dynamics::Side::Rigid, 0u}
            )
            / 300.0;
  }

  std::cout << "dynamicsrow: crate drift " << drift << " fastest " << fastest
            << " load " << load << " weight " << 1.5 * 9.81 << " msPerStep "
            << elapsed / 300.0 << '\n';

  CHECK(drift < 0.01);
  CHECK(fastest < 0.05);
  CHECK(std::abs(load - 1.5 * 9.81) <= 0.10 * 1.5 * 9.81);
}

TEST_CASE(
  "dynamics row free collision: a lattice cube hits a rigid box in free space",
  "[.][dynamicsrows]"
) {
  std::vector<talus::PhysicalMaterial> lanes(1);

  lanes[0].density = 1000.0f;
  lanes[0].youngsModulus = 1.2854547e6f;
  lanes[0].poissonsRatio = 0.45f;

  const glm::dvec3 half(0.05);
  const talus::field::SampledSdf cubeField = rowBox(half, 0.0125);
  const auto mesh = talus::deformable::buildMesh(
    cubeField,
    lanes,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    talus::deformable::VolumeElementType::Hex
  );
  const talus::field::SampledSdf crate = rowBox(half, 0.01);
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
  const glm::dvec3 g = world.parameters.gravity;
  const double start = talus::dynamics::energyOf(world.bodies, g).total();
  double worst = 0.0;
  double peak = 0.0;
  double elapsed = 0.0;
  bool touched = false;

  for (uint32_t frame = 0u; frame != 60u; ++frame) {
    const auto clock = std::chrono::steady_clock::now();
    const talus::dynamics::StepReport report = talus::dynamics::step(world);

    elapsed += std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - clock
    )
                 .count();
    REQUIRE(!report.nonFinite);

    for (const talus::dynamics::Contact& contact : world.contacts) {
      touched = touched || contact.memory.normal > 0.0;
    }

    worst = std::max(worst, glm::length(momentum().first - before) / mass);
    peak = std::max(
      peak,
      talus::dynamics::energyOf(world.bodies, g).total() / start
    );
  }

  std::cout << "dynamicsrow: freecollision momentumError " << worst
            << " touched " << (touched ? 1 : 0) << " boxVelocity "
            << world.bodies.rigids.front().velocity.x << " latticeVelocity "
            << momentum().first.x - world.bodies.rigids.front().velocity.x
            << " peakEnergyRatio " << peak << " msPerStep "
            << elapsed / 60.0 << '\n';

  CHECK(touched);
  CHECK(worst <= 1.0e-4);
  CHECK(world.bodies.rigids.front().velocity.x > 0.0);
  CHECK(peak <= 1.02);
}
