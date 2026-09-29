#pragma once

#include "fieldprimitives.h"
#include "plasticbar.h"

#include <talus/deformable/contact/contact.h>
#include <talus/deformable/material/material.h>
#include <talus/deformable/solver/solver.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <catch2/catch_test_macros.hpp>

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace landingbar {

inline constexpr float LANDING_GRAVITY = 9.81f;
inline constexpr float LANDING_DROP = 0.3f;
inline constexpr uint32_t LANDING_FRAMES = 600u;
inline constexpr uint32_t LANDING_REST_WINDOW = 60u;
inline constexpr uint32_t LANDING_TAIL_WINDOW = 100u;
inline constexpr float LANDING_REST_SPEED = 1.0e-2f;
inline constexpr uint32_t LANDING_TRACE = 25u;
inline constexpr float LANDING_SLAB_HALF_HEIGHT = 0.05f;
inline constexpr float LANDING_SLAB_CENTRE = -0.05f;
inline constexpr float LANDING_SLAB_EXTENT = 0.6f;
inline constexpr float LANDING_WIDE_SLAB = 3.0f;

inline double landingKinetic(
  const talus::objectmodel::PreparedMesh& mesh,
  const talus::objectmodel::DeformableState& state
) {
  double kinetic = 0.0;

  for (size_t node = 0u; node != mesh.nodeMass.size(); ++node) {
    const glm::dvec3 moving(state.velocity[node]);

    kinetic +=
      0.5 * static_cast<double>(mesh.nodeMass[node]) * glm::dot(moving, moving);
  }

  return kinetic;
}

inline double landingMass(const talus::objectmodel::PreparedMesh& mesh) {
  double mass = 0.0;

  for (const float held : mesh.nodeMass) {
    mass += static_cast<double>(held);
  }

  return mass;
}

inline glm::dvec3 landingCentre(
  const talus::objectmodel::PreparedMesh& mesh,
  const talus::objectmodel::DeformableState& state
) {
  glm::dvec3 centre(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    centre += static_cast<double>(mesh.nodeMass[node])
              * glm::dvec3(state.position[node]);
  }

  return centre / landingMass(mesh);
}

struct LandingLaw {
  std::vector<talus::deformable::TetStiffness> tet;
  std::vector<talus::deformable::HexStiffness> hex;
};

inline talus::deformable::ElasticParams landingParams(
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials,
  uint32_t lane
) {
  const talus::objectmodel::PhysicalMaterial& held = materials.at(lane);

  return {held.youngsModulus, held.poissonsRatio, held.density};
}

inline LandingLaw landingLaw(
  const talus::objectmodel::PreparedMesh& mesh,
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials
) {
  LandingLaw law;

  for (const talus::objectmodel::PreparedTet& tet : mesh.tets) {
    law.tet.push_back(
      talus::deformable::tetStiffness(
        tet,
        landingParams(materials, tet.material)
      )
    );
  }

  for (const talus::objectmodel::PreparedHex& hex : mesh.hexes) {
    law.hex.push_back(
      talus::deformable::hexStiffness(
        hex,
        landingParams(materials, hex.material)
      )
    );
  }

  return law;
}

inline double landingStrain(
  const talus::objectmodel::PreparedMesh& mesh,
  const talus::objectmodel::DeformableState& state,
  const LandingLaw& law
) {
  double strain = 0.0;

  std::array<glm::vec3, 4> tetDeformed{};
  std::array<glm::vec3, 4> tetRest{};

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const talus::objectmodel::PreparedTet& tet = mesh.tets[element];

    for (int corner = 0; corner != 4; ++corner) {
      tetDeformed[static_cast<size_t>(corner)] =
        state.position[tet.nodes[corner]];
      tetRest[static_cast<size_t>(corner)] =
        mesh.restPositions[tet.nodes[corner]];
    }

    strain += static_cast<double>(
      talus::deformable::trial(tetDeformed, tetRest, tet, law.tet[element])
        .energy
    );
  }

  std::array<glm::vec3, 8> hexDeformed{};
  std::array<glm::vec3, 8> hexRest{};

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];

    for (size_t corner = 0u; corner != 8u; ++corner) {
      hexDeformed[corner] = state.position[hex.nodes[corner]];
      hexRest[corner] = mesh.restPositions[hex.nodes[corner]];
    }

    strain += static_cast<double>(
      talus::deformable::hexTrial(hexDeformed, hexRest, hex, law.hex[element])
        .energy
    );
  }

  return strain;
}

inline std::shared_ptr<const talus::objectmodel::PreparedMesh> landingBar(
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials,
  talus::deformable::VolumeElementType element
) {
  const glm::vec3 half(
    plasticbar::HALF_LENGTH,
    plasticbar::HALF_WIDTH,
    plasticbar::HALF_WIDTH
  );

  talus::field::Primitive primitive = fixture::box(
    glm::vec3(0.0f),
    half + glm::vec3(0.2f * plasticbar::COARSE_VOXEL)
  );

  primitive.shapeConfig.y = plasticbar::LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(glm::round(2.0f * half / plasticbar::COARSE_VOXEL))
      + glm::uvec3(1u)
  );

  std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
    talus::deformable::buildMesh(
      lattice,
      materials,
      glm::mat4(1.0f),
      std::span<const talus::deformable::SupportVolume>{},
      element
    );

  REQUIRE(mesh != nullptr);

  return mesh;
}

inline talus::field::SampledSdf landingSlab(
  float extent = LANDING_SLAB_EXTENT
) {
  const glm::vec3 half(extent, LANDING_SLAB_HALF_HEIGHT, extent);

  talus::field::Primitive primitive = fixture::box(glm::vec3(0.0f), half);

  primitive.shapeConfig.y = plasticbar::LANE;

  const glm::vec3 low = -half - glm::vec3(2.0f * plasticbar::COARSE_VOXEL);
  const glm::vec3 high = half + glm::vec3(2.0f * plasticbar::COARSE_VOXEL);

  return talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    low,
    high,
    glm::uvec3(glm::round((high - low) / plasticbar::COARSE_VOXEL))
      + glm::uvec3(1u)
  );
}

inline glm::vec3 landingGravity() {
  return glm::vec3(0.0f, -LANDING_GRAVITY, 0.0f);
}

inline std::vector<talus::deformable::SurfaceContact> landingContacts(
  const talus::objectmodel::PreparedMesh& mesh,
  const talus::objectmodel::DeformableState& state,
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials,
  const talus::field::SampledSdf& slab
) {
  talus::deformable::ShapeProbe probe;

  probe.field = &slab;
  probe.centerOfMass = glm::vec3(0.0f, LANDING_SLAB_CENTRE, 0.0f);

  const float voxel =
    std::max(mesh.voxelSize.x, std::max(mesh.voxelSize.y, mesh.voxelSize.z));
  const talus::deformable::ContactGaps gaps{0.1f * voxel, 0.2f * voxel};

  return talus::deformable::findContacts(
    mesh,
    state,
    materials,
    landingGravity(),
    std::span<const talus::deformable::ShapeProbe>(&probe, 1u),
    plasticbar::TIME_STEP,
    gaps
  );
}

inline talus::deformable::RigidPartner landingPartner(
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials
) {
  talus::deformable::RigidPartner partner;

  partner.centerOfMass = glm::vec3(0.0f, LANDING_SLAB_CENTRE, 0.0f);
  partner.dynamic = false;
  partner.friction = materials[plasticbar::LANE].friction;

  return partner;
}

inline void landingStep(
  const talus::objectmodel::PreparedMesh& mesh,
  talus::objectmodel::DeformableState& state,
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  std::span<const talus::deformable::SurfaceContact> contacts,
  std::span<talus::deformable::RigidPartner> partners,
  talus::deformable::DeformableSolverState* solver,
  talus::deformable::StepReport& report
) {
  talus::deformable::step(
    mesh,
    state,
    materials,
    settings,
    landingGravity(),
    plasticbar::TIME_STEP,
    contacts,
    partners,
    solver,
    report
  );
}

struct LandingRun {
  double release = 0.0;
  double releaseKinetic = 0.0;
  double releaseStrain = 0.0;
  double peak = 0.0;
  uint32_t peakFrame = 0u;
  double peakKinetic = 0.0;
  double peakStrain = 0.0;
  double peakPotential = 0.0;
  double peakReportedKinetic = 0.0;
  double peakReportedElastic = 0.0;
  float peakSpeed = 0.0f;
  size_t peakContacts = 0u;
  double maxKineticRatio = 0.0;
  uint32_t maxKineticFrame = 0u;
  double peakAfterContact = 0.0;
  uint32_t peakAfterContactFrame = 0u;
  double end = 0.0;
  double endKinetic = 0.0;
  double endStrain = 0.0;
  double endPotential = 0.0;
  uint32_t firstContact = 0u;
  uint32_t impacts = 0u;
  double worstImpactChange = -std::numeric_limits<double>::infinity();
  uint32_t worstImpactFrame = 0u;
  std::vector<uint32_t> impactFrames;
  std::vector<double> impactChanges;
  bool impactPending = false;
  double contactKinetic = 0.0;
  double contactStrain = 0.0;
  uint32_t velocityPasses = 0u;
  uint32_t velocityTotalPasses = 0u;
  float velocityResidual = 0.0f;
  uint32_t velocityCapped = 0u;
  uint32_t posedSubsteps = 0u;
  uint32_t posedSolves = 0u;
  uint32_t posedCapped = 0u;
  double posedKineticChange = 0.0;
  double reboundSpeed = 0.0;
  float maxPenetration = 0.0f;
  uint32_t cappedSolves = 0u;
  uint32_t cappedFrames = 0u;
  uint32_t contactOverTolerance = 0u;
  uint32_t firstNonFinite = 0u;
  float restWindowSpeed = 0.0f;
  float tailWindowSpeed = 0.0f;
  uint32_t restFrame = 1u;
  double endSpin = 0.0;
  double endHeight = 0.0;
  bool atRest = false;
  bool aboveSlab = false;
  std::string trace;
  talus::objectmodel::DeformableState contactState;
  talus::objectmodel::DeformableState endState;
};

inline LandingRun landBar(
  const talus::objectmodel::PreparedMesh& mesh,
  const std::vector<talus::objectmodel::PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  float rate,
  const glm::vec3& axis = glm::vec3(1.0f, 0.0f, 0.0f),
  float extent = LANDING_SLAB_EXTENT
) {
  const talus::field::SampledSdf slab = landingSlab(extent);
  const double mass = landingMass(mesh);
  const double datum = static_cast<double>(plasticbar::HALF_WIDTH);

  talus::objectmodel::Pose pose;

  pose.position = glm::vec3(0.0f, LANDING_DROP + plasticbar::HALF_WIDTH, 0.0f);

  talus::objectmodel::DeformableState state =
    talus::deformable::restState(mesh, pose);

  const glm::vec3 centre(landingCentre(mesh, state));
  const glm::vec3 spin = rate * axis;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    state.velocity[node] = glm::cross(spin, state.position[node] - centre);
    state.acceleration[node] = glm::vec3(0.0f);
  }

  const LandingLaw law = landingLaw(mesh, materials);

  const auto potential = [&]() {
    return mass * static_cast<double>(LANDING_GRAVITY)
           * (landingCentre(mesh, state).y - datum);
  };

  const auto total = [&]() {
    return landingKinetic(mesh, state) + landingStrain(mesh, state, law)
           + potential();
  };

  LandingRun run;

  run.releaseKinetic = landingKinetic(mesh, state);
  run.releaseStrain = landingStrain(mesh, state, law);
  run.release = total();

  const talus::deformable::RigidPartner partner = landingPartner(materials);

  auto solver = std::make_unique<talus::deformable::DeformableSolverState>();
  std::vector<talus::deformable::RigidPartner> partners;
  std::ostringstream trace;

  double entering = run.release;
  double impactEntering = 0.0;
  uint32_t impactFrame = 0u;
  bool touching = false;

  for (uint32_t frame = 0u; frame != LANDING_FRAMES; ++frame) {
    const std::vector<talus::deformable::SurfaceContact> contacts =
      landingContacts(mesh, state, materials, slab);

    const bool onset = !contacts.empty() && !touching;

    touching = !contacts.empty();

    if (!contacts.empty() && run.firstContact == 0u) {
      run.contactState = state;
    }

    partners.assign(1u, partner);

    talus::deformable::StepReport report;

    landingStep(
      mesh,
      state,
      materials,
      settings,
      contacts,
      partners,
      solver.get(),
      report
    );

    if (!contacts.empty() && run.firstContact == 0u) {
      run.firstContact = frame + 1u;
    }

    run.maxPenetration = std::max(run.maxPenetration, report.maxPenetration);
    run.velocityPasses =
      std::max(run.velocityPasses, report.velocityContactPasses);
    run.velocityTotalPasses += report.velocityContactTotalPasses;
    run.velocityResidual =
      std::max(run.velocityResidual, report.velocityContactResidual);
    run.velocityCapped += report.velocityContactCapped;
    run.posedSubsteps += report.posedSubsteps;
    run.posedSolves += report.posedSolves;
    run.posedCapped += report.posedCapped;
    run.posedKineticChange += report.posedKineticChange;
    run.cappedSolves += report.cgCappedSolves;
    run.cappedFrames += report.cgCappedSolves != 0u ? 1u : 0u;

    if (
      !contacts.empty() && report.contactResidual > settings.contactTolerance
    ) {
      ++run.contactOverTolerance;
    }

    const double kinetic = landingKinetic(mesh, state);
    const double strain = landingStrain(mesh, state, law);
    const double lifted = potential();
    const double energy = kinetic + strain + lifted;

    if (frame + 1u == run.firstContact) {
      run.contactKinetic = kinetic;
      run.contactStrain = strain;
    }

    if (impactFrame != 0u && frame == impactFrame) {
      const double change = energy - impactEntering;

      run.impactFrames.push_back(impactFrame);
      run.impactChanges.push_back(change);

      if (change > run.worstImpactChange) {
        run.worstImpactChange = change;
        run.worstImpactFrame = impactFrame;
      }

      ++run.impacts;
      impactFrame = 0u;
    }

    if (onset) {
      impactEntering = entering;
      impactFrame = frame + 1u;
    }

    entering = energy;

    if (run.firstContact != 0u) {
      glm::dvec3 lift(0.0);

      for (size_t node = 0u; node != state.velocity.size(); ++node) {
        lift += static_cast<double>(mesh.nodeMass[node])
                * glm::dvec3(state.velocity[node]);
      }

      run.reboundSpeed = std::max(run.reboundSpeed, lift.y / mass);
    }

    if (kinetic / run.release > run.maxKineticRatio) {
      run.maxKineticRatio = kinetic / run.release;
      run.maxKineticFrame = frame + 1u;
    }

    if (!std::isfinite(energy) && run.firstNonFinite == 0u) {
      run.firstNonFinite = frame + 1u;
    }

    if (energy / run.release > run.peak) {
      run.peak = energy / run.release;
      run.peakFrame = frame + 1u;
      run.peakKinetic = kinetic;
      run.peakStrain = strain;
      run.peakPotential = lifted;
      run.peakReportedKinetic = static_cast<double>(report.kineticEnergy);
      run.peakReportedElastic = static_cast<double>(report.elasticEnergy);
      run.peakContacts = contacts.size();
      run.peakSpeed = 0.0f;

      for (const glm::vec3& moving : state.velocity) {
        run.peakSpeed = std::max(run.peakSpeed, glm::length(moving));
      }
    }

    if (run.firstContact != 0u && energy / run.release > run.peakAfterContact) {
      run.peakAfterContact = energy / run.release;
      run.peakAfterContactFrame = frame + 1u;
    }

    if (frame + LANDING_REST_WINDOW >= LANDING_FRAMES) {
      for (const glm::vec3& moving : state.velocity) {
        const float speed = glm::length(moving);

        run.restWindowSpeed = std::isfinite(speed)
                                ? std::max(run.restWindowSpeed, speed)
                                : std::numeric_limits<float>::infinity();
      }
    }

    float fastest = 0.0f;

    for (const glm::vec3& moving : state.velocity) {
      const float speed = glm::length(moving);

      fastest = std::isfinite(speed) ? std::max(fastest, speed)
                                     : std::numeric_limits<float>::infinity();
    }

    if (!(fastest < LANDING_REST_SPEED)) {
      run.restFrame = frame + 2u;
    }

    if (frame + LANDING_TAIL_WINDOW >= LANDING_FRAMES) {
      run.tailWindowSpeed = std::max(run.tailWindowSpeed, fastest);
    }

    if ((frame + 1u) % LANDING_TRACE == 0u) {
      trace << " f" << frame + 1u << ' ' << energy / run.release;
    }
  }

  run.impactPending = impactFrame != 0u;
  run.endKinetic = landingKinetic(mesh, state);
  run.endStrain = landingStrain(mesh, state, law);
  run.endPotential = potential();
  run.end = (run.endKinetic + run.endStrain + run.endPotential) / run.release;

  const glm::dvec3 ended = landingCentre(mesh, state);
  const glm::dvec3 about = glm::normalize(glm::dvec3(axis));

  double momentum = 0.0;
  double inertia = 0.0;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(state.position[node]) - ended;
    const glm::dvec3 across = arm - glm::dot(arm, about) * about;
    const double held = static_cast<double>(mesh.nodeMass[node]);

    momentum +=
      held * glm::dot(glm::cross(arm, glm::dvec3(state.velocity[node])), about);
    inertia += held * glm::dot(across, across);
  }

  run.endSpin = momentum / inertia;
  run.endHeight = ended.y;
  run.atRest =
    run.firstNonFinite == 0u && run.restWindowSpeed < LANDING_REST_SPEED;
  run.aboveSlab =
    std::isfinite(run.endHeight)
    && run.endHeight
         > static_cast<double>(LANDING_SLAB_CENTRE + LANDING_SLAB_HALF_HEIGHT);
  run.trace = trace.str();
  run.endState = state;

  return run;
}

}
