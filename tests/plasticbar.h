#pragma once

#include "fieldprimitives.h"

#include <talus/deformable/solver/solver.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <catch2/catch_test_macros.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace plasticbar {

inline constexpr uint32_t LANE = 1u;

inline constexpr float HALF_LENGTH = 0.2f;
inline constexpr float HALF_WIDTH = 0.025f;
inline constexpr float GAUGE = 0.1f;

inline constexpr float YOUNGS = 2.0e8f;
inline constexpr float POISSONS = 0.3f;
inline constexpr float DENSITY = 1000.0f;
inline constexpr float YIELD = 2.0e5f;
inline constexpr float HARDENING = 2.0e7f;

inline constexpr float COARSE_VOXEL = 0.025f;

inline constexpr float AREA = 4.0f * HALF_WIDTH * HALF_WIDTH;

inline constexpr float TIME_STEP = 1.0f / 60.0f;

inline constexpr float PAST_STRESS = 1.5f * YIELD;
inline constexpr float PAST_TOTAL =
  PAST_STRESS / YOUNGS + (PAST_STRESS - YIELD) / HARDENING;
inline constexpr float PAST_PERMANENT = (PAST_STRESS - YIELD) / HARDENING;

inline constexpr std::array<talus::deformable::VolumeElementType, 2> ELEMENTS{
  talus::deformable::VolumeElementType::Hex,
  talus::deformable::VolumeElementType::Tet
};

inline const char* elementName(talus::deformable::VolumeElementType element) {
  return element == talus::deformable::VolumeElementType::Hex ? "hex" : "tet";
}

inline std::vector<talus::objectmodel::PhysicalMaterial> barMaterials(
  float yield,
  float hardening = HARDENING
) {
  std::vector<talus::objectmodel::PhysicalMaterial> materials(2);

  materials[LANE].density = DENSITY;
  materials[LANE].youngsModulus = YOUNGS;
  materials[LANE].poissonsRatio = POISSONS;
  materials[LANE].yieldStress = yield;
  materials[LANE].hardeningModulus = hardening;

  return materials;
}

inline talus::deformable::Settings barSolve(
  talus::deformable::VolumeElementType element
) {
  talus::deformable::Settings settings;

  settings.element = element;
  settings.cgIterations = 1200u;
  settings.tolerance = 1.0e-5f;

  return settings;
}

inline talus::deformable::Settings barStep(
  talus::deformable::VolumeElementType element
) {
  talus::deformable::Settings settings;

  settings.element = element;

  return settings;
}

struct Bar {
  std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh;
  talus::deformable::VolumeElementType element =
    talus::deformable::VolumeElementType::Hex;
  std::vector<uint32_t> near;
  std::vector<uint32_t> far;
  std::vector<glm::vec3> pull;
};

inline Bar buildPrism(
  const glm::vec3& half,
  float voxel,
  std::span<const talus::objectmodel::PhysicalMaterial> materials,
  talus::deformable::VolumeElementType element
) {
  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * voxel));

  primitive.shapeConfig.y = LANE;

  const glm::uvec3 resolution =
    glm::uvec3(glm::round(2.0f * half / voxel)) + glm::uvec3(1u);

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    resolution
  );

  const talus::deformable::SupportVolume clamp{
    glm::translate(glm::mat4(1.0f), glm::vec3(half.x + 1.0f, 0.0f, 0.0f)),
    glm::vec3(1.0f + 0.25f * voxel, 1.0f, 1.0f)
  };

  Bar bar;

  bar.element = element;
  bar.mesh = talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>(&clamp, 1u),
    element
  );

  REQUIRE(bar.mesh != nullptr);

  const std::vector<glm::vec3>& rest = bar.mesh->restPositions;

  bar.pull.assign(rest.size(), glm::vec3(0.0f));

  const float edge = 0.25f * voxel;

  float shares = 0.0f;

  for (size_t node = 0u; node != rest.size(); ++node) {
    if (std::abs(rest[node].x - half.x) < edge) {
      const float alongY = half.y - std::abs(rest[node].y) < edge ? 0.5f : 1.0f;
      const float alongZ = half.z - std::abs(rest[node].z) < edge ? 0.5f : 1.0f;

      bar.pull[node].x = alongY * alongZ;
      shares += alongY * alongZ;
    }
  }

  REQUIRE(shares > 0.0f);

  for (glm::vec3& share : bar.pull) {
    share /= shares;
  }

  return bar;
}

inline Bar buildBar(
  float voxel,
  std::span<const talus::objectmodel::PhysicalMaterial> materials,
  talus::deformable::VolumeElementType element
) {
  Bar bar = buildPrism(
    glm::vec3(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH),
    voxel,
    materials,
    element
  );

  const std::vector<glm::vec3>& rest = bar.mesh->restPositions;
  const float edge = 0.25f * voxel;

  for (size_t node = 0u; node != rest.size(); ++node) {
    if (std::abs(rest[node].x + GAUGE) < edge) {
      bar.near.push_back(static_cast<uint32_t>(node));
    }

    if (std::abs(rest[node].x - GAUGE) < edge) {
      bar.far.push_back(static_cast<uint32_t>(node));
    }
  }

  REQUIRE(!bar.near.empty());
  REQUIRE(bar.near.size() == bar.far.size());

  return bar;
}

inline std::vector<glm::vec3> scaled(
  const std::vector<glm::vec3>& pull,
  float force
) {
  std::vector<glm::vec3> load(pull.size());

  std::transform(
    pull.begin(),
    pull.end(),
    load.begin(),
    [&](const glm::vec3& share) { return force * share; }
  );

  return load;
}

inline float gaugeStrain(
  const Bar& bar,
  const talus::objectmodel::DeformableState& state
) {
  const auto meanX = [&](const std::vector<uint32_t>& nodes) {
    double sum = 0.0;

    for (const uint32_t node : nodes) {
      sum += static_cast<double>(
        state.position[node].x - bar.mesh->restPositions[node].x
      );
    }

    return sum / static_cast<double>(nodes.size());
  };

  return static_cast<float>((meanX(bar.far) - meanX(bar.near)) / (2.0 * GAUGE));
}

inline float largestEquivalent(
  const talus::objectmodel::DeformableState& state
) {
  float largest = 0.0f;

  for (const talus::objectmodel::PlasticPoint& point : state.plastic) {
    largest = std::max(largest, point.equivalent);
  }

  return largest;
}

struct Cycle {
  float loaded = 0.0f;
  float held = 0.0f;
  float kept = 0.0f;
  float plastic = 0.0f;
};

inline Cycle cycleBar(
  const Bar& bar,
  std::span<const talus::objectmodel::PhysicalMaterial> materials
) {
  const talus::objectmodel::DeformableState loaded =
    talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      talus::objectmodel::Pose{},
      barSolve(bar.element),
      glm::vec3(0.0f),
      scaled(bar.pull, PAST_STRESS * AREA)
    );

  const talus::objectmodel::DeformableState held =
    talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      loaded,
      barSolve(bar.element),
      glm::vec3(0.0f),
      scaled(bar.pull, PAST_STRESS * AREA)
    );

  const talus::objectmodel::DeformableState unloaded =
    talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      held,
      barSolve(bar.element),
      glm::vec3(0.0f)
    );

  return Cycle{
    gaugeStrain(bar, loaded),
    gaugeStrain(bar, held),
    gaugeStrain(bar, unloaded),
    largestEquivalent(unloaded)
  };
}

inline bool allFinite(const talus::objectmodel::DeformableState& state) {
  return std::all_of(
    state.position.begin(),
    state.position.end(),
    [](const glm::vec3& position) {
      return std::isfinite(position.x) && std::isfinite(position.y)
             && std::isfinite(position.z);
    }
  );
}

inline constexpr uint32_t CYCLE_RAMP = 60u;
inline constexpr uint32_t CYCLE_HOLD = 60u;

inline float bodyPull() {
  return PAST_STRESS / (DENSITY * HALF_LENGTH);
}

struct Reference {
  float loaded = 0.0f;
  float kept = 0.0f;
  float plastic = 0.0f;
};

inline Reference staticReference(
  const Bar& bar,
  std::span<const talus::objectmodel::PhysicalMaterial> materials
) {
  const talus::objectmodel::DeformableState loaded =
    talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      talus::objectmodel::Pose{},
      barSolve(bar.element),
      glm::vec3(bodyPull(), 0.0f, 0.0f)
    );
  const talus::objectmodel::DeformableState kept =
    talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      loaded,
      barSolve(bar.element),
      glm::vec3(0.0f)
    );

  return Reference{
    gaugeStrain(bar, loaded),
    gaugeStrain(bar, kept),
    largestEquivalent(kept)
  };
}

struct Stepped {
  float ramped = 0.0f;
  float held = 0.0f;
  float kept = 0.0f;
  float plastic = 0.0f;
  uint32_t diverged = 0u;
  uint64_t substeps = 0u;
  uint64_t evaluations = 0u;
  uint64_t solves = 0u;
  uint64_t iterations = 0u;
  uint64_t capped = 0u;
  uint64_t backtracks = 0u;
  uint64_t linearReads = 0u;
  uint64_t cgCapped = 0u;
  uint32_t worstSolves = 0u;
  uint32_t worstIterations = 0u;
  float worstResidual = 0.0f;
  float worstRelativeResidual = 0.0f;
};

inline Stepped stepCycle(
  const Bar& bar,
  std::span<const talus::objectmodel::PhysicalMaterial> materials,
  const talus::deformable::Settings& settings,
  talus::deformable::DeformableSolverState* solver,
  talus::objectmodel::DeformableState& state
) {
  Stepped run;

  uint32_t frame = 0u;

  const auto advance = [&](float share) {
    ++frame;

    if (run.diverged != 0u) {
      return;
    }

    talus::deformable::StepReport report;

    talus::deformable::step(
      *bar.mesh,
      state,
      materials,
      settings,
      glm::vec3(share * bodyPull(), 0.0f, 0.0f),
      TIME_STEP,
      {},
      {},
      solver,
      report
    );

    run.substeps += std::max(settings.substeps, 1u);
    run.evaluations += report.plasticEvaluations;
    run.solves += report.plasticSolves;
    run.iterations += report.cgIterations;
    run.capped += report.plasticCapped;
    run.backtracks += report.plasticBacktracks;
    run.linearReads += report.plasticLinearReads;
    run.cgCapped += report.cgCappedSolves;
    run.worstSolves = std::max(run.worstSolves, report.plasticSolves);
    run.worstIterations = std::max(run.worstIterations, report.worstSubstep);

    if (report.plasticWorstResidual > run.worstResidual) {
      run.worstResidual = report.plasticWorstResidual;
      run.worstRelativeResidual = report.plasticWorstRelativeResidual;
    }

    if (!allFinite(state) || std::abs(gaugeStrain(bar, state)) > 1.0f) {
      run.diverged = frame;
    }
  };

  for (uint32_t at = 1u; at <= CYCLE_RAMP; ++at) {
    advance(static_cast<float>(at) / CYCLE_RAMP);
  }

  run.ramped = gaugeStrain(bar, state);

  for (uint32_t at = 0u; at != CYCLE_HOLD; ++at) {
    advance(1.0f);
  }

  run.held = gaugeStrain(bar, state);

  for (uint32_t at = 1u; at <= CYCLE_RAMP; ++at) {
    advance(static_cast<float>(CYCLE_RAMP - at) / CYCLE_RAMP);
  }

  for (uint32_t at = 0u; at != CYCLE_HOLD; ++at) {
    advance(0.0f);
  }

  run.kept = gaugeStrain(bar, state);
  run.plastic = largestEquivalent(state);

  return run;
}

}
