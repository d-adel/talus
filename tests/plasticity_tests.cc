#include <catch2/catch_test_macros.hpp>

#include "landingbar.h"
#include "plasticbar.h"

#include <talus/deformable/factor/factor.h>
#include <talus/deformable/material/material.h>
#include <talus/deformable/solver/solver.h>
#include <talus/objectmodel/component/deformablebody/deformablebody.h>
#include <talus/objectmodel/component/material/material.h>
#include <talus/objectmodel/component/pose/pose.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <vector>

using talus::deformable::VolumeElementType;
using talus::objectmodel::DeformableState;
using talus::objectmodel::PhysicalMaterial;

using namespace plasticbar;

namespace {

float largestMotion(const Bar& bar, const DeformableState& state) {
  float largest = 0.0f;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    largest = std::max(
      largest,
      glm::length(state.position[node] - bar.mesh->restPositions[node])
    );
  }

  return largest;
}

float largestAcceleration(const DeformableState& state) {
  float largest = 0.0f;

  for (const glm::vec3& acceleration : state.acceleration) {
    largest = std::max(largest, glm::length(acceleration));
  }

  return largest;
}

std::array<glm::vec3, 8> unitHex(const glm::vec3& half) {
  std::array<glm::vec3, 8> corners{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    corners[corner] = half
                      * glm::vec3(
                        (corner & 1u) != 0u ? 1.0f : -1.0f,
                        (corner & 2u) != 0u ? 1.0f : -1.0f,
                        (corner & 4u) != 0u ? 1.0f : -1.0f
                      );
  }

  return corners;
}

talus::objectmodel::PreparedTet preparedTet(
  const std::array<glm::vec3, 4>& rest
) {
  const glm::mat3 shape(
    rest[1] - rest[0],
    rest[2] - rest[0],
    rest[3] - rest[0]
  );

  talus::objectmodel::PreparedTet tet{};

  tet.restInverse = glm::inverse(shape);
  tet.volume = glm::determinant(shape) / 6.0f;

  return tet;
}

std::shared_ptr<const talus::objectmodel::PreparedMesh> freeBar(
  std::span<const PhysicalMaterial> materials,
  VolumeElementType element
) {
  const glm::vec3 half(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH);

  talus::field::Primitive primitive =
    fixture::box(glm::vec3(0.0f), half + glm::vec3(0.2f * COARSE_VOXEL));

  primitive.shapeConfig.y = LANE;

  const talus::field::SampledSdf lattice = talus::field::bake(
    std::vector<talus::field::Primitive>{primitive},
    -half,
    half,
    glm::uvec3(glm::round(2.0f * half / COARSE_VOXEL)) + glm::uvec3(1u)
  );

  return talus::deformable::buildMesh(
    lattice,
    materials,
    glm::mat4(1.0f),
    std::span<const talus::deformable::SupportVolume>{},
    element
  );
}

void spinFreeBar(uint32_t frames) {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, element);

    REQUIRE(mesh != nullptr);

    for (const float rate : {5.0f, 10.0f}) {
      for (const bool steady : {false, true}) {
        const talus::deformable::Settings settings = barStep(element);

        auto solver =
          std::make_unique<talus::deformable::DeformableSolverState>();

        DeformableState state =
          talus::deformable::restState(*mesh, talus::objectmodel::Pose{});

        glm::vec3 centre(0.0f);
        float mass = 0.0f;

        for (size_t node = 0u; node != state.position.size(); ++node) {
          centre += mesh->nodeMass[node] * state.position[node];
          mass += mesh->nodeMass[node];
        }

        centre /= mass;

        const glm::vec3 spin(0.0f, 0.0f, rate);

        for (size_t node = 0u; node != state.position.size(); ++node) {
          const glm::vec3 arm = state.position[node] - centre;

          state.velocity[node] = glm::cross(spin, arm);
          state.acceleration[node] =
            steady ? glm::cross(spin, glm::cross(spin, arm)) : glm::vec3(0.0f);
        }

        uint32_t firstFlow = 0u;
        uint64_t linearReads = 0u;

        for (uint32_t frame = 0u; frame != frames && firstFlow == 0u; ++frame) {
          talus::deformable::StepReport report;

          talus::deformable::step(
            *mesh,
            state,
            materials,
            settings,
            glm::vec3(0.0f),
            TIME_STEP,
            {},
            {},
            solver.get(),
            report
          );

          linearReads += report.plasticLinearReads;

          if (largestEquivalent(state) > 0.0f) {
            firstFlow = frame + 1u;
          }
        }

        std::cout << "spin " << frames << " frames: " << elementName(element)
                  << " rate " << rate << " rad/s start "
                  << (steady ? "steady spin" : "zero acceleration")
                  << ": first flow frame " << firstFlow
                  << ", largest equivalent " << largestEquivalent(state)
                  << ", linear reads " << linearReads << "\n";

        INFO(
          elementName(element)
          << " rate " << rate << " rad/s start "
          << (steady ? "steady spin" : "zero acceleration")
          << ": first flow frame " << firstFlow << ", largest equivalent "
          << largestEquivalent(state) << ", linear reads " << linearReads
        );

        CHECK(firstFlow == 0u);
        CHECK(linearReads == 0u);
      }
    }
  }
}

}

TEST_CASE(
  "a hex below yield answers the same through the plastic point path",
  "[plasticity]"
) {
  const glm::vec3 half(0.02f, 0.015f, 0.01f);

  talus::objectmodel::PreparedHex hex{};

  hex.halfExtents = half;
  hex.volume = 8.0f * half.x * half.y * half.z;

  const talus::deformable::ElasticParams elastic{YOUNGS, POISSONS, DENSITY};
  const talus::deformable::PlasticParams plastic{
    std::numeric_limits<float>::max(),
    HARDENING
  };

  REQUIRE(talus::deformable::yields(plastic));

  const std::array<glm::vec3, 8> rest = unitHex(half);

  const glm::mat3 turned = glm::mat3(
    glm::rotate(
      glm::mat4(1.0f),
      0.4f,
      glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))
    )
  );

  std::array<glm::vec3, 8> deformed{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::vec3 at = rest[corner];
    const glm::vec3 strained =
      at
      + 1.0e-3f
          * glm::vec3(
            2.0f * at.x + 0.5f * at.y,
            -0.7f * at.y + 0.3f * at.z + 40.0f * at.x * at.y,
            0.4f * at.z - 60.0f * at.y * at.z
          );

    deformed[corner] = turned * strained + glm::vec3(0.3f, -0.1f, 0.2f);
  }

  const talus::deformable::HexStiffness stiffness =
    talus::deformable::hexStiffness(hex, elastic);

  const talus::deformable::HexTrial reference =
    talus::deformable::hexTrial(deformed, rest, hex, stiffness);

  const talus::deformable::HexPointTrial point =
    talus::deformable::hexPointTrial(deformed, rest, hex, elastic, plastic, {});

  float scale = 0.0f;

  for (const glm::vec3& force : reference.force) {
    scale = std::max(scale, glm::length(force));
  }

  float blockScale = 0.0f;

  for (const glm::mat3& block : stiffness.block) {
    for (int column = 0; column != 3; ++column) {
      blockScale = std::max(blockScale, glm::length(block[column]));
    }
  }

  REQUIRE(scale > 0.0f);
  REQUIRE(point.yielding == 0u);
  REQUIRE(point.plasticWork == 0.0f);

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    REQUIRE(
      glm::length(point.force[corner] - reference.force[corner])
      <= 1.0e-5f * scale
    );
  }

  REQUIRE(
    std::abs(point.energy - reference.energy) <= 1.0e-5f * reference.energy
  );

  for (size_t block = 0u; block != stiffness.block.size(); ++block) {
    for (int column = 0; column != 3; ++column) {
      REQUIRE(
        glm::length(
          point.tangent.block[block][column] - stiffness.block[block][column]
        )
        <= 1.0e-5f * blockScale
      );
    }
  }
}

TEST_CASE(
  "the plastic tangent is the derivative of the plastic force",
  "[plasticity]"
) {
  constexpr float STRETCH = 4.0e-3f;
  constexpr float FURTHER = 4.5e-3f;

  const glm::vec3 half(0.5f * COARSE_VOXEL);

  talus::objectmodel::PreparedHex hex{};

  hex.halfExtents = half;
  hex.volume = 8.0f * half.x * half.y * half.z;

  const talus::deformable::ElasticParams elastic{YOUNGS, POISSONS, DENSITY};
  const talus::deformable::PlasticParams plastic{YIELD, HARDENING};

  const std::array<glm::vec3, 8> rest = unitHex(half);

  const auto shaped = [&](float axial, float shear, float twist) {
    std::array<glm::vec3, 8> deformed{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const glm::vec3 at = rest[corner];

      deformed[corner] =
        at
        + glm::vec3(
          axial * at.x + twist * at.y * at.z / half.x,
          -0.5f * axial * at.y + shear * at.z + twist * at.x * at.y / half.x,
          -0.5f * axial * at.z + shear * at.y
        );
    }

    return deformed;
  };

  const talus::deformable::HexPointTrial first =
    talus::deformable::hexPointTrial(
      shaped(STRETCH, 0.0f, 0.0f),
      rest,
      hex,
      elastic,
      plastic,
      {}
    );

  REQUIRE(first.yielding == 8u);

  const std::array<glm::vec3, 8> deformed = shaped(FURTHER, 1.0e-3f, 2.0e-3f);

  const talus::deformable::HexPointTrial base =
    talus::deformable::hexPointTrial(
      deformed,
      rest,
      hex,
      elastic,
      plastic,
      first.points
    );

  REQUIRE(base.yielding == 8u);

  const float delta = 1.0e-3f * half.x;

  float worst = 0.0f;
  float scale = 0.0f;

  for (uint32_t column = 0u; column != 8u; ++column) {
    for (int axis = 0; axis != 3; ++axis) {
      std::array<glm::vec3, 8> plus = deformed;
      std::array<glm::vec3, 8> minus = deformed;

      plus[column][axis] += delta;
      minus[column][axis] -= delta;

      const talus::deformable::HexPointTrial up =
        talus::deformable::hexPointTrial(
          plus,
          rest,
          hex,
          elastic,
          plastic,
          first.points
        );
      const talus::deformable::HexPointTrial down =
        talus::deformable::hexPointTrial(
          minus,
          rest,
          hex,
          elastic,
          plastic,
          first.points
        );

      for (uint32_t row = 0u; row != 8u; ++row) {
        const glm::vec3 numeric =
          -(up.force[row] - down.force[row]) / (2.0f * delta);
        const glm::vec3 analytic =
          base.rotation * base.tangent.block[row * 8u + column][axis];

        worst = std::max(worst, glm::length(numeric - analytic));
        scale = std::max(scale, glm::length(analytic));
      }
    }
  }

  INFO("worst " << worst << " of " << scale);

  REQUIRE(scale > 0.0f);
  REQUIRE(worst <= 0.01f * scale);
}

TEST_CASE(
  "a tet below yield answers the same through the plastic point path",
  "[plasticity]"
) {
  const std::array<glm::vec3, 4> rest{
    glm::vec3(0.0f),
    glm::vec3(0.02f, 0.001f, 0.0f),
    glm::vec3(0.004f, 0.015f, 0.002f),
    glm::vec3(0.003f, 0.002f, 0.01f)
  };

  const talus::objectmodel::PreparedTet tet = preparedTet(rest);

  REQUIRE(tet.volume > 0.0f);

  const talus::deformable::ElasticParams elastic{YOUNGS, POISSONS, DENSITY};
  const talus::deformable::PlasticParams plastic{
    std::numeric_limits<float>::max(),
    HARDENING
  };

  REQUIRE(talus::deformable::yields(plastic));

  const glm::mat3 turned = glm::mat3(
    glm::rotate(
      glm::mat4(1.0f),
      0.4f,
      glm::normalize(glm::vec3(1.0f, 2.0f, 3.0f))
    )
  );

  std::array<glm::vec3, 4> deformed{};

  for (uint32_t corner = 0u; corner != 4u; ++corner) {
    const glm::vec3 at = rest[corner];
    const glm::vec3 strained = at
                               + 1.0e-3f
                                   * glm::vec3(
                                     2.0f * at.x + 0.5f * at.y,
                                     -0.7f * at.y + 0.3f * at.z,
                                     0.4f * at.z - 0.6f * at.x
                                   );

    deformed[corner] = turned * strained + glm::vec3(0.3f, -0.1f, 0.2f);
  }

  const talus::deformable::TetStiffness stiffness =
    talus::deformable::tetStiffness(tet, elastic);

  const talus::deformable::TrialState reference =
    talus::deformable::trial(deformed, rest, tet, stiffness);

  const talus::deformable::TetPointTrial point =
    talus::deformable::tetPointTrial(deformed, rest, tet, elastic, plastic, {});

  float scale = 0.0f;

  for (const glm::vec3& force : reference.force) {
    scale = std::max(scale, glm::length(force));
  }

  float blockScale = 0.0f;

  for (const glm::mat3& block : stiffness.block) {
    for (int column = 0; column != 3; ++column) {
      blockScale = std::max(blockScale, glm::length(block[column]));
    }
  }

  REQUIRE(scale > 0.0f);
  REQUIRE(point.yielding == 0u);
  REQUIRE(point.plasticWork == 0.0f);

  for (uint32_t corner = 0u; corner != 4u; ++corner) {
    CHECK(
      glm::length(point.force[corner] - reference.force[corner])
      <= 1.0e-5f * scale
    );
  }

  CHECK(
    std::abs(point.energy - reference.energy) <= 1.0e-5f * reference.energy
  );

  for (size_t block = 0u; block != stiffness.block.size(); ++block) {
    for (int column = 0; column != 3; ++column) {
      CHECK(
        glm::length(
          point.tangent.block[block][column] - stiffness.block[block][column]
        )
        <= 1.0e-5f * blockScale
      );
    }
  }
}

TEST_CASE(
  "the plastic tet tangent is the derivative of the plastic force",
  "[plasticity]"
) {
  constexpr float STRETCH = 4.0e-3f;
  constexpr float FURTHER = 4.5e-3f;

  const std::array<glm::vec3, 8> cube = unitHex(glm::vec3(0.5f * COARSE_VOXEL));
  const std::array<glm::vec3, 4> rest{cube[0], cube[1], cube[3], cube[7]};
  const float half = 0.5f * COARSE_VOXEL;

  const talus::objectmodel::PreparedTet tet = preparedTet(rest);

  REQUIRE(tet.volume > 0.0f);

  const talus::deformable::ElasticParams elastic{YOUNGS, POISSONS, DENSITY};
  const talus::deformable::PlasticParams plastic{YIELD, HARDENING};

  const auto shaped = [&](float axial, float shear, float twist) {
    std::array<glm::vec3, 4> deformed{};

    for (uint32_t corner = 0u; corner != 4u; ++corner) {
      const glm::vec3 at = rest[corner];

      deformed[corner] =
        at
        + glm::vec3(
          axial * at.x + twist * at.y * at.z / half,
          -0.5f * axial * at.y + shear * at.z + twist * at.x * at.y / half,
          -0.5f * axial * at.z + shear * at.y
        );
    }

    return deformed;
  };

  const talus::deformable::TetPointTrial first =
    talus::deformable::tetPointTrial(
      shaped(STRETCH, 0.0f, 0.0f),
      rest,
      tet,
      elastic,
      plastic,
      {}
    );

  REQUIRE(first.yielding == 1u);

  const std::array<glm::vec3, 4> deformed = shaped(FURTHER, 1.0e-3f, 2.0e-3f);

  const talus::deformable::TetPointTrial base =
    talus::deformable::tetPointTrial(
      deformed,
      rest,
      tet,
      elastic,
      plastic,
      first.point
    );

  REQUIRE(base.yielding == 1u);

  const float delta = 1.0e-3f * half;

  float worst = 0.0f;
  float scale = 0.0f;

  for (uint32_t column = 0u; column != 4u; ++column) {
    for (int axis = 0; axis != 3; ++axis) {
      std::array<glm::vec3, 4> plus = deformed;
      std::array<glm::vec3, 4> minus = deformed;

      plus[column][axis] += delta;
      minus[column][axis] -= delta;

      const talus::deformable::TetPointTrial up =
        talus::deformable::tetPointTrial(
          plus,
          rest,
          tet,
          elastic,
          plastic,
          first.point
        );
      const talus::deformable::TetPointTrial down =
        talus::deformable::tetPointTrial(
          minus,
          rest,
          tet,
          elastic,
          plastic,
          first.point
        );

      for (uint32_t row = 0u; row != 4u; ++row) {
        const glm::vec3 numeric =
          -(up.force[row] - down.force[row]) / (2.0f * delta);
        const glm::vec3 analytic =
          base.rotation * base.tangent.block[row * 4u + column][axis];

        worst = std::max(worst, glm::length(numeric - analytic));
        scale = std::max(scale, glm::length(analytic));
      }
    }
  }

  INFO("worst " << worst << " of " << scale);

  REQUIRE(scale > 0.0f);
  REQUIRE(worst <= 0.01f * scale);
}

TEST_CASE("a plastic bar loaded below yield springs back", "[plasticity]") {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const Bar bar = buildBar(COARSE_VOXEL, materials, element);

    const DeformableState loaded = talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      talus::objectmodel::Pose{},
      barSolve(bar.element),
      glm::vec3(0.0f),
      scaled(bar.pull, 0.5f * YIELD * AREA)
    );

    const DeformableState unloaded = talus::deformable::solveEquilibrium(
      *bar.mesh,
      materials,
      loaded,
      barSolve(bar.element),
      glm::vec3(0.0f)
    );

    const float peak = largestMotion(bar, loaded);
    const float left = largestMotion(bar, unloaded);

    INFO(
      elementName(element) << ": peak " << peak << " left " << left
                           << " gauge strain " << gaugeStrain(bar, loaded)
    );

    CHECK(largestEquivalent(loaded) == 0.0f);
    CHECK(largestEquivalent(unloaded) == 0.0f);
    REQUIRE(peak > 0.0f);
    CHECK(left <= 0.01f * peak);
  }
}

TEST_CASE(
  "a plastic bar loaded past yield keeps the uniaxial permanent strain",
  "[plasticity]"
) {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const Bar bar = buildBar(COARSE_VOXEL, materials, element);
    const Cycle cycle = cycleBar(bar, materials);

    INFO(
      elementName(element) << ": loaded " << cycle.loaded << " against "
                           << PAST_TOTAL << "; held " << cycle.held << "; kept "
                           << cycle.kept << " against " << PAST_PERMANENT
    );

    CHECK(std::abs(cycle.loaded - PAST_TOTAL) <= 0.05f * PAST_TOTAL);
    CHECK(std::abs(cycle.kept - PAST_PERMANENT) <= 0.05f * PAST_PERMANENT);
    CHECK(
      std::abs(cycle.held - cycle.loaded)
      <= talus::deformable::EQUILIBRIUM_MOTION / (2.0f * GAUGE)
    );
    REQUIRE(cycle.plastic > 0.0f);
  }
}

TEST_CASE(
  "a plastic bar keeps its permanent strain under two refinements",
  "[.][plasticityrefined]"
) {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    std::array<float, 3> kept{};

    for (size_t refinement = 0u; refinement != kept.size(); ++refinement) {
      const float voxel = COARSE_VOXEL / static_cast<float>(1u << refinement);
      const Bar bar = buildBar(voxel, materials, element);
      const Cycle cycle = cycleBar(bar, materials);

      kept[refinement] = cycle.kept;

      std::cout << "plasticity: " << elementName(element) << " voxel " << voxel
                << " nodes " << bar.mesh->restPositions.size() << " loaded "
                << cycle.loaded << " reference " << PAST_TOTAL << " held "
                << cycle.held << " kept " << cycle.kept << " reference "
                << PAST_PERMANENT << '\n';

      INFO(elementName(element) << " voxel " << voxel);

      CHECK(std::abs(cycle.loaded - PAST_TOTAL) <= 0.05f * PAST_TOTAL);
      CHECK(std::abs(cycle.kept - PAST_PERMANENT) <= 0.05f * PAST_PERMANENT);
      CHECK(
        std::abs(cycle.held - cycle.loaded)
        <= talus::deformable::EQUILIBRIUM_MOTION / (2.0f * GAUGE)
      );

      if (refinement != 0u) {
        CHECK(
          std::abs(kept[refinement] - kept[refinement - 1u])
          <= 0.05f * kept[refinement - 1u]
        );
      }
    }
  }
}

TEST_CASE(
  "a plastic bar keeps its static strain at one production substep",
  "[.][plasticruntime]"
) {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const Bar bar = buildBar(COARSE_VOXEL, materials, element);
    const Reference reference = staticReference(bar, materials);

    talus::deformable::Settings settings = barStep(bar.element);

    settings.substeps = 1u;

    auto solver = std::make_unique<talus::deformable::DeformableSolverState>();

    DeformableState state =
      talus::deformable::restState(*bar.mesh, talus::objectmodel::Pose{});

    const Stepped run =
      stepCycle(bar, materials, settings, solver.get(), state);

    std::cout << "plasticruntime: " << elementName(element) << " substeps "
              << settings.substeps << " cg " << settings.cgIterations
              << " factor budget " << settings.factorBudget << " tolerance "
              << settings.tolerance << " spectral radius "
              << settings.spectralRadius << " plastic passes "
              << settings.plasticPasses << " factored; static loaded "
              << reference.loaded << " kept " << reference.kept
              << "; stepped held " << run.held << " ("
              << 100.0f * (run.held / reference.loaded - 1.0f) << "%) kept "
              << run.kept << " (" << 100.0f * (run.kept / reference.kept - 1.0f)
              << "%) capped substeps " << run.capped << " linear reads "
              << run.linearReads << '\n';

    INFO(
      elementName(element) << ": static loaded " << reference.loaded << " kept "
                           << reference.kept << "; stepped held " << run.held
                           << " kept " << run.kept
    );

    REQUIRE(reference.plastic > 0.0f);
    REQUIRE(run.diverged == 0u);
    CHECK(run.plastic > 0.0f);
    CHECK(run.linearReads == 0u);
    CHECK(std::abs(run.held - reference.loaded) <= 0.05f * reference.loaded);
    CHECK(std::abs(run.kept - reference.kept) <= 0.05f * reference.kept);
  }
}

TEST_CASE(
  "a capped plastic substep ends no worse than its linear iterate",
  "[.][plasticcapped]"
) {
  constexpr float SOFT_HARDENING = YOUNGS / 100.0f;

  const std::vector<PhysicalMaterial> materials =
    barMaterials(YIELD, SOFT_HARDENING);
  const Bar bar = buildBar(COARSE_VOXEL, materials, VolumeElementType::Hex);

  talus::deformable::Settings settings = barStep(bar.element);

  settings.substeps = 1u;
  settings.plasticPasses = 8u;

  talus::deformable::Settings linear = settings;

  linear.plasticPasses = 2u;

  const auto advance = [&](
                         DeformableState& moved,
                         const talus::deformable::Settings& chosen,
                         talus::deformable::DeformableSolverState* solver,
                         const glm::vec3& gravity
                       ) {
    talus::deformable::StepReport report;

    talus::deformable::step(
      *bar.mesh,
      moved,
      materials,
      chosen,
      gravity,
      TIME_STEP,
      {},
      {},
      solver,
      report
    );

    return report;
  };

  auto solver = std::make_unique<talus::deformable::DeformableSolverState>();

  DeformableState state =
    talus::deformable::restState(*bar.mesh, talus::objectmodel::Pose{});

  uint32_t capped = 0u;
  uint64_t linearReads = 0u;

  for (uint32_t frame = 1u; frame <= CYCLE_RAMP + CYCLE_HOLD; ++frame) {
    const float share = std::min(static_cast<float>(frame) / CYCLE_RAMP, 1.0f);
    const glm::vec3 gravity(share * bodyPull(), 0.0f, 0.0f);
    const DeformableState before = state;
    const talus::deformable::StepReport stepped =
      advance(state, settings, solver.get(), gravity);

    linearReads += stepped.plasticLinearReads;

    if (stepped.plasticCapped == 0u) {
      continue;
    }

    ++capped;

    DeformableState full = before;
    DeformableState once = before;

    auto fullSolver =
      std::make_unique<talus::deformable::DeformableSolverState>();
    auto onceSolver =
      std::make_unique<talus::deformable::DeformableSolverState>();

    const talus::deformable::StepReport fullReport =
      advance(full, settings, fullSolver.get(), gravity);
    const talus::deformable::StepReport onceReport =
      advance(once, linear, onceSolver.get(), gravity);

    linearReads +=
      fullReport.plasticLinearReads + onceReport.plasticLinearReads;

    INFO(
      "frame " << frame << ": " << settings.plasticPasses << " passes capped "
               << fullReport.plasticCapped << " residual "
               << fullReport.plasticWorstResidual << " acceleration "
               << largestAcceleration(full) << "; " << linear.plasticPasses
               << " passes capped " << onceReport.plasticCapped << " residual "
               << onceReport.plasticWorstResidual
    );

    CHECK(fullReport.plasticCapped == 1u);
    CHECK(onceReport.plasticCapped == 1u);
    CHECK(fullReport.plasticWorstResidual <= onceReport.plasticWorstResidual);
    CHECK(largestAcceleration(full) > 0.0f);
  }

  std::cout << "plasticcapped: hex H " << SOFT_HARDENING << " substeps "
            << settings.substeps << " cg " << settings.cgIterations
            << " factor budget " << settings.factorBudget << " tolerance "
            << settings.tolerance << " spectral radius "
            << settings.spectralRadius << " plastic passes "
            << settings.plasticPasses << " against " << linear.plasticPasses
            << " factored; capped substeps " << capped << " of "
            << CYCLE_RAMP + CYCLE_HOLD << " linear reads " << linearReads
            << '\n';

  REQUIRE(capped != 0u);
  CHECK(linearReads == 0u);
}

TEST_CASE(
  "a plastic bar's first released step unloads along the elastic stiffness",
  "[plasticity]"
) {
  for (const VolumeElementType element : ELEMENTS) {
    std::array<float, 2> moved{};
    float yielded = 0.0f;
    size_t points = 0u;
    size_t changed = 0u;

    for (size_t yielding = 0u; yielding != moved.size(); ++yielding) {
      const std::vector<PhysicalMaterial> materials = barMaterials(
        yielding != 0u ? YIELD : std::numeric_limits<float>::infinity()
      );
      const Bar bar = buildBar(COARSE_VOXEL, materials, element);

      DeformableState state = talus::deformable::solveEquilibrium(
        *bar.mesh,
        materials,
        talus::objectmodel::Pose{},
        barSolve(bar.element),
        glm::vec3(0.0f),
        scaled(bar.pull, PAST_STRESS * AREA)
      );

      const float loaded = gaugeStrain(bar, state);
      const std::vector<talus::objectmodel::PlasticPoint> before =
        state.plastic;

      talus::deformable::StepReport report;

      talus::deformable::step(
        *bar.mesh,
        state,
        materials,
        barStep(bar.element),
        glm::vec3(0.0f),
        TIME_STEP,
        {},
        {},
        nullptr,
        report
      );

      REQUIRE(allFinite(state));

      moved[yielding] = gaugeStrain(bar, state) - loaded;

      if (yielding == 0u) {
        continue;
      }

      REQUIRE(state.plastic.size() == before.size());

      yielded = largestEquivalent(state);
      points = before.size();

      for (size_t point = 0u; point != points; ++point) {
        const bool same =
          state.plastic[point].equivalent == before[point].equivalent
          && state.plastic[point].strain == before[point].strain;

        changed += same ? 0u : 1u;
      }
    }

    INFO(
      elementName(element) << ": elastic moved " << moved[0]
                           << ", plastic moved " << moved[1] << "; " << changed
                           << " of " << points << " plastic points changed"
    );

    REQUIRE(yielded > 0.0f);
    REQUIRE(moved[0] < 0.0f);
    CHECK(changed == 0u);
    CHECK(std::abs(moved[1] - moved[0]) <= 0.01f * std::abs(moved[0]));
  }
}

TEST_CASE(
  "a free plastic bar spun rigidly at 5 and 10 rad/s does not flow",
  "[plasticity]"
) {
  spinFreeBar(60u);
}

TEST_CASE(
  "a free plastic bar spun rigidly at 5 and 10 rad/s does not flow over 300 "
  "frames",
  "[.][plasticspinfull]"
) {
  spinFreeBar(300u);
}

namespace {

namespace spun {

constexpr float GAUSS = 0.5773502691896258f;
constexpr double BAND = 0.05;
constexpr double ENERGY_RISE = 4.5e-4;
constexpr uint32_t FRAMES = 300u;
constexpr float HELD_RATE = 20.0f;

struct Row {
  VolumeElementType element = VolumeElementType::Hex;
  float rate = 0.0f;
  double tight = 0.0;
};

constexpr std::array<Row, 3> ROWS{
  Row{VolumeElementType::Hex, 20.0f, 6.98937e-5},
  Row{VolumeElementType::Tet, 20.0f, 6.97767e-5},
  Row{VolumeElementType::Hex, 30.0f, 1.74551e-4}
};

double principal(const glm::mat3& strain) {
  const double a00 = strain[0][0];
  const double a11 = strain[1][1];
  const double a22 = strain[2][2];
  const double a01 = 0.5 * (double(strain[1][0]) + double(strain[0][1]));
  const double a02 = 0.5 * (double(strain[2][0]) + double(strain[0][2]));
  const double a12 = 0.5 * (double(strain[2][1]) + double(strain[1][2]));
  const double off = a01 * a01 + a02 * a02 + a12 * a12;
  const double mean = (a00 + a11 + a22) / 3.0;
  const double spread = (a00 - mean) * (a00 - mean)
                        + (a11 - mean) * (a11 - mean)
                        + (a22 - mean) * (a22 - mean) + 2.0 * off;

  if (spread <= 1.0e-60) {
    return mean;
  }

  const double scale = std::sqrt(spread / 6.0);
  const double b00 = (a00 - mean) / scale;
  const double b11 = (a11 - mean) / scale;
  const double b22 = (a22 - mean) / scale;
  const double b01 = a01 / scale;
  const double b02 = a02 / scale;
  const double b12 = a12 / scale;
  const double half =
    0.5
    * (b00 * (b11 * b22 - b12 * b12) - b01 * (b01 * b22 - b12 * b02) + b02 * (b01 * b12 - b11 * b02));
  const double angle = std::acos(std::clamp(half, -1.0, 1.0)) / 3.0;

  return mean + 2.0 * scale * std::cos(angle);
}

std::array<glm::vec3, 8> hexGradient(
  const glm::vec3& at,
  const glm::vec3& halfSize
) {
  std::array<glm::vec3, 8> gradient{};

  for (uint32_t corner = 0u; corner != 8u; ++corner) {
    const glm::vec3 sign(
      (corner & 1u) != 0u ? 1.0f : -1.0f,
      (corner & 2u) != 0u ? 1.0f : -1.0f,
      (corner & 4u) != 0u ? 1.0f : -1.0f
    );

    gradient[corner] = glm::vec3(
      0.125f * sign.x * (1.0f + sign.y * at.y) * (1.0f + sign.z * at.z)
        / halfSize.x,
      0.125f * sign.y * (1.0f + sign.x * at.x) * (1.0f + sign.z * at.z)
        / halfSize.y,
      0.125f * sign.z * (1.0f + sign.x * at.x) * (1.0f + sign.y * at.y)
        / halfSize.z
    );
  }

  return gradient;
}

double largestPrincipal(
  const talus::objectmodel::PreparedMesh& built,
  const DeformableState& moved
) {
  double largest = -std::numeric_limits<double>::infinity();

  for (const talus::objectmodel::PreparedHex& hex : built.hexes) {
    const std::array<glm::vec3, 8> centreGradient =
      hexGradient(glm::vec3(0.0f), hex.halfExtents);

    glm::mat3 gradientOf(0.0f);
    glm::vec3 deformedCentre(0.0f);
    glm::vec3 restCentre(0.0f);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      gradientOf += glm::outerProduct(
        moved.position[hex.nodes[corner]],
        centreGradient[corner]
      );
      deformedCentre += 0.125f * moved.position[hex.nodes[corner]];
      restCentre += 0.125f * built.restPositions[hex.nodes[corner]];
    }

    const glm::mat3 inverse =
      glm::transpose(talus::deformable::polarRotation(gradientOf));

    std::array<glm::vec3, 8> offset{};
    glm::mat3 centreDisplacement(0.0f);

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      offset[corner] =
        inverse * (moved.position[hex.nodes[corner]] - deformedCentre)
        - (built.restPositions[hex.nodes[corner]] - restCentre);
      centreDisplacement +=
        glm::outerProduct(offset[corner], centreGradient[corner]);
    }

    const float volumetric =
      (centreDisplacement[0][0] + centreDisplacement[1][1]
       + centreDisplacement[2][2])
      / 3.0f;

    for (uint32_t point = 0u; point != 8u; ++point) {
      const glm::vec3 at = GAUSS
                           * glm::vec3(
                             (point & 1u) != 0u ? 1.0f : -1.0f,
                             (point & 2u) != 0u ? 1.0f : -1.0f,
                             (point & 4u) != 0u ? 1.0f : -1.0f
                           );
      const std::array<glm::vec3, 8> pointGradient =
        hexGradient(at, hex.halfExtents);

      glm::mat3 displacement(0.0f);

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        displacement +=
          glm::outerProduct(offset[corner], pointGradient[corner]);
      }

      const glm::mat3 symmetric =
        0.5f * (displacement + glm::transpose(displacement));
      const float trace = symmetric[0][0] + symmetric[1][1] + symmetric[2][2];
      const glm::mat3 strain = symmetric - (trace / 3.0f) * glm::mat3(1.0f)
                               + volumetric * glm::mat3(1.0f);

      largest = std::max(largest, principal(strain));
    }
  }

  for (const talus::objectmodel::PreparedTet& tet : built.tets) {
    std::array<glm::vec3, 4> deformed{};
    std::array<glm::vec3, 4> rest{};

    for (int corner = 0; corner != 4; ++corner) {
      deformed[static_cast<size_t>(corner)] = moved.position[tet.nodes[corner]];
      rest[static_cast<size_t>(corner)] =
        built.restPositions[tet.nodes[corner]];
    }

    const glm::mat3 deformation = glm::mat3(
                                    deformed[1] - deformed[0],
                                    deformed[2] - deformed[0],
                                    deformed[3] - deformed[0]
                                  )
                                  * tet.restInverse;
    const glm::mat3 inverse =
      glm::transpose(talus::deformable::polarRotation(deformation));
    const std::array<glm::vec3, 4> gradient =
      talus::deformable::shapeGradients(tet.restInverse);
    const glm::vec3 deformedCentre =
      0.25f * (deformed[0] + deformed[1] + deformed[2] + deformed[3]);
    const glm::vec3 restCentre =
      0.25f * (rest[0] + rest[1] + rest[2] + rest[3]);

    glm::mat3 displacement(0.0f);

    for (size_t corner = 0u; corner != 4u; ++corner) {
      displacement += glm::outerProduct(
        inverse * (deformed[corner] - deformedCentre)
          - (rest[corner] - restCentre),
        gradient[corner]
      );
    }

    largest = std::max(
      largest,
      principal(0.5f * (displacement + glm::transpose(displacement)))
    );
  }

  return largest;
}

double strainEnergy(
  const talus::objectmodel::PreparedMesh& built,
  const DeformableState& moved,
  std::span<const PhysicalMaterial> materials
) {
  const auto elasticOf = [&](uint32_t lane) {
    return talus::deformable::ElasticParams{
      materials[lane].youngsModulus,
      materials[lane].poissonsRatio,
      materials[lane].density
    };
  };
  const auto plasticOf = [&](uint32_t lane) {
    return talus::deformable::PlasticParams{
      materials[lane].yieldStress,
      materials[lane].hardeningModulus
    };
  };
  const bool sized =
    moved.plastic.size() == 8u * built.hexes.size() + built.tets.size();

  double energy = 0.0;

  for (size_t element = 0u; element != built.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = built.hexes[element];

    std::array<glm::vec3, 8> deformed{};
    std::array<glm::vec3, 8> rest{};
    std::array<talus::objectmodel::PlasticPoint, 8> committed{};

    for (size_t corner = 0u; corner != 8u; ++corner) {
      deformed[corner] = moved.position[hex.nodes[corner]];
      rest[corner] = built.restPositions[hex.nodes[corner]];

      if (sized) {
        committed[corner] = moved.plastic[8u * element + corner];
      }
    }

    energy += talus::deformable::hexPointTrial(
                deformed,
                rest,
                hex,
                elasticOf(hex.material),
                plasticOf(hex.material),
                committed
    )
                .energy;
  }

  for (size_t element = 0u; element != built.tets.size(); ++element) {
    const talus::objectmodel::PreparedTet& tet = built.tets[element];

    std::array<glm::vec3, 4> deformed{};
    std::array<glm::vec3, 4> rest{};

    for (int corner = 0; corner != 4; ++corner) {
      deformed[static_cast<size_t>(corner)] = moved.position[tet.nodes[corner]];
      rest[static_cast<size_t>(corner)] =
        built.restPositions[tet.nodes[corner]];
    }

    const talus::objectmodel::PlasticPoint committed =
      sized ? moved.plastic[8u * built.hexes.size() + element]
            : talus::objectmodel::PlasticPoint{};

    energy += talus::deformable::tetPointTrial(
                deformed,
                rest,
                tet,
                elasticOf(tet.material),
                plasticOf(tet.material),
                committed
    )
                .energy;
  }

  return energy;
}

double kineticEnergy(
  const talus::objectmodel::PreparedMesh& built,
  const DeformableState& moved
) {
  double energy = 0.0;

  for (size_t node = 0u; node != moved.velocity.size(); ++node) {
    energy += 0.5 * double(built.nodeMass[node])
              * double(glm::dot(moved.velocity[node], moved.velocity[node]));
  }

  return energy;
}

DeformableState spunStart(
  const talus::objectmodel::PreparedMesh& mesh,
  const glm::vec3& spin,
  bool steady
) {
  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  glm::vec3 centre(0.0f);
  float mass = 0.0f;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    centre += mesh.nodeMass[node] * state.position[node];
    mass += mesh.nodeMass[node];
  }

  centre /= mass;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 lever = state.position[node] - centre;

    state.velocity[node] = glm::cross(spin, lever);
    state.acceleration[node] =
      steady ? glm::cross(spin, glm::cross(spin, lever)) : glm::vec3(0.0f);
  }

  return state;
}

}

}

TEST_CASE(
  "a free plastic bar spun at 20 and 30 rad/s from zero acceleration stays "
  "elastic",
  "[.][plasticspinelastic]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const spun::Row& row : spun::ROWS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, row.element);

    REQUIRE(mesh != nullptr);

    talus::deformable::Settings settings = barStep(row.element);

    settings.plasticPasses = 8u;

    auto solver = std::make_unique<talus::deformable::DeformableSolverState>();

    DeformableState state =
      spun::spunStart(*mesh, glm::vec3(0.0f, 0.0f, row.rate), false);

    const double before = spun::kineticEnergy(*mesh, state)
                          + spun::strainEnergy(*mesh, state, materials);

    talus::deformable::StepReport report;

    talus::deformable::step(
      *mesh,
      state,
      materials,
      settings,
      glm::vec3(0.0f),
      TIME_STEP,
      {},
      {},
      solver.get(),
      report
    );

    const double principal = spun::largestPrincipal(*mesh, state);
    const double after = spun::kineticEnergy(*mesh, state)
                         + spun::strainEnergy(*mesh, state, materials);
    const double rise = (after - before) / before;

    std::cout << "spin elastic: " << elementName(row.element) << " rate "
              << row.rate << " rad/s zero start step 1: plasticCapped "
              << report.plasticCapped << " plasticLinearReads "
              << report.plasticLinearReads << " plasticHandoffs "
              << report.plasticHandoffs << " plasticFallbacks "
              << report.plasticFallbacks << " plasticFallbackRejections "
              << report.plasticFallbackRejections << " plasticEvaluations "
              << report.plasticEvaluations << " plasticSolves "
              << report.plasticSolves << " largest equivalent "
              << largestEquivalent(state) << " principal " << principal
              << " tight " << row.tight << " principal over tight "
              << principal / row.tight << " energy before " << before
              << " after " << after << " rise " << rise << '\n';

    INFO(
      elementName(row.element)
      << " rate " << row.rate << " rad/s: capped " << report.plasticCapped
      << ", fallbacks " << report.plasticFallbacks << ", largest equivalent "
      << largestEquivalent(state) << ", principal " << principal
      << " against tight " << row.tight << ", energy rise " << rise
    );

    CHECK(largestEquivalent(state) == 0.0f);
    CHECK(report.plasticCapped == 0u);
    CHECK(report.plasticFallbacks == 1u);
    CHECK(std::abs(principal - row.tight) <= spun::BAND * row.tight);

    if (row.rate == spun::HELD_RATE) {
      CHECK(rise <= spun::ENERGY_RISE);
    }
  }

  for (const VolumeElementType element : ELEMENTS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, element);

    REQUIRE(mesh != nullptr);

    for (const bool steady : {false, true}) {
      const talus::deformable::Settings settings = barStep(element);

      auto solver =
        std::make_unique<talus::deformable::DeformableSolverState>();

      DeformableState state =
        spun::spunStart(*mesh, glm::vec3(0.0f, 0.0f, spun::HELD_RATE), steady);

      uint32_t firstFlow = 0u;

      for (uint32_t frame = 0u; frame != spun::FRAMES && firstFlow == 0u;
           ++frame) {
        talus::deformable::StepReport report;

        talus::deformable::step(
          *mesh,
          state,
          materials,
          settings,
          glm::vec3(0.0f),
          TIME_STEP,
          {},
          {},
          solver.get(),
          report
        );

        if (largestEquivalent(state) > 0.0f) {
          firstFlow = frame + 1u;
        }
      }

      std::cout << "spin elastic " << spun::FRAMES
                << " frames: " << elementName(element) << " rate "
                << spun::HELD_RATE << " rad/s start "
                << (steady ? "steady spin" : "zero acceleration")
                << ": first flow frame " << firstFlow << ", largest equivalent "
                << largestEquivalent(state) << '\n';

      INFO(
        elementName(element)
        << " rate " << spun::HELD_RATE << " rad/s start "
        << (steady ? "steady spin" : "zero acceleration")
        << ": first flow frame " << firstFlow << ", largest equivalent "
        << largestEquivalent(state)
      );

      CHECK(firstFlow == 0u);
    }
  }
}

namespace {

namespace impact {

constexpr float TILT = 0.35f;
constexpr float SPIN = 4.0f;
constexpr float CLEARANCE = 0.1f;
constexpr uint32_t FRAMES = 600u;
constexpr uint32_t TRACE = 25u;
constexpr double BAND = 0.1;

enum class Band : uint8_t {
  Held,
  Unquoted,
  BelowYield
};

constexpr const char* TIGHT_REDERIVED =
  "contact-relative-close lane 1 tight on 1e94548";

struct Row {
  VolumeElementType element = VolumeElementType::Hex;
  float speed = 0.0f;
  Band band = Band::BelowYield;
  double tightEnd = 0.0;
  double tightSummed = 0.0;
  const char* tightSource = "";
};

constexpr std::array<Row, 8> ROWS{
  Row{
    VolumeElementType::Hex,
    0.75f,
    Band::Held,
    2.48999713e-4,
    4.93885891e-4,
    TIGHT_REDERIVED
  },
  Row{
    VolumeElementType::Hex,
    3.0f,
    Band::Held,
    7.62368087e-4,
    3.6999743e-3,
    TIGHT_REDERIVED
  },
  Row{
    VolumeElementType::Tet,
    4.0f,
    Band::Held,
    1.23234629e-3,
    1.12125934e-2,
    TIGHT_REDERIVED
  },
  Row{
    VolumeElementType::Tet,
    8.0f,
    Band::Held,
    5.9895328e-4,
    2.64709596e-3,
    TIGHT_REDERIVED
  },
  Row{
    VolumeElementType::Hex,
    8.0f,
    Band::Held,
    2.10550125e-3,
    2.77583262e-2,
    TIGHT_REDERIVED
  },
  Row{VolumeElementType::Hex, 4.0f, Band::Unquoted},
  Row{VolumeElementType::Hex, 0.5f},
  Row{VolumeElementType::Tet, 1.0f}
};

const char* bandName(Band band) {
  switch (band) {
    case Band::Held:
      return "held";
    case Band::Unquoted:
      return "unquoted";
    case Band::BelowYield:
      return "below yield";
  }

  return "unknown";
}

struct Fallback {
  uint32_t frame = 0u;
  double principal = 0.0;
};

struct Impact {
  uint32_t contactFrame = 0u;
  uint32_t contactEvaluations = 0u;
  size_t contactCount = 0u;
  float beforeContact = 0.0f;
  float atContact = 0.0f;
  float ended = 0.0f;
  double spinBefore = 0.0;
  double spinAfter = 0.0;
  float restWindowSpeed = 0.0f;
  uint32_t restFrame = 0u;
  uint32_t firstNonFinite = 0u;
  double endHeight = 0.0;
  double summed = 0.0;
  uint64_t capped = 0u;
  uint64_t fallbacks = 0u;
  uint64_t rejections = 0u;
  std::vector<Fallback> fallbackSteps;
  std::string trace;
};

double spinAbout(
  const talus::objectmodel::PreparedMesh& mesh,
  const DeformableState& state
) {
  const glm::dvec3 centre = landingbar::landingCentre(mesh, state);

  double momentum = 0.0;
  double inertia = 0.0;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::dvec3 arm = glm::dvec3(state.position[node]) - centre;
    const double held = static_cast<double>(mesh.nodeMass[node]);

    momentum += held * glm::cross(arm, glm::dvec3(state.velocity[node])).z;
    inertia += held * (arm.x * arm.x + arm.y * arm.y);
  }

  return momentum / inertia;
}

DeformableState dropped(
  const talus::objectmodel::PreparedMesh& mesh,
  float speed,
  float tilt
) {
  talus::objectmodel::Pose pose;

  pose.rotation = glm::angleAxis(-tilt, glm::vec3(0.0f, 0.0f, 1.0f));

  DeformableState state = talus::deformable::restState(mesh, pose);

  float lowest = std::numeric_limits<float>::infinity();

  for (const glm::vec3& at : state.position) {
    lowest = std::min(lowest, at.y);
  }

  const float lift = CLEARANCE - lowest;

  for (glm::vec3& at : state.position) {
    at.y += lift;
  }

  const glm::vec3 centre(landingbar::landingCentre(mesh, state));
  const glm::vec3 spin(0.0f, 0.0f, -SPIN);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    state.velocity[node] = glm::vec3(0.0f, -speed, 0.0f)
                           + glm::cross(spin, state.position[node] - centre);
    state.acceleration[node] = glm::vec3(0.0f);
  }

  return state;
}

Impact impactBar(
  const talus::objectmodel::PreparedMesh& mesh,
  const std::vector<PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  float speed
) {
  const talus::field::SampledSdf slab = landingbar::landingSlab();
  const talus::deformable::RigidPartner partner =
    landingbar::landingPartner(materials);

  DeformableState state = dropped(mesh, speed, TILT);

  auto solver = std::make_unique<talus::deformable::DeformableSolverState>();
  std::vector<talus::deformable::RigidPartner> partners;
  std::ostringstream trace;

  Impact run;
  size_t previous = 0u;

  for (uint32_t frame = 0u; frame != FRAMES; ++frame) {
    const std::vector<talus::deformable::SurfaceContact> contacts =
      landingbar::landingContacts(mesh, state, materials, slab);

    const bool contactStep = !contacts.empty() && run.contactFrame == 0u;

    if (contactStep) {
      run.beforeContact = largestEquivalent(state);
      run.spinBefore = spinAbout(mesh, state);
    }

    partners.assign(1u, partner);

    talus::deformable::StepReport report;

    landingbar::landingStep(
      mesh,
      state,
      materials,
      settings,
      contacts,
      partners,
      solver.get(),
      report
    );

    run.capped += report.plasticCapped;
    run.fallbacks += report.plasticFallbacks;
    run.rejections += report.plasticFallbackRejections;

    if (report.plasticFallbacks > 0u && (!contacts.empty() || previous != 0u)) {
      run.fallbackSteps.push_back(
        Fallback{frame + 1u, spun::largestPrincipal(mesh, state)}
      );
    }

    previous = contacts.size();

    if (contactStep) {
      run.contactFrame = frame + 1u;
      run.contactEvaluations = report.plasticEvaluations;
      run.contactCount = contacts.size();
      run.atContact = largestEquivalent(state);
      run.spinAfter = spinAbout(mesh, state);
    }

    float fastest = 0.0f;

    for (const glm::vec3& moving : state.velocity) {
      const float speedOf = glm::length(moving);

      fastest = std::isfinite(speedOf) ? std::max(fastest, speedOf)
                                       : std::numeric_limits<float>::infinity();
    }

    if (!allFinite(state) && run.firstNonFinite == 0u) {
      run.firstNonFinite = frame + 1u;
    }

    if (!(fastest < landingbar::LANDING_REST_SPEED)) {
      run.restFrame = frame + 2u;
    }

    if (frame + landingbar::LANDING_REST_WINDOW >= FRAMES) {
      run.restWindowSpeed = std::max(run.restWindowSpeed, fastest);
    }

    if ((frame + 1u) % TRACE == 0u) {
      trace << " f" << frame + 1u << ' ' << largestEquivalent(state) << ' '
            << fastest << ' ' << landingbar::landingCentre(mesh, state).y;
    }
  }

  run.trace = trace.str();

  run.ended = largestEquivalent(state);
  run.endHeight = landingbar::landingCentre(mesh, state).y;

  for (const talus::objectmodel::PlasticPoint& point : state.plastic) {
    run.summed += static_cast<double>(point.equivalent);
  }

  return run;
}

}

}

TEST_CASE(
  "a plastic bar dropped obliquely on a field slab comes to rest with finite "
  "plastic strain",
  "[.][plasticimpact]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);
  const double surface = static_cast<double>(
    landingbar::LANDING_SLAB_CENTRE + landingbar::LANDING_SLAB_HALF_HEIGHT
  );

  size_t measuredSteps = 0u;

  for (const impact::Row& row : impact::ROWS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, row.element);

    REQUIRE(mesh != nullptr);

    const talus::deformable::Settings settings = barStep(row.element);
    const impact::Impact run =
      impact::impactBar(*mesh, materials, settings, row.speed);

    const bool yielding = row.band != impact::Band::BelowYield;
    const bool banded = row.band == impact::Band::Held;
    const bool contact = run.contactFrame != 0u && run.contactEvaluations > 0u
                         && run.beforeContact == 0.0f;
    const bool finite = run.firstNonFinite == 0u && std::isfinite(run.ended);
    const bool atRest = run.restWindowSpeed < landingbar::LANDING_REST_SPEED;
    const bool above = run.endHeight > surface;
    const double endOff =
      banded ? static_cast<double>(run.ended) / row.tightEnd - 1.0 : 0.0;
    const double summedOff = banded ? run.summed / row.tightSummed - 1.0 : 0.0;
    const bool endHeld = std::abs(endOff) <= impact::BAND;
    const bool summedHeld = std::abs(summedOff) <= impact::BAND;

    measuredSteps += run.fallbackSteps.size();

    std::cout << "impact: " << elementName(row.element) << " tilt "
              << impact::TILT << " rad about z, spin " << -impact::SPIN
              << " rad/s about z, speed " << row.speed
              << " m/s down, clearance " << impact::CLEARANCE << " m, substeps "
              << settings.substeps << " spectralRadius "
              << settings.spectralRadius << " plasticPasses "
              << settings.plasticPasses << " frames " << impact::FRAMES
              << ": band " << impact::bandName(row.band) << " contact frame "
              << run.contactFrame << " contacts " << run.contactCount
              << " evaluations " << run.contactEvaluations << " spin before "
              << run.spinBefore << " after " << run.spinAfter
              << " equivalent before " << run.beforeContact << " at contact "
              << run.atContact << " end " << run.ended << " rest frame "
              << run.restFrame << " rest window speed " << run.restWindowSpeed
              << " end centre y " << run.endHeight << " summed plastic "
              << run.summed << " first non-finite " << run.firstNonFinite
              << " plasticCapped " << run.capped << " plasticFallbacks "
              << run.fallbacks << " plasticFallbackRejections "
              << run.rejections << '\n';

    if (banded) {
      std::cout << "impact against tight: " << elementName(row.element)
                << " speed " << row.speed << " band "
                << impact::bandName(row.band) << ": end " << run.ended
                << " tight " << row.tightEnd << " off " << endOff
                << " threshold " << impact::BAND << " within " << endHeld
                << ", summed " << run.summed << " tight " << row.tightSummed
                << " off " << summedOff << " threshold " << impact::BAND
                << " within " << summedHeld << ", tight from "
                << row.tightSource << '\n';
    } else if (yielding) {
      std::cout << "impact against tight: " << elementName(row.element)
                << " speed " << row.speed << " band "
                << impact::bandName(row.band) << ": end " << run.ended
                << ", summed " << run.summed << ", no tight reference quoted\n";
    }

    std::cout << "impact trace: " << elementName(row.element) << " speed "
              << row.speed << " frame equivalent fastest centreY" << run.trace
              << '\n';

    for (const impact::Fallback& step : run.fallbackSteps) {
      std::cout << "impact fallback: " << elementName(row.element) << " speed "
                << row.speed << " frame " << step.frame << " principal "
                << step.principal << '\n';
    }

    std::cout << "impact checks: " << elementName(row.element) << " speed "
              << row.speed << " band " << impact::bandName(row.band)
              << ": contact " << contact << " finite " << finite << " at rest "
              << atRest << " above slab " << above << " fallback steps "
              << run.fallbackSteps.size() << " no rejections "
              << (run.rejections == 0u) << " end within " << endHeld
              << " summed within " << summedHeld << " end zero "
              << (run.ended == 0.0f) << '\n';

    INFO(
      elementName(row.element)
      << " speed " << row.speed << " m/s, band " << impact::bandName(row.band)
      << ": contact frame " << run.contactFrame << ", equivalent at contact "
      << run.atContact << ", end " << run.ended << " against tight "
      << row.tightEnd << ", summed " << run.summed << " against tight "
      << row.tightSummed << ", rest window speed " << run.restWindowSpeed
      << ", end centre y " << run.endHeight << ", fallbacks " << run.fallbacks
      << ", rejections " << run.rejections
    );

    CHECK(run.contactFrame != 0u);
    CHECK(run.contactEvaluations > 0u);
    CHECK(run.beforeContact == 0.0f);
    CHECK(run.firstNonFinite == 0u);
    CHECK(std::isfinite(run.ended));
    CHECK(run.restWindowSpeed < landingbar::LANDING_REST_SPEED);
    CHECK(run.endHeight > surface);

    if (yielding) {
      CHECK(run.rejections == 0u);
    } else {
      CHECK(run.ended == 0.0f);
    }

    if (banded) {
      CHECK(endHeld);
      CHECK(summedHeld);
    }

    CHECK(run.fallbackSteps.empty());
  }

  std::cout << "impact fallback steps: measured " << measuredSteps << '\n';
}

namespace {

namespace yieldingcontact {

constexpr double BAND = 0.1;
constexpr float CLEARANCE = 1.0e-3f;
constexpr uint32_t TRACE = 20u;
constexpr const char* TIGHT_SOURCE =
  "contact-relative-close lane 1 tight on 1e94548";
constexpr const char* MAIN_SOURCE =
  "contact-relative-close lane 1 main on 1e94548";
constexpr double CLOSE_INCREMENT = 0.01;
constexpr double CLOSE_PRINCIPAL = 0.1;
constexpr const char* CENSUS_SOURCE = "contact-relative-close-b lane-2 census";

struct Close {
  uint32_t frame = 0u;
  double replayedIncrement = 0.0;
  double replayedPrincipal = 0.0;
  double freshIncrement = 0.0;
  double freshPrincipal = 0.0;
};

constexpr std::array<Close, 133> F_CLOSES{
  Close{2u, 1.22627844e-3, 5.74165908e-5, 1.22799324e-3, 5.75840351e-5},
  Close{3u, 8.97948482e-4, 7.64370519e-5, 8.97428701e-4, 7.66827807e-5},
  Close{4u, 5.16186867e-4, 8.32630368e-5, 5.16290394e-4, 8.31934952e-5},
  Close{5u, 1.25070546e-4, 8.61495185e-5, 1.27041316e-4, 8.62826319e-5},
  Close{6u, 0.0, 7.63477313e-5, 0.0, 7.6208747e-5},
  Close{7u, 3.49961545e-6, 7.7786018e-5, 3.96144424e-6, 7.75040633e-5},
  Close{8u, 4.29929514e-8, 7.967748e-5, 4.87284524e-8, 7.94174889e-5},
  Close{9u, 0.0, 7.96277363e-5, 0.0, 7.97916797e-5},
  Close{10u, 1.5173157e-7, 8.03923245e-5, 1.5854414e-7, 8.01035699e-5},
  Close{11u, 5.95206302e-8, 7.97806038e-5, 4.68400287e-8, 7.99832492e-5},
  Close{12u, 2.33636797e-7, 8.01043645e-5, 1.68322742e-7, 7.99307074e-5},
  Close{13u, 2.08350978e-6, 8.07303004e-5, 2.02600107e-6, 8.05444508e-5},
  Close{14u, 1.56816554e-7, 8.10416434e-5, 3.98671432e-7, 8.09986993e-5},
  Close{15u, 4.70590521e-7, 8.06219908e-5, 4.37523795e-7, 8.05556711e-5},
  Close{16u, 6.05182874e-7, 8.03301815e-5, 6.95536852e-7, 8.03602061e-5},
  Close{17u, 1.73529167e-5, 7.83313305e-5, 1.80598666e-5, 7.82463447e-5},
  Close{18u, 1.5377536e-7, 7.53787083e-5, 3.69782697e-7, 7.53809835e-5},
  Close{19u, 6.31182672e-7, 8.10520006e-5, 8.13277296e-7, 8.10737792e-5},
  Close{20u, 4.86889323e-6, 8.23611493e-5, 5.02098225e-6, 8.22149022e-5},
  Close{21u, 6.60518893e-8, 8.10233619e-5, 4.65475267e-8, 8.07291699e-5},
  Close{22u, 7.07873656e-6, 7.96629643e-5, 7.55774283e-6, 7.93680526e-5},
  Close{23u, 2.14120007e-7, 7.78353851e-5, 2.9434085e-7, 7.78351606e-5},
  Close{24u, 7.72941945e-7, 8.14525152e-5, 7.52492014e-7, 8.12478236e-5},
  Close{25u, 1.46800979e-5, 8.45584684e-5, 1.37533083e-5, 8.45720023e-5},
  Close{26u, 1.85215868e-6, 8.01362971e-5, 2.10575231e-6, 8.00056127e-5},
  Close{27u, 1.60210267e-5, 6.76398516e-5, 1.55348972e-5, 6.78829426e-5},
  Close{28u, 8.3597742e-5, 6.92416427e-5, 8.34480296e-5, 6.90209512e-5},
  Close{29u, 1.46269698e-4, 4.6795118e-5, 1.46226882e-4, 4.65902535e-5},
  Close{30u, 2.03533585e-4, 9.13488989e-5, 2.03028402e-4, 9.14111651e-5},
  Close{31u, 4.43675683e-6, 8.25108496e-5, 4.78372158e-6, 8.21388581e-5},
  Close{32u, 5.9765062e-6, 8.47268029e-5, 5.98611325e-6, 8.4575225e-5},
  Close{33u, 1.74217666e-6, 8.51250068e-5, 1.88640837e-6, 8.49529518e-5},
  Close{34u, 1.13725046e-5, 7.22479712e-5, 1.17667966e-5, 7.18844918e-5},
  Close{35u, 1.55622665e-4, 5.87106817e-5, 1.55133007e-4, 5.86780096e-5},
  Close{36u, 1.66237133e-4, 9.57432369e-5, 1.6950667e-4, 9.55234935e-5},
  Close{37u, 0.0, 6.99242847e-5, 0.0, 6.97352346e-5},
  Close{38u, 2.65021289e-5, 8.53252452e-5, 2.70866076e-5, 8.52526775e-5},
  Close{39u, 1.42933271e-5, 7.66884051e-5, 1.4679309e-5, 7.67624595e-5},
  Close{40u, 1.09621412e-5, 8.38791053e-5, 1.13080175e-5, 8.39697104e-5},
  Close{41u, 5.86517803e-6, 8.86438004e-5, 6.47301567e-6, 8.86522093e-5},
  Close{42u, 1.34586427e-5, 7.75644143e-5, 1.41625102e-5, 7.74746696e-5},
  Close{43u, 9.52461789e-6, 8.33323196e-5, 8.52301446e-6, 8.33544269e-5},
  Close{44u, 5.33318413e-6, 7.90804511e-5, 5.69478019e-6, 7.9030828e-5},
  Close{45u, 8.60794e-6, 8.52425884e-5, 8.22602021e-6, 8.52839576e-5},
  Close{46u, 1.42898617e-5, 8.92280706e-5, 1.49518613e-5, 8.92829039e-5},
  Close{47u, 9.57407754e-6, 7.92453013e-5, 9.66857028e-6, 7.89527709e-5},
  Close{48u, 1.05352726e-5, 8.52913839e-5, 1.09102363e-5, 8.50948997e-5},
  Close{49u, 4.30632127e-6, 7.94336696e-5, 4.7389193e-6, 7.94818362e-5},
  Close{50u, 7.22226628e-6, 8.69551988e-5, 7.46467475e-6, 8.70127361e-5},
  Close{51u, 8.59043013e-7, 7.91365199e-5, 8.68410382e-7, 7.90744358e-5},
  Close{52u, 5.90508701e-4, 1.05518979e-4, 5.94225537e-4, 1.05407393e-4},
  Close{53u, 1.29374126e-5, 8.41893178e-5, 1.32445058e-5, 8.44392169e-5},
  Close{54u, 0.0, 9.29915259e-5, 0.0, 9.30887431e-5},
  Close{55u, 5.13224325e-6, 9.86947043e-5, 4.36473618e-6, 9.85488973e-5},
  Close{56u, 1.68956285e-6, 9.80466767e-5, 1.82954773e-6, 9.8048676e-5},
  Close{57u, 6.19791535e-7, 9.7241569e-5, 1.64542143e-7, 9.70441181e-5},
  Close{58u, 0.0, 9.60716711e-5, 0.0, 9.60895497e-5},
  Close{59u, 0.0, 9.70210112e-5, 0.0, 9.69324786e-5},
  Close{67u, 3.46625302e-6, 9.90542545e-5, 3.41473151e-6, 9.91746156e-5},
  Close{68u, 4.9537731e-6, 9.95055684e-5, 4.73336377e-6, 9.94959387e-5},
  Close{69u, 3.97942063e-6, 9.97188626e-5, 3.8939882e-6, 9.9686094e-5},
  Close{70u, 4.72661101e-6, 1.00163588e-4, 5.11590474e-6, 1.00257356e-4},
  Close{71u, 5.97339525e-6, 1.00358217e-4, 6.45380049e-6, 1.00502929e-4},
  Close{72u, 5.51641224e-6, 1.00704277e-4, 5.16059484e-6, 1.00685461e-4},
  Close{73u, 8.51516958e-6, 1.01243199e-4, 8.69829418e-6, 1.01257374e-4},
  Close{74u, 6.24754117e-6, 1.01497952e-4, 5.66143314e-6, 1.01651448e-4},
  Close{75u, 8.86155226e-6, 1.0191406e-4, 7.73578199e-6, 1.01887546e-4},
  Close{76u, 7.64775376e-6, 1.02410129e-4, 7.55160204e-6, 1.02388356e-4},
  Close{77u, 1.02342741e-5, 1.02960429e-4, 1.04453058e-5, 1.02956957e-4},
  Close{78u, 1.3493778e-5, 1.03093612e-4, 1.27527419e-5, 1.03151574e-4},
  Close{79u, 1.16754061e-5, 1.03547863e-4, 1.24042134e-5, 1.03468153e-4},
  Close{80u, 1.5491642e-5, 1.03897348e-4, 1.63641556e-5, 1.03894706e-4},
  Close{81u, 1.6228396e-5, 1.04346104e-4, 1.59223227e-5, 1.04335071e-4},
  Close{82u, 1.76748171e-5, 1.04854974e-4, 1.68060438e-5, 1.04774294e-4},
  Close{83u, 1.81774284e-5, 1.05351532e-4, 1.95038072e-5, 1.05496232e-4},
  Close{84u, 1.81997726e-5, 1.06249965e-4, 1.82121859e-5, 1.06130107e-4},
  Close{85u, 2.3116565e-5, 1.06296342e-4, 2.39139877e-5, 1.06369491e-4},
  Close{86u, 2.45937286e-5, 1.07499821e-4, 2.66509636e-5, 1.07417028e-4},
  Close{87u, 2.33822197e-5, 1.0792753e-4, 2.67609752e-5, 1.07893003e-4},
  Close{125u, 0.0, 1.28092688e-4, 0.0, 1.27347718e-4},
  Close{126u, 0.0, 1.26325441e-4, 0.0, 1.2599391e-4},
  Close{127u, 0.0, 1.24228934e-4, 0.0, 1.23717432e-4},
  Close{128u, 0.0, 1.22630349e-4, 0.0, 1.22260874e-4},
  Close{129u, 0.0, 1.20949599e-4, 0.0, 1.20622824e-4},
  Close{130u, 0.0, 1.19467219e-4, 0.0, 1.19111052e-4},
  Close{131u, 0.0, 1.18258817e-4, 0.0, 1.17813893e-4},
  Close{132u, 0.0, 1.16802835e-4, 0.0, 1.16614657e-4},
  Close{133u, 0.0, 1.15945838e-4, 0.0, 1.15377401e-4},
  Close{134u, 0.0, 1.14487328e-4, 0.0, 1.14192558e-4},
  Close{135u, 0.0, 1.13474101e-4, 0.0, 1.13121715e-4},
  Close{136u, 0.0, 1.12824222e-4, 0.0, 1.12474758e-4},
  Close{137u, 0.0, 1.11736935e-4, 0.0, 1.11502741e-4},
  Close{138u, 0.0, 1.11153798e-4, 0.0, 1.11170685e-4},
  Close{139u, 0.0, 1.10250562e-4, 0.0, 1.10214599e-4},
  Close{140u, 0.0, 1.09603224e-4, 0.0, 1.09259892e-4},
  Close{141u, 0.0, 1.08809761e-4, 0.0, 1.08489383e-4},
  Close{142u, 0.0, 1.08887735e-4, 0.0, 1.08812971e-4},
  Close{143u, 4.88362275e-8, 1.08771348e-4, 2.80389941e-7, 1.08396076e-4},
  Close{144u, 8.58821295e-7, 1.09187031e-4, 8.25394636e-7, 1.09054747e-4},
  Close{145u, 1.7813727e-6, 1.09176019e-4, 2.07932499e-6, 1.09040378e-4},
  Close{146u, 2.73439172e-5, 1.04643124e-4, 2.65491829e-5, 1.04499671e-4},
  Close{147u, 1.24959562e-4, 1.17771756e-4, 1.26974228e-4, 1.17427157e-4},
  Close{148u, 4.23980302e-5, 1.08012317e-4, 4.38387938e-5, 1.07786424e-4},
  Close{149u, 3.67630715e-5, 1.10974186e-4, 1.67404513e-5, 1.09760981e-4},
  Close{150u, 2.62914523e-6, 1.1323842e-4, 3.44398893e-6, 1.13566769e-4},
  Close{151u, 2.6693283e-6, 1.06963985e-4, 3.80113019e-5, 1.1504667e-4},
  Close{152u, 9.51462645e-5, 1.18032042e-4, 1.18032933e-5, 1.1307975e-4},
  Close{153u, 3.58831037e-5, 1.16963278e-4, 4.83465556e-7, 1.079555e-4},
  Close{154u, 1.61088168e-5, 1.13632768e-4, 9.96569736e-6, 1.13491592e-4},
  Close{155u, 3.01808029e-6, 1.08255286e-4, 3.24653035e-6, 1.08204124e-4},
  Close{156u, 6.43485691e-7, 1.03577242e-4, 5.8360456e-7, 1.03458975e-4},
  Close{157u, 1.0157633e-6, 1.03230671e-4, 5.60879991e-7, 1.03078767e-4},
  Close{160u, 4.32955503e-8, 1.08721227e-4, 1.40759262e-7, 1.08708171e-4},
  Close{163u, 0.0, 1.06465972e-4, 0.0, 1.06446198e-4},
  Close{164u, 3.45128417e-6, 1.07835296e-4, 3.86909108e-6, 1.07960867e-4},
  Close{165u, 5.18230471e-6, 1.05329983e-4, 5.12353297e-6, 1.05195774e-4},
  Close{170u, 0.0, 8.94749959e-5, 0.0, 8.90523257e-5},
  Close{172u, 1.34622553e-6, 8.7866594e-5, 1.181176e-6, 8.79348527e-5},
  Close{174u, 2.75487903e-5, 8.40217627e-5, 2.67936906e-5, 8.49304691e-5},
  Close{177u, 1.33922863e-5, 8.14776007e-5, 1.5461359e-5, 8.17434608e-5},
  Close{181u, 1.15013768e-6, 7.96929031e-5, 7.93252866e-7, 7.98103621e-5},
  Close{186u, 0.0, 7.97400333e-5, 0.0, 9.10958266e-5},
  Close{187u, 0.0, 9.52883459e-5, 0.0, 9.44579883e-5},
  Close{188u, 0.0, 1.13932428e-4, 0.0, 1.14044923e-4},
  Close{189u, 0.0, 8.16000575e-5, 0.0, 8.16945332e-5},
  Close{190u, 0.0, 8.74240815e-5, 0.0, 8.72246755e-5},
  Close{191u, 0.0, 9.12169679e-5, 0.0, 9.13121722e-5},
  Close{193u, 0.0, 8.60287683e-5, 0.0, 8.64638655e-5},
  Close{195u, 0.0, 8.27388106e-5, 0.0, 8.27687051e-5},
  Close{196u, 0.0, 8.66752788e-5, 0.0, 8.62461856e-5},
  Close{197u, 0.0, 7.90582575e-5, 0.0, 7.92995767e-5},
  Close{198u, 0.0, 8.67114367e-5, 0.0, 8.66768368e-5},
  Close{200u, 0.0, 8.06479176e-5, 0.0, 8.05673101e-5}
};

constexpr std::array<Close, 160> A_CLOSES{
  Close{1u, 2.70572558e-4, 3.13832059e-5, 2.70572558e-4, 3.13832059e-5},
  Close{2u, 2.22552581e-3, 5.89759737e-5, 2.22853418e-3, 5.91955719e-5},
  Close{3u, 2.76338984e-3, 8.99367674e-5, 2.76451152e-3, 8.99052036e-5},
  Close{4u, 2.79656394e-3, 1.15225332e-4, 2.79798311e-3, 1.15178073e-4},
  Close{5u, 2.33924699e-3, 1.38394041e-4, 2.34901175e-3, 1.38322511e-4},
  Close{6u, 2.39133964e-3, 1.62205483e-4, 2.4064988e-3, 1.61950475e-4},
  Close{7u, 1.99521792e-3, 1.80902144e-4, 2.02078702e-3, 1.80383193e-4},
  Close{8u, 1.68037141e-3, 1.97771695e-4, 1.70406935e-3, 1.97215741e-4},
  Close{9u, 1.50983766e-3, 2.12311427e-4, 1.54492506e-3, 2.11853128e-4},
  Close{10u, 1.36976708e-3, 2.26181181e-4, 1.40183948e-3, 2.25639855e-4},
  Close{12u, 1.07775703e-3, 2.49120995e-4, 1.11738643e-3, 2.48713002e-4},
  Close{13u, 9.40816141e-4, 2.58936865e-4, 9.8246101e-4, 2.58587857e-4},
  Close{14u, 8.72716509e-4, 2.68149269e-4, 9.15153276e-4, 2.67319076e-4},
  Close{15u, 7.38575102e-4, 2.75833686e-4, 7.8501757e-4, 2.75176048e-4},
  Close{16u, 6.87260853e-4, 2.82881369e-4, 7.39496532e-4, 2.82260589e-4},
  Close{17u, 5.89166972e-4, 2.89058058e-4, 6.36136819e-4, 2.88621046e-4},
  Close{18u, 5.49510826e-4, 2.95392677e-4, 6.10673289e-4, 2.94732241e-4},
  Close{19u, 4.68090403e-4, 3.00044208e-4, 5.28487944e-4, 2.99504162e-4},
  Close{20u, 4.35608723e-4, 3.05441657e-4, 5.07735976e-4, 3.04526105e-4},
  Close{21u, 3.6628236e-4, 3.09056751e-4, 4.35581997e-4, 3.08454962e-4},
  Close{22u, 3.54180435e-4, 3.13415002e-4, 4.2605219e-4, 3.12328087e-4},
  Close{23u, 1.19593863e-4, 3.1554781e-4, 1.85619757e-4, 3.14982611e-4},
  Close{25u, 7.01434465e-6, 3.14306799e-4, 3.03002918e-5, 3.13411431e-4},
  Close{26u, 1.15762759e-6, 3.13720056e-4, 2.2844117e-5, 3.13020138e-4},
  Close{28u, 2.13944618e-5, 3.16612351e-4, 4.55359873e-5, 3.15599884e-4},
  Close{29u, 5.98672796e-6, 3.16030721e-4, 3.30980635e-5, 3.15465861e-4},
  Close{30u, 1.02037652e-6, 3.15183483e-4, 2.44172183e-5, 3.1475264e-4},
  Close{31u, 6.18331133e-5, 3.17822229e-4, 8.79200361e-5, 3.17361184e-4},
  Close{32u, 2.22759158e-5, 3.18184078e-4, 4.06943922e-5, 3.17190409e-4},
  Close{33u, 2.18881264e-5, 3.18545461e-4, 5.68516614e-5, 3.17829761e-4},
  Close{34u, 4.39846777e-5, 3.19295914e-4, 6.21451875e-5, 3.18504057e-4},
  Close{35u, 3.80074032e-5, 3.19591839e-4, 7.32600704e-5, 3.19113177e-4},
  Close{36u, 5.33427191e-5, 3.20917189e-4, 7.41096406e-5, 3.19991453e-4},
  Close{37u, 5.32658431e-5, 3.21497269e-4, 8.30277113e-5, 3.20790659e-4},
  Close{38u, 6.37045879e-5, 3.22971028e-4, 8.87086082e-5, 3.2217492e-4},
  Close{39u, 1.86219613e-5, 3.23060819e-4, 4.63341859e-5, 3.22366787e-4},
  Close{40u, 8.41127738e-5, 3.24139276e-4, 1.18077192e-4, 3.23773594e-4},
  Close{42u, 1.26515469e-4, 3.28693684e-4, 1.49214863e-4, 3.27839149e-4},
  Close{43u, 1.19418207e-4, 3.29848724e-4, 1.44705139e-4, 3.29352177e-4},
  Close{44u, 6.51335423e-5, 3.29719833e-4, 8.78984751e-5, 3.28877658e-4},
  Close{45u, 2.59773591e-5, 3.29347654e-4, 6.46968261e-5, 3.28380792e-4},
  Close{46u, 5.99934735e-5, 3.28512505e-4, 9.77954975e-5, 3.28011566e-4},
  Close{47u, 7.65503564e-4, 3.36465416e-4, 8.13892644e-4, 3.35794416e-4},
  Close{48u, 6.08739371e-4, 3.41735029e-4, 6.64356894e-4, 3.40968446e-4},
  Close{49u, 3.99028237e-4, 3.45030339e-4, 4.64813133e-4, 3.44161625e-4},
  Close{50u, 5.25897699e-5, 3.40453141e-4, 7.63217946e-5, 3.3951143e-4},
  Close{51u, 6.39962491e-6, 3.41835006e-4, 2.13316322e-5, 3.40855149e-4},
  Close{52u, 3.75265316e-5, 3.4273627e-4, 5.41640587e-5, 3.41766053e-4},
  Close{53u, 2.03259855e-4, 3.45122544e-4, 2.27940937e-4, 3.44511371e-4},
  Close{54u, 5.25868218e-5, 3.45989591e-4, 7.23029841e-5, 3.45312109e-4},
  Close{127u, 1.24759081e-5, 6.00236857e-4, 1.85133613e-5, 5.9850171e-4},
  Close{128u, 0.0, 6.01924184e-4, 5.88526655e-6, 6.00116311e-4},
  Close{129u, 0.0, 5.8246831e-4, 3.7535483e-7, 5.80909276e-4},
  Close{130u, 0.0, 5.79810267e-4, 0.0, 5.78349672e-4},
  Close{131u, 0.0, 5.81840015e-4, 9.67406231e-6, 5.96547556e-4},
  Close{132u, 0.0, 5.79713147e-4, 0.0, 5.78025092e-4},
  Close{133u, 0.0, 5.78929547e-4, 0.0, 5.77255427e-4},
  Close{134u, 2.47332719e-5, 5.95881552e-4, 3.94218914e-5, 5.94330419e-4},
  Close{135u, 1.08904351e-5, 5.95859055e-4, 3.14682238e-5, 5.94380123e-4},
  Close{136u, 1.18925745e-5, 5.95190691e-4, 2.83655809e-5, 5.93359171e-4},
  Close{137u, 1.86721445e-5, 5.93955809e-4, 4.78186867e-5, 5.92375945e-4},
  Close{138u, 1.77837064e-5, 5.93274672e-4, 4.04029506e-5, 5.91549449e-4},
  Close{139u, 0.0, 5.72008166e-4, 6.59788506e-7, 5.705568e-4},
  Close{140u, 0.0, 5.7126902e-4, 0.0, 5.70028214e-4},
  Close{141u, 2.16762674e-5, 5.87400424e-4, 2.86092982e-5, 5.85819012e-4},
  Close{142u, 6.53009065e-5, 5.91934713e-4, 1.01602119e-4, 5.90379054e-4},
  Close{143u, 4.16407584e-5, 5.9128979e-4, 6.7552679e-5, 5.89618656e-4},
  Close{144u, 1.78889495e-5, 5.91074543e-4, 5.51756511e-5, 5.89352993e-4},
  Close{145u, 9.49325658e-6, 5.89941482e-4, 4.677587e-5, 5.88274799e-4},
  Close{146u, 7.11580833e-6, 5.88531311e-4, 4.88549704e-5, 5.86956346e-4},
  Close{147u, 1.8494502e-6, 5.88038119e-4, 3.46309216e-5, 5.86431689e-4},
  Close{148u, 1.34580478e-6, 5.83766034e-4, 2.62435365e-5, 5.82202218e-4},
  Close{149u, 4.03652593e-5, 5.86188889e-4, 5.54076883e-5, 5.6731984e-4},
  Close{150u, 2.00026671e-4, 5.89816941e-4, 2.00890251e-4, 5.91498681e-4},
  Close{151u, 6.50186485e-6, 5.86347676e-4, 3.23472705e-6, 5.87647322e-4},
  Close{152u, 3.57691533e-7, 5.81326548e-4, 0.0, 5.83769872e-4},
  Close{153u, 6.68177108e-6, 5.74699637e-4, 7.46415571e-6, 5.75129821e-4},
  Close{154u, 9.00491978e-6, 5.73840467e-4, 3.02346962e-6, 5.71651586e-4},
  Close{155u, 6.21930154e-5, 5.75617837e-4, 5.28950955e-5, 5.75523554e-4},
  Close{156u, 1.3263666e-5, 5.69579293e-4, 1.1801706e-5, 5.69570059e-4},
  Close{157u, 0.0, 5.664651e-4, 0.0, 5.68297414e-4},
  Close{158u, 4.29846295e-6, 5.66431133e-4, 9.98967698e-6, 5.68950811e-4},
  Close{159u, 2.44695866e-5, 5.69492586e-4, 1.55942624e-5, 5.67656477e-4},
  Close{160u, 0.0, 5.64301405e-4, 2.02521278e-6, 5.63874394e-4},
  Close{161u, 0.0, 5.63359542e-4, 0.0, 5.6086837e-4},
  Close{163u, 0.0, 5.63244991e-4, 0.0, 5.64186305e-4},
  Close{164u, 0.0, 5.74272067e-4, 0.0, 5.73153751e-4},
  Close{165u, 3.93796654e-7, 5.81249683e-4, 0.0, 5.75367406e-4},
  Close{166u, 1.28158053e-5, 5.77584341e-4, 1.32360055e-5, 5.77589641e-4},
  Close{168u, 4.80243341e-6, 5.7339665e-4, 4.23551626e-6, 5.73355781e-4},
  Close{169u, 1.03550042e-5, 5.67541298e-4, 1.13062374e-5, 5.66775479e-4},
  Close{170u, 0.0, 5.69018778e-4, 1.94129825e-6, 5.66787603e-4},
  Close{171u, 0.0, 5.74220528e-4, 0.0, 5.72301396e-4},
  Close{172u, 0.0, 5.69655984e-4, 0.0, 5.69630367e-4},
  Close{173u, 0.0, 5.7023263e-4, 0.0, 5.699987e-4},
  Close{174u, 5.0676681e-7, 5.62087306e-4, 3.34112883e-7, 5.58078032e-4},
  Close{175u, 0.0, 5.61012121e-4, 0.0, 5.63217603e-4},
  Close{176u, 1.24843791e-6, 5.67594597e-4, 1.33943104e-7, 5.68107093e-4},
  Close{177u, 0.0, 5.65366898e-4, 0.0, 5.6104905e-4},
  Close{178u, 0.0, 5.67497832e-4, 0.0, 5.67929482e-4},
  Close{179u, 0.0, 5.60308059e-4, 0.0, 5.60622995e-4},
  Close{180u, 0.0, 5.66444209e-4, 0.0, 5.68587245e-4},
  Close{181u, 0.0, 5.66580565e-4, 0.0, 5.61501257e-4},
  Close{182u, 1.85726094e-7, 5.6400344e-4, 7.15008355e-8, 5.63247515e-4},
  Close{183u, 4.06157878e-5, 5.59588759e-4, 4.04980547e-5, 5.59316421e-4},
  Close{184u, 8.63153673e-6, 5.56044346e-4, 0.0, 5.51514952e-4},
  Close{185u, 9.52684786e-7, 5.68114012e-4, 0.0, 5.66881411e-4},
  Close{186u, 0.0, 5.70099324e-4, 0.0, 5.62365601e-4},
  Close{187u, 6.74855983e-7, 5.70446974e-4, 8.33770173e-7, 5.69618879e-4},
  Close{188u, 0.0, 5.67486001e-4, 3.51309855e-5, 5.72902787e-4},
  Close{189u, 0.0, 5.72721997e-4, 0.0, 5.73778293e-4},
  Close{190u, 6.07559559e-6, 5.72602812e-4, 0.0, 5.71559398e-4},
  Close{191u, 0.0, 5.64401911e-4, 0.0, 5.64750865e-4},
  Close{192u, 4.06944309e-7, 5.72201872e-4, 8.5451029e-7, 5.72469043e-4},
  Close{193u, 1.31806737e-5, 5.71577678e-4, 6.48069545e-6, 5.71367324e-4},
  Close{194u, 0.0, 5.62739305e-4, 0.0, 5.62853476e-4},
  Close{196u, 1.56555948e-5, 5.70195441e-4, 1.43956649e-5, 5.70175028e-4},
  Close{197u, 0.0, 5.60106163e-4, 3.27218004e-8, 5.65748958e-4},
  Close{198u, 0.0, 5.61584196e-4, 0.0, 5.60610272e-4},
  Close{199u, 9.8520286e-6, 5.57927086e-4, 3.92733909e-6, 5.59141726e-4},
  Close{201u, 0.0, 5.61403525e-4, 3.44443288e-6, 5.60437279e-4},
  Close{202u, 0.0, 5.54166446e-4, 0.0, 5.55687572e-4},
  Close{203u, 2.59711669e-7, 5.55397499e-4, 4.2288957e-7, 5.55266283e-4},
  Close{204u, 0.0, 5.64240571e-4, 0.0, 5.62785309e-4},
  Close{205u, 0.0, 5.57148019e-4, 1.78773553e-6, 5.56050612e-4},
  Close{206u, 0.0, 5.63005664e-4, 7.45357283e-7, 5.60660684e-4},
  Close{207u, 1.58160401e-6, 5.67270903e-4, 3.49961556e-6, 5.60166334e-4},
  Close{208u, 5.11450535e-5, 5.67867901e-4, 5.34194314e-5, 5.67138216e-4},
  Close{209u, 0.0, 5.60472527e-4, 0.0, 5.59501596e-4},
  Close{210u, 0.0, 5.69165586e-4, 2.52104655e-6, 5.61919426e-4},
  Close{211u, 1.79996077e-5, 5.66012436e-4, 1.62297765e-5, 5.66302668e-4},
  Close{212u, 0.0, 5.61952145e-4, 0.0, 5.6720041e-4},
  Close{213u, 0.0, 5.58134217e-4, 0.0, 5.6042256e-4},
  Close{214u, 0.0, 5.57073286e-4, 0.0, 5.52590347e-4},
  Close{215u, 1.9832366e-5, 5.54539009e-4, 1.2401295e-5, 5.52520865e-4},
  Close{216u, 0.0, 5.6349346e-4, 0.0, 5.62515051e-4},
  Close{217u, 0.0, 5.63025972e-4, 0.0, 5.63226839e-4},
  Close{218u, 1.39286058e-5, 5.69297369e-4, 5.09022357e-7, 5.71308544e-4},
  Close{219u, 0.0, 5.60461283e-4, 0.0, 5.61009048e-4},
  Close{220u, 0.0, 5.57403695e-4, 0.0, 5.57422538e-4},
  Close{221u, 0.0, 5.56300945e-4, 0.0, 5.57701946e-4},
  Close{222u, 2.28721519e-5, 5.45259294e-4, 1.13096094e-5, 5.46377502e-4},
  Close{223u, 0.0, 5.51727346e-4, 0.0, 5.4888116e-4},
  Close{224u, 0.0, 5.48858656e-4, 0.0, 5.53229927e-4},
  Close{225u, 0.0, 5.58548938e-4, 0.0, 5.58255465e-4},
  Close{226u, 1.51446511e-6, 5.53269613e-4, 0.0, 5.59599106e-4},
  Close{227u, 0.0, 5.52076808e-4, 2.23275856e-6, 5.56775477e-4},
  Close{228u, 0.0, 5.53436927e-4, 0.0, 5.53795311e-4},
  Close{229u, 0.0, 5.56110154e-4, 0.0, 5.55471815e-4},
  Close{230u, 0.0, 5.54390422e-4, 0.0, 5.54831138e-4},
  Close{231u, 0.0, 5.70690672e-4, 0.0, 5.69570794e-4},
  Close{232u, 0.0, 5.64063859e-4, 0.0, 5.63633787e-4},
  Close{233u, 0.0, 5.59907185e-4, 0.0, 5.62387754e-4},
  Close{234u, 0.0, 5.60827629e-4, 0.0, 5.60591645e-4},
  Close{235u, 0.0, 5.51517871e-4, 0.0, 5.64280588e-4},
  Close{236u, 0.0, 5.56321311e-4, 0.0, 5.50110997e-4},
  Close{237u, 0.0, 5.54397566e-4, 0.0, 5.54098605e-4},
  Close{238u, 0.0, 5.52639569e-4, 0.0, 5.53973539e-4},
  Close{239u, 0.0, 5.5872377e-4, 0.0, 5.56012278e-4},
  Close{240u, 2.59453373e-7, 5.65737642e-4, 0.0, 5.63807573e-4}
};

struct Row {
  const char* name = "";
  VolumeElementType element = VolumeElementType::Hex;
  float yield = 0.0f;
  float hardening = 0.0f;
  float from = 0.0f;
  float to = 0.0f;
  uint32_t ramp = 0u;
  uint32_t frames = 0u;
  bool summedGated = false;
  double tightEnd = 0.0;
  double tightSummed = 0.0;
  uint64_t tightSolves = 0u;
  uint64_t mainSolves = 0u;
  std::span<const Close> closes{};
};

constexpr std::array<Row, 5> ROWS{
  Row{
    "k",
    VolumeElementType::Hex,
    5.0e3f,
    HARDENING,
    12.0f,
    13.0f,
    120u,
    200u,
    true,
    7.55287649e-4,
    4.04702209e-2,
    891u,
    432u
  },
  Row{
    "f",
    VolumeElementType::Hex,
    5.0e3f,
    2.0e8f,
    5.0f,
    20.0f,
    120u,
    200u,
    true,
    1.25384409e-4,
    8.25596101e-3,
    791u,
    548u,
    F_CLOSES
  },
  Row{
    "h",
    VolumeElementType::Tet,
    5.0e3f,
    2.0e8f,
    5.0f,
    20.0f,
    120u,
    200u,
    true,
    3.16937731e-5,
    3.39277538e-3,
    956u,
    635u
  },
  Row{
    "a",
    VolumeElementType::Hex,
    5.0e3f,
    HARDENING,
    5.0f,
    20.0f,
    120u,
    240u,
    false,
    5.32848935e-4,
    6.55629511e-2,
    1362u,
    930u,
    A_CLOSES
  },
  Row{
    "b",
    VolumeElementType::Tet,
    5.0e3f,
    HARDENING,
    5.0f,
    20.0f,
    120u,
    240u,
    true,
    3.03478882e-4,
    3.25790389e-2,
    1238u,
    1022u
  }
};

struct Closed {
  uint32_t frame = 0u;
  double increment = 0.0;
  double principal = 0.0;
};

struct Spun {
  uint32_t contactSteps = 0u;
  uint64_t contactFallbacks = 0u;
  uint64_t contactRejections = 0u;
  uint64_t capped = 0u;
  uint64_t fallbacks = 0u;
  uint64_t rejections = 0u;
  uint64_t solves = 0u;
  uint64_t evaluations = 0u;
  float ended = 0.0f;
  double summed = 0.0;
  float fastest = 0.0f;
  double centreY = 0.0;
  std::string trace;
  std::vector<Closed> closes;
};

double summedEquivalent(const DeformableState& state) {
  double summed = 0.0;

  for (const talus::objectmodel::PlasticPoint& point : state.plastic) {
    summed += static_cast<double>(point.equivalent);
  }

  return summed;
}

Spun spin(
  const talus::objectmodel::PreparedMesh& mesh,
  const std::vector<PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  const Row& row
) {
  const talus::field::SampledSdf slab = landingbar::landingSlab();
  const talus::deformable::RigidPartner partner =
    landingbar::landingPartner(materials);

  talus::objectmodel::Pose pose;

  pose.position = glm::vec3(0.0f, HALF_WIDTH + CLEARANCE, 0.0f);

  DeformableState state = talus::deformable::restState(mesh, pose);

  auto solver = std::make_unique<talus::deformable::DeformableSolverState>();
  std::vector<talus::deformable::RigidPartner> partners;
  std::ostringstream trace;

  trace.precision(9);

  Spun run;

  for (uint32_t frame = 0u; frame != row.frames; ++frame) {
    const float along = frame < row.ramp ? static_cast<float>(frame)
                                             / static_cast<float>(row.ramp)
                                         : 1.0f;
    const float level = row.from + (row.to - row.from) * along;

    if (frame < row.ramp) {
      const glm::vec3 centre(landingbar::landingCentre(mesh, state));

      double momentum = 0.0;
      double inertia = 0.0;

      for (size_t node = 0u; node != state.position.size(); ++node) {
        const glm::dvec3 arm = glm::dvec3(state.position[node] - centre);
        const double held = static_cast<double>(mesh.nodeMass[node]);

        momentum += held * glm::cross(arm, glm::dvec3(state.velocity[node])).y;
        inertia += held * (arm.x * arm.x + arm.z * arm.z);
      }

      const glm::vec3 turn(
        0.0f,
        level - static_cast<float>(momentum / inertia),
        0.0f
      );

      for (size_t node = 0u; node != state.position.size(); ++node) {
        state.velocity[node] += glm::cross(turn, state.position[node] - centre);
      }
    }

    const std::vector<talus::deformable::SurfaceContact> contacts =
      landingbar::landingContacts(mesh, state, materials, slab);
    const double before = summedEquivalent(state);

    partners.assign(1u, partner);

    const bool contactStep = std::ranges::any_of(
      contacts,
      [&](const talus::deformable::SurfaceContact& held) {
        return held.partner >= 0
               && static_cast<size_t>(held.partner) < partners.size();
      }
    );

    talus::deformable::StepReport report;

    landingbar::landingStep(
      mesh,
      state,
      materials,
      settings,
      contacts,
      partners,
      solver.get(),
      report
    );

    run.contactSteps += contactStep ? 1u : 0u;
    run.contactFallbacks += contactStep ? report.plasticFallbacks : 0u;
    run.contactRejections +=
      contactStep ? report.plasticFallbackRejections : 0u;
    run.capped += report.plasticCapped;
    run.fallbacks += report.plasticFallbacks;
    run.rejections += report.plasticFallbackRejections;
    run.solves += report.plasticSolves;
    run.evaluations += report.plasticEvaluations;

    const bool closed =
      contactStep && report.plasticCapped == 0u && report.plasticFallbacks == 0u
      && report.plasticFallbackRejections == 0u
      && report.plasticLinearReads == 0u && report.plasticHandoffs == 0u
      && report.plasticEvaluations
           == report.plasticSolves + report.plasticBacktracks + 1u;

    if (closed && !row.closes.empty()) {
      run.closes.push_back(
        Closed{
          frame + 1u,
          summedEquivalent(state) - before,
          spun::largestPrincipal(mesh, state)
        }
      );
    }

    if ((frame + 1u) % TRACE == 0u) {
      trace << " f" << frame + 1u << ' ' << largestEquivalent(state) << ' '
            << summedEquivalent(state) << ' '
            << landingbar::landingCentre(mesh, state).y;
    }
  }

  run.ended = largestEquivalent(state);
  run.summed = summedEquivalent(state);
  run.centreY = landingbar::landingCentre(mesh, state).y;

  for (const glm::vec3& moving : state.velocity) {
    run.fastest = std::max(run.fastest, glm::length(moving));
  }

  run.trace = trace.str();

  return run;
}

}

}

TEST_CASE(
  "a plastic bar spun while resting on a field slab yields without a fallback "
  "and within its tight summed strain",
  "[.][plasticyieldcontact]"
) {
  std::cout.precision(9);

  for (const yieldingcontact::Row& row : yieldingcontact::ROWS) {
    const std::vector<PhysicalMaterial> materials =
      barMaterials(row.yield, row.hardening);
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, row.element);

    REQUIRE(mesh != nullptr);

    const talus::deformable::Settings settings = barStep(row.element);
    const yieldingcontact::Spun run =
      yieldingcontact::spin(*mesh, materials, settings, row);

    const double endOff = static_cast<double>(run.ended) / row.tightEnd - 1.0;
    const double summedOff = run.summed / row.tightSummed - 1.0;
    const bool summedHeld = std::abs(summedOff) <= yieldingcontact::BAND;

    std::cout << "yielding contact: row " << row.name << ' '
              << elementName(row.element) << " yield " << row.yield
              << " hardening " << row.hardening << " spin " << row.from
              << " to " << row.to << " rad/s over " << row.ramp
              << " frames, frames " << row.frames << ", clearance "
              << yieldingcontact::CLEARANCE << " m, substeps "
              << settings.substeps << " spectralRadius "
              << settings.spectralRadius << " plasticPasses "
              << settings.plasticPasses << ": contact steps "
              << run.contactSteps << " contact fallbacks "
              << run.contactFallbacks << " contact rejections "
              << run.contactRejections << " capped " << run.capped
              << " fallbacks " << run.fallbacks << " rejections "
              << run.rejections << " end " << run.ended << " fastest "
              << run.fastest << " centreY " << run.centreY << '\n';

    std::cout << "yielding contact against tight: row " << row.name << ": end "
              << run.ended << " tight " << row.tightEnd << " off " << endOff
              << " printed, summed " << run.summed << " tight "
              << row.tightSummed << " off " << summedOff << " threshold "
              << yieldingcontact::BAND << ' '
              << (row.summedGated ? "gated" : "printed") << " within "
              << summedHeld << ", plasticSolves " << run.solves << " main "
              << row.mainSolves << " tight " << row.tightSolves
              << " plasticEvaluations " << run.evaluations << ", tight from "
              << yieldingcontact::TIGHT_SOURCE << ", main from "
              << yieldingcontact::MAIN_SOURCE << '\n';

    std::cout << "yielding contact trace: row " << row.name
              << " frame largest summed centreY" << run.trace << '\n';

    INFO(
      "row " << row.name << ' ' << elementName(row.element)
             << ": contact steps " << run.contactSteps << ", contact fallbacks "
             << run.contactFallbacks << ", contact rejections "
             << run.contactRejections << ", summed " << run.summed
             << " against tight " << row.tightSummed << " off " << summedOff
    );

    CHECK(run.contactSteps > 0u);
    CHECK(run.contactFallbacks == 0u);
    CHECK(run.contactRejections == 0u);

    if (row.summedGated) {
      CHECK(summedHeld);
    }

    if (row.closes.empty()) {
      continue;
    }

    std::vector<uint32_t> judgedFrames;
    std::vector<uint32_t> quotedFrames;
    std::ostringstream carried;

    carried.precision(9);

    for (const yieldingcontact::Close& quoted : row.closes) {
      quotedFrames.push_back(quoted.frame);
    }

    uint32_t misses = 0u;
    uint32_t unquoted = 0u;
    double worstIncrement = 0.0;
    double worstPrincipal = 0.0;
    uint32_t worstIncrementFrame = 0u;
    uint32_t worstPrincipalFrame = 0u;

    for (const yieldingcontact::Closed& closed : run.closes) {
      judgedFrames.push_back(closed.frame);
      carried << " f" << closed.frame << ' ' << closed.increment << ' '
              << closed.principal;

      const auto quoted = std::ranges::find(
        row.closes,
        closed.frame,
        &yieldingcontact::Close::frame
      );

      if (quoted == row.closes.end()) {
        ++unquoted;

        std::cout << "yielding contact close unquoted: row " << row.name
                  << " frame " << closed.frame << " increment "
                  << closed.increment << " principal " << closed.principal
                  << '\n';

        continue;
      }

      const double incrementShare =
        std::min(
          std::abs(closed.increment - quoted->replayedIncrement),
          std::abs(closed.increment - quoted->freshIncrement)
        )
        / run.summed;
      const double principalOff = std::min(
        std::abs(closed.principal / quoted->replayedPrincipal - 1.0),
        std::abs(closed.principal / quoted->freshPrincipal - 1.0)
      );
      const bool held = incrementShare <= yieldingcontact::CLOSE_INCREMENT
                        && principalOff <= yieldingcontact::CLOSE_PRINCIPAL;

      if (incrementShare >= worstIncrement) {
        worstIncrement = incrementShare;
        worstIncrementFrame = closed.frame;
      }

      if (principalOff >= worstPrincipal) {
        worstPrincipal = principalOff;
        worstPrincipalFrame = closed.frame;
      }

      if (!held) {
        ++misses;

        std::cout << "yielding contact close miss: row " << row.name
                  << " frame " << closed.frame << " increment "
                  << closed.increment << " replayed tight "
                  << quoted->replayedIncrement << " fresh tight "
                  << quoted->freshIncrement << " share " << incrementShare
                  << ", principal " << closed.principal << " replayed tight "
                  << quoted->replayedPrincipal << " fresh tight "
                  << quoted->freshPrincipal << " off " << principalOff << '\n';
      }

      INFO(
        "row " << row.name << " frame " << closed.frame << ": increment "
               << closed.increment << " share " << incrementShare
               << " of summed " << run.summed << " from the nearer tight arm, "
               << "principal " << closed.principal << " off " << principalOff
               << " from the nearer tight arm, tight arms from "
               << yieldingcontact::CENSUS_SOURCE
      );

      CHECK(held);
    }

    std::cout << "yielding contact closes: row " << row.name
              << ": contact-step closes on the no-loading exit judged "
              << judgedFrames.size() << " quoted " << quotedFrames.size()
              << " unquoted " << unquoted << " misses " << misses
              << ", worst increment share " << worstIncrement << " at frame "
              << worstIncrementFrame << " threshold "
              << yieldingcontact::CLOSE_INCREMENT
              << " of summed, worst principal off " << worstPrincipal
              << " at frame " << worstPrincipalFrame << " threshold "
              << yieldingcontact::CLOSE_PRINCIPAL
              << ", each against the nearer of the replayed and fresh tight "
                 "arms from "
              << yieldingcontact::CENSUS_SOURCE << '\n';

    std::cout << "yielding contact closes carried: row " << row.name
              << " frame increment principal" << carried.str() << '\n';

    CHECK(judgedFrames == quotedFrames);
  }
}

namespace {

namespace spawn {

constexpr float SPEED = 8.0f;
constexpr double RESIDUAL = 1.0e-6;
constexpr float ROTATION = 1.0e-6f;
constexpr uint32_t HASHED = 5u;

void frame(
  const talus::objectmodel::PreparedMesh& mesh,
  DeformableState& state,
  const std::vector<PhysicalMaterial>& materials,
  const talus::deformable::Settings& settings,
  const talus::field::SampledSdf& slab,
  talus::deformable::DeformableSolverState& solver,
  talus::deformable::StepReport& report
) {
  const std::vector<talus::deformable::SurfaceContact> contacts =
    landingbar::landingContacts(mesh, state, materials, slab);

  std::vector<talus::deformable::RigidPartner> partners(
    1u,
    landingbar::landingPartner(materials)
  );

  landingbar::landingStep(
    mesh,
    state,
    materials,
    settings,
    contacts,
    partners,
    &solver,
    report
  );
}

double rotationResidual(
  const talus::deformable::SystemFactor& factor,
  const talus::objectmodel::PreparedMesh& mesh
) {
  const talus::deformable::FactorReference& reference = factor.reference();
  const size_t nodes = mesh.restPositions.size();

  double worst = 0.0;

  for (int axis = 0; axis != 3; ++axis) {
    glm::vec3 about(0.0f);

    about[axis] = 1.0f;

    std::vector<glm::vec3> mode(nodes, glm::vec3(0.0f));
    std::vector<glm::vec3> solved(nodes, glm::vec3(0.0f));

    for (size_t node = 0u; node != nodes; ++node) {
      mode[node] =
        glm::cross(about, reference.positions[node] - reference.centroid);
      solved[node] = reference.inertia * mesh.nodeMass[node] * mode[node];
    }

    talus::deformable::applyFactor(factor, glm::mat3(1.0f), mesh, solved);

    double apart = 0.0;
    double length = 0.0;

    for (size_t node = 0u; node != nodes; ++node) {
      const glm::dvec3 miss = glm::dvec3(solved[node]) - glm::dvec3(mode[node]);

      apart += glm::dot(miss, miss);
      length += glm::dot(glm::dvec3(mode[node]), glm::dvec3(mode[node]));
    }

    worst = std::max(worst, std::sqrt(apart / length));
  }

  return worst;
}

float rotationDeviation(
  const talus::objectmodel::PreparedMesh& mesh,
  const DeformableState& state
) {
  float worst = 0.0f;

  const auto compare =
    [&worst](const glm::mat3& stored, const glm::mat3& polar) {
      for (int column = 0; column != 3; ++column) {
        for (int row = 0; row != 3; ++row) {
          worst =
            std::max(worst, std::abs(stored[column][row] - polar[column][row]));
        }
      }
    };

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];

    std::array<glm::vec3, 8> corners{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      corners[corner] = state.position[hex.nodes[corner]];
    }

    compare(
      state.committed[element].rotation,
      talus::deformable::hexRotation(corners, hex.halfExtents)
    );
  }

  if (mesh.hexes.empty()) {
    for (size_t element = 0u; element != mesh.tets.size(); ++element) {
      const talus::objectmodel::PreparedTet& tet = mesh.tets[element];
      const glm::vec3& first = state.position[tet.nodes[0]];
      const glm::mat3 gradient = glm::mat3(
                                   state.position[tet.nodes[1]] - first,
                                   state.position[tet.nodes[2]] - first,
                                   state.position[tet.nodes[3]] - first
                                 )
                                 * tet.restInverse;

      compare(
        state.committed[element].rotation,
        talus::deformable::polarRotation(gradient)
      );
    }
  }

  return worst;
}

uint64_t stateHash(const DeformableState& state) {
  uint64_t hash = 14695981039346656037ull;

  const auto mix = [&hash](float value) {
    hash = (hash ^ std::bit_cast<uint32_t>(value)) * 1099511628211ull;
  };

  const auto mixMatrix = [&mix](const glm::mat3& value) {
    for (int column = 0; column != 3; ++column) {
      for (int row = 0; row != 3; ++row) {
        mix(value[column][row]);
      }
    }
  };

  for (const std::vector<glm::vec3>* field :
       {&state.position, &state.velocity, &state.acceleration}) {
    for (const glm::vec3& held : *field) {
      mix(held.x);
      mix(held.y);
      mix(held.z);
    }
  }

  for (const talus::objectmodel::CommittedTet& held : state.committed) {
    mixMatrix(held.rotation);
  }

  for (const talus::objectmodel::PlasticPoint& held : state.plastic) {
    mixMatrix(held.strain);
    mix(held.equivalent);
  }

  return hash;
}

}

}

TEST_CASE(
  "a tilted plastic bar spawn builds one factor in its first frame",
  "[plasticity]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);
  const talus::field::SampledSdf slab = landingbar::landingSlab();

  for (const VolumeElementType element : ELEMENTS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, element);

    REQUIRE(mesh != nullptr);

    const talus::deformable::Settings settings = barStep(element);

    DeformableState state = impact::dropped(*mesh, spawn::SPEED, impact::TILT);

    auto solver = std::make_unique<talus::deformable::DeformableSolverState>();
    talus::deformable::StepReport report;

    spawn::frame(*mesh, state, materials, settings, slab, *solver, report);

    const double residual = spawn::rotationResidual(solver->factor, *mesh);

    std::cout << "spawn build: " << elementName(element) << " tilt "
              << impact::TILT << " speed " << spawn::SPEED << " factorBudget "
              << settings.factorBudget << " cgIterations "
              << settings.cgIterations << ": builds "
              << solver->stats.refactorCount << " generation "
              << solver->factor.generation() << " refreshRequests "
              << report.refreshRequests << " retries " << report.retries
              << " cgCappedSolves " << report.cgCappedSolves
              << " rotation residual " << residual << '\n';

    INFO(
      elementName(element) << ": builds " << solver->stats.refactorCount
                           << " generation " << solver->factor.generation()
                           << " refreshRequests " << report.refreshRequests
                           << " retries " << report.retries
                           << " rotation residual " << residual
    );

    CHECK(solver->stats.refactorCount == 1u);
    CHECK(solver->factor.generation() == 1u);
    CHECK(report.refreshRequests == 0u);
    CHECK(report.retries == 0u);
    CHECK(residual <= spawn::RESIDUAL);
  }
}

TEST_CASE(
  "a spawned plastic bar's committed rotations are its elements' polar "
  "rotations",
  "[plasticity]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, element);

    REQUIRE(mesh != nullptr);

    for (const float tilt : {impact::TILT, 0.0f}) {
      const DeformableState state = impact::dropped(*mesh, spawn::SPEED, tilt);

      REQUIRE(
        state.committed.size()
        == (mesh->hexes.empty() ? mesh->tets.size() : mesh->hexes.size())
      );

      const float deviation = spawn::rotationDeviation(*mesh, state);

      std::cout << "spawn rotations: " << elementName(element) << " tilt "
                << tilt << " elements " << state.committed.size()
                << ": largest entry off the polar rotation " << deviation
                << '\n';

      INFO(
        elementName(element)
        << " tilt " << tilt << ": largest entry off the polar rotation "
        << deviation
      );

      CHECK(deviation <= spawn::ROTATION);
    }
  }
}

TEST_CASE(
  "an identity-pose plastic bar spawn steps as with identity rotations",
  "[plasticity]"
) {
  const std::vector<PhysicalMaterial> materials = barMaterials(YIELD);
  const talus::field::SampledSdf slab = landingbar::landingSlab();

  for (const VolumeElementType element : ELEMENTS) {
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(materials, element);

    REQUIRE(mesh != nullptr);

    const talus::deformable::Settings settings = barStep(element);

    DeformableState spawned = impact::dropped(*mesh, spawn::SPEED, 0.0f);
    DeformableState identity = spawned;

    identity.committed.assign(
      identity.committed.size(),
      talus::objectmodel::CommittedTet{}
    );

    auto spawnedSolver =
      std::make_unique<talus::deformable::DeformableSolverState>();
    auto identitySolver =
      std::make_unique<talus::deformable::DeformableSolverState>();

    std::ostringstream hashes;

    hashes << std::hex << spawn::stateHash(spawned);

    CHECK(spawn::stateHash(spawned) == spawn::stateHash(identity));

    for (uint32_t step = 0u; step != spawn::HASHED; ++step) {
      talus::deformable::StepReport spawnedReport;
      talus::deformable::StepReport identityReport;

      spawn::frame(
        *mesh,
        spawned,
        materials,
        settings,
        slab,
        *spawnedSolver,
        spawnedReport
      );
      spawn::frame(
        *mesh,
        identity,
        materials,
        settings,
        slab,
        *identitySolver,
        identityReport
      );

      hashes << ' ' << spawn::stateHash(spawned);

      INFO(elementName(element) << " step " << step + 1u);

      CHECK(spawn::stateHash(spawned) == spawn::stateHash(identity));
    }

    std::cout << "spawn identity: " << elementName(element) << " speed "
              << spawn::SPEED << " state hashes over " << spawn::HASHED
              << " steps " << hashes.str() << '\n';
  }
}

namespace {

namespace damping {

constexpr double SPECTRAL = 0.5;
constexpr double ALPHA_MASS = (2.0 * SPECTRAL - 1.0) / (SPECTRAL + 1.0);
constexpr double ALPHA_FORCE = SPECTRAL / (SPECTRAL + 1.0);
constexpr double GAMMA = 0.5 - ALPHA_MASS + ALPHA_FORCE;

constexpr float SPIN_TAU_HEX = 2.3648e-4f;
constexpr float SPIN_TAU_TET = 1.67241e-4f;
constexpr float CLAMPED_TAU_HEX = 1.40943e-3f;
constexpr float CLAMPED_TAU_TET = 9.16921e-4f;
constexpr uint32_t SPIN_FRAMES = 300u;
constexpr uint32_t RAMP_FRAMES = 60u;
constexpr uint32_t HOLD_FRAMES = 60u;
constexpr double MOMENTUM_BAR = 1.0e-3;
constexpr double ACCOUNT_BAR = 1.0e-2;
constexpr double OPERATOR_BAR = 1.0e-5;
constexpr uint32_t SUSTAINED_FRAMES = 10u;

std::vector<PhysicalMaterial> dampedMaterials(float tau) {
  std::vector<PhysicalMaterial> made = barMaterials(YIELD);

  made[LANE].stiffnessDamping = tau;

  return made;
}

glm::dvec3 angularOf(
  const talus::objectmodel::PreparedMesh& mesh,
  const DeformableState& state,
  const std::vector<glm::vec3>& field
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

  glm::dvec3 made(0.0);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    made += static_cast<double>(mesh.nodeMass[node])
            * glm::cross(
              glm::dvec3(state.position[node]) - centre,
              glm::dvec3(field[node]) - mean
            );
  }

  return made;
}

DeformableState spinStart(
  const talus::objectmodel::PreparedMesh& mesh,
  float rate,
  bool steady
) {
  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});

  glm::vec3 centre(0.0f);
  float mass = 0.0f;

  for (size_t node = 0u; node != state.position.size(); ++node) {
    centre += mesh.nodeMass[node] * state.position[node];
    mass += mesh.nodeMass[node];
  }

  centre /= mass;

  const glm::vec3 spin(0.0f, 0.0f, rate);

  for (size_t node = 0u; node != state.position.size(); ++node) {
    const glm::vec3 arm = state.position[node] - centre;

    state.velocity[node] = glm::cross(spin, arm);
    state.acceleration[node] =
      steady ? glm::cross(spin, glm::cross(spin, arm)) : glm::vec3(0.0f);
  }

  return state;
}

talus::deformable::StepReport advance(
  const talus::objectmodel::PreparedMesh& mesh,
  DeformableState& state,
  std::span<const PhysicalMaterial> materials,
  const talus::deformable::Settings& settings,
  const glm::vec3& gravity,
  talus::deformable::DeformableSolverState* solver
) {
  talus::deformable::DeformableSolverState fresh;
  talus::deformable::StepReport report;

  talus::deformable::step(
    mesh,
    state,
    materials,
    settings,
    gravity,
    TIME_STEP,
    {},
    {},
    solver != nullptr ? solver : &fresh,
    report
  );

  return report;
}

template<size_t Count, typename Value>
std::array<glm::dvec3, Count> relativeOf(
  const std::array<glm::vec3, Count>& at,
  const std::array<Value, Count>& values
) {
  glm::dvec3 centre(0.0);

  for (const glm::vec3& held : at) {
    centre += glm::dvec3(held);
  }

  centre /= static_cast<double>(Count);

  std::array<glm::dvec3, Count> arms{};
  glm::dmat3 inertia(0.0);

  for (size_t corner = 0u; corner != Count; ++corner) {
    arms[corner] = glm::dvec3(at[corner]) - centre;
    inertia += glm::dot(arms[corner], arms[corner]) * glm::dmat3(1.0)
               - glm::outerProduct(arms[corner], arms[corner]);
  }

  glm::dvec3 mean(0.0);
  glm::dvec3 moment(0.0);

  for (size_t corner = 0u; corner != Count; ++corner) {
    mean += glm::dvec3(values[corner]);
    moment += glm::cross(arms[corner], glm::dvec3(values[corner]));
  }

  mean /= static_cast<double>(Count);

  const glm::dvec3 spin = glm::inverse(inertia) * moment;

  std::array<glm::dvec3, Count> made{};

  for (size_t corner = 0u; corner != Count; ++corner) {
    made[corner] =
      glm::dvec3(values[corner]) - mean - glm::cross(spin, arms[corner]);
  }

  return made;
}

template<size_t Count>
std::array<glm::dvec3, Count> dampedForce(
  const std::array<glm::vec3, Count>& at,
  const glm::mat3& rotation,
  std::span<const glm::mat3> blocks,
  const std::array<glm::vec3, Count>& values
) {
  const std::array<glm::dvec3, Count> relative = relativeOf(at, values);
  const glm::dmat3 turn(rotation);

  std::array<glm::dvec3, Count> produced{};

  for (size_t row = 0u; row != Count; ++row) {
    glm::dvec3 sum(0.0);

    for (size_t column = 0u; column != Count; ++column) {
      sum += glm::dmat3(blocks[row * Count + column])
             * (glm::transpose(turn) * relative[column]);
    }

    produced[row] = turn * sum;
  }

  return relativeOf(at, produced);
}

template<size_t Count>
double viscousPower(
  const std::array<glm::vec3, Count>& at,
  const glm::mat3& rotation,
  std::span<const glm::mat3> blocks,
  const std::array<glm::vec3, Count>& velocity
) {
  const std::array<glm::dvec3, Count> relative = relativeOf(at, velocity);
  const glm::dmat3 turn(rotation);

  double power = 0.0;

  for (size_t row = 0u; row != Count; ++row) {
    glm::dvec3 sum(0.0);

    for (size_t column = 0u; column != Count; ++column) {
      sum += glm::dmat3(blocks[row * Count + column])
             * (glm::transpose(turn) * relative[column]);
    }

    power += glm::dot(turn * sum, relative[row]);
  }

  return power;
}

struct Account {
  double elastic = 0.0;
  double plastic = 0.0;
  double viscous = 0.0;
};

Account account(
  const Bar& bar,
  std::span<const PhysicalMaterial> materials,
  const DeformableState& before,
  const DeformableState& after,
  double tau
) {
  const talus::objectmodel::PreparedMesh& mesh = *bar.mesh;
  const talus::deformable::ElasticParams elastic{
    materials[LANE].youngsModulus,
    materials[LANE].poissonsRatio,
    materials[LANE].density
  };
  const talus::deformable::PlasticParams plastic{
    materials[LANE].yieldStress,
    materials[LANE].hardeningModulus
  };
  const talus::deformable::ElementRotation rotation =
    talus::deformable::ElementRotation::Invertible;
  const float lag = static_cast<float>(ALPHA_FORCE);
  const size_t points = 8u * mesh.hexes.size() + mesh.tets.size();

  std::vector<talus::objectmodel::PlasticPoint> committed = before.plastic;

  committed.resize(points);

  Account made;

  for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
    const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];

    std::array<glm::vec3, 8> ended{};
    std::array<glm::vec3, 8> read{};
    std::array<glm::vec3, 8> rest{};
    std::array<glm::vec3, 8> moving{};
    std::array<talus::objectmodel::PlasticPoint, 8> held{};

    for (uint32_t corner = 0u; corner != 8u; ++corner) {
      const uint32_t node = hex.nodes[corner];

      ended[corner] = after.position[node];
      read[corner] =
        (1.0f - lag) * after.position[node] + lag * before.position[node];
      rest[corner] = mesh.restPositions[node];
      moving[corner] =
        (1.0f - lag) * after.velocity[node] + lag * before.velocity[node];
      held[corner] = committed[8u * element + corner];
    }

    const talus::deformable::HexPointTrial end =
      talus::deformable::hexPointTrial(
        ended,
        rest,
        hex,
        elastic,
        plastic,
        held,
        0.0f,
        rotation
      );
    const talus::deformable::HexPointTrial at =
      talus::deformable::hexPointTrial(
        read,
        rest,
        hex,
        elastic,
        plastic,
        held,
        0.0f,
        rotation
      );

    made.elastic += static_cast<double>(end.energy);
    made.plastic += static_cast<double>(end.plasticWork);
    made.viscous += static_cast<double>(TIME_STEP) * tau
                    * viscousPower(
                      read,
                      at.rotation,
                      talus::deformable::hexStiffness(hex, elastic).block,
                      moving
                    );
  }

  for (size_t element = 0u; element != mesh.tets.size(); ++element) {
    const talus::objectmodel::PreparedTet& tet = mesh.tets[element];
    const size_t point = 8u * mesh.hexes.size() + element;

    std::array<glm::vec3, 4> ended{};
    std::array<glm::vec3, 4> read{};
    std::array<glm::vec3, 4> rest{};
    std::array<glm::vec3, 4> moving{};

    for (int corner = 0; corner != 4; ++corner) {
      const size_t local = static_cast<size_t>(corner);
      const uint32_t node = tet.nodes[corner];

      ended[local] = after.position[node];
      read[local] =
        (1.0f - lag) * after.position[node] + lag * before.position[node];
      rest[local] = mesh.restPositions[node];
      moving[local] =
        (1.0f - lag) * after.velocity[node] + lag * before.velocity[node];
    }

    const talus::deformable::TetPointTrial end =
      talus::deformable::tetPointTrial(
        ended,
        rest,
        tet,
        elastic,
        plastic,
        committed[point],
        0.0f,
        rotation
      );
    const talus::deformable::TetPointTrial at =
      talus::deformable::tetPointTrial(
        read,
        rest,
        tet,
        elastic,
        plastic,
        committed[point],
        0.0f,
        rotation
      );

    made.elastic += static_cast<double>(end.energy);
    made.plastic += static_cast<double>(end.plasticWork);
    made.viscous += static_cast<double>(TIME_STEP) * tau
                    * viscousPower(
                      read,
                      at.rotation,
                      talus::deformable::tetStiffness(tet, elastic).block,
                      moving
                    );
  }

  return made;
}

double kinetic(
  const talus::objectmodel::PreparedMesh& mesh,
  const DeformableState& state
) {
  double sum = 0.0;

  for (size_t node = 0u; node != state.velocity.size(); ++node) {
    const glm::dvec3 moving(state.velocity[node]);

    sum +=
      0.5 * static_cast<double>(mesh.nodeMass[node]) * glm::dot(moving, moving);
  }

  return sum;
}

double operatorApart(
  const Bar& bar,
  std::span<const PhysicalMaterial> materials,
  const talus::deformable::StepReport& report,
  double tau
) {
  const talus::objectmodel::PreparedMesh& mesh = *bar.mesh;
  const talus::deformable::ElasticParams elastic{
    materials[LANE].youngsModulus,
    materials[LANE].poissonsRatio,
    materials[LANE].density
  };
  const size_t nodes = mesh.restPositions.size();
  const size_t rows = 3u * nodes;
  const double weight =
    (1.0 - ALPHA_FORCE) * GAMMA * static_cast<double>(TIME_STEP) * tau;

  REQUIRE(report.systemDamping.size() == rows * rows);
  REQUIRE(report.systemPositions.size() == nodes);

  double difference = 0.0;
  double size = 0.0;

  std::vector<glm::vec3> unit(nodes, glm::vec3(0.0f));

  for (size_t column = 0u; column != rows; ++column) {
    if (mesh.anchored[column / 3u] != 0u) {
      continue;
    }

    unit[column / 3u][static_cast<int>(column % 3u)] = 1.0f;

    std::vector<glm::dvec3> reference(nodes, glm::dvec3(0.0));

    const auto spread = [&](const auto& corners, const auto& forces) {
      for (size_t corner = 0u; corner != forces.size(); ++corner) {
        reference[corners[corner]] += weight * forces[corner];
      }
    };

    for (size_t element = 0u; element != mesh.hexes.size(); ++element) {
      const talus::objectmodel::PreparedHex& hex = mesh.hexes[element];

      std::array<glm::vec3, 8> at{};
      std::array<glm::vec3, 8> input{};

      for (uint32_t corner = 0u; corner != 8u; ++corner) {
        at[corner] = report.systemPositions[hex.nodes[corner]];
        input[corner] = unit[hex.nodes[corner]];
      }

      spread(
        hex.nodes,
        dampedForce(
          at,
          report.systemRotations[element],
          talus::deformable::hexStiffness(hex, elastic).block,
          input
        )
      );
    }

    for (size_t element = 0u; element != mesh.tets.size(); ++element) {
      const talus::objectmodel::PreparedTet& tet = mesh.tets[element];
      const std::array<uint32_t, 4> corners{
        tet.nodes[0],
        tet.nodes[1],
        tet.nodes[2],
        tet.nodes[3]
      };

      std::array<glm::vec3, 4> at{};
      std::array<glm::vec3, 4> input{};

      for (size_t corner = 0u; corner != 4u; ++corner) {
        at[corner] = report.systemPositions[corners[corner]];
        input[corner] = unit[corners[corner]];
      }

      spread(
        corners,
        dampedForce(
          at,
          report.systemRotations[element],
          talus::deformable::tetStiffness(tet, elastic).block,
          input
        )
      );
    }

    for (size_t row = 0u; row != rows; ++row) {
      if (mesh.anchored[row / 3u] != 0u) {
        continue;
      }

      const double wanted = reference[row / 3u][static_cast<int>(row % 3u)];
      const double gap =
        static_cast<double>(report.systemDamping[row * rows + column]) - wanted;

      difference += gap * gap;
      size += wanted * wanted;
    }

    unit[column / 3u][static_cast<int>(column % 3u)] = 0.0f;
  }

  return size > 0.0 ? std::sqrt(difference / size) : std::sqrt(difference);
}

struct Clamped {
  double worstPlastic = 0.0;
  double worstViscous = 0.0;
  double work = 0.0;
  double largestWork = 0.0;
  double plastic = 0.0;
  double viscous = 0.0;
  std::vector<double> numerical;
  std::vector<double> operators;
  uint32_t yielding = 0u;
  uint32_t capped = 0u;
  uint64_t solves = 0u;
  uint64_t linearReads = 0u;
  bool finite = true;
};

Clamped clampedRamp(const Bar& bar, float tau, VolumeElementType element) {
  const std::vector<PhysicalMaterial> materials = dampedMaterials(tau);
  const talus::objectmodel::PreparedMesh& mesh = *bar.mesh;
  const talus::deformable::Settings settings = barStep(element);

  DeformableState state =
    talus::deformable::restState(mesh, talus::objectmodel::Pose{});
  talus::deformable::DeformableSolverState solver;

  Clamped made;

  for (uint32_t frame = 1u; frame <= RAMP_FRAMES + HOLD_FRAMES; ++frame) {
    const float share =
      frame <= RAMP_FRAMES
        ? static_cast<float>(frame) / static_cast<float>(RAMP_FRAMES)
        : 1.0f;
    const glm::vec3 gravity(share * bodyPull(), 0.0f, 0.0f);
    const bool checked = tau > 0.0f && (frame == 40u || frame == RAMP_FRAMES);

    talus::deformable::Settings chosen = settings;

    chosen.reportSystem = checked;

    const DeformableState before = state;
    const float flowBefore = largestEquivalent(state);
    const talus::deformable::StepReport report =
      advance(mesh, state, materials, chosen, gravity, &solver);

    made.finite = made.finite && allFinite(state);
    made.capped += report.plasticCapped;
    made.solves += report.plasticSolves;
    made.linearReads += report.plasticLinearReads;

    const Account step =
      account(bar, materials, before, state, static_cast<double>(tau));

    for (size_t node = 0u; node != mesh.restPositions.size(); ++node) {
      made.work +=
        static_cast<double>(mesh.nodeMass[node])
        * glm::dot(
          glm::dvec3(gravity),
          glm::dvec3(state.position[node]) - glm::dvec3(before.position[node])
        );
    }

    made.largestWork = std::max(made.largestWork, made.work);
    made.plastic += step.plastic;
    made.viscous += step.viscous;
    made.worstPlastic = std::min(made.worstPlastic, step.plastic);
    made.worstViscous = std::min(made.worstViscous, step.viscous);
    made.yielding += step.plastic > 0.0 ? 1u : 0u;
    made.numerical.push_back(
      made.work - step.elastic - kinetic(mesh, state) - made.plastic
      - made.viscous
    );

    if (checked) {
      REQUIRE(largestEquivalent(state) > flowBefore);
      REQUIRE(report.plasticSolves > 0u);

      made.operators.push_back(
        operatorApart(bar, materials, report, static_cast<double>(tau))
      );
    }
  }

  return made;
}

}

}

TEST_CASE(
  "a damped plastic body keeps its damping on the Newton path",
  "[.][dampedplastic]"
) {
  std::cout.precision(9);

  for (const VolumeElementType element : ELEMENTS) {
    const bool hex = element == VolumeElementType::Hex;
    const float spinTau = hex ? damping::SPIN_TAU_HEX : damping::SPIN_TAU_TET;
    const std::vector<PhysicalMaterial> damped =
      damping::dampedMaterials(spinTau);
    const std::vector<PhysicalMaterial> plain = damping::dampedMaterials(0.0f);
    const std::shared_ptr<const talus::objectmodel::PreparedMesh> mesh =
      freeBar(plain, element);

    REQUIRE(mesh != nullptr);

    const talus::deformable::Settings settings = barStep(element);
    talus::deformable::Settings split = settings;

    split.rigidSplit = talus::deformable::RigidSplit::Unprojected;

    for (const float rate : {5.0f, 10.0f}) {
      for (const bool steady : {false, true}) {
        DeformableState arm = damping::spinStart(*mesh, rate, steady);
        talus::deformable::DeformableSolverState armSolver;

        const double start =
          glm::length(damping::angularOf(*mesh, arm, arm.velocity));

        glm::dvec3 summed(0.0);
        glm::dvec3 torqued(0.0);
        uint32_t firstFlow = 0u;
        uint32_t capped = 0u;
        uint64_t linearReads = 0u;
        bool finite = true;

        for (uint32_t frame = 1u; frame <= damping::SPIN_FRAMES; ++frame) {
          DeformableState withTau = arm;
          DeformableState without = arm;

          const talus::deformable::StepReport fromTau = damping::advance(
            *mesh,
            withTau,
            damped,
            split,
            glm::vec3(0.0f),
            nullptr
          );
          const talus::deformable::StepReport fromZero = damping::advance(
            *mesh,
            without,
            plain,
            split,
            glm::vec3(0.0f),
            nullptr
          );

          const talus::deformable::StepReport fromArm = damping::advance(
            *mesh,
            arm,
            damped,
            settings,
            glm::vec3(0.0f),
            &armSolver
          );

          linearReads += fromTau.plasticLinearReads
                         + fromZero.plasticLinearReads
                         + fromArm.plasticLinearReads;

          const glm::dvec3 moved =
            damping::angularOf(*mesh, withTau, withTau.velocity)
            - damping::angularOf(*mesh, without, without.velocity);
          const glm::dvec3 turned =
            damping::angularOf(*mesh, withTau, withTau.acceleration)
            - damping::angularOf(*mesh, without, without.acceleration);

          summed += moved;
          torqued +=
            moved
            + (1.0 - damping::GAMMA) * static_cast<double>(TIME_STEP) * turned;
          capped += fromTau.plasticCapped + fromZero.plasticCapped;
          finite = finite && allFinite(arm);

          if (firstFlow == 0u && largestEquivalent(arm) > 0.0f) {
            firstFlow = frame;
          }
        }

        const double pairedSum = glm::length(summed) / start;
        const double pairedTorqued = glm::length(torqued) / start;

        std::cout << "dampedplastic: spinner " << elementName(element)
                  << " rate " << rate << " start "
                  << (steady ? "steady" : "zero") << " tau " << spinTau
                  << " frames " << damping::SPIN_FRAMES << " substeps "
                  << settings.substeps << " plasticPasses "
                  << settings.plasticPasses << " tolerance "
                  << settings.tolerance << " pairedSumOverL0 " << pairedSum
                  << " pairedWithTorqueOverL0 " << pairedTorqued
                  << " firstFlow " << firstFlow << " largestEquivalent "
                  << largestEquivalent(arm) << " plasticCapped " << capped
                  << " linearReads " << linearReads << " bar "
                  << damping::MOMENTUM_BAR << '\n';

        REQUIRE(finite);
        REQUIRE(firstFlow == 0u);
        REQUIRE(linearReads == 0u);
        REQUIRE(pairedSum <= damping::MOMENTUM_BAR);
        REQUIRE(pairedTorqued <= damping::MOMENTUM_BAR);
      }
    }

    const Bar bar = buildBar(COARSE_VOXEL, plain, element);
    const float clampedTau =
      hex ? damping::CLAMPED_TAU_HEX : damping::CLAMPED_TAU_TET;

    const damping::Clamped withTau =
      damping::clampedRamp(bar, clampedTau, element);
    const damping::Clamped without = damping::clampedRamp(bar, 0.0f, element);

    REQUIRE(withTau.numerical.size() == without.numerical.size());

    double worstGap = std::numeric_limits<double>::infinity();
    uint32_t worstFrame = 0u;

    for (size_t frame = 0u; frame != withTau.numerical.size(); ++frame) {
      const double gap = withTau.numerical[frame] - without.numerical[frame];

      if (gap < worstGap) {
        worstGap = gap;
        worstFrame = static_cast<uint32_t>(frame + 1u);
      }
    }

    const double scale = withTau.largestWork;
    const double worstOperator =
      withTau.operators.empty()
        ? 1.0
        : *std::max_element(withTau.operators.begin(), withTau.operators.end());

    std::cout << "dampedplastic: clamped " << elementName(element) << " tau "
              << clampedTau << " rampFrames " << damping::RAMP_FRAMES
              << " holdFrames " << damping::HOLD_FRAMES << " peakStress "
              << 2.0f * PAST_STRESS << " yieldingFramesDamped "
              << withTau.yielding << " yieldingFramesUndamped "
              << without.yielding << " work " << withTau.work
              << " plasticOverWork " << withTau.plastic / scale
              << " viscousOverWork " << withTau.viscous / scale
              << " numericalOverWork " << withTau.numerical.back() / scale
              << " undampedNumericalOverWork "
              << without.numerical.back() / without.largestWork
              << " worstNumericalGapOverWork " << worstGap / scale << " at "
              << worstFrame << " worstStepPlastic " << withTau.worstPlastic
              << " worstStepViscous " << withTau.worstViscous
              << " newtonOperator " << worstOperator << " plasticCapped "
              << withTau.capped << ' ' << without.capped << " solves "
              << withTau.solves << ' ' << without.solves << " linearReads "
              << withTau.linearReads << ' ' << without.linearReads << " bars "
              << damping::ACCOUNT_BAR << ' ' << damping::OPERATOR_BAR << '\n';

    REQUIRE(withTau.finite);
    REQUIRE(without.finite);
    REQUIRE(withTau.linearReads == 0u);
    REQUIRE(without.linearReads == 0u);
    REQUIRE(withTau.yielding >= damping::SUSTAINED_FRAMES);
    REQUIRE(withTau.worstPlastic >= 0.0);
    REQUIRE(withTau.worstViscous >= 0.0);
    REQUIRE(without.worstPlastic >= 0.0);
    REQUIRE(worstGap >= -damping::ACCOUNT_BAR * scale);
    REQUIRE(withTau.operators.size() == 2u);
    REQUIRE(worstOperator <= damping::OPERATOR_BAR);
  }
}

namespace {

namespace release {

constexpr uint32_t FRAMES = 3u;
constexpr double LINEAR_BAR = 1.0e-5;
constexpr std::array<float, 4> FACTORS{0.1f, 0.25f, 0.5f, 0.9f};

std::vector<PhysicalMaterial> releaseMaterials(float yield, float tau) {
  std::vector<PhysicalMaterial> made = barMaterials(yield);

  made[LANE].stiffnessDamping = tau;

  return made;
}

std::vector<glm::vec3> tipLoad(const Bar& bar, float factor) {
  const float yieldMoment = YIELD * AREA * 2.0f * HALF_WIDTH / 6.0f;
  const float force = factor * yieldMoment / (2.0f * HALF_LENGTH);

  std::vector<glm::vec3> load(bar.pull.size(), glm::vec3(0.0f));

  for (size_t node = 0u; node != load.size(); ++node) {
    load[node].y = force * bar.pull[node].x;
  }

  return load;
}

DeformableState heldAt(const Bar& bar, float factor) {
  const std::vector<PhysicalMaterial> linear =
    barMaterials(std::numeric_limits<float>::infinity());

  DeformableState state = talus::deformable::solveEquilibrium(
    *bar.mesh,
    linear,
    talus::objectmodel::Pose{},
    barSolve(bar.element),
    glm::vec3(0.0f),
    tipLoad(bar, factor)
  );

  const size_t nodes = bar.mesh->restPositions.size();

  state.velocity.assign(nodes, glm::vec3(0.0f));
  state.acceleration.assign(nodes, glm::vec3(0.0f));

  return state;
}

double largestGap(const DeformableState& a, const DeformableState& b) {
  double gap = 0.0;

  for (size_t node = 0u; node != a.position.size(); ++node) {
    gap = std::max(
      gap,
      static_cast<double>(glm::length(a.position[node] - b.position[node]))
    );
  }

  return gap;
}

}

}

TEST_CASE(
  "a plastic bar released below yield follows the linear path",
  "[plasticity]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> plain = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const bool hex = element == VolumeElementType::Hex;
    const Bar bar = buildPrism(
      glm::vec3(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH),
      COARSE_VOXEL,
      plain,
      element
    );
    const talus::objectmodel::PreparedMesh& mesh = *bar.mesh;
    const talus::deformable::Settings settings = barStep(element);
    const float clampedTau =
      hex ? damping::CLAMPED_TAU_HEX : damping::CLAMPED_TAU_TET;

    std::vector<DeformableState> held;

    for (const float factor : release::FACTORS) {
      held.push_back(release::heldAt(bar, factor));
    }

    for (const float tau : {0.0f, clampedTau}) {
      const std::vector<PhysicalMaterial> plastic =
        release::releaseMaterials(YIELD, tau);
      const std::vector<PhysicalMaterial> linear =
        release::releaseMaterials(std::numeric_limits<float>::infinity(), tau);

      std::cout << "linearrelease: " << elementName(element) << " tau " << tau
                << " zeta1 " << (tau > 0.0f ? 0.1 : 0.0) << " nodes "
                << mesh.restPositions.size() << " elements "
                << (hex ? mesh.hexes.size() : mesh.tets.size()) << " substeps "
                << settings.substeps << " spectralRadius "
                << settings.spectralRadius << " cgIterations "
                << settings.cgIterations << " factorBudget "
                << settings.factorBudget << " tolerance " << settings.tolerance
                << " plasticPasses " << settings.plasticPasses << " frames "
                << release::FRAMES
                << " start linear static, fresh solver "
                   "per step, gravity 0, contacts 0, bar "
                << release::LINEAR_BAR << " m\n";

      for (size_t at = 0u; at != release::FACTORS.size(); ++at) {
        const float factor = release::FACTORS[at];

        DeformableState reference = held[at];

        for (uint32_t frame = 1u; frame <= release::FRAMES; ++frame) {
          DeformableState fromPlastic = reference;
          DeformableState fromLinear = reference;

          const talus::deformable::StepReport report = damping::advance(
            mesh,
            fromPlastic,
            plastic,
            settings,
            glm::vec3(0.0f),
            nullptr
          );

          damping::advance(
            mesh,
            fromLinear,
            linear,
            settings,
            glm::vec3(0.0f),
            nullptr
          );

          const double gap = release::largestGap(fromPlastic, fromLinear);
          const double moved = release::largestGap(fromLinear, reference);

          std::cout << "linearrelease: " << elementName(element) << " tau "
                    << tau << " factor " << factor << " frame " << frame
                    << " gap " << gap << " linearStepMotion " << moved
                    << " solves " << report.plasticSolves << " backtracks "
                    << report.plasticBacktracks << " capped "
                    << report.plasticCapped << " within "
                    << (gap <= release::LINEAR_BAR ? 1 : 0) << '\n';

          INFO(
            elementName(element)
            << " tau " << tau << " factor " << factor << " of M_y frame "
            << frame << ": gap " << gap << " m, linear step motion " << moved
          );

          CHECK(allFinite(fromPlastic));
          CHECK(gap <= release::LINEAR_BAR);

          reference = fromLinear;
        }
      }
    }
  }
}

namespace {

namespace springback {

constexpr uint32_t FRAMES = 3u;
constexpr float HEX_FACTOR = 1.5f;
constexpr std::array<float, 4> TET_FACTORS{1.2f, 1.5f, 2.0f, 3.0f};
constexpr std::array<float, 4> BELOW{0.5f, 0.9f, 0.95f, 0.99f};
constexpr float CANTILEVER = 2.0f * HALF_LENGTH;

glm::vec3 stepGravity(float factor) {
  const float yieldMoment = YIELD * AREA * 2.0f * HALF_WIDTH / 6.0f;

  return glm::vec3(
    0.0f,
    2.0f * factor * yieldMoment / (DENSITY * AREA * CANTILEVER * CANTILEVER),
    0.0f
  );
}

DeformableState yieldedAt(
  const Bar& bar,
  std::span<const PhysicalMaterial> materials,
  float factor
) {
  const std::vector<glm::vec3> load = release::tipLoad(bar, factor);

  const DeformableState loaded = talus::deformable::solveEquilibrium(
    *bar.mesh,
    materials,
    talus::objectmodel::Pose{},
    barSolve(bar.element),
    glm::vec3(0.0f),
    load
  );

  DeformableState held = talus::deformable::solveEquilibrium(
    *bar.mesh,
    materials,
    loaded,
    barSolve(bar.element),
    glm::vec3(0.0f),
    load
  );

  const size_t nodes = bar.mesh->restPositions.size();

  held.velocity.assign(nodes, glm::vec3(0.0f));
  held.acceleration.assign(nodes, glm::vec3(0.0f));

  return held;
}

void printSettings(
  const char* fixture,
  VolumeElementType element,
  float tau,
  const talus::deformable::Settings& settings,
  uint32_t frames,
  const char* start
) {
  std::cout << fixture << ": settings " << elementName(element) << " tau "
            << tau << " substeps " << settings.substeps << " spectralRadius "
            << settings.spectralRadius << " cgIterations "
            << settings.cgIterations << " factorBudget "
            << settings.factorBudget << " tolerance " << settings.tolerance
            << " plasticPasses " << settings.plasticPasses << " timeStep "
            << TIME_STEP << " frames " << frames << ' ' << start << '\n';
}

double printRow(
  const char* fixture,
  VolumeElementType element,
  float tau,
  float factor,
  uint32_t frame,
  const DeformableState& start,
  const DeformableState& fromPlastic,
  const DeformableState& fromReference,
  const talus::deformable::StepReport& plasticReport,
  const talus::deformable::StepReport& referenceReport
) {
  const double gap = release::largestGap(fromPlastic, fromReference);
  const double moved = release::largestGap(fromReference, start);

  std::cout << fixture << ": " << elementName(element) << " tau " << tau
            << " factor " << factor << " frame " << frame << " gap " << gap
            << " stepMotion " << moved << " gapOverMotion "
            << (moved > 0.0 ? gap / moved : 0.0) << " within "
            << (gap <= release::LINEAR_BAR ? 1 : 0) << " linearReads "
            << plasticReport.plasticLinearReads << " handoffs "
            << plasticReport.plasticHandoffs << " backtracks "
            << plasticReport.plasticBacktracks << " capped "
            << plasticReport.plasticCapped << " solves "
            << plasticReport.plasticSolves << " equivalentStart "
            << largestEquivalent(start) << " equivalentAfter "
            << largestEquivalent(fromPlastic) << " finite "
            << (allFinite(fromPlastic) ? 1 : 0) << " referenceLinearReads "
            << referenceReport.plasticLinearReads << " referenceBacktracks "
            << referenceReport.plasticBacktracks << " referenceCapped "
            << referenceReport.plasticCapped << '\n';

  return gap;
}

void released(
  const char* fixture,
  const Bar& bar,
  const DeformableState& yielded,
  float factor,
  float tau
) {
  const talus::deformable::Settings settings = barStep(bar.element);
  const std::vector<PhysicalMaterial> plastic =
    release::releaseMaterials(YIELD, tau);
  const std::vector<PhysicalMaterial> eigen =
    release::releaseMaterials(std::numeric_limits<float>::max(), tau);

  printSettings(
    fixture,
    bar.element,
    tau,
    settings,
    FRAMES,
    "start plastic static under the tip load, released at rest, fresh solver "
    "per step, reference yield FLT_MAX with the committed plastic state, "
    "advanced on the reference, gravity 0, contacts 0, frame 1 asserted"
  );

  DeformableState start = yielded;

  for (uint32_t frame = 1u; frame <= FRAMES; ++frame) {
    DeformableState fromPlastic = start;
    DeformableState fromReference = start;

    const talus::deformable::StepReport plasticReport = damping::advance(
      *bar.mesh,
      fromPlastic,
      plastic,
      settings,
      glm::vec3(0.0f),
      nullptr
    );
    const talus::deformable::StepReport referenceReport = damping::advance(
      *bar.mesh,
      fromReference,
      eigen,
      settings,
      glm::vec3(0.0f),
      nullptr
    );

    const double gap = printRow(
      fixture,
      bar.element,
      tau,
      factor,
      frame,
      start,
      fromPlastic,
      fromReference,
      plasticReport,
      referenceReport
    );

    if (frame == 1u) {
      INFO(
        elementName(bar.element)
        << " tau " << tau << " factor " << factor << " of M_y frame 1: gap "
        << gap << " m, linear reads " << plasticReport.plasticLinearReads
        << ", handoffs " << plasticReport.plasticHandoffs << ", backtracks "
        << plasticReport.plasticBacktracks
      );

      CHECK(allFinite(fromPlastic));
      CHECK(gap <= release::LINEAR_BAR);
    }

    start = fromReference;
  }
}

}

}

TEST_CASE(
  "a hex bar yielded and released at rest follows its eigenstrain path",
  "[.][springback]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> plain = barMaterials(YIELD);
  const Bar bar = buildPrism(
    glm::vec3(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH),
    COARSE_VOXEL,
    plain,
    VolumeElementType::Hex
  );
  const DeformableState yielded =
    springback::yieldedAt(bar, plain, springback::HEX_FACTOR);

  INFO("hex at " << springback::HEX_FACTOR << " M_y");

  REQUIRE(largestEquivalent(yielded) > 0.0f);

  for (const float tau : {0.0f, damping::CLAMPED_TAU_HEX}) {
    springback::released(
      "springback",
      bar,
      yielded,
      springback::HEX_FACTOR,
      tau
    );
  }
}

TEST_CASE(
  "a tet bar yielded and released at rest follows its eigenstrain path",
  "[.][springback]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> plain = barMaterials(YIELD);
  const Bar bar = buildPrism(
    glm::vec3(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH),
    COARSE_VOXEL,
    plain,
    VolumeElementType::Tet
  );

  DeformableState yielded;
  float chosen = 0.0f;

  for (const float factor : springback::TET_FACTORS) {
    yielded = springback::yieldedAt(bar, plain, factor);

    std::cout << "springbacktet: static factor " << factor << " equivalent "
              << largestEquivalent(yielded) << '\n';

    if (largestEquivalent(yielded) > 0.0f) {
      chosen = factor;

      break;
    }
  }

  std::cout << "springbacktet: chosen factor " << chosen << '\n';

  REQUIRE(chosen > 0.0f);

  for (const float tau : {0.0f, damping::CLAMPED_TAU_TET}) {
    springback::released("springbacktet", bar, yielded, chosen, tau);
  }
}

TEST_CASE(
  "a clamped plastic bar under a step load hands its first step to Newton",
  "[.][springback]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> plain = damping::dampedMaterials(0.0f);
  const glm::vec3 gravity(bodyPull(), 0.0f, 0.0f);

  for (const VolumeElementType element : ELEMENTS) {
    const Bar bar = buildBar(COARSE_VOXEL, plain, element);
    const float clampedTau = element == VolumeElementType::Hex
                               ? damping::CLAMPED_TAU_HEX
                               : damping::CLAMPED_TAU_TET;
    const talus::deformable::Settings settings = barStep(element);

    for (const float tau : {clampedTau, 0.0f}) {
      const std::vector<PhysicalMaterial> materials =
        damping::dampedMaterials(tau);

      springback::printSettings(
        "handoff",
        element,
        tau,
        settings,
        1u,
        "start rest, fresh solver, body pull x at full load from frame 1, peak "
        "stress twice PAST_STRESS, contacts 0"
      );

      DeformableState state =
        talus::deformable::restState(*bar.mesh, talus::objectmodel::Pose{});

      const talus::deformable::StepReport report = damping::advance(
        *bar.mesh,
        state,
        materials,
        settings,
        gravity,
        nullptr
      );

      std::cout << "handoff: " << elementName(element) << " tau " << tau
                << " frame 1 linearReads " << report.plasticLinearReads
                << " handoffs " << report.plasticHandoffs << " backtracks "
                << report.plasticBacktracks << " capped "
                << report.plasticCapped << " solves " << report.plasticSolves
                << " equivalent " << largestEquivalent(state) << " finite "
                << (allFinite(state) ? 1 : 0) << '\n';

      INFO(
        elementName(element)
        << " tau " << tau << ": linear reads " << report.plasticLinearReads
        << ", handoffs " << report.plasticHandoffs
      );

      CHECK(allFinite(state));
      CHECK(report.plasticHandoffs > 0u);
    }
  }
}

TEST_CASE(
  "a plastic bar at rest under a step load below yield follows the linear "
  "path",
  "[.][springback]"
) {
  std::cout.precision(9);

  const std::vector<PhysicalMaterial> plain = barMaterials(YIELD);

  for (const VolumeElementType element : ELEMENTS) {
    const Bar bar = buildPrism(
      glm::vec3(HALF_LENGTH, HALF_WIDTH, HALF_WIDTH),
      COARSE_VOXEL,
      plain,
      element
    );
    const talus::deformable::Settings settings = barStep(element);
    const float clampedTau = element == VolumeElementType::Hex
                               ? damping::CLAMPED_TAU_HEX
                               : damping::CLAMPED_TAU_TET;
    const DeformableState rest =
      talus::deformable::restState(*bar.mesh, talus::objectmodel::Pose{});

    for (const float tau : {0.0f, clampedTau}) {
      const std::vector<PhysicalMaterial> plastic =
        release::releaseMaterials(YIELD, tau);
      const std::vector<PhysicalMaterial> linear =
        release::releaseMaterials(std::numeric_limits<float>::infinity(), tau);

      springback::printSettings(
        "steplinear",
        element,
        tau,
        settings,
        1u,
        "start rest, fresh solver, load a body force along y whose static "
        "root moment is factor M_y, against the linear path, contacts 0"
      );

      for (const float factor : springback::BELOW) {
        const glm::vec3 gravity = springback::stepGravity(factor);

        DeformableState fromPlastic = rest;
        DeformableState fromLinear = rest;

        const talus::deformable::StepReport plasticReport = damping::advance(
          *bar.mesh,
          fromPlastic,
          plastic,
          settings,
          gravity,
          nullptr
        );
        const talus::deformable::StepReport linearReport = damping::advance(
          *bar.mesh,
          fromLinear,
          linear,
          settings,
          gravity,
          nullptr
        );

        const double gap = springback::printRow(
          "steplinear",
          element,
          tau,
          factor,
          1u,
          rest,
          fromPlastic,
          fromLinear,
          plasticReport,
          linearReport
        );

        INFO(
          elementName(element)
          << " tau " << tau << " factor " << factor << " of M_y frame 1: gap "
          << gap << " m, linear reads " << plasticReport.plasticLinearReads
        );

        CHECK(allFinite(fromPlastic));
        CHECK(gap <= release::LINEAR_BAR);
      }
    }
  }
}
