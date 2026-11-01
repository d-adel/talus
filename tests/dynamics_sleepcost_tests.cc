#define _CRT_SECURE_NO_WARNINGS
#include <catch2/catch_test_macros.hpp>

#include "fieldprimitives.h"

#include <fixtures/mover.h>

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/edits/edits.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

using namespace talus::dynamics;

enum class Scene {
  Idle,
  Falling,
  Characters
};

constexpr int phaseCount = 18;
constexpr uint32_t warmupSteps = 30u;
constexpr uint32_t measuredSteps = 300u;
constexpr uint32_t settleLimit = 900u;
constexpr double crateHalf = 0.05;
constexpr double crateMass = 1.5;
constexpr double idleGrowthLimitUs = 20000.0 * 0.002;
constexpr double charactersGrowthLimitUs = 55.0;

constexpr std::array<const char*, phaseCount> phaseNames{
  "travel",
  "broadphase",
  "storeSync",
  "storeUpdate",
  "lattice",
  "islands",
  "solverSetup",
  "gather",
  "prepare",
  "substeps",
  "integrate",
  "restitute",
  "storeBack",
  "joints",
  "continuous",
  "events",
  "sleep",
  "total"
};

struct SceneRun {
  std::array<double, phaseCount> median{};
  std::array<double, phaseCount> low{};
  std::array<double, phaseCount> high{};
  double awake = 0.0;
};

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

std::array<double, phaseCount> phasesOf(const StepProfile& profile) {
  return {
    profile.travel,
    profile.broadphase,
    profile.storeSync,
    profile.storeUpdate,
    profile.lattice,
    profile.islands,
    profile.solverSetup,
    profile.gather,
    profile.prepare,
    profile.substeps,
    profile.integrate,
    profile.restitute,
    profile.storeBack,
    profile.joints,
    profile.continuous,
    profile.events,
    profile.sleep,
    profile.total
  };
}

double quantile(std::vector<double> values, double fraction) {
  std::ranges::sort(values);

  return values[static_cast<size_t>(
    fraction * static_cast<double>(values.size() - 1u)
  )];
}

bool allAsleep(const World& world) {
  return std::ranges::all_of(world.bodies.rigids, [](const RigidBody& body) {
    return body.asleep;
  });
}

SceneRun runScene(Scene scene, uint32_t sleepers, bool forceFull) {
  static const talus::field::SampledSdf floorField =
    boxField(glm::dvec3(60.0, 1.0, 60.0), 0.25);
  static const talus::field::SampledSdf crateField =
    boxField(glm::dvec3(crateHalf), 0.05);

  World world;

  world.workerCount = 1u;
  (
    void
  )addStatic(world.bodies, floorField, 0.5, 0.0, glm::dvec3(0.0, -1.0, 0.0));

  const uint32_t side =
    static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(sleepers))));

  for (uint32_t index = 0u; index != sleepers; ++index) {
    (void)addRigidBox(
      world.bodies,
      crateField,
      glm::dvec3(crateHalf),
      crateMass,
      glm::dvec3(
        8.0 + 0.3 * static_cast<double>(index % side),
        crateHalf,
        0.3
          * (static_cast<double>(index / side) - 0.5 * static_cast<double>(side))
      ),
      glm::dvec3(0.0),
      0.5,
      3u
    );
  }

  for (uint32_t settle = 0u; settle != settleLimit && !allAsleep(world);
       ++settle) {
    REQUIRE_FALSE(step(world).nonFinite);
  }

  REQUIRE(allAsleep(world));

  const std::array<glm::dvec3, 2> offsets{
    glm::dvec3(-0.4, 0.2, 0.3),
    glm::dvec3(0.4, 0.2, 0.3)
  };
  std::vector<fixture::Character> characters;
  std::vector<std::array<fixture::Grip, 2>> grips;

  if (scene == Scene::Falling) {
    (void)addRigidBox(
      world.bodies,
      crateField,
      glm::dvec3(crateHalf),
      crateMass,
      glm::dvec3(-20.0, 400.0, 0.0),
      glm::dvec3(0.5, 0.0, 0.0),
      0.5,
      3u
    );
  } else if (scene == Scene::Characters) {
    for (uint32_t index = 0u; index != 4u; ++index) {
      characters.push_back(
        fixture::addCharacter(
          world,
          glm::dvec3(
            2.0,
            fixture::pogoRest - fixture::footAnchor.y,
            -9.0 + 6.0 * static_cast<double>(index)
          )
        )
      );
      (void)fixture::addUprightSpring(world, characters.back().body);
      grips.push_back(
        {fixture::addGrip(
           world,
           characters.back().body,
           offsets[0],
           glm::dvec3(0.1),
           3.0
         ),
         fixture::addGrip(
           world,
           characters.back().body,
           offsets[1],
           glm::dvec3(0.1),
           3.0
         )}
      );
    }
  }

  std::array<std::vector<double>, phaseCount> samples;
  double awakeSum = 0.0;

  for (uint32_t frame = 0u; frame != warmupSteps + measuredSteps; ++frame) {
    bool edited = true;

    if (scene == Scene::Characters) {
      const double time = static_cast<double>(frame) / 60.0;

      for (size_t index = 0u; index != characters.size(); ++index) {
        fixture::MoverStep move;

        move.velocity = 2.0 * glm::dvec3(-std::sin(time), 0.0, std::cos(time));
        edited =
          fixture::tryMoveCharacter(world, characters[index], move).applied
          && edited;

        for (size_t hand = 0u; hand != 2u; ++hand) {
          edited = fixture::driveGrip(
                     world,
                     grips[index][hand],
                     characters[index].body,
                     offsets[hand]
                   )
                   && edited;
        }
      }
    }

    if (forceFull) {
      world.bodies.layout += 1u;
    }

    const StepReport report = step(world);

    REQUIRE_FALSE(report.nonFinite);
    REQUIRE(edited);

    if (frame < warmupSteps) {
      continue;
    }

    const std::array<double, phaseCount> phases = phasesOf(report.profile);

    for (int phase = 0; phase != phaseCount; ++phase) {
      samples[phase].push_back(phases[phase]);
    }

    awakeSum += static_cast<double>(
      std::ranges::count_if(world.bodies.rigids, [](const RigidBody& body) {
        return !body.asleep;
      })
    );
  }

  SceneRun run;

  run.awake = awakeSum / static_cast<double>(measuredSteps);

  for (int phase = 0; phase != phaseCount; ++phase) {
    run.median[phase] = 1000.0 * quantile(samples[phase], 0.5);
    run.low[phase] = 1000.0 * quantile(samples[phase], 0.1);
    run.high[phase] = 1000.0 * quantile(samples[phase], 0.9);
  }

  return run;
}

const char* sceneName(Scene scene) {
  return scene == Scene::Idle
           ? "idle"
           : (scene == Scene::Falling ? "falling" : "characters");
}

}

TEST_CASE("step cost among many sleeping bodies", "[.][sleepcost]") {
  for (const Scene scene : {Scene::Idle, Scene::Falling, Scene::Characters}) {
    double emptyStepUs = 0.0;

    for (const uint32_t sleepers : {0u, 5000u, 20000u}) {
      const SceneRun run = runScene(scene, sleepers, false);
      const double stepUs = run.median[phaseCount - 1];

      if (sleepers == 0u) {
        emptyStepUs = stepUs;
      }

      std::printf(
        "sleepcost scene %s bodies %u awake %.1f\n",
        sceneName(scene),
        sleepers,
        run.awake
      );

      for (int phase = 0; phase != phaseCount; ++phase) {
        std::printf(
          "sleepcost scene %s bodies %u phase %-11s median_us %9.2f p10 %9.2f "
          "p90 %9.2f\n",
          sceneName(scene),
          sleepers,
          phaseNames[phase],
          run.median[phase],
          run.low[phase],
          run.high[phase]
        );
      }

      const double perSleepingNs =
        sleepers == 0u
          ? 0.0
          : 1000.0 * (stepUs - emptyStepUs) / static_cast<double>(sleepers);

      std::printf(
        "sleepcost scene %s bodies %u step_us %.2f per_sleeping_ns %.1f\n",
        sceneName(scene),
        sleepers,
        stepUs,
        perSleepingNs
      );

      const SceneRun forced = runScene(scene, sleepers, true);

      std::printf(
        "sleepcost scene %s bodies %u forced_full_step_us %.2f\n",
        sceneName(scene),
        sleepers,
        forced.median[phaseCount - 1]
      );

      if (sleepers != 20000u) {
        continue;
      }

      const double growthUs = stepUs - emptyStepUs;

      std::printf(
        "sleepcost scene %s growth_us %.2f from 0 to %u sleeping bodies\n",
        sceneName(scene),
        growthUs,
        sleepers
      );

      if (scene == Scene::Idle) {
        CHECK(growthUs < idleGrowthLimitUs);
      } else if (scene == Scene::Characters) {
        CHECK(growthUs < charactersGrowthLimitUs);
      }
    }
  }
}
