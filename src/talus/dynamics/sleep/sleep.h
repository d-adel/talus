#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/continuous/continuous.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/islands/islands.h>

#include <span>

namespace talus::dynamics {

[[nodiscard]] bool resting(const BodySet& bodies, const BodyRef& ref);

[[nodiscard]] bool islandAsleep(const BodySet& bodies, const Island& island);

void wakeBody(BodySet& bodies, const BodyRef& ref);

void wakeAll(BodySet& bodies);

[[nodiscard]] double sleepSpeed(
  const RigidBody& body,
  const RigidPose& start,
  double dt
);

[[nodiscard]] double sleepSpeed(const LatticeBody& body, double dt);

void settleIslands(
  BodySet& bodies,
  ManifoldCache& cache,
  IslandStore& store,
  std::span<const Island> islands,
  std::span<const RigidPose> start,
  const IntegrationParameters& parameters
);

}
