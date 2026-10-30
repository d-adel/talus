#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>

namespace talus::dynamics {

struct SoftstepArena;
struct StepProfile;

struct SoftstepTouching {
  uint32_t points = 0u;
  uint32_t active = 0u;
  std::vector<uint32_t> hits;
};

struct SoftstepResult {
  uint32_t widePairs = 0u;
  uint32_t substeps = 0u;
};

[[nodiscard]] std::shared_ptr<SoftstepArena> makeSoftstepArena();

SoftstepResult solveSoftstep(
  BodySet& bodies,
  ManifoldCache& cache,
  std::span<const uint32_t> rigids,
  const IntegrationParameters& parameters,
  std::vector<uint32_t>& stateOf,
  const Workers& workers,
  uint32_t width,
  SoftstepArena* arena = nullptr,
  StepProfile* profile = nullptr,
  SoftstepTouching* touching = nullptr,
  const std::function<void()>* alongside = nullptr,
  JointSet* joints = nullptr,
  const IslandStore* islandStore = nullptr,
  std::span<const uint32_t> guests = {},
  bool partial = false
);

}
