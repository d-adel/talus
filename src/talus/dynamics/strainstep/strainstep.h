#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/integrationparameters/integrationparameters.h>
#include <talus/dynamics/islands/islands.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace talus::dynamics {

inline constexpr size_t PARALLEL_CELLS = 256u;

struct StrainstepArena;

struct StrainstepStats {
  size_t inverses = 0u;
  uint32_t builds = 0u;
  uint32_t reuses = 0u;
  size_t cachedCells = 0u;
  uint32_t parallelBatches = 0u;
};

[[nodiscard]] std::shared_ptr<StrainstepArena> makeStrainstepArena();

[[nodiscard]] StrainstepStats strainstepStats(const StrainstepArena& arena);

void forgetLattices(StrainstepArena& arena, const BodySet& bodies);

[[nodiscard]] uint32_t islandSubsteps(
  const BodySet& bodies,
  const Island& island,
  const IntegrationParameters& parameters
);

uint32_t solveStrainstep(
  BodySet& bodies,
  ManifoldCache& cache,
  std::span<Contact> contacts,
  const Island& island,
  const IntegrationParameters& parameters,
  StrainstepArena& arena,
  const Workers& workers
);

}
