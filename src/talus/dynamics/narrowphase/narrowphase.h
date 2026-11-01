#pragma once

#include <talus/dynamics/broadphase/broadphase.h>
#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/scheduler/scheduler.h>

#include <glm/glm.hpp>

#include <span>
#include <vector>

namespace talus::dynamics {

struct World;
struct PairWork;
class ManifoldCache;

struct PairEvents {
  std::vector<uint32_t> separated;
  std::vector<uint32_t> changed;
  std::vector<uint32_t> lost;
};

PairEvents updateCachedPairs(
  const BodySet& bodies,
  ManifoldCache& cache,
  double band,
  std::span<const double> travel,
  double dt,
  const Workers& workers = Workers{},
  const BroadPhase* broadPhase = nullptr,
  PairWork* work = nullptr,
  const std::vector<uint32_t>* active = nullptr,
  bool sameIndices = false
);

[[nodiscard]] glm::dvec3 sidePoint(
  const BodySet& bodies,
  const ContactSide& side
);
[[nodiscard]] std::vector<Contact> narrowphase(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  const Workers& workers = Workers{}
);
[[nodiscard]] std::vector<Contact> narrowphase(
  const BodySet& bodies,
  std::span<const BodyPair> pairs,
  double band,
  std::span<const double> travel,
  const Workers& workers = Workers{}
);

[[nodiscard]] uint64_t pairKey(
  const BodySet& bodies,
  const BodyRef& first,
  const BodyRef& second
);

[[nodiscard]] double normalForceOn(const World& world, const BodyRef& ref);

}
