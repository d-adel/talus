#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/dynamics/bounds/bounds.h>
#include <talus/dynamics/contact/contact.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace talus::dynamics {

struct RayHit {
  uint32_t body = 0u;
  uint32_t child = noChild;
  double fraction = 0.0;
};

struct RaySkip {
  uint32_t body = 0xFFFFFFFFu;
  uint32_t child = noChild;
};

[[nodiscard]] std::vector<uint32_t> rayBlockers(const BodySet& bodies, const Bounds& region);

[[nodiscard]] std::optional<RayHit> firstHit(
  const BodySet& bodies,
  std::span<const uint32_t> blockers,
  const glm::dvec3& from,
  const glm::dvec3& to,
  const RaySkip& skip
);

}
