#pragma once

#include <talus/dynamics/contact/contact.h>
#include <talus/dynamics/manifoldcache/manifoldcache.h>
#include <talus/dynamics/pipeline/pipeline.h>
#include <talus/dynamics/rays/rays.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <span>

namespace talus::dynamics {

struct WorldRayHit {
  Side side = Side::Static;
  uint32_t body = noBody;
  uint32_t child = noChild;
  glm::dvec3 point{0.0};
  glm::dvec3 normal{0.0};
  double fraction = 0.0;
};

[[nodiscard]] std::optional<WorldRayHit> castRay(
  const World& world,
  const glm::dvec3& from,
  const glm::dvec3& to,
  std::span<const RaySkip> skips = {}
);

}
