#pragma once

#include <talus/field/field.h>
#include <talus/field/analytic/analytic.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <utility>

namespace talus::field {

constexpr float MARCH_EPSILON = TALUS_MARCH_EPSILON;
constexpr int MARCH_MAX_ITERATIONS = TALUS_MARCH_MAX_ITERATIONS;
constexpr float MARCH_MAX_DISTANCE = TALUS_MARCH_MAX_DISTANCE;
constexpr float MARCH_OMEGA = TALUS_MARCH_OMEGA;
constexpr float RAY_EPSILON = 1e-8f;

struct Ray {
  glm::vec3 origin;
  glm::vec3 direction;
};

struct RayHit {
  glm::vec3 position;
  float travelled = 0.0f;
  uint32_t material = 0;
};

struct MarchSettings {
  float epsilon = MARCH_EPSILON;
  int maxIterations = MARCH_MAX_ITERATIONS;
  float maxDistance = MARCH_MAX_DISTANCE;
  float omega = MARCH_OMEGA;
};

Ray screenPointToRay(
  const glm::mat4& inverseViewProj,
  const glm::vec3& cameraPosition,
  const glm::vec2& ndc
);

[[nodiscard]] std::optional<float> alongAxis(
  const glm::vec3& anchor,
  const glm::vec3& along,
  const Ray& ray
);

std::optional<RayHit> raymarch(
  std::span<const Primitive> primitives,
  const Ray& ray,
  const MarchSettings& settings = {}
);

std::optional<std::pair<float, float>> intersect(
  const Aabb& box,
  const Ray& ray
);

}
