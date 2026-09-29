#pragma once

#include <talus/field/capability/distancefield.h>
#include <talus/field/capability/gradientquery.h>
#include <talus/field/capability/materialquery.h>
#include <talus/field/capability/stepbound.h>
#include <talus/field/field.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <span>

namespace talus::field {

constexpr float DEFAULT_GRADIENT_EPSILON = TALUS_GRADIENT_EPSILON;

void updateBounds(Primitive& primitive);

struct Aabb {
  glm::vec3 minimum;
  glm::vec3 maximum;
};

std::optional<Aabb> boundedExtent(std::span<const Primitive> primitives);

std::optional<Aabb> primitiveBox(const Primitive& primitive);

struct AnalyticSample {
  float distance = 0.0f;
  uint32_t material = 0;
};

AnalyticSample combine(
  const AnalyticSample& accumulated,
  const Primitive& primitive,
  const glm::vec3& point
);

AnalyticSample sample(
  std::span<const Primitive> primitives,
  const glm::vec3& point
);

glm::vec3 gradient(
  std::span<const Primitive> primitives,
  const glm::vec3& point,
  float epsilon = DEFAULT_GRADIENT_EPSILON
);

glm::vec3 normal(
  std::span<const Primitive> primitives,
  const glm::vec3& point,
  float epsilon = DEFAULT_GRADIENT_EPSILON
);

struct AnalyticFieldView {
  std::span<const Primitive> primitives;
};

float distance(AnalyticFieldView field, const glm::vec3& point);

uint32_t material(AnalyticFieldView field, const glm::vec3& point);

glm::vec3 gradient(
  AnalyticFieldView field,
  const glm::vec3& point,
  float epsilon = DEFAULT_GRADIENT_EPSILON
);

float stepBound(
  AnalyticFieldView field,
  const glm::vec3& point,
  const glm::vec3& direction
);

static_assert(DistanceField<AnalyticFieldView>);
static_assert(GradientQueryable<AnalyticFieldView>);
static_assert(MaterialQueryable<AnalyticFieldView>);
static_assert(StepBounded<AnalyticFieldView>);

}
