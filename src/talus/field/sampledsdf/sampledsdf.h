#pragma once

#include <talus/field/capability/distancefield.h>
#include <talus/field/capability/gradientquery.h>
#include <talus/field/capability/materialquery.h>
#include <talus/field/capability/stepbound.h>
#include <talus/field/field.h>

#include <glm/glm.hpp>

#include <cstdint>
#include <span>
#include <vector>

namespace talus::field {

inline constexpr uint32_t BRICK_CELLS = TALUS_BRICK_CELLS;

struct SampledSdf {
  glm::vec3 origin{0.0f};
  glm::vec3 voxelSize{1.0f};
  glm::uvec3 resolution{0u};

  std::vector<float> distances;
  std::vector<uint8_t> materials;

  std::vector<float> brickLipschitz;
  float globalLipschitz = 0.0f;

  std::vector<float> brickMinimum;
  std::vector<float> brickMaximum;
};

struct SampleBox {
  glm::uvec3 minimum{0u};
  glm::uvec3 extent{0u};
};

SampleBox merge(const SampleBox& first, const SampleBox& second);

struct BrickRange {
  glm::uvec3 begin{0u};
  glm::uvec3 end{0u};
};

struct WriteReport {
  SampleBox samples;
  glm::uvec3 brickBegin{0u};
  glm::uvec3 brickEnd{0u};
};

BrickRange brickRange(const SampledSdf& field, const SampleBox& box);

glm::vec3 voxelSize(
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution
);

SampledSdf bake(
  std::span<const Primitive> primitives,
  const glm::vec3& minimum,
  const glm::vec3& maximum,
  const glm::uvec3& resolution
);

float distance(const SampledSdf& field, const glm::vec3& point);

uint32_t material(const SampledSdf& field, const glm::vec3& point);

glm::vec3 gradient(const SampledSdf& field, const glm::vec3& point);

template<typename Real>
struct FieldProbeOf {
  Real distance = Real(0);
  glm::vec<3, Real> normal{Real(0), Real(1), Real(0)};
  glm::vec<3, Real> gradient{Real(0)};
};

using FieldProbe = FieldProbeOf<float>;

float probeDistance(const SampledSdf& field, const glm::vec3& point);

[[nodiscard]] float probeLipschitz(const SampledSdf& field);

template<typename Real>
FieldProbeOf<Real> probe(
  const SampledSdf& field,
  const glm::vec<3, Real>& point
);

extern template FieldProbeOf<float> probe<float>(
  const SampledSdf& field,
  const glm::vec3& point
);

extern template FieldProbeOf<double> probe<double>(
  const SampledSdf& field,
  const glm::dvec3& point
);

float stepBound(
  const SampledSdf& field,
  const glm::vec3& point,
  const glm::vec3& direction
);

glm::uvec3 brickCounts(const SampledSdf& field);
bool brickHoldsSurface(const SampledSdf& field, uint32_t brick);

WriteReport write(
  SampledSdf& field,
  const SampleBox& box,
  std::span<const float> distances,
  std::span<const uint8_t> materials
);

static_assert(DistanceField<SampledSdf>);
static_assert(MaterialQueryable<SampledSdf>);
static_assert(GradientQueryable<SampledSdf>);
static_assert(StepBounded<SampledSdf>);

}
