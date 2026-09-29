#pragma once

#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>

#include <glm/glm.hpp>

#include <cstdint>

namespace talus::field {

inline constexpr uint32_t DEFORM_BAND_VOXELS = 4u;

SampleBox affectedBox(
  const SampledSdf& field,
  const Primitive& primitive,
  uint32_t marginVoxels
);

WriteReport deform(SampledSdf& field, const Primitive& primitive);

struct DeformReport {
  WriteReport write;
  float converged = 0.0f;
  uint32_t cellsRemoved = 0u;
};

DeformReport deformAndRepair(
  SampledSdf& field,
  const Primitive& primitive,
  uint32_t sweeps = 8u
);

}
