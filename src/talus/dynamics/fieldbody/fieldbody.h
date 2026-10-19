#pragma once

#include <talus/dynamics/bodyset/bodyset.h>
#include <talus/field/field.h>
#include <talus/field/sampledsdf/sampledsdf.h>
#include <talus/material.h>

#include <cstdint>
#include <span>

namespace talus::dynamics {

[[nodiscard]] RigidBody rigidFromField(
  const field::SampledSdf& field,
  std::span<const field::Primitive> primitives,
  std::span<const talus::PhysicalMaterial> materials,
  uint32_t sampleBudget
);

}
